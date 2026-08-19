// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_walker/cdr_writer.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/pointcloud/point_cloud_io.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "tf_static_calibrate_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <vector>

// End-to-end test of `bagwiz tf static calibrate`'s run path: synthetic
// map.pcd + traj.tum + an MCAP bag drive run_tf_static_calibrate() directly
// (no CLI parsing). The scene follows the amended correlated-scene pattern
// from bagwiz_pointcloud/test/core/calib/correlated_scene.hpp (see the Task 6
// report referenced there): a frontal wall of map points at a fixed depth,
// each carrying an intensity equal to its own ground-truth projected pixel
// column, painted against an image that is a plain horizontal gray ramp. That
// keeps the gray/lidar joint histogram diagonal at the true pose instead of
// producing the quantization plateau a splatted, piecewise-constant render
// would, which is what makes the happy-path recovery test converge reliably.
namespace
{

using bagwiz::commands::run_tf_static_calibrate;
using bagwiz::commands::TfStaticCalibrateArgs;

constexpr const char * kParentFrame = "base_link";
constexpr const char * kChildFrame = "cam_link";
constexpr const char * kImageTopic = "/cam/image_raw";
constexpr const char * kCamInfoTopic = "/cam/camera_info";
constexpr const char * kTfStaticTopic = "/tf_static";

constexpr std::uint32_t kImageWidth = 320;
constexpr std::uint32_t kImageHeight = 240;
constexpr double kFx = 250.0;
constexpr double kFy = 250.0;
constexpr double kCx = 160.0;
constexpr double kCy = 120.0;
constexpr double kWallZ = 8.0;

// ---- hand-rolled CDR fixture serializers -----------------------------------
//
// Both go through bagwiz_msg's cdr_walker::CdrWriter (the alignment-aware CDR-1
// writer already proven against CdrReader — the same reader
// core::image::extract_camera_info / extract_raw_image use) rather than a
// bespoke byte-writer, so the alignment/length-prefix rules do not have to be
// re-derived by hand here.

// sensor_msgs/msg/Image: header{stamp{int32 sec, uint32 nsec}, string
// frame_id}, uint32 height, uint32 width, string encoding, uint8 is_bigendian,
// uint32 step, uint8[] data.
std::vector<std::byte> serialize_image_bgr8(
  std::int32_t sec, std::uint32_t nsec, const std::string & frame_id, std::uint32_t width,
  std::uint32_t height, const std::vector<std::uint8_t> & bgr)
{
  bagwiz::core::cdr_walker::CdrWriter w;
  w.write_i32(sec);
  w.write_u32(nsec);
  w.write_string(frame_id);
  w.write_u32(height);
  w.write_u32(width);
  w.write_string("bgr8");
  w.write_u8(0);  // is_bigendian
  w.write_u32(width * 3);
  w.write_sequence_length(static_cast<std::uint32_t>(bgr.size()));
  w.write_bytes(std::as_bytes(std::span<const std::uint8_t>(bgr)));
  return w.take();
}

// sensor_msgs/msg/CameraInfo: header, uint32 height, uint32 width, string
// distortion_model, float64[] d, float64[9] k, float64[9] r, float64[12] p,
// uint32 binning_x, uint32 binning_y, RegionOfInterest{4x uint32, bool}.
std::vector<std::byte> serialize_camera_info(
  std::int32_t sec, std::uint32_t nsec, const std::string & frame_id, std::uint32_t width,
  std::uint32_t height, const std::array<double, 9> & k)
{
  bagwiz::core::cdr_walker::CdrWriter w;
  w.write_i32(sec);
  w.write_u32(nsec);
  w.write_string(frame_id);
  w.write_u32(height);
  w.write_u32(width);
  w.write_string("plumb_bob");
  w.write_sequence_length(0);  // d: no distortion
  for (const double v : k) {
    w.write_f64(v);
  }
  constexpr std::array<double, 9> kIdentityR{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (const double v : kIdentityR) {
    w.write_f64(v);
  }
  const std::array<double, 12> p{k[0], 0, k[2], 0, 0, k[4], k[5], 0, 0, 0, 1, 0};
  for (const double v : p) {
    w.write_f64(v);
  }
  w.write_u32(0);       // binning_x
  w.write_u32(0);       // binning_y
  w.write_u32(0);       // roi.x_offset
  w.write_u32(0);       // roi.y_offset
  w.write_u32(0);       // roi.width
  w.write_u32(0);       // roi.height
  w.write_bool(false);  // roi.do_rectify
  return w.take();
}

std::array<double, 9> camera_k()
{
  return {kFx, 0, kCx, 0, kFy, kCy, 0, 0, 1};
}

// ---- scene: map points + the shared ramp image -----------------------------

struct Scene
{
  std::vector<std::array<float, 3>> points;
  std::vector<float> intensities;
  std::vector<std::uint8_t> ramp_bgr;  // shared by every fixture image message
};

// 60x30 grid (1800 points, comfortably over NidParams::min_points' default
// floor of 1000) on a frontal wall at z=kWallZ, spacing 0.1 m. Each point's
// intensity is its own ground-truth projected u (identity camera, no
// distortion) so equalize_intensity_bins()'s rank order matches the image's
// gray ramp exactly at the true pose. The image itself is a plain horizontal
// ramp (gray = px * 255 / (width - 1), constant along v) rendered into a real
// packed-BGR24 raster (b == g == r == gray, so gray_from_bgr24's BT.601
// weights reconstruct it exactly) — no splatting.
Scene build_scene()
{
  Scene scene;
  for (int iy = -15; iy < 15; ++iy) {
    for (int ix = -30; ix < 30; ++ix) {
      const float x = 0.1F * static_cast<float>(ix);
      const float y = 0.1F * static_cast<float>(iy);
      const float z = static_cast<float>(kWallZ);
      scene.points.push_back({x, y, z});
      const double u = kFx * (static_cast<double>(x) / z) + kCx;
      scene.intensities.push_back(static_cast<float>(u));
    }
  }
  scene.ramp_bgr.resize(static_cast<std::size_t>(kImageWidth) * kImageHeight * 3);
  for (std::uint32_t py = 0; py < kImageHeight; ++py) {
    for (std::uint32_t px = 0; px < kImageWidth; ++px) {
      const auto gray = static_cast<std::uint8_t>(px * 255 / (kImageWidth - 1));
      const std::size_t base = (static_cast<std::size_t>(py) * kImageWidth + px) * 3;
      scene.ramp_bgr[base + 0] = gray;
      scene.ramp_bgr[base + 1] = gray;
      scene.ramp_bgr[base + 2] = gray;
    }
  }
  return scene;
}

void write_map_pcd(const std::filesystem::path & path, const Scene & scene, bool with_intensity)
{
  std::ofstream out(path, std::ios::binary);
  ASSERT_TRUE(out.is_open());
  if (with_intensity) {
    bagwiz::core::pointcloud::write_pcd(out, scene.points, scene.intensities);
  } else {
    bagwiz::core::pointcloud::write_pcd(out, scene.points);
  }
}

// A static vehicle: identity pose at each end of the span, so
// interpolate_trajectory's slerp/lerp is trivial and the trajectory frame
// coincides with the map's own world frame everywhere in between.
void write_traj_tum(const std::filesystem::path & path, std::int64_t t0_ns, std::int64_t t1_ns)
{
  std::vector<bagwiz::core::TrajectoryPose> poses(2);
  poses[0].timestamp_ns = t0_ns;
  poses[0].qw = 1.0;
  poses[1].timestamp_ns = t1_ns;
  poses[1].qw = 1.0;
  std::ofstream out(path);
  ASSERT_TRUE(out.is_open());
  bagwiz::core::write_tum(out, poses);
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::TopicInfo tf_static_topic_info()
{
  bagwiz::io::TopicInfo t;
  t.name = kTfStaticTopic;
  t.type = "tf2_msgs/msg/TFMessage";
  t.serialization_format = "cdr";
  t.schema_encoding = "ros2msg";
  t.schema_text = bagwiz::core::kTfMessageWireSchema;
  return t;
}

// base_link -> cam_link, rotated by `yaw_rad` about z. cam_link is the
// fixture's optical frame too (identity optical rotation), so this is the
// chain's single edited edge end to end.
geometry_msgs::msg::TransformStamped make_static_edge(double yaw_rad)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = kParentFrame;
  ts.child_frame_id = kChildFrame;
  ts.transform.rotation.z = std::sin(yaw_rad / 2.0);
  ts.transform.rotation.w = std::cos(yaw_rad / 2.0);
  return ts;
}

// Writes /tf_static (base_link -> cam_link, off by `edge_yaw_rad` from the
// identity pose the images were actually rendered at), /cam/camera_info (one
// message), and /cam/image_raw (one bgr8 message per entry of
// `image_stamps_ns`, all sharing the scene's ramp raster).
void write_fixture_bag(
  const std::filesystem::path & path, double edge_yaw_rad,
  std::span<const std::int64_t> image_stamps_ns, const Scene & scene)
{
  bagwiz::io::TopicInfo image_topic;
  image_topic.name = kImageTopic;
  image_topic.type = "sensor_msgs/msg/Image";
  image_topic.serialization_format = "cdr";

  bagwiz::io::TopicInfo cam_info_topic;
  cam_info_topic.name = kCamInfoTopic;
  cam_info_topic.type = "sensor_msgs/msg/CameraInfo";
  cam_info_topic.serialization_format = "cdr";

  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(tf_static_topic_info());
  writer->declare_topic(image_topic);
  writer->declare_topic(cam_info_topic);

  const auto edge = make_static_edge(edge_yaw_rad);
  const auto tf_cdr = bagwiz::core::serialize_tf_message(
    std::span<const geometry_msgs::msg::TransformStamped>(&edge, 1));
  writer->write(kTfStaticTopic, 0, std::span<const std::byte>(tf_cdr.data(), tf_cdr.size()));

  const auto cam_info_cdr =
    serialize_camera_info(0, 0, kChildFrame, kImageWidth, kImageHeight, camera_k());
  writer->write(
    kCamInfoTopic, 0, std::span<const std::byte>(cam_info_cdr.data(), cam_info_cdr.size()));

  for (const std::int64_t stamp_ns : image_stamps_ns) {
    const auto sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
    const auto image_cdr =
      serialize_image_bgr8(sec, 0, kChildFrame, kImageWidth, kImageHeight, scene.ramp_bgr);
    writer->write(
      kImageTopic, stamp_ns, std::span<const std::byte>(image_cdr.data(), image_cdr.size()));
  }
  writer->close();
}

std::vector<std::int64_t> default_image_stamps_ns()
{
  std::vector<std::int64_t> stamps;
  for (const int sec : {15, 19, 23, 27, 31, 35, 39, 43}) {
    stamps.push_back(static_cast<std::int64_t>(sec) * 1'000'000'000LL);
  }
  return stamps;
}

TfStaticCalibrateArgs base_args(const std::filesystem::path & tmp_dir)
{
  TfStaticCalibrateArgs args;
  args.input_path = (tmp_dir / "bag.mcap").string();
  args.map_path = (tmp_dir / "map.pcd").string();
  args.traj_path = (tmp_dir / "traj.tum").string();
  args.traj_frame = kParentFrame;
  args.topic = kImageTopic;
  args.parent_frame = kParentFrame;
  args.child_frame = kChildFrame;
  args.output_path = (tmp_dir / "out.yaml").string();
  args.samples = 5;
  args.max_rot_deg = 2.0;
  args.min_depth = 1.0;
  args.max_depth = 50.0;
  return args;
}

class TfStaticCalibrateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_tf_static_calibrate_" +
                std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_dir_);
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override { std::filesystem::remove_all(tmp_dir_); }

  std::filesystem::path tmp_dir_;
};

}  // namespace

// Pins the fixture serializers' alignment against the real parsers before
// they are trusted inside a bag (per the task brief's ruling).
TEST(TfStaticCalibrateFixtureSerializersTest, CameraInfoRoundTrips)
{
  const auto payload =
    serialize_camera_info(15, 0, kChildFrame, kImageWidth, kImageHeight, camera_k());
  const auto result = bagwiz::core::image::extract_camera_info(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.info->width, kImageWidth);
  EXPECT_EQ(result.info->height, kImageHeight);
  EXPECT_EQ(result.info->frame_id, kChildFrame);
  EXPECT_DOUBLE_EQ(result.info->k[0], kFx);
  EXPECT_DOUBLE_EQ(result.info->k[2], kCx);
  EXPECT_DOUBLE_EQ(result.info->k[5], kCy);
  EXPECT_TRUE(result.info->d.empty());
}

TEST(TfStaticCalibrateFixtureSerializersTest, ImageRoundTrips)
{
  const std::vector<std::uint8_t> bgr(static_cast<std::size_t>(4) * 3 * 2, 0x42);
  const auto payload = serialize_image_bgr8(20, 5, kChildFrame, 4, 2, bgr);
  const auto result = bagwiz::core::image::extract_raw_image(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 4U);
  EXPECT_EQ(result.image->height, 2U);
  EXPECT_EQ(result.image->encoding, "bgr8");
  EXPECT_EQ(result.image->header_stamp_ns, 20'000'000'005LL);
  ASSERT_EQ(result.image->data.size(), bgr.size());
  EXPECT_EQ(static_cast<std::uint8_t>(result.image->data[0]), 0x42);
}

TEST_F(TfStaticCalibrateTest, RejectsMapWithoutIntensity)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/false);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_tf_static_calibrate(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(TfStaticCalibrateTest, RejectsEdgeNotOnChain)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  auto args = base_args(tmp_dir_);
  args.child_frame = "bogus";
  EXPECT_EQ(run_tf_static_calibrate(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(TfStaticCalibrateTest, RejectsWhenImageStampsMissTrajectory)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  // Trajectory spans 100..200 s; the fixture's images stay at 15..43 s, so no
  // image stamp falls inside the (margin-shrunk) trajectory span.
  write_traj_tum(tmp_dir_ / "traj.tum", 100'000'000'000LL, 200'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_tf_static_calibrate(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(TfStaticCalibrateTest, RecoversInjectedYawIntoYaml)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  // The bag's base_link -> cam_link edge is off by +1 deg of yaw from the
  // identity pose the images were actually rendered at; refine must find
  // delta yaw ~= -1 deg so the refined edge is (near) identity again.
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  write_fixture_bag(tmp_dir_ / "bag.mcap", kInjectedYawRad, default_image_stamps_ns(), scene);

  auto args = base_args(tmp_dir_);
  // At a single-depth frontal wall, yaw and translation are projectively
  // near-degenerate (the same reason extrinsic_refine_test.cpp's core-level
  // equivalent fixes translation): constrain to rotation only.
  args.fix_axes = "x,y,z";

  ASSERT_EQ(run_tf_static_calibrate(args), 0);
  ASSERT_TRUE(std::filesystem::exists(args.output_path));

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  ASSERT_EQ(parsed.transforms->size(), 1U);
  const auto & t = parsed.transforms->front();
  EXPECT_EQ(t.header.frame_id, kParentFrame);
  EXPECT_EQ(t.child_frame_id, kChildFrame);

  const auto rpy = bagwiz::core::quaternion_to_rpy(t.transform.rotation);
  constexpr double kToleranceRad = 0.15 * M_PI / 180.0;
  EXPECT_NEAR(rpy.yaw, 0.0, kToleranceRad);
}
