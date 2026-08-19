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
#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

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

// End-to-end test of `bagwiz calib cam-lidar`'s run path: synthetic
// map.pcd + traj.tum + an MCAP bag drive run_calib_cam_lidar() directly
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

using bagwiz::commands::CalibCamLidarArgs;
using bagwiz::commands::run_calib_cam_lidar;

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

// base_link -> cam_link carrying an arbitrary rpy triple, for the fixtures
// whose edited edge is not a plain yaw about z.
geometry_msgs::msg::TransformStamped make_static_edge_rpy(
  double roll_rad, double pitch_rad, double yaw_rad)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = kParentFrame;
  ts.child_frame_id = kChildFrame;
  ts.transform.rotation = bagwiz::core::rpy_to_quaternion({roll_rad, pitch_rad, yaw_rad});
  return ts;
}

// The same wall, re-expressed for a camera mounted in the optical convention
// (rpy = -90, 0, -90): that rotation sends camera +z to world +x, camera +x to
// world -y and camera +y to world -z, so a point the scene places at
// (x, y, kWallZ) in camera coordinates sits at (kWallZ, -x, -y) in the world.
// Its intensity is unchanged — it was already derived from the ground-truth
// projected column, which the remapping does not move.
Scene optical_convention_scene(const Scene & scene)
{
  Scene out = scene;
  for (auto & p : out.points) {
    p = {p[2], -p[0], -p[1]};
  }
  return out;
}

// Writes /tf_static (`static_edges`), /cam/camera_info (one message, whose
// header.frame_id is the chain's optical frame), and /cam/image_raw (one bgr8
// message per entry of `image_stamps_ns`, all sharing the scene's ramp
// raster).
void write_fixture_bag_edges(
  const std::filesystem::path & path,
  std::span<const geometry_msgs::msg::TransformStamped> static_edges,
  const std::string & optical_frame, std::span<const std::int64_t> image_stamps_ns,
  const Scene & scene)
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

  const auto tf_cdr = bagwiz::core::serialize_tf_message(static_edges);
  writer->write(kTfStaticTopic, 0, std::span<const std::byte>(tf_cdr.data(), tf_cdr.size()));

  const auto cam_info_cdr =
    serialize_camera_info(0, 0, optical_frame, kImageWidth, kImageHeight, camera_k());
  writer->write(
    kCamInfoTopic, 0, std::span<const std::byte>(cam_info_cdr.data(), cam_info_cdr.size()));

  for (const std::int64_t stamp_ns : image_stamps_ns) {
    const auto sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
    const auto image_cdr =
      serialize_image_bgr8(sec, 0, optical_frame, kImageWidth, kImageHeight, scene.ramp_bgr);
    writer->write(
      kImageTopic, stamp_ns, std::span<const std::byte>(image_cdr.data(), image_cdr.size()));
  }
  writer->close();
}

// The single-edge fixture the yaw-recovery tests use: base_link -> cam_link
// off by `edge_yaw_rad` from the identity pose the images were rendered at,
// with cam_link doubling as the optical frame.
void write_fixture_bag(
  const std::filesystem::path & path, double edge_yaw_rad,
  std::span<const std::int64_t> image_stamps_ns, const Scene & scene)
{
  const auto edge = make_static_edge(edge_yaw_rad);
  write_fixture_bag_edges(
    path, std::span<const geometry_msgs::msg::TransformStamped>(&edge, 1), kChildFrame,
    image_stamps_ns, scene);
}

// The six per-axis values of one field of render_calibrate_json's output, in
// its fixed x,y,z,roll,pitch,yaw order. The JSON is hand-built with a stable
// shape, so scanning for the field's occurrences in order is enough and keeps
// a JSON dependency out of the test. ("before" cannot collide with
// "nid_before": the leading quote is part of the key.)
std::array<double, 6> json_axis_field(const std::string & json, const std::string & field)
{
  std::array<double, 6> out{};
  const std::string key = "\"" + field + "\": ";
  std::size_t pos = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    pos = json.find(key, pos);
    EXPECT_NE(pos, std::string::npos) << "missing '" << field << "' #" << i << " in:\n" << json;
    if (pos == std::string::npos) {
      return out;
    }
    pos += key.size();
    out[i] = std::stod(json.substr(pos));
  }
  return out;
}

std::vector<std::int64_t> default_image_stamps_ns()
{
  std::vector<std::int64_t> stamps;
  for (const int sec : {15, 19, 23, 27, 31, 35, 39, 43}) {
    stamps.push_back(static_cast<std::int64_t>(sec) * 1'000'000'000LL);
  }
  return stamps;
}

CalibCamLidarArgs base_args(const std::filesystem::path & tmp_dir)
{
  CalibCamLidarArgs args;
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

class CalibCamLidarTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_calib_cam_lidar_" +
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
TEST(CalibCamLidarFixtureSerializersTest, CameraInfoRoundTrips)
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

TEST(CalibCamLidarFixtureSerializersTest, ImageRoundTrips)
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

TEST_F(CalibCamLidarTest, RejectsMapWithoutIntensity)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/false);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsEdgeNotOnChain)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  auto args = base_args(tmp_dir_);
  args.child_frame = "bogus";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsWhenImageStampsMissTrajectory)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  // Trajectory spans 100..200 s; the fixture's images stay at 15..43 s, so no
  // image stamp falls inside the (margin-shrunk) trajectory span.
  write_traj_tum(tmp_dir_ / "traj.tum", 100'000'000'000LL, 200'000'000'000LL);
  write_fixture_bag(tmp_dir_ / "bag.mcap", 0.0, default_image_stamps_ns(), scene);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RecoversInjectedYawIntoYaml)
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

  ASSERT_EQ(run_calib_cam_lidar(args), 0);
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
  // Roll and pitch were never perturbed, so they must come back near zero
  // too. The tolerance is deliberately looser than the yaw one: the ramp
  // fixture only varies along u, so roll and pitch are far more weakly
  // determined than yaw and the optimizer drifts on them (~0.5 deg observed
  // on this fixture). 0.75 deg still catches a gross mis-parametrization —
  // the axis swap a right-multiplied delta produces is degrees wide — while
  // leaving room for that drift.
  constexpr double kWeakAxisToleranceRad = 0.75 * M_PI / 180.0;
  EXPECT_NEAR(rpy.roll, 0.0, kWeakAxisToleranceRad);
  EXPECT_NEAR(rpy.pitch, 0.0, kWeakAxisToleranceRad);
}

// --keyframe-dist on the (stationary) fixture: the pose gate collapses to one
// interval, so the command must warn, fall back to plain even time spacing,
// and still succeed end to end — the gate is an optimization, never a new
// failure mode.
TEST_F(CalibCamLidarTest, KeyframeGateFallsBackOnStationaryTrajectory)
{
  const auto scene = build_scene();
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  write_fixture_bag(tmp_dir_ / "bag.mcap", kInjectedYawRad, default_image_stamps_ns(), scene);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";
  args.keyframe_dist = 0.5;
  args.keyframe_rot_deg = 10.0;

  ASSERT_EQ(run_calib_cam_lidar(args), 0);
  ASSERT_TRUE(std::filesystem::exists(args.output_path));

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  const auto rpy = bagwiz::core::quaternion_to_rpy(parsed.transforms->front().transform.rotation);
  EXPECT_NEAR(rpy.yaw, 0.0, 0.15 * M_PI / 180.0);
}

// The regression test for the parametrization the report and the emitted YAML
// used to disagree on. The edited edge carries a real optical-convention
// rotation (rpy = -90, 0, -90 deg: camera +z along world +x, world +z up),
// the case where an additive delta and a right-multiplied SE3 factor differ by
// an axis swap — an additive yaw pans the camera about the mast, a
// right-multiplied one rolls it about its own optical axis. The assertions are
// about the parametrization, not about recovery quality (which
// RecoversInjectedYawIntoYaml covers): a uniform column shift is nearly
// invisible to a mutual-information cost when both the image ramp and the
// point intensities are monotone in that column, so this fixture's optimizer
// only nibbles at the injected error. What must hold regardless is that the
// YAML on disk is the same edge the report printed, and that a --fix'd axis
// does not move at all — a right-multiplied yaw delta would have rotated the
// held roll and pitch right along with it.
TEST_F(CalibCamLidarTest, OpticalConventionEdgeYamlMatchesTheReportedEdge)
{
  constexpr double kDeg = M_PI / 180.0;
  constexpr double kTrueRoll = -90.0 * kDeg;
  constexpr double kTruePitch = 0.0;
  constexpr double kTrueYaw = -90.0 * kDeg;
  constexpr double kInjectedYaw = 2.0 * kDeg;

  const auto scene = optical_convention_scene(build_scene());
  write_map_pcd(tmp_dir_ / "map.pcd", scene, /*with_intensity=*/true);
  write_traj_tum(tmp_dir_ / "traj.tum", 10'000'000'000LL, 50'000'000'000LL);
  const auto edge = make_static_edge_rpy(kTrueRoll, kTruePitch, kTrueYaw + kInjectedYaw);
  const auto stamps = default_image_stamps_ns();
  write_fixture_bag_edges(
    tmp_dir_ / "bag.mcap", std::span<const geometry_msgs::msg::TransformStamped>(&edge, 1),
    kChildFrame, stamps, scene);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z,roll,pitch";
  args.max_rot_deg = 5.0;  // the injected 2 deg must sit inside the trust region
  args.json = true;        // machine-readable before/after/delta to check against the YAML

  ::testing::internal::CaptureStdout();
  const int rc = run_calib_cam_lidar(args);
  const std::string report = ::testing::internal::GetCapturedStdout();
  ASSERT_EQ(rc, 0) << report;
  ASSERT_TRUE(std::filesystem::exists(args.output_path));

  const auto before = json_axis_field(report, "before");
  const auto after = json_axis_field(report, "after");
  const auto delta = json_axis_field(report, "delta");
  for (std::size_t axis = 0; axis < 6; ++axis) {
    EXPECT_NEAR(after[axis], before[axis] + delta[axis], 1e-12) << "axis " << axis;
  }
  // The bag's own values came back verbatim, and the held axes did not move.
  EXPECT_NEAR(before[3], kTrueRoll, 1e-9);
  EXPECT_NEAR(before[4], kTruePitch, 1e-9);
  EXPECT_NEAR(before[5], kTrueYaw + kInjectedYaw, 1e-9);
  EXPECT_EQ(delta[3], 0.0);
  EXPECT_EQ(delta[4], 0.0);
  // The free axis did move, so the fixed-axis check above is not vacuous: with
  // a right-multiplied delta this much yaw would have shifted roll and pitch
  // by a comparable amount.
  EXPECT_GT(std::abs(delta[5]), 0.01 * kDeg);

  // The file on disk is that same edge, axis for axis.
  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  ASSERT_EQ(parsed.transforms->size(), 1U);
  const auto & t = parsed.transforms->front();
  EXPECT_NEAR(t.transform.translation.x, after[0], 1e-9);
  EXPECT_NEAR(t.transform.translation.y, after[1], 1e-9);
  EXPECT_NEAR(t.transform.translation.z, after[2], 1e-9);
  const auto rpy = bagwiz::core::quaternion_to_rpy(t.transform.rotation);
  EXPECT_NEAR(rpy.roll, after[3], 1e-9);
  EXPECT_NEAR(rpy.pitch, after[4], 1e-9);
  EXPECT_NEAR(rpy.yaw, after[5], 1e-9);
}
