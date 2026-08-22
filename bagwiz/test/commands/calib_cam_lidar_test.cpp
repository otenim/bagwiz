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
#include "bagwiz/core/introspection/introspection_loader.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rmw/types.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <span>
#include <string>
#include <vector>

// End-to-end test of `bagwiz calib cam-lidar`'s run path: one synthetic MCAP
// bag carrying /tf_static, CameraInfo, the image topic, the --pcd PointCloud2
// topic, and the --pose PoseStamped topic drives run_calib_cam_lidar()
// directly (no CLI parsing). The scene follows the amended correlated-scene
// pattern from bagwiz_pointcloud/test/core/calib/correlated_scene.hpp: a
// frontal wall of map points at a fixed depth, each carrying an intensity
// equal to its own ground-truth projected pixel column, painted against an
// image that is a plain horizontal gray ramp. That keeps the gray/lidar joint
// histogram diagonal at the true pose instead of producing the quantization
// plateau a splatted, piecewise-constant render would, which is what makes
// the happy-path recovery test converge reliably.
namespace
{

using bagwiz::commands::CalibCamLidarArgs;
using bagwiz::commands::run_calib_cam_lidar;
namespace pc = bagwiz::core::pointcloud;

constexpr const char * kParentFrame = "base_link";
constexpr const char * kChildFrame = "cam_link";
constexpr const char * kLidarFrame = "lidar";
constexpr const char * kRefFrame = "map";
constexpr const char * kImageTopic = "/cam/image_raw";
constexpr const char * kCamInfoTopic = "/cam/camera_info";
constexpr const char * kPcdTopic = "/lidar/points";
constexpr const char * kPoseTopic = "/pose";
constexpr const char * kTfStaticTopic = "/tf_static";
constexpr const char * kImuTopic = "/imu";
constexpr const char * kImuFrame = "imu_link";

constexpr std::uint32_t kImageWidth = 320;
constexpr std::uint32_t kImageHeight = 240;
constexpr double kFx = 250.0;
constexpr double kFy = 250.0;
constexpr double kCx = 160.0;
constexpr double kCy = 120.0;
constexpr double kWallZ = 8.0;

// ---- hand-rolled CDR fixture serializers -----------------------------------
//
// The image and CameraInfo serializers go through bagwiz_msg's
// cdr_walker::CdrWriter (the alignment-aware CDR-1 writer already proven
// against CdrReader — the same reader core::image::extract_camera_info /
// extract_raw_image use) rather than a bespoke byte-writer, so the
// alignment/length-prefix rules do not have to be re-derived by hand here. The
// pose topic is consumed through the introspection decoder instead, so it is
// serialized with the matching typesupport (serialize_typed below), and the
// cloud with the library's own serialize_pointcloud2.

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

// Typed-message CDR round-trip through the introspection typesupport (the
// pcd_undistort_common_test idiom), for the pose topic the trajectory builder
// reads through the introspection decoder.
template <typename T>
std::vector<std::byte> serialize_typed(const T & msg, const char * type_name)
{
  auto intro = bagwiz::core::load_introspection(type_name);
  EXPECT_TRUE(intro.ok()) << intro.error;

  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  EXPECT_EQ(rmw_serialized_message_init(&serialized, 0, &alloc), RMW_RET_OK);
  EXPECT_EQ(rmw_serialize(&msg, intro.typesupport, &serialized), RMW_RET_OK);
  std::vector<std::byte> out(serialized.buffer_length);
  if (serialized.buffer_length > 0) {
    std::memcpy(out.data(), serialized.buffer, serialized.buffer_length);
  }
  rmw_serialized_message_fini(&serialized);
  return out;
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
Scene build_scene(bool with_dots = false)
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
  if (with_dots) {
    // Sparse soft blobs on top of the ramp: corners for the --cam-offset auto
    // tracker, too sparse to disturb the ramp's gray/intensity correlation.
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> ux(0.0, kImageWidth);
    std::uniform_real_distribution<double> uy(0.0, kImageHeight);
    std::uniform_real_distribution<double> amplitude_dist(-70.0, 70.0);
    for (int b = 0; b < 250; ++b) {
      const double cx = ux(rng);
      const double cy = uy(rng);
      const double amp = amplitude_dist(rng);
      for (int dy = -6; dy <= 6; ++dy) {
        for (int dx = -6; dx <= 6; ++dx) {
          const int x = static_cast<int>(cx) + dx;
          const int y = static_cast<int>(cy) + dy;
          if (
            x < 0 || y < 0 || x >= static_cast<int>(kImageWidth) ||
            y >= static_cast<int>(kImageHeight)) {
            continue;
          }
          const double g = std::exp(-(dx * dx + dy * dy) / (2.0 * 2.0 * 2.0));
          const std::size_t base =
            (static_cast<std::size_t>(y) * kImageWidth + static_cast<std::size_t>(x)) * 3;
          for (int c = 0; c < 3; ++c) {
            const double v =
              std::clamp(static_cast<double>(scene.ramp_bgr[base + c]) + amp * g, 0.0, 255.0);
            scene.ramp_bgr[base + c] = static_cast<std::uint8_t>(v);
          }
        }
      }
    }
  }
  return scene;
}

// The ramp raster as seen by a camera rotated by `yaw_rad` about its own z
// (the fixture's base_link z is the optical axis, so a pose yaw rolls the
// image): a pure rotation, so the view is the base raster warped by the
// homography K R K^-1 — exact for the wall at any depth. Bilinear sampling,
// mid-gray outside.
std::vector<std::uint8_t> render_rolled_ramp(const std::vector<std::uint8_t> & base, double yaw_rad)
{
  std::vector<std::uint8_t> out(base.size(), 128);
  const double c = std::cos(yaw_rad);
  const double sn = std::sin(yaw_rad);
  for (std::uint32_t v = 0; v < kImageHeight; ++v) {
    for (std::uint32_t u = 0; u < kImageWidth; ++u) {
      // K^-1 p, rotate by R(yaw) about z, K again (fx == fy so the scale drops out).
      const double x = (static_cast<double>(u) - kCx) / kFx;
      const double y = (static_cast<double>(v) - kCy) / kFy;
      const double xs = (c * x - sn * y) * kFx + kCx;
      const double ys = (sn * x + c * y) * kFy + kCy;
      if (xs < 0.0 || ys < 0.0 || xs >= kImageWidth - 1.0 || ys >= kImageHeight - 1.0) {
        continue;
      }
      const auto x0 = static_cast<std::size_t>(xs);
      const auto y0 = static_cast<std::size_t>(ys);
      const double fx = xs - static_cast<double>(x0);
      const double fy = ys - static_cast<double>(y0);
      for (std::size_t ch = 0; ch < 3; ++ch) {
        const auto at = [&](std::size_t xx, std::size_t yy) {
          return static_cast<double>(base[(yy * kImageWidth + xx) * 3 + ch]);
        };
        const double g = at(x0, y0) * (1 - fx) * (1 - fy) + at(x0 + 1, y0) * fx * (1 - fy) +
                         at(x0, y0 + 1) * (1 - fx) * fy + at(x0 + 1, y0 + 1) * fx * fy;
        out[(static_cast<std::size_t>(v) * kImageWidth + u) * 3 + ch] =
          static_cast<std::uint8_t>(std::clamp(g, 0.0, 255.0));
      }
    }
  }
  return out;
}

// The scene's wall as one PointCloud2 message in the lidar frame: x/y/z
// float32, a float32 intensity field (omitted when `with_intensity` is false,
// for the rejection test), and — when `with_time_field` is set — an all-zero
// float32 "t" field, the shape a cloud that already went through `pcd
// undistort` carries and which the map accumulation must NOT re-deskew.
pc::PointCloud2 make_wall_cloud(
  const Scene & scene, std::int64_t stamp_ns, bool with_intensity, bool with_time_field)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = kLidarFrame;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(scene.points.size());
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  std::uint32_t offset = 12;
  std::optional<std::uint32_t> intensity_offset;
  if (with_intensity) {
    c.fields.push_back({"intensity", offset, pc::PointFieldType::kFloat32, 1});
    intensity_offset = offset;
    offset += 4;
  }
  if (with_time_field) {
    c.fields.push_back({"t", offset, pc::PointFieldType::kFloat32, 1});
    offset += 4;
  }
  c.point_step = offset;
  c.row_step = c.point_step * c.width;
  c.is_dense = true;
  c.data.resize(static_cast<std::size_t>(c.row_step));
  for (std::size_t i = 0; i < scene.points.size(); ++i) {
    std::byte * base = c.data.data() + i * c.point_step;
    std::memcpy(base, scene.points[i].data(), sizeof(float) * 3);
    if (intensity_offset.has_value()) {
      std::memcpy(base + *intensity_offset, &scene.intensities[i], sizeof(float));
    }
    // The time field, when present, stays all zeros (uniform → no sweep motion).
  }
  return c;
}

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "none";
  return options;
}

bagwiz::io::TopicInfo topic_info(const std::string & name, const std::string & type)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = type;
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::TopicInfo tf_static_topic_info()
{
  bagwiz::io::TopicInfo t = topic_info(kTfStaticTopic, "tf2_msgs/msg/TFMessage");
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

// Everything the fixture bag carries, with defaults matching the happy path:
// a static vehicle (identity poses bracketing the image span), the wall cloud
// stamped inside the pose span.
struct FixtureBagOptions
{
  // The edited static edge(s). The writer appends the identity
  // base_link -> lidar edge the --pcd cloud's frame resolves through.
  std::vector<geometry_msgs::msg::TransformStamped> static_edges;
  std::string optical_frame = kChildFrame;
  std::vector<std::int64_t> image_stamps_ns;
  std::int64_t cloud_stamp_ns = 20'000'000'000LL;
  std::int64_t pose_t0_ns = 10'000'000'000LL;
  std::int64_t pose_t1_ns = 50'000'000'000LL;
  // Extra identity poses between the two span endpoints (empty by default).
  // The --skip-start/--skip-end tests need interior poses so trimming the
  // endpoints still leaves an interpolatable trajectory.
  std::vector<std::int64_t> mid_pose_stamps_ns;
  // Yaw (about z) of the pose at pose_t1_ns; every other pose stays identity,
  // so a non-zero value makes the trajectory turn between the last interior
  // pose and the end of the span. The --cam-offset test needs a trajectory
  // whose interpolated pose actually depends on the lookup time.
  double pose_t1_yaw_rad = 0.0;
  // A continuously yawing trajectory instead: yaw(t) = amplitude * sin(2 pi
  // (t - pose_t0) / period) at every pose (pose_t1_yaw_rad is ignored when
  // the amplitude is non-zero). What `--cam-offset auto` needs: rotation
  // throughout the span for the visual gyro to time against.
  double pose_yaw_amplitude_rad = 0.0;
  double pose_yaw_period_s = 8.0;
  // Render each image as the ramp seen from the yawing camera at the image's
  // TRUE capture time, stamp + image_true_offset_ns (a negative value: the
  // camera clock stamps late). Off: every image is the plain ramp.
  bool render_images_from_pose = false;
  std::int64_t image_true_offset_ns = 0;
  // An Imu topic (kImuTopic, frame kImuFrame) carrying the yaw rate at this
  // rate, each sample stamped imu_latency_ns AFTER its true time. 0 = none.
  // The writer adds the identity base_link -> imu_link static edge.
  double imu_rate_hz = 0.0;
  std::int64_t imu_latency_ns = 0;
  bool cloud_with_intensity = true;
  bool cloud_with_time_field = false;

  [[nodiscard]] double yaw_at(std::int64_t stamp_ns) const
  {
    if (pose_yaw_amplitude_rad != 0.0) {
      const double t = static_cast<double>(stamp_ns - pose_t0_ns) / 1e9;
      return pose_yaw_amplitude_rad * std::sin(2.0 * M_PI * t / pose_yaw_period_s);
    }
    return stamp_ns == pose_t1_ns ? pose_t1_yaw_rad : 0.0;
  }
  [[nodiscard]] double yaw_rate_at(std::int64_t stamp_ns) const
  {
    if (pose_yaw_amplitude_rad == 0.0) {
      return 0.0;
    }
    const double t = static_cast<double>(stamp_ns - pose_t0_ns) / 1e9;
    const double w = 2.0 * M_PI / pose_yaw_period_s;
    return pose_yaw_amplitude_rad * w * std::cos(w * t);
  }
};

// Writes one bag with /tf_static (the edges plus the identity lidar edge),
// /cam/camera_info (one message, whose header.frame_id is the chain's optical
// frame), /cam/image_raw (one bgr8 message per image stamp, all sharing the
// scene's ramp raster), /lidar/points (the scene's wall cloud), and /pose
// (identity PoseStamped in the ref frame at the two span endpoints).
void write_fixture_bag(
  const std::filesystem::path & path, const Scene & scene, const FixtureBagOptions & opts)
{
  auto writer = bagwiz::io::open_write(path, mcap_options());
  writer->declare_topic(tf_static_topic_info());
  writer->declare_topic(topic_info(kImageTopic, "sensor_msgs/msg/Image"));
  writer->declare_topic(topic_info(kCamInfoTopic, "sensor_msgs/msg/CameraInfo"));
  writer->declare_topic(topic_info(kPcdTopic, "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(topic_info(kPoseTopic, "geometry_msgs/msg/PoseStamped"));
  if (opts.imu_rate_hz > 0.0) {
    writer->declare_topic(topic_info(kImuTopic, "sensor_msgs/msg/Imu"));
  }

  auto edges = opts.static_edges;
  geometry_msgs::msg::TransformStamped lidar_edge;
  lidar_edge.header.frame_id = kParentFrame;
  lidar_edge.child_frame_id = kLidarFrame;
  lidar_edge.transform.rotation.w = 1.0;
  edges.push_back(lidar_edge);
  if (opts.imu_rate_hz > 0.0) {
    geometry_msgs::msg::TransformStamped imu_edge;
    imu_edge.header.frame_id = kParentFrame;
    imu_edge.child_frame_id = kImuFrame;
    imu_edge.transform.rotation.w = 1.0;
    edges.push_back(imu_edge);
  }
  const auto tf_cdr = bagwiz::core::serialize_tf_message(edges);
  writer->write(kTfStaticTopic, 0, std::span<const std::byte>(tf_cdr.data(), tf_cdr.size()));

  const auto cam_info_cdr =
    serialize_camera_info(0, 0, opts.optical_frame, kImageWidth, kImageHeight, camera_k());
  writer->write(
    kCamInfoTopic, 0, std::span<const std::byte>(cam_info_cdr.data(), cam_info_cdr.size()));

  for (const std::int64_t stamp_ns : opts.image_stamps_ns) {
    const auto sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
    const auto nsec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
    const std::vector<std::uint8_t> raster =
      opts.render_images_from_pose
        ? render_rolled_ramp(scene.ramp_bgr, opts.yaw_at(stamp_ns + opts.image_true_offset_ns))
        : scene.ramp_bgr;
    const auto image_cdr =
      serialize_image_bgr8(sec, nsec, opts.optical_frame, kImageWidth, kImageHeight, raster);
    writer->write(
      kImageTopic, stamp_ns, std::span<const std::byte>(image_cdr.data(), image_cdr.size()));
  }
  if (opts.imu_rate_hz > 0.0) {
    const auto period_ns = static_cast<std::int64_t>(std::llround(1e9 / opts.imu_rate_hz));
    for (std::int64_t t = opts.pose_t0_ns; t <= opts.pose_t1_ns; t += period_ns) {
      sensor_msgs::msg::Imu imu;
      imu.header.frame_id = kImuFrame;
      const std::int64_t stamp = t + opts.imu_latency_ns;
      imu.header.stamp.sec = static_cast<std::int32_t>(stamp / 1'000'000'000LL);
      imu.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % 1'000'000'000LL);
      imu.angular_velocity.z = opts.yaw_rate_at(t);
      imu.linear_acceleration.z = 9.81;
      imu.orientation.w = 1.0;
      const auto imu_cdr = serialize_typed(imu, "sensor_msgs/msg/Imu");
      writer->write(kImuTopic, stamp, std::span<const std::byte>(imu_cdr.data(), imu_cdr.size()));
    }
  }

  const auto cloud = make_wall_cloud(
    scene, opts.cloud_stamp_ns, opts.cloud_with_intensity, opts.cloud_with_time_field);
  const auto cloud_cdr = pc::serialize_pointcloud2(cloud);
  writer->write(
    kPcdTopic, opts.cloud_stamp_ns, std::span<const std::byte>(cloud_cdr.data(), cloud_cdr.size()));

  std::vector<std::int64_t> pose_stamps_ns{opts.pose_t0_ns};
  pose_stamps_ns.insert(
    pose_stamps_ns.end(), opts.mid_pose_stamps_ns.begin(), opts.mid_pose_stamps_ns.end());
  pose_stamps_ns.push_back(opts.pose_t1_ns);
  for (const std::int64_t stamp_ns : pose_stamps_ns) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = kRefFrame;
    pose.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
    pose.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
    const double yaw_rad = opts.yaw_at(stamp_ns);
    pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
    const auto pose_cdr = serialize_typed(pose, "geometry_msgs/msg/PoseStamped");
    writer->write(
      kPoseTopic, stamp_ns, std::span<const std::byte>(pose_cdr.data(), pose_cdr.size()));
  }
  writer->close();
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
  args.pcd_topic = kPcdTopic;
  args.pose_topic = kPoseTopic;
  args.cam_topic = kImageTopic;
  args.of_frame = kParentFrame;
  args.ref_frame = kRefFrame;
  args.parent_frame = kParentFrame;
  args.child_frame = kChildFrame;
  args.output_path = (tmp_dir / "out.yaml").string();
  args.samples = 5;
  args.max_rot_deg = 2.0;
  args.min_depth = 1.0;
  args.max_depth = 50.0;
  // The scene is a single synthetic sweep of 1800 points with no repeated
  // measurements — the redundancy the default voxel grid exists to collapse is
  // not there, so the grid could only thin the correlation signal these tests
  // recover an injected error from. VoxelGridSurvivesTheRunPath covers the
  // grid end to end instead.
  args.voxel_size = 0.0;
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
// they are trusted inside a bag.
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

TEST(CalibCamLidarFixtureSerializersTest, PointCloud2RoundTrips)
{
  const auto scene = build_scene();
  const auto cloud = make_wall_cloud(scene, 20'000'000'000LL, true, true);
  const auto payload = pc::serialize_pointcloud2(cloud);
  const auto parsed = pc::parse_pointcloud2(payload);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  EXPECT_EQ(parsed.cloud->timestamp_ns, 20'000'000'000LL);
  EXPECT_EQ(parsed.cloud->frame_id, kLidarFrame);
  EXPECT_EQ(parsed.cloud->width, scene.points.size());
  EXPECT_TRUE(parsed.cloud->field_offset("intensity").has_value());
  EXPECT_TRUE(parsed.cloud->field_offset("t").has_value());
  ASSERT_EQ(parsed.cloud->data.size(), cloud.data.size());
  float x = 0.0F;
  std::memcpy(&x, parsed.cloud->data.data(), sizeof(float));
  EXPECT_FLOAT_EQ(x, scene.points[0][0]);
}

TEST_F(CalibCamLidarTest, RejectsCloudWithoutIntensity)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  opts.cloud_with_intensity = false;
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsMissingPcdTopic)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.pcd_topic = "/no/such/topic";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsMissingPoseTopic)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.pose_topic = "/no/such/topic";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsEdgeNotOnChain)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.child_frame = "bogus";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsWhenImageStampsMissTrajectory)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  // The pose topic spans 100..200 s while the images stay at 15..43 s, so no
  // image stamp falls inside the (margin-shrunk) trajectory span. The cloud is
  // stamped inside the pose span so the map itself accumulates fine.
  opts.pose_t0_ns = 100'000'000'000LL;
  opts.pose_t1_ns = 200'000'000'000LL;
  opts.cloud_stamp_ns = 150'000'000'000LL;
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  const auto args = base_args(tmp_dir_);
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RecoversInjectedYawIntoYaml)
{
  const auto scene = build_scene();
  // The bag's base_link -> cam_link edge is off by +1 deg of yaw from the
  // identity pose the images were actually rendered at; refine must find
  // delta yaw ~= -1 deg so the refined edge is (near) identity again.
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(kInjectedYawRad)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

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

// --skip-start/--skip-end end to end: the skipped ranges are measured from
// the bag's time extent ([0, 50] s here), so 12 s/8 s trims the 10 s and 50 s
// endpoint poses and shrinks the trajectory span to [20, 40] s — and the run
// still recovers the injected yaw from what remains inside the window.
TEST_F(CalibCamLidarTest, SkipStartEndShrinkTheTrajectorySpan)
{
  const auto scene = build_scene();
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(kInjectedYawRad)};
  opts.image_stamps_ns = default_image_stamps_ns();
  opts.mid_pose_stamps_ns = {20'000'000'000LL, 30'000'000'000LL, 40'000'000'000LL};
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";
  args.skip_start = "12s";
  args.skip_end = "8s";

  ::testing::internal::CaptureStderr();
  const int rc = run_calib_cam_lidar(args);
  const std::string logs = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(rc, 0) << logs;
  ASSERT_TRUE(std::filesystem::exists(args.output_path));
  // The trim must actually have engaged and shrunk the span to [20, 40] s
  // (20.000 s wide), not silently passed the full trajectory through.
  EXPECT_NE(logs.find("Skipping"), std::string::npos) << logs;
  EXPECT_NE(logs.find("spanning 20.000 s"), std::string::npos) << logs;

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  const auto rpy = bagwiz::core::quaternion_to_rpy(parsed.transforms->front().transform.rotation);
  EXPECT_NEAR(rpy.yaw, 0.0, 0.15 * M_PI / 180.0);
}

TEST_F(CalibCamLidarTest, RejectsSkipsCoveringTheWholeBag)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  // The bag spans [0, 50] s; 30 s + 30 s covers it and more.
  args.skip_start = "30s";
  args.skip_end = "30s";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

TEST_F(CalibCamLidarTest, RejectsSkipsLeavingTooFewPoses)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  // The window [0, 45] s trims the 50 s endpoint pose, leaving only the 10 s
  // one — too few to interpolate.
  args.skip_end = "5s";
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

// --cam-offset end to end. The trajectory is identity through 35 s and then
// turns to +1.5 deg of yaw at 50 s, while the images are stamped 37..45 s but
// were "really" taken 20 s earlier, at identity poses (the bag's camera clock
// ran 20 s late). Without the offset every sample is placed at a turned pose
// (0.2..1.0 deg) and the refinement absorbs that as a spurious edge yaw; with
// `--cam-offset -20s` the samples land back on the identity poses and the
// refined edge stays where the bag put it. The same bag, the same images, the
// same map — only the image-stamp lookup time moves.
TEST_F(CalibCamLidarTest, CamOffsetPlacesImagesAtTheShiftedPose)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  for (const int sec : {37, 39, 41, 43, 45}) {
    opts.image_stamps_ns.push_back(static_cast<std::int64_t>(sec) * 1'000'000'000LL);
  }
  opts.mid_pose_stamps_ns = {20'000'000'000LL, 30'000'000'000LL, 35'000'000'000LL};
  opts.pose_t1_yaw_rad = 1.5 * M_PI / 180.0;
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  const auto refined_yaw = [&](const std::string & cam_offset, std::string * report) {
    auto args = base_args(tmp_dir_);
    args.fix_axes = "x,y,z";
    args.cam_offset = cam_offset;
    args.json = true;
    args.overwrite = true;
    ::testing::internal::CaptureStdout();
    ::testing::internal::CaptureStderr();
    const int rc = run_calib_cam_lidar(args);
    *report = ::testing::internal::GetCapturedStdout();
    const std::string logs = ::testing::internal::GetCapturedStderr();
    EXPECT_EQ(rc, 0) << logs;
    EXPECT_TRUE(std::filesystem::exists(args.output_path));
    const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
    EXPECT_TRUE(parsed.ok()) << parsed.error;
    return bagwiz::core::quaternion_to_rpy(parsed.transforms->front().transform.rotation).yaw;
  };

  std::string report_unshifted;
  const double yaw_unshifted = refined_yaw("", &report_unshifted);
  // Placed at turned poses, the refinement must have moved the edge's yaw
  // well away from the bag value (the per-sample compensation is 0.2..1.0
  // deg, so the compromise sits far above the 0.15 deg recovery tolerance).
  EXPECT_GT(std::abs(yaw_unshifted), 0.3 * M_PI / 180.0);
  EXPECT_NE(report_unshifted.find("\"cam_offset_ns\": 0"), std::string::npos) << report_unshifted;

  std::string report_shifted;
  const double yaw_shifted = refined_yaw("-20s", &report_shifted);
  EXPECT_NEAR(yaw_shifted, 0.0, 0.15 * M_PI / 180.0);
  EXPECT_NE(report_shifted.find("\"cam_offset_ns\": -20000000000"), std::string::npos)
    << report_shifted;
}

// The `--cam-offset auto` fixture: a trajectory yawing +-3 deg with an 8 s
// period across 10..50 s (poses at 10 Hz), images at 5 Hz over 15..45 s
// stamped 120 ms late (each shows the camera 120 ms before its stamp), the
// wall cloud stamped where the yaw passes through zero (18 s) so the map sits
// at the identity pose the scene was built for, and the ramp dotted so the
// tracker has corners.
FixtureBagOptions auto_offset_fixture()
{
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  for (std::int64_t t = 15'000'000'000LL; t <= 45'000'000'000LL; t += 200'000'000LL) {
    opts.image_stamps_ns.push_back(t);
  }
  for (std::int64_t t = 10'100'000'000LL; t < 50'000'000'000LL; t += 100'000'000LL) {
    opts.mid_pose_stamps_ns.push_back(t);
  }
  opts.pose_yaw_amplitude_rad = 3.0 * M_PI / 180.0;
  opts.pose_yaw_period_s = 8.0;
  opts.cloud_stamp_ns = 18'000'000'000LL;
  opts.render_images_from_pose = true;
  opts.image_true_offset_ns = -120'000'000LL;
  return opts;
}

// The `cam_offset_ns` / `offset_ns` the JSON report carries.
std::int64_t json_int_field(const std::string & json, const std::string & field)
{
  const std::string key = "\"" + field + "\": ";
  const std::size_t pos = json.find(key);
  EXPECT_NE(pos, std::string::npos) << "missing '" << field << "' in:\n" << json;
  if (pos == std::string::npos) {
    return 0;
  }
  return std::stoll(json.substr(pos + key.size()));
}

// `--cam-offset auto` against the trajectory: the estimate lands on the
// -120 ms the images were stamped by, is applied (cam_offset_ns), and the
// refined edge stays at the bag value it was rendered for.
TEST_F(CalibCamLidarTest, CamOffsetAutoEstimatesAgainstTheTrajectory)
{
  const auto scene = build_scene(/*with_dots=*/true);
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, auto_offset_fixture());

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";
  args.cam_offset = "auto";
  args.json = true;
  ::testing::internal::CaptureStdout();
  ::testing::internal::CaptureStderr();
  const int rc = run_calib_cam_lidar(args);
  const std::string report = ::testing::internal::GetCapturedStdout();
  const std::string logs = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(rc, 0) << logs;
  ASSERT_TRUE(std::filesystem::exists(args.output_path));
  EXPECT_NE(logs.find("Estimated --cam-offset"), std::string::npos) << logs;
  EXPECT_NE(report.find("\"method\": \"trajectory\""), std::string::npos) << report;
  const std::int64_t applied = json_int_field(report, "cam_offset_ns");
  EXPECT_NEAR(static_cast<double>(applied) / 1e6, -120.0, 30.0) << report;
  EXPECT_EQ(json_int_field(report, "offset_ns"), applied) << report;

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  const auto rpy = bagwiz::core::quaternion_to_rpy(parsed.transforms->front().transform.rotation);
  EXPECT_NEAR(rpy.yaw, 0.0, 0.2 * M_PI / 180.0);
}

// The same with --imu: both legs of the bridge are reported, the gyro's own
// +50 ms latency cancels, and the estimate is the same -120 ms.
TEST_F(CalibCamLidarTest, CamOffsetAutoBridgesThroughTheImu)
{
  const auto scene = build_scene(/*with_dots=*/true);
  auto opts = auto_offset_fixture();
  opts.imu_rate_hz = 100.0;
  opts.imu_latency_ns = 50'000'000LL;
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";
  args.cam_offset = "auto";
  args.imu_topic = kImuTopic;
  args.json = true;
  ::testing::internal::CaptureStdout();
  ::testing::internal::CaptureStderr();
  const int rc = run_calib_cam_lidar(args);
  const std::string report = ::testing::internal::GetCapturedStdout();
  const std::string logs = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(rc, 0) << logs;
  EXPECT_NE(report.find("\"method\": \"imu\""), std::string::npos) << report;
  EXPECT_NEAR(static_cast<double>(json_int_field(report, "cam_offset_ns")) / 1e6, -120.0, 30.0)
    << report;
  // camera vs gyro carries the gyro latency (+50 ms) on top of the -120 ms;
  // pose vs gyro is the latency alone.
  EXPECT_NEAR(
    static_cast<double>(json_int_field(report, "camera_imu_offset_ns")) / 1e6, -70.0, 30.0)
    << report;
  EXPECT_NEAR(static_cast<double>(json_int_field(report, "pose_imu_offset_ns")) / 1e6, 50.0, 20.0)
    << report;
}

TEST_F(CalibCamLidarTest, RejectsImuWithoutAuto)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.imu_topic = kImuTopic;  // no --cam-offset auto
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

// A static bag has no rotation to time against: `auto` must refuse rather
// than apply a meaningless zero.
TEST_F(CalibCamLidarTest, CamOffsetAutoFailsWithoutRotation)
{
  const auto scene = build_scene(/*with_dots=*/true);
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.cam_offset = "auto";
  ::testing::internal::CaptureStderr();
  const int rc = run_calib_cam_lidar(args);
  const std::string logs = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(rc, 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
  EXPECT_NE(logs.find("could not estimate the camera stamp offset"), std::string::npos) << logs;
}

// An offset that carries every image stamp outside the (margin-shrunk)
// trajectory span is the same failure as images that never overlapped it:
// the run stops before any map work rather than silently running on nothing.
TEST_F(CalibCamLidarTest, RejectsCamOffsetPushingEveryImageOutOfSpan)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();  // 15..43 s in a 10..50 s span
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.cam_offset = "40s";  // 55..83 s: nothing left inside [13, 47] s
  EXPECT_EQ(run_calib_cam_lidar(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

// The all-zero "t" field a cloud carries after `pcd undistort` rewrote it:
// uniform per-point times must pass the accumulation's deskew gate untouched
// (no sweep motion to undo) and the command still succeeds end to end.
TEST_F(CalibCamLidarTest, AcceptsCloudWithUniformTimeField)
{
  const auto scene = build_scene();
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(kInjectedYawRad)};
  opts.image_stamps_ns = default_image_stamps_ns();
  opts.cloud_with_time_field = true;
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";

  ASSERT_EQ(run_calib_cam_lidar(args), 0);
  ASSERT_TRUE(std::filesystem::exists(args.output_path));

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  const auto rpy = bagwiz::core::quaternion_to_rpy(parsed.transforms->front().transform.rotation);
  EXPECT_NEAR(rpy.yaw, 0.0, 0.15 * M_PI / 180.0);
}

// --voxel end to end: the grid must reach the accumulated map (not just the
// unit-tested accumulator), collapse it, and still leave a map the run can
// calibrate against. base_args turns the grid off for every other run-path
// test, so this is the one that walks the voxel path.
TEST_F(CalibCamLidarTest, VoxelGridSurvivesTheRunPath)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.voxel_size = 0.1;

  ::testing::internal::CaptureStderr();
  const int rc = run_calib_cam_lidar(args);
  const std::string logs = ::testing::internal::GetCapturedStderr();
  ASSERT_EQ(rc, 0) << logs;
  EXPECT_TRUE(std::filesystem::exists(args.output_path));
  // The log discloses both counts, and the grid must actually have collapsed
  // something: fewer map points than the 1800 the sweep carried.
  EXPECT_NE(logs.find("voxel grid"), std::string::npos) << logs;
  EXPECT_NE(logs.find("(1800 point(s) read"), std::string::npos) << logs;
  const std::size_t on_grid = logs.find("Map: ");
  ASSERT_NE(on_grid, std::string::npos) << logs;
  EXPECT_LT(std::stoul(logs.substr(on_grid + 5)), 1800U) << logs;
}

// The DEFAULT --fix (auto) end to end: on the ramp-wall scene, sliding the
// camera along y changes only the image row, which the histogram cannot see,
// so that direction is flat and must be held at the bag value automatically —
// disclosed in the report — while the observable directions stay free. No
// injected edge error: the run stays near the identity optimum, where the y
// curvature is exactly zero.
TEST_F(CalibCamLidarTest, AutoFixHoldsFlatDirectionAtBagValue)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(0.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);  // fix_axes stays at its "auto" default

  ::testing::internal::CaptureStdout();
  const int rc = run_calib_cam_lidar(args);
  const std::string report = ::testing::internal::GetCapturedStdout();
  ASSERT_EQ(rc, 0) << report;
  ASSERT_TRUE(std::filesystem::exists(args.output_path));
  EXPECT_NE(report.find("held at bag value (auto):"), std::string::npos) << report;

  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(args.output_path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  const auto & t = parsed.transforms->front().transform;
  // The held direction keeps the bag's y (0) within a small leak. The free
  // axes are NOT pinned down here: they drift the way the optimizer always
  // does on this fixture, so only the held axis is asserted.
  EXPECT_NEAR(t.translation.y, 0.0, 1e-3);
}

// --keyframe-dist on the (stationary) fixture: the pose gate collapses to one
// interval, so the command must warn, fall back to plain even time spacing,
// and still succeed end to end — the gate is an optimization, never a new
// failure mode.
TEST_F(CalibCamLidarTest, KeyframeGateFallsBackOnStationaryTrajectory)
{
  const auto scene = build_scene();
  constexpr double kInjectedYawRad = 1.0 * M_PI / 180.0;
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(kInjectedYawRad)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

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
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge_rpy(kTrueRoll, kTruePitch, kTrueYaw + kInjectedYaw)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

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

// -j/--threads end to end: the same fixture refined with every pass on the
// calling thread and with a 4-way pool writes the same bytes, because the map
// is filled in the same order and the NID histograms count the same integers
// whatever the split.
TEST_F(CalibCamLidarTest, ThreadCountDoesNotChangeTheYaml)
{
  const auto scene = build_scene();
  FixtureBagOptions opts;
  opts.static_edges = {make_static_edge(1.0 * M_PI / 180.0)};
  opts.image_stamps_ns = default_image_stamps_ns();
  write_fixture_bag(tmp_dir_ / "bag.mcap", scene, opts);

  auto args = base_args(tmp_dir_);
  args.fix_axes = "x,y,z";
  const auto read_all = [](const std::filesystem::path & path) {
    std::ifstream in(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  };

  args.threads = 1;
  args.output_path = (tmp_dir_ / "sync.yaml").string();
  ASSERT_EQ(run_calib_cam_lidar(args), 0);
  args.threads = 4;
  args.output_path = (tmp_dir_ / "pool.yaml").string();
  ASSERT_EQ(run_calib_cam_lidar(args), 0);

  const auto sync_yaml = read_all(tmp_dir_ / "sync.yaml");
  ASSERT_FALSE(sync_yaml.empty());
  EXPECT_EQ(read_all(tmp_dir_ / "pool.yaml"), sync_yaml);
}
