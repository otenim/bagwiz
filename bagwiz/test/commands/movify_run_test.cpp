// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/movify.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "core/image/image_fixture.hpp"
#include "movify_map_basemap.hpp"    // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_panel.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_tiles.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_track.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "movify_test_util.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "topic_slot_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::commands::check_video_source;
using bagwiz::commands::MovifyArgs;
using bagwiz::commands::run_movify;
using bagwiz::commands::VideoSourceStatus;

// Little-endian CDR-1 builder, matching the wire format the production reader
// consumes (see raw_image_test.cpp for the alignment rationale).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void f64(double v)
  {
    align(8);
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(v));
    std::memcpy(&bits, &v, sizeof(v));
    for (std::size_t i = 0; i < 8; ++i) {
      buf_.push_back(static_cast<std::byte>((bits >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }
  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

// Serialize a sensor_msgs/msg/Image with tightly-packed (step = width*3) pixels.
std::vector<std::byte> make_image_payload(
  std::uint32_t w, std::uint32_t h, const std::string & encoding, std::uint8_t fill)
{
  std::vector<std::byte> data(static_cast<std::size_t>(w) * h * 3, std::byte{fill});
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str(encoding);
  b.u8(0);       // is_bigendian
  b.u32(w * 3);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

// Serialize a sensor_msgs/msg/CompressedImage carrying `format` and the given
// compressed bytes.
std::vector<std::byte> make_compressed_payload(
  const std::string & format, std::span<const std::byte> data)
{
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.str(format);
  b.byte_seq(data);
  return b.take();
}

// Serialize a sensor_msgs/msg/CameraInfo with the given intrinsics. Distortion
// coefficients are empty and the rectification matrix is identity.
std::vector<std::byte> make_camera_info_payload(
  std::uint32_t w, std::uint32_t h, const std::array<double, 9> & k,
  const std::string & frame_id = "cam")
{
  CdrBuilder b;
  b.i32(0);            // header.stamp.sec
  b.u32(0);            // header.stamp.nanosec
  b.str(frame_id);     // header.frame_id
  b.u32(h);            // height
  b.u32(w);            // width
  b.str("plumb_bob");  // distortion_model
  b.u32(0);            // d.length
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(k[i]);
  }
  // r = identity
  constexpr std::array<double, 9> identity_r{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 9; ++i) {
    b.f64(identity_r[i]);
  }
  // p = [K 0]
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      double value = 0.0;
      if (col < 3) {
        value = k[row * 3 + col];
      }
      b.f64(value);
    }
  }
  b.u32(0);  // binning_x
  b.u32(0);  // binning_y
  b.u32(0);  // roi.x_offset
  b.u32(0);  // roi.y_offset
  b.u32(0);  // roi.width
  b.u32(0);  // roi.height
  b.u8(0);   // roi.do_rectify
  return b.take();
}

// Serialize a sensor_msgs/msg/PointCloud2 with float32 x/y/z fields.
std::vector<std::byte> make_pointcloud2_payload(
  std::int64_t timestamp_ns, const std::string & frame_id,
  const std::vector<std::array<float, 3>> & points)
{
  constexpr std::uint32_t kPointStep = 12;
  std::vector<std::byte> data(points.size() * kPointStep, std::byte{0});
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(data.data() + i * kPointStep, points[i].data(), kPointStep);
  }

  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL));   // sec
  b.u32(static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL));  // nanosec
  b.str(frame_id);
  b.u32(1);                                          // height
  b.u32(static_cast<std::uint32_t>(points.size()));  // width
  b.u32(3);                                          // fields length
  b.str("x");
  b.u32(0);
  b.u8(7);  // float32
  b.u32(1);
  b.str("y");
  b.u32(4);
  b.u8(7);
  b.u32(1);
  b.str("z");
  b.u32(8);
  b.u8(7);
  b.u32(1);
  b.u8(0);                                         // is_bigendian
  b.u32(kPointStep);                               // point_step
  b.u32(static_cast<std::uint32_t>(data.size()));  // row_step
  b.byte_seq({data.data(), data.size()});
  b.u8(1);  // is_dense
  return b.take();
}

// Build a tf2_msgs/msg/TFMessage CDR payload for a static edge parent <- child.
std::vector<std::byte> make_tf_static_payload(const std::string & parent, const std::string & child)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = 0;
  ts.header.stamp.nanosec = 0;
  ts.child_frame_id = child;
  ts.transform.translation.x = 0.0;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.0;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;
  std::vector<geometry_msgs::msg::TransformStamped> transforms{ts};
  return bagwiz::core::serialize_tf_message(transforms);
}

// A sensor_msgs/msg/NavSatFix payload with a fix at the given position.
std::vector<std::byte> make_navsatfix_payload(
  std::int64_t stamp_ns, double latitude, double longitude, double altitude)
{
  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));
  b.u32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));
  b.str("gnss");
  b.u8(0);  // NavSatStatus.status: FIX
  // NavSatStatus.service (uint16, GPS): the stamp's 4+4 bytes, the 4+5-byte
  // frame_id "gnss" and the status put it at body offset 18 — even, so no
  // alignment padding precedes it.
  b.u8(1);
  b.u8(0);
  b.f64(latitude);
  b.f64(longitude);
  b.f64(altitude);
  for (int i = 0; i < 9; ++i) {
    b.f64(0.0);  // position_covariance
  }
  b.u8(0);  // position_covariance_type: UNKNOWN
  return b.take();
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

constexpr const char * kImageTopic = "/cam/image";
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedTopic = "/cam/image/compressed";
constexpr const char * kCompressedType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// Build an MCAP bag with an Image topic (`frames` messages at 100 ms spacing,
// WxH, `encoding`) plus a CompressedImage topic and a lidar topic (declared so
// type-based validation can be exercised).
std::filesystem::path build_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h,
  const std::string & encoding)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kImageTopic, kImageType));
  writer->declare_topic(make_topic("/cam/image/compressed", "sensor_msgs/msg/CompressedImage"));
  writer->declare_topic(make_topic("/sensing/lidar", "sensor_msgs/msg/PointCloud2"));
  for (int i = 0; i < frames; ++i) {
    const auto payload = make_image_payload(w, h, encoding, static_cast<std::uint8_t>(i * 20));
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write(kImageTopic, ts, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// Build an MCAP bag whose CompressedImage topic carries `frames` JPEG messages
// (WxH solid colors at 100 ms spacing). The per-frame fill varies so the encoded
// frames differ.
std::filesystem::path build_compressed_bag(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kCompressedTopic, kCompressedType));
  for (int i = 0; i < frames; ++i) {
    const auto jpeg =
      bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
    const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write(kCompressedTopic, ts, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// Build an MCAP bag with an image topic named `image_topic` plus a sibling
// `/camera_info` topic. When `compressed` is true the image topic carries JPEG
// frames; otherwise it carries raw bgr8 frames. The CameraInfo is sized to match
// the image dimensions.
std::filesystem::path build_bag_with_camera_info(
  const std::filesystem::path & dir, const std::string & image_topic, bool compressed, int frames,
  std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());

  const std::string camera_info_topic =
    bagwiz::core::camera_info::resolve_camera_info_topic_name(image_topic)
      .value_or(image_topic + "/camera_info");

  writer->declare_topic(make_topic(image_topic, compressed ? kCompressedType : kImageType));
  writer->declare_topic(make_topic(camera_info_topic, kCameraInfoType));

  const std::array<double, 9> k{
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  const auto camera_info_payload = make_camera_info_payload(w, h, k);
  writer->write(
    camera_info_topic, 1'000'000'000LL, {camera_info_payload.data(), camera_info_payload.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    if (compressed) {
      const auto jpeg =
        bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
      const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    } else {
      const auto payload = make_image_payload(w, h, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    }
  }
  writer->close();
  return path;
}

// Build an MCAP bag with an image topic, sibling CameraInfo, a PointCloud2 topic,
// and a /tf_static edge from the camera frame to the cloud frame. The cloud
// contains one point per frame straight ahead of the camera.
std::filesystem::path build_bag_with_pointcloud_overlay(
  const std::filesystem::path & dir, const std::string & image_topic, bool compressed, int frames,
  std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());

  const std::string camera_info_topic = [image_topic]() {
    std::string stem = image_topic;
    for (const auto & suffix :
         {"/image_raw/compressed", "/image_rect_color/compressed", "/image_rect_color"}) {
      if (
        stem.size() > std::string_view{suffix}.size() &&
        std::string_view{stem}.substr(stem.size() - std::string_view{suffix}.size()) == suffix) {
        stem.resize(stem.size() - std::string_view{suffix}.size());
        break;
      }
    }
    return stem + "/camera_info";
  }();

  writer->declare_topic(make_topic(image_topic, compressed ? kCompressedType : kImageType));
  writer->declare_topic(make_topic(camera_info_topic, kCameraInfoType));
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

  const std::array<double, 9> k{
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  const auto camera_info_payload = make_camera_info_payload(w, h, k, "cam");
  writer->write(
    camera_info_topic, 1'000'000'000LL, {camera_info_payload.data(), camera_info_payload.size()});

  const auto tf_payload = make_tf_static_payload("cam", "lidar");
  writer->write("/tf_static", 1'000'000'000LL, {tf_payload.data(), tf_payload.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    const auto pcd_payload = make_pointcloud2_payload(ts, "lidar", {{0.0f, 0.0f, 5.0f}});
    writer->write("/points", ts, {pcd_payload.data(), pcd_payload.size()});

    if (compressed) {
      const auto jpeg =
        bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
      const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    } else {
      const auto payload = make_image_payload(w, h, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    }
  }
  writer->close();
  return path;
}

// Build an MCAP bag with an image topic, sibling CameraInfo, two PointCloud2
// topics, and /tf_static edges from the camera frame to each cloud frame.
std::filesystem::path build_bag_with_two_pointcloud_overlays(
  const std::filesystem::path & dir, const std::string & image_topic, bool compressed, int frames,
  std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());

  const std::string camera_info_topic = [image_topic]() {
    std::string stem = image_topic;
    for (const auto & suffix :
         {"/image_raw/compressed", "/image_rect_color/compressed", "/image_rect_color"}) {
      if (
        stem.size() > std::string_view{suffix}.size() &&
        std::string_view{stem}.substr(stem.size() - std::string_view{suffix}.size()) == suffix) {
        stem.resize(stem.size() - std::string_view{suffix}.size());
        break;
      }
    }
    return stem + "/camera_info";
  }();

  writer->declare_topic(make_topic(image_topic, compressed ? kCompressedType : kImageType));
  writer->declare_topic(make_topic(camera_info_topic, kCameraInfoType));
  writer->declare_topic(make_topic("/points/front", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/points/rear", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

  const std::array<double, 9> k{
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  const auto camera_info_payload = make_camera_info_payload(w, h, k, "cam");
  writer->write(
    camera_info_topic, 1'000'000'000LL, {camera_info_payload.data(), camera_info_payload.size()});

  const auto tf_front = make_tf_static_payload("cam", "lidar_front");
  const auto tf_rear = make_tf_static_payload("cam", "lidar_rear");
  writer->write("/tf_static", 1'000'000'000LL, {tf_front.data(), tf_front.size()});
  writer->write("/tf_static", 1'000'000'000LL, {tf_rear.data(), tf_rear.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    const auto front_payload = make_pointcloud2_payload(ts, "lidar_front", {{0.0f, 0.0f, 5.0f}});
    const auto rear_payload = make_pointcloud2_payload(ts, "lidar_rear", {{0.0f, 0.0f, 5.0f}});
    writer->write("/points/front", ts, {front_payload.data(), front_payload.size()});
    writer->write("/points/rear", ts, {rear_payload.data(), rear_payload.size()});

    if (compressed) {
      const auto jpeg =
        bagwiz::test::encode_still_image("jpeg", w, h, static_cast<std::uint8_t>(i * 20), 100, 50);
      const auto payload = make_compressed_payload("jpeg", {jpeg.data(), jpeg.size()});
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    } else {
      const auto payload = make_image_payload(w, h, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    }
  }
  writer->close();
  return path;
}

// True if any entry in `dir` looks like a leftover movify temp file.
bool any_partial_left(const std::filesystem::path & dir)
{
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    if (entry.path().filename().string().find(".bagwiz-partial") != std::string::npos) {
      return true;
    }
  }
  return false;
}

class MovifyRunTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_movify_run_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }
  std::filesystem::path tmp_dir_;
};

// ---- check_video_source ---------------------------------------------------

TEST_F(MovifyRunTest, CheckOkForImage)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto c = check_video_source(in, kImageTopic);
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  EXPECT_EQ(c.topic_type, kImageType);
}

TEST_F(MovifyRunTest, CheckOkForCompressedImage)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto c = check_video_source(in, "/cam/image/compressed");
  EXPECT_EQ(c.status, VideoSourceStatus::kOk);
  EXPECT_EQ(c.topic_type, "sensor_msgs/msg/CompressedImage");
}

TEST_F(MovifyRunTest, CheckTopicNotFound)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  EXPECT_EQ(check_video_source(in, "/nope").status, VideoSourceStatus::kTopicNotFound);
}

TEST_F(MovifyRunTest, CheckUnsupportedType)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  EXPECT_EQ(check_video_source(in, "/sensing/lidar").status, VideoSourceStatus::kUnsupportedType);
}

TEST_F(MovifyRunTest, CheckInputUnopenable)
{
  EXPECT_EQ(
    check_video_source(tmp_dir_ / "does_not_exist", kImageTopic).status,
    VideoSourceStatus::kInputUnopenable);
}

// ---- run_movify: failure paths ------------------------------------

TEST_F(MovifyRunTest, RunMissingTopicFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  const MovifyArgs args{in, "/nope", out, false};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, RunUnsupportedTypeFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  const MovifyArgs args{in, "/sensing/lidar", out, false};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// A CompressedImage topic carrying a payload that is neither JPEG nor PNG (by
// its magic bytes) stops the run with no output and no leftover temp.
TEST_F(MovifyRunTest, RunCompressedImageUnrecognizedFormatFails)
{
  const auto path = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
    writer->declare_topic(make_topic(kCompressedTopic, kCompressedType));
    const std::vector<std::byte> garbage(8, std::byte{0x01});
    const auto payload = make_compressed_payload("weird", {garbage.data(), garbage.size()});
    writer->write(kCompressedTopic, 1'000'000'000LL, {payload.data(), payload.size()});
    writer->write(kCompressedTopic, 1'100'000'000LL, {payload.data(), payload.size()});
    writer->close();
  }
  const auto out = tmp_dir_ / "out.avi";
  const MovifyArgs args{path, kCompressedTopic, out, false};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
}

TEST_F(MovifyRunTest, RunUnsupportedEncodingFailsAndLeavesNothing)
{
  const auto in = build_bag(tmp_dir_, 3, 16, 16, "mono16");
  const auto out = tmp_dir_ / "out.avi";
  const MovifyArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
}

TEST_F(MovifyRunTest, RunOddDimensionsFails)
{
  const auto in = build_bag(tmp_dir_, 3, 15, 16, "bgr8");  // odd width
  const auto out = tmp_dir_ / "out.avi";
  const MovifyArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));
}

// ---- run_movify: success path -------------------------------------

TEST_F(MovifyRunTest, RunEncodesImageTopicToVideo)
{
  constexpr int kFrames = 4;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";  // MJPEG: no libx264 dependency

  const MovifyArgs args{in, kImageTopic, out, false};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));  // no temp left behind

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// A CompressedImage (JPEG) topic decodes frame-by-frame and encodes to a video
// with the decoded geometry and frame count — the headline new capability.
TEST_F(MovifyRunTest, RunEncodesCompressedImageTopicToVideo)
{
  constexpr int kFrames = 4;
  const auto in = build_compressed_bag(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";  // MJPEG: no libx264 dependency

  const MovifyArgs args{in, kCompressedTopic, out, false};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_FALSE(any_partial_left(tmp_dir_));

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, RunExistingOutputWithoutOverwriteFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  {
    std::ofstream f(out, std::ios::binary);
    f << "SENTINEL";
  }

  const MovifyArgs args{in, kImageTopic, out, false};
  EXPECT_EQ(run_movify(args), 1);

  // The pre-existing file is left untouched.
  std::ifstream f(out, std::ios::binary);
  const std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, "SENTINEL");
}

TEST_F(MovifyRunTest, RunOverwriteReplacesExistingOutput)
{
  constexpr int kFrames = 4;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";
  {
    std::ofstream f(out, std::ios::binary);
    f << "SENTINEL";
  }

  const MovifyArgs args{in, kImageTopic, out, true};  // --overwrite
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

// ---- camera-info auto-resolution ------------------------------------------

TEST_F(MovifyRunTest, AutoResolvesCameraInfoForImageRawCompressed)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_raw/compressed", true, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_raw/compressed", out, false};
  args.rectify = true;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

// The composed output size is a product the user never types — grid columns
// times the primary frame's cell size — so a run reports one that has grown
// past 4K. --width drives the cell size here, which keeps the source frames
// (and the fixture bag) tiny while still composing an oversized canvas.
TEST_F(MovifyRunTest, RunWarnsWhenTheComposedOutputIsOversized)
{
  constexpr int kFrames = 2;
  // 4096 wide at the source's 16:9 aspect composes 4096x2304 = 9.4 Mpx.
  const auto in = build_bag(tmp_dir_, kFrames, 32, 18, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.width = 4096;
  args.enable_parallel_pipeline = false;

  ::testing::internal::CaptureStderr();
  ASSERT_EQ(run_movify(args), 0);
  const std::string err = ::testing::internal::GetCapturedStderr();

  EXPECT_NE(err.find("4096x2304"), std::string::npos) << err;
  EXPECT_NE(err.find("larger than 3840x2160"), std::string::npos) << err;
  // A single view has no grid worth naming.
  EXPECT_EQ(err.find("grid of"), std::string::npos) << err;
}

// An ordinary single-view render is silent: the guard must not nag about sizes
// nobody would call surprising.
TEST_F(MovifyRunTest, RunStaysQuietForAnOrdinaryOutputSize)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.enable_parallel_pipeline = false;

  ::testing::internal::CaptureStderr();
  ASSERT_EQ(run_movify(args), 0);
  const std::string err = ::testing::internal::GetCapturedStderr();

  EXPECT_EQ(err.find("larger than"), std::string::npos) << err;
}

TEST_F(MovifyRunTest, AutoResolvesCameraInfoForImageRectColor)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.rectify = true;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, AutoResolvesCameraInfoForImageRectColorCompressed)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_rect_color/compressed", true, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color/compressed", out, false};
  args.rectify = true;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

// With rectification on by default, a bag without a derivable CameraInfo
// still renders: the view is left unrectified with a warning rather than
// failing the run.
TEST_F(MovifyRunTest, RectifyWithoutCameraInfoWarnsAndRendersUnrectified)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");  // /cam/image, no /cam/camera_info
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

// --no-rectify opts out even when a CameraInfo is available.
TEST_F(MovifyRunTest, NoRectifyOptsOut)
{
  constexpr int kFrames = 2;
  const auto in =
    build_bag_with_camera_info(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.rectify = false;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, ExplicitCameraInfoTopicWorksForRectify)
{
  constexpr int kFrames = 2;
  const auto in = tmp_dir_ / "input";

  // Build a bag with /cam/image and an unrelated /other/camera_info topic so
  // auto-resolution fails and explicit selection is required.
  {
    auto writer = bagwiz::io::open_write(in, mcap_dir_opts());
    writer->declare_topic(make_topic(kImageTopic, kImageType));
    writer->declare_topic(make_topic("/other/camera_info", kCameraInfoType));
    const std::array<double, 9> k{16, 0, 8, 0, 16, 8, 0, 0, 1};
    const auto ci_payload = make_camera_info_payload(16, 16, k);
    writer->write("/other/camera_info", 1'000'000'000LL, {ci_payload.data(), ci_payload.size()});
    for (int i = 0; i < kFrames; ++i) {
      const auto payload = make_image_payload(16, 16, "bgr8", static_cast<std::uint8_t>(i * 20));
      const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
      writer->write(kImageTopic, ts, {payload.data(), payload.size()});
    }
    writer->close();
  }

  const auto out = tmp_dir_ / "out.avi";
  MovifyArgs args{in, kImageTopic, out, false};
  args.camera_info_entries = {"/other/camera_info"};
  args.rectify = true;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, ExplicitCameraInfoTopicWithWrongTypeFails)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.camera_info_entries = {"/sensing/lidar"};  // PointCloud2, not CameraInfo
  args.rectify = true;
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// ---- point-cloud overlay ----------------------------------------------------

TEST_F(MovifyRunTest, PointCloudTopicWithWrongTypeFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.cam_pcd_entries = {kImageTopic};  // Image, not PointCloud2
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, PointCloudTopicNotFoundFails)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.cam_pcd_entries = {"/nope"};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, PointCloudOverlayRequiresCameraInfo)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");  // /cam/image, no /cam/camera_info
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.cam_pcd_entries = {"/sensing/lidar"};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, PointCloudOverlayWorksOnRawImage)
{
  constexpr int kFrames = 3;
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.cam_pcd_entries = {"/points"};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, PointCloudOverlayWorksOnCompressedImage)
{
  constexpr int kFrames = 3;
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_raw/compressed", true, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_raw/compressed", out, false};
  args.cam_pcd_entries = {"/points"};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

// Read a file into `out` for bit-exact comparison. The helper is void so it can
// use gtest's fatal FAIL() macro if the file cannot be opened.
void read_file_bytes(const std::filesystem::path & path, std::vector<std::byte> & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    FAIL() << "could not open " << path;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  const std::string content = ss.str();
  out.clear();
  out.reserve(content.size());
  for (unsigned char c : content) {
    out.push_back(static_cast<std::byte>(c));
  }
}

TEST_F(MovifyRunTest, ThreadedPointCloudOverlayMatchesSynchronous)
{
  // Use enough frames to satisfy the internal threshold for threaded projection.
  constexpr int kFrames = 6;
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out_threaded = tmp_dir_ / "out_threaded.avi";
  const auto out_sync = tmp_dir_ / "out_sync.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out_threaded, false};
  args.cam_pcd_entries = {"/points"};
  args.enable_parallel_pipeline = true;
  ASSERT_EQ(run_movify(args), 0);

  args.output_path = out_sync;
  args.enable_parallel_pipeline = false;
  ASSERT_EQ(run_movify(args), 0);

  std::vector<std::byte> threaded_bytes;
  std::vector<std::byte> sync_bytes;
  read_file_bytes(out_threaded, threaded_bytes);
  read_file_bytes(out_sync, sync_bytes);
  EXPECT_EQ(threaded_bytes, sync_bytes);
}

TEST_F(MovifyRunTest, MultiplePointCloudTopicsOverlayWorks)
{
  constexpr int kFrames = 3;
  const auto in = build_bag_with_two_pointcloud_overlays(
    tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.cam_pcd_entries = {"/points/front", "/points/rear"};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, MultiplePointCloudTopicsFailIfOneHasWrongType)
{
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_rect_color", false, 2, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.cam_pcd_entries = {"/points", "/cam/image_rect_color"};  // one of them is an image
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, MultiplePointCloudTopicsFailIfOneMissing)
{
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_rect_color", false, 2, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.cam_pcd_entries = {"/points", "/also_missing"};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, ThreadedMultiPointCloudOverlayMatchesSynchronous)
{
  constexpr int kFrames = 6;
  const auto in = build_bag_with_two_pointcloud_overlays(
    tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out_threaded = tmp_dir_ / "out_threaded.avi";
  const auto out_sync = tmp_dir_ / "out_sync.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out_threaded, false};
  args.cam_pcd_entries = {"/points/front", "/points/rear"};
  args.enable_parallel_pipeline = true;
  ASSERT_EQ(run_movify(args), 0);

  args.output_path = out_sync;
  args.enable_parallel_pipeline = false;
  ASSERT_EQ(run_movify(args), 0);

  std::vector<std::byte> threaded_bytes;
  std::vector<std::byte> sync_bytes;
  read_file_bytes(out_threaded, threaded_bytes);
  read_file_bytes(out_sync, sync_bytes);
  EXPECT_EQ(threaded_bytes, sync_bytes);
}

// ---- multi-view grid ----------------------------------------------------------

// Build an MCAP bag with two raw-bgr8 image topics: /cam/a (16x16 frames at
// 100 ms spacing) and /cam/b (8x8 frames at 250 ms spacing, offset 50 ms).
std::filesystem::path build_two_camera_bag(
  const std::filesystem::path & dir, int primary_frames, int secondary_frames)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/cam/a", kImageType));
  writer->declare_topic(make_topic("/cam/b", kImageType));
  for (int i = 0; i < primary_frames; ++i) {
    const auto payload = make_image_payload(16, 16, "bgr8", static_cast<std::uint8_t>(i * 20));
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write("/cam/a", ts, {payload.data(), payload.size()});
  }
  for (int i = 0; i < secondary_frames; ++i) {
    const auto payload = make_image_payload(8, 8, "bgr8", static_cast<std::uint8_t>(200 - i * 20));
    const std::int64_t ts = 1'050'000'000LL + static_cast<std::int64_t>(i) * 250'000'000LL;
    writer->write("/cam/b", ts, {payload.data(), payload.size()});
  }
  writer->close();
  return path;
}

// Build an MCAP bag with two raw image topics (each with a sibling CameraInfo
// in the same "cam" frame), one PointCloud2 topic, and the /tf_static edge
// from the camera frame to the cloud frame.
std::filesystem::path build_two_camera_bag_with_pointcloud(
  const std::filesystem::path & dir, int frames, std::uint32_t w, std::uint32_t h)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/cam/a/image_rect_color", kImageType));
  writer->declare_topic(make_topic("/cam/b/image_rect_color", kImageType));
  writer->declare_topic(make_topic("/cam/a/camera_info", kCameraInfoType));
  writer->declare_topic(make_topic("/cam/b/camera_info", kCameraInfoType));
  writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));

  const std::array<double, 9> k{
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  for (const auto * info_topic : {"/cam/a/camera_info", "/cam/b/camera_info"}) {
    const auto camera_info_payload = make_camera_info_payload(w, h, k, "cam");
    writer->write(
      info_topic, 1'000'000'000LL, {camera_info_payload.data(), camera_info_payload.size()});
  }

  const auto tf_payload = make_tf_static_payload("cam", "lidar");
  writer->write("/tf_static", 1'000'000'000LL, {tf_payload.data(), tf_payload.size()});

  for (int i = 0; i < frames; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    const auto pcd_payload = make_pointcloud2_payload(ts, "lidar", {{0.0f, 0.0f, 5.0f}});
    writer->write("/points", ts, {pcd_payload.data(), pcd_payload.size()});
    for (const auto * image_topic : {"/cam/a/image_rect_color", "/cam/b/image_rect_color"}) {
      const auto payload = make_image_payload(w, h, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(image_topic, ts, {payload.data(), payload.size()});
    }
  }
  writer->close();
  return path;
}

TEST_F(MovifyRunTest, TwoViewsRenderSideBySideAtPrimaryRate)
{
  constexpr int kFrames = 4;
  const auto in = build_two_camera_bag(tmp_dir_, kFrames, 2);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a", out, false};
  args.cam_topics.push_back("/cam/b");
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  // The auto grid for two views is 2x1; the cell is the primary's 16x16.
  EXPECT_EQ(probe.width, 32U);
  EXPECT_EQ(probe.height, 16U);
  // The primary topic drives the output timing and frame count.
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, ExplicitGridStacksViewsVertically)
{
  const auto in = build_two_camera_bag(tmp_dir_, 2, 2);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a", out, false};
  args.cam_topics.push_back("/cam/b");
  args.grid = "1x2";
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 32U);
}

TEST_F(MovifyRunTest, GridSmallerThanTheViewCountFails)
{
  const auto in = build_two_camera_bag(tmp_dir_, 2, 2);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a", out, false};
  args.cam_topics.push_back("/cam/b");
  args.grid = "1x1";
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, GlobalPointCloudProjectsOntoEveryView)
{
  constexpr int kFrames = 3;
  const auto in = build_two_camera_bag_with_pointcloud(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a/image_rect_color", out, false};
  args.cam_topics.push_back("/cam/b/image_rect_color");
  args.cam_pcd_entries = {"/points"};
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 32U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, PerViewPointCloudBindingRenders)
{
  constexpr int kFrames = 3;
  const auto in = build_two_camera_bag_with_pointcloud(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a/image_rect_color", out, false};
  args.cam_topics.push_back("/cam/b/image_rect_color");
  args.cam_pcd_entries = {"/cam/a/image_rect_color=/points"};
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 32U);
  EXPECT_EQ(probe.height, 16U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, PerViewBindingToAnUnlistedViewFails)
{
  const auto in = build_two_camera_bag_with_pointcloud(tmp_dir_, 2, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a/image_rect_color", out, false};
  args.cam_pcd_entries = {"/cam/not_a_view=/points"};
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(MovifyRunTest, ThreadedTwoViewOverlayMatchesSynchronous)
{
  // Use enough frames to satisfy the internal threshold for threaded projection.
  constexpr int kFrames = 6;
  const auto in = build_two_camera_bag_with_pointcloud(tmp_dir_, kFrames, 16, 16);
  const auto out_threaded = tmp_dir_ / "out_threaded.avi";
  const auto out_sync = tmp_dir_ / "out_sync.avi";

  MovifyArgs args{in, "/cam/a/image_rect_color", out_threaded, false};
  args.cam_topics.push_back("/cam/b/image_rect_color");
  args.cam_pcd_entries = {"/cam/a/image_rect_color=/points"};
  args.enable_parallel_pipeline = true;
  ASSERT_EQ(run_movify(args), 0);

  args.output_path = out_sync;
  args.enable_parallel_pipeline = false;
  ASSERT_EQ(run_movify(args), 0);

  std::vector<std::byte> threaded_bytes;
  std::vector<std::byte> sync_bytes;
  read_file_bytes(out_threaded, threaded_bytes);
  read_file_bytes(out_sync, sync_bytes);
  EXPECT_EQ(threaded_bytes, sync_bytes);
}

// Multiple views without --pcd also run the parallel pipeline; it must produce
// the same bytes as the synchronous loop there too.
TEST_F(MovifyRunTest, ThreadedMultiViewWithoutPointCloudMatchesSynchronous)
{
  // Use enough frames to satisfy the internal threshold for the parallel pipeline.
  constexpr int kFrames = 6;
  const auto in = build_two_camera_bag(tmp_dir_, kFrames, kFrames);
  const auto out_threaded = tmp_dir_ / "out_threaded.avi";
  const auto out_sync = tmp_dir_ / "out_sync.avi";

  MovifyArgs args{in, "/cam/a", out_threaded, false};
  args.cam_topics.push_back("/cam/b");
  args.enable_parallel_pipeline = true;
  ASSERT_EQ(run_movify(args), 0);

  args.output_path = out_sync;
  args.enable_parallel_pipeline = false;
  ASSERT_EQ(run_movify(args), 0);

  std::vector<std::byte> threaded_bytes;
  std::vector<std::byte> sync_bytes;
  read_file_bytes(out_threaded, threaded_bytes);
  read_file_bytes(out_sync, sync_bytes);
  EXPECT_EQ(threaded_bytes, sync_bytes);
}

// ---- --width ------------------------------------------------------------------

// --width fixes the composed output width: with two views on the auto 2x1
// grid, a 16 px width leaves an 8 px cell, and the cell height follows the
// primary frame's aspect ratio.
TEST_F(MovifyRunTest, WidthDerivesTheCellSizeFromTheOutputWidth)
{
  constexpr int kFrames = 3;
  const auto in = build_two_camera_bag(tmp_dir_, kFrames, 2);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/a", out, false};
  args.cam_topics.push_back("/cam/b");
  args.width = 16;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16U);
  EXPECT_EQ(probe.height, 8U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// Single-view, --width behaves like a plain output-width constraint.
TEST_F(MovifyRunTest, WidthUpscalesASingleView)
{
  constexpr int kFrames = 2;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.width = 32;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 32U);
  EXPECT_EQ(probe.height, 32U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

TEST_F(MovifyRunTest, WidthConflictsWithResize)
{
  const auto in = build_bag(tmp_dir_, 2, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.width = 32;
  args.resize_scale = 0.5f;
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

// A raw image topic can be down-scaled while preserving aspect ratio.
TEST_F(MovifyRunTest, ResizeScalesRawImageDimensions)
{
  constexpr int kFrames = 3;
  const auto in = build_bag(tmp_dir_, kFrames, 16, 16, "bgr8");
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kImageTopic, out, false};
  args.resize_scale = 0.5f;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 8U);
  EXPECT_EQ(probe.height, 8U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// A compressed image topic can be up-scaled while preserving aspect ratio.
TEST_F(MovifyRunTest, ResizeScalesCompressedImageDimensions)
{
  constexpr int kFrames = 3;
  const auto in = build_compressed_bag(tmp_dir_, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, kCompressedTopic, out, false};
  args.resize_scale = 2.0f;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 32U);
  EXPECT_EQ(probe.height, 32U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// Resizing a point-cloud overlay produces output at the scaled resolution while
// keeping the projected points aligned because the camera info is scaled too.
TEST_F(MovifyRunTest, ResizeScalesPointCloudOverlay)
{
  constexpr int kFrames = 3;
  const auto in =
    build_bag_with_pointcloud_overlay(tmp_dir_, "/cam/image_rect_color", false, kFrames, 16, 16);
  const auto out = tmp_dir_ / "out.avi";

  MovifyArgs args{in, "/cam/image_rect_color", out, false};
  args.cam_pcd_entries = {"/points"};
  args.resize_scale = 0.5f;
  ASSERT_EQ(run_movify(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 8U);
  EXPECT_EQ(probe.height, 8U);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// Command-level integration test for the point-cloud overlay path against a real
// rosbag. The bag path comes from the BAGWIZ_REAL_BAG environment variable; the
// test skips gracefully when it is unset or the bag is unavailable (e.g. CI), so
// it only runs on a machine that has the recording. The topic names and expected
// video geometry below assume that specific recording.
TEST_F(MovifyRunTest, PointCloudOverlayOnRealBag)
{
  const char * const real_bag_env = std::getenv("BAGWIZ_REAL_BAG");
  if (real_bag_env == nullptr || !std::filesystem::exists(real_bag_env)) {
    GTEST_SKIP() << "Real test bag not available; set BAGWIZ_REAL_BAG to run this test";
  }
  const std::string kRealBag = real_bag_env;

  const auto out = tmp_dir_ / "out.avi";
  MovifyArgs args{kRealBag, "/sensing/camera/camera0/image_raw/compressed", out, false};
  args.cam_pcd_entries = {"/sensing/lidar/front/seyond_points"};
  ASSERT_EQ(run_movify(args), 0);

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;

  constexpr std::uint32_t kExpectedWidth = 3840U;
  constexpr std::uint32_t kExpectedHeight = 2160U;
  constexpr std::int64_t kExpectedFrames = 600;
  EXPECT_EQ(probe.width, kExpectedWidth);
  EXPECT_EQ(probe.height, kExpectedHeight);
  EXPECT_EQ(probe.frame_count, kExpectedFrames);
  EXPECT_NEAR(probe.duration_s, 30.0, 1.0);
}

// A point-cloud topic alone renders one 3D panel per cloud at the default
// 1280x720 cell, one frame per cloud (the topic is the clock).
TEST_F(MovifyRunTest, RunRendersAPointCloudPanelAlone)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto payload =
        make_pointcloud2_payload(ts, "lidar", {{static_cast<float>(i + 1), 0.0f, 0.0f}});
      writer->write("/points", ts, {payload.data(), payload.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.pcd_topics = {"/points"};
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
  EXPECT_EQ(probe.width, 1280u);
  EXPECT_EQ(probe.height, 720u);
}

// A NavSatFix topic alone renders a map panel in the default 1280x720 cell,
// one frame per message.
TEST_F(MovifyRunTest, RunRendersAMapPanelAlone)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/gnss", "sensor_msgs/msg/NavSatFix"));
    for (int i = 0; i < 4; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto payload = make_navsatfix_payload(ts, 35.0 + i * 1e-4, 139.0, 40.0);
      writer->write("/gnss", ts, {payload.data(), payload.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.gnss_topic = "/gnss";
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 4);
  EXPECT_EQ(probe.width, 1280u);
  EXPECT_EQ(probe.height, 720u);
}

// The map panel over tiles read from a local directory through a file:
// template: the tiles the fitted 1280x720 view of the track needs are laid
// out as <z>/<x>/<y>.png, and the run reads them without any network.
TEST_F(MovifyRunTest, RunDrawsTheMapPanelOverFileTiles)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/gnss", "sensor_msgs/msg/NavSatFix"));
    for (int i = 0; i < 4; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto payload = make_navsatfix_payload(ts, 35.0 + i * 1e-4, 139.0, 40.0);
      writer->write("/gnss", ts, {payload.data(), payload.size()});
    }
    writer->close();
  }
  const auto loaded = bagwiz::commands::load_map_track(bag, "/gnss");
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  bagwiz::commands::MapBasemap::Options basemap_options;
  basemap_options.origin = loaded.track->origin;
  bagwiz::commands::MapBasemap basemap(std::move(basemap_options));
  const auto range = basemap.tile_range_of(
    bagwiz::commands::fit_map_viewport(*loaded.track, bagwiz::commands::PanelSize{1280, 720}));
  ASSERT_FALSE(range.empty());
  const auto tiles = tmp_dir_ / "tiles";
  for (int y = range.y0; y <= range.y1; ++y) {
    for (int x = range.x0; x <= range.x1; ++x) {
      const auto path =
        tiles / std::to_string(range.zoom) / std::to_string(x) / (std::to_string(y) + ".png");
      std::filesystem::create_directories(path.parent_path());
      const cv::Mat tile(
        bagwiz::commands::kMapTileSizePx, bagwiz::commands::kMapTileSizePx, CV_8UC3,
        cv::Scalar(90, 120, 150));
      ASSERT_TRUE(cv::imwrite(path.string(), tile));
    }
  }

  MovifyArgs args;
  args.input_path = bag;
  args.gnss_topic = "/gnss";
  args.map_tiles = "file://" + (tiles / "{z}/{x}/{y}.png").string();
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 4);
}

// Tiles that cannot be fetched are a warning, not a failure: the run draws
// the plain plan view. A malformed template is rejected up front.
TEST_F(MovifyRunTest, RunKeepsGoingWithoutTilesButRejectsABadTemplate)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/gnss", "sensor_msgs/msg/NavSatFix"));
    const std::int64_t ts = 1'000'000'000LL;
    const auto payload = make_navsatfix_payload(ts, 35.0, 139.0, 40.0);
    writer->write("/gnss", ts, {payload.data(), payload.size()});
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.gnss_topic = "/gnss";
  args.map_tiles = "file://" + (tmp_dir_ / "no-such-dir" / "{z}/{x}/{y}.png").string();
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  EXPECT_TRUE(std::filesystem::exists(args.output_path));

  args.map_tiles = "https://tiles.example/{z}/{x}.png";
  args.output_path = tmp_dir_ / "rejected.avi";
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

// A point cloud next to the map: the cloud is the clock, the map follows the
// vehicle at --map-range, and the two compose a 2x1 grid of 1280x720 cells.
TEST_F(MovifyRunTest, RunComposesAPointCloudAndAMapPanel)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
    writer->declare_topic(make_topic("/gnss", "sensor_msgs/msg/NavSatFix"));
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto cloud =
        make_pointcloud2_payload(ts, "lidar", {{static_cast<float>(i + 1), 0.0f, 0.0f}});
      writer->write("/points", ts, {cloud.data(), cloud.size()});
      const auto fix = make_navsatfix_payload(ts, 35.0, 139.0 + i * 1e-4, 40.0);
      writer->write("/gnss", ts, {fix.data(), fix.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.pcd_topics = {"/points"};
  args.gnss_topic = "/gnss";
  args.map_range_m = 50.0;
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
  EXPECT_EQ(probe.width, 2560u);
  EXPECT_EQ(probe.height, 720u);
}

// A camera and a point-cloud topic compose a 2x1 grid whose cell is the
// camera frame; both a 3D and a BEV view add a panel each (3 panels -> 2x2).
TEST_F(MovifyRunTest, RunComposesCameraAndPointCloudPanels)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic(kImageTopic, kImageType));
    writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto image = make_image_payload(8, 4, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(kImageTopic, ts, {image.data(), image.size()});
      const auto cloud =
        make_pointcloud2_payload(ts, "lidar", {{static_cast<float>(i + 1), 0.0f, 0.0f}});
      writer->write("/points", ts, {cloud.data(), cloud.size()});
    }
    writer->close();
  }
  MovifyArgs args(bag, kImageTopic, tmp_dir_ / "out.avi", false);
  args.pcd_topics = {"/points"};
  EXPECT_EQ(run_movify(args), 0);
  auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
  EXPECT_EQ(probe.width, 16u);  // 2x1 grid of 8x4 cells
  EXPECT_EQ(probe.height, 4u);

  args.overwrite = true;
  args.views = {
    bagwiz::core::pointcloud::CloudProjection::kPerspective,
    bagwiz::core::pointcloud::CloudProjection::kBev};
  EXPECT_EQ(run_movify(args), 0);
  probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 16u);  // 3 panels on an auto 2x2 grid
  EXPECT_EQ(probe.height, 8u);
}

// A JPEG camera shown as decoded (no rectification, resize, overlay or other
// panel) streams through the direct pass: every frame lands in the video at
// the decoded geometry.
TEST_F(MovifyRunTest, RunStreamsAJpegCameraDirect)
{
  const auto jpeg = bagwiz::test::encode_still_image("jpeg", 32, 16, 10, 120, 200);
  if (jpeg.empty()) {
    GTEST_SKIP() << "no JPEG encoder in this FFmpeg build";
  }
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic(kCompressedTopic, kCompressedType));
    for (int i = 0; i < 5; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto payload = make_compressed_payload("jpeg", jpeg);
      writer->write(kCompressedTopic, ts, {payload.data(), payload.size()});
    }
    writer->close();
  }
  MovifyArgs args(bag, kCompressedTopic, tmp_dir_ / "out.mp4", false);
  args.rectify = false;
  args.encoder = bagwiz::core::video::H264Backend::kX264;
  {
    bagwiz::core::video::VideoEncoderOptions probe_options;
    probe_options.backend = args.encoder;
    if (!bagwiz::core::video::open_video_encoder(
           tmp_dir_ / "probe.mp4", 32, 16, 10, 1, probe_options)
           .ok()) {
      GTEST_SKIP() << "libx264 unavailable in this FFmpeg build";
    }
  }
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 5);
  EXPECT_EQ(probe.width, 32u);
  EXPECT_EQ(probe.height, 16u);
}

// A point-cloud panel with a --pose trajectory: the Odometry topic is read
// whole and the trajectory drawn over every sweep.
TEST_F(MovifyRunTest, RunDrawsAPoseTrajectoryOverAPointCloudPanel)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
    writer->declare_topic(make_topic("/odom", "nav_msgs/msg/Odometry"));
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const auto tf = make_tf_static_payload("base_link", "lidar");  // the body is known
    writer->write("/tf_static", 1'000'000'000LL, {tf.data(), tf.size()});
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto cloud = make_pointcloud2_payload(ts, "lidar", {{1.0f, 0.0f, 0.0f}});
      writer->write("/points", ts, {cloud.data(), cloud.size()});
      const auto odom = bagwiz::test::movify_odometry_payload(
        ts, "map", "lidar", static_cast<double>(i), 0.0, 0.0, 0.0);
      writer->write("/odom", ts, {odom.data(), odom.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.pcd_topics = {"/points"};
  args.pose_topic = "/odom";
  args.pose_window_s = 1.0;
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
}

constexpr const char * kPoseCameraInfoTopic = "/cam/pose_camera_info";

// A camera panel with a --pose trajectory: the trajectory of the camera's
// own frame projects through its camera info.
TEST_F(MovifyRunTest, RunDrawsAPoseTrajectoryOverACameraPanel)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic(kImageTopic, kImageType));
    writer->declare_topic(make_topic(kPoseCameraInfoTopic, kCameraInfoType));
    writer->declare_topic(make_topic("/odom", "nav_msgs/msg/Odometry"));
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const auto tf = make_tf_static_payload("base_link", "cam");  // the body is known
    writer->write("/tf_static", 1'000'000'000LL, {tf.data(), tf.size()});
    const std::array<double, 9> k{8.0, 0.0, 4.0, 0.0, 4.0, 2.0, 0.0, 0.0, 1.0};
    const auto info = make_camera_info_payload(8, 4, k, "cam");
    writer->write(kPoseCameraInfoTopic, 1'000'000'000LL, {info.data(), info.size()});
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto image = make_image_payload(8, 4, "bgr8", 0x10);
      writer->write(kImageTopic, ts, {image.data(), image.size()});
      const auto odom = bagwiz::test::movify_odometry_payload(
        ts, "map", "cam", 0.0, 0.0, static_cast<double>(i), 0.0);
      writer->write("/odom", ts, {odom.data(), odom.size()});
    }
    writer->close();
  }
  MovifyArgs args(bag, kImageTopic, tmp_dir_ / "out.avi", false);
  args.rectify = false;
  args.camera_info_entries = {kPoseCameraInfoTopic};
  args.pose_topic = "/odom";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
}

// Two lidars in different frames merge into one panel through the bag's TF:
// with --frame naming the vehicle frame, each cloud is looked up at its own
// stamp and the run succeeds.
TEST_F(MovifyRunTest, RunMergesPointCloudTopicsThroughTf)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/front", "sensor_msgs/msg/PointCloud2"));
    writer->declare_topic(make_topic("/rear", "sensor_msgs/msg/PointCloud2"));
    writer->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    for (const auto * child : {"front_lidar", "rear_lidar"}) {
      const auto tf = make_tf_static_payload("base_link", child);
      writer->write("/tf_static", 1'000'000'000LL, {tf.data(), tf.size()});
    }
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto front = make_pointcloud2_payload(ts, "front_lidar", {{5.0f, 0.0f, 0.0f}});
      writer->write("/front", ts, {front.data(), front.size()});
      const auto rear = make_pointcloud2_payload(ts, "rear_lidar", {{-5.0f, 0.0f, 0.0f}});
      writer->write("/rear", ts, {rear.data(), rear.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.pcd_topics = {"/front", "/rear"};
  args.frame = "base_link";
  args.views = {bagwiz::core::pointcloud::CloudProjection::kBev};
  args.range_m = 10.0;
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
  EXPECT_EQ(probe.width, 1280u);
  EXPECT_EQ(probe.height, 720u);
}

// Without TF, a cloud outside the view frame cannot be placed: the run stops
// (leaving no output) instead of drawing it somewhere plausible but wrong.
TEST_F(MovifyRunTest, RunFailsWhenAPointCloudPanelNeedsMissingTf)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic("/front", "sensor_msgs/msg/PointCloud2"));
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto front = make_pointcloud2_payload(ts, "front_lidar", {{5.0f, 0.0f, 0.0f}});
      writer->write("/front", ts, {front.data(), front.size()});
    }
    writer->close();
  }
  MovifyArgs args;
  args.input_path = bag;
  args.pcd_topics = {"/front"};
  args.frame = "base_link";
  args.output_path = tmp_dir_ / "out.avi";
  EXPECT_EQ(run_movify(args), 1);
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
}

// A camera panel next to a point-cloud clock: the sweeps are the frames, the
// cell is the point-cloud panel's 1280x720, and the camera fits into it.
TEST_F(MovifyRunTest, RunComposesCameraAndPointCloudClock)
{
  const auto bag = tmp_dir_ / "input";
  {
    auto writer = bagwiz::io::open_write(bag, mcap_dir_opts());
    writer->declare_topic(make_topic(kImageTopic, kImageType));
    writer->declare_topic(make_topic("/points", "sensor_msgs/msg/PointCloud2"));
    for (int i = 0; i < 3; ++i) {
      const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
      const auto image = make_image_payload(8, 4, "bgr8", static_cast<std::uint8_t>(i * 20));
      writer->write(kImageTopic, ts, {image.data(), image.size()});
      const auto cloud =
        make_pointcloud2_payload(ts, "lidar", {{static_cast<float>(i + 1), 0.0f, 0.0f}});
      writer->write("/points", ts, {cloud.data(), cloud.size()});
    }
    writer->close();
  }
  MovifyArgs args(bag, kImageTopic, tmp_dir_ / "out.avi", false);
  args.pcd_topics = {"/points"};
  args.clock = "/points";
  EXPECT_EQ(run_movify(args), 0);
  const auto probe = bagwiz::core::video::probe_video(args.output_path);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 3);
  EXPECT_EQ(probe.width, 2560u);
  EXPECT_EQ(probe.height, 720u);
}

// Exercises the real MovifyCommand::configure() — reached through the
// process-wide command registry that movify.cpp's BAGWIZ_REGISTER_COMMAND
// registrar populates — rather than a hand-mirrored copy of its wiring. The
// assertions pin down the slot semantics the multi-view surface relies on:
// --cam and --pcd are multi-value glob slots (a glob expands to its matches
// in lexicographic order, so grid placement stays deterministic), neither
// required at the parser level (the run reports "nothing to render" itself),
// --clock is a literal single-topic slot, --cam-info is a literal
// pair-optional slot (bare value or <image>=<info>), and --cam-pcd is a glob
// pair slot whose selector is the right half (<image>=<pcd_selector>).
TEST(MovifyCliWiring, TopicSlotsAreDeclaredWithPairSemantics)
{
  bagwiz::commands::Command * movify_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "movify") {
      movify_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(movify_cmd, nullptr);

  CLI::App app{"movify"};
  movify_cmd->configure(app);
  EXPECT_TRUE(app.get_subcommands({}).empty());  // a single-action command

  const auto slots = bagwiz::commands::topic_slots_of(app);
  ASSERT_EQ(slots.size(), 7U);  // --cam, --pcd, --gnss, --pose, --clock, --cam-info, --cam-pcd

  const auto * cam_slot = bagwiz::test::slot_for(slots, "cam");
  ASSERT_NE(cam_slot, nullptr);
  EXPECT_EQ(cam_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kGlob);
  EXPECT_EQ(cam_slot->spec.allowed_types.size(), 2U);  // Image, CompressedImage
  EXPECT_FALSE(cam_slot->option->get_required());
  EXPECT_NE(cam_slot->multi_target, nullptr);

  const auto * clock_slot = bagwiz::test::slot_for(slots, "clock");
  ASSERT_NE(clock_slot, nullptr);
  EXPECT_EQ(clock_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
  EXPECT_EQ(clock_slot->spec.allowed_types.size(), 4U);  // + PointCloud2, NavSatFix
  EXPECT_FALSE(clock_slot->option->get_required());

  const auto * pcd_panel_slot = bagwiz::test::slot_for(slots, "pcd");
  ASSERT_NE(pcd_panel_slot, nullptr);
  EXPECT_EQ(pcd_panel_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kGlob);
  EXPECT_FALSE(pcd_panel_slot->spec.pair_value);
  EXPECT_FALSE(pcd_panel_slot->option->get_required());

  const auto * cam_info_slot = bagwiz::test::slot_for(slots, "cam-info");
  ASSERT_NE(cam_info_slot, nullptr);
  EXPECT_EQ(cam_info_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
  EXPECT_TRUE(cam_info_slot->spec.pair_value);
  EXPECT_TRUE(cam_info_slot->spec.pair_optional);
  EXPECT_NE(cam_info_slot->multi_target, nullptr);

  const auto * pcd_slot = bagwiz::test::slot_for(slots, "cam-pcd");
  ASSERT_NE(pcd_slot, nullptr);
  EXPECT_EQ(pcd_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kGlob);
  EXPECT_TRUE(pcd_slot->spec.pair_value);
  EXPECT_TRUE(pcd_slot->spec.pair_selector_rhs);
}

// Rectification is requested by default, so the CLI carries the opt-out alone:
// there is no --rectify to ask for what the command already does. That leaves
// MovifyArgs::rectify's initializer as the sole carrier of the default —
// CLI11 leaves a negated-only flag's target untouched when the flag is absent
// — so both halves are pinned here.
TEST(MovifyCliWiring, RectificationIsOptOutOnly)
{
  bagwiz::commands::Command * movify_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "movify") {
      movify_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(movify_cmd, nullptr);

  CLI::App app{"movify"};
  movify_cmd->configure(app);

  EXPECT_NE(app.get_option_no_throw("--no-rectify"), nullptr);
  EXPECT_EQ(app.get_option_no_throw("--rectify"), nullptr);
  EXPECT_TRUE(MovifyArgs{}.rectify);
}

// jet is the colour scheme every bagwiz visualization starts from. Both
// carriers of the default are pinned: the CLI option (CLI11 spells an enum
// default by its underlying value) and MovifyArgs' initializer, which the
// tests that bypass the parser rely on.
TEST(MovifyCliWiring, SchemeDefaultsToJet)
{
  using bagwiz::core::pointcloud::ColorScheme;

  bagwiz::commands::Command * movify_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "movify") {
      movify_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(movify_cmd, nullptr);

  CLI::App app{"movify"};
  movify_cmd->configure(app);

  const auto * scheme = app.get_option("--scheme");
  ASSERT_NE(scheme, nullptr);
  EXPECT_EQ(scheme->get_default_str(), std::to_string(static_cast<int>(ColorScheme::kJet)));
  EXPECT_EQ(MovifyArgs{}.colorscheme, ColorScheme::kJet);
}

// The CLI draws the map from OpenStreetMap unless --map-tiles says
// otherwise, while MovifyArgs itself defaults to no tiles, so a direct
// caller (these tests) never touches the network.
TEST(MovifyCliWiring, MapTilesDefaultToOpenStreetMapOnTheCliOnly)
{
  bagwiz::commands::Command * movify_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "movify") {
      movify_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(movify_cmd, nullptr);

  CLI::App app{"movify"};
  movify_cmd->configure(app);
  const auto * option = app.get_option_no_throw("--map-tiles");
  ASSERT_NE(option, nullptr);
  EXPECT_EQ(option->get_default_str(), bagwiz::commands::kDefaultMapTileTemplate);
  EXPECT_EQ(MovifyArgs{}.map_tiles, bagwiz::commands::kMapTilesNone);
}

}  // namespace
