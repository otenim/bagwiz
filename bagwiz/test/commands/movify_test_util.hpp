// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_TEST_UTIL_HPP_
#define COMMANDS__MOVIFY_TEST_UTIL_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Fixtures shared by the movify unit tests: a per-test tmp directory, small
// MCAP bags with declared image / camera-info / point-cloud topics, and a
// minimal CDR builder for a raw bgr8 image payload.
namespace bagwiz::test
{

inline constexpr const char * kMovifyImageType = "sensor_msgs/msg/Image";
inline constexpr const char * kMovifyCameraInfoType = "sensor_msgs/msg/CameraInfo";
inline constexpr const char * kMovifyPointCloudType = "sensor_msgs/msg/PointCloud2";

// A per-test tmp directory, removed on teardown.
class MovifyTmpDirTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_gvc_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

inline bagwiz::io::CreateOptions movify_mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

inline void movify_declare_topic(
  bagwiz::io::BagWriter & w, const std::string & name, const std::string & type)
{
  bagwiz::io::TopicInfo info;
  info.name = name;
  info.type = type;
  info.serialization_format = "cdr";
  w.declare_topic(info);
}

// Four bytes no image decoder accepts, for topics whose payloads are never
// decoded (timestamp-only scans, nearest-message matching).
inline constexpr std::array<std::byte, 4> kMovifyGarbagePayload{
  std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};

// A bag with a single raw-image topic and `frames` garbage-payload messages at
// 100 ms spacing starting at 1 s (scan reads timestamps only, never payloads).
inline std::filesystem::path movify_write_image_bag(
  const std::filesystem::path & dir, const std::string & name, int frames)
{
  const auto path = dir / name;
  auto w = bagwiz::io::open_write(path, movify_mcap_options());
  movify_declare_topic(*w, "/cam/image", kMovifyImageType);
  for (int i = 0; i < frames; ++i) {
    w->write("/cam/image", 1'000'000'000LL + i * 100'000'000LL, kMovifyGarbagePayload);
  }
  w->close();
  return path;
}

// Little-endian CDR-1 builder for the image decode fixture.
class MovifyCdrBuilder
{
public:
  MovifyCdrBuilder()
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
  std::vector<std::byte> take() { return std::move(buf_); }

private:
  void align(std::size_t n)
  {
    while (buf_.size() % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

// A sensor_msgs/msg/Image payload: w x h bgr8, every byte `fill`, no stamp.
inline std::vector<std::byte> movify_bgr8_image_payload(
  std::uint32_t w, std::uint32_t h, std::uint8_t fill)
{
  std::vector<std::byte> data(static_cast<std::size_t>(w) * h * 3, std::byte{fill});
  MovifyCdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str("bgr8");
  b.u8(0);       // is_bigendian
  b.u32(w * 3);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

// A sensor_msgs/msg/PointCloud2 payload: float32 x/y/z points (point_step
// 12) stamped `timestamp_ns` in `frame_id`.
inline std::vector<std::byte> movify_pointcloud2_payload(
  std::int64_t timestamp_ns, const std::string & frame_id,
  const std::vector<std::array<float, 3>> & points)
{
  constexpr std::uint32_t kPointStep = 12;
  std::vector<std::byte> data(points.size() * kPointStep, std::byte{0});
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(data.data() + i * kPointStep, points[i].data(), kPointStep);
  }
  MovifyCdrBuilder b;
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

// A bag with one PointCloud2 topic `topic` in frame `frame_id` carrying
// `clouds` one-point clouds at 100 ms spacing starting at 1 s, the i-th
// point at x = i + 1.
inline std::filesystem::path movify_write_cloud_bag(
  const std::filesystem::path & dir, const std::string & name, const std::string & topic,
  const std::string & frame_id, int clouds)
{
  const auto path = dir / name;
  auto w = bagwiz::io::open_write(path, movify_mcap_options());
  movify_declare_topic(*w, topic, kMovifyPointCloudType);
  for (int i = 0; i < clouds; ++i) {
    const std::int64_t ts = 1'000'000'000LL + i * 100'000'000LL;
    const auto payload =
      movify_pointcloud2_payload(ts, frame_id, {{{static_cast<float>(i + 1), 0.0F, 0.0F}}});
    w->write(topic, ts, payload);
  }
  w->close();
  return path;
}

inline void movify_write_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << content;
}

inline std::string movify_read_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace bagwiz::test

#endif  // COMMANDS__MOVIFY_TEST_UTIL_HPP_
