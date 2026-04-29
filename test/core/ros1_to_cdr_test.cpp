// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_to_cdr.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/message_formatter.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Helpers that emit raw ROS 1 little-endian payloads (no encapsulation,
// no alignment). Tests build a payload with these, run it through
// convert_ros1_to_cdr, then deserialize the CDR result with the same
// pipeline used by `bagwiz walk` to validate the round-trip.

class Ros1Builder
{
public:
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }

  void u32(std::uint32_t v)
  {
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFF));
    }
  }

  void f64(double v)
  {
    std::uint64_t bits;
    std::memcpy(&bits, &v, 8);
    for (int i = 0; i < 8; ++i) {
      buf_.push_back(static_cast<std::byte>((bits >> (8 * i)) & 0xFF));
    }
  }

  void time(std::uint32_t sec, std::uint32_t nsec)
  {
    u32(sec);
    u32(nsec);
  }

  void string(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size()));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(c));
    }
  }

  std::vector<std::byte> take() && { return std::move(buf_); }
  const std::vector<std::byte> & view() const { return buf_; }

private:
  std::vector<std::byte> buf_;
};

std::string format_ros2(const std::string & ros2_type, const std::vector<std::byte> & cdr)
{
  // The Phase D/E pipeline replaces the old IntrospectionLoad + format_message
  // overload with a TopicInfo-driven factory. Build a minimal TopicInfo so the
  // factory routes through introspection (no schema_text), decode the bytes,
  // and feed the resulting Value to the new format_message().
  bagwiz::io::TopicInfo topic;
  topic.name = "/ros1_to_cdr_test";
  topic.type = ros2_type;
  topic.serialization_format = "cdr";

  auto open = bagwiz::core::decoder::open_decoder(topic);
  if (!open.ok()) {
    return "<open fail: " + open.error + ">";
  }
  const auto decoded = open.decoder->decode(cdr);
  if (!decoded.ok()) {
    return "<decode fail: " + decoded.error + ">";
  }
  auto formatted = bagwiz::core::format_message(*decoded.value);
  if (!formatted.ok()) {
    return "<format fail: " + formatted.error + ">";
  }
  return formatted.text;
}

}  // namespace

TEST(Ros1ToCdr, MapsKnownTypes)
{
  EXPECT_EQ(*bagwiz::core::map_ros1_type("std_msgs/Header"), "std_msgs/msg/Header");
  EXPECT_EQ(
    *bagwiz::core::map_ros1_type("geometry_msgs/PoseStamped"), "geometry_msgs/msg/PoseStamped");
  EXPECT_EQ(*bagwiz::core::map_ros1_type("tf/tfMessage"), "tf2_msgs/msg/TFMessage");
  EXPECT_EQ(*bagwiz::core::map_ros1_type("can_msgs/Frame"), "can_msgs/msg/Frame");
}

TEST(Ros1ToCdr, MapsUnknownTypeToNullopt)
{
  EXPECT_FALSE(bagwiz::core::map_ros1_type("foo_pkg/Bar").has_value());
  EXPECT_FALSE(bagwiz::core::map_ros1_type("std_msgs/SomethingWeird").has_value());
}

TEST(Ros1ToCdr, ConvertsStdMsgsString)
{
  // ROS 1 std_msgs/String: just `string data` -> u32 length + bytes.
  Ros1Builder b;
  b.string("hello");

  const auto result = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/String", b.view());
  ASSERT_TRUE(result.ok) << result.error;

  const auto text = format_ros2("std_msgs/msg/String", result.cdr);
  EXPECT_NE(text.find("data: 'hello'"), std::string::npos) << "got:\n" << text;
}

TEST(Ros1ToCdr, ConvertsHeaderDroppingSeq)
{
  // ROS 1 std_msgs/Header layout: u32 seq + Time stamp + string frame_id.
  Ros1Builder b;
  b.u32(42);                  // seq, must be discarded
  b.time(1700000000, 12345);  // stamp.sec, stamp.nsec
  b.string("base_link");      // frame_id

  const auto result = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/Header", b.view());
  ASSERT_TRUE(result.ok) << result.error;

  const auto text = format_ros2("std_msgs/msg/Header", result.cdr);
  EXPECT_NE(text.find("sec: 1700000000"), std::string::npos) << text;
  EXPECT_NE(text.find("nanosec: 12345"), std::string::npos) << text;
  EXPECT_NE(text.find("frame_id: 'base_link'"), std::string::npos) << text;
  // The `42` from seq must not leak through.
  EXPECT_EQ(text.find("42"), std::string::npos) << text;
}

TEST(Ros1ToCdr, ConvertsVector3)
{
  // 3 x float64 = 24 bytes, no padding in either format.
  Ros1Builder b;
  b.f64(1.5);
  b.f64(-2.25);
  b.f64(3.75);

  const auto result = bagwiz::core::convert_ros1_to_cdr("geometry_msgs/msg/Vector3", b.view());
  ASSERT_TRUE(result.ok) << result.error;

  const auto text = format_ros2("geometry_msgs/msg/Vector3", result.cdr);
  EXPECT_NE(text.find("x: 1.5"), std::string::npos) << text;
  EXPECT_NE(text.find("y: -2.25"), std::string::npos) << text;
  EXPECT_NE(text.find("z: 3.75"), std::string::npos) << text;
}

TEST(Ros1ToCdr, ConvertsTransformStamped)
{
  Ros1Builder b;
  // header
  b.u32(7);  // seq -> dropped
  b.time(1700000001, 250);
  b.string("map");
  // child_frame_id
  b.string("base_link");
  // transform.translation
  b.f64(10.0);
  b.f64(20.0);
  b.f64(30.0);
  // transform.rotation
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(1.0);

  const auto result =
    bagwiz::core::convert_ros1_to_cdr("geometry_msgs/msg/TransformStamped", b.view());
  ASSERT_TRUE(result.ok) << result.error;

  const auto text = format_ros2("geometry_msgs/msg/TransformStamped", result.cdr);
  EXPECT_NE(text.find("frame_id: 'map'"), std::string::npos) << text;
  EXPECT_NE(text.find("child_frame_id: 'base_link'"), std::string::npos) << text;
  EXPECT_NE(text.find("x: 10"), std::string::npos) << text;
  EXPECT_NE(text.find("y: 20"), std::string::npos) << text;
  EXPECT_NE(text.find("z: 30"), std::string::npos) << text;
  EXPECT_NE(text.find("w: 1"), std::string::npos) << text;
}

TEST(Ros1ToCdr, ConvertsTfMessageWithSequenceOfTransforms)
{
  // tf2_msgs/TFMessage = sequence<TransformStamped> transforms.
  // ROS 1 raw layout: u32 count + N x TransformStamped (each one
  // includes its own Header with seq dropped).
  Ros1Builder b;
  b.u32(2);  // 2 transforms

  // transform #1
  b.u32(1);
  b.time(100, 0);
  b.string("a");
  b.string("b");
  b.f64(1.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(1.0);

  // transform #2
  b.u32(2);
  b.time(101, 500);
  b.string("c");
  b.string("d");
  b.f64(2.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(1.0);

  const auto result = bagwiz::core::convert_ros1_to_cdr("tf2_msgs/msg/TFMessage", b.view());
  ASSERT_TRUE(result.ok) << result.error;

  const auto text = format_ros2("tf2_msgs/msg/TFMessage", result.cdr);
  EXPECT_NE(text.find("frame_id: 'a'"), std::string::npos) << text;
  EXPECT_NE(text.find("child_frame_id: 'b'"), std::string::npos) << text;
  EXPECT_NE(text.find("frame_id: 'c'"), std::string::npos) << text;
  EXPECT_NE(text.find("child_frame_id: 'd'"), std::string::npos) << text;
}

TEST(Ros1ToCdr, RejectsTrailingBytes)
{
  // Build a valid Vector3 then add an extra trailing byte; conversion
  // should fail rather than silently truncate.
  Ros1Builder b;
  b.f64(1.0);
  b.f64(2.0);
  b.f64(3.0);
  b.u8(0xAA);

  const auto result = bagwiz::core::convert_ros1_to_cdr("geometry_msgs/msg/Vector3", b.view());
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("trailing"), std::string::npos) << result.error;
}

TEST(Ros1ToCdr, FailsForUnknownDestType)
{
  Ros1Builder b;
  const auto result = bagwiz::core::convert_ros1_to_cdr("foo_pkg/msg/Bar", b.view());
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("introspection"), std::string::npos) << result.error;
}
