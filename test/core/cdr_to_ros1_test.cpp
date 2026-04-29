// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_to_ros1.hpp"

#include "bagwiz/core/ros1_message_definitions.hpp"
#include "bagwiz/core/ros1_to_cdr.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

// Reuses the same hand-built ROS 1 wire builder as ros1_to_cdr_test.cpp.
// Kept duplicated rather than extracted into a shared header to keep the
// two test TUs independent.
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

}  // namespace

TEST(CdrToRos1, AutoDerivesArbitraryTypes)
{
  // Any well-formed `pkg/msg/Type` strips the `/msg/` infix to produce
  // the ROS 1 form, regardless of whether the type was previously
  // hard-coded in a whitelist.
  EXPECT_EQ(*bagwiz::core::map_ros2_type("std_msgs/msg/Header"), "std_msgs/Header");
  EXPECT_EQ(
    *bagwiz::core::map_ros2_type("geometry_msgs/msg/PoseStamped"), "geometry_msgs/PoseStamped");
  // tf modernisation: forward had `tf/tfMessage` (legacy) and
  // `tf2_msgs/TFMessage` (modern); reverse always picks the modern form
  // since auto-derive strips `/msg/` directly.
  EXPECT_EQ(*bagwiz::core::map_ros2_type("tf2_msgs/msg/TFMessage"), "tf2_msgs/TFMessage");
  EXPECT_EQ(*bagwiz::core::map_ros2_type("can_msgs/msg/Frame"), "can_msgs/Frame");
  // Custom types not in any historical whitelist now derive cleanly.
  EXPECT_EQ(
    *bagwiz::core::map_ros2_type("autoware_msgs/msg/DetectedObject"),
    "autoware_msgs/DetectedObject");
  EXPECT_EQ(*bagwiz::core::map_ros2_type("foo_pkg/msg/Bar"), "foo_pkg/Bar");
}

TEST(CdrToRos1, MalformedTypeReturnsNullopt)
{
  // Inputs without the `/msg/` infix or with malformed segments must
  // return nullopt so callers skip the topic.
  EXPECT_FALSE(bagwiz::core::map_ros2_type("").has_value());
  EXPECT_FALSE(bagwiz::core::map_ros2_type("just_a_word").has_value());
  EXPECT_FALSE(bagwiz::core::map_ros2_type("foo/Bar").has_value());          // missing /msg/
  EXPECT_FALSE(bagwiz::core::map_ros2_type("/msg/Bar").has_value());         // empty pkg
  EXPECT_FALSE(bagwiz::core::map_ros2_type("foo/msg/").has_value());         // empty type
  EXPECT_FALSE(bagwiz::core::map_ros2_type("foo/msg/Sub/Bar").has_value());  // extra segment
  EXPECT_FALSE(bagwiz::core::map_ros2_type("1foo/msg/Bar").has_value());  // pkg starts with digit
  EXPECT_FALSE(bagwiz::core::map_ros2_type("foo-pkg/msg/Bar").has_value());  // hyphen invalid
}

TEST(CdrToRos1, CanMsgsFrameRos1MetaMatchesUpstream)
{
  // Verified against ROS 1 Noetic (`rosmsg md5 can_msgs/Frame` on
  // ros-noetic-can-msgs 0.8.5). If this assertion fails after a typo
  // or unintended schema edit, ROS 1 consumers of bags written by the
  // 2to1 converter will reject the connection record at read time.
  const auto * meta = bagwiz::core::find_ros1_meta("can_msgs/Frame");
  ASSERT_NE(meta, nullptr);
  EXPECT_EQ(meta->md5sum, "64ae5cebf967dc6aae4e78f5683a5b25");
  // The first stanza of message_definition must be the .msg text as
  // emitted by `rosmsg show -r can_msgs/Frame` on Noetic.
  EXPECT_NE(
    meta->message_definition.find("Header header\nuint32 id\nbool is_rtr\n"), std::string::npos);
  // And the std_msgs/Header dependency must be appended after the
  // standard 80-`=` separator (otherwise ROS 1 readers can't resolve
  // the Header type).
  EXPECT_NE(meta->message_definition.find("MSG: std_msgs/Header"), std::string::npos);
}

namespace
{

// Round-trip helper: ROS 1 input -> CDR -> ROS 1 output. The Header `seq`
// field is dropped on the forward path and synthesized as zero on the
// backward path, so callers that include a Header in their fixture must
// pass `seq = 0` on input (or accept a known mismatch on that one field).
::testing::AssertionResult roundtrip_equal(
  const std::string & ros2_type, const std::vector<std::byte> & ros1_in)
{
  const auto fwd = bagwiz::core::convert_ros1_to_cdr(ros2_type, ros1_in);
  if (!fwd.ok) {
    return ::testing::AssertionFailure() << "forward conversion failed: " << fwd.error;
  }
  const auto back = bagwiz::core::convert_cdr_to_ros1(ros2_type, fwd.cdr);
  if (!back.ok) {
    return ::testing::AssertionFailure() << "backward conversion failed: " << back.error;
  }
  if (back.ros1.size() != ros1_in.size()) {
    return ::testing::AssertionFailure()
           << "size mismatch: in=" << ros1_in.size() << " out=" << back.ros1.size();
  }
  for (std::size_t i = 0; i < ros1_in.size(); ++i) {
    if (ros1_in[i] != back.ros1[i]) {
      return ::testing::AssertionFailure() << "byte mismatch at offset " << i;
    }
  }
  return ::testing::AssertionSuccess();
}

}  // namespace

TEST(CdrToRos1, RoundTripsStdMsgsString)
{
  Ros1Builder b;
  b.string("hello world");
  EXPECT_TRUE(roundtrip_equal("std_msgs/msg/String", b.view()));
}

TEST(CdrToRos1, RoundTripsStdMsgsStringEmpty)
{
  Ros1Builder b;
  b.string("");
  EXPECT_TRUE(roundtrip_equal("std_msgs/msg/String", b.view()));
}

TEST(CdrToRos1, RoundTripsHeaderWithZeroSeq)
{
  // Header roundtrip: forward drops seq, backward synthesizes seq=0.
  // Input must therefore have seq=0 to be byte-equal on the way back.
  Ros1Builder b;
  b.u32(0);                   // seq
  b.time(1700000000, 12345);  // stamp
  b.string("base_link");      // frame_id
  EXPECT_TRUE(roundtrip_equal("std_msgs/msg/Header", b.view()));
}

TEST(CdrToRos1, HeaderBackwardSynthesizesZeroSeq)
{
  // If the original ROS 1 had a non-zero seq, forward drops it; backward
  // emits zero. Verify that explicitly: backward output has seq=0
  // regardless of forward input.
  Ros1Builder b;
  b.u32(42);
  b.time(1700000000, 12345);
  b.string("base_link");

  const auto fwd = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/Header", b.view());
  ASSERT_TRUE(fwd.ok) << fwd.error;

  const auto back = bagwiz::core::convert_cdr_to_ros1("std_msgs/msg/Header", fwd.cdr);
  ASSERT_TRUE(back.ok) << back.error;

  // First 4 bytes of ROS 1 Header are the seq (u32 little-endian).
  ASSERT_GE(back.ros1.size(), 4u);
  std::uint32_t seq = 0;
  std::memcpy(&seq, back.ros1.data(), 4);
  EXPECT_EQ(seq, 0u);
}

TEST(CdrToRos1, RoundTripsVector3)
{
  Ros1Builder b;
  b.f64(1.5);
  b.f64(-2.25);
  b.f64(3.75);
  EXPECT_TRUE(roundtrip_equal("geometry_msgs/msg/Vector3", b.view()));
}

TEST(CdrToRos1, RoundTripsTransformStampedZeroSeq)
{
  Ros1Builder b;
  // header with seq=0 for round-trip equality
  b.u32(0);
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
  EXPECT_TRUE(roundtrip_equal("geometry_msgs/msg/TransformStamped", b.view()));
}

TEST(CdrToRos1, RoundTripsTfMessageWithSequenceOfTransforms)
{
  Ros1Builder b;
  b.u32(2);  // 2 transforms
  // transform #1
  b.u32(0);
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
  b.u32(0);
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
  EXPECT_TRUE(roundtrip_equal("tf2_msgs/msg/TFMessage", b.view()));
}

TEST(CdrToRos1, RoundTripsCanMsgsFrame)
{
  // can_msgs/Frame from ros-industrial/ros_canopen. Wire layout matches
  // ROS 2; the only structural difference is the ROS 1 Header.seq
  // prefix (synthesized as zero on the way back).
  Ros1Builder b;
  // header (seq=0 for round-trip)
  b.u32(0);
  b.time(123, 456);
  b.string("can0");
  // id
  b.u32(0x18FEF100);
  // bools
  b.u8(0);  // is_rtr
  b.u8(1);  // is_extended
  b.u8(0);  // is_error
  // dlc
  b.u8(8);
  // data[8]
  for (std::uint8_t i = 0; i < 8; ++i) {
    b.u8(static_cast<std::uint8_t>(0xA0 + i));
  }
  EXPECT_TRUE(roundtrip_equal("can_msgs/msg/Frame", b.view()));
}

TEST(CdrToRos1, RoundTripsImuWithCovariance)
{
  // Imu = Header + Quaternion + 9*f64 + Vector3 + 9*f64 + Vector3 + 9*f64.
  Ros1Builder b;
  // header (seq=0 for round-trip)
  b.u32(0);
  b.time(123, 456);
  b.string("imu_link");
  // orientation (Quaternion: 4 * f64)
  b.f64(0.0);
  b.f64(0.0);
  b.f64(0.0);
  b.f64(1.0);
  // orientation_covariance 9 f64
  for (int i = 0; i < 9; ++i) {
    b.f64(static_cast<double>(i));
  }
  // angular_velocity (Vector3)
  b.f64(0.1);
  b.f64(0.2);
  b.f64(0.3);
  for (int i = 0; i < 9; ++i) {
    b.f64(static_cast<double>(10 + i));
  }
  // linear_acceleration (Vector3)
  b.f64(1.0);
  b.f64(2.0);
  b.f64(3.0);
  for (int i = 0; i < 9; ++i) {
    b.f64(static_cast<double>(20 + i));
  }
  EXPECT_TRUE(roundtrip_equal("sensor_msgs/msg/Imu", b.view()));
}

TEST(CdrToRos1, FailsForUnknownSrcType)
{
  std::vector<std::byte> dummy_cdr = {
    std::byte{0x00}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}};
  const auto result = bagwiz::core::convert_cdr_to_ros1("foo_pkg/msg/Bar", dummy_cdr);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("introspection"), std::string::npos) << result.error;
}

TEST(CdrToRos1, FailsForTruncatedEncapsulationHeader)
{
  // Input shorter than 4-byte CDR encapsulation header.
  std::vector<std::byte> tiny = {std::byte{0x00}, std::byte{0x01}};
  const auto result = bagwiz::core::convert_cdr_to_ros1("std_msgs/msg/String", tiny);
  EXPECT_FALSE(result.ok);
}

TEST(CdrToRos1, AcceptsTrailingPaddingFromEncapsulationOptions)
{
  // OMG DDS-XTYPES 1.3 §7.6.3.1.2: the lower two bits of
  // representation_options (byte 3 of the encapsulation header) encode
  // the count of pad bytes (0-3) appended after the body so the total
  // ends on a 4-byte boundary. FastDDS / CycloneDDS emit this; the
  // converter must trim those bytes instead of reporting "trailing
  // bytes after decoding". Reproduces the failure observed on
  // CameraInfo / NavSatFix / CompressedImage payloads from real bags.
  Ros1Builder b;
  b.string("hi");
  const auto fwd = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/String", b.view());
  ASSERT_TRUE(fwd.ok) << fwd.error;

  for (std::uint8_t pad = 1; pad <= 3; ++pad) {
    std::vector<std::byte> padded = fwd.cdr;
    padded[3] = std::byte{pad};  // encode pad count in options LSB
    for (std::uint8_t i = 0; i < pad; ++i) {
      padded.push_back(std::byte{0});
    }

    const auto back = bagwiz::core::convert_cdr_to_ros1("std_msgs/msg/String", padded);
    ASSERT_TRUE(back.ok) << "pad=" << static_cast<int>(pad) << ": " << back.error;
    ASSERT_EQ(back.ros1.size(), b.view().size());
    for (std::size_t i = 0; i < b.view().size(); ++i) {
      EXPECT_EQ(back.ros1[i], b.view()[i])
        << "byte " << i << " (pad=" << static_cast<int>(pad) << ")";
    }
  }
}

TEST(CdrToRos1, RejectsNonZeroTrailingBytesNotAccountedForByOptions)
{
  // Regression guard for the trailing-pad heuristic: when options claim
  // zero pad bytes, NON-zero trailing data must still surface as a
  // decode failure (the heuristic only tolerates zero bytes that bring
  // the total to a 4-byte boundary).
  Ros1Builder b;
  b.string("hi");
  const auto fwd = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/String", b.view());
  ASSERT_TRUE(fwd.ok) << fwd.error;

  std::vector<std::byte> with_garbage = fwd.cdr;
  ASSERT_EQ(static_cast<std::uint8_t>(with_garbage[3]) & 0x03, 0u);
  with_garbage.push_back(std::byte{0xAB});
  with_garbage.push_back(std::byte{0xCD});
  with_garbage.push_back(std::byte{0xEF});

  const auto back = bagwiz::core::convert_cdr_to_ros1("std_msgs/msg/String", with_garbage);
  EXPECT_FALSE(back.ok);
  EXPECT_NE(back.error.find("trailing"), std::string::npos) << back.error;
}

TEST(CdrToRos1, AcceptsBodyAlignmentPaddingWithoutOptionsSignal)
{
  // Real bags from rmw_fastrtps + rosbag2_storage_mcap leave the
  // encapsulation header at the literal `00 01 00 00` of PLAIN_CDR_LE
  // (representation_options = 0) but still pad the body to a 4-byte
  // boundary, leaving 1-3 zero bytes after the last field. Honor this
  // pragmatic convention so realistic payloads decode rather than
  // failing the strict spec check.
  Ros1Builder b;
  b.string("hi");
  const auto fwd = bagwiz::core::convert_ros1_to_cdr("std_msgs/msg/String", b.view());
  ASSERT_TRUE(fwd.ok) << fwd.error;
  ASSERT_EQ(static_cast<std::uint8_t>(fwd.cdr[3]) & 0x03, 0u);

  // Pad up to a 4-byte boundary with zeros, leaving options=0 untouched.
  std::vector<std::byte> padded = fwd.cdr;
  while (padded.size() % 4 != 0) {
    padded.push_back(std::byte{0});
  }
  // Force at least one trailing pad byte so the test exercises the path
  // even if the original payload happened to land on a 4-byte boundary.
  if (padded.size() == fwd.cdr.size()) {
    padded.insert(padded.end(), 4U, std::byte{0});
  }

  const auto back = bagwiz::core::convert_cdr_to_ros1("std_msgs/msg/String", padded);
  ASSERT_TRUE(back.ok) << back.error;
  ASSERT_EQ(back.ros1.size(), b.view().size());
  for (std::size_t i = 0; i < b.view().size(); ++i) {
    EXPECT_EQ(back.ros1[i], b.view()[i]) << "byte " << i;
  }
}
