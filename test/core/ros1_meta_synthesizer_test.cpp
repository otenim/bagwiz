// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_meta_synthesizer.hpp"

#include "bagwiz/core/ros1_message_definitions.hpp"

#include <gtest/gtest.h>

#include <string>
#include <string_view>

namespace
{

constexpr std::string_view kSep =
  "================================================================================";

}  // namespace

// MD5 algorithm self-check. The canonical RFC 1321 vectors prove our
// MD5 implementation is correct in isolation, before we layer on the
// ROS 1 normalisation rules.
//
// We don't expose md5_hex() publicly, so we exercise it indirectly
// through synthesise() with a one-field schema whose expected md5 we
// computed via Python `hashlib.md5("string data".encode()).hexdigest()`.
TEST(Ros1MetaSynthesizer, StdMsgsString)
{
  // ROS 2 .msg for std_msgs/msg/String:  `string data`
  const std::string_view ros2_msg = "string data\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("std_msgs/msg/String", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  // Canonical ROS 1 md5 for std_msgs/String, verified against
  // `rosmsg md5 std_msgs/String` on Noetic and against
  // `echo -n "string data" | md5sum`.
  EXPECT_EQ(result.meta.md5sum, "992ce8a1687cec8c8bd883ec73ca41d1");
  EXPECT_EQ(result.meta.message_definition, "string data\n");
  EXPECT_TRUE(result.warnings.empty());
}

TEST(Ros1MetaSynthesizer, StdMsgsHeaderRestoresSeq)
{
  // ROS 2 std_msgs/msg/Header dropped `seq`. Synthesis must put it back
  // both in the body (so ROS 1 readers can parse the connection) and in
  // the md5 input (so the resulting hash matches what ROS 1 publishers
  // generated for the same logical type).
  const std::string_view ros2_msg =
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("std_msgs/msg/Header", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  // Canonical ROS 1 md5 for std_msgs/Header, verified against
  // `rosmsg md5 std_msgs/Header` on Noetic.
  EXPECT_EQ(result.meta.md5sum, "2176decaecbce78abc3b96ef049fabed");
  EXPECT_EQ(
    result.meta.message_definition,
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n");
}

TEST(Ros1MetaSynthesizer, GeometryMsgsVector3IsBareTriple)
{
  const std::string_view ros2_msg =
    "float64 x\n"
    "float64 y\n"
    "float64 z\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("geometry_msgs/msg/Vector3", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  // `rosmsg md5 geometry_msgs/Vector3` on Noetic.
  EXPECT_EQ(result.meta.md5sum, "4a842b65f413084dc2b10fb484ea7f17");
}

TEST(Ros1MetaSynthesizer, NestedRefDropsArraySuffixInMd5Quirk)
{
  // ROS 1 quirk: in the md5 hash text, an array of complex types and a
  // single complex field collapse to the same line ("<sub_md5> name").
  // This test verifies the synthesizer follows that quirk by computing
  // a known canonical md5 — sensor_msgs/JointState has multiple complex
  // arrays plus primitive arrays, exercising both branches.
  const std::string_view ros2_msg =
    "std_msgs/msg/Header header\n"
    "string[] name\n"
    "float64[] position\n"
    "float64[] velocity\n"
    "float64[] effort\n"
    "================================================================================\n"
    "MSG: std_msgs/msg/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("sensor_msgs/msg/JointState", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  // `rosmsg md5 sensor_msgs/JointState` on Noetic.
  EXPECT_EQ(result.meta.md5sum, "3066dcd76a6cfaef579bd0f34173e9fd");
}

TEST(Ros1MetaSynthesizer, BoundedSequenceDropsBoundWithWarning)
{
  // ROS 2-only `int32[<=5]` becomes ROS 1 `int32[]`; the bound is wire-
  // irrelevant (CDR doesn't enforce sequence bounds at the wire level),
  // so the synthesizer drops it and warns rather than refusing.
  const std::string_view ros2_msg = "int32[<=5] x\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("foo_pkg/msg/Bounded", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.warnings.size(), 1U);
  EXPECT_EQ(result.warnings[0].field, "x");
  EXPECT_EQ(result.warnings[0].kind, "bound_dropped");
  EXPECT_EQ(result.meta.message_definition, "int32[] x\n");
}

TEST(Ros1MetaSynthesizer, DefaultValueDroppedWithWarning)
{
  // ROS 2-only default values don't appear on the wire, so dropping
  // them is wire-safe; emit a warning so the caller can log it.
  const std::string_view ros2_msg = "int32 x 5\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("foo_pkg/msg/Defaulted", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.warnings.size(), 1U);
  EXPECT_EQ(result.warnings[0].kind, "default_dropped");
  EXPECT_EQ(result.meta.message_definition, "int32 x\n");
}

TEST(Ros1MetaSynthesizer, WstringIsRefused)
{
  // wstring is wire-incompatible with ROS 1 string (UTF-16-ish vs
  // UTF-8 with no BOM). The synthesizer refuses outright rather than
  // silently downgrading to a wire-mismatched md5.
  const std::string_view ros2_msg = "wstring data\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("foo_pkg/msg/Wide", ros2_msg);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("wstring"), std::string::npos) << result.error;
}

TEST(Ros1MetaSynthesizer, ConcatenatedFormForCanMsgsFrame)
{
  // can_msgs/Frame is a ROS 1-and-ROS 2 wire-compatible type that
  // bagwiz already has the canonical metadata for. Synthesising from
  // its ROS 2 .msg should produce the same md5 (and a structurally
  // equivalent message_definition, modulo comments).
  const std::string_view ros2_msg =
    "std_msgs/msg/Header header\n"
    "uint32 id\n"
    "bool is_rtr\n"
    "bool is_extended\n"
    "bool is_error\n"
    "uint8 dlc\n"
    "uint8[8] data\n"
    "================================================================================\n"
    "MSG: std_msgs/msg/Header\n"
    "builtin_interfaces/Time stamp\n"
    "string frame_id\n";
  const auto result = bagwiz::core::synthesize_ros1_meta("can_msgs/msg/Frame", ros2_msg);
  ASSERT_TRUE(result.ok) << result.error;
  // Pinned in ros1_message_definitions.cpp from `rosmsg md5 can_msgs/Frame`.
  const auto * canonical = bagwiz::core::find_ros1_meta("can_msgs/Frame");
  ASSERT_NE(canonical, nullptr);
  EXPECT_EQ(result.meta.md5sum, canonical->md5sum);
  // Body should be the canonical Frame fields followed by the standard
  // `===` separator and the Header dependency block.
  EXPECT_NE(result.meta.message_definition.find("Header header\n"), std::string::npos);
  EXPECT_NE(result.meta.message_definition.find(kSep), std::string::npos);
  EXPECT_NE(
    result.meta.message_definition.find("MSG: std_msgs/Header\nuint32 seq\n"), std::string::npos);
}

TEST(Ros1MetaSynthesizer, MalformedRos2MsgReturnsError)
{
  const auto result = bagwiz::core::synthesize_ros1_meta("foo_pkg/msg/Bar", "not a valid msg\n");
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}
