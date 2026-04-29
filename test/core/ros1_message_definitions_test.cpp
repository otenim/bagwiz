// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_message_definitions.hpp"

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

namespace
{

// Authoritative md5 values, captured by running `rosmsg md5 <type>` inside
// a Noetic container with these apt packages installed:
//   ros-noetic-ros-base (provides std_msgs / geometry_msgs / sensor_msgs /
//                        nav_msgs / diagnostic_msgs / tf2_msgs)
//   ros-noetic-tf       (provides the legacy tf/tfMessage alias)
//   ros-noetic-can-msgs (provides can_msgs/Frame from ros-industrial)
//
// These are the values every ROS 1 publisher/subscriber/bag reader will
// expect on the wire. If the converter writes a different md5 into a bag's
// CONNECTION record, ROS 1 consumers reject the message at read time with
// a "datatype/md5sum mismatch" error. Pinning them as a test guards both
// against transcription typos in the hardcoded table and against
// accidental edits to the .msg text snippets that would change the hash.
//
// To regenerate this list (e.g. after adding a new type):
//   1. Add the type to bagwiz's table in src/core/ros1_message_definitions.cpp
//   2. In a Noetic container, run `rosmsg md5 pkg/Type` and paste below
//   3. Re-run the test; it should pass
const std::vector<std::pair<std::string, std::string>> & expected_md5s()
{
  static const std::vector<std::pair<std::string, std::string>> kEntries = {
    {"std_msgs/Bool", "8b94c1b53db61fb6aed406028ad6332a"},
    {"std_msgs/Header", "2176decaecbce78abc3b96ef049fabed"},
    {"std_msgs/String", "992ce8a1687cec8c8bd883ec73ca41d1"},
    {"std_msgs/Float32", "73fcbf46b49191e672908e50842a83d4"},
    {"std_msgs/Float64", "fdb28210bfa9d7c91146260178d9a584"},
    {"std_msgs/Int32", "da5909fbe378aeaf85e547e830cc1bb7"},
    {"std_msgs/Int64", "34add168574510e6e17f5d23ecc077ef"},
    {"std_msgs/UInt32", "304a39449588c7f8ce2df6e8001c5fce"},
    {"std_msgs/UInt64", "1b2a79973e8bf53d7b53acb71299cb57"},

    {"geometry_msgs/Vector3", "4a842b65f413084dc2b10fb484ea7f17"},
    // Point and Vector3 reduce to the same 3-float64 schema, so they
    // collide on md5 by upstream design.
    {"geometry_msgs/Point", "4a842b65f413084dc2b10fb484ea7f17"},
    {"geometry_msgs/Quaternion", "a779879fadf0160734f906b8c19c7004"},
    {"geometry_msgs/Pose", "e45d45a5a1ce597b249e23fb30fc871f"},
    {"geometry_msgs/Transform", "ac9eff44abf714214112b05d54a3cf9b"},
    // Twist and Accel similarly reduce to two Vector3s.
    {"geometry_msgs/Twist", "9f195f881246fdfa2798d1d3eebca84a"},
    {"geometry_msgs/Accel", "9f195f881246fdfa2798d1d3eebca84a"},

    {"geometry_msgs/Vector3Stamped", "7b324c7325e683bf02a9b14b01090ec7"},
    {"geometry_msgs/PointStamped", "c63aecb41bfdfd6b7e1fac37c7cbe7bf"},
    {"geometry_msgs/QuaternionStamped", "e57f1e547e0e1fd13504588ffc8334e2"},
    {"geometry_msgs/PoseStamped", "d3812c3cbc69362b77dc0b19b345f8f5"},
    {"geometry_msgs/TransformStamped", "b5764a33bfeb3588febc2682852579b0"},
    {"geometry_msgs/TwistStamped", "98d34b0043a2093cf9d9345ab6eef12e"},
    {"geometry_msgs/AccelStamped", "d8a98a5d81351b6eb0578c78557e7659"},

    {"geometry_msgs/PoseWithCovariance", "c23e848cf1b7533a8d7c259073a97e6f"},
    {"geometry_msgs/PoseWithCovarianceStamped", "953b798c0f514ff060a53a3498ce6246"},
    {"geometry_msgs/TwistWithCovariance", "1fe8a28e6890a4cc3ae4c3ca5c7d82e6"},
    {"geometry_msgs/TwistWithCovarianceStamped", "8927a1a12fb2607ceea095b2dc440a96"},

    // tf2_msgs/TFMessage and the legacy tf/tfMessage alias share an md5
    // because they wrap the same TransformStamped[] schema.
    {"tf2_msgs/TFMessage", "94810edda583a504dfda3829e70d7eec"},
    {"tf/tfMessage", "94810edda583a504dfda3829e70d7eec"},

    {"nav_msgs/Odometry", "cd5e73d190d741a2f92e81eda573aca7"},
    {"nav_msgs/Path", "6227e2b7e9cce15051f669a5e197bbf7"},

    {"sensor_msgs/Imu", "6a62c6daae103f4ff57a132d6f95cec2"},
    {"sensor_msgs/Image", "060021388200f6f0f447d0fcd9c64743"},
    {"sensor_msgs/CompressedImage", "8f7a12909da2c9d3332d540a0977563f"},
    {"sensor_msgs/CameraInfo", "c9a58c1b0b154e0e6da7578cb991d214"},
    {"sensor_msgs/PointCloud2", "1158d486dd51d683ce2f1be655c3c181"},
    {"sensor_msgs/PointField", "268eacb2962780ceac86cbd17e328150"},
    {"sensor_msgs/NavSatFix", "2d3a8cd499b9b4a0249fb98fd05cfa48"},
    {"sensor_msgs/NavSatStatus", "331cdbddfa4bc96ffc3b9ad98900a54c"},
    {"sensor_msgs/LaserScan", "90c7ef2dc6895d81024acba2ac42f369"},
    {"sensor_msgs/Range", "c005c34273dc426c67a020a87bc24148"},
    {"sensor_msgs/Temperature", "ff71b307acdbe7c871a5a6d7ed359100"},
    {"sensor_msgs/FluidPressure", "804dc5cea1c5306d6a2eb80b9833befe"},
    {"sensor_msgs/MagneticField", "2f3b0b43eed0c9501de0fa3ff89a45aa"},

    {"diagnostic_msgs/KeyValue", "cf57fdc6617a881a88c16e768132149c"},
    {"diagnostic_msgs/DiagnosticStatus", "d0ce08bc6e5ba34c7754f563a9cabaf1"},
    {"diagnostic_msgs/DiagnosticArray", "60810da900de1dd6ddd437c3503511da"},

    {"can_msgs/Frame", "64ae5cebf967dc6aae4e78f5683a5b25"},
  };
  return kEntries;
}

}  // namespace

TEST(Ros1MessageDefinitions, EveryTypeMd5MatchesNoeticUpstream)
{
  // Loop with an explicit per-type assertion so a single mismatch
  // produces a focused error message identifying which type drifted.
  for (const auto & [type, expected] : expected_md5s()) {
    SCOPED_TRACE(type);
    const auto * meta = bagwiz::core::find_ros1_meta(type);
    ASSERT_NE(meta, nullptr) << "type missing from ros1_message_definitions table: " << type;
    EXPECT_EQ(meta->md5sum, expected)
      << type << " md5 differs from upstream Noetic value (rosmsg md5 " << type << ")";
  }
}

TEST(Ros1MessageDefinitions, EveryTypeHasNonEmptyMessageDefinition)
{
  // Catch the degenerate case where someone adds a type but forgets to
  // populate message_definition; ROS 1 readers need this string to
  // reconstruct the schema when the type isn't installed locally.
  for (const auto & [type, _md5] : expected_md5s()) {
    SCOPED_TRACE(type);
    const auto * meta = bagwiz::core::find_ros1_meta(type);
    ASSERT_NE(meta, nullptr);
    EXPECT_FALSE(meta->message_definition.empty()) << type << " has empty message_definition";
  }
}

TEST(Ros1MessageDefinitions, UnknownTypeReturnsNull)
{
  EXPECT_EQ(bagwiz::core::find_ros1_meta("foo_pkg/Bar"), nullptr);
  EXPECT_EQ(bagwiz::core::find_ros1_meta(""), nullptr);
}
