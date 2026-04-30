// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_message_definitions.hpp"
#include "bagwiz/core/ros1_meta_synthesizer.hpp"
#include "bagwiz/core/schema_resolver.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

// End-to-end pipeline tests: resolve_schema (AMENT path) → synthesize_ros1_meta
// → compare md5 against the canonical ROS 1 reference. The reference values
// are the ones pinned in src/core/ros1_message_definitions.cpp, themselves
// verified against `rosmsg md5 <type>` on ROS 1 Noetic.
//
// These exist to catch silent drift between the dynamic synthesis path
// (resolve_schema → synthesize_ros1_meta) and the static whitelist that
// the dynamic path replaced. If a normalisation rule regresses — Header.
// seq dropped from the md5 input, a `builtin_interfaces` alias not
// unwrapped to `time` / `duration`, a stray comment leaking in — the
// pipeline-derived md5 will diverge from the pinned reference and one
// of these cases fails.

namespace
{

using bagwiz::core::find_ros1_meta;
using bagwiz::core::resolve_schema;
using bagwiz::core::ResolveSchemaInput;
using bagwiz::core::SchemaSource;
using bagwiz::core::synthesize_ros1_meta;

const bagwiz::core::ResolvedSchemaCandidate * pick_candidate(
  const bagwiz::core::ResolveSchemaResult & r, SchemaSource source)
{
  for (const auto & c : r.candidates) {
    if (c.source == source && c.ok()) {
      return &c;
    }
  }
  return nullptr;
}

}  // namespace

// AMENT-resolved schema, synthesised md5, must equal the pinned ROS 1
// reference for every well-known type the project has metadata for.
// This is the most direct silent-corruption guard: if the synthesizer
// or resolver diverges from upstream ROS 1 even by one byte (a wrong
// Header.seq position, a stray comment leaking into the md5 input, a
// builtin_interfaces alias not unwrapped), one of these cases fails.
TEST(SchemaPipeline, AmentMd5MatchesRos1ReferenceForCanonicalTypes)
{
  // Each entry is a "ROS 2 type name" plus the ROS 1 short name used
  // for the find_ros1_meta() reference lookup. The two differ only by
  // the `/msg/` infix; we keep them explicit so the read at the
  // assertion site is unambiguous.
  struct Case
  {
    std::string ros2_type;
    std::string ros1_short_name;
  };
  const std::vector<Case> cases = {
    {"std_msgs/msg/Bool", "std_msgs/Bool"},
    {"std_msgs/msg/String", "std_msgs/String"},
    {"std_msgs/msg/Float32", "std_msgs/Float32"},
    {"std_msgs/msg/Float64", "std_msgs/Float64"},
    {"std_msgs/msg/Int32", "std_msgs/Int32"},
    {"std_msgs/msg/Int64", "std_msgs/Int64"},
    {"std_msgs/msg/UInt32", "std_msgs/UInt32"},
    {"std_msgs/msg/UInt64", "std_msgs/UInt64"},
    {"std_msgs/msg/Header", "std_msgs/Header"},
    {"geometry_msgs/msg/Vector3", "geometry_msgs/Vector3"},
    {"geometry_msgs/msg/Point", "geometry_msgs/Point"},
    {"geometry_msgs/msg/Quaternion", "geometry_msgs/Quaternion"},
    {"geometry_msgs/msg/Pose", "geometry_msgs/Pose"},
    {"geometry_msgs/msg/PoseStamped", "geometry_msgs/PoseStamped"},
    {"geometry_msgs/msg/Twist", "geometry_msgs/Twist"},
    {"geometry_msgs/msg/TwistStamped", "geometry_msgs/TwistStamped"},
    {"geometry_msgs/msg/Transform", "geometry_msgs/Transform"},
    {"geometry_msgs/msg/TransformStamped", "geometry_msgs/TransformStamped"},
    {"sensor_msgs/msg/Imu", "sensor_msgs/Imu"},
    {"sensor_msgs/msg/Image", "sensor_msgs/Image"},
    {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"},
    {"sensor_msgs/msg/PointField", "sensor_msgs/PointField"},
    {"sensor_msgs/msg/CompressedImage", "sensor_msgs/CompressedImage"},
    {"sensor_msgs/msg/LaserScan", "sensor_msgs/LaserScan"},
    {"nav_msgs/msg/Odometry", "nav_msgs/Odometry"},
    {"tf2_msgs/msg/TFMessage", "tf2_msgs/TFMessage"},
  };

  for (const auto & tc : cases) {
    SCOPED_TRACE(tc.ros2_type);

    ResolveSchemaInput in;
    in.ros2_type = tc.ros2_type;
    const auto resolved = resolve_schema(in);
    ASSERT_TRUE(resolved.ok) << "schema resolution failed for " << tc.ros2_type;

    const auto meta = synthesize_ros1_meta(tc.ros2_type, resolved.text);
    ASSERT_TRUE(meta.ok) << "synthesis failed: " << meta.error;

    const auto * canonical = find_ros1_meta(tc.ros1_short_name);
    ASSERT_NE(canonical, nullptr) << "no pinned reference for " << tc.ros1_short_name
                                  << " — add an entry to ros1_message_definitions.cpp";
    EXPECT_EQ(meta.meta.md5sum, canonical->md5sum)
      << "md5 drift detected for " << tc.ros2_type << " — synthesised=" << meta.meta.md5sum
      << " pinned=" << canonical->md5sum;
  }
}

// Crosscheck source: AMENT and introspection must produce the same
// md5 for types whose .msg has no constants (the introspection metadata
// can't recover constants — see schema_resolver.hpp). When a single
// bag-conversion run has both sources available, this is what the
// crosscheck reporting in convert.cpp would compare.
//
// Constant-bearing types (NavSatStatus, BatteryState, …) are
// deliberately excluded: their introspection-derived md5 *will* diverge
// from the AMENT-derived md5, and that divergence is the documented
// limitation of the introspection fallback (also called out in
// schema_resolver.hpp).
TEST(SchemaPipeline, AmentAndIntrospectionAgreeForConstantFreeTypes)
{
  // sensor_msgs/PointCloud2, sensor_msgs/NavSatFix, and nav_msgs/Path
  // are deliberately excluded because they reference PointField /
  // NavSatStatus which carry constants — the divergence there is
  // expected and documented (introspection metadata cannot recover
  // constants).
  const std::vector<std::string> types = {
    "std_msgs/msg/String",
    "std_msgs/msg/Header",
    "geometry_msgs/msg/Vector3",
    "geometry_msgs/msg/Quaternion",
    "geometry_msgs/msg/Pose",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/Twist",
    "geometry_msgs/msg/Transform",
    "geometry_msgs/msg/TransformStamped",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/Image",
    "nav_msgs/msg/Odometry",
    "tf2_msgs/msg/TFMessage",
  };

  for (const auto & type : types) {
    SCOPED_TRACE(type);

    ResolveSchemaInput in;
    in.ros2_type = type;
    const auto resolved = resolve_schema(in);
    ASSERT_TRUE(resolved.ok);

    const auto * ament = pick_candidate(resolved, SchemaSource::AmentInstall);
    const auto * intro = pick_candidate(resolved, SchemaSource::Introspection);
    ASSERT_NE(ament, nullptr) << "AMENT path failed for " << type;
    ASSERT_NE(intro, nullptr) << "introspection path failed for " << type;

    const auto ament_meta = synthesize_ros1_meta(type, ament->text);
    const auto intro_meta = synthesize_ros1_meta(type, intro->text);
    ASSERT_TRUE(ament_meta.ok) << "AMENT synthesis: " << ament_meta.error;
    ASSERT_TRUE(intro_meta.ok) << "introspection synthesis: " << intro_meta.error;

    EXPECT_EQ(ament_meta.meta.md5sum, intro_meta.meta.md5sum)
      << type << ": AMENT and introspection sources disagree on md5 — "
      << "either the introspection emitter regressed or " << type
      << " gained a constant (in which case move it out of this list).";
  }
}

// Bag-embedded text takes priority and goes through the same synthesizer
// pipeline. This is the 2to1 happy path — a self-describing rosbag2 mcap
// supplies the schema text, the synthesizer derives the ROS 1 metadata
// from it, and the resulting md5 must match the canonical reference.
TEST(SchemaPipeline, BagEmbeddedTextProducesSameMd5AsAment)
{
  // First resolve via AMENT to get a schema text we know is valid; then
  // feed that same text back as `bag_embedded_text` and confirm the
  // resolver returns it (priority) and the synthesizer's md5 matches.
  ResolveSchemaInput ament_in;
  ament_in.ros2_type = "sensor_msgs/msg/Imu";
  const auto ament_resolved = resolve_schema(ament_in);
  ASSERT_TRUE(ament_resolved.ok);
  ASSERT_EQ(ament_resolved.source, SchemaSource::AmentInstall);

  ResolveSchemaInput bag_in;
  bag_in.ros2_type = "sensor_msgs/msg/Imu";
  bag_in.bag_embedded_text = ament_resolved.text;
  bag_in.bag_embedded_encoding = "ros2msg";
  const auto bag_resolved = resolve_schema(bag_in);
  ASSERT_TRUE(bag_resolved.ok);
  EXPECT_EQ(bag_resolved.source, SchemaSource::BagEmbedded);

  const auto ament_meta = synthesize_ros1_meta("sensor_msgs/msg/Imu", ament_resolved.text);
  const auto bag_meta = synthesize_ros1_meta("sensor_msgs/msg/Imu", bag_resolved.text);
  ASSERT_TRUE(ament_meta.ok);
  ASSERT_TRUE(bag_meta.ok);
  EXPECT_EQ(ament_meta.meta.md5sum, bag_meta.meta.md5sum);

  const auto * canonical = find_ros1_meta("sensor_msgs/Imu");
  ASSERT_NE(canonical, nullptr);
  EXPECT_EQ(bag_meta.meta.md5sum, canonical->md5sum);
}

// Negative case: a wstring field aborts synthesis at the boundary, even
// when the schema itself resolves cleanly (the resolver doesn't inspect
// field types — only the synthesizer does). wstring is the one ROS 2
// construct that has no wire-equivalent ROS 1 representation, so the
// synthesizer refuses outright rather than emitting a misleading md5.
TEST(SchemaPipeline, WstringFieldRefusedAtSynthesisBoundary)
{
  // Construct a schema text by hand — there's no installed ROS 2 type
  // with `wstring` in our test corpus.
  const std::string ros2_msg = "wstring data\n";
  const auto meta = synthesize_ros1_meta("foo_pkg/msg/Wide", ros2_msg);
  EXPECT_FALSE(meta.ok);
  EXPECT_NE(meta.error.find("wstring"), std::string::npos) << meta.error;
}

// Negative case: an unknown type fails every resolver source. The
// resolver's candidates list still describes what was tried, which is
// what convert.cpp surfaces in the SkippedTopic detail string.
TEST(SchemaPipeline, UnknownTypeReportsErrorOnEverySource)
{
  ResolveSchemaInput in;
  in.ros2_type = "nonexistent_pkg_qwxyz/msg/Type";
  const auto resolved = resolve_schema(in);
  EXPECT_FALSE(resolved.ok);
  EXPECT_FALSE(resolved.candidates.empty());
  for (const auto & c : resolved.candidates) {
    EXPECT_FALSE(c.error.empty()) << "every failed candidate must carry a reason";
  }
}
