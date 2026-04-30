// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/schema_resolver.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <string_view>

namespace
{

using bagwiz::core::resolve_schema;
using bagwiz::core::ResolveSchemaInput;
using bagwiz::core::SchemaSource;

// Pull every candidate matching `source` out of a result; convenient
// shorthand for tests that only care about a particular path.
const bagwiz::core::ResolvedSchemaCandidate * find_candidate(
  const bagwiz::core::ResolveSchemaResult & r, SchemaSource source)
{
  for (const auto & c : r.candidates) {
    if (c.source == source) {
      return &c;
    }
  }
  return nullptr;
}

// Count occurrences of `needle` in `hay`. Used to verify that the
// resolver emits each dep exactly once even when multiple parent fields
// reference the same nested type.
std::size_t count_occurrences(const std::string & hay, std::string_view needle)
{
  std::size_t n = 0;
  std::size_t pos = 0;
  while ((pos = hay.find(needle, pos)) != std::string::npos) {
    ++n;
    pos += needle.size();
  }
  return n;
}

}  // namespace

// Bag-embedded text wins outright when it is non-empty and the encoding
// matches "ros2msg" — even if the type also happens to be installed via
// AMENT. The producer-shipped schema is the only source guaranteed to
// match what was actually serialised, so it must beat local install
// state.
TEST(SchemaResolver, BagEmbeddedWinsOverAmentWhenAvailable)
{
  ResolveSchemaInput in;
  in.ros2_type = "std_msgs/msg/String";
  in.bag_embedded_text = "string data\n";
  in.bag_embedded_encoding = "ros2msg";

  const auto r = resolve_schema(in);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.source, SchemaSource::BagEmbedded);
  EXPECT_EQ(r.text, "string data\n");
  EXPECT_EQ(r.encoding, "ros2msg");
}

// Empty bag-embedded text falls through to the AMENT path. This is the
// common case for bags older than Iron / not produced by rosbag2.
TEST(SchemaResolver, FallsBackToAmentWhenBagEmbeddedMissing)
{
  ResolveSchemaInput in;
  in.ros2_type = "std_msgs/msg/Header";
  in.bag_embedded_text = "";
  in.bag_embedded_encoding = "";

  const auto r = resolve_schema(in);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.source, SchemaSource::AmentInstall);
  EXPECT_EQ(r.encoding, "ros2msg");
  // AMENT-resolved Header text references builtin_interfaces/Time as a
  // dep, with the rosbag2-style separator.
  EXPECT_NE(r.text.find("builtin_interfaces/Time stamp"), std::string::npos);
  EXPECT_NE(r.text.find("MSG: builtin_interfaces/Time"), std::string::npos);
}

// A bag-embedded schema with a non-"ros2msg" encoding (e.g. "ros1msg"
// from a 1to2 conversion in a transitional bag) must not be honoured —
// the resolver falls through to the next source. This guards against
// silently passing a ROS 1-shaped definition into the ROS 2 pipeline.
TEST(SchemaResolver, RejectsBagEmbeddedWithWrongEncoding)
{
  ResolveSchemaInput in;
  in.ros2_type = "std_msgs/msg/String";
  in.bag_embedded_text = "string data\n";
  in.bag_embedded_encoding = "ros1msg";

  const auto r = resolve_schema(in);
  ASSERT_TRUE(r.ok);
  EXPECT_NE(r.source, SchemaSource::BagEmbedded);

  const auto * bag = find_candidate(r, SchemaSource::BagEmbedded);
  ASSERT_NE(bag, nullptr);
  EXPECT_FALSE(bag->ok());
  EXPECT_NE(bag->error.find("ros2msg"), std::string::npos)
    << "bag-embedded rejection should mention the expected encoding";
}

// Empty type names short-circuit before any source is tried; the result
// is unambiguously "not resolved" with an empty candidate list rather
// than three useless failures.
TEST(SchemaResolver, EmptyTypeNameReturnsEmptyResult)
{
  ResolveSchemaInput in;
  in.ros2_type = "";

  const auto r = resolve_schema(in);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(r.text.empty());
  EXPECT_TRUE(r.encoding.empty());
  EXPECT_TRUE(r.candidates.empty());
}

// Every source is recorded in `candidates`, even on success, so callers
// can drive multi-source MD5 crosscheck reporting without re-running
// the pipeline. Order matches priority: bag-embedded, AMENT,
// introspection.
TEST(SchemaResolver, RecordsAllCandidatesForCrosscheck)
{
  ResolveSchemaInput in;
  in.ros2_type = "std_msgs/msg/Header";
  in.bag_embedded_text = "";
  in.bag_embedded_encoding = "";

  const auto r = resolve_schema(in);
  ASSERT_EQ(r.candidates.size(), 3U);
  EXPECT_EQ(r.candidates[0].source, SchemaSource::BagEmbedded);
  EXPECT_EQ(r.candidates[1].source, SchemaSource::AmentInstall);
  EXPECT_EQ(r.candidates[2].source, SchemaSource::Introspection);

  // Bag-embedded should fail (no input), AMENT should succeed,
  // introspection should also succeed (std_msgs is installed and ships
  // an introspection .so).
  EXPECT_FALSE(r.candidates[0].ok());
  EXPECT_TRUE(r.candidates[1].ok());
  EXPECT_TRUE(r.candidates[2].ok());
}

// Unknown types fail every path. The result is `ok=false` with no usable
// text, but the candidate list still describes which sources were tried
// — this is the diagnostic surface for "I sourced the wrong distro" /
// "this package isn't installed" errors.
TEST(SchemaResolver, AllSourcesFailForUnknownPackage)
{
  ResolveSchemaInput in;
  in.ros2_type = "no_such_pkg_qwxyz/msg/Type";

  const auto r = resolve_schema(in);
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(r.text.empty());
  ASSERT_EQ(r.candidates.size(), 3U);
  for (const auto & c : r.candidates) {
    EXPECT_FALSE(c.ok());
    EXPECT_FALSE(c.error.empty()) << "every failed candidate must carry a reason";
  }
}

// Same-package shorthand handling on the introspection path. Even
// without bag-embedded or AMENT (we only inspect the introspection
// candidate here), a sensor_msgs/Imu introspection synthesis must
// reference geometry_msgs/Vector3 and Quaternion, std_msgs/Header,
// builtin_interfaces/Time, all transitively.
TEST(SchemaResolver, IntrospectionSynthesisIncludesAllTransitiveDeps)
{
  ResolveSchemaInput in;
  in.ros2_type = "sensor_msgs/msg/Imu";

  const auto r = resolve_schema(in);
  ASSERT_TRUE(r.ok);

  const auto * intro = find_candidate(r, SchemaSource::Introspection);
  ASSERT_NE(intro, nullptr);
  ASSERT_TRUE(intro->ok());
  EXPECT_EQ(intro->encoding, "ros2msg");

  // Top-level body uses Header / Quaternion / Vector3 (twice — angular
  // velocity + linear acceleration) plus three covariance arrays.
  EXPECT_NE(intro->text.find("std_msgs/Header header"), std::string::npos);
  EXPECT_NE(intro->text.find("geometry_msgs/Quaternion orientation"), std::string::npos);
  EXPECT_NE(intro->text.find("geometry_msgs/Vector3 angular_velocity"), std::string::npos);
  EXPECT_NE(intro->text.find("geometry_msgs/Vector3 linear_acceleration"), std::string::npos);
  EXPECT_NE(intro->text.find("float64[9]"), std::string::npos)
    << "fixed-size 3x3 covariance array must be rendered as float64[9]";

  // Each dep is emitted exactly once even if referenced multiple times.
  EXPECT_EQ(count_occurrences(intro->text, "MSG: std_msgs/Header"), 1U);
  EXPECT_EQ(count_occurrences(intro->text, "MSG: geometry_msgs/Vector3"), 1U);
  EXPECT_EQ(count_occurrences(intro->text, "MSG: geometry_msgs/Quaternion"), 1U);
  EXPECT_EQ(count_occurrences(intro->text, "MSG: builtin_interfaces/Time"), 1U);
}

// to_string() is the label used in CLI summaries / crosscheck output.
// The labels are part of the user-facing API and should not change
// silently.
TEST(SchemaResolver, SourceToStringLabels)
{
  EXPECT_EQ(bagwiz::core::to_string(SchemaSource::BagEmbedded), "bag-embedded");
  EXPECT_EQ(bagwiz::core::to_string(SchemaSource::AmentInstall), "ament");
  EXPECT_EQ(bagwiz::core::to_string(SchemaSource::Introspection), "introspection");
}

// The legacy short ("pkg/Type") form must work everywhere the canonical
// form does — bag readers and existing internal callers may pass either.
TEST(SchemaResolver, AcceptsLegacyShortFormInput)
{
  ResolveSchemaInput in_short;
  in_short.ros2_type = "std_msgs/Header";
  ResolveSchemaInput in_canonical;
  in_canonical.ros2_type = "std_msgs/msg/Header";

  const auto r_short = resolve_schema(in_short);
  const auto r_canonical = resolve_schema(in_canonical);
  ASSERT_TRUE(r_short.ok);
  ASSERT_TRUE(r_canonical.ok);
  EXPECT_EQ(r_short.source, r_canonical.source);
  EXPECT_EQ(r_short.text, r_canonical.text);
}
