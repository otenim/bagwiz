// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/topic_match.hpp"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::topic_glob_match;

TEST(TopicGlobMatch, ExactMatchWithoutWildcard)
{
  EXPECT_TRUE(topic_glob_match("/foo", "/foo"));
  EXPECT_FALSE(topic_glob_match("/foo", "/bar"));
  // No substring matching: an exact pattern must match the whole name.
  EXPECT_FALSE(topic_glob_match("/foo", "/foobar"));
  EXPECT_FALSE(topic_glob_match("/foobar", "/foo"));
}

TEST(TopicGlobMatch, LoneStarMatchesEverything)
{
  EXPECT_TRUE(topic_glob_match("*", "/foo"));
  EXPECT_TRUE(topic_glob_match("*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("*", ""));
}

TEST(TopicGlobMatch, PrefixStarMatchesAcrossSlashes)
{
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/lidar"));
  // The prefix before '*' must still match literally.
  EXPECT_FALSE(topic_glob_match("/sensing/*", "/perception/objects"));
  // '*' matches the empty string, so the bare prefix matches too.
  EXPECT_TRUE(topic_glob_match("/sensing/*", "/sensing/"));
}

TEST(TopicGlobMatch, SuffixStarMatchesTrailing)
{
  EXPECT_TRUE(topic_glob_match("*/image_raw", "/camera0/image_raw"));
  EXPECT_TRUE(topic_glob_match("*/image_raw", "/a/b/c/image_raw"));
  EXPECT_FALSE(topic_glob_match("*/image_raw", "/camera0/image_compressed"));
}

TEST(TopicGlobMatch, MiddleStarMatchesInfix)
{
  EXPECT_TRUE(topic_glob_match("/camera*/image", "/camera0/image"));
  EXPECT_TRUE(topic_glob_match("/camera*/image", "/camera/front/image"));
  EXPECT_FALSE(topic_glob_match("/camera*/image", "/lidar0/image"));
}

TEST(TopicGlobMatch, MultipleStars)
{
  EXPECT_TRUE(topic_glob_match("*camera*", "/sensing/camera/image"));
  EXPECT_TRUE(topic_glob_match("/*/*/image", "/sensing/cam/image"));
  EXPECT_FALSE(topic_glob_match("/*/*/image", "/sensing/image"));
}

TEST(TopicGlobMatch, ConsecutiveStarsCollapse)
{
  EXPECT_TRUE(topic_glob_match("/foo**", "/foobar"));
  EXPECT_TRUE(topic_glob_match("**", "/anything"));
}

TEST(TopicGlobMatch, EmptyPatternOnlyMatchesEmpty)
{
  EXPECT_TRUE(topic_glob_match("", ""));
  EXPECT_FALSE(topic_glob_match("", "/foo"));
}

}  // namespace

using bagwiz::core::resolve_topic_selectors;
using bagwiz::core::TopicEntry;

namespace
{
std::vector<TopicEntry> sample_topics()
{
  return {
    {"/sensing/lidar/right/points", "sensor_msgs/msg/PointCloud2"},
    {"/sensing/lidar/left/points", "sensor_msgs/msg/PointCloud2"},
    {"/sensing/camera/image_raw", "sensor_msgs/msg/Image"},
    {"/tf", "tf2_msgs/msg/TFMessage"},
  };
}

constexpr std::array<std::string_view, 1> kCloudType{{"sensor_msgs/msg/PointCloud2"}};
}  // namespace

TEST(ResolveTopicSelectors, LiteralPassesThroughUnchecked)
{
  // A literal is never matched, filtered, or validated here: the command's own
  // presence and type checks own that error, and their wording must not change.
  const std::vector<std::string> selectors{"/not/in/the/bag", "/tf"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_TRUE(result.unmatched.empty());
  EXPECT_EQ(result.matched, (std::vector<std::string>{"/not/in/the/bag", "/tf"}));
}

TEST(ResolveTopicSelectors, GlobIsFilteredByAllowedTypes)
{
  const std::vector<std::string> selectors{"*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_EQ(
    result.matched,
    (std::vector<std::string>{"/sensing/lidar/left/points", "/sensing/lidar/right/points"}));
}

TEST(ResolveTopicSelectors, EmptyAllowedTypesMatchesEveryType)
{
  const std::vector<std::string> selectors{"*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, {});

  EXPECT_EQ(result.matched.size(), 4U);
}

TEST(ResolveTopicSelectors, MatchesWithinOneSelectorAreSortedLexicographically)
{
  // The bag lists right before left; the result must not depend on that.
  const std::vector<std::string> selectors{"/sensing/lidar/*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_EQ(
    result.matched,
    (std::vector<std::string>{"/sensing/lidar/left/points", "/sensing/lidar/right/points"}));
}

TEST(ResolveTopicSelectors, SelectorsKeepArgumentOrder)
{
  const std::vector<std::string> selectors{"/sensing/lidar/right/*", "/sensing/lidar/left/*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_EQ(
    result.matched,
    (std::vector<std::string>{"/sensing/lidar/right/points", "/sensing/lidar/left/points"}));
}

TEST(ResolveTopicSelectors, DeduplicatesAcrossLiteralAndGlob)
{
  const std::vector<std::string> selectors{"/sensing/lidar/left/points", "/sensing/lidar/*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_EQ(
    result.matched,
    (std::vector<std::string>{"/sensing/lidar/left/points", "/sensing/lidar/right/points"}));
}

TEST(ResolveTopicSelectors, GlobMatchingNothingIsReportedUnmatched)
{
  const std::vector<std::string> selectors{"/nope/*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_TRUE(result.matched.empty());
  EXPECT_EQ(result.unmatched, (std::vector<std::string>{"/nope/*"}));
}

TEST(ResolveTopicSelectors, GlobMatchingOnlyWrongTypesIsUnmatched)
{
  // '*' does match /sensing/camera/image_raw by name, but the type filter
  // removes it, so the selector reports as matching nothing.
  const std::vector<std::string> selectors{"/sensing/camera/*"};
  const auto topics = sample_topics();

  const auto result = resolve_topic_selectors(selectors, topics, kCloudType);

  EXPECT_TRUE(result.matched.empty());
  EXPECT_EQ(result.unmatched, (std::vector<std::string>{"/sensing/camera/*"}));
}
