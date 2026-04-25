// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_pattern.hpp"

#include <gtest/gtest.h>

namespace
{

using bagwiz::core::TopicPattern;

TEST(TopicPattern, EmptyMatchesEverything)
{
  TopicPattern p("");
  EXPECT_TRUE(p.match_all());
  EXPECT_TRUE(p.matches("/anything"));
  EXPECT_TRUE(p.matches(""));
}

TEST(TopicPattern, PlainTextIsSubstring)
{
  TopicPattern p("lidar");
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_TRUE(p.matches("lidar"));
  EXPECT_FALSE(p.matches("/perception/object"));
}

TEST(TopicPattern, LeadingSlashIsLiteralNotPrefix)
{
  // Without glob metacharacters, the leading '/' has no special meaning;
  // the pattern is matched as a substring.
  TopicPattern p("/sensing");
  EXPECT_TRUE(p.matches("/sensing"));
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_FALSE(p.matches("/perception/object"));
  // Substring semantics: '/sensing' inside a longer namespace is also kept.
  EXPECT_TRUE(p.matches("/foo/sensing/bar"));
}

TEST(TopicPattern, GlobAsteriskAnchored)
{
  // Glob is anchored at both ends.
  TopicPattern p("/sensing/*");
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_TRUE(p.matches("/sensing/imu"));
  EXPECT_FALSE(p.matches("/perception/object"));
  EXPECT_FALSE(p.matches("/foo/sensing/bar"));
}

TEST(TopicPattern, GlobAsteriskInTheMiddle)
{
  TopicPattern p("/sensing/*/points");
  EXPECT_TRUE(p.matches("/sensing/lidar/points"));
  // '*' may span multiple path segments, so deeper namespaces still match.
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_FALSE(p.matches("/sensing/lidar/front/imu"));
}

TEST(TopicPattern, GlobTrailingSuffix)
{
  TopicPattern p("*/points");
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_TRUE(p.matches("/a/points"));
  EXPECT_FALSE(p.matches("/sensing/lidar/points/raw"));
}

TEST(TopicPattern, GlobQuestionMark)
{
  TopicPattern p("/tf?");
  EXPECT_TRUE(p.matches("/tfa"));
  EXPECT_FALSE(p.matches("/tf"));
  EXPECT_FALSE(p.matches("/tf_static"));
}

TEST(TopicPattern, RegexMetacharactersAreLiteral)
{
  // '.' and other regex metacharacters are not special; only '*' and '?'
  // trigger glob mode.
  TopicPattern p("/tf.static");
  EXPECT_TRUE(p.matches("/tf.static"));
  EXPECT_TRUE(p.matches("/prefix/tf.static/suffix"));
  EXPECT_FALSE(p.matches("/tfXstatic"));
}

}  // namespace
