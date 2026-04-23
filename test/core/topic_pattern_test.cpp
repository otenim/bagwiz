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

TEST(TopicPattern, AbsolutePrefixMatch)
{
  TopicPattern p("/sensing");
  EXPECT_TRUE(p.matches("/sensing"));
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_FALSE(p.matches("/perception/object"));
  // A substring that is not at the start must not match.
  EXPECT_FALSE(p.matches("/foo/sensing"));
}

TEST(TopicPattern, RelativeSubstringMatch)
{
  TopicPattern p("lidar/front");
  EXPECT_TRUE(p.matches("/sensing/lidar/front/points"));
  EXPECT_TRUE(p.matches("lidar/front"));
  EXPECT_FALSE(p.matches("/sensing/lidar/rear/points"));
  EXPECT_FALSE(p.matches("/perception/object"));
}

TEST(TopicPattern, AsteriskIsLiteral)
{
  // '*' is no longer a wildcard; it only matches itself.
  TopicPattern p("/*/nebula_packets");
  EXPECT_FALSE(p.matches("/sensing/nebula_packets"));
  EXPECT_FALSE(p.matches("/foo/nebula_packets"));
  EXPECT_TRUE(p.matches("/*/nebula_packets"));
}

TEST(TopicPattern, RegexMetacharactersAreLiteral)
{
  // '.' and other regex metacharacters are treated as literal characters
  // because the matcher is a plain string compare.
  TopicPattern p("/tf.static");
  EXPECT_TRUE(p.matches("/tf.static"));
  EXPECT_FALSE(p.matches("/tfXstatic"));
}

}  // namespace
