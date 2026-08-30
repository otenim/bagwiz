// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_track.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/io/bag_io.hpp"
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::kMapHeadingMinMoveM;
using bagwiz::commands::load_map_track;
using bagwiz::commands::map_heading_at;
using bagwiz::commands::MapFix;
using bagwiz::commands::MapTrack;
using bagwiz::commands::nearest_map_fix;
using bagwiz::test::kMovifyNavSatFixType;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::movify_write_gnss_bag;
using bagwiz::test::MovifyGnssFix;
using bagwiz::test::MovifyTmpDirTest;

constexpr std::int8_t kFix = 0;
constexpr std::int8_t kNoFix = -1;
constexpr double kLat = 35.0;
constexpr double kLon = 139.0;
// One 1e-4 degree step is ~11.1 m of latitude, and ~9.1 m of longitude at 35 N.
constexpr double kStepDeg = 1e-4;
constexpr double kNorthPerStepM = 11.1;
constexpr double kEastPerStepM = 9.1;
constexpr double kMetersTolerance = 0.2;
constexpr std::int64_t kNs = 1'000'000'000LL;

MapTrack track_of(const std::vector<MapFix> & fixes)
{
  MapTrack track;
  track.fixes = fixes;
  return track;
}

MapFix at(double east, double north, std::int64_t record_ns = 0)
{
  MapFix fix;
  fix.east = east;
  fix.north = north;
  fix.record_ns = record_ns;
  return fix;
}

TEST_F(MovifyTmpDirTest, LoadsTheFixesInBagOrderProjectedToEnu)
{
  const std::vector<MovifyGnssFix> fixes{
    {kNs, kFix, kLat, kLon, 10.0},
    {kNs + 100'000'000LL, kFix, kLat + kStepDeg, kLon, 10.0},
    {kNs + 200'000'000LL, kFix, kLat + kStepDeg, kLon + kStepDeg, 12.0},
  };
  const auto bag = movify_write_gnss_bag(tmp_dir_, "in.mcap", "/gnss", fixes);

  const auto loaded = load_map_track(bag, "/gnss");
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  const auto & track = *loaded.track;
  ASSERT_EQ(track.fixes.size(), 3u);
  // The first fix anchors the plane.
  EXPECT_NEAR(track.fixes[0].east, 0.0, 1e-6);
  EXPECT_NEAR(track.fixes[0].north, 0.0, 1e-6);
  EXPECT_EQ(track.fixes[0].record_ns, kNs);
  EXPECT_EQ(track.fixes[0].stamp_ns, kNs);
  EXPECT_DOUBLE_EQ(track.fixes[0].latitude, kLat);
  EXPECT_DOUBLE_EQ(track.fixes[0].longitude, kLon);
  EXPECT_DOUBLE_EQ(track.fixes[0].altitude, 10.0);
  // A step of latitude is north, a step of longitude is east.
  EXPECT_NEAR(track.fixes[1].north, kNorthPerStepM, kMetersTolerance);
  EXPECT_NEAR(track.fixes[1].east, 0.0, kMetersTolerance);
  EXPECT_NEAR(track.fixes[2].east, kEastPerStepM, kMetersTolerance);
  EXPECT_NEAR(track.fixes[2].north, kNorthPerStepM, kMetersTolerance);
  EXPECT_EQ(track.fixes[2].record_ns, kNs + 200'000'000LL);
  // The bounds span the fixes.
  EXPECT_NEAR(track.min_east, 0.0, 1e-6);
  EXPECT_NEAR(track.max_east, kEastPerStepM, kMetersTolerance);
  EXPECT_NEAR(track.min_north, 0.0, 1e-6);
  EXPECT_NEAR(track.max_north, kNorthPerStepM, kMetersTolerance);
}

TEST_F(MovifyTmpDirTest, DropsTheFixesWithoutAPosition)
{
  const std::vector<MovifyGnssFix> fixes{
    {kNs, kNoFix, 0.0, 0.0, 0.0},  // no fix yet: skipped, so the next one anchors
    {kNs + 100'000'000LL, kFix, kLat, kLon, 10.0},
    {kNs + 200'000'000LL, kNoFix, kLat, kLon, 10.0},
    {kNs + 300'000'000LL, kFix, std::numeric_limits<double>::quiet_NaN(), kLon, 10.0},
    {kNs + 400'000'000LL, kFix, kLat + kStepDeg, kLon, 10.0},
  };
  const auto bag = movify_write_gnss_bag(tmp_dir_, "in.mcap", "/gnss", fixes);

  const auto loaded = load_map_track(bag, "/gnss");
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  ASSERT_EQ(loaded.track->fixes.size(), 2u);
  EXPECT_EQ(loaded.track->fixes[0].record_ns, kNs + 100'000'000LL);
  EXPECT_NEAR(loaded.track->fixes[0].north, 0.0, 1e-6);
  EXPECT_EQ(loaded.track->fixes[1].record_ns, kNs + 400'000'000LL);
  EXPECT_NEAR(loaded.track->fixes[1].north, kNorthPerStepM, kMetersTolerance);
}

TEST_F(MovifyTmpDirTest, RejectsATopicWithoutMessages)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/gnss", kMovifyNavSatFixType);
    w->close();
  }
  const auto loaded = load_map_track(bag, "/gnss");
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.error, "topic '/gnss' has no messages to render.");
}

TEST_F(MovifyTmpDirTest, RejectsATrackWithoutAnyPosition)
{
  const std::vector<MovifyGnssFix> fixes{
    {kNs, kNoFix, 0.0, 0.0, 0.0},
    {kNs + 100'000'000LL, kNoFix, 0.0, 0.0, 0.0},
  };
  const auto bag = movify_write_gnss_bag(tmp_dir_, "in.mcap", "/gnss", fixes);
  const auto loaded = load_map_track(bag, "/gnss");
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(
    loaded.error, "topic '/gnss' carries no fix with a position (every message is NO_FIX).");
}

TEST_F(MovifyTmpDirTest, RejectsAMalformedMessage)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/gnss", kMovifyNavSatFixType);
    w->write("/gnss", kNs, bagwiz::test::kMovifyGarbagePayload);
    w->close();
  }
  const auto loaded = load_map_track(bag, "/gnss");
  EXPECT_FALSE(loaded.ok());
  EXPECT_NE(
    loaded.error.find("topic '/gnss': failed to parse the NavSatFix message at record time"),
    std::string::npos)
    << loaded.error;
}

TEST_F(MovifyTmpDirTest, RejectsAnUnopenableBag)
{
  const auto loaded = load_map_track(tmp_dir_ / "missing.mcap", "/gnss");
  EXPECT_FALSE(loaded.ok());
  EXPECT_NE(loaded.error.find("failed to open"), std::string::npos) << loaded.error;
}

TEST(MapTrackLookup, NearestFixByRecordTimeTiesToTheEarlier)
{
  const auto track = track_of({at(0, 0, 100), at(1, 0, 200), at(2, 0, 300)});
  EXPECT_EQ(nearest_map_fix(track, 50), 0u);
  EXPECT_EQ(nearest_map_fix(track, 100), 0u);
  EXPECT_EQ(nearest_map_fix(track, 149), 0u);
  EXPECT_EQ(nearest_map_fix(track, 150), 0u);  // tie: the earlier fix
  EXPECT_EQ(nearest_map_fix(track, 151), 1u);
  EXPECT_EQ(nearest_map_fix(track, 250), 1u);
  EXPECT_EQ(nearest_map_fix(track, 260), 2u);
  EXPECT_EQ(nearest_map_fix(track, 1000), 2u);
}

TEST(MapTrackLookup, NearestFixOfASingleFixTrack)
{
  const auto track = track_of({at(0, 0, 100)});
  EXPECT_EQ(nearest_map_fix(track, 0), 0u);
  EXPECT_EQ(nearest_map_fix(track, 500), 0u);
}

TEST(MapTrackLookup, HeadingComesFromTheLastHalfMeterOfMovement)
{
  // A short jitter (0.1 m) is not a heading; the first real move is east,
  // the next north.
  const auto track = track_of({at(0, 0), at(0.1, 0), at(1, 0), at(1, 1)});
  EXPECT_FALSE(map_heading_at(track, 0).has_value());
  EXPECT_FALSE(map_heading_at(track, 1).has_value());
  ASSERT_TRUE(map_heading_at(track, 2).has_value());
  EXPECT_NEAR(*map_heading_at(track, 2), 0.0, 1e-9);
  ASSERT_TRUE(map_heading_at(track, 3).has_value());
  EXPECT_NEAR(*map_heading_at(track, 3), std::atan2(1.0, 0.0), 1e-9);
  static_assert(kMapHeadingMinMoveM > 0.1 && kMapHeadingMinMoveM <= 1.0);
}

TEST(MapTrackLookup, HeadingOfAnOutOfRangeIndexIsEmpty)
{
  const auto track = track_of({at(0, 0), at(5, 0)});
  EXPECT_FALSE(map_heading_at(track, 2).has_value());
}

}  // namespace
