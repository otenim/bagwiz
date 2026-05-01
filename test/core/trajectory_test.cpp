// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::TrajectoryPose;
using bagwiz::core::write_tum;

TEST(WriteTum, EmitsExpectedLineLayout)
{
  std::vector<TrajectoryPose> poses;
  // 1.5 s, identity rotation at (1, 2, 3).
  poses.push_back({1'500'000'000LL, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});
  // 2.75 s, a quarter turn about Z at (0, 0, 0).
  poses.push_back(
    {2'750'000'000LL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7071067811865475, 0.7071067811865475});

  std::ostringstream os;
  write_tum(os, poses);

  const std::string text = os.str();
  ASSERT_FALSE(text.empty());

  // Nanosecond-precision timestamp + 7 whitespace-separated values per line.
  EXPECT_NE(
    text.find(
      "1.500000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 "
      "0.000000000 1.000000000\n"),
    std::string::npos)
    << "got:\n"
    << text;
  EXPECT_NE(
    text.find(
      "2.750000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 "
      "0.707106781 0.707106781\n"),
    std::string::npos)
    << "got:\n"
    << text;
}

TEST(WriteTum, EmitsBitExactNanosecondsAtModernEpochs)
{
  // The double ULP near 1.77e18 (year-2026 magnitudes in ns) is ~256,
  // so a `static_cast<double>(ns) / 1e9` round trip silently drifts
  // the last few decimal digits. The formatter must format sec /
  // nanosec from the integer directly so the TUM timestamp is
  // bit-exact with the source header.stamp.
  const std::int64_t sec = 1773211197LL;
  const std::int64_t nsec = 937418279LL;
  const std::int64_t ts_ns = sec * 1'000'000'000LL + nsec;

  std::vector<TrajectoryPose> poses;
  poses.push_back({ts_ns, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  std::ostringstream os;
  write_tum(os, poses);
  const std::string text = os.str();

  EXPECT_NE(text.find("1773211197.937418279 "), std::string::npos) << "got:\n" << text;
}

TEST(WriteTum, EmitsNothingForEmptyTrajectory)
{
  std::ostringstream os;
  std::vector<TrajectoryPose> empty;
  write_tum(os, empty);
  EXPECT_EQ(os.str(), "");
}

TEST(WriteTum, RestoresStreamFormatting)
{
  std::ostringstream os;
  os << 0.1;  // default precision
  const std::string before = os.str();

  os.str({});
  os.clear();
  write_tum(os, std::vector<TrajectoryPose>{{1'000'000'000LL, 0, 0, 0, 0, 0, 0, 1}});

  // After writing, the default precision should be restored for the
  // caller so they do not silently inherit fixed/9-digit formatting.
  os.str({});
  os.clear();
  os << 0.1;
  EXPECT_EQ(os.str(), before);
}

}  // namespace
