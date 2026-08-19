// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "tf_static_calibrate_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <vector>

namespace commands = bagwiz::commands;

namespace
{
commands::TfStaticCalibrateArgs valid_args()
{
  commands::TfStaticCalibrateArgs args;
  args.input_path = "in.db3";
  args.map_path = "map.pcd";
  args.traj_path = "traj.tum";
  args.traj_frame = "base_link";
  args.topic = "/cam/image_raw/compressed";
  args.parent_frame = "cabin";
  args.child_frame = "cam_link";
  return args;
}
}  // namespace

TEST(TfStaticCalibrateCommonTest, ValidArgsPass)
{
  EXPECT_EQ(commands::validate_calibrate_flags(valid_args()), "");
}

TEST(TfStaticCalibrateCommonTest, RejectsTooFewSamplesAndBadDepthWindow)
{
  auto args = valid_args();
  args.samples = 2;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
  args = valid_args();
  args.min_depth = 10.0;
  args.max_depth = 5.0;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
}

TEST(TfStaticCalibrateCommonTest, ParseFixedAxes)
{
  const auto [flags, err] = commands::parse_fixed_axes("x,yaw");
  EXPECT_EQ(err, "");
  EXPECT_TRUE(flags[0]);
  EXPECT_FALSE(flags[1]);
  EXPECT_TRUE(flags[5]);
  EXPECT_NE(commands::parse_fixed_axes("x,bogus").second, "");
  EXPECT_NE(commands::parse_fixed_axes("x,y,z,roll,pitch,yaw").second, "");
  EXPECT_EQ(commands::parse_fixed_axes("").second, "");
}

TEST(TfStaticCalibrateCommonTest, PickSampleIndicesRespectsMarginAndSpread)
{
  std::vector<std::int64_t> stamps;
  for (int i = 0; i < 100; ++i) {
    stamps.push_back(static_cast<std::int64_t>(i) * 1'000'000'000);  // 1 Hz, 0..99 s
  }
  const auto picks = commands::pick_sample_indices(stamps, 0, 99'000'000'000, 8, 3'000'000'000);
  ASSERT_EQ(picks.size(), 8U);
  EXPECT_GE(stamps[picks.front()], 3'000'000'000);
  EXPECT_LE(stamps[picks.back()], 96'000'000'000);
  for (std::size_t i = 1; i < picks.size(); ++i) {
    EXPECT_GT(picks[i], picks[i - 1]);
  }
}

TEST(TfStaticCalibrateCommonTest, InterpolateTrajectoryLerpsBetweenPoses)
{
  std::vector<bagwiz::core::TrajectoryPose> poses(2);
  poses[0].timestamp_ns = 0;
  poses[0].tx = 0.0;
  poses[0].qw = 1.0;
  poses[1].timestamp_ns = 2'000'000'000;
  poses[1].tx = 4.0;
  poses[1].qw = 1.0;
  const auto t = commands::interpolate_trajectory(poses, 1'000'000'000);
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR((*t)[12], 2.0, 1e-9);
  EXPECT_FALSE(commands::interpolate_trajectory(poses, 3'000'000'000).has_value());
}

TEST(TfStaticCalibrateCommonTest, DefaultOutputPathUsesInputStem)
{
  EXPECT_EQ(
    commands::default_calibrate_output_path("/data/run_0.db3"), "run_0_tf_static_calib.yaml");
}
