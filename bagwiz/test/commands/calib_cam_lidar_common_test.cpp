// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace commands = bagwiz::commands;

namespace
{
commands::CalibCamLidarArgs valid_args()
{
  commands::CalibCamLidarArgs args;
  args.input_path = "in.db3";
  args.map_path = "map.pcd";
  args.traj_path = "traj.tum";
  args.traj_frame = "base_link";
  args.topic = "/cam/image_raw/compressed";
  args.parent_frame = "cabin";
  args.child_frame = "cam_link";
  return args;
}

// The edited edge's bag value the render tests measure "before + delta"
// against: deliberately non-zero on every axis so a dropped or swapped one
// shows up.
constexpr std::array<double, 6> kEdgeBefore{1.5, -0.25, 2.0, 0.1, -0.2, 0.3};

bagwiz::core::calib::RefineResult sample_result()
{
  using bagwiz::core::calib::AxisObservability;
  bagwiz::core::calib::RefineResult result;
  result.ok = true;
  result.delta = {0.01, -0.02, 0.03, 0.001, -0.002, 0.003};
  result.nid_before = 0.5;
  result.nid_after = 0.25;
  result.samples_used = 7;
  result.observability = {AxisObservability::kStrong,     AxisObservability::kWeak,
                          AxisObservability::kDegenerate, AxisObservability::kStrong,
                          AxisObservability::kFixed,      AxisObservability::kStrong};
  return result;
}

// One per-axis row of render_calibrate_summary's table, located by its leading
// axis name so the test does not restate the column formatting.
struct AxisRow
{
  double before = 0.0;
  double after = 0.0;
  double delta = 0.0;
  std::string observability;
};

AxisRow parse_summary_row(const std::string & summary, const std::string & axis)
{
  std::istringstream lines(summary);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream fields(line);
    std::string name;
    AxisRow row;
    if (!(fields >> name) || name != axis) {
      continue;
    }
    if (fields >> row.before >> row.after >> row.delta >> row.observability) {
      return row;
    }
  }
  ADD_FAILURE() << "no row for axis '" << axis << "' in:\n" << summary;
  return {};
}

// The six per-axis values of one field of render_calibrate_json's output, in
// its fixed x,y,z,roll,pitch,yaw order. The JSON is hand-built with a stable
// shape, so scanning for the field's occurrences in order is enough and keeps
// a JSON dependency out of the test. ("before" cannot collide with
// "nid_before": the leading quote is part of the key.)
std::array<double, 6> json_axis_field(const std::string & json, const std::string & field)
{
  std::array<double, 6> out{};
  const std::string key = "\"" + field + "\": ";
  std::size_t pos = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    pos = json.find(key, pos);
    EXPECT_NE(pos, std::string::npos) << "missing '" << field << "' #" << i << " in:\n" << json;
    if (pos == std::string::npos) {
      return out;
    }
    pos += key.size();
    out[i] = std::stod(json.substr(pos));
  }
  return out;
}
}  // namespace

TEST(CalibCamLidarCommonTest, ValidArgsPass)
{
  EXPECT_EQ(commands::validate_calibrate_flags(valid_args()), "");
}

TEST(CalibCamLidarCommonTest, RejectsTooFewSamplesAndBadDepthWindow)
{
  auto args = valid_args();
  args.samples = 2;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
  args = valid_args();
  args.min_depth = 10.0;
  args.max_depth = 5.0;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
}

TEST(CalibCamLidarCommonTest, ParseFixedAxes)
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

TEST(CalibCamLidarCommonTest, PickSampleIndicesRespectsMarginAndSpread)
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

TEST(CalibCamLidarCommonTest, InterpolateTrajectoryLerpsBetweenPoses)
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

TEST(CalibCamLidarCommonTest, DefaultOutputPathUsesInputStem)
{
  EXPECT_EQ(
    commands::default_calib_cam_lidar_output_path("/data/run_0.db3"), "run_0_calib_cam_lidar.yaml");
}

TEST(CalibCamLidarCommonTest, Mat4FromQuatRejectsUnusableRotations)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(commands::mat4_from_quat(1, 2, 3, 0, 0, 0, 1).has_value());
  // A default-initialised geometry_msgs Quaternion is all zeros: normalizing
  // it would divide by zero and fill the transform with NaNs.
  EXPECT_FALSE(commands::mat4_from_quat(0, 0, 0, 0, 0, 0, 0).has_value());
  EXPECT_FALSE(commands::mat4_from_quat(0, 0, 0, nan, 0, 0, 1).has_value());
  EXPECT_FALSE(commands::mat4_from_quat(nan, 0, 0, 0, 0, 0, 1).has_value());
}

TEST(CalibCamLidarCommonTest, RenderCalibrateSummaryIsAdditivePerAxis)
{
  const auto args = valid_args();
  const auto result = sample_result();
  const std::string summary =
    commands::render_calibrate_summary(args, result, kEdgeBefore, "/tmp/out.yaml");

  const std::array<const char *, 6> names{"x", "y", "z", "roll", "pitch", "yaw"};
  for (std::size_t axis = 0; axis < names.size(); ++axis) {
    const auto row = parse_summary_row(summary, names[axis]);
    // Rotations are shown in degrees, translations in meters; either way the
    // refined value is the bag value plus the delta in the SAME unit.
    const double scale = axis >= 3 ? 180.0 / M_PI : 1.0;
    EXPECT_NEAR(row.before, kEdgeBefore[axis] * scale, 2e-6) << names[axis];
    EXPECT_NEAR(row.delta, result.delta[axis] * scale, 2e-6) << names[axis];
    EXPECT_NEAR(row.after, row.before + row.delta, 2e-6) << names[axis];
  }
  EXPECT_EQ(parse_summary_row(summary, "x").observability, "strong");
  EXPECT_EQ(parse_summary_row(summary, "y").observability, "weak");
  EXPECT_EQ(parse_summary_row(summary, "z").observability, "degenerate");
  EXPECT_EQ(parse_summary_row(summary, "pitch").observability, "fixed");

  // The degenerate axis warns that its delta is unconstrained — it is NOT the
  // bag's value, which is what --fix would give.
  EXPECT_NE(
    summary.find(
      "warning: z is not observable from this data; the delta shown is unconstrained — re-run "
      "with --fix z to hold the bag value"),
    std::string::npos)
    << summary;
  // Only the degenerate axis warns.
  EXPECT_EQ(summary.find("warning: x is not"), std::string::npos) << summary;
  EXPECT_NE(
    summary.find("apply with: bagwiz tf static update -i in.db3 --yaml /tmp/out.yaml"),
    std::string::npos)
    << summary;
  EXPECT_NE(summary.find("samples used: 7"), std::string::npos) << summary;
  EXPECT_NE(summary.find("calib cam-lidar: cabin -> cam_link"), std::string::npos) << summary;
}

TEST(CalibCamLidarCommonTest, RenderCalibrateJsonIsAdditivePerAxis)
{
  const auto args = valid_args();
  const auto result = sample_result();
  const std::string json = commands::render_calibrate_json(args, result, kEdgeBefore);

  ASSERT_FALSE(json.empty());
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '}');
  int depth = 0;
  for (const char c : json) {
    depth += c == '{' ? 1 : c == '}' ? -1 : 0;
    ASSERT_GE(depth, 0) << json;
  }
  EXPECT_EQ(depth, 0) << json;
  EXPECT_EQ(json.find(",\n  }"), std::string::npos) << "trailing comma:\n" << json;
  EXPECT_EQ(json.find(",\n    }"), std::string::npos) << "trailing comma:\n" << json;

  // Rotation axes stay in radians here, unlike the human table.
  const auto before = json_axis_field(json, "before");
  const auto after = json_axis_field(json, "after");
  const auto delta = json_axis_field(json, "delta");
  for (std::size_t axis = 0; axis < 6; ++axis) {
    EXPECT_DOUBLE_EQ(before[axis], kEdgeBefore[axis]) << "axis " << axis;
    EXPECT_DOUBLE_EQ(delta[axis], result.delta[axis]) << "axis " << axis;
    EXPECT_DOUBLE_EQ(after[axis], kEdgeBefore[axis] + result.delta[axis]) << "axis " << axis;
  }
  EXPECT_NE(json.find("\"parent\": \"cabin\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"child\": \"cam_link\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"samples\": 7"), std::string::npos) << json;
  EXPECT_NE(json.find("\"nid_before\": 0.5"), std::string::npos) << json;
  EXPECT_NE(json.find("\"nid_after\": 0.25"), std::string::npos) << json;
  EXPECT_NE(json.find("\"observability\": \"degenerate\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"observability\": \"fixed\""), std::string::npos) << json;
}

TEST(CalibCamLidarCommonTest, RenderCalibrateJsonEscapesFrameNames)
{
  auto args = valid_args();
  args.parent_frame = "ca\"b\\in";
  const std::string json = commands::render_calibrate_json(args, sample_result(), kEdgeBefore);
  EXPECT_NE(json.find("\"parent\": \"ca\\\"b\\\\in\""), std::string::npos) << json;
}

TEST(CalibCamLidarCommonTest, PoseGateSplitsOnTranslation)
{
  namespace calib = bagwiz::core::calib;
  std::vector<calib::Mat4> poses;
  for (int i = 0; i < 5; ++i) {
    poses.push_back(calib::make_transform({0.4 * i, 0.0, 0.0}, {0, 0, 0}));
  }
  // Anchors at x=0.0 then x=1.2 (the first pose >= 1.0 m from its anchor).
  const auto intervals = commands::pose_gate_intervals(poses, 1.0, 0.0);
  ASSERT_EQ(intervals.size(), 2U);
  EXPECT_EQ(intervals[0], (std::pair<std::size_t, std::size_t>{0, 3}));
  EXPECT_EQ(intervals[1], (std::pair<std::size_t, std::size_t>{3, 5}));
}

TEST(CalibCamLidarCommonTest, PoseGateSplitsOnRotation)
{
  namespace calib = bagwiz::core::calib;
  std::vector<calib::Mat4> poses;
  for (int i = 0; i < 3; ++i) {
    poses.push_back(calib::make_transform({0, 0, 0}, {0, 0, 0.06 * i}));
  }
  // Translation half disabled (<= 0); 0.12 rad >= 0.1 opens the second bucket.
  const auto intervals = commands::pose_gate_intervals(poses, 0.0, 0.1);
  ASSERT_EQ(intervals.size(), 2U);
  EXPECT_EQ(intervals[0], (std::pair<std::size_t, std::size_t>{0, 2}));
  EXPECT_EQ(intervals[1], (std::pair<std::size_t, std::size_t>{2, 3}));
}

TEST(CalibCamLidarCommonTest, PoseGateStationaryYieldsOneInterval)
{
  namespace calib = bagwiz::core::calib;
  const std::vector<calib::Mat4> poses(6, calib::identity_mat4());
  const auto intervals = commands::pose_gate_intervals(poses, 1.0, 0.1);
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_EQ(intervals[0], (std::pair<std::size_t, std::size_t>{0, 6}));
  EXPECT_TRUE(commands::pose_gate_intervals({}, 1.0, 0.1).empty());
}

TEST(CalibCamLidarCommonTest, GraySharpnessPrefersEdgesOverUniform)
{
  namespace calib = bagwiz::core::calib;
  calib::GrayImage uniform;
  uniform.width = 8;
  uniform.height = 8;
  uniform.gray.assign(64, 128);
  // Width-2 vertical stripes, not a 1-px checkerboard: a period-2 pattern is
  // invisible to central differences (gray[x+1] == gray[x-1]).
  calib::GrayImage stripes = uniform;
  for (std::uint32_t y = 0; y < 8; ++y) {
    for (std::uint32_t x = 0; x < 8; ++x) {
      stripes.gray[y * 8 + x] = ((x / 2) % 2 == 0) ? 0 : 255;
    }
  }
  EXPECT_EQ(commands::gray_sharpness(uniform), 0.0);
  EXPECT_GT(commands::gray_sharpness(stripes), 100.0);
  calib::GrayImage degenerate;  // no interior pixels
  degenerate.width = 2;
  degenerate.height = 2;
  degenerate.gray.assign(4, 0);
  EXPECT_EQ(commands::gray_sharpness(degenerate), 0.0);
}

TEST(CalibCamLidarCommonTest, ValidateRejectsNegativeKeyframeThresholds)
{
  auto args = valid_args();
  args.keyframe_dist = -0.1;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
  args = valid_args();
  args.keyframe_rot_deg = -1.0;
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
  args = valid_args();
  args.keyframe_dist = 1.0;
  args.keyframe_rot_deg = 10.0;
  EXPECT_EQ(commands::validate_calibrate_flags(args), "");
}
