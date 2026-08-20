// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/pointcloud/point_cloud_io.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace commands = bagwiz::commands;
namespace pc = bagwiz::core::pointcloud;

namespace
{
commands::CalibCamLidarArgs valid_args()
{
  commands::CalibCamLidarArgs args;
  args.input_path = "in.db3";
  args.pcd_topic = "/lidar/points";
  args.pose_topic = "/odom";
  args.cam_topic = "/cam/image_raw/compressed";
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

TEST(CalibCamLidarCommonTest, ParseFixSpec)
{
  {
    // Empty (the default) means auto: no manual axes, auto-fix on.
    const auto [spec, err] = commands::parse_fix_spec("");
    EXPECT_EQ(err, "");
    EXPECT_TRUE(spec.auto_fix);
    EXPECT_FALSE(spec.fixed[0]);
    EXPECT_FALSE(spec.fixed[5]);
  }
  {
    const auto [spec, err] = commands::parse_fix_spec("auto");
    EXPECT_EQ(err, "");
    EXPECT_TRUE(spec.auto_fix);
  }
  {
    const auto [spec, err] = commands::parse_fix_spec("none");
    EXPECT_EQ(err, "");
    EXPECT_FALSE(spec.auto_fix);
    EXPECT_FALSE(spec.fixed[0]);
  }
  {
    // A manual axis list replaces the default auto.
    const auto [spec, err] = commands::parse_fix_spec("x,yaw");
    EXPECT_EQ(err, "");
    EXPECT_FALSE(spec.auto_fix);
    EXPECT_TRUE(spec.fixed[0]);
    EXPECT_FALSE(spec.fixed[1]);
    EXPECT_TRUE(spec.fixed[5]);
  }
  {
    // auto composes with manual axes.
    const auto [spec, err] = commands::parse_fix_spec("auto,x");
    EXPECT_EQ(err, "");
    EXPECT_TRUE(spec.auto_fix);
    EXPECT_TRUE(spec.fixed[0]);
  }
  EXPECT_NE(commands::parse_fix_spec("x,bogus").second, "");
  EXPECT_NE(commands::parse_fix_spec("x,y,z,roll,pitch,yaw").second, "");
  EXPECT_NE(commands::parse_fix_spec("none,x").second, "");
  EXPECT_NE(commands::parse_fix_spec("auto,none").second, "");
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

  // The degenerate axis warns that its delta is still in the output and was
  // NOT held at the bag's value, which is what --fix would give. These are the
  // default (--fix auto) args and nothing was held, so it is the auto wording;
  // the --fix none wording has its own test.
  EXPECT_NE(
    summary.find(
      "warning: z reads degenerate on its own probe, but no direction --fix auto could hold "
      "covers it; the delta shown is weakly constrained, not held — re-run with --fix z to "
      "pin it"),
    std::string::npos)
    << summary;
  // Only the degenerate axis warns.
  EXPECT_EQ(summary.find("warning: x "), std::string::npos) << summary;
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

TEST(CalibCamLidarCommonTest, RenderSummaryListsHeldDirections)
{
  const auto args = valid_args();
  auto result = sample_result();
  bagwiz::core::calib::HeldDirection y_held;
  y_held.unit = {0, 1, 0, 0, 0, 0};
  y_held.curvature = 1e-7;
  y_held.std_error = 5e-8;
  result.auto_held.push_back(y_held);
  bagwiz::core::calib::HeldDirection mixed;
  const double s = std::sqrt(0.5);
  mixed.unit = {0, s, 0, 0, 0, s};  // 0.71y + 0.71yaw in the normalized coordinates
  result.auto_held.push_back(mixed);
  // y reads degenerate here so the warning-suppression path is exercised: it
  // is covered by the y-dominated held direction, so no warning may appear
  // for it, while the uncovered degenerate z keeps its warning.
  result.observability[1] = bagwiz::core::calib::AxisObservability::kDegenerate;

  const std::string summary =
    commands::render_calibrate_summary(args, result, kEdgeBefore, "/tmp/out.yaml");
  EXPECT_NE(summary.find("held at bag value (auto): 1.00y"), std::string::npos) << summary;
  // The mixed direction renders in physical units: 0.71 * 0.02 m of y against
  // 0.71 * 0.0035 rad of yaw — dominant y with a small yaw share.
  EXPECT_NE(summary.find("held at bag value (auto): 0.99y + 0.17yaw"), std::string::npos)
    << summary;
  EXPECT_EQ(summary.find("warning: y "), std::string::npos) << summary;
  // z is degenerate and uncovered under the default --fix auto: the warning
  // must describe what actually happened (the axis probe and the auto-hold
  // decision look along different directions) instead of the pre-auto
  // "unconstrained" claim, which would also be telling the user to run the
  // --fix auto that is already running.
  EXPECT_NE(summary.find("warning: z reads degenerate on its own probe"), std::string::npos)
    << summary;
  EXPECT_EQ(summary.find("warning: z is not observable"), std::string::npos) << summary;
}

TEST(CalibCamLidarCommonTest, RenderSummaryWarnsUnconstrainedWithFixNone)
{
  // With auto off, a degenerate axis really is an unconstrained delta nobody
  // held, so the warning keeps the pre-auto wording and its --fix advice.
  auto args = valid_args();
  args.fix_axes = "none";
  auto result = sample_result();
  result.observability[1] = bagwiz::core::calib::AxisObservability::kDegenerate;

  const std::string summary =
    commands::render_calibrate_summary(args, result, kEdgeBefore, "/tmp/out.yaml");
  EXPECT_NE(summary.find("warning: y is not observable from this data"), std::string::npos)
    << summary;
  EXPECT_NE(summary.find("re-run with --fix y to hold the bag value"), std::string::npos)
    << summary;
  EXPECT_EQ(summary.find("reads degenerate on its own probe"), std::string::npos) << summary;
  EXPECT_EQ(summary.find("held at bag value"), std::string::npos) << summary;
}

TEST(CalibCamLidarCommonTest, RenderJsonListsHeldDirections)
{
  auto result = sample_result();
  const std::string empty_json = commands::render_calibrate_json(valid_args(), result, kEdgeBefore);
  EXPECT_NE(empty_json.find("\"held\": []"), std::string::npos) << empty_json;

  bagwiz::core::calib::HeldDirection y_held;
  y_held.unit = {0, 1, 0, 0, 0, 0};
  y_held.curvature = 1e-7;
  y_held.std_error = 5e-8;
  result.auto_held.push_back(y_held);
  const std::string json = commands::render_calibrate_json(valid_args(), result, kEdgeBefore);
  EXPECT_NE(json.find("\"held\": ["), std::string::npos) << json;
  EXPECT_NE(json.find("\"direction\": {\"y\": 1}"), std::string::npos) << json;
  EXPECT_EQ(json.find(",\n  ]"), std::string::npos) << "trailing comma:\n" << json;
  EXPECT_EQ(json.find(",\n    }"), std::string::npos) << "trailing comma:\n" << json;
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

TEST(CalibCamLidarCommonTest, ValidateRejectsEmptyOfOrRefFrame)
{
  auto args = valid_args();
  args.of_frame.clear();
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
  args = valid_args();
  args.ref_frame.clear();
  EXPECT_NE(commands::validate_calibrate_flags(args), "");
}

// ---- MapAccumulator ---------------------------------------------------------

TEST(CalibCamLidarCommonTest, VoxelAccumulatorCollapsesPointsSharingAVoxel)
{
  // Four points inside one 0.5 m voxel collapse to their centroid, carrying
  // the mean intensity — the driving platform re-measuring one surface must
  // not cost the map four points.
  commands::MapAccumulator acc{0.5};
  EXPECT_TRUE(acc.add({1.0F, 1.0F, 1.0F}, 0.2F));
  EXPECT_TRUE(acc.add({1.2F, 1.0F, 1.0F}, 0.4F));
  EXPECT_TRUE(acc.add({1.0F, 1.4F, 1.0F}, 0.6F));
  EXPECT_TRUE(acc.add({1.2F, 1.4F, 1.4F}, 0.8F));
  EXPECT_EQ(acc.size(), 1U);
  const auto map = acc.finish();
  ASSERT_EQ(map.points.size(), 1U);
  EXPECT_FLOAT_EQ(map.points[0][0], 1.1F);
  EXPECT_FLOAT_EQ(map.points[0][1], 1.2F);
  EXPECT_FLOAT_EQ(map.points[0][2], 1.1F);
  ASSERT_EQ(map.intensities.size(), 1U);
  EXPECT_FLOAT_EQ(map.intensities[0], 0.5F);
}

TEST(CalibCamLidarCommonTest, VoxelAccumulatorSeparatesVoxelsAcrossZero)
{
  // Quantization must FLOOR, not truncate toward zero: -0.05 and +0.05 are
  // one 0.1 m voxel apart, and truncation would merge them (and every other
  // pair straddling an axis) into one.
  commands::MapAccumulator acc{0.1};
  EXPECT_TRUE(acc.add({-0.05F, 0.0F, 0.0F}, 1.0F));
  EXPECT_TRUE(acc.add({0.05F, 0.0F, 0.0F}, 1.0F));
  EXPECT_EQ(acc.size(), 2U);
  const auto map = acc.finish();
  ASSERT_EQ(map.points.size(), 2U);
  // Sorted by voxel index, so the negative one comes first.
  EXPECT_LT(map.points[0][0], 0.0F);
  EXPECT_GT(map.points[1][0], 0.0F);
}

TEST(CalibCamLidarCommonTest, VoxelAccumulatorDisabledKeepsEveryPointVerbatim)
{
  // Size 0 is the opt-out: no collapsing, no reordering, no centroid rounding.
  commands::MapAccumulator acc{0.0};
  EXPECT_TRUE(acc.add({1.0F, 1.0F, 1.0F}, 0.25F));
  EXPECT_TRUE(acc.add({1.0F, 1.0F, 1.0F}, 0.75F));
  EXPECT_EQ(acc.size(), 2U);
  const auto map = acc.finish();
  ASSERT_EQ(map.points.size(), 2U);
  EXPECT_FLOAT_EQ(map.intensities[0], 0.25F);
  EXPECT_FLOAT_EQ(map.intensities[1], 0.75F);
}

TEST(CalibCamLidarCommonTest, VoxelAccumulatorOutputDoesNotDependOnInsertionOrder)
{
  // The emitted map is sorted by voxel index, so the same set of points gives
  // byte-identical output whatever order the bag delivered them in — the hash
  // container's own iteration order must never reach the map.
  const std::vector<std::array<float, 3>> pts{
    {5.0F, 0.0F, 0.0F}, {-3.0F, 2.0F, 1.0F}, {0.0F, 0.0F, 0.0F}, {1.0F, -7.0F, 4.0F}};
  commands::MapAccumulator forward{0.5};
  for (const auto & p : pts) {
    EXPECT_TRUE(forward.add(p, 0.5F));
  }
  commands::MapAccumulator backward{0.5};
  for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
    EXPECT_TRUE(backward.add(*it, 0.5F));
  }
  const auto a = forward.finish();
  const auto b = backward.finish();
  ASSERT_EQ(a.points.size(), pts.size());
  ASSERT_EQ(b.points.size(), pts.size());
  for (std::size_t i = 0; i < a.points.size(); ++i) {
    EXPECT_EQ(a.points[i], b.points[i]) << i;
  }
}

TEST(CalibCamLidarCommonTest, VoxelAccumulatorRejectsUnquantizablePoint)
{
  // A coordinate too large to index a voxel is as unusable as a NaN: refused
  // here rather than overflowing the index and landing in a wrong voxel.
  commands::MapAccumulator acc{0.1};
  EXPECT_FALSE(acc.add({1e30F, 0.0F, 0.0F}, 1.0F));
  EXPECT_TRUE(acc.empty());
}

// ---- accumulate_cloud_into_map ----------------------------------------------

// Two poses at 0 s and 10 s translating along +x at `v_mps`, so the
// interpolated T_ref_of at stamp t seconds is the translation (v*t, 0, 0).
std::vector<bagwiz::core::TrajectoryPose> moving_trajectory(double v_mps)
{
  std::vector<bagwiz::core::TrajectoryPose> poses(2);
  poses[0].timestamp_ns = 0;
  poses[0].qw = 1.0;
  poses[1].timestamp_ns = 10'000'000'000LL;
  poses[1].tx = v_mps * 10.0;
  poses[1].qw = 1.0;
  return poses;
}

geometry_msgs::msg::Transform translation_transform(double tx, double ty, double tz)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = tx;
  t.translation.y = ty;
  t.translation.z = tz;
  t.rotation.w = 1.0;
  return t;
}

// x/y/z/intensity as 4x float32 (point_step 16). `pts`: {x, y, z, intensity}.
pc::PointCloud2 make_cloud_xyzi(
  const std::vector<std::array<float, 4>> & pts, std::int64_t stamp_ns)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "lidar";
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
    {"intensity", 12, pc::PointFieldType::kFloat32, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(float) * 4);
  }
  return c;
}

// x/y/z/intensity/t as 5x float32 (point_step 20). `pts`: {x, y, z, intensity,
// t_seconds} with t relative to the cloud's header stamp.
pc::PointCloud2 make_cloud_xyzit(
  const std::vector<std::array<float, 5>> & pts, std::int64_t stamp_ns)
{
  pc::PointCloud2 c;
  c.timestamp_ns = stamp_ns;
  c.frame_id = "lidar";
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 20;
  c.row_step = c.point_step * c.width;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},  {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},  {"intensity", 12, pc::PointFieldType::kFloat32, 1},
    {"t", 16, pc::PointFieldType::kFloat32, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(float) * 5);
  }
  return c;
}

TEST(CalibCamLidarCommonTest, AccumulateCloudAppendsPointsWithIntensities)
{
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);
  const auto error = commands::accumulate_cloud_into_map(
    map, make_cloud_xyzi({{1.0F, 2.0F, 3.0F, 0.25F}, {4.0F, 5.0F, 6.0F, 0.75F}}, 5'000'000'000LL),
    trajectory, std::nullopt, stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  ASSERT_EQ(out.points.size(), 2U);
  EXPECT_FLOAT_EQ(out.points[0][0], 1.0F);
  EXPECT_FLOAT_EQ(out.points[0][1], 2.0F);
  EXPECT_FLOAT_EQ(out.points[0][2], 3.0F);
  ASSERT_EQ(out.intensities.size(), 2U);
  EXPECT_FLOAT_EQ(out.intensities[0], 0.25F);
  EXPECT_FLOAT_EQ(out.intensities[1], 0.75F);
  EXPECT_EQ(stats.clouds_read, 1U);
  EXPECT_EQ(stats.points_added, 2U);
  EXPECT_EQ(stats.clouds_deskewed, 0U);
}

TEST(CalibCamLidarCommonTest, AccumulateCloudVoxelizesRepeatedSweeps)
{
  // The real reason the grid exists: a stationary platform re-measuring one
  // surface. Two sweeps of the same three points must leave three map points,
  // not six — while points_added still counts everything that was read, so
  // the log can show the gap.
  commands::MapAccumulator map{0.5};
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);
  const std::vector<std::array<float, 4>> sweep{
    {1.0F, 0.0F, 0.0F, 0.2F}, {1.1F, 0.0F, 0.0F, 0.4F}, {9.0F, 0.0F, 0.0F, 1.0F}};
  for (int i = 0; i < 2; ++i) {
    const auto error = commands::accumulate_cloud_into_map(
      map, make_cloud_xyzi(sweep, 5'000'000'000LL), trajectory, std::nullopt, stats);
    EXPECT_FALSE(error.has_value()) << *error;
  }
  EXPECT_EQ(stats.points_added, 6U);
  const auto out = map.finish();
  // (1.0, 1.1) share the [1.0, 1.5) voxel; 9.0 is its own.
  ASSERT_EQ(out.points.size(), 2U);
  EXPECT_FLOAT_EQ(out.points[0][0], 1.05F);
  EXPECT_FLOAT_EQ(out.points[1][0], 9.0F);
  ASSERT_EQ(out.intensities.size(), 2U);
  EXPECT_FLOAT_EQ(out.intensities[0], 0.3F);  // mean of 0.2 and 0.4, twice over
  EXPECT_FLOAT_EQ(out.intensities[1], 1.0F);
}

TEST(CalibCamLidarCommonTest, AccumulateCloudAppliesPoseAndExtrinsic)
{
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  // T_ref_of(5 s) = (5, 0, 0); the extrinsic lifts the cloud frame by +1 in y,
  // so (1, 2, 3) in the cloud lands at (6, 3, 3) in the ref frame.
  const auto trajectory = moving_trajectory(1.0);
  const auto error = commands::accumulate_cloud_into_map(
    map, make_cloud_xyzi({{1.0F, 2.0F, 3.0F, 0.5F}}, 5'000'000'000LL), trajectory,
    translation_transform(0.0, 1.0, 0.0), stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_FLOAT_EQ(out.points[0][0], 6.0F);
  EXPECT_FLOAT_EQ(out.points[0][1], 3.0F);
  EXPECT_FLOAT_EQ(out.points[0][2], 3.0F);
}

TEST(CalibCamLidarCommonTest, AccumulateCloudRejectsCloudWithoutIntensity)
{
  pc::PointCloud2 c;
  c.timestamp_ns = 5'000'000'000LL;
  c.frame_id = "lidar";
  c.height = 1;
  c.width = 1;
  c.point_step = 12;
  c.row_step = c.point_step;
  c.fields = {
    {"x", 0, pc::PointFieldType::kFloat32, 1},
    {"y", 4, pc::PointFieldType::kFloat32, 1},
    {"z", 8, pc::PointFieldType::kFloat32, 1},
  };
  c.data.resize(c.point_step);
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);
  EXPECT_TRUE(
    commands::accumulate_cloud_into_map(map, c, trajectory, std::nullopt, stats).has_value());
  const auto out = map.finish();
  EXPECT_TRUE(out.points.empty());
}

TEST(CalibCamLidarCommonTest, AccumulateCloudSkipsStampOutsideTrajectorySpan)
{
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);  // spans 0..10 s
  const auto error = commands::accumulate_cloud_into_map(
    map, make_cloud_xyzi({{1.0F, 2.0F, 3.0F, 0.5F}}, 20'000'000'000LL), trajectory, std::nullopt,
    stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  EXPECT_TRUE(out.points.empty());
  EXPECT_EQ(stats.clouds_skipped_out_of_span, 1U);
  EXPECT_EQ(stats.points_added, 0U);
}

// A cloud whose per-point times are all one value (e.g. the all-zero field
// `pcd undistort` leaves behind) carries no sweep motion, so it must be
// accumulated as-is — deskewing it would be a no-op that only rewrites bytes.
TEST(CalibCamLidarCommonTest, AccumulateCloudDoesNotDeskewUniformTimeField)
{
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(1.0);
  const auto error = commands::accumulate_cloud_into_map(
    map, make_cloud_xyzit({{0.0F, 0.0F, 0.0F, 0.5F, 0.0F}}, 5'000'000'000LL), trajectory,
    std::nullopt, stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_FLOAT_EQ(out.points[0][0], 5.0F);  // placed at the header stamp, un-deskewed
  EXPECT_EQ(stats.clouds_deskewed, 0U);
}

// A non-uniform per-point time field means a real sweep: deskew each point to
// the header stamp before the ref-frame placement, so the point captured 0.5 s
// into the sweep lands at the pose of 5.5 s, not 5 s.
TEST(CalibCamLidarCommonTest, AccumulateCloudDeskewsVaryingTimeField)
{
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(1.0);
  const auto error = commands::accumulate_cloud_into_map(
    map,
    make_cloud_xyzit(
      {{0.0F, 0.0F, 0.0F, 0.5F, 0.5F}, {0.0F, 0.0F, 0.0F, 0.25F, 0.0F}}, 5'000'000'000LL),
    trajectory, std::nullopt, stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  ASSERT_EQ(out.points.size(), 2U);
  EXPECT_FLOAT_EQ(out.points[0][0], 5.5F);
  EXPECT_FLOAT_EQ(out.points[1][0], 5.0F);
  EXPECT_EQ(stats.clouds_deskewed, 1U);
}

TEST(CalibCamLidarCommonTest, AccumulateCloudDropsNonFinitePoints)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);
  const auto error = commands::accumulate_cloud_into_map(
    map, make_cloud_xyzi({{nan, 0.0F, 0.0F, 0.5F}, {1.0F, 2.0F, 3.0F, 0.5F}}, 5'000'000'000LL),
    trajectory, std::nullopt, stats);
  EXPECT_FALSE(error.has_value()) << *error;
  const auto out = map.finish();
  ASSERT_EQ(out.points.size(), 1U);
  EXPECT_FLOAT_EQ(out.points[0][0], 1.0F);
  EXPECT_EQ(stats.points_dropped_nonfinite, 1U);
}

TEST(CalibCamLidarCommonTest, AccumulateCloudRejectsBigEndianCloud)
{
  auto c = make_cloud_xyzi({{1.0F, 2.0F, 3.0F, 0.5F}}, 5'000'000'000LL);
  c.is_bigendian = true;
  commands::MapAccumulator map{0.0};  // grid off: these pin placement, not downsampling
  commands::MapAccumulationStats stats;
  const auto trajectory = moving_trajectory(0.0);
  EXPECT_TRUE(
    commands::accumulate_cloud_into_map(map, c, trajectory, std::nullopt, stats).has_value());
  const auto out = map.finish();
  EXPECT_TRUE(out.points.empty());
}
