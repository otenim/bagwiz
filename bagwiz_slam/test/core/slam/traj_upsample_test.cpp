// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/traj_upsample.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{

constexpr std::int64_t kMs = 1'000'000;

Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  return t;
}

Eigen::Isometry3d yaw_at(double radians, double x, double y)
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = Eigen::AngleAxisd(radians, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  t.translation() = Eigen::Vector3d(x, y, 0.0);
  return t;
}

// The stamp and +x position of sample `i` in a chain of `n` samples spaced
// `step_ns` from `begin_ns`, translating at `speed` m/s from `x0`.
std::pair<std::int64_t, double> straight_knot(
  std::int64_t begin_ns, int i, std::int64_t step_ns, double x0, double speed)
{
  const std::int64_t stamp_ns = begin_ns + static_cast<std::int64_t>(i) * step_ns;
  const double seconds = static_cast<double>(stamp_ns - begin_ns) / 1e9;
  return {stamp_ns, x0 + speed * seconds};
}

// A raw GLIM chain (odometry world<-IMU), translating along +x. Identity orientation.
std::vector<StampedImuPose> imu_chain(
  std::int64_t begin_ns, int n, std::int64_t step_ns, double x0, double speed)
{
  std::vector<StampedImuPose> poses;
  poses.reserve(n);
  for (int i = 0; i < n; ++i) {
    const auto [stamp_ns, x] = straight_knot(begin_ns, i, step_ns, x0, speed);
    poses.push_back({stamp_ns, translation(x, 0.0, 0.0)});
  }
  return poses;
}

// An already re-anchored chain (world<-LiDAR) as upsample_trajectory consumes it.
ImuRateChain lidar_chain(
  std::int64_t begin_ns, int n, std::int64_t step_ns, double x0, double speed)
{
  ImuRateChain chain;
  chain.poses.reserve(n);
  for (int i = 0; i < n; ++i) {
    const auto [stamp_ns, x] = straight_knot(begin_ns, i, step_ns, x0, speed);
    chain.poses.push_back({stamp_ns, translation(x, 0.0, 0.0)});
  }
  return chain;
}

void expect_pose_near(
  const Eigen::Isometry3d & actual, const Eigen::Isometry3d & expected, double tol = 1e-9)
{
  EXPECT_LT((actual.translation() - expected.translation()).norm(), tol)
    << "translation: " << actual.translation().transpose() << " vs "
    << expected.translation().transpose();
  const Eigen::Quaterniond qa(actual.rotation());
  const Eigen::Quaterniond qe(expected.rotation());
  EXPECT_LT(std::abs(std::abs(qa.dot(qe)) - 1.0), tol) << "rotation mismatch";
}

TEST(ReanchorChain, PlacesTheRelativeMotionOnTheOptimizedAnchor)
{
  // Odometry-frame chain: starts at x=5 and travels +x at 1 m/s for 100 ms.
  const std::vector<StampedImuPose> raw = imu_chain(0, 11, 10 * kMs, 5.0, 1.0);

  // The optimized pose for the same frame sits somewhere else entirely.
  const ImuRateChain chain =
    reanchor_chain(raw, translation(100.0, 7.0, 0.0), std::nullopt, Eigen::Isometry3d::Identity());

  ASSERT_EQ(chain.poses.size(), raw.size());
  // The first knot lands exactly on the optimized anchor.
  expect_pose_near(chain.poses.front().T_world_lidar, translation(100.0, 7.0, 0.0));
  // The relative motion is preserved: +1 m/s along +x.
  for (std::size_t i = 0; i < chain.poses.size(); ++i) {
    const double seconds = static_cast<double>(i) * 0.01;
    EXPECT_EQ(chain.poses[i].stamp_ns, raw[i].stamp_ns);
    expect_pose_near(chain.poses[i].T_world_lidar, translation(100.0 + seconds, 7.0, 0.0));
  }
}

TEST(ReanchorChain, AppliesTheLidarImuExtrinsic)
{
  // A pure yaw sweep in the odometry frame, IMU offset 1 m ahead of the LiDAR.
  std::vector<StampedImuPose> raw;
  for (int i = 0; i < 5; ++i) {
    raw.push_back({static_cast<std::int64_t>(i) * 10 * kMs, yaw_at(0.1 * i, 0.0, 0.0)});
  }
  const Eigen::Isometry3d T_lidar_imu = translation(1.0, 0.0, 0.0);
  const Eigen::Isometry3d T_world_lidar_begin = Eigen::Isometry3d::Identity();

  const ImuRateChain chain = reanchor_chain(raw, T_world_lidar_begin, std::nullopt, T_lidar_imu);

  ASSERT_EQ(chain.poses.size(), raw.size());
  for (std::size_t i = 0; i < chain.poses.size(); ++i) {
    // Closed form: T_world_lidar(t) = (begin * T_lidar_imu) * delta(t) * T_lidar_imu^-1
    const Eigen::Isometry3d delta = raw.front().T_world_imu.inverse() * raw[i].T_world_imu;
    const Eigen::Isometry3d expected =
      T_world_lidar_begin * T_lidar_imu * delta * T_lidar_imu.inverse();
    expect_pose_near(chain.poses[i].T_world_lidar, expected);
  }
  // The extrinsic makes the LiDAR swing on a 1 m radius rather than spin in place.
  EXPECT_GT(chain.poses.back().T_world_lidar.translation().norm(), 1e-3);
}

TEST(ReanchorChain, EndpointBlendLandsOnTheOptimizedEndPose)
{
  const std::vector<StampedImuPose> raw = imu_chain(0, 11, 10 * kMs, 0.0, 1.0);
  const Eigen::Isometry3d begin = translation(0.0, 0.0, 0.0);
  // The optimized next frame disagrees with free-run dead reckoning by 5 cm and a
  // small yaw — exactly the correction the global optimization applies.
  const Eigen::Isometry3d end = yaw_at(0.02, 0.95, 0.05);

  const ImuRateChain chain = reanchor_chain(raw, begin, end, Eigen::Isometry3d::Identity());

  ASSERT_EQ(chain.poses.size(), raw.size());
  expect_pose_near(chain.poses.front().T_world_lidar, begin);
  expect_pose_near(chain.poses.back().T_world_lidar, end);
  // The correction is spread across the chain, not dumped on the last knot.
  const double midpoint_x = chain.poses[5].T_world_lidar.translation().x();
  EXPECT_GT(midpoint_x, 0.45);
  EXPECT_LT(midpoint_x, 0.51);
}

TEST(ReanchorChain, BlendKeepsTheChainMonotonicAndContinuous)
{
  const std::vector<StampedImuPose> raw = imu_chain(0, 11, 10 * kMs, 0.0, 1.0);
  const ImuRateChain chain = reanchor_chain(
    raw, Eigen::Isometry3d::Identity(), yaw_at(0.05, 1.10, 0.10), Eigen::Isometry3d::Identity());

  ASSERT_EQ(chain.poses.size(), 11u);
  for (std::size_t i = 1; i < chain.poses.size(); ++i) {
    EXPECT_GT(chain.poses[i].stamp_ns, chain.poses[i - 1].stamp_ns);
    const double step =
      (chain.poses[i].T_world_lidar.translation() - chain.poses[i - 1].T_world_lidar.translation())
        .norm();
    EXPECT_GT(step, 0.0);
    EXPECT_LT(step, 0.2) << "discontinuous jump at knot " << i;
  }
}

TEST(ReanchorChain, RejectsChainsTooShortToAnchor)
{
  EXPECT_TRUE(
    reanchor_chain({}, Eigen::Isometry3d::Identity(), std::nullopt, Eigen::Isometry3d::Identity())
      .poses.empty());

  const std::vector<StampedImuPose> single = imu_chain(0, 1, 10 * kMs, 0.0, 1.0);
  EXPECT_TRUE(reanchor_chain(
                single, Eigen::Isometry3d::Identity(), std::nullopt, Eigen::Isometry3d::Identity())
                .poses.empty());
}

TEST(ReanchorChain, IgnoresTheBlendWhenTheChainHasNoSpan)
{
  // All knots share a stamp: the blend fraction would divide by zero.
  std::vector<StampedImuPose> raw;
  raw.push_back({0, translation(0.0, 0.0, 0.0)});
  raw.push_back({0, translation(1.0, 0.0, 0.0)});

  const ImuRateChain chain = reanchor_chain(
    raw, Eigen::Isometry3d::Identity(), translation(9.0, 0.0, 0.0), Eigen::Isometry3d::Identity());

  ASSERT_EQ(chain.poses.size(), 2u);
  expect_pose_near(chain.poses.front().T_world_lidar, translation(0.0, 0.0, 0.0));
  expect_pose_near(chain.poses.back().T_world_lidar, translation(1.0, 0.0, 0.0));
}

// --- upsample_trajectory ---------------------------------------------------

// Two optimized poses 100 ms apart, with the IMU-rate chain that spans them.
std::vector<StampedPose> two_scan_trajectory()
{
  return {{0, translation(0.0, 0.0, 0.0)}, {100 * kMs, translation(1.0, 0.0, 0.0)}};
}

TEST(UpsampleTrajectory, EmitsAGridAnchoredAtTheFirstStamp)
{
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 11, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 20 * kMs, stats);

  // 0, 20, 40, 60, 80, 100 ms — the 0 and 100 rows come from `optimized`.
  ASSERT_EQ(dense.size(), 6u);
  for (std::size_t i = 0; i < dense.size(); ++i) {
    EXPECT_EQ(dense[i].stamp_ns, static_cast<std::int64_t>(i) * 20 * kMs);
  }
  EXPECT_EQ(stats.grid_poses, 4u);  // 20, 40, 60, 80
  EXPECT_EQ(stats.uncovered_gaps, 0u);
}

TEST(UpsampleTrajectory, InterpolatesGridPosesFromTheChain)
{
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 11, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 25 * kMs, stats);

  // 25 ms is between chain knots (20 ms and 30 ms) and must interpolate.
  ASSERT_EQ(dense.size(), 5u);      // 0, 25, 50, 75, 100
  EXPECT_EQ(stats.grid_poses, 3u);  // 25, 50, 75 — the 100 ms grid point collides
  expect_pose_near(dense[1].T_world_lidar, translation(0.25, 0.0, 0.0));
  expect_pose_near(dense[2].T_world_lidar, translation(0.50, 0.0, 0.0));
  expect_pose_near(dense[3].T_world_lidar, translation(0.75, 0.0, 0.0));
}

TEST(UpsampleTrajectory, OptimizedPosesWinOnAStampCollision)
{
  // A chain that deliberately disagrees with the optimized end pose: the row at
  // 100 ms must still carry the optimized value, not the chain's.
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 11, 10 * kMs, 0.0, 50.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 50 * kMs, stats);

  ASSERT_EQ(dense.size(), 3u);
  EXPECT_EQ(dense.back().stamp_ns, 100 * kMs);
  expect_pose_near(dense.back().T_world_lidar, translation(1.0, 0.0, 0.0));
  EXPECT_EQ(stats.grid_poses, 1u);  // only the 50 ms row is new
}

TEST(UpsampleTrajectory, CoalescesGridSamplesThatLandInsideTheStampNoise)
{
  // The optimized stamps carry a few hundred ns of noise (they round-trip through
  // a double holding epoch-scale seconds), so a grid phased on the first pose
  // lands just beside each later one. Those samples are duplicates, not new rows.
  const std::int64_t jitter = 120;
  const std::vector<StampedPose> optimized = {
    {0, translation(0.0, 0.0, 0.0)},
    {100 * kMs - jitter, translation(1.0, 0.0, 0.0)},
    {200 * kMs + jitter, translation(2.0, 0.0, 0.0)}};
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 21, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 50 * kMs, stats);

  // Grid at 50/100/150/200 ms; the 100 and 200 ms samples sit 120 ns from an
  // optimized row, so only 50 and 150 ms survive.
  ASSERT_EQ(dense.size(), 5u);
  EXPECT_EQ(stats.grid_poses, 2u);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    EXPECT_GT(dense[i].stamp_ns - dense[i - 1].stamp_ns, jitter)
      << "near-duplicate rows at index " << i;
  }
}

TEST(UpsampleTrajectory, CoalescesGridSamplesInsideTheSensorStampJitter)
{
  // Real scan stamps jitter around their nominal period by tens of microseconds,
  // far above the representation noise. At a 10 ms grid the window is 100 us, so
  // a scan 12 us off the grid still absorbs its sample instead of doubling it.
  const std::int64_t jitter = 12'000;  // 12 us, as measured on a driving bag
  std::vector<StampedPose> optimized;
  for (int i = 0; i <= 3; ++i) {
    const std::int64_t sign = (i % 2 == 0) ? 1 : -1;
    optimized.push_back(
      {static_cast<std::int64_t>(i) * 100 * kMs + (i == 0 ? 0 : sign * jitter),
       translation(static_cast<double>(i), 0.0, 0.0)});
  }
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 31, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 10 * kMs, stats);

  for (std::size_t i = 1; i < dense.size(); ++i) {
    EXPECT_GT(dense[i].stamp_ns - dense[i - 1].stamp_ns, jitter)
      << "near-duplicate rows at index " << i;
  }
  // Every optimized pose still survives.
  EXPECT_EQ(dense.front().stamp_ns, optimized.front().stamp_ns);
  EXPECT_EQ(dense.back().stamp_ns, optimized.back().stamp_ns);
}

TEST(UpsampleTrajectory, CoalescingNeverSwallowsAFineGrid)
{
  // The tolerance is capped below the period, so a grid finer than the tolerance
  // still comes through in full.
  const std::vector<StampedPose> optimized = {
    {0, translation(0.0, 0.0, 0.0)}, {2'000, translation(1.0, 0.0, 0.0)}};
  std::vector<ImuRateChain> chains(1);
  for (int i = 0; i <= 4; ++i) {
    chains[0].poses.push_back(
      {static_cast<std::int64_t>(i) * 500, translation(0.25 * i, 0.0, 0.0)});
  }

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 500, stats);

  // 500 ns grid over a 2 us span: 500, 1000, 1500 are new, 2000 collides.
  ASSERT_EQ(dense.size(), 5u);
  EXPECT_EQ(stats.grid_poses, 3u);
}

TEST(UpsampleTrajectory, SkipsSpansNoChainCovers)
{
  // Optimized poses at 0, 100, 200, 300 ms but a chain only over [0, 100] and
  // [200, 300] — the middle span has no IMU-rate poses to resample.
  const std::vector<StampedPose> optimized = {
    {0, translation(0.0, 0.0, 0.0)},
    {100 * kMs, translation(1.0, 0.0, 0.0)},
    {200 * kMs, translation(2.0, 0.0, 0.0)},
    {300 * kMs, translation(3.0, 0.0, 0.0)}};
  const std::vector<ImuRateChain> chains = {
    lidar_chain(0, 11, 10 * kMs, 0.0, 10.0), lidar_chain(200 * kMs, 11, 10 * kMs, 2.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 25 * kMs, stats);

  // No row may fall strictly inside the uncovered (100, 200) ms span.
  for (const auto & pose : dense) {
    const bool inside_gap = pose.stamp_ns > 100 * kMs && pose.stamp_ns < 200 * kMs;
    EXPECT_FALSE(inside_gap) << "stamp " << pose.stamp_ns << " should not have been emitted";
  }
  EXPECT_EQ(stats.uncovered_gaps, 1u);
  // The optimized poses all survive.
  EXPECT_EQ(dense.front().stamp_ns, 0);
  EXPECT_EQ(dense.back().stamp_ns, 300 * kMs);
}

TEST(UpsampleTrajectory, CoarsePeriodSubsamples)
{
  const std::vector<StampedPose> optimized = {
    {0, translation(0.0, 0.0, 0.0)},
    {100 * kMs, translation(1.0, 0.0, 0.0)},
    {200 * kMs, translation(2.0, 0.0, 0.0)}};
  const std::vector<ImuRateChain> chains = {
    lidar_chain(0, 11, 10 * kMs, 0.0, 10.0), lidar_chain(100 * kMs, 11, 10 * kMs, 1.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 150 * kMs, stats);

  // Grid at 150 ms only; the three optimized rows stay.
  ASSERT_EQ(dense.size(), 4u);
  EXPECT_EQ(dense[2].stamp_ns, 150 * kMs);
  EXPECT_EQ(stats.grid_poses, 1u);
}

TEST(UpsampleTrajectory, StampsAreStrictlyIncreasing)
{
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 11, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 7 * kMs, stats);

  ASSERT_GT(dense.size(), 2u);
  for (std::size_t i = 1; i < dense.size(); ++i) {
    EXPECT_GT(dense[i].stamp_ns, dense[i - 1].stamp_ns);
  }
}

TEST(UpsampleTrajectory, PassesTheInputThroughWhenDisabled)
{
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<ImuRateChain> chains = {lidar_chain(0, 11, 10 * kMs, 0.0, 10.0)};

  UpsampleStats stats;
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, chains, 0, stats);

  ASSERT_EQ(dense.size(), optimized.size());
  EXPECT_EQ(stats.grid_poses, 0u);
  EXPECT_EQ(stats.uncovered_gaps, 0u);
}

TEST(UpsampleTrajectory, HandlesDegenerateInputs)
{
  UpsampleStats stats;
  EXPECT_TRUE(upsample_trajectory({}, {}, 10 * kMs, stats).empty());

  const std::vector<StampedPose> single = {{0, translation(0.0, 0.0, 0.0)}};
  EXPECT_EQ(upsample_trajectory(single, {}, 10 * kMs, stats).size(), 1u);

  // No chains at all: nothing to resample from, the whole span is one gap.
  const std::vector<StampedPose> optimized = two_scan_trajectory();
  const std::vector<StampedPose> dense = upsample_trajectory(optimized, {}, 20 * kMs, stats);
  EXPECT_EQ(dense.size(), optimized.size());
  EXPECT_EQ(stats.grid_poses, 0u);
  EXPECT_EQ(stats.uncovered_gaps, 1u);
}

}  // namespace
}  // namespace bagwiz::core::slam
