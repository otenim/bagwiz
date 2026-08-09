// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__TRAJ_UPSAMPLE_HPP_
#define BAGWIZ__CORE__SLAM__TRAJ_UPSAMPLE_HPP_

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// Resampling the exported trajectory onto an arbitrary rate (`map slam --upsample`).
//
// The exported trajectory carries one pose per LiDAR scan (per visual keyframe in
// camera-only mode) because that is where the factor graph puts its states. With
// an IMU, though, GLIM already estimates the motion BETWEEN those states and then
// throws it away: glim::SubMapping::insert_frame integrates the IMU across each
// [frame, next frame] interval, pins both ends to the odometry states with tight
// priors, LM-optimizes the chain, and stores the result in
// EstimationFrame::imu_rate_trajectory. Those poses are not dead reckoning — they
// are a smoothed, both-ends-constrained interpolation of the measured motion.
//
// This module turns that per-frame chain into extra trajectory rows:
//
//   1. reanchor_chain() moves one chain from GLIM's odometry world frame onto the
//      globally-optimized poses, spreading the endpoint disagreement across the
//      interval so the result stays continuous across scan boundaries.
//   2. upsample_trajectory() resamples the re-anchored chains onto a uniform grid
//      and unions that with the optimized poses.
//
// It is deliberately GLIM-free and Eigen-only: the caller (cloud_mapper.cpp) is
// what knows about EstimationFrame and SubMap, so the geometry here can be
// unit-tested with synthetic chains and no SLAM stack.
//
// Stamps are integer nanoseconds throughout. warmup_fill.hpp's TimedPose carries
// double seconds, which is fine for a ~1 s propagation window but cannot express
// an exactly reproducible resample grid over a whole run.
namespace bagwiz::core::slam
{

// One knot of GLIM's per-frame IMU-rate chain: a world<-IMU pose in the ODOMETRY
// world frame (the frame EstimationFrame::imu_rate_trajectory is expressed in),
// which is NOT the globally-optimized world frame. reanchor_chain bridges them.
struct StampedImuPose
{
  std::int64_t stamp_ns = 0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
};

// One trajectory sample: a world<-LiDAR pose in the globally-optimized world
// frame — the frame traj.tum is written in.
struct StampedPose
{
  std::int64_t stamp_ns = 0;
  Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
};

// One frame's re-anchored IMU-rate chain: ascending by stamp, spanning
// [frame stamp, next frame stamp]. Consecutive chains share their endpoint stamp.
struct ImuRateChain
{
  std::vector<StampedPose> poses;
};

// Move `raw` — GLIM's IMU-rate chain for one frame, in the odometry world frame,
// ascending by stamp — onto the globally-optimized trajectory.
//
// The chain is treated as RELATIVE motion anchored at its own first knot, so the
// odometry-vs-optimized frame difference cancels:
//
//   delta(t)         = raw.front()^-1 * raw(t)
//   T_world_imu(t)   = (T_world_lidar_begin * T_lidar_imu) * delta(t)
//   T_world_lidar(t) = T_world_imu(t) * T_lidar_imu^-1
//
// That alone lands the last knot on `T_world_lidar_begin` composed with odometry
// dead reckoning, which disagrees with the next frame's optimized pose by
// whatever the sub/global optimization corrected (millimetres inside a submap,
// more across a submap boundary or a loop closure). When `T_world_lidar_end` is
// given the disagreement is spread across the interval — translation linearly,
// rotation by SLERP from identity — so the chain leaves `T_world_lidar_begin`
// exactly and arrives at `T_world_lidar_end` exactly, with no jump at either scan
// boundary. Pass std::nullopt when there is no next optimized pose (the last
// frame of a run); the chain is then only re-anchored.
//
// Returns an empty chain when `raw` has fewer than two knots (nothing to
// resample). A chain whose knots all share one stamp is re-anchored without the
// blend, whose fraction would be undefined.
ImuRateChain reanchor_chain(
  std::span<const StampedImuPose> raw, const Eigen::Isometry3d & T_world_lidar_begin,
  const std::optional<Eigen::Isometry3d> & T_world_lidar_end,
  const Eigen::Isometry3d & T_lidar_imu);

// What upsample_trajectory added, for the run summary.
struct UpsampleStats
{
  std::size_t grid_poses = 0;      // rows the grid contributed (stamp collisions excluded)
  std::size_t uncovered_gaps = 0;  // contiguous runs of grid stamps no chain covered
};

// Resample onto a uniform `period_ns` grid and union that with `optimized`.
//
// The grid is phased on `optimized.front().stamp_ns` (t0 + k * period_ns, integer
// arithmetic) and runs to `optimized.back().stamp_ns`, so identical inputs always
// produce identical stamps. `optimized` survives verbatim: its rows are the only
// ones an optimizer actually solved for, so on a stamp collision the optimized
// pose wins and the grid row is dropped. "Collision" is a small window, not an
// exact match: the optimized stamps themselves carry a few hundred nanoseconds
// of noise (see kStampCoalesceNs in the .cpp), and a grid sample landing inside
// that noise would be a duplicate row rather than a new sample.
//
// A grid stamp is emitted only where some chain covers it. Spans with no chain —
// the warmup/cooldown fill windows, IMU dropouts — keep their input density
// instead of being interpolated from the optimized poses alone: TUM has no
// per-row quality field, so solved rows and plain interpolation would mix
// invisibly.
//
// `optimized` must be ascending by stamp; `chains` may arrive in any order.
// Returns `optimized` unchanged when `period_ns <= 0` or it holds under two poses.
std::vector<StampedPose> upsample_trajectory(
  std::span<const StampedPose> optimized, std::span<const ImuRateChain> chains,
  std::int64_t period_ns, UpsampleStats & stats);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__TRAJ_UPSAMPLE_HPP_
