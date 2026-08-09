// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/traj_upsample.hpp"

#include <algorithm>
#include <iterator>
#include <map>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

namespace
{

// How close a grid stamp may come to an existing pose before it is dropped as a
// duplicate of it: 1% of the sampling interval, floored at kStampNoiseNs and
// capped at a quarter period so a very fine grid is never coalesced away.
//
// A grid phased on the first pose does not land exactly on the later optimized
// stamps, for two reasons. The small one is representation: a trajectory stamp
// round-trips through GLIM's EstimationFrame::stamp, a double holding seconds
// since the epoch, whose ULP at ~1e9 s is already ~240 ns. The large one is the
// sensor: real scan stamps jitter around their nominal period by tens of
// microseconds (a driving Autoware bag measures +-12 us on a 100 ms LiDAR). Both
// leave a grid sample a hair from a solved pose, which is a duplicate row rather
// than a new sample. Tying the window to the requested interval keeps "the same
// instant" meaningful at any rate; the floor keeps it above the representation
// noise when the interval itself is tiny.
constexpr std::int64_t kStampNoiseNs = 1'000;

std::int64_t coalesce_window_ns(std::int64_t period_ns)
{
  return std::min(std::max(kStampNoiseNs, period_ns / 100), period_ns / 4);
}

// Interpolate `chain` at `stamp_ns`: translation linearly, rotation by
// shortest-path SLERP (Eigen's slerp already picks the shorter arc). The caller
// guarantees `chain` is ascending, holds at least two knots, and spans the stamp.
Eigen::Isometry3d interpolate_chain(std::span<const StampedPose> chain, std::int64_t stamp_ns)
{
  const auto upper = std::upper_bound(
    chain.begin(), chain.end(), stamp_ns,
    [](std::int64_t stamp, const StampedPose & pose) { return stamp < pose.stamp_ns; });
  if (upper == chain.begin()) {
    return chain.front().T_world_lidar;
  }
  const auto lower = std::prev(upper);
  if (upper == chain.end() || lower->stamp_ns == stamp_ns) {
    return lower->T_world_lidar;  // exact hit, or the trailing knot
  }

  const auto span = static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  const double t = static_cast<double>(stamp_ns - lower->stamp_ns) / span;

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = lower->T_world_lidar.translation() +
                      t * (upper->T_world_lidar.translation() - lower->T_world_lidar.translation());
  const Eigen::Quaterniond from(lower->T_world_lidar.rotation());
  const Eigen::Quaterniond to(upper->T_world_lidar.rotation());
  out.linear() = from.slerp(t, to).normalized().toRotationMatrix();
  return out;
}

}  // namespace

ImuRateChain reanchor_chain(
  std::span<const StampedImuPose> raw, const Eigen::Isometry3d & T_world_lidar_begin,
  const std::optional<Eigen::Isometry3d> & T_world_lidar_end, const Eigen::Isometry3d & T_lidar_imu)
{
  ImuRateChain chain;
  if (raw.size() < 2) {
    return chain;  // a single knot carries no motion to resample
  }

  const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();
  const Eigen::Isometry3d anchor_world_imu = T_world_lidar_begin * T_lidar_imu;
  const Eigen::Isometry3d raw_anchor_inverse = raw.front().T_world_imu.inverse();

  chain.poses.reserve(raw.size());
  for (const auto & knot : raw) {
    const Eigen::Isometry3d delta = raw_anchor_inverse * knot.T_world_imu;
    chain.poses.push_back({knot.stamp_ns, anchor_world_imu * delta * T_imu_lidar});
  }

  const std::int64_t begin_ns = chain.poses.front().stamp_ns;
  const std::int64_t span_ns = chain.poses.back().stamp_ns - begin_ns;
  if (!T_world_lidar_end.has_value() || span_ns <= 0) {
    return chain;
  }

  // How far dead reckoning from the (optimized) anchor ends up from the optimized
  // next pose. Copied, not referenced: the loop below rewrites the last knot.
  const Eigen::Isometry3d free_run_end = chain.poses.back().T_world_lidar;
  const Eigen::Vector3d translation_residual =
    T_world_lidar_end->translation() - free_run_end.translation();
  const Eigen::Quaterniond rotation_residual =
    Eigen::Quaterniond(free_run_end.rotation()).conjugate() *
    Eigen::Quaterniond(T_world_lidar_end->rotation());
  const Eigen::Quaterniond no_rotation = Eigen::Quaterniond::Identity();

  for (auto & pose : chain.poses) {
    const double fraction =
      static_cast<double>(pose.stamp_ns - begin_ns) / static_cast<double>(span_ns);
    const Eigen::Quaterniond rotated = Eigen::Quaterniond(pose.T_world_lidar.rotation()) *
                                       no_rotation.slerp(fraction, rotation_residual);
    pose.T_world_lidar.linear() = rotated.normalized().toRotationMatrix();
    pose.T_world_lidar.translation() += fraction * translation_residual;
  }
  return chain;
}

std::vector<StampedPose> upsample_trajectory(
  std::span<const StampedPose> optimized, std::span<const ImuRateChain> chains,
  std::int64_t period_ns, UpsampleStats & stats)
{
  stats = UpsampleStats{};

  std::vector<StampedPose> out(optimized.begin(), optimized.end());
  if (period_ns <= 0 || optimized.size() < 2) {
    return out;
  }

  // Ascending by first stamp so the grid walk below can advance one cursor
  // monotonically instead of searching every chain per grid stamp.
  std::vector<const ImuRateChain *> ordered;
  ordered.reserve(chains.size());
  for (const auto & chain : chains) {
    if (chain.poses.size() >= 2) {
      ordered.push_back(&chain);
    }
  }
  std::sort(ordered.begin(), ordered.end(), [](const ImuRateChain * a, const ImuRateChain * b) {
    return a->poses.front().stamp_ns < b->poses.front().stamp_ns;
  });

  // Keyed by stamp so the output comes out time-ordered and the optimized poses,
  // inserted first, win every collision with the grid.
  std::map<std::int64_t, StampedPose> merged;
  for (const auto & pose : optimized) {
    merged[pose.stamp_ns] = pose;
  }

  const std::int64_t begin_ns = optimized.front().stamp_ns;
  const std::int64_t end_ns = optimized.back().stamp_ns;
  const std::int64_t coalesce_ns = coalesce_window_ns(period_ns);
  std::size_t cursor = 0;
  bool in_gap = false;

  // Phased on begin_ns and stepped in integer nanoseconds: the same inputs must
  // always yield the same stamps. The bound is written as a subtraction so the
  // accumulator can never step past end_ns.
  for (std::int64_t stamp_ns = begin_ns; stamp_ns <= end_ns - period_ns;) {
    stamp_ns += period_ns;

    while (cursor < ordered.size() && ordered[cursor]->poses.back().stamp_ns < stamp_ns) {
      ++cursor;
    }
    const bool covered =
      cursor < ordered.size() && ordered[cursor]->poses.front().stamp_ns <= stamp_ns;
    if (!covered) {
      if (!in_gap) {
        ++stats.uncovered_gaps;
        in_gap = true;
      }
      continue;
    }
    in_gap = false;

    // Never emit a row a hair away from one already there; the existing pose
    // (an optimized one, or the previous grid sample) already covers the instant.
    const auto next = merged.lower_bound(stamp_ns);
    const bool duplicates_next = next != merged.end() && next->first - stamp_ns <= coalesce_ns;
    const bool duplicates_previous =
      next != merged.begin() && stamp_ns - std::prev(next)->first <= coalesce_ns;
    if (duplicates_next || duplicates_previous) {
      continue;
    }

    merged.emplace_hint(
      next, stamp_ns, StampedPose{stamp_ns, interpolate_chain(ordered[cursor]->poses, stamp_ns)});
    ++stats.grid_poses;
  }

  out.clear();
  out.reserve(merged.size());
  for (auto & entry : merged) {
    out.push_back(std::move(entry.second));
  }
  return out;
}

}  // namespace bagwiz::core::slam
