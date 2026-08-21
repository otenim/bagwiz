// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/time_offset.hpp"

#include "bagwiz/core/tf/trajectory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace calib = bagwiz::core::calib;
using bagwiz::core::TrajectoryPose;

namespace
{

constexpr std::int64_t kSec = 1'000'000'000LL;
constexpr std::int64_t kMs = 1'000'000LL;

// A smooth, three-axis angular velocity with enough spectral content to time
// against: sums of sinusoids, the strongest on z (yaw) like a turning
// vehicle, smaller pitch/roll wobble. rad/s.
std::array<double, 3> omega_at(double t)
{
  return {
    0.08 * std::sin(2.0 * M_PI * 0.7 * t + 0.3) + 0.03 * std::sin(2.0 * M_PI * 2.1 * t),
    0.06 * std::sin(2.0 * M_PI * 0.45 * t + 1.1) + 0.02 * std::cos(2.0 * M_PI * 1.7 * t),
    0.30 * std::sin(2.0 * M_PI * 0.11 * t) + 0.10 * std::sin(2.0 * M_PI * 0.9 * t + 0.7),
  };
}

// Unit quaternion (x, y, z, w) integrated from omega_at with a fine step, so
// the world holds one consistent orientation history every synthetic stream
// is read from. q(t) for t in [0, duration].
class World
{
public:
  explicit World(double duration_s, double step_s = 0.0005) : step_s_(step_s)
  {
    const auto n = static_cast<std::size_t>(duration_s / step_s) + 2;
    q_.reserve(n);
    std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};
    q_.push_back(q);
    for (std::size_t i = 1; i < n; ++i) {
      const double t = (static_cast<double>(i) - 0.5) * step_s;
      const auto w = omega_at(t);
      // q <- q * exp(w * dt / 2) (body-frame rate)
      const double ang = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * step_s;
      std::array<double, 4> dq{0.0, 0.0, 0.0, 1.0};
      if (ang > 1e-15) {
        const double s = std::sin(ang / 2.0) / (ang / step_s);
        dq = {w[0] * s, w[1] * s, w[2] * s, std::cos(ang / 2.0)};
      }
      q = mul(q, dq);
      const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      for (auto & c : q) {
        c /= norm;
      }
      q_.push_back(q);
    }
  }

  // Orientation at time t (seconds), nearest fine step.
  [[nodiscard]] std::array<double, 4> at(double t) const
  {
    const auto i = static_cast<std::size_t>(std::llround(t / step_s_));
    return q_[std::min(i, q_.size() - 1)];
  }

  [[nodiscard]] TrajectoryPose pose_at(std::int64_t stamp_ns, double true_time_s) const
  {
    const auto q = at(true_time_s);
    TrajectoryPose p;
    p.timestamp_ns = stamp_ns;
    p.qx = q[0];
    p.qy = q[1];
    p.qz = q[2];
    p.qw = q[3];
    return p;
  }

  static std::array<double, 4> mul(const std::array<double, 4> & a, const std::array<double, 4> & b)
  {
    return {
      a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
      a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
      a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
      a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
  }

private:
  double step_s_;
  std::vector<std::array<double, 4>> q_;
};

// Trajectory poses stamped on the true clock at `rate_hz` over [start, end].
std::vector<TrajectoryPose> make_trajectory(
  const World & world, double start_s, double end_s, double rate_hz)
{
  std::vector<TrajectoryPose> poses;
  for (double t = start_s; t <= end_s + 1e-9; t += 1.0 / rate_hz) {
    poses.push_back(world.pose_at(static_cast<std::int64_t>(std::llround(t * 1e9)), t));
  }
  return poses;
}

// "Camera" rotation intervals: frames stamped at `rate_hz` starting at
// `first_stamp_s`, each really captured `true_offset_s` later than its stamp
// (the stamp clock runs early by that much), with Gaussian noise on the
// measured rotation vector. Rotation between consecutive frames in the body
// frame at the earlier frame.
std::vector<calib::RotationInterval> make_sensor_intervals(
  const World & world, double first_stamp_s, double last_stamp_s, double rate_hz,
  double true_offset_s, double noise_rad, std::uint64_t seed)
{
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_rad);
  std::vector<calib::RotationInterval> out;
  double prev_stamp = first_stamp_s;
  for (double t = first_stamp_s + 1.0 / rate_hz; t <= last_stamp_s + 1e-9; t += 1.0 / rate_hz) {
    calib::RotationInterval iv;
    iv.t0_ns = static_cast<std::int64_t>(std::llround(prev_stamp * 1e9));
    iv.t1_ns = static_cast<std::int64_t>(std::llround(t * 1e9));
    iv.rotvec = calib::relative_rotation_vector(
      world.at(prev_stamp + true_offset_s), world.at(t + true_offset_s));
    for (auto & c : iv.rotvec) {
      c += noise(rng);
    }
    out.push_back(iv);
    prev_stamp = t;
  }
  return out;
}

// Gyro samples stamped `latency_s` LATER than the true time of the reading
// (software stamping at arrival), at `rate_hz`.
std::vector<calib::GyroSample> make_gyro(
  double start_s, double end_s, double rate_hz, double latency_s, double noise_rad_s,
  std::uint64_t seed)
{
  std::mt19937_64 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_rad_s);
  std::vector<calib::GyroSample> out;
  for (double t = start_s; t <= end_s + 1e-9; t += 1.0 / rate_hz) {
    calib::GyroSample s;
    s.stamp_ns = static_cast<std::int64_t>(std::llround((t + latency_s) * 1e9));
    s.omega = omega_at(t);
    for (auto & c : s.omega) {
      c += noise(rng);
    }
    out.push_back(s);
  }
  return out;
}

}  // namespace

TEST(TimeOffsetTest, RelativeRotationVectorIsBodyFrameLog)
{
  // 10 deg about z then the same again: q_a^-1 q_b is the second 10 deg, in
  // the body frame.
  const double h = 5.0 * M_PI / 180.0;  // half angle
  const std::array<double, 4> qa{0.0, 0.0, std::sin(h), std::cos(h)};
  const std::array<double, 4> qb{0.0, 0.0, std::sin(2 * h), std::cos(2 * h)};
  const auto r = calib::relative_rotation_vector(qa, qb);
  EXPECT_NEAR(r[0], 0.0, 1e-12);
  EXPECT_NEAR(r[1], 0.0, 1e-12);
  EXPECT_NEAR(r[2], 2 * h, 1e-12);
  // Opposite order: the inverse rotation.
  const auto rr = calib::relative_rotation_vector(qb, qa);
  EXPECT_NEAR(rr[2], -2 * h, 1e-12);
  // Sign-flipped (q and -q are the same rotation) must give the same vector.
  const std::array<double, 4> qb_neg{-qb[0], -qb[1], -qb[2], -qb[3]};
  const auto rn = calib::relative_rotation_vector(qa, qb_neg);
  EXPECT_NEAR(rn[2], 2 * h, 1e-12);
}

TEST(TimeOffsetTest, GyroIntegralMatchesOrientationChange)
{
  const World world(10.0);
  const auto gyro = make_gyro(0.0, 10.0, 400.0, 0.0, 0.0, 1);
  const calib::GyroIntegral integral(gyro);
  ASSERT_EQ(integral.size(), gyro.size());
  // Over 100 ms windows the trapezoid integral of omega matches the
  // quaternion log of the true orientation change to well under a mrad.
  for (double t = 1.0; t < 9.0; t += 0.37) {
    const auto got = integral.rotation_over(
      static_cast<std::int64_t>(std::llround(t * 1e9)),
      static_cast<std::int64_t>(std::llround((t + 0.1) * 1e9)));
    ASSERT_TRUE(got.has_value());
    const auto want = calib::relative_rotation_vector(world.at(t), world.at(t + 0.1));
    for (std::size_t k = 0; k < 3; ++k) {
      EXPECT_NEAR((*got)[k], want[k], 2e-4) << "t=" << t << " axis " << k;
    }
  }
  // Outside the span: nullopt, not an extrapolation.
  EXPECT_FALSE(integral.rotation_over(-1 * kSec, 0).has_value());
  EXPECT_FALSE(integral.rotation_over(9 * kSec, 11 * kSec).has_value());
}

TEST(TimeOffsetTest, TrajectoryProviderInterpolatesAndRefusesOutOfSpan)
{
  const World world(10.0);
  const auto poses = make_trajectory(world, 0.0, 10.0, 10.0);
  const auto provider = calib::trajectory_rotation_provider(poses);
  // A window aligned with the poses is the exact relative rotation.
  const auto exact = provider(2 * kSec, 2 * kSec + 100 * kMs);
  ASSERT_TRUE(exact.has_value());
  const auto want = calib::relative_rotation_vector(world.at(2.0), world.at(2.1));
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR((*exact)[k], want[k], 1e-9);
  }
  // A window between poses follows the slerp: close to the truth at this
  // smoothness (the slerp error over 100 ms is far below the fit's scale).
  const auto mid = provider(2 * kSec + 37 * kMs, 2 * kSec + 137 * kMs);
  ASSERT_TRUE(mid.has_value());
  const auto want_mid = calib::relative_rotation_vector(world.at(2.037), world.at(2.137));
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR((*mid)[k], want_mid[k], 5e-4);
  }
  EXPECT_FALSE(provider(-100 * kMs, 0).has_value());
  EXPECT_FALSE(provider(9950 * kMs, 10050 * kMs).has_value());
}

TEST(TimeOffsetTest, TrajectoryIntervalsAreConsecutivePairsWithinMaxDt)
{
  const World world(3.0);
  auto poses = make_trajectory(world, 0.0, 3.0, 10.0);
  // Open a 1 s gap: intervals across it are dropped at max_dt = 0.5 s.
  poses.erase(poses.begin() + 10, poses.begin() + 19);
  const auto intervals = calib::trajectory_rotation_intervals(poses, 500 * kMs);
  EXPECT_EQ(intervals.size(), poses.size() - 2);
  for (const auto & iv : intervals) {
    EXPECT_LT(iv.t1_ns - iv.t0_ns, 500 * kMs);
    EXPECT_GT(iv.t1_ns, iv.t0_ns);
  }
}

TEST(TimeOffsetTest, RecoversOffsetAgainstTrajectory)
{
  const World world(60.0);
  const auto poses = make_trajectory(world, 0.0, 60.0, 10.0);
  // Frames at 10 Hz, phase-shifted from the poses, stamped 43 ms early
  // (captured at stamp + 43 ms), with 1 mrad of measurement noise.
  const double true_offset = 0.043;
  const auto measured = make_sensor_intervals(world, 3.05, 57.0, 10.0, true_offset, 1e-3, 7);
  calib::TimeOffsetParams params;
  const auto result =
    calib::fit_time_offset(measured, calib::trajectory_rotation_provider(poses), params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(static_cast<double>(result.fit.offset_ns) / 1e9, true_offset, 0.004);
  EXPECT_GT(result.fit.intervals, 500U);
  EXPECT_LT(result.fit.residual_rms_after_rad, result.fit.residual_rms_before_rad);
  EXPECT_GT(result.fit.std_ns, 0);
  EXPECT_LT(result.fit.std_ns, 10 * kMs);
  EXPECT_FALSE(result.fit.on_grid_edge);
}

TEST(TimeOffsetTest, RecoversNegativeOffsetAgainstTrajectory)
{
  const World world(60.0);
  const auto poses = make_trajectory(world, 0.0, 60.0, 10.0);
  const double true_offset = -0.120;
  const auto measured = make_sensor_intervals(world, 3.0, 57.0, 10.0, true_offset, 1e-3, 11);
  const auto result = calib::fit_time_offset(
    measured, calib::trajectory_rotation_provider(poses), calib::TimeOffsetParams{});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(static_cast<double>(result.fit.offset_ns) / 1e9, true_offset, 0.004);
}

TEST(TimeOffsetTest, GyroBridgeCancelsGyroLatency)
{
  const World world(60.0);
  const auto poses = make_trajectory(world, 0.0, 60.0, 10.0);
  const double true_offset = -0.042;  // sensor stamps vs the trajectory clock
  const double gyro_latency = 0.065;  // gyro stamps late by this much
  const auto measured = make_sensor_intervals(world, 3.05, 57.0, 10.0, true_offset, 1e-3, 3);
  const auto gyro = make_gyro(0.0, 60.0, 200.0, gyro_latency, 2e-3, 5);
  const calib::GyroIntegral integral(gyro);
  const auto traj_intervals = calib::trajectory_rotation_intervals(poses, 500 * kMs);
  const auto bridged =
    calib::fit_time_offset_via_gyro(measured, traj_intervals, integral, calib::TimeOffsetParams{});
  ASSERT_TRUE(bridged.sensor_gyro.ok) << bridged.sensor_gyro.error;
  ASSERT_TRUE(bridged.trajectory_gyro.ok) << bridged.trajectory_gyro.error;
  ASSERT_TRUE(bridged.combined.ok) << bridged.combined.error;
  // sensor vs gyro: true_offset + latency; trajectory vs gyro: latency.
  EXPECT_NEAR(
    static_cast<double>(bridged.sensor_gyro.fit.offset_ns) / 1e9, true_offset + gyro_latency,
    0.004);
  EXPECT_NEAR(
    static_cast<double>(bridged.trajectory_gyro.fit.offset_ns) / 1e9, gyro_latency, 0.004);
  EXPECT_NEAR(static_cast<double>(bridged.combined.fit.offset_ns) / 1e9, true_offset, 0.005);
  // The combined spread is the root sum of squares of the two.
  const double expect_std = std::hypot(
    static_cast<double>(bridged.sensor_gyro.fit.std_ns),
    static_cast<double>(bridged.trajectory_gyro.fit.std_ns));
  EXPECT_NEAR(static_cast<double>(bridged.combined.fit.std_ns), expect_std, 2.0);
}

TEST(TimeOffsetTest, FailsOnTooFewIntervals)
{
  const World world(10.0);
  const auto poses = make_trajectory(world, 0.0, 10.0, 10.0);
  const auto measured = make_sensor_intervals(world, 3.0, 5.0, 10.0, 0.02, 1e-3, 1);  // ~20
  const auto result = calib::fit_time_offset(
    measured, calib::trajectory_rotation_provider(poses), calib::TimeOffsetParams{});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("interval"), std::string::npos) << result.error;
}

TEST(TimeOffsetTest, FailsWithoutRotationSignal)
{
  // A static platform: identity poses and zero measured rotation (plus a
  // little noise) is "no signal", not "offset 0".
  std::vector<TrajectoryPose> poses;
  for (int i = 0; i <= 600; ++i) {
    TrajectoryPose p;
    p.timestamp_ns = static_cast<std::int64_t>(i) * 100 * kMs;
    p.qw = 1.0;
    poses.push_back(p);
  }
  std::mt19937_64 rng(9);
  std::normal_distribution<double> noise(0.0, 2e-5);
  std::vector<calib::RotationInterval> measured;
  for (int i = 30; i < 570; ++i) {
    calib::RotationInterval iv;
    iv.t0_ns = static_cast<std::int64_t>(i) * 100 * kMs + 37 * kMs;
    iv.t1_ns = iv.t0_ns + 100 * kMs;
    iv.rotvec = {noise(rng), noise(rng), noise(rng)};
    measured.push_back(iv);
  }
  const auto result = calib::fit_time_offset(
    measured, calib::trajectory_rotation_provider(poses), calib::TimeOffsetParams{});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("rotation"), std::string::npos) << result.error;
}

TEST(TimeOffsetTest, FailsWhenMinimumSitsOnTheGridEdge)
{
  const World world(60.0);
  const auto poses = make_trajectory(world, 0.0, 60.0, 10.0);
  // A 400 ms offset with a +-250 ms grid: the best grid point is the edge.
  const auto measured = make_sensor_intervals(world, 3.0, 57.0, 10.0, 0.400, 1e-3, 2);
  const auto result = calib::fit_time_offset(
    measured, calib::trajectory_rotation_provider(poses), calib::TimeOffsetParams{});
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("edge"), std::string::npos) << result.error;
}

TEST(TimeOffsetTest, FailsWhenTheSpreadIsTooWide)
{
  const World world(60.0);
  const auto poses = make_trajectory(world, 0.0, 60.0, 10.0);
  // Noise far above the signal (30 mrad vs a few mrad per interval): the
  // bootstrap spread blows past max_std_ns.
  const auto measured = make_sensor_intervals(world, 3.0, 57.0, 10.0, 0.02, 3e-2, 4);
  calib::TimeOffsetParams params;
  params.max_std_ns = 5 * kMs;
  const auto result =
    calib::fit_time_offset(measured, calib::trajectory_rotation_provider(poses), params);
  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("spread"), std::string::npos) << result.error;
}
