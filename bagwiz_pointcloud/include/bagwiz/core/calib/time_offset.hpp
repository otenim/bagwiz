// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__TIME_OFFSET_HPP_
#define BAGWIZ__CORE__CALIB__TIME_OFFSET_HPP_

#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

// Clock-offset estimation between a rotation-measuring sensor (a camera, via
// frame-to-frame rotations) and a time reference (the --pose trajectory, or a
// gyro). The problem: the sensor's stamps run a constant `d` apart from the
// reference clock, so the rotation the sensor measured over its stamped window
// [t0, t1] happened over [t0 + d, t1 + d] on the reference clock. The estimate
// is the `d` at which the reference's rotation over the shifted windows best
// matches the measured rotations — the value a caller adds to every sensor
// stamp to place it on the reference clock. Eigen/OpenCV-free like the rest
// of this package; rotations are rotation vectors (axis * angle, radians).
namespace bagwiz::core::calib
{

// One measured rotation: the body rotated by `rotvec` (expressed in the body
// frame at t0) between the sensor stamps t0 and t1. Every interval must have
// t1 > t0; the rotation is small enough (a frame period) that rotation-vector
// differences are a valid residual.
struct RotationInterval
{
  std::int64_t t0_ns = 0;
  std::int64_t t1_ns = 0;
  std::array<double, 3> rotvec{};
};

// One gyro sample: angular velocity of the body in the body frame (rad/s) at
// the gyro's stamp. Samples must be sorted ascending; a caller rotates them
// into the body frame the RotationIntervals are expressed in before building
// the integral.
struct GyroSample
{
  std::int64_t stamp_ns = 0;
  std::array<double, 3> omega{};
};

// The reference's rotation over [a, b] (rotation vector in the body frame at
// a), or nullopt when [a, b] is not covered by the reference. Both providers
// below produce this shape so fit_time_offset does not care which reference
// it is matched against.
using RotationOverInterval =
  std::function<std::optional<std::array<double, 3>>(std::int64_t a_ns, std::int64_t b_ns)>;

// Cumulative trapezoid integral of a gyro stream; rotation_over(a, b) is
// Theta(b) - Theta(a) with Theta linearly interpolated between samples. The
// small-angle composition is exact enough over one frame period (the
// commutation error is second order in the swept angle). nullopt outside the
// sample span.
class GyroIntegral
{
public:
  explicit GyroIntegral(std::span<const GyroSample> samples);
  [[nodiscard]] std::optional<std::array<double, 3>> rotation_over(
    std::int64_t a_ns, std::int64_t b_ns) const;
  [[nodiscard]] std::size_t size() const noexcept { return stamps_ns_.size(); }
  [[nodiscard]] RotationOverInterval provider() const;

private:
  std::vector<std::int64_t> stamps_ns_;
  std::vector<std::array<double, 3>> theta_;  // integral up to each stamp
  [[nodiscard]] std::array<double, 3> theta_at(std::int64_t t_ns) const;
};

// The trajectory's rotation over [a, b]: the relative rotation between the
// poses interpolated at a and b (the same lerp/slerp the rest of calib uses),
// expressed in the body frame at a. nullopt when either stamp falls outside
// the trajectory's span — no extrapolation, so a shift that carries a window
// past the ends drops that window rather than inventing motion. `poses` must
// be sorted ascending and outlive the returned provider.
[[nodiscard]] RotationOverInterval trajectory_rotation_provider(
  std::span<const TrajectoryPose> poses);

// The trajectory's own consecutive-pose rotations as RotationIntervals, for
// matching the trajectory against a gyro: one interval per adjacent pose pair
// whose spacing is within (0, max_dt_ns]. Rotation in the body frame of the
// earlier pose.
[[nodiscard]] std::vector<RotationInterval> trajectory_rotation_intervals(
  std::span<const TrajectoryPose> poses, std::int64_t max_dt_ns);

struct TimeOffsetParams
{
  std::int64_t grid_half_span_ns = 250'000'000;  // search d in [-span, +span]
  std::int64_t grid_step_ns = 1'000'000;
  int bootstrap_draws = 200;  // 0 disables the bootstrap (std_ns = 0)
  std::size_t min_intervals = 50;
  // Below this rms rotation per interval (radians) the data does not move
  // enough to time: a static platform reads as "no signal" rather than as an
  // offset of 0.
  double min_signal_rms_rad = 5e-4;
  // A bootstrap spread above this is reported as a failure: the estimate is
  // not good enough to apply silently.
  std::int64_t max_std_ns = 20'000'000;
  std::uint64_t seed = 0;
};

struct TimeOffsetFit
{
  std::int64_t offset_ns = 0;
  std::int64_t std_ns = 0;  // bootstrap spread; 0 when the bootstrap is off
  std::size_t intervals = 0;
  double signal_rms_rad = 0.0;           // rms |measured rotvec| over the intervals
  double residual_rms_before_rad = 0.0;  // at d = 0
  double residual_rms_after_rad = 0.0;   // at d = offset
  bool on_grid_edge = false;
};

struct TimeOffsetResult
{
  bool ok = false;
  std::string error;  // why not ok: too few intervals, no signal, grid edge, spread
  TimeOffsetFit fit;
};

// Fit the offset d that best explains `measured` against `reference`: a
// robust (Huber) sum over intervals of |measured.rotvec - reference(t0 + d,
// t1 + d)| minimized over the grid, refined with a parabola through the
// minimum, with a bootstrap over intervals for the spread. Intervals the
// reference cannot cover at a given d do not contribute at that d, which is
// why the result checks the count at the minimum and the grid edge. Fails (ok
// = false, error set) on too few intervals, too little signal, a minimum on
// the grid edge, or a spread above max_std_ns.
[[nodiscard]] TimeOffsetResult fit_time_offset(
  std::span<const RotationInterval> measured, const RotationOverInterval & reference,
  const TimeOffsetParams & params);

// The gyro-bridged estimate: the sensor's offset to the gyro clock (measured
// intervals vs the gyro) minus the trajectory's offset to the gyro clock
// (trajectory intervals vs the gyro) is the sensor's offset to the trajectory
// clock, with the gyro's own latency cancelling in the difference. Both fits
// must succeed; the combined spread is the root sum of squares.
struct GyroBridgedOffset
{
  TimeOffsetResult combined;  // offset = sensor_gyro - trajectory_gyro
  TimeOffsetResult sensor_gyro;
  TimeOffsetResult trajectory_gyro;
};
[[nodiscard]] GyroBridgedOffset fit_time_offset_via_gyro(
  std::span<const RotationInterval> measured, std::span<const RotationInterval> trajectory,
  const GyroIntegral & gyro, const TimeOffsetParams & params);

// Rotation vector of the relative rotation q_a^-1 * q_b (body frame at a),
// from two normalized (x, y, z, w) quaternions. Exposed for the callers that
// turn their own pose pairs into RotationIntervals.
[[nodiscard]] std::array<double, 3> relative_rotation_vector(
  const std::array<double, 4> & q_a, const std::array<double, 4> & q_b);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__TIME_OFFSET_HPP_
