// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/time_offset.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::calib
{
namespace
{

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // x, y, z, w

double norm3(const Vec3 & v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Quat quat_normalized(Quat q)
{
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 0.0) {
    for (auto & c : q) {
      c /= n;
    }
  } else {
    q = {0.0, 0.0, 0.0, 1.0};
  }
  return q;
}

// a * b (Hamilton).
Quat quat_mul(const Quat & a, const Quat & b)
{
  return {
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

// Rotation vector of a unit quaternion: axis * angle, angle in [0, pi].
Vec3 quat_log(Quat q)
{
  if (q[3] < 0.0) {  // q and -q are one rotation; keep the short arc
    for (auto & c : q) {
      c = -c;
    }
  }
  const double sin_half = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
  const double angle = 2.0 * std::atan2(sin_half, q[3]);
  if (sin_half < 1e-12) {
    return {2.0 * q[0], 2.0 * q[1], 2.0 * q[2]};  // small-angle limit
  }
  const double k = angle / sin_half;
  return {q[0] * k, q[1] * k, q[2] * k};
}

Quat quat_of(const TrajectoryPose & p)
{
  return quat_normalized({p.qx, p.qy, p.qz, p.qw});
}

// Huber loss with scale c.
double huber(double r, double c)
{
  return r < c ? 0.5 * r * r : c * (r - 0.5 * c);
}

double median_of(std::vector<double> v)
{
  if (v.empty()) {
    return 0.0;
  }
  const std::size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
  return v[mid];
}

// The minimum of a cost curve over the valid grid indices, refined with a
// parabola through the minimum and its two neighbours (falls back to the
// grid point when a neighbour is invalid or the curvature is not positive).
// Returns {offset_ns, grid index, on_edge}.
struct GridMinimum
{
  double offset_ns = 0.0;
  std::size_t index = 0;
  bool on_edge = false;
  bool found = false;
};

GridMinimum grid_minimum(
  const std::vector<double> & cost, const std::vector<bool> & valid,
  const std::vector<std::int64_t> & grid_ns)
{
  GridMinimum out;
  double best = std::numeric_limits<double>::infinity();
  std::size_t first_valid = grid_ns.size();
  std::size_t last_valid = 0;
  for (std::size_t i = 0; i < grid_ns.size(); ++i) {
    if (!valid[i]) {
      continue;
    }
    first_valid = std::min(first_valid, i);
    last_valid = i;
    if (cost[i] < best) {
      best = cost[i];
      out.index = i;
      out.found = true;
    }
  }
  if (!out.found) {
    return out;
  }
  out.on_edge = out.index == first_valid || out.index == last_valid;
  out.offset_ns = static_cast<double>(grid_ns[out.index]);
  if (!out.on_edge && valid[out.index - 1] && valid[out.index + 1]) {
    const double y0 = cost[out.index - 1];
    const double y1 = cost[out.index];
    const double y2 = cost[out.index + 1];
    const double den = y0 - 2.0 * y1 + y2;
    if (den > 0.0) {
      const double frac = 0.5 * (y0 - y2) / den;  // in [-1, 1] for a true minimum
      const double step = static_cast<double>(grid_ns[out.index + 1] - grid_ns[out.index]);
      out.offset_ns += std::clamp(frac, -1.0, 1.0) * step;
    }
  }
  return out;
}

}  // namespace

std::array<double, 3> relative_rotation_vector(
  const std::array<double, 4> & q_a, const std::array<double, 4> & q_b)
{
  const Quat a = quat_normalized(q_a);
  const Quat b = quat_normalized(q_b);
  const Quat a_inv{-a[0], -a[1], -a[2], a[3]};
  return quat_log(quat_mul(a_inv, b));
}

// ---- GyroIntegral ------------------------------------------------------------

GyroIntegral::GyroIntegral(std::span<const GyroSample> samples)
{
  stamps_ns_.reserve(samples.size());
  theta_.reserve(samples.size());
  Vec3 theta{0.0, 0.0, 0.0};
  std::optional<std::size_t> last;  // the last sample kept (the integration's left end)
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (last.has_value()) {
      const double dt = static_cast<double>(samples[i].stamp_ns - samples[*last].stamp_ns) / 1e9;
      if (dt <= 0.0) {
        continue;  // a duplicate or reordered stamp is dropped outright: it carries no
                   // integration step and would break the monotonic stamp search
      }
      for (std::size_t k = 0; k < 3; ++k) {
        theta[k] += 0.5 * (samples[i].omega[k] + samples[*last].omega[k]) * dt;
      }
    }
    last = i;
    stamps_ns_.push_back(samples[i].stamp_ns);
    theta_.push_back(theta);
  }
}

std::array<double, 3> GyroIntegral::theta_at(std::int64_t t_ns) const
{
  // Precondition: stamps_ns_.front() <= t_ns <= stamps_ns_.back(), size >= 2.
  const auto it = std::lower_bound(stamps_ns_.begin(), stamps_ns_.end(), t_ns);
  if (it == stamps_ns_.begin()) {
    return theta_.front();
  }
  const auto hi = static_cast<std::size_t>(it - stamps_ns_.begin());
  if (hi >= stamps_ns_.size()) {
    return theta_.back();
  }
  const std::size_t lo = hi - 1;
  const double span = static_cast<double>(stamps_ns_[hi] - stamps_ns_[lo]);
  const double u = span > 0.0 ? static_cast<double>(t_ns - stamps_ns_[lo]) / span : 0.0;
  Vec3 out;
  for (std::size_t k = 0; k < 3; ++k) {
    out[k] = theta_[lo][k] + u * (theta_[hi][k] - theta_[lo][k]);
  }
  return out;
}

std::optional<std::array<double, 3>> GyroIntegral::rotation_over(
  std::int64_t a_ns, std::int64_t b_ns) const
{
  if (
    stamps_ns_.size() < 2 || a_ns < stamps_ns_.front() || b_ns > stamps_ns_.back() ||
    b_ns < stamps_ns_.front() || a_ns > stamps_ns_.back()) {
    return std::nullopt;
  }
  const Vec3 ta = theta_at(a_ns);
  const Vec3 tb = theta_at(b_ns);
  return Vec3{tb[0] - ta[0], tb[1] - ta[1], tb[2] - ta[2]};
}

RotationOverInterval GyroIntegral::provider() const
{
  return [this](std::int64_t a_ns, std::int64_t b_ns) { return rotation_over(a_ns, b_ns); };
}

// ---- trajectory reference ---------------------------------------------------

RotationOverInterval trajectory_rotation_provider(std::span<const TrajectoryPose> poses)
{
  std::vector<std::int64_t> stamps;
  stamps.reserve(poses.size());
  for (const auto & p : poses) {
    stamps.push_back(p.timestamp_ns);
  }
  // Orientation at t by lerp/slerp between the bracketing poses; nullopt
  // outside [first, last]. Captures the stamp vector by value and the pose
  // span by value (the caller keeps the poses alive).
  auto orientation_at = [poses, stamps = std::move(stamps)](std::int64_t t) -> std::optional<Quat> {
    if (stamps.size() < 2 || t < stamps.front() || t > stamps.back()) {
      return std::nullopt;
    }
    const auto it = std::lower_bound(stamps.begin(), stamps.end(), t);
    if (it == stamps.end()) {
      return quat_of(poses.back());  // t == stamps.back() is caught above; defensive
    }
    const auto hi = static_cast<std::size_t>(it - stamps.begin());
    if (stamps[hi] == t) {
      return quat_of(poses[hi]);
    }
    if (hi == 0) {
      return quat_of(poses.front());
    }
    const std::size_t lo = hi - 1;
    const double span = static_cast<double>(stamps[hi] - stamps[lo]);
    const double u = span > 0.0 ? static_cast<double>(t - stamps[lo]) / span : 0.0;
    return quat_of(interpolate_poses(poses[lo], poses[hi], u));
  };
  return [orientation_at](std::int64_t a_ns, std::int64_t b_ns) -> std::optional<Vec3> {
    const auto qa = orientation_at(a_ns);
    const auto qb = orientation_at(b_ns);
    if (!qa.has_value() || !qb.has_value()) {
      return std::nullopt;
    }
    return relative_rotation_vector(*qa, *qb);
  };
}

std::vector<RotationInterval> trajectory_rotation_intervals(
  std::span<const TrajectoryPose> poses, std::int64_t max_dt_ns)
{
  std::vector<RotationInterval> out;
  for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
    const std::int64_t dt = poses[i + 1].timestamp_ns - poses[i].timestamp_ns;
    if (dt <= 0 || dt > max_dt_ns) {
      continue;
    }
    RotationInterval iv;
    iv.t0_ns = poses[i].timestamp_ns;
    iv.t1_ns = poses[i + 1].timestamp_ns;
    iv.rotvec = relative_rotation_vector(quat_of(poses[i]), quat_of(poses[i + 1]));
    out.push_back(iv);
  }
  return out;
}

// ---- the fit -----------------------------------------------------------------

TimeOffsetResult fit_time_offset(
  std::span<const RotationInterval> measured, const RotationOverInterval & reference,
  const TimeOffsetParams & params)
{
  TimeOffsetResult result;
  if (measured.size() < params.min_intervals) {
    result.error = "only " + std::to_string(measured.size()) + " rotation interval(s); at least " +
                   std::to_string(params.min_intervals) + " are needed";
    return result;
  }
  double signal_sq = 0.0;
  for (const auto & iv : measured) {
    const double n = norm3(iv.rotvec);
    signal_sq += n * n;
  }
  result.fit.signal_rms_rad = std::sqrt(signal_sq / static_cast<double>(measured.size()));
  if (result.fit.signal_rms_rad < params.min_signal_rms_rad) {
    result.error = "too little rotation to time against (rms " +
                   std::to_string(result.fit.signal_rms_rad * 1e3) + " mrad per interval; " +
                   std::to_string(params.min_signal_rms_rad * 1e3) + " needed)";
    return result;
  }

  // The grid, and the residual norm of every interval at every grid point
  // (NaN where the reference does not cover the shifted window). Computed
  // once; the fit and the bootstrap both read it.
  std::vector<std::int64_t> grid;
  const std::int64_t step = std::max<std::int64_t>(params.grid_step_ns, 1);
  for (std::int64_t d = -params.grid_half_span_ns; d <= params.grid_half_span_ns; d += step) {
    grid.push_back(d);
  }
  const std::size_t n_grid = grid.size();
  const std::size_t n_iv = measured.size();
  std::vector<double> resid(n_grid * n_iv, std::numeric_limits<double>::quiet_NaN());
  std::size_t zero_index = n_grid;
  for (std::size_t gi = 0; gi < n_grid; ++gi) {
    if (grid[gi] == 0) {
      zero_index = gi;
    }
    for (std::size_t i = 0; i < n_iv; ++i) {
      const auto ref = reference(measured[i].t0_ns + grid[gi], measured[i].t1_ns + grid[gi]);
      if (!ref.has_value()) {
        continue;
      }
      const Vec3 r{
        measured[i].rotvec[0] - (*ref)[0], measured[i].rotvec[1] - (*ref)[1],
        measured[i].rotvec[2] - (*ref)[2]};
      resid[gi * n_iv + i] = norm3(r);
    }
  }

  // Robust scale and the "before" residual from the unshifted residuals (the
  // grid point nearest d = 0): one reference that does not depend on the
  // answer, so the cost is comparable across d — a scale taken at the winning
  // shift would move with the very minimum being searched for.
  if (zero_index == n_grid) {
    zero_index = n_grid / 2;
  }
  std::vector<double> at_zero;
  for (std::size_t i = 0; i < n_iv; ++i) {
    const double r = resid[zero_index * n_iv + i];
    if (!std::isnan(r)) {
      at_zero.push_back(r);
    }
  }
  if (at_zero.size() < params.min_intervals) {
    result.error = "the reference covers only " + std::to_string(at_zero.size()) +
                   " of the measured interval(s) at zero shift; at least " +
                   std::to_string(params.min_intervals) + " are needed";
    return result;
  }
  const double scale = 3.0 * median_of(at_zero) + 1e-9;
  double before_sq = 0.0;
  for (const double r : at_zero) {
    before_sq += r * r;
  }
  result.fit.residual_rms_before_rad = std::sqrt(before_sq / static_cast<double>(at_zero.size()));

  // Mean robust cost per grid point over the intervals the reference covers
  // there; a grid point with too few covered intervals is invalid (its mean
  // would be over a different, smaller population).
  const auto cost_curve = [&](
                            const std::vector<std::size_t> & idx, std::vector<double> & cost,
                            std::vector<bool> & valid) {
    cost.assign(n_grid, 0.0);
    valid.assign(n_grid, false);
    for (std::size_t gi = 0; gi < n_grid; ++gi) {
      double sum = 0.0;
      std::size_t count = 0;
      for (const std::size_t i : idx) {
        const double r = resid[gi * n_iv + i];
        if (std::isnan(r)) {
          continue;
        }
        sum += huber(r, scale);
        ++count;
      }
      if (count >= params.min_intervals) {
        cost[gi] = sum / static_cast<double>(count);
        valid[gi] = true;
      }
    }
  };

  std::vector<std::size_t> all(n_iv);
  for (std::size_t i = 0; i < n_iv; ++i) {
    all[i] = i;
  }
  std::vector<double> cost;
  std::vector<bool> valid;
  cost_curve(all, cost, valid);
  const GridMinimum best = grid_minimum(cost, valid, grid);
  if (!best.found) {
    result.error = "the reference covers too few of the measured intervals at every shift";
    return result;
  }
  result.fit.offset_ns = static_cast<std::int64_t>(std::llround(best.offset_ns));
  result.fit.on_grid_edge = best.on_edge;

  // Residual at the found offset, and the count of intervals it rests on.
  {
    double after_sq = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < n_iv; ++i) {
      const auto ref = reference(
        measured[i].t0_ns + result.fit.offset_ns, measured[i].t1_ns + result.fit.offset_ns);
      if (!ref.has_value()) {
        continue;
      }
      const Vec3 r{
        measured[i].rotvec[0] - (*ref)[0], measured[i].rotvec[1] - (*ref)[1],
        measured[i].rotvec[2] - (*ref)[2]};
      const double n = norm3(r);
      after_sq += n * n;
      ++count;
    }
    result.fit.intervals = count;
    result.fit.residual_rms_after_rad =
      count > 0 ? std::sqrt(after_sq / static_cast<double>(count)) : 0.0;
  }
  if (best.on_edge) {
    result.error = "the best shift sits on the search grid's edge (" +
                   std::to_string(static_cast<double>(result.fit.offset_ns) / 1e6) +
                   " ms); the offset is beyond +-" +
                   std::to_string(static_cast<double>(params.grid_half_span_ns) / 1e6) + " ms";
    return result;
  }

  // Bootstrap over intervals for the spread of the minimum.
  if (params.bootstrap_draws > 0) {
    std::mt19937_64 rng(params.seed);
    std::uniform_int_distribution<std::size_t> pick(0, n_iv - 1);
    std::vector<double> mins;
    mins.reserve(static_cast<std::size_t>(params.bootstrap_draws));
    std::vector<std::size_t> idx(n_iv);
    std::vector<double> bcost;
    std::vector<bool> bvalid;
    for (int draw = 0; draw < params.bootstrap_draws; ++draw) {
      for (auto & i : idx) {
        i = pick(rng);
      }
      cost_curve(idx, bcost, bvalid);
      const GridMinimum m = grid_minimum(bcost, bvalid, grid);
      if (m.found) {
        mins.push_back(m.offset_ns);
      }
    }
    if (mins.size() > 1) {
      double mean = 0.0;
      for (const double m : mins) {
        mean += m;
      }
      mean /= static_cast<double>(mins.size());
      double var = 0.0;
      for (const double m : mins) {
        var += (m - mean) * (m - mean);
      }
      var /= static_cast<double>(mins.size());
      result.fit.std_ns = static_cast<std::int64_t>(std::llround(std::sqrt(var)));
    }
    if (result.fit.std_ns > params.max_std_ns) {
      result.error = "the estimate's spread is too wide (+-" +
                     std::to_string(static_cast<double>(result.fit.std_ns) / 1e6) + " ms; " +
                     std::to_string(static_cast<double>(params.max_std_ns) / 1e6) + " ms allowed)";
      return result;
    }
  }
  result.ok = true;
  return result;
}

GyroBridgedOffset fit_time_offset_via_gyro(
  std::span<const RotationInterval> measured, std::span<const RotationInterval> trajectory,
  const GyroIntegral & gyro, const TimeOffsetParams & params)
{
  GyroBridgedOffset out;
  const auto provider = gyro.provider();
  out.sensor_gyro = fit_time_offset(measured, provider, params);
  out.trajectory_gyro = fit_time_offset(trajectory, provider, params);
  if (!out.sensor_gyro.ok) {
    out.combined.error = "sensor vs gyro: " + out.sensor_gyro.error;
    return out;
  }
  if (!out.trajectory_gyro.ok) {
    out.combined.error = "trajectory vs gyro: " + out.trajectory_gyro.error;
    return out;
  }
  out.combined.ok = true;
  out.combined.fit = out.sensor_gyro.fit;
  out.combined.fit.offset_ns = out.sensor_gyro.fit.offset_ns - out.trajectory_gyro.fit.offset_ns;
  out.combined.fit.std_ns = static_cast<std::int64_t>(std::llround(
    std::hypot(
      static_cast<double>(out.sensor_gyro.fit.std_ns),
      static_cast<double>(out.trajectory_gyro.fit.std_ns))));
  out.combined.fit.on_grid_edge = false;
  return out;
}

}  // namespace bagwiz::core::calib
