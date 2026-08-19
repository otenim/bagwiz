// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/extrinsic_refine.hpp"

#include "bagwiz/core/calib/nelder_mead.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <cmath>
#include <future>
#include <limits>
#include <vector>

namespace bagwiz::core::calib
{

namespace
{

constexpr double kDeg = M_PI / 180.0;
// Curvature thresholds for the observability probe, calibrated against the
// 2026-08-19 spike bag: strongly observable axes showed second differences
// of ~1e-4..1e-3 at the probe step, degenerate ones under ~1e-5.
constexpr double kStrongCurvature = 5e-5;
constexpr double kWeakCurvature = 5e-6;
constexpr double kProbeTrans = 0.02;      // m
constexpr double kProbeRot = 0.2 * kDeg;  // rad

double total_cost(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const std::array<double, 6> & delta, const NidParams & nid, int * valid_out)
{
  const Mat4 d = make_transform({delta[0], delta[1], delta[2]}, {delta[3], delta[4], delta[5]});
  const Mat4 t_trajframe_cam = mat4_multiply(
    chain.t_trajframe_parent,
    mat4_multiply(chain.t_parent_child, mat4_multiply(d, chain.t_child_camoptical)));

  std::vector<std::future<std::optional<double>>> futures;
  futures.reserve(samples.size());
  for (const auto & sample : samples) {
    futures.push_back(std::async(std::launch::async, [&sample, &cam, &nid, &t_trajframe_cam] {
      const Mat4 t_cam_world =
        rigid_inverse(mat4_multiply(sample.t_world_trajframe, t_trajframe_cam));
      return nid_cost(sample, cam, t_cam_world, nid);
    }));
  }
  double sum = 0.0;
  int valid = 0;
  for (auto & f : futures) {
    const auto c = f.get();
    if (c.has_value()) {
      sum += *c;
      ++valid;
    }
  }
  if (valid_out != nullptr) {
    *valid_out = valid;
  }
  return valid > 0 ? sum / valid : std::numeric_limits<double>::infinity();
}

}  // namespace

RefineResult refine_extrinsic(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const RefineParams & params)
{
  RefineResult result;

  std::vector<int> free_axes;
  for (int i = 0; i < 6; ++i) {
    if (!params.fixed[i]) {
      free_axes.push_back(i);
    }
  }
  if (free_axes.empty()) {
    result.error = "every axis is fixed; nothing to optimize";
    return result;
  }

  const auto expand = [&](std::span<const double> reduced) {
    std::array<double, 6> full{};
    for (std::size_t i = 0; i < free_axes.size(); ++i) {
      full[free_axes[i]] = reduced[i];
    }
    return full;
  };
  const auto in_trust_region = [&](const std::array<double, 6> & d) {
    const double trans = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    const double rot = std::sqrt(d[3] * d[3] + d[4] * d[4] + d[5] * d[5]);
    return trans <= params.max_trans && rot <= params.max_rot;
  };
  const auto cost = [&](std::span<const double> reduced) {
    const auto full = expand(reduced);
    if (!in_trust_region(full)) {
      return std::numeric_limits<double>::infinity();
    }
    return total_cost(samples, cam, chain, full, params.nid, nullptr);
  };

  int valid = 0;
  result.nid_before = total_cost(samples, cam, chain, {}, params.nid, &valid);
  result.samples_used = valid;
  if (!std::isfinite(result.nid_before)) {
    result.error =
      "no sample projects enough map points at the initial extrinsic; "
      "check --traj-frame, the TF chain, and the depth window";
    return result;
  }

  const auto pass = [&](std::span<const double> start, double trans_step, double rot_step) {
    NelderMeadParams nm;
    nm.max_iterations = params.max_iterations;
    for (const int axis : free_axes) {
      nm.init_step.push_back(axis < 3 ? trans_step : rot_step);
    }
    return nelder_mead_minimize(cost, start, nm);
  };
  const std::vector<double> zero(free_axes.size(), 0.0);
  const auto coarse = pass(zero, 0.01, 0.1 * kDeg);
  const auto polish = pass(coarse.x, 0.002, 0.02 * kDeg);

  result.delta = expand(polish.x);
  result.nid_after = polish.value;

  // Observability: symmetric second difference per free axis at the optimum.
  for (int axis = 0; axis < 6; ++axis) {
    if (params.fixed[axis]) {
      result.observability[axis] = AxisObservability::kFixed;
      continue;
    }
    const double h = axis < 3 ? kProbeTrans : kProbeRot;
    auto plus = result.delta;
    auto minus = result.delta;
    plus[axis] += h;
    minus[axis] -= h;
    const double c_plus = total_cost(samples, cam, chain, plus, params.nid, nullptr);
    const double c_minus = total_cost(samples, cam, chain, minus, params.nid, nullptr);
    const double curvature = c_plus + c_minus - 2.0 * result.nid_after;
    result.observability[axis] = curvature > kStrongCurvature ? AxisObservability::kStrong
                                 : curvature > kWeakCurvature ? AxisObservability::kWeak
                                                              : AxisObservability::kDegenerate;
  }

  result.ok = true;
  return result;
}

}  // namespace bagwiz::core::calib
