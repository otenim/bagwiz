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
#include "bagwiz/core/calib/observability.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <future>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace bagwiz::core::calib
{

namespace
{

double probe_step(std::size_t axis)
{
  return axis < 3 ? kProbeStepTrans : kProbeStepRot;
}

// Per-sample NID costs at one delta (one async task per sample, as the old
// mean-only total_cost ran them).
std::vector<std::optional<double>> sample_costs(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const std::array<double, 6> & delta, const NidParams & nid)
{
  const Mat4 t_trajframe_cam = mat4_multiply(
    chain.t_trajframe_parent,
    mat4_multiply(edge_transform(chain.edge_bag, delta), chain.t_child_camoptical));

  std::vector<std::future<std::optional<double>>> futures;
  futures.reserve(samples.size());
  for (const auto & sample : samples) {
    futures.push_back(std::async(std::launch::async, [&sample, &cam, &nid, &t_trajframe_cam] {
      const Mat4 t_cam_world =
        rigid_inverse(mat4_multiply(sample.t_world_trajframe, t_trajframe_cam));
      return nid_cost(sample, cam, t_cam_world, nid);
    }));
  }
  std::vector<std::optional<double>> out;
  out.reserve(samples.size());
  for (auto & f : futures) {
    out.push_back(f.get());
  }
  return out;
}

// Mean over the valid samples; infinity when none are valid.
double mean_of_valid(std::span<const std::optional<double>> per_sample, int * valid_out = nullptr)
{
  double sum = 0.0;
  int valid = 0;
  for (const auto & c : per_sample) {
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

bool in_trust_region(const std::array<double, 6> & d, double max_trans, double max_rot)
{
  const double trans = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  const double rot = std::sqrt(d[3] * d[3] + d[4] * d[4] + d[5] * d[5]);
  return trans <= max_trans && rot <= max_rot;
}

// The delta (real units) of coordinates `u` under `basis` (columns unit in
// probe-step-normalized space): delta_i = h_i * sum_j basis[j][i] * u[j].
std::array<double, 6> compose_delta(
  const std::vector<std::array<double, 6>> & basis, std::span<const double> u)
{
  std::array<double, 6> delta{};
  for (std::size_t j = 0; j < basis.size(); ++j) {
    for (std::size_t i = 0; i < 6; ++i) {
      delta[i] += probe_step(i) * basis[j][i] * u[j];
    }
  }
  return delta;
}

// Coordinates of `delta` in `basis` (the columns are orthonormal in
// normalized space, so this is a plain projection).
std::vector<double> project_coords(
  const std::vector<std::array<double, 6>> & basis, const std::array<double, 6> & delta)
{
  std::vector<double> u(basis.size(), 0.0);
  for (std::size_t j = 0; j < basis.size(); ++j) {
    for (std::size_t i = 0; i < 6; ++i) {
      u[j] += basis[j][i] * (delta[i] / probe_step(i));
    }
  }
  return u;
}

// The refinement context shared by the optimizer and the analysis probes.
struct RefineContext
{
  std::span<const CalibSample> samples;
  const CameraModel & cam;
  const EdgeChain & chain;
  const RefineParams & params;

  std::vector<std::optional<double>> costs_at(
    const std::vector<std::array<double, 6>> & basis, std::span<const double> u) const
  {
    return sample_costs(samples, cam, chain, compose_delta(basis, u), params.nid);
  }
};

// Two-pass Nelder-Mead over the current basis. Init steps are uniform in
// normalized coordinates (0.5 then 0.1 steps — the old delta-space
// 0.01 m / 0.1 deg and 0.002 m / 0.02 deg passes divided by the probe steps).
NelderMeadResult optimize(
  const RefineContext & ctx, const std::vector<std::array<double, 6>> & basis,
  std::span<const double> start)
{
  const auto cost = [&](std::span<const double> u) {
    const auto delta = compose_delta(basis, u);
    if (!in_trust_region(delta, ctx.params.max_trans, ctx.params.max_rot)) {
      return std::numeric_limits<double>::infinity();
    }
    return mean_of_valid(ctx.costs_at(basis, u));
  };
  const auto pass = [&](std::span<const double> x0, double step) {
    NelderMeadParams nm;
    nm.max_iterations = ctx.params.max_iterations;
    nm.init_step.assign(basis.size(), step);
    return nelder_mead_minimize(cost, x0, nm);
  };
  const auto coarse = pass(start, 0.5);
  return pass(coarse.x, 0.1);
}

// One analysis round at `u_star` (the current basis's coordinates of the
// optimum): the finite-difference Hessian in the basis coordinates, its
// eigensystem, and per-eigen-direction paired-curvature significance.
struct Analysis
{
  Eigensystem eigen;
  std::vector<CurvatureEstimate> curvature;  // per eigen-direction
  std::vector<bool> significant;
};

Analysis analyze(
  const RefineContext & ctx, const std::vector<std::array<double, 6>> & basis,
  const std::vector<double> & u_star)
{
  const std::size_t k = basis.size();
  const auto eval = [&](std::span<const double> u) { return ctx.costs_at(basis, u); };

  std::vector<double> probe(u_star);
  const auto center = eval(u_star);

  // Hessian by central differences with a unit (one-probe-step) step in each
  // basis coordinate. Entries are means over each evaluation's valid samples
  // (pairing is only for the significance test, not the point estimate). An
  // entry whose evaluations are all invalid is set to 0 — direction finding
  // degrades, but the per-direction probes below remain the classification
  // authority and score the overlap as unobservable on their own.
  std::vector<std::vector<double>> hessian(k, std::vector<double>(k, 0.0));
  for (std::size_t i = 0; i < k; ++i) {
    probe = u_star;
    probe[i] += 1.0;
    const auto plus = eval(probe);
    probe[i] -= 2.0;
    const auto minus = eval(probe);
    const double h_ii = mean_of_valid(plus) + mean_of_valid(minus) - 2.0 * mean_of_valid(center);
    hessian[i][i] = std::isfinite(h_ii) ? h_ii : 0.0;
  }
  for (std::size_t i = 0; i < k; ++i) {
    for (std::size_t j = i + 1; j < k; ++j) {
      probe = u_star;
      probe[i] += 1.0;
      probe[j] += 1.0;
      const auto pp = eval(probe);
      probe[j] -= 2.0;
      const auto pm = eval(probe);
      probe[i] -= 2.0;
      probe[j] += 2.0;
      const auto mp = eval(probe);
      probe[j] -= 2.0;
      const auto mm = eval(probe);
      const double h_ij =
        (mean_of_valid(pp) - mean_of_valid(pm) - mean_of_valid(mp) + mean_of_valid(mm)) / 4.0;
      hessian[i][j] = hessian[j][i] = std::isfinite(h_ij) ? h_ij : 0.0;
    }
  }

  Analysis out;
  out.eigen = jacobi_eigen(hessian);
  out.curvature.resize(k);
  out.significant.resize(k, false);
  for (std::size_t j = 0; j < k; ++j) {
    std::vector<double> dir(k);
    for (std::size_t i = 0; i < k; ++i) {
      dir[i] = out.eigen.vectors[i][j];
    }
    std::vector<double> up(k), um(k);
    for (std::size_t i = 0; i < k; ++i) {
      up[i] = u_star[i] + dir[i];
      um[i] = u_star[i] - dir[i];
    }
    out.curvature[j] = paired_curvature(center, eval(up), eval(um));
    out.significant[j] = curvature_significant(out.curvature[j]);
  }
  return out;
}

// The HeldDirection form of one eigen-direction (given in basis coordinates):
// mapped through the basis into normalized 6-axis coordinates — exactly the
// direction whose delta content is zeroed — with the largest-magnitude axis
// made positive for a stable report. `curvature` is copied into the
// HeldDirection for the report.
HeldDirection held_direction_of(
  const std::vector<std::array<double, 6>> & basis, const std::vector<double> & dir,
  const CurvatureEstimate & curvature)
{
  HeldDirection held;
  held.curvature = curvature.mean;
  held.std_error = curvature.std_error;
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j < basis.size(); ++j) {
      held.unit[i] += basis[j][i] * dir[j];
    }
  }
  std::size_t dominant = 0;
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::abs(held.unit[i]) > std::abs(held.unit[dominant])) {
      dominant = i;
    }
  }
  if (held.unit[dominant] < 0.0) {
    for (auto & c : held.unit) {
      c = -c;
    }
  }
  return held;
}

AxisObservability classify_axis(const CurvatureEstimate & est)
{
  if (!curvature_significant(est)) {
    return AxisObservability::kDegenerate;
  }
  return est.mean >= std::max(kStrongSigma * est.std_error, kStrongCurvatureFloor)
           ? AxisObservability::kStrong
           : AxisObservability::kWeak;
}

}  // namespace

std::array<double, 6> apply_edge_delta(
  const std::array<double, 6> & edge_bag, const std::array<double, 6> & delta)
{
  std::array<double, 6> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = edge_bag[i] + delta[i];
  }
  return out;
}

Mat4 edge_transform(const std::array<double, 6> & edge_bag, const std::array<double, 6> & delta)
{
  const auto e = apply_edge_delta(edge_bag, delta);
  return make_transform({e[0], e[1], e[2]}, {e[3], e[4], e[5]});
}

RefineResult refine_extrinsic(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const RefineParams & params)
{
  RefineResult result;
  const RefineContext ctx{samples, cam, chain, params};

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

  int valid = 0;
  result.nid_before = mean_of_valid(sample_costs(samples, cam, chain, {}, params.nid), &valid);
  result.samples_used = valid;
  if (!std::isfinite(result.nid_before)) {
    result.error =
      "no sample projects enough map points at the initial extrinsic; "
      "check --of, the TF chain, and the depth window";
    return result;
  }

  // The free subspace, as orthonormal basis columns in probe-step-normalized
  // coordinates: initially the free axes' unit vectors, then the surviving
  // eigen-directions after each auto-fix round.
  std::vector<std::array<double, 6>> basis;
  for (const int axis : free_axes) {
    std::array<double, 6> e{};
    e[axis] = 1.0;
    basis.push_back(e);
  }
  std::vector<double> u(basis.size(), 0.0);
  std::array<double, 6> delta{};

  // Optimize, then (with --fix auto) analyze: unobservable eigen-directions
  // are held at the bag value and the loop re-optimizes the rest. Every round
  // holds at least one direction, so the loop terminates within six rounds.
  for (int round = 0;; ++round) {
    const auto opt = optimize(ctx, basis, u);
    delta = compose_delta(basis, opt.x);
    if (!params.auto_fix) {
      break;
    }
    const auto analysis = analyze(ctx, basis, opt.x);

    std::vector<std::size_t> kept;
    std::size_t degenerate = 0;
    for (std::size_t j = 0; j < basis.size(); ++j) {
      if (analysis.significant[j]) {
        kept.push_back(j);
      } else {
        ++degenerate;
      }
    }
    if (degenerate == 0) {
      break;
    }
    // Record every degenerate direction as held at the bag value (delta-space
    // display form, dominant axis positive).
    for (std::size_t j = 0; j < basis.size(); ++j) {
      if (!analysis.significant[j]) {
        std::vector<double> dir(basis.size());
        for (std::size_t i = 0; i < basis.size(); ++i) {
          dir[i] = analysis.eigen.vectors[i][j];
        }
        result.auto_held.push_back(held_direction_of(basis, dir, analysis.curvature[j]));
      }
    }
    if (kept.empty()) {
      if (round == 0) {
        result.error =
          "the samples constrain no direction of this edge (every direction's curvature is "
          "insignificant); cannot refine — check the trajectory span, the TF chain, and the "
          "depth window";
        result.auto_held.clear();  // meaningless without a result; the error says it all
        return result;
      }
      // Everything left is unobservable: the remaining coordinates stay at
      // zero (the bag value), which is the projected start of this round.
      delta = compose_delta(basis, std::vector<double>(basis.size(), 0.0));
      break;
    }
    // Re-parameterize to the surviving eigen-directions and restart from the
    // current optimum projected into that subspace (the held content is
    // zeroed — it is untrustworthy by construction).
    std::vector<std::array<double, 6>> next_basis;
    for (const std::size_t j : kept) {
      std::array<double, 6> col{};
      for (std::size_t i = 0; i < 6; ++i) {
        for (std::size_t r = 0; r < basis.size(); ++r) {
          col[i] += basis[r][i] * analysis.eigen.vectors[r][j];
        }
      }
      next_basis.push_back(col);
    }
    basis = std::move(next_basis);
    u = project_coords(basis, delta);
  }

  result.delta = delta;

  // Per-axis observability at the final optimum (report only): paired
  // curvature along each delta-space axis, classified by the same
  // significance test the auto-fix directions used. The costs at the optimum
  // are both the reported nid_after and the center of every probe below, so
  // they are evaluated once.
  const auto center = sample_costs(samples, cam, chain, delta, params.nid);
  result.nid_after = mean_of_valid(center);
  for (int axis = 0; axis < 6; ++axis) {
    if (params.fixed[axis]) {
      result.observability[axis] = AxisObservability::kFixed;
      continue;
    }
    auto plus = delta;
    auto minus = delta;
    plus[axis] += probe_step(axis);
    minus[axis] -= probe_step(axis);
    const auto est = paired_curvature(
      center, sample_costs(samples, cam, chain, plus, params.nid),
      sample_costs(samples, cam, chain, minus, params.nid));
    result.observability[axis] = classify_axis(est);
  }

  result.ok = true;
  return result;
}

}  // namespace bagwiz::core::calib
