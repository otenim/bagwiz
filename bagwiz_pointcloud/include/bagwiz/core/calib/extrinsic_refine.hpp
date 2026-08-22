// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_
#define BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_

#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/observability.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::calib
{

enum class AxisObservability { kFixed, kStrong, kWeak, kDegenerate };

struct EdgeChain
{
  Mat4 t_trajframe_parent{};  // trajectory frame -> edited edge's parent
  // The edited edge as recorded in the bag, in the six scalars of the
  // static-transform-publisher schema: x, y, z (meters) then roll, pitch, yaw
  // (radians, tf2 fixed-axis convention). Kept as scalars rather than a Mat4
  // because the delta is ADDITIVE on them (see apply_edge_delta) — the same
  // parametrization `bagwiz tf static dump` writes and
  // `bagwiz tf static update` applies.
  std::array<double, 6> edge_bag{};
  Mat4 t_child_camoptical{};  // edited edge's child -> camera optical frame
};

/// The edited edge's six scalars displaced by `delta`, one axis at a time.
/// This is THE parametrization refine_extrinsic searches, so the reported
/// "refined value" and the emitted YAML are the same function of the delta as
/// the cost the optimizer minimized — they cannot describe different edges.
/// Additive rather than a right-multiplied SE3 factor: it matches the
/// static-transform-publisher scalar schema, and on a non-commuting rotation
/// (an optical-convention edge, say) the two differ by an axis swap.
[[nodiscard]] std::array<double, 6> apply_edge_delta(
  const std::array<double, 6> & edge_bag, const std::array<double, 6> & delta);

/// apply_edge_delta as a rigid transform: make_transform of the summed
/// translation and rpy triples.
[[nodiscard]] Mat4 edge_transform(
  const std::array<double, 6> & edge_bag, const std::array<double, 6> & delta);

struct RefineParams
{
  NidParams nid;
  std::array<bool, 6> fixed{};        // x, y, z, roll, pitch, yaw
  double max_trans = 0.2;             // trust region, meters
  double max_rot = 2.0 * M_PI / 180;  // trust region, radians
  int max_iterations = 256;
  // --fix auto (the default): after optimizing, directions of the cost
  // landscape the samples clearly cannot constrain — eigen-directions of the
  // finite-difference Hessian whose paired curvature stays within kHoldSigma
  // standard errors of zero (observability.hpp) — are held at the bag value
  // and the remaining directions are re-optimized. Borderline directions
  // (insignificant but not clearly so) are left free rather than pinned on a
  // noisy reading. The held set is reported via RefineResult::auto_held. When
  // false, every free axis is optimized and the observability classification
  // is report-only.
  bool auto_fix = true;
};

// One direction of the 6-axis delta space held at the bag value by --fix
// auto: a unit vector in probe-step-normalized coordinates (axis i scaled by
// kProbeStepTrans / kProbeStepRot — the coordinates the Hessian analysis and
// the curvature floors work in), i.e. THE exact direction whose delta content
// was zeroed: sum_i (delta_i / step_i) * unit_i == 0. To render it as an axis
// mixture in physical units, scale component i by step_i and renormalize.
// `curvature`/`std_error` are the paired-curvature measurement that judged
// the direction unobservable.
struct HeldDirection
{
  std::array<double, 6> unit{};
  double curvature = 0.0;
  double std_error = 0.0;
};

struct RefineResult
{
  bool ok = false;
  std::string error;
  // Added to EdgeChain::edge_bag axis by axis to give the refined edge; see
  // apply_edge_delta.
  std::array<double, 6> delta{};  // x,y,z,roll,pitch,yaw on the edge
  double nid_before = 0.0;
  double nid_after = 0.0;
  // Per-axis verdict at the final optimum. With RefineParams::auto_fix a
  // kDegenerate label means the auto-hold took that axis's content: an axis
  // whose own probe fails significance while no held direction covers it (see
  // held_directions_cover_axis) is reported kWeak instead, so the label can
  // never contradict the held set. The axis probe and the eigen-direction
  // analysis are different statistics over different directions, so a
  // borderline axis can fail its own test while every eigen-direction passes;
  // the curvature evidence below keeps that visible. Without auto_fix the
  // label is the raw verdict of the axis's own significance test.
  std::array<AxisObservability, 6> observability{};
  // The paired-curvature measurement behind each axis's verdict, at the final
  // optimum. Fixed axes keep the default (pairs == 0).
  std::array<CurvatureEstimate, 6> curvature{};
  // Directions held at the bag value by --fix auto, in the order they were
  // held; empty when auto_fix is off or every direction was observable.
  std::vector<HeldDirection> auto_held;
  int samples_used = 0;
};

/// True when some held direction's physical-units axis mixture carries at
/// least half its weight on `axis` — i.e. the axis's degenerate reading is
/// already accounted for by the held set. The mixture rescales each
/// normalized component by its axis's probe step, the same conversion the
/// CLI's held-direction display uses.
[[nodiscard]] bool held_directions_cover_axis(
  std::span<const HeldDirection> held, std::size_t axis);

/// Per-sample NID costs at `delta` (nullopt for a sample that projects too
/// few points), evaluated over `pool` — nullptr runs everything on the
/// calling thread. Every sample's points are projected as one range per
/// worker, then each sample's histogram is built from its ranges. The costs
/// are the same for every pool size: projection is per point, the depth
/// cull's nearest depth is a min-reduction and the histograms count integers.
[[nodiscard]] std::vector<std::optional<double>> evaluate_sample_costs(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const std::array<double, 6> & delta, const NidParams & nid, WorkerPool * pool);

/// Two-pass Nelder-Mead refinement of the free axes of `chain`'s edited edge,
/// minimizing mean NID cost over `samples`, followed by a per-axis
/// observability probe around the optimum (significance-tested paired
/// curvature; see observability.hpp). With RefineParams::auto_fix the probe
/// runs on the full Hessian's eigen-directions instead, unobservable
/// directions are held at the bag value, and the rest are re-optimized. Every
/// cost evaluation runs over `pool` (see evaluate_sample_costs); nullptr runs
/// them on the calling thread. The result does not depend on the pool.
[[nodiscard]] RefineResult refine_extrinsic(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const RefineParams & params, WorkerPool * pool = nullptr);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_
