// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_
#define BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_

#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <array>
#include <cmath>
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
  // landscape the samples cannot constrain — eigen-directions of the
  // finite-difference Hessian whose paired curvature is not significant
  // (observability.hpp) — are held at the bag value and the remaining
  // directions are re-optimized. The held set is reported via
  // RefineResult::auto_held. When false, every free axis is optimized and the
  // observability classification is report-only.
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
  std::array<AxisObservability, 6> observability{};
  // Directions held at the bag value by --fix auto, in the order they were
  // held; empty when auto_fix is off or every direction was observable.
  std::vector<HeldDirection> auto_held;
  int samples_used = 0;
};

/// Two-pass Nelder-Mead refinement of the free axes of `chain`'s edited edge,
/// minimizing mean NID cost over `samples`, followed by a per-axis
/// observability probe around the optimum (significance-tested paired
/// curvature; see observability.hpp). With RefineParams::auto_fix the probe
/// runs on the full Hessian's eigen-directions instead, unobservable
/// directions are held at the bag value, and the rest are re-optimized.
[[nodiscard]] RefineResult refine_extrinsic(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const RefineParams & params);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_
