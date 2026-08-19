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

namespace bagwiz::core::calib
{

enum class AxisObservability { kFixed, kStrong, kWeak, kDegenerate };

struct EdgeChain
{
  Mat4 t_trajframe_parent{};  // trajectory frame -> edited edge's parent
  Mat4 t_parent_child{};      // the edited edge, as recorded in the bag
  Mat4 t_child_camoptical{};  // edited edge's child -> camera optical frame
};

struct RefineParams
{
  NidParams nid;
  std::array<bool, 6> fixed{};        // x, y, z, roll, pitch, yaw
  double max_trans = 0.2;             // trust region, meters
  double max_rot = 2.0 * M_PI / 180;  // trust region, radians
  int max_iterations = 256;
};

struct RefineResult
{
  bool ok = false;
  std::string error;
  std::array<double, 6> delta{};  // x,y,z,roll,pitch,yaw on the edge
  double nid_before = 0.0;
  double nid_after = 0.0;
  std::array<AxisObservability, 6> observability{};
  int samples_used = 0;
};

/// Two-pass Nelder-Mead refinement of the free axes of `chain`'s edited edge,
/// minimizing mean NID cost over `samples`, followed by a per-axis
/// observability probe around the optimum.
[[nodiscard]] RefineResult refine_extrinsic(
  std::span<const CalibSample> samples, const CameraModel & cam, const EdgeChain & chain,
  const RefineParams & params);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__EXTRINSIC_REFINE_HPP_
