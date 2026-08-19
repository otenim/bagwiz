// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__NELDER_MEAD_HPP_
#define BAGWIZ__CORE__CALIB__NELDER_MEAD_HPP_

#include <functional>
#include <span>
#include <vector>

namespace bagwiz::core::calib
{

struct NelderMeadParams
{
  std::vector<double> init_step;  // per-dimension simplex seed offsets
  double convergence_var = 1e-8;  // sum of coordinate variances threshold
  int max_iterations = 256;
};

struct NelderMeadResult
{
  std::vector<double> x;
  double value = 0.0;
  int iterations = 0;
  bool converged = false;
};

/// Minimize a cost function using the Nelder-Mead downhill simplex algorithm.
/// The simplex is initialized by perturbing x0 by init_step in each dimension.
/// Note: init_step.size() must equal x0.size() (asserted in implementation).
/// The cost function f may return +inf (e.g., to model trust-region walls or
/// degenerate projections); the simplex orders such points last and contracts
/// away from them.
[[nodiscard]] NelderMeadResult nelder_mead_minimize(
  const std::function<double(std::span<const double>)> & f, std::span<const double> x0,
  const NelderMeadParams & params);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__NELDER_MEAD_HPP_
