// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__OBSERVABILITY_HPP_
#define BAGWIZ__CORE__CALIB__OBSERVABILITY_HPP_

#include <cmath>
#include <optional>
#include <span>
#include <vector>

// Curvature-based observability machinery for refine_extrinsic: a noise-aware
// significance test over per-sample paired second differences, and a small
// Jacobi eigensolver for the symmetric Hessian the degenerate directions are
// read from. GLIM/Eigen-free like the rest of this package.
namespace bagwiz::core::calib
{

// Absolute curvature floor in cost units (per normalized probe-step squared):
// an axis whose mean curvature falls under it reads degenerate however quiet
// the samples are. Calibrated against the 2026-08-19 spike bag, where
// strongly observable axes showed second differences of ~1e-4..1e-3 at the
// probe step and degenerate ones under ~1e-5. The strong/weak boundary is the
// matching upper floor.
inline constexpr double kDegenerateCurvatureFloor = 5e-6;
inline constexpr double kStrongCurvatureFloor = 5e-5;

// The probe step per delta axis, translation (meters) and rotation (radians).
// All observability probes displace the delta by one step, and the Hessian
// the degenerate directions are read from is built in coordinates normalized
// by these steps, so "1 step" along any (possibly eigen-rotated) direction is
// a comparable physical displacement and the floors above keep one scale.
inline constexpr double kProbeStepTrans = 0.02;              // m
inline constexpr double kProbeStepRot = 0.2 * M_PI / 180.0;  // rad

// How many standard errors the mean curvature must clear to count as
// measured at all (degenerate boundary) resp. well-measured (strong/weak
// boundary). With few samples the standard error is itself noisy, so the
// floors above keep the old absolute behavior as the backstop.
inline constexpr double kDegenerateSigma = 2.0;
inline constexpr double kStrongSigma = 5.0;

// A second-difference curvature measurement along one probe direction,
// estimated per-sample and summarized as mean ± standard error across
// samples. `pairs` counts the samples valid at all three evaluation points.
struct CurvatureEstimate
{
  double mean = 0.0;
  double std_error = 0.0;
  int pairs = 0;
};

// Paired per-sample second difference: for every sample valid at all three
// points, kappa_k = plus_k + minus_k - 2*center_k; the estimate is the mean of
// kappa_k and its standard error (sample stddev / sqrt(pairs); 0 with one
// pair). Pairing cancels the per-sample level differences a pooled comparison
// would mistake for curvature noise.
[[nodiscard]] CurvatureEstimate paired_curvature(
  std::span<const std::optional<double>> center, std::span<const std::optional<double>> plus,
  std::span<const std::optional<double>> minus);

// The degenerate/observable boundary: the mean curvature must both clear the
// absolute floor and stand out of its own noise band. With no spread
// information (pairs < 2, std_error == 0) this reduces to the absolute floor.
[[nodiscard]] bool curvature_significant(const CurvatureEstimate & est);

// Eigensystem of a small symmetric matrix: eigenvalues ascending, with the
// matching orthonormal eigenvectors stored as COLUMNS (vectors[row][col]).
struct Eigensystem
{
  std::vector<double> values;
  std::vector<std::vector<double>> vectors;
};

// Cyclic Jacobi sweeps for a symmetric k x k (k is tiny here — 6 at most).
// The input must be symmetric; only the full matrix is read.
[[nodiscard]] Eigensystem jacobi_eigen(const std::vector<std::vector<double>> & sym);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__OBSERVABILITY_HPP_
