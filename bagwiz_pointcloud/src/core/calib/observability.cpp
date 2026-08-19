// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/observability.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace bagwiz::core::calib
{

CurvatureEstimate paired_curvature(
  std::span<const std::optional<double>> center, std::span<const std::optional<double>> plus,
  std::span<const std::optional<double>> minus)
{
  CurvatureEstimate est;
  double sum = 0.0;
  double sum_sq = 0.0;
  // The three spans are per-sample readings of the same sample set, so they
  // are equal-length on every call here; the min keeps a mismatched caller
  // reading in bounds rather than off the end.
  const std::size_t n = std::min({center.size(), plus.size(), minus.size()});
  for (std::size_t i = 0; i < n; ++i) {
    if (!center[i].has_value() || !plus[i].has_value() || !minus[i].has_value()) {
      continue;
    }
    const double kappa = *plus[i] + *minus[i] - 2.0 * (*center[i]);
    sum += kappa;
    sum_sq += kappa * kappa;
    ++est.pairs;
  }
  if (est.pairs == 0) {
    return est;
  }
  est.mean = sum / est.pairs;
  if (est.pairs >= 2) {
    // Sample variance (N-1): the spread of the per-sample curvatures, scaled
    // to the standard error of their mean.
    const double var = (sum_sq - est.pairs * est.mean * est.mean) / (est.pairs - 1);
    est.std_error = std::sqrt(std::max(var, 0.0) / est.pairs);
  }
  return est;
}

bool curvature_significant(const CurvatureEstimate & est)
{
  return est.pairs > 0 &&
         est.mean > std::max(kDegenerateSigma * est.std_error, kDegenerateCurvatureFloor);
}

Eigensystem jacobi_eigen(const std::vector<std::vector<double>> & sym)
{
  const std::size_t n = sym.size();
  std::vector<std::vector<double>> a = sym;
  std::vector<std::vector<double>> v(n, std::vector<double>(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    v[i][i] = 1.0;
  }

  // Cyclic Jacobi: sweep the off-diagonal entries, annihilating each with a
  // Givens rotation, until the largest off-diagonal magnitude falls under the
  // tolerance. k <= 6 here, so the quadratic sweeps cost nothing; the 50-sweep
  // cap only bounds pathological input.
  const double eps = 1e-15;
  for (int sweep = 0; sweep < 50; ++sweep) {
    double off_max = 0.0;
    for (std::size_t p = 0; p < n; ++p) {
      for (std::size_t q = p + 1; q < n; ++q) {
        off_max = std::max(off_max, std::abs(a[p][q]));
      }
    }
    if (off_max < eps) {
      break;
    }
    for (std::size_t p = 0; p < n; ++p) {
      for (std::size_t q = p + 1; q < n; ++q) {
        if (std::abs(a[p][q]) < eps * 1e-3) {
          continue;
        }
        // Stable Jacobi rotation angle (Numerical Recipes' form: t = tan
        // theta via the cot(2*theta) path, avoiding the quadratic formula's
        // cancellation).
        const double app = a[p][p];
        const double aqq = a[q][q];
        const double apq = a[p][q];
        const double theta = (aqq - app) / (2.0 * apq);
        const double t =
          (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        for (std::size_t k = 0; k < n; ++k) {
          const double akp = a[k][p];
          const double akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (std::size_t k = 0; k < n; ++k) {
          const double apk = a[p][k];
          const double aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (std::size_t k = 0; k < n; ++k) {
          const double vkp = v[k][p];
          const double vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }

  // Ascending eigenvalue order, eigenvectors permuted to match.
  std::vector<std::size_t> order(n);
  for (std::size_t i = 0; i < n; ++i) {
    order[i] = i;
  }
  std::sort(
    order.begin(), order.end(), [&a](std::size_t i, std::size_t j) { return a[i][i] < a[j][j]; });
  Eigensystem es;
  es.values.resize(n);
  es.vectors.assign(n, std::vector<double>(n, 0.0));
  for (std::size_t j = 0; j < n; ++j) {
    es.values[j] = a[order[j]][order[j]];
    for (std::size_t i = 0; i < n; ++i) {
      es.vectors[i][j] = v[i][order[j]];
    }
  }
  return es;
}

}  // namespace bagwiz::core::calib
