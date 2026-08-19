// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nelder_mead.hpp"

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

namespace bagwiz::core::calib
{

namespace
{
struct Vertex
{
  std::vector<double> x;
  double value = 0.0;
};
}  // namespace

NelderMeadResult nelder_mead_minimize(
  const std::function<double(std::span<const double>)> & f, std::span<const double> x0,
  const NelderMeadParams & params)
{
  assert(params.init_step.size() == x0.size());
  const std::size_t n = x0.size();
  std::vector<Vertex> simplex;
  simplex.reserve(n + 1);
  simplex.push_back({{x0.begin(), x0.end()}, 0.0});
  simplex[0].value = f(simplex[0].x);
  for (std::size_t i = 0; i < n; ++i) {
    Vertex v{{x0.begin(), x0.end()}, 0.0};
    v.x[i] += params.init_step[i];
    v.value = f(v.x);
    simplex.push_back(std::move(v));
  }

  const auto by_value = [](const Vertex & a, const Vertex & b) { return a.value < b.value; };
  NelderMeadResult result;
  for (int iter = 0; iter < params.max_iterations; ++iter) {
    result.iterations = iter + 1;
    std::sort(simplex.begin(), simplex.end(), by_value);

    // Convergence: total coordinate variance across the simplex.
    double var = 0.0;
    for (std::size_t d = 0; d < n; ++d) {
      double mean = 0.0;
      for (const auto & v : simplex) {
        mean += v.x[d];
      }
      mean /= static_cast<double>(simplex.size());
      for (const auto & v : simplex) {
        var += (v.x[d] - mean) * (v.x[d] - mean);
      }
    }
    if (var < params.convergence_var) {
      result.converged = true;
      break;
    }

    // Centroid of all but the worst vertex.
    std::vector<double> centroid(n, 0.0);
    for (std::size_t v = 0; v < n; ++v) {
      for (std::size_t d = 0; d < n; ++d) {
        centroid[d] += simplex[v].x[d];
      }
    }
    for (double & c : centroid) {
      c /= static_cast<double>(n);
    }

    const auto blend = [&](double factor) {
      std::vector<double> x(n);
      for (std::size_t d = 0; d < n; ++d) {
        x[d] = centroid[d] + factor * (centroid[d] - simplex[n].x[d]);
      }
      return x;
    };

    Vertex reflected{blend(1.0), 0.0};
    reflected.value = f(reflected.x);
    if (reflected.value < simplex[0].value) {
      Vertex expanded{blend(2.0), 0.0};
      expanded.value = f(expanded.x);
      simplex[n] = expanded.value < reflected.value ? std::move(expanded) : std::move(reflected);
    } else if (reflected.value < simplex[n - 1].value) {
      simplex[n] = std::move(reflected);
    } else {
      Vertex contracted{blend(-0.5), 0.0};
      contracted.value = f(contracted.x);
      if (contracted.value < simplex[n].value) {
        simplex[n] = std::move(contracted);
      } else {
        // Shrink every vertex towards the best.
        for (std::size_t v = 1; v <= n; ++v) {
          for (std::size_t d = 0; d < n; ++d) {
            simplex[v].x[d] = simplex[0].x[d] + 0.5 * (simplex[v].x[d] - simplex[0].x[d]);
          }
          simplex[v].value = f(simplex[v].x);
        }
      }
    }
  }

  std::sort(simplex.begin(), simplex.end(), by_value);
  result.x = simplex[0].x;
  result.value = simplex[0].value;
  return result;
}

}  // namespace bagwiz::core::calib
