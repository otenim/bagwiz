// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/ground_segmentation.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
// Same guard as VoxelGrid: never divide by a non-positive cell size.
constexpr double kMinCellSize = 1e-3;

// A plane needs three non-collinear supports; below that the PCA scatter is
// rank-deficient and the fit is meaningless.
constexpr std::size_t kMinFitPoints = 3;

// Teschner et al. spatial-hash constants, as in cloud_filters.cpp / dynamic_removal.cpp.
constexpr std::size_t kHashX = 73856093U;
constexpr std::size_t kHashY = 19349663U;

struct CellKey
{
  std::int32_t x;
  std::int32_t y;
  bool operator==(const CellKey & other) const { return x == other.x && y == other.y; }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    const auto mix = [](std::int32_t index, std::size_t multiplier) {
      return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
    };
    return mix(key.x, kHashX) ^ mix(key.y, kHashY);
  }
};

// Eigenvector of the smallest eigenvalue of the symmetric 3x3 matrix given by
// its upper triangle, via cyclic Jacobi rotations. A handful of sweeps drives
// the off-diagonals far below float input precision; the routine is closed
// over its inputs (no iteration-order dependence), so the result is
// deterministic. Eigen-free on purpose: the plain bagwiz_slam library links no
// linear-algebra package.
std::array<double, 3> smallest_eigenvector(
  double xx, double xy, double xz, double yy, double yz, double zz)
{
  std::array<std::array<double, 3>, 3> a{{{xx, xy, xz}, {xy, yy, yz}, {xz, yz, zz}}};
  std::array<std::array<double, 3>, 3> v{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  constexpr int kSweeps = 16;
  constexpr std::array<std::array<int, 2>, 3> kPairs{{{0, 1}, {0, 2}, {1, 2}}};
  for (int sweep = 0; sweep < kSweeps; ++sweep) {
    const double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
    if (off == 0.0) {
      break;
    }
    for (const auto & pair : kPairs) {
      const int p = pair[0];
      const int q = pair[1];
      if (a[p][q] == 0.0) {
        continue;
      }
      const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
      const double t =
        (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
      const double c = 1.0 / std::sqrt(t * t + 1.0);
      const double s = t * c;
      for (int k = 0; k < 3; ++k) {
        const double akp = a[k][p];
        const double akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (int k = 0; k < 3; ++k) {
        const double apk = a[p][k];
        const double aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
        const double vkp = v[k][p];
        const double vkq = v[k][q];
        v[k][p] = c * vkp - s * vkq;
        v[k][q] = s * vkp + c * vkq;
      }
    }
  }
  int smallest = 0;
  for (int i = 1; i < 3; ++i) {
    if (a[i][i] < a[smallest][smallest]) {
      smallest = i;
    }
  }
  return {v[0][smallest], v[1][smallest], v[2][smallest]};
}

bool finite_point(const std::array<float, 3> & point)
{
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}

}  // namespace

std::size_t segment_ground(
  std::span<const std::array<float, 3>> points, const GroundSegmentationConfig & config,
  std::span<std::uint8_t> ground)
{
  assert(ground.size() >= points.size());
  const double cell_size = std::max(config.cell_size, kMinCellSize);
  const double inv_cell_size = 1.0 / cell_size;
  const std::size_t seed_count = static_cast<std::size_t>(std::max(config.seed_count, 1));
  const std::size_t min_cell_points =
    std::max(static_cast<std::size_t>(std::max(config.min_cell_points, 0)), kMinFitPoints);

  // Bin the finite points into square x-y cells; non-finite points are labeled
  // non-ground up front and never join a fit.
  std::unordered_map<CellKey, std::vector<std::uint32_t>, CellKeyHash> cells;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!finite_point(points[i])) {
      ground[i] = 0U;
      continue;
    }
    const CellKey key{
      static_cast<std::int32_t>(std::floor(static_cast<double>(points[i][0]) * inv_cell_size)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(points[i][1]) * inv_cell_size))};
    cells[key].push_back(static_cast<std::uint32_t>(i));
  }

  std::size_t ground_count = 0;
  std::vector<float> heights;
  std::vector<std::uint32_t> inliers;
  for (auto & entry : cells) {
    const std::vector<std::uint32_t> & members = entry.second;

    // Seed reference: mean z of the cell's lowest points. The seed band both
    // initializes the plane fit and serves as the fallback rule whenever a fit
    // is impossible (sparse cell) or rejected (vertical "plane").
    heights.clear();
    heights.reserve(members.size());
    for (const std::uint32_t index : members) {
      heights.push_back(points[index][2]);
    }
    const std::size_t seeds_used = std::min(seed_count, heights.size());
    std::nth_element(
      heights.begin(), heights.begin() + static_cast<std::ptrdiff_t>(seeds_used - 1),
      heights.end());
    double lowest_sum = 0.0;
    for (std::size_t i = 0; i < seeds_used; ++i) {
      lowest_sum += static_cast<double>(heights[i]);
    }
    const double seed_limit = lowest_sum / static_cast<double>(seeds_used) + config.seed_margin;

    const auto apply_seed_rule = [&]() {
      for (const std::uint32_t index : members) {
        const bool is_ground = static_cast<double>(points[index][2]) < seed_limit;
        ground[index] = is_ground ? 1U : 0U;
        ground_count += is_ground ? 1U : 0U;
      }
    };

    if (members.size() < min_cell_points) {
      apply_seed_rule();
      continue;
    }

    inliers.clear();
    for (const std::uint32_t index : members) {
      if (static_cast<double>(points[index][2]) < seed_limit) {
        inliers.push_back(index);
      }
    }
    if (inliers.size() < kMinFitPoints) {
      apply_seed_rule();
      continue;
    }

    // Iterative PCA refinement: plane through the inlier centroid with the
    // scatter's smallest-eigenvalue direction as normal, then re-select the
    // inliers from ALL cell points with the one-sided margin (points under the
    // plane are always ground).
    bool fit_ok = true;
    const int iterations = std::max(config.iterations, 1);
    for (int iteration = 0; iteration < iterations && fit_ok; ++iteration) {
      double cx = 0.0;
      double cy = 0.0;
      double cz = 0.0;
      for (const std::uint32_t index : inliers) {
        cx += static_cast<double>(points[index][0]);
        cy += static_cast<double>(points[index][1]);
        cz += static_cast<double>(points[index][2]);
      }
      const double inv_count = 1.0 / static_cast<double>(inliers.size());
      cx *= inv_count;
      cy *= inv_count;
      cz *= inv_count;

      double xx = 0.0;
      double xy = 0.0;
      double xz = 0.0;
      double yy = 0.0;
      double yz = 0.0;
      double zz = 0.0;
      for (const std::uint32_t index : inliers) {
        const double dx = static_cast<double>(points[index][0]) - cx;
        const double dy = static_cast<double>(points[index][1]) - cy;
        const double dz = static_cast<double>(points[index][2]) - cz;
        xx += dx * dx;
        xy += dx * dy;
        xz += dx * dz;
        yy += dy * dy;
        yz += dy * dz;
        zz += dz * dz;
      }

      std::array<double, 3> normal = smallest_eigenvector(xx, xy, xz, yy, yz, zz);
      const double norm =
        std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
      if (!(norm > 0.0)) {
        fit_ok = false;
        break;
      }
      if (normal[2] < 0.0) {
        normal = {-normal[0], -normal[1], -normal[2]};
      }
      // A near-horizontal normal means the cell's dominant plane is a wall
      // face, not ground; trusting it would label the whole wall ground.
      if (normal[2] / norm < config.min_normal_z) {
        fit_ok = false;
        break;
      }

      inliers.clear();
      for (const std::uint32_t index : members) {
        const double dx = static_cast<double>(points[index][0]) - cx;
        const double dy = static_cast<double>(points[index][1]) - cy;
        const double dz = static_cast<double>(points[index][2]) - cz;
        const double distance = (normal[0] * dx + normal[1] * dy + normal[2] * dz) / norm;
        if (distance < config.inlier_margin) {
          inliers.push_back(index);
        }
      }
      if (inliers.size() < kMinFitPoints) {
        fit_ok = false;
        break;
      }
    }

    if (!fit_ok) {
      apply_seed_rule();
      continue;
    }
    for (const std::uint32_t index : members) {
      ground[index] = 0U;
    }
    for (const std::uint32_t index : inliers) {
      ground[index] = 1U;
    }
    ground_count += inliers.size();
  }
  return ground_count;
}

}  // namespace bagwiz::core::slam
