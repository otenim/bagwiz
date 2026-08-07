// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/euclidean_clustering.hpp"

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
// Same guard as VoxelGrid: never divide by a non-positive tolerance.
constexpr double kMinTolerance = 1e-3;

// Teschner et al. spatial-hash constants, as in cloud_filters.cpp / dynamic_removal.cpp.
constexpr std::size_t kHashX = 73856093U;
constexpr std::size_t kHashY = 19349663U;
constexpr std::size_t kHashZ = 83492791U;

struct CellKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;
  bool operator==(const CellKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    const auto mix = [](std::int32_t index, std::size_t multiplier) {
      return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
    };
    return mix(key.x, kHashX) ^ mix(key.y, kHashY) ^ mix(key.z, kHashZ);
  }
};

// Coordinates whose scaled magnitude exceeds this cannot be binned into an
// int32 voxel index (the float-to-int conversion would be undefined); such
// finite-but-absurd points are treated like non-finite ones.
constexpr double kMaxBinnableIndex = 2.0e9;

bool binnable_point(const std::array<float, 3> & point, double inv_tolerance)
{
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]) &&
         std::abs(static_cast<double>(point[0])) * inv_tolerance < kMaxBinnableIndex &&
         std::abs(static_cast<double>(point[1])) * inv_tolerance < kMaxBinnableIndex &&
         std::abs(static_cast<double>(point[2])) * inv_tolerance < kMaxBinnableIndex;
}

}  // namespace

std::size_t cluster_points(
  std::span<const std::array<float, 3>> points, std::span<const std::uint8_t> mask,
  const EuclideanClusteringConfig & config, std::span<std::uint32_t> cluster_ids)
{
  assert(mask.size() >= points.size());
  assert(cluster_ids.size() >= points.size());
  std::fill(
    cluster_ids.begin(), cluster_ids.begin() + static_cast<std::ptrdiff_t>(points.size()), 0U);

  const double tolerance = std::max(config.tolerance, kMinTolerance);
  const double tolerance_sq = tolerance * tolerance;
  const double inv_tolerance = 1.0 / tolerance;
  const std::size_t min_cluster_size =
    static_cast<std::size_t>(std::max(config.min_cluster_size, 1));

  // Bucket the eligible points into cubic voxels of side `tolerance`: any two
  // points within the tolerance are then at most one voxel index apart on each
  // axis, so a candidate's neighbors all live in the 27 surrounding voxels.
  const auto cell_of = [inv_tolerance](const std::array<float, 3> & point) {
    return CellKey{
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[0]) * inv_tolerance)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[1]) * inv_tolerance)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[2]) * inv_tolerance))};
  };
  std::unordered_map<CellKey, std::vector<std::uint32_t>, CellKeyHash> cells;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (mask[i] == 0U || !binnable_point(points[i], inv_tolerance)) {
      continue;
    }
    cells[cell_of(points[i])].push_back(static_cast<std::uint32_t>(i));
  }

  // Grow connected components by BFS, seeding in ascending point index. The
  // components themselves are order-independent (connectivity is symmetric and
  // transitive), and the ascending seeds make the id numbering canonical: the
  // component whose smallest member comes first gets the smaller id.
  std::vector<std::uint8_t> visited(points.size(), 0U);
  std::vector<std::uint32_t> component;
  std::size_t cluster_count = 0;
  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed] != 0U || mask[seed] == 0U || !binnable_point(points[seed], inv_tolerance)) {
      continue;
    }
    component.clear();
    component.push_back(static_cast<std::uint32_t>(seed));
    visited[seed] = 1U;
    for (std::size_t next = 0; next < component.size(); ++next) {
      const std::array<float, 3> & center = points[component[next]];
      const CellKey base = cell_of(center);
      for (std::int32_t ox = -1; ox <= 1; ++ox) {
        for (std::int32_t oy = -1; oy <= 1; ++oy) {
          for (std::int32_t oz = -1; oz <= 1; ++oz) {
            const auto found = cells.find({base.x + ox, base.y + oy, base.z + oz});
            if (found == cells.end()) {
              continue;
            }
            for (const std::uint32_t candidate : found->second) {
              if (visited[candidate] != 0U) {
                continue;
              }
              const double dx =
                static_cast<double>(points[candidate][0]) - static_cast<double>(center[0]);
              const double dy =
                static_cast<double>(points[candidate][1]) - static_cast<double>(center[1]);
              const double dz =
                static_cast<double>(points[candidate][2]) - static_cast<double>(center[2]);
              if (dx * dx + dy * dy + dz * dz <= tolerance_sq) {
                visited[candidate] = 1U;
                component.push_back(candidate);
              }
            }
          }
        }
      }
    }
    if (component.size() < min_cluster_size) {
      continue;  // stays id 0
    }
    ++cluster_count;
    for (const std::uint32_t member : component) {
      cluster_ids[member] = static_cast<std::uint32_t>(cluster_count);
    }
  }
  return cluster_count;
}

}  // namespace bagwiz::core::slam
