// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__EUCLIDEAN_CLUSTERING_HPP_
#define BAGWIZ__CORE__SLAM__EUCLIDEAN_CLUSTERING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// GLIM-free and Eigen-free single-linkage Euclidean clustering: two points are
// connected when their distance is at most the tolerance, and a cluster is a
// connected component of that graph with at least the minimum size. Used by the
// ERASOR2-style dynamic-point removal to group a scan's non-ground points into
// object instances so the keep/drop decision is instance-level, not per-point
// (ERASOR2, Lim et al., RSS 2023, leaves the segmenter pluggable; this is the
// built-in one). Kept in the plain bagwiz_slam library (like cloud_filters.hpp)
// so it builds and unit-tests in every configuration.
namespace bagwiz::core::slam
{

struct EuclideanClusteringConfig
{
  // Neighbor distance [m]: points at most this far apart join the same
  // cluster (single linkage, so chains connect transitively). Must be > 0
  // (clamped to a tiny epsilon otherwise).
  double tolerance = 0.5;

  // Connected components smaller than this stay unclustered (id 0): they are
  // too small to score as an object instance. Values < 1 are treated as 1.
  int min_cluster_size = 10;
};

// Assign 1-based cluster ids to the masked-in points (`mask[i] != 0`); every
// masked-out, non-finite (or too far out for the int32 voxel binning), or
// too-small-component point gets id 0. Ids are
// canonical: clusters are numbered in order of their smallest member index, so
// the labeling depends only on the point set and mask, never on internal
// iteration order. `mask.size()` and `cluster_ids.size()` must be >=
// `points.size()`. Returns the number of clusters. Pure and single-threaded;
// concurrent calls on disjoint outputs are safe.
std::size_t cluster_points(
  std::span<const std::array<float, 3>> points, std::span<const std::uint8_t> mask,
  const EuclideanClusteringConfig & config, std::span<std::uint32_t> cluster_ids);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__EUCLIDEAN_CLUSTERING_HPP_
