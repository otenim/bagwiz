// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GROUND_SEGMENTATION_HPP_
#define BAGWIZ__CORE__SLAM__GROUND_SEGMENTATION_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// GLIM-free and Eigen-free per-cell ground/non-ground labeling, a native
// implementation of region-wise ground plane fitting in the style of ERASOR's
// R-GPF (Lim et al., RA-L 2021): the x-y plane is tiled into square cells, and
// each cell seeds a plane from its lowest points, then refines it with a few
// PCA iterations. Fitting per cell instead of globally keeps curbs and slopes
// locally planar. Kept in the plain bagwiz_slam library (like cloud_filters.hpp)
// so it builds and unit-tests in every configuration; the caller passes
// world-frame xyz points.
namespace bagwiz::core::slam
{

// Tuning for the per-cell plane fit. The ERASOR paper specifies the plane-fit
// recipe (lowest-point seeding, 3 PCA refinement rounds, inlier margin 0.15 m)
// but leaves the seeding constants and the cell size open; the defaults below
// are this implementation's choices, documented per field.
struct GroundSegmentationConfig
{
  // Side [m] of the square x-y cells the plane is fitted per. Coarser than the
  // occupancy analysis grid on purpose: a stable PCA plane fit needs area
  // support, while ground curvature bounds it from above. Must be > 0 (clamped
  // to a tiny epsilon otherwise).
  double cell_size = 2.0;

  // Seed band [m] above the mean of the cell's lowest points: points below
  // (lowest mean + seed_margin) form the initial plane inliers. ERASOR leaves
  // the value open; 0.4 m accepts curb-height structure into the first fit
  // without swallowing car bodies.
  double seed_margin = 0.4;

  // Number of lowest-z points averaged into the seed reference. Values < 1 are
  // treated as 1.
  int seed_count = 10;

  // One-sided inlier margin [m]: a point is ground when its signed distance
  // along the upward plane normal is below this (points UNDER the plane are
  // always ground). ERASOR's tau_g.
  double inlier_margin = 0.15;

  // Cells with fewer points than this skip the PCA fit and fall back to the
  // seed rule alone (z below lowest mean + seed_margin), since a plane through
  // a handful of points is dominated by noise. Values < 3 are treated as 3.
  int min_cell_points = 8;

  // PCA refinement rounds after seeding. ERASOR uses 3.
  int iterations = 3;

  // Reject fits whose upward normal z-component falls below this and fall back
  // to the seed rule for the cell: such a "plane" is a wall face, not ground
  // (0.7 still admits slopes up to ~45 degrees). Implementation choice; the
  // paper has no explicit guard.
  double min_normal_z = 0.7;
};

// Label every point ground (1) or non-ground (0). `ground.size()` must be
// >= `points.size()`; non-finite points — and finite ones too far out for the
// int32 cell binning — are labeled non-ground. Returns the number of ground
// points. Pure and single-threaded; the labeling depends only
// on the point set (not its order), so concurrent calls on disjoint outputs
// are safe.
std::size_t segment_ground(
  std::span<const std::array<float, 3>> points, const GroundSegmentationConfig & config,
  std::span<std::uint8_t> ground);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__GROUND_SEGMENTATION_HPP_
