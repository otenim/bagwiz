// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_
#define BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_

#include <cstdint>
#include <span>
#include <vector>

namespace bagwiz::core::calib
{

// One projected map point: pixel coordinates u/v (already verified in
// [0,width) x [0,height)) and depth in meters. `cell` is scratch space that
// the depth cull fills with the point's cell index on its first pass and
// reuses on its second, so the cell is computed once per point instead of
// twice.
struct DepthCullPoint
{
  float u = 0.0F;
  float v = 0.0F;
  float depth = 0.0F;
  std::uint32_t cell = 0;
};

// The cell grid one depth cull works over: the nearest depth seen in each
// cell_px x cell_px pixel cell. reset() sizes it for an image and clears it,
// observe() folds a set of projected points in — writing each point's `cell`
// — merge() folds another grid of the same shape in, and keeps() is the
// per-point verdict once every point was observed. The nearest depth is a
// min-reduction, so the points may be observed in any order, in any number
// of batches, and into any number of grids merged afterwards, without
// changing a single verdict — which is what lets a caller project one image's
// points on several threads.
class DepthCullGrid
{
public:
  // Precondition: cell_px > 0.
  void reset(std::uint32_t width, std::uint32_t height, std::uint32_t cell_px);
  void observe(std::span<DepthCullPoint> points);
  // Per-cell min with `other`, which must have been reset() with the same
  // image size and cell size.
  void merge(const DepthCullGrid & other);
  [[nodiscard]] bool keeps(const DepthCullPoint & point, float margin_m) const
  {
    return point.depth <= nearest_[point.cell] + margin_m;
  }

private:
  std::uint32_t cell_px_ = 1;
  std::uint32_t grid_w_ = 0;
  std::vector<float> nearest_;
};

// Writes keep flags (1 = keep) into keep_out, parallel to points: one grid
// over all of `points`, then keeps() per point.
// Preconditions: points.size() == keep_out.size(); cell_px > 0.
void depth_cull_keep(
  std::span<DepthCullPoint> points, std::uint32_t width, std::uint32_t height,
  std::uint32_t cell_px, float margin_m, std::span<std::uint8_t> keep_out);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_
