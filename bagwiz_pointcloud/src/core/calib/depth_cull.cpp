// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/depth_cull.hpp"

#include <cassert>
#include <limits>
#include <vector>

namespace bagwiz::core::calib
{

void DepthCullGrid::reset(std::uint32_t width, std::uint32_t height, std::uint32_t cell_px)
{
  assert(cell_px > 0);
  cell_px_ = cell_px;
  grid_w_ = (width + cell_px - 1) / cell_px;
  const std::uint32_t grid_h = (height + cell_px - 1) / cell_px;
  nearest_.assign(
    static_cast<std::size_t>(grid_w_) * grid_h, std::numeric_limits<float>::infinity());
}

void DepthCullGrid::observe(std::span<DepthCullPoint> points)
{
  // The cell count is bounded by (width/cell_px) * (height/cell_px) for real
  // image sizes, far below UINT32_MAX, so the scratch field never truncates.
  for (auto & p : points) {
    const auto cx = static_cast<std::uint32_t>(p.u) / cell_px_;
    const auto cy = static_cast<std::uint32_t>(p.v) / cell_px_;
    p.cell = cy * grid_w_ + cx;
    if (p.depth < nearest_[p.cell]) {
      nearest_[p.cell] = p.depth;
    }
  }
}

void depth_cull_keep(
  std::span<DepthCullPoint> points, std::uint32_t width, std::uint32_t height,
  std::uint32_t cell_px, float margin_m, std::span<std::uint8_t> keep_out)
{
  assert(points.size() == keep_out.size());
  DepthCullGrid grid;
  grid.reset(width, height, cell_px);
  grid.observe(points);
  for (std::size_t i = 0; i < points.size(); ++i) {
    keep_out[i] = grid.keeps(points[i], margin_m) ? 1 : 0;
  }
}

}  // namespace bagwiz::core::calib
