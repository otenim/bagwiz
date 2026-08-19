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

std::vector<std::uint8_t> depth_cull_keep(
  std::span<const float> u, std::span<const float> v, std::span<const float> depth,
  std::uint32_t width, std::uint32_t height, std::uint32_t cell_px, float margin_m)
{
  assert(u.size() == v.size() && v.size() == depth.size());
  assert(cell_px > 0);

  const std::uint32_t grid_w = (width + cell_px - 1) / cell_px;
  const std::uint32_t grid_h = (height + cell_px - 1) / cell_px;
  std::vector<float> nearest(
    static_cast<std::size_t>(grid_w) * grid_h, std::numeric_limits<float>::infinity());

  const auto cell_of = [&](std::size_t i) {
    const auto cx = static_cast<std::uint32_t>(u[i]) / cell_px;
    const auto cy = static_cast<std::uint32_t>(v[i]) / cell_px;
    return static_cast<std::size_t>(cy) * grid_w + cx;
  };

  for (std::size_t i = 0; i < depth.size(); ++i) {
    const std::size_t c = cell_of(i);
    if (depth[i] < nearest[c]) {
      nearest[c] = depth[i];
    }
  }
  std::vector<std::uint8_t> keep(depth.size(), 0);
  for (std::size_t i = 0; i < depth.size(); ++i) {
    keep[i] = depth[i] <= nearest[cell_of(i)] + margin_m ? 1 : 0;
  }
  return keep;
}

}  // namespace bagwiz::core::calib
