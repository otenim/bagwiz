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
// depth_cull_keep fills with the point's cell index on its first pass and
// reuses on its second, so the cell is computed once per point instead of
// twice.
struct DepthCullPoint
{
  float u = 0.0F;
  float v = 0.0F;
  float depth = 0.0F;
  std::uint32_t cell = 0;
};

// Writes keep flags (1 = keep) into keep_out, parallel to points.
// Preconditions: points.size() == keep_out.size(); cell_px > 0.
void depth_cull_keep(
  std::span<DepthCullPoint> points, std::uint32_t width, std::uint32_t height,
  std::uint32_t cell_px, float margin_m, std::span<std::uint8_t> keep_out);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_
