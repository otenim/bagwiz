// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_raster.hpp"

#include <algorithm>
#include <limits>

namespace bagwiz::core::pointcloud
{

void PointRaster::reset(std::uint32_t width, std::uint32_t height)
{
  width_ = width;
  height_ = height;
  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  bgr_.assign(pixels * 3U, std::byte{0});
  depth_.assign(pixels, std::numeric_limits<float>::infinity());
}

void PointRaster::draw(
  std::span<const ProjectedPoint> points, const ColorMapper & mapper, double value_min,
  double value_max, std::uint32_t point_size)
{
  if (width_ == 0 || height_ == 0) {
    return;
  }
  const std::int32_t w = static_cast<std::int32_t>(width_);
  const std::int32_t h = static_cast<std::int32_t>(height_);
  // A square of `side` pixels whose top-left corner sits `half` pixels up and
  // left of the point's pixel, so the on-screen extent equals point_size and
  // odd and even sizes both grow by one pixel per unit.
  const std::int32_t side = std::max<std::int32_t>(1, static_cast<std::int32_t>(point_size));
  const std::int32_t half = side / 2;

  for (const auto & p : points) {
    const std::int32_t x0 = std::max<std::int32_t>(0, p.u - half);
    const std::int32_t y0 = std::max<std::int32_t>(0, p.v - half);
    const std::int32_t x1 = std::min<std::int32_t>(w, p.u - half + side);
    const std::int32_t y1 = std::min<std::int32_t>(h, p.v - half + side);
    if (x0 >= x1 || y0 >= y1) {
      continue;  // entirely off the canvas
    }
    const BgrColor color = mapper.map(p.value, value_min, value_max);
    for (std::int32_t y = y0; y < y1; ++y) {
      const std::size_t row = static_cast<std::size_t>(y) * width_;
      for (std::int32_t x = x0; x < x1; ++x) {
        const std::size_t idx = row + static_cast<std::size_t>(x);
        if (p.depth < depth_[idx]) {
          depth_[idx] = p.depth;
          std::byte * px = bgr_.data() + idx * 3U;
          px[0] = static_cast<std::byte>(color[0]);
          px[1] = static_cast<std::byte>(color[1]);
          px[2] = static_cast<std::byte>(color[2]);
        }
      }
    }
  }
}

}  // namespace bagwiz::core::pointcloud
