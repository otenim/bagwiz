// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__POINT_RASTER_HPP_
#define BAGWIZ__CORE__POINTCLOUD__POINT_RASTER_HPP_

#include "bagwiz/core/pointcloud/color_mapper.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Draws projected points into a packed BGR24 canvas with a depth test. Each
// point paints a point_size x point_size square centered on its pixel
// (clipped at the canvas edges) in the color of its `value`; a pixel keeps
// the point with the smallest `depth` — the one nearest a perspective camera,
// or, when the projection wrote depth = -z, the highest point of a bird's-eye
// view. Because depth is resolved per pixel, draw order does not matter and
// no sort is needed, unlike an overlay painted onto a camera image.
namespace bagwiz::core::pointcloud
{

class PointRaster
{
public:
  // Size the canvas for a frame: every pixel goes black and every depth to
  // +infinity. The buffers are kept between frames of the same size.
  void reset(std::uint32_t width, std::uint32_t height);

  // Draw `points` (from a projection onto a canvas of this size) with the
  // color `mapper` gives their `value` over [value_min, value_max], as
  // squares of `point_size` pixels (at least 1). Points whose square lies
  // entirely off the canvas are skipped.
  void draw(
    std::span<const ProjectedPoint> points, const ColorMapper & mapper, double value_min,
    double value_max, std::uint32_t point_size);

  [[nodiscard]] std::uint32_t width() const noexcept { return width_; }
  [[nodiscard]] std::uint32_t height() const noexcept { return height_; }
  // The canvas: packed BGR24, width * 3 * height bytes.
  [[nodiscard]] const std::vector<std::byte> & bgr() const noexcept { return bgr_; }

private:
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  std::vector<std::byte> bgr_;
  std::vector<float> depth_;
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__POINT_RASTER_HPP_
