// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_raster.hpp"

#include "bagwiz/core/pointcloud/color_mapper.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::ColorMapper;
using bagwiz::core::pointcloud::ColorScheme;
using bagwiz::core::pointcloud::PointRaster;
using bagwiz::core::pointcloud::ProjectedPoint;

ProjectedPoint point(std::int32_t u, std::int32_t v, float depth, float value)
{
  ProjectedPoint p;
  p.u = u;
  p.v = v;
  p.depth = depth;
  p.value = value;
  return p;
}

// The BGR bytes of pixel (x, y).
std::array<std::uint8_t, 3> pixel(const PointRaster & raster, std::uint32_t x, std::uint32_t y)
{
  const std::size_t idx = (static_cast<std::size_t>(y) * raster.width() + x) * 3U;
  return {
    static_cast<std::uint8_t>(raster.bgr()[idx]), static_cast<std::uint8_t>(raster.bgr()[idx + 1]),
    static_cast<std::uint8_t>(raster.bgr()[idx + 2])};
}

bool is_black(const std::array<std::uint8_t, 3> & px)
{
  return px[0] == 0 && px[1] == 0 && px[2] == 0;
}

// The mapper's color for `value` over [0, 1], so a test can name what a drawn
// point must look like without depending on the scheme's actual colors.
std::array<std::uint8_t, 3> color_of(const ColorMapper & mapper, double value)
{
  return mapper.map(value, 0.0, 1.0);
}

TEST(PointRasterTest, ResetSizesABlackCanvas)
{
  PointRaster raster;
  raster.reset(4, 2);
  EXPECT_EQ(raster.width(), 4u);
  EXPECT_EQ(raster.height(), 2u);
  ASSERT_EQ(raster.bgr().size(), 24u);
  for (const auto b : raster.bgr()) {
    EXPECT_EQ(b, std::byte{0});
  }
}

TEST(PointRasterTest, DrawsASinglePixelPointInItsValueColor)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  PointRaster raster;
  raster.reset(4, 4);
  const std::vector<ProjectedPoint> points{point(1, 2, 5.0F, 1.0F)};
  raster.draw(points, mapper, 0.0, 1.0, 1);
  EXPECT_EQ(pixel(raster, 1, 2), color_of(mapper, 1.0));
  // Every other pixel stays black.
  for (std::uint32_t y = 0; y < 4; ++y) {
    for (std::uint32_t x = 0; x < 4; ++x) {
      if (x == 1 && y == 2) {
        continue;
      }
      EXPECT_TRUE(is_black(pixel(raster, x, y))) << x << "," << y;
    }
  }
}

TEST(PointRasterTest, NearestPointWinsAContestedPixelRegardlessOfDrawOrder)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  const auto near_color = color_of(mapper, 1.0);
  const auto far_color = color_of(mapper, 0.0);
  ASSERT_NE(near_color, far_color);

  for (const bool near_first : {true, false}) {
    PointRaster raster;
    raster.reset(3, 3);
    std::vector<ProjectedPoint> points{point(1, 1, 2.0F, 1.0F), point(1, 1, 9.0F, 0.0F)};
    if (!near_first) {
      std::swap(points[0], points[1]);
    }
    raster.draw(points, mapper, 0.0, 1.0, 1);
    EXPECT_EQ(pixel(raster, 1, 1), near_color) << (near_first ? "near first" : "far first");
  }
}

TEST(PointRasterTest, DepthPersistsAcrossDrawCallsUntilReset)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  PointRaster raster;
  raster.reset(2, 2);
  const std::vector<ProjectedPoint> near{point(0, 0, 1.0F, 1.0F)};
  const std::vector<ProjectedPoint> far{point(0, 0, 3.0F, 0.0F)};
  raster.draw(near, mapper, 0.0, 1.0, 1);
  raster.draw(far, mapper, 0.0, 1.0, 1);  // a later, farther cloud cannot overwrite
  EXPECT_EQ(pixel(raster, 0, 0), color_of(mapper, 1.0));
  raster.reset(2, 2);
  raster.draw(far, mapper, 0.0, 1.0, 1);
  EXPECT_EQ(pixel(raster, 0, 0), color_of(mapper, 0.0));
}

TEST(PointRasterTest, PointSizeGrowsTheSquareByOnePixelPerUnit)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  // A size-3 point centered on (3, 3) covers x, y in [2, 4]; size 2 covers
  // [2, 3] (its top-left offset is size / 2 = 1).
  for (const std::uint32_t size : {2U, 3U}) {
    PointRaster raster;
    raster.reset(7, 7);
    const std::vector<ProjectedPoint> points{point(3, 3, 1.0F, 1.0F)};
    raster.draw(points, mapper, 0.0, 1.0, size);
    std::uint32_t painted = 0;
    for (std::uint32_t y = 0; y < 7; ++y) {
      for (std::uint32_t x = 0; x < 7; ++x) {
        const bool inside = x >= 2 && x < 2 + size && y >= 2 && y < 2 + size;
        EXPECT_EQ(!is_black(pixel(raster, x, y)), inside)
          << "size " << size << " at " << x << "," << y;
        painted += is_black(pixel(raster, x, y)) ? 0 : 1;
      }
    }
    EXPECT_EQ(painted, size * size);
  }
}

TEST(PointRasterTest, ClipsSquaresAtTheCanvasEdges)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  PointRaster raster;
  raster.reset(3, 3);
  // A size-3 square centered on the corner pixel paints only the 2x2 block
  // that lies on the canvas; a point far outside paints nothing.
  const std::vector<ProjectedPoint> points{point(0, 0, 1.0F, 1.0F), point(10, 10, 1.0F, 1.0F)};
  raster.draw(points, mapper, 0.0, 1.0, 3);
  std::uint32_t painted = 0;
  for (std::uint32_t y = 0; y < 3; ++y) {
    for (std::uint32_t x = 0; x < 3; ++x) {
      painted += is_black(pixel(raster, x, y)) ? 0 : 1;
    }
  }
  EXPECT_EQ(painted, 4u);
  EXPECT_FALSE(is_black(pixel(raster, 1, 1)));
  EXPECT_TRUE(is_black(pixel(raster, 2, 2)));
}

TEST(PointRasterTest, SizeZeroDrawsOnePixel)
{
  const ColorMapper mapper(ColorScheme::kViridis);
  PointRaster raster;
  raster.reset(3, 3);
  const std::vector<ProjectedPoint> points{point(1, 1, 1.0F, 1.0F)};
  raster.draw(points, mapper, 0.0, 1.0, 0);
  std::uint32_t painted = 0;
  for (const auto b : raster.bgr()) {
    painted += b == std::byte{0} ? 0 : 1;
  }
  EXPECT_GT(painted, 0u);
  EXPECT_TRUE(is_black(pixel(raster, 0, 0)));
  EXPECT_TRUE(is_black(pixel(raster, 2, 2)));
}

}  // namespace
