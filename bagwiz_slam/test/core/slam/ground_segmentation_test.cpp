// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/ground_segmentation.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// GLIM-free unit tests for the R-GPF-style per-cell ground labeling behind
// `map slam --remove-dynamic --dynamic-method erasor2`. Scenes are built from
// regular point lattices so the expected label of every point is known by
// construction.
namespace
{
namespace slam = bagwiz::core::slam;

using Point = std::array<float, 3>;

// Regular x-y lattice on the plane z = z0 + slope_x * x, `spacing` meters
// apart, spanning [x0, x1) x [y0, y1).
std::vector<Point> make_plane(
  double x0, double x1, double y0, double y1, double spacing, double z0, double slope_x = 0.0)
{
  std::vector<Point> points;
  for (double x = x0; x < x1; x += spacing) {
    for (double y = y0; y < y1; y += spacing) {
      points.push_back(
        {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z0 + slope_x * x)});
    }
  }
  return points;
}

// Dense box of points spanning [x0, x1) x [y0, y1) x [z0, z1).
std::vector<Point> make_box(
  double x0, double x1, double y0, double y1, double z0, double z1, double spacing)
{
  std::vector<Point> points;
  for (double x = x0; x < x1; x += spacing) {
    for (double y = y0; y < y1; y += spacing) {
      for (double z = z0; z < z1; z += spacing) {
        points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    }
  }
  return points;
}

std::vector<std::uint8_t> run(
  const std::vector<Point> & points, const slam::GroundSegmentationConfig & config,
  std::size_t & ground_count)
{
  std::vector<std::uint8_t> ground(points.size(), 2U);  // 2 = untouched sentinel
  ground_count = slam::segment_ground(points, config, ground);
  return ground;
}

TEST(SegmentGround, FlatPlaneIsAllGround)
{
  const auto points = make_plane(0.0, 4.0, 0.0, 4.0, 0.2, 0.0);
  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(points, config, ground_count);
  EXPECT_EQ(ground_count, points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(ground[i], 1U) << "point " << i;
  }
}

TEST(SegmentGround, BoxAbovePlaneIsNonGround)
{
  auto points = make_plane(0.0, 4.0, 0.0, 4.0, 0.2, 0.0);
  const std::size_t plane_count = points.size();
  const auto box = make_box(1.0, 2.0, 1.0, 2.0, 0.5, 1.5, 0.25);
  points.insert(points.end(), box.begin(), box.end());

  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(points, config, ground_count);

  EXPECT_EQ(ground_count, plane_count);
  for (std::size_t i = 0; i < plane_count; ++i) {
    EXPECT_EQ(ground[i], 1U) << "plane point " << i;
  }
  for (std::size_t i = plane_count; i < points.size(); ++i) {
    EXPECT_EQ(ground[i], 0U) << "box point " << i;
  }
}

TEST(SegmentGround, SlopedPlaneIsGround)
{
  // 10% grade: a global horizontal-plane fit would cut the far end off, the
  // per-cell fit must not.
  const auto points = make_plane(0.0, 8.0, 0.0, 2.0, 0.2, 0.0, 0.1);
  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(points, config, ground_count);
  EXPECT_EQ(ground_count, points.size());
}

TEST(SegmentGround, SparseCellFallsBackToTheSeedRule)
{
  // 4 points in one cell (below min_cell_points = 8): the two low points are
  // ground by the seed rule, the two high points are not.
  const std::vector<Point> points{
    {0.5F, 0.5F, 0.0F}, {1.5F, 1.5F, 0.05F}, {0.5F, 1.5F, 1.2F}, {1.5F, 0.5F, 1.5F}};
  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(points, config, ground_count);
  EXPECT_EQ(ground_count, 2U);
  EXPECT_EQ(ground[0], 1U);
  EXPECT_EQ(ground[1], 1U);
  EXPECT_EQ(ground[2], 0U);
  EXPECT_EQ(ground[3], 0U);
}

TEST(SegmentGround, WallCellDoesNotFitAVerticalPlane)
{
  // A cell holding only a wall face: the PCA "plane" is vertical, so the fit
  // must fall back to the seed rule instead of labeling the whole wall ground.
  const auto wall = make_box(0.4, 0.6, 0.0, 2.0, 0.0, 2.0, 0.1);
  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(wall, config, ground_count);
  // Only the seed band at the wall's foot may be ground.
  EXPECT_LT(ground_count, wall.size() / 2);
  for (std::size_t i = 0; i < wall.size(); ++i) {
    if (wall[i][2] > 0.5F) {
      EXPECT_EQ(ground[i], 0U) << "wall point " << i << " at z " << wall[i][2];
    }
  }
}

TEST(SegmentGround, NonFinitePointsAreNonGround)
{
  auto points = make_plane(0.0, 2.0, 0.0, 2.0, 0.2, 0.0);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  points.push_back({nan, 0.5F, 0.0F});
  points.push_back({0.5F, 0.5F, nan});
  slam::GroundSegmentationConfig config;
  std::size_t ground_count = 0;
  const auto ground = run(points, config, ground_count);
  EXPECT_EQ(ground_count, points.size() - 2);
  EXPECT_EQ(ground[points.size() - 2], 0U);
  EXPECT_EQ(ground[points.size() - 1], 0U);
}

TEST(SegmentGround, EmptyInputReturnsZero)
{
  slam::GroundSegmentationConfig config;
  std::vector<Point> points;
  std::vector<std::uint8_t> ground;
  EXPECT_EQ(slam::segment_ground(points, config, ground), 0U);
}

}  // namespace
