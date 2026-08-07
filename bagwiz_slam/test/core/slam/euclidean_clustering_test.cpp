// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/euclidean_clustering.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

// GLIM-free unit tests for the single-linkage Euclidean clustering behind the
// erasor2 dynamic-removal method's instance grouping. Scenes use hand-placed
// points so every expected component is known by construction.
namespace
{
namespace slam = bagwiz::core::slam;

using Point = std::array<float, 3>;

// Dense cube of side `side` centered at (cx, cy, cz), `spacing` apart; every
// nearest-neighbor gap is `spacing`, so the cube is one connected component
// whenever spacing <= tolerance.
std::vector<Point> make_blob(double cx, double cy, double cz, double side, double spacing)
{
  std::vector<Point> points;
  for (double x = -side / 2; x <= side / 2; x += spacing) {
    for (double y = -side / 2; y <= side / 2; y += spacing) {
      for (double z = -side / 2; z <= side / 2; z += spacing) {
        points.push_back(
          {static_cast<float>(cx + x), static_cast<float>(cy + y), static_cast<float>(cz + z)});
      }
    }
  }
  return points;
}

std::vector<std::uint32_t> run(
  const std::vector<Point> & points, const slam::EuclideanClusteringConfig & config,
  std::size_t & cluster_count, const std::vector<std::uint8_t> * mask_in = nullptr)
{
  const std::vector<std::uint8_t> mask =
    mask_in != nullptr ? *mask_in : std::vector<std::uint8_t>(points.size(), 1U);
  std::vector<std::uint32_t> ids(points.size(), 999U);  // sentinel: must be overwritten
  cluster_count = slam::cluster_points(points, mask, config, ids);
  return ids;
}

TEST(ClusterPoints, TwoSeparatedBlobsFormTwoClusters)
{
  auto points = make_blob(0.0, 0.0, 0.0, 1.0, 0.25);
  const std::size_t first_size = points.size();
  const auto second = make_blob(5.0, 0.0, 0.0, 1.0, 0.25);
  points.insert(points.end(), second.begin(), second.end());

  slam::EuclideanClusteringConfig config;
  std::size_t cluster_count = 0;
  const auto ids = run(points, config, cluster_count);

  EXPECT_EQ(cluster_count, 2U);
  // Canonical numbering: the cluster containing index 0 is id 1, the later one id 2.
  for (std::size_t i = 0; i < first_size; ++i) {
    EXPECT_EQ(ids[i], 1U) << "first blob point " << i;
  }
  for (std::size_t i = first_size; i < points.size(); ++i) {
    EXPECT_EQ(ids[i], 2U) << "second blob point " << i;
  }
}

TEST(ClusterPoints, ChainWithinToleranceIsOneCluster)
{
  // 30 points in a line, 0.4 m apart (single linkage connects transitively
  // even though the endpoints are 11.6 m apart).
  std::vector<Point> points;
  for (int i = 0; i < 30; ++i) {
    points.push_back({static_cast<float>(0.4 * i), 0.0F, 0.0F});
  }
  slam::EuclideanClusteringConfig config;
  std::size_t cluster_count = 0;
  const auto ids = run(points, config, cluster_count);
  EXPECT_EQ(cluster_count, 1U);
  for (const auto id : ids) {
    EXPECT_EQ(id, 1U);
  }
}

TEST(ClusterPoints, ComponentSmallerThanMinSizeStaysUnclustered)
{
  auto points = make_blob(0.0, 0.0, 0.0, 1.0, 0.25);  // 125 points: a cluster
  points.push_back({10.0F, 0.0F, 0.0F});              // 3-point far component:
  points.push_back({10.2F, 0.0F, 0.0F});              // below min_cluster_size
  points.push_back({10.4F, 0.0F, 0.0F});

  slam::EuclideanClusteringConfig config;
  std::size_t cluster_count = 0;
  const auto ids = run(points, config, cluster_count);

  EXPECT_EQ(cluster_count, 1U);
  EXPECT_EQ(ids[points.size() - 3], 0U);
  EXPECT_EQ(ids[points.size() - 2], 0U);
  EXPECT_EQ(ids[points.size() - 1], 0U);
}

TEST(ClusterPoints, MaskedOutPointsNeitherClusterNorBridge)
{
  // Two blobs 1.2 m apart with a masked-out midpoint that would bridge them.
  auto points = make_blob(0.0, 0.0, 0.0, 0.5, 0.25);
  const std::size_t first_size = points.size();
  const auto second = make_blob(1.2, 0.0, 0.0, 0.5, 0.25);
  points.insert(points.end(), second.begin(), second.end());
  points.push_back({0.6F, 0.0F, 0.0F});  // the would-be bridge

  std::vector<std::uint8_t> mask(points.size(), 1U);
  mask[points.size() - 1] = 0U;

  slam::EuclideanClusteringConfig config;
  config.min_cluster_size = 5;
  std::size_t cluster_count = 0;
  const auto ids = run(points, config, cluster_count, &mask);

  EXPECT_EQ(cluster_count, 2U);
  EXPECT_EQ(ids[points.size() - 1], 0U);
  EXPECT_EQ(ids[0], 1U);
  EXPECT_EQ(ids[first_size], 2U);
}

TEST(ClusterPoints, NonFinitePointsAreUnclustered)
{
  auto points = make_blob(0.0, 0.0, 0.0, 1.0, 0.25);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  points.push_back({nan, 0.0F, 0.0F});

  slam::EuclideanClusteringConfig config;
  std::size_t cluster_count = 0;
  const auto ids = run(points, config, cluster_count);
  EXPECT_EQ(cluster_count, 1U);
  EXPECT_EQ(ids[points.size() - 1], 0U);
}

TEST(ClusterPoints, EmptyInputReturnsZero)
{
  slam::EuclideanClusteringConfig config;
  std::vector<Point> points;
  std::vector<std::uint8_t> mask;
  std::vector<std::uint32_t> ids;
  EXPECT_EQ(slam::cluster_points(points, mask, config, ids), 0U);
}

}  // namespace
