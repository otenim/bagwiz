// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/instance_occupancy.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

// GLIM-free unit tests for the ERASOR2-style instance-aware pseudo-occupancy
// classifier behind `map slam --remove-dynamic --dynamic-method erasor2`. The
// scenes are regular lattices whose per-cell statistics are known by
// construction, so every posterior below is hand-computable: a cell hit by the
// ratio test in k scans accumulates k * case_gain * base_increment log-odds
// (0.3 each with the test config), a ground-dominant cell k * base_increment.
namespace
{
namespace slam = bagwiz::core::slam;

using Point = std::array<float, 3>;
using Origin = std::array<double, 3>;

// All scans share one origin 1.7 m above ground level z = 0.
constexpr Origin kOrigin{0.0, 0.0, 1.7};

slam::InstanceOccupancyConfig make_config()
{
  slam::InstanceOccupancyConfig config;
  config.sensor_height = 1.7;  // ground level exactly z = 0 under kOrigin
  config.max_radius = 50.0;
  // 5 m instead of 30 m so the test car ~8.5 m out takes the far threshold
  // (0.75); the near threshold would demand ~46 ratio-test hits.
  config.near_far_distance = 5.0;
  return config;
}

// Regular x-y lattice on the plane z = z0, `spacing` meters apart.
std::vector<Point> make_plane(double x0, double x1, double y0, double y1, double spacing, double z0)
{
  std::vector<Point> points;
  for (double x = x0; x < x1 - 1e-9; x += spacing) {
    for (double y = y0; y < y1 - 1e-9; y += spacing) {
      points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z0)});
    }
  }
  return points;
}

// Dense box of points spanning [x0, x1) x [y0, y1) x [z0, z1).
std::vector<Point> make_box(
  double x0, double x1, double y0, double y1, double z0, double z1, double spacing)
{
  std::vector<Point> points;
  for (double x = x0; x < x1 - 1e-9; x += spacing) {
    for (double y = y0; y < y1 - 1e-9; y += spacing) {
      for (double z = z0; z < z1 - 1e-9; z += spacing) {
        points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    }
  }
  return points;
}

void append(std::vector<Point> & to, const std::vector<Point> & from)
{
  to.insert(to.end(), from.begin(), from.end());
}

// Ground x in [4, 12), y in [-4, 4) at z = 0 (25 points per 0.5 m cell), plus
// a wall at x in [5.0, 5.4), y in [2, 4), z in [0, 2).
std::vector<Point> make_static_scan()
{
  auto scan = make_plane(4.0, 12.0, -4.0, 4.0, 0.1, 0.0);
  append(scan, make_box(5.0, 5.4, 2.0, 4.0, 0.0, 2.0, 0.2));
  return scan;
}

// Car body spanning exactly cells x 15..18, y -2..1 at 0.5 m cell size,
// hovering 0.4 m over the ground (above the sqrt(3) * 0.2 = 0.346 m erosion
// radius, so the ground beneath survives the erosion around it).
std::vector<Point> make_car()
{
  return make_box(7.5, 9.5, -1.0, 1.0, 0.4, 1.8, 0.25);
}

// Drive all five phases and return one keep mask per scan.
std::vector<std::vector<std::uint8_t>> run_classifier(
  const slam::InstanceOccupancyConfig & config, const std::vector<std::vector<Point>> & scans,
  int num_threads = 1, bool reverse_order = false)
{
  slam::InstanceOccupancyClassifier classifier(config, scans.size());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const std::size_t id = reverse_order ? scans.size() - 1 - i : i;
    classifier.add_scan(id, scans[id], kOrigin);
  }
  classifier.finalize_grid(num_threads);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    const std::size_t id = reverse_order ? scans.size() - 1 - i : i;
    classifier.propose(id, scans[id]);
  }
  classifier.finalize_proposals();
  std::vector<std::vector<std::uint8_t>> keep(scans.size());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    keep[i].assign(scans[i].size(), 2U);  // 2 = untouched sentinel
    classifier.classify(i, scans[i], keep[i]);
  }
  return keep;
}

// 5 scans with a parked car that then leaves for 15 scans.
std::vector<std::vector<Point>> make_transient_car_scans()
{
  std::vector<std::vector<Point>> scans;
  for (int t = 0; t < 20; ++t) {
    auto scan = make_static_scan();
    if (t < 5) {
      append(scan, make_car());
    }
    scans.push_back(std::move(scan));
  }
  return scans;
}

TEST(InstanceOccupancy, TransientObjectIsRemovedFromItsScans)
{
  const auto scans = make_transient_car_scans();
  const std::size_t static_size = make_static_scan().size();
  const auto keep = run_classifier(make_config(), scans);

  // Scans 0..4: exactly the car points (appended after the static scene) drop.
  for (std::size_t t = 0; t < 5; ++t) {
    for (std::size_t i = 0; i < static_size; ++i) {
      EXPECT_EQ(keep[t][i], 1U) << "static point " << i << " of scan " << t;
    }
    for (std::size_t i = static_size; i < scans[t].size(); ++i) {
      EXPECT_EQ(keep[t][i], 0U) << "car point " << i << " of scan " << t;
    }
  }
  // Scans 5..19 hold only static structure; nothing may drop.
  for (std::size_t t = 5; t < scans.size(); ++t) {
    for (std::size_t i = 0; i < scans[t].size(); ++i) {
      EXPECT_EQ(keep[t][i], 1U) << "static point " << i << " of scan " << t;
    }
  }
}

TEST(InstanceOccupancy, StaticSceneIsUntouched)
{
  std::vector<std::vector<Point>> scans(20, make_static_scan());
  const auto keep = run_classifier(make_config(), scans);
  for (std::size_t t = 0; t < scans.size(); ++t) {
    for (std::size_t i = 0; i < scans[t].size(); ++i) {
      EXPECT_EQ(keep[t][i], 1U) << "point " << i << " of scan " << t;
    }
  }
}

TEST(InstanceOccupancy, SingleScanRemovesNothing)
{
  // One scan carries no re-observation evidence: the map IS the query, every
  // profile ratio is 1, and no posterior can clear the proposal threshold.
  auto scan = make_static_scan();
  append(scan, make_car());
  const std::vector<std::vector<Point>> scans{scan};
  const auto keep = run_classifier(make_config(), scans);
  for (std::size_t i = 0; i < scans[0].size(); ++i) {
    EXPECT_EQ(keep[0][i], 1U) << "point " << i;
  }
}

TEST(InstanceOccupancy, DeterministicAcrossThreadCountsAndScanOrder)
{
  const auto scans = make_transient_car_scans();
  const auto forward = run_classifier(make_config(), scans, 1, false);
  const auto reversed = run_classifier(make_config(), scans, 8, true);
  ASSERT_EQ(forward.size(), reversed.size());
  for (std::size_t t = 0; t < forward.size(); ++t) {
    EXPECT_EQ(forward[t], reversed[t]) << "scan " << t;
  }
}

TEST(InstanceOccupancy, CellPosteriorDiagnostics)
{
  const auto scans = make_transient_car_scans();
  slam::InstanceOccupancyClassifier classifier(make_config(), scans.size());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    classifier.add_scan(i, scans[i], kOrigin);
  }
  classifier.finalize_grid(2);

  // Car cells: 15 vacated observations at +0.3 log-odds = 4.5.
  EXPECT_NEAR(classifier.cell_posterior(8.5, 0.0), 1.0 / (1.0 + std::exp(-4.5)), 1e-3);
  // Wall cells: occupied in every scan, no evidence either way.
  EXPECT_DOUBLE_EQ(classifier.cell_posterior(5.2, 3.0), 0.5);
  // Pure-ground cells: 20 mild observations at +0.15 log-odds = 3.0.
  EXPECT_NEAR(classifier.cell_posterior(10.0, -3.0), 1.0 / (1.0 + std::exp(-3.0)), 1e-3);
  // Cells never observed stay at the prior.
  EXPECT_DOUBLE_EQ(classifier.cell_posterior(100.0, 100.0), 0.5);
  EXPECT_GT(classifier.proposed_cell_count(), 0U);
}

TEST(InstanceOccupancy, PointsOutsideTheVolumeOfInterestAreNeverRemoved)
{
  // A "pole" over the car sticking out above the height band, and a point
  // beyond the radial bound: both are outside the analysis volume and must
  // survive even though the car below/near them is removed.
  auto scans = make_transient_car_scans();
  const std::size_t base_size = scans[0].size();
  for (std::size_t t = 0; t < 5; ++t) {
    scans[t].push_back({8.5F, 0.0F, 3.5F});   // above height_max = 3 over ground 0
    scans[t].push_back({60.0F, 0.0F, 1.0F});  // beyond max_radius = 50
  }
  const auto keep = run_classifier(make_config(), scans);
  for (std::size_t t = 0; t < 5; ++t) {
    EXPECT_EQ(keep[t][base_size], 1U) << "pole point of scan " << t;
    EXPECT_EQ(keep[t][base_size + 1], 1U) << "far point of scan " << t;
  }
}

TEST(InstanceOccupancy, UnderSegmentationSplitsAMergedCluster)
{
  // A big static wall, a transient car, and a thin bridge of points welding
  // them into ONE cluster: the merged instance's average posterior stays under
  // the drop threshold (the wall dominates), so only the under-segmentation
  // check can — and must — carve out the points on the car's high-posterior
  // cells while keeping the wall and the bridge. "Wheels" under the car
  // (ground-labeled, so never part of any cluster) prove the carved points
  // seed the volumetric erosion like a normally-clustered removal would. The
  // bridge points stay farther than the erosion radius (0.346 m) from every
  // car point, so the weld itself must survive.
  const auto ground = make_plane(4.0, 12.0, -4.0, 4.0, 0.1, 0.0);
  const auto wall = make_box(5.0, 5.4, 0.0, 4.0, 0.0, 2.6, 0.13);
  const auto car = make_box(7.0, 9.0, 0.0, 2.0, 0.4, 1.8, 0.25);
  const auto wheels = make_box(7.25, 8.1, 0.5, 1.35, 0.12, 0.13, 0.25);
  const std::vector<Point> bridge{
    {5.5F, 1.0F, 1.0F}, {5.9F, 1.0F, 1.0F}, {6.3F, 1.0F, 1.0F}, {6.65F, 1.0F, 1.0F}};

  std::vector<std::vector<Point>> scans;
  std::vector<std::array<std::size_t, 2>> car_span;  // [begin, end) per scan
  for (int t = 0; t < 20; ++t) {
    std::vector<Point> scan = ground;
    append(scan, wall);
    append(scan, bridge);
    const std::size_t begin = scan.size();
    if (t < 5) {
      append(scan, car);
      append(scan, wheels);
    }
    car_span.push_back({begin, scan.size()});
    scans.push_back(std::move(scan));
  }

  auto config = make_config();
  config.usc_area = 3.0;  // the merged footprint (~6.8 m^2) must clear it
  const auto keep = run_classifier(config, scans);

  for (std::size_t t = 0; t < scans.size(); ++t) {
    for (std::size_t i = 0; i < car_span[t][0]; ++i) {
      EXPECT_EQ(keep[t][i], 1U) << "static point " << i << " of scan " << t;
    }
    for (std::size_t i = car_span[t][0]; i < car_span[t][1]; ++i) {
      EXPECT_EQ(keep[t][i], 0U) << "car/wheel point " << i << " of scan " << t;
    }
  }
}

TEST(InstanceOccupancy, PhaseContractViolationsThrow)
{
  // The five-phase contract must fail loudly in every build type, not read
  // out of bounds when asserts are compiled out.
  slam::InstanceOccupancyClassifier classifier(make_config(), 1);
  const std::vector<Point> points{{1.0F, 1.0F, 0.0F}};
  std::vector<std::uint8_t> keep(points.size(), 1U);

  EXPECT_THROW(classifier.propose(0, points), std::logic_error);
  EXPECT_THROW(classifier.classify(0, points, keep), std::logic_error);
  EXPECT_THROW(classifier.add_scan(1, points, kOrigin), std::invalid_argument);
  classifier.add_scan(0, points, kOrigin);
  EXPECT_THROW(classifier.add_scan(0, points, kOrigin), std::logic_error);
  classifier.finalize_grid(1);
  EXPECT_THROW(classifier.finalize_grid(1), std::logic_error);
  EXPECT_THROW(classifier.finalize_proposals(), std::logic_error);  // nothing proposed yet
  classifier.propose(0, points);
  EXPECT_THROW(classifier.propose(0, points), std::logic_error);
  classifier.finalize_proposals();
  std::vector<std::uint8_t> undersized;
  EXPECT_THROW(classifier.classify(0, points, undersized), std::invalid_argument);
  EXPECT_EQ(classifier.classify(0, points, keep), 0U);
}

TEST(InstanceOccupancy, HugeCoordinatesAreOutsideTheAnalysis)
{
  // Finite but absurd coordinates (a corrupt cloud) must neither be removed
  // nor derail the grid: they fall outside the volume of interest by the
  // binnable-coordinate guard.
  auto scans = make_transient_car_scans();
  const std::size_t base_size = scans[0].size();
  for (std::size_t t = 0; t < scans.size(); ++t) {
    scans[t].push_back({1.0e30F, 1.0F, 0.5F});
  }
  const auto keep = run_classifier(make_config(), scans);
  for (std::size_t t = 0; t < scans.size(); ++t) {
    EXPECT_EQ(keep[t][base_size], 1U) << "huge point of scan " << t;
  }
}

TEST(InstanceOccupancy, VolumetricErosionSweepsGroundLabeledStragglers)
{
  // "Wheels" 0.12 m over the ground directly under the car body: labeled
  // ground by the segmentation (inside the 0.15 m inlier margin), so they can
  // never join the car instance — only the volumetric erosion around the
  // confidently-removed car body (0.28 m above them) can sweep them.
  auto scans = make_transient_car_scans();
  std::vector<std::array<std::size_t, 2>> wheel_span(scans.size(), {0, 0});
  for (std::size_t t = 0; t < 5; ++t) {
    const std::size_t begin = scans[t].size();
    append(scans[t], make_box(7.75, 8.6, -0.5, 0.35, 0.12, 0.13, 0.25));
    wheel_span[t] = {begin, scans[t].size()};
  }

  const auto keep = run_classifier(make_config(), scans);
  for (std::size_t t = 0; t < 5; ++t) {
    ASSERT_GT(wheel_span[t][1], wheel_span[t][0]);
    for (std::size_t i = wheel_span[t][0]; i < wheel_span[t][1]; ++i) {
      EXPECT_EQ(keep[t][i], 0U) << "wheel point " << i << " of scan " << t;
    }
  }

  // With the erosion barred from ground-labeled points, the wheels survive.
  auto config = make_config();
  config.vor_erode_ground = false;
  const auto keep_no_ground = run_classifier(config, scans);
  for (std::size_t t = 0; t < 5; ++t) {
    for (std::size_t i = wheel_span[t][0]; i < wheel_span[t][1]; ++i) {
      EXPECT_EQ(keep_no_ground[t][i], 1U) << "wheel point " << i << " of scan " << t;
    }
  }
}

TEST(InstanceOccupancy, EmptyScansAreHandled)
{
  std::vector<std::vector<Point>> scans(3, make_static_scan());
  scans[1].clear();
  const auto keep = run_classifier(make_config(), scans);
  EXPECT_TRUE(keep[1].empty());
  for (std::size_t i = 0; i < scans[0].size(); ++i) {
    EXPECT_EQ(keep[0][i], 1U);
    EXPECT_EQ(keep[2][i], 1U);
  }
}

}  // namespace
