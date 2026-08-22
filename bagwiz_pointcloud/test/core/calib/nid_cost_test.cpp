// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nid_cost.hpp"

#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "correlated_scene.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace calib = bagwiz::core::calib;

TEST(NidCostTest, AlignedPoseScoresLowerThanPerturbedPose)
{
  const auto cam = test_camera();
  calib::NidParams params;
  params.min_points = 100;
  const auto sample = make_correlated_sample(cam, params.bins);
  const calib::Mat4 aligned = calib::identity_mat4();
  const calib::Mat4 yawed =
    calib::rigid_inverse(calib::make_transform({0, 0, 0}, {0, 0, 1.0 * M_PI / 180.0}));
  const auto nid_aligned = calib::nid_cost(sample, cam, aligned, params);
  const auto nid_yawed = calib::nid_cost(sample, cam, yawed, params);
  ASSERT_TRUE(nid_aligned.has_value());
  ASSERT_TRUE(nid_yawed.has_value());
  EXPECT_LT(*nid_aligned, *nid_yawed);
  EXPECT_GE(*nid_aligned, 0.0);
  EXPECT_LE(*nid_yawed, 1.0);
}

TEST(NidCostTest, TooFewSurvivingPointsReturnsNullopt)
{
  const auto cam = test_camera();
  calib::NidParams params;  // min_points = 1000 > the 4 points below
  calib::CalibSample sample;
  sample.t_world_trajframe = calib::identity_mat4();
  sample.image.width = cam.width;
  sample.image.height = cam.height;
  sample.image.gray.assign(static_cast<std::size_t>(cam.width) * cam.height, 0);
  for (int i = 0; i < 4; ++i) {
    sample.points_world.push_back({0.0F, 0.0F, 8.0F});
    sample.intensity_bins.push_back(0);
  }
  EXPECT_FALSE(calib::nid_cost(sample, cam, calib::identity_mat4(), params).has_value());
}

TEST(NidCostTest, EqualizeIntensityBinsSpreadsSkewedInput)
{
  // 90% of mass at low values, 10% high: plain linear binning would put
  // everything into two bins; equalization must occupy most of them.
  std::vector<float> intensities;
  for (int i = 0; i < 900; ++i) {
    intensities.push_back(static_cast<float>(i % 10));
  }
  for (int i = 0; i < 100; ++i) {
    intensities.push_back(200.0F + static_cast<float>(i));
  }
  const auto bins = calib::equalize_intensity_bins(intensities, 16);
  std::array<int, 16> used{};
  for (const auto b : bins) {
    ASSERT_LT(b, 16);
    used[b] = 1;
  }
  int occupied = 0;
  for (const int u : used) {
    occupied += u;
  }
  EXPECT_GE(occupied, 8);
}

TEST(NidCostTest, GrayFromBgr24UsesLuma)
{
  // One blue, one white pixel: white must be brighter than blue.
  const std::array<std::byte, 6> bgr{std::byte{255}, std::byte{0},   std::byte{0},
                                     std::byte{255}, std::byte{255}, std::byte{255}};
  const auto gray = calib::gray_from_bgr24(bgr, 2, 1);
  ASSERT_EQ(gray.gray.size(), 2U);
  EXPECT_LT(gray.gray[0], gray.gray[1]);
}

TEST(NidCostTest, NonFiniteAndUlpEdgePointsAreSkipped)
{
  // Two points the negative-form bounds checks used to let through: a
  // non-finite one (every `<`/`>` against a NaN is false) and one whose column
  // is just inside the frame as a double but rounds UP to exactly `width` once
  // narrowed to float, which then indexes one past the last column of both the
  // image row and depth_cull's cell grid. Neither may reach the histogram, so
  // adding them must not move the cost at all.
  const auto cam = test_camera();
  calib::NidParams params;
  params.min_points = 100;
  const auto clean = make_correlated_sample(cam, params.bins);
  const auto baseline = calib::nid_cost(clean, cam, calib::identity_mat4(), params);
  ASSERT_TRUE(baseline.has_value());

  // Pin the fixture's ULP property so the case cannot silently stop being
  // covered if the test camera or the wall depth ever changes.
  constexpr float kUlpEdgeX = 5.12F;
  const double u_edge = cam.k[0] * (static_cast<double>(kUlpEdgeX) / 8.0) + cam.k[2];
  ASSERT_LT(u_edge, static_cast<double>(cam.width));
  ASSERT_FLOAT_EQ(static_cast<float>(u_edge), static_cast<float>(cam.width));

  auto poisoned = clean;
  const float nan_f = std::numeric_limits<float>::quiet_NaN();
  poisoned.points_world.push_back({nan_f, nan_f, nan_f});
  poisoned.intensity_bins.push_back(0);
  poisoned.points_world.push_back({kUlpEdgeX, 0.0F, 8.0F});
  poisoned.intensity_bins.push_back(0);

  const auto poisoned_cost = calib::nid_cost(poisoned, cam, calib::identity_mat4(), params);
  ASSERT_TRUE(poisoned_cost.has_value());
  EXPECT_DOUBLE_EQ(*poisoned_cost, *baseline);
}

TEST(NidCostTest, EqualizeIntensityBinsOnThePoolMatchesTheSerialOne)
{
  // 300,000 intensities drawn from a skewed, tie-heavy distribution, binned
  // on the calling thread and on a 4-way pool: identical bins. Exact
  // agreement is asserted because a value's rank is the count of strictly
  // smaller values — a function of the value set, not of how equal values
  // were ordered — and the bin is a function of the rank.
  std::vector<float> intensities;
  std::uint64_t state = 17;
  for (int i = 0; i < 300000; ++i) {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    const auto r = static_cast<double>(state >> 11) / 9007199254740992.0;
    intensities.push_back(static_cast<float>(static_cast<int>(r * r * 255.0)));
  }
  const auto serial = calib::equalize_intensity_bins(intensities, 16, nullptr);
  bagwiz::core::WorkerPool pool{4};
  const auto pooled = calib::equalize_intensity_bins(intensities, 16, &pool);
  ASSERT_EQ(pooled.size(), serial.size());
  EXPECT_EQ(pooled, serial);
}

TEST(NidCostTest, NidOfProjectedMatchesNidCostForAnyChunking)
{
  // The same sample projected as 1, 4 and 7 ranges and scored from those
  // chunks gives exactly nid_cost's value. Exact agreement is asserted because
  // projection is per point, the depth cull's nearest depth is a min-reduction
  // over the same set, the histograms count the same integers, and the
  // entropy sums run over the bins in one fixed order.
  const auto cam = test_camera();
  calib::NidParams params;
  params.min_points = 100;
  const auto sample = make_correlated_sample(cam, params.bins);
  const calib::Mat4 pose =
    calib::rigid_inverse(calib::make_transform({0.01, -0.02, 0.0}, {0.0, 0.0, 0.5 * M_PI / 180.0}));
  const auto reference = calib::nid_cost(sample, cam, pose, params);
  ASSERT_TRUE(reference.has_value());
  for (const std::size_t ranges : {std::size_t{1}, std::size_t{4}, std::size_t{7}}) {
    const std::size_t n = sample.points_world.size();
    const std::size_t per = (n + ranges - 1) / ranges;
    std::vector<std::vector<calib::DepthCullPoint>> points(ranges);
    std::vector<std::vector<std::uint8_t>> bins(ranges);
    std::vector<calib::ProjectedChunk> chunks;
    for (std::size_t r = 0; r < ranges; ++r) {
      const std::size_t begin = std::min(n, r * per);
      const std::size_t end = std::min(n, begin + per);
      calib::project_sample_points(sample, cam, pose, params, begin, end, points[r], bins[r]);
    }
    // Chunks handed over in reverse, to pin that their order is irrelevant.
    for (std::size_t r = ranges; r-- > 0;) {
      chunks.push_back(calib::ProjectedChunk{points[r], bins[r]});
    }
    calib::NidScratch scratch;
    const auto chunked = calib::nid_of_projected(sample, params, chunks, scratch);
    ASSERT_TRUE(chunked.has_value()) << ranges;
    EXPECT_EQ(*chunked, *reference) << ranges << " ranges";
  }
}

TEST(NidCostTest, ProjectSampleRangeCoversExactlyItsRange)
{
  // Two ranges concatenate to the whole projection, point for point.
  const auto cam = test_camera();
  const calib::NidParams params;
  const auto sample = make_correlated_sample(cam, params.bins);
  const auto pose = calib::identity_mat4();
  std::vector<calib::DepthCullPoint> whole;
  std::vector<std::uint8_t> whole_bins;
  calib::project_sample_points(
    sample, cam, pose, params, 0, sample.points_world.size(), whole, whole_bins);
  ASSERT_GT(whole.size(), 100U);
  std::vector<calib::DepthCullPoint> split;
  std::vector<std::uint8_t> split_bins;
  const std::size_t cut = sample.points_world.size() / 3;
  calib::project_sample_points(sample, cam, pose, params, 0, cut, split, split_bins);
  calib::project_sample_points(
    sample, cam, pose, params, cut, sample.points_world.size(), split, split_bins);
  ASSERT_EQ(split.size(), whole.size());
  for (std::size_t i = 0; i < whole.size(); ++i) {
    EXPECT_EQ(split[i].u, whole[i].u);
    EXPECT_EQ(split[i].v, whole[i].v);
    EXPECT_EQ(split[i].depth, whole[i].depth);
    EXPECT_EQ(split_bins[i], whole_bins[i]);
  }
}
