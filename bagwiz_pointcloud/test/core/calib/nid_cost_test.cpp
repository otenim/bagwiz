// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nid_cost.hpp"

#include "bagwiz/core/calib/se3.hpp"
#include "correlated_scene.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
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
