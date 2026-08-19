// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/extrinsic_refine.hpp"

#include "bagwiz/core/calib/se3.hpp"
#include "correlated_scene.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace calib = bagwiz::core::calib;

namespace
{
constexpr double kDeg = M_PI / 180.0;

calib::RefineParams test_params()
{
  calib::RefineParams params;
  params.nid.min_points = 100;
  return params;
}
}  // namespace

TEST(ExtrinsicRefineTest, RecoversInjectedYawError)
{
  const auto cam = test_camera();
  auto params = test_params();
  // At a single-depth frontal wall, yaw and lateral/forward translation are
  // projectively indistinguishable (the same degeneracy the real-bag spike
  // measured), so this test constrains translations and asserts pure
  // rotation recovery.
  params.fixed[0] = params.fixed[1] = params.fixed[2] = true;
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  // The bag's recorded edge is wrong by +1 deg of yaw; the image was rendered
  // at identity, so refine must find delta yaw ~= -1 deg.
  chain.t_parent_child = calib::make_transform({0, 0, 0}, {0, 0, 1.0 * kDeg});
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.delta[5], -1.0 * kDeg, 0.1 * kDeg);
  EXPECT_LT(result.nid_after, result.nid_before);
  EXPECT_EQ(result.samples_used, 1);
}

TEST(ExtrinsicRefineTest, FixedAxisDoesNotMove)
{
  const auto cam = test_camera();
  auto params = test_params();
  params.fixed[5] = true;  // yaw held at the bag value
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.t_parent_child = calib::make_transform({0, 0, 0}, {0, 0, 1.0 * kDeg});
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.delta[5], 0.0);
  EXPECT_EQ(result.observability[5], calib::AxisObservability::kFixed);
}

TEST(ExtrinsicRefineTest, FlatSceneReportsDegenerateVerticalAxis)
{
  // The scene's stripes vary only along x: sliding the camera along y changes
  // nothing the histogram can see, so y must be classified degenerate while
  // x stays observable.
  const auto cam = test_camera();
  const auto params = test_params();
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.t_parent_child = calib::identity_mat4();
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.observability[1], calib::AxisObservability::kDegenerate);
  EXPECT_NE(result.observability[0], calib::AxisObservability::kDegenerate);
}

TEST(ExtrinsicRefineTest, AllPointsOutOfViewFailsCleanly)
{
  const auto cam = test_camera();
  const auto params = test_params();
  auto sample = make_correlated_sample(cam, params.nid.bins);
  for (auto & p : sample.points_world) {
    p[2] = -8.0F;  // behind the camera
  }
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.t_parent_child = calib::identity_mat4();
  chain.t_child_camoptical = calib::identity_mat4();
  const std::vector<calib::CalibSample> samples{sample};
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}
