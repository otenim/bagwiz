// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/extrinsic_refine.hpp"

#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/observability.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "correlated_scene.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
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

TEST(ExtrinsicRefineTest, HeldDirectionsCoverAxisUsesPhysicalShares)
{
  EXPECT_FALSE(calib::held_directions_cover_axis({}, 1));

  calib::HeldDirection y_held;
  y_held.unit = {0, 1, 0, 0, 0, 0};
  const std::vector<calib::HeldDirection> just_y{y_held};
  EXPECT_TRUE(calib::held_directions_cover_axis(just_y, 1));
  EXPECT_FALSE(calib::held_directions_cover_axis(just_y, 0));

  // 0.71y + 0.71yaw in the probe-step-normalized coordinates: after the step
  // rescaling (0.02 m vs 0.2 deg) the physical mixture is ~0.99y + 0.17yaw,
  // which covers y (>= half the weight) but not yaw.
  calib::HeldDirection mixed;
  const double s = std::sqrt(0.5);
  mixed.unit = {0, s, 0, 0, 0, s};
  const std::vector<calib::HeldDirection> just_mixed{mixed};
  EXPECT_TRUE(calib::held_directions_cover_axis(just_mixed, 1));
  EXPECT_FALSE(calib::held_directions_cover_axis(just_mixed, 5));

  // Half the weight is the cutoff: a direction with a 0.51 physical share on
  // x covers it, one with 0.49 does not.
  const auto share_direction = [](double x_share) {
    calib::HeldDirection h;
    const double z_share = std::sqrt(1.0 - x_share * x_share);
    const double cx = x_share / calib::kProbeStepTrans;
    const double cz = z_share / calib::kProbeStepTrans;
    const double n = std::sqrt(cx * cx + cz * cz);
    h.unit = {cx / n, 0, cz / n, 0, 0, 0};
    return h;
  };
  const auto above = share_direction(0.51);
  const auto below = share_direction(0.49);
  EXPECT_TRUE(calib::held_directions_cover_axis({&above, 1}, 0));
  EXPECT_FALSE(calib::held_directions_cover_axis({&below, 1}, 0));
  EXPECT_TRUE(calib::held_directions_cover_axis({&below, 1}, 2));
}

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
  chain.edge_bag = {0, 0, 0, 0, 0, 1.0 * kDeg};
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
  chain.edge_bag = {0, 0, 0, 0, 0, 1.0 * kDeg};
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.delta[5], 0.0);
  EXPECT_EQ(result.observability[5], calib::AxisObservability::kFixed);
  // A fixed axis is never probed: its curvature estimate stays the default.
  EXPECT_EQ(result.curvature[5].pairs, 0);
}

TEST(ExtrinsicRefineTest, FlatSceneReportsDegenerateVerticalAxis)
{
  // The scene's stripes vary only along x: sliding the camera along y changes
  // nothing the histogram can see, so y must be classified degenerate while
  // x stays observable.
  const auto cam = test_camera();
  auto params = test_params();
  params.auto_fix = false;  // pin the classify-only path; the auto-hold tests are below
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.edge_bag = {};
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.observability[1], calib::AxisObservability::kDegenerate);
  EXPECT_NE(result.observability[0], calib::AxisObservability::kDegenerate);
  EXPECT_TRUE(result.auto_held.empty());
  // The curvature evidence rides along for every probed axis: one sample, one
  // pair per axis — and the fixed-axis test above shows the fixed axis keeps
  // the default (unprobed) estimate.
  EXPECT_EQ(result.curvature[0].pairs, 1);
  EXPECT_EQ(result.curvature[1].pairs, 1);
  // The flat axis's curvature sits under the absolute floor while the
  // observable one's clears it: the numbers behind the two verdicts.
  EXPECT_LT(result.curvature[1].mean, calib::kDegenerateCurvatureFloor);
  EXPECT_GT(result.curvature[0].mean, calib::kDegenerateCurvatureFloor);
}

TEST(ExtrinsicRefineTest, AutoFixHoldsDegenerateAxisAtBagValue)
{
  // Same flat scene, default --fix auto behavior: the y axis the histogram
  // cannot see must be held at the bag value (zero delta, exactly), reported
  // as a held direction dominated by e_y, while the observable x axis is left
  // free to move.
  const auto cam = test_camera();
  auto params = test_params();
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.edge_bag = {};
  chain.t_child_camoptical = calib::identity_mat4();
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  // Not exactly zero: the held eigen-direction is only y-DOMINANT, so a few
  // percent of the y axis lives in the kept subspace and the optimizer leaks
  // that much into delta[1]. A y axis left genuinely free would wander to the
  // centimeter scale or beyond inside the trust region.
  EXPECT_NEAR(result.delta[1], 0.0, 1e-3);
  bool y_held = false;
  for (const auto & held : result.auto_held) {
    if (std::abs(held.unit[1]) > 0.9) {
      y_held = true;
    }
    // Every held direction is genuinely unobservable: its content in the
    // probe-step-normalized coordinates is zeroed exactly, so what remains of
    // the delta along it is exactly the bag's value.
    double component = 0.0;
    for (std::size_t axis = 0; axis < 6; ++axis) {
      const double step = axis < 3 ? calib::kProbeStepTrans : calib::kProbeStepRot;
      component += (result.delta[axis] / step) * held.unit[axis];
    }
    EXPECT_NEAR(component, 0.0, 1e-9);
  }
  EXPECT_TRUE(y_held) << "expected a held direction dominated by the y axis";
  // The x axis stays free: its observability is reported, not held.
  EXPECT_NE(result.observability[0], calib::AxisObservability::kDegenerate);
  // Under --fix auto a degenerate label is only emitted for content the hold
  // actually took: y is covered by the y-dominated held direction, so it keeps
  // the degenerate label rather than being downgraded to weak.
  EXPECT_EQ(result.observability[1], calib::AxisObservability::kDegenerate);
  EXPECT_TRUE(calib::held_directions_cover_axis(result.auto_held, 1));
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
  chain.edge_bag = {};
  chain.t_child_camoptical = calib::identity_mat4();
  const std::vector<calib::CalibSample> samples{sample};
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}

TEST(ExtrinsicRefineTest, NonCommutingEdgeRecoversAdditiveYaw)
{
  // The edited edge carries an optical-convention rotation (rpy = -90, 0, -90
  // deg), where an additive yaw and a right-multiplied one differ by an axis
  // swap. Only yaw is left free because yaw is the OUTERMOST factor of
  // R = Rz(yaw)Ry(pitch)Rx(roll), so it maximizes that difference; roll would
  // not discriminate at all, being the innermost factor where the two
  // forms coincide.
  const auto cam = test_camera();
  auto params = test_params();
  params.fixed = {true, true, true, true, true, false};
  params.max_rot = 5.0 * kDeg;
  const std::array<double, 3> true_rpy{-90.0 * kDeg, 0.0, -90.0 * kDeg};
  constexpr double kInjectedYaw = 2.0 * kDeg;

  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.edge_bag = {0, 0, 0, true_rpy[0], true_rpy[1], true_rpy[2] + kInjectedYaw};
  // The optical leg undoes the true edge, so the camera pose the correlated
  // scene was rendered at is exactly the identity when the delta is right.
  chain.t_child_camoptical = calib::rigid_inverse(calib::make_transform({0, 0, 0}, true_rpy));

  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  const auto result = calib::refine_extrinsic(samples, cam, chain, params);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.delta[5], -kInjectedYaw, 0.3 * kDeg);
  // The refined edge is the bag's scalars plus the delta, per axis: the
  // recovered yaw lands back on the true one and nothing leaks into the other
  // five axes (a right-multiplied delta would have needed a pitch correction
  // here instead, which is the axis swap this pins down).
  const auto after = calib::apply_edge_delta(chain.edge_bag, result.delta);
  EXPECT_NEAR(after[3], true_rpy[0], 1e-12);
  EXPECT_NEAR(after[4], true_rpy[1], 1e-12);
  EXPECT_NEAR(after[5], true_rpy[2], 0.3 * kDeg);
}

TEST(ExtrinsicRefineTest, EdgeTransformIsAdditiveOnANonCommutingRotation)
{
  // The exact statement of the parametrization the optimizer searches, on the
  // rotation where an additive and a right-multiplied delta disagree: the
  // composed edge is make_transform of the summed scalars, nothing else.
  const std::array<double, 6> edge_bag{0.1, -0.2, 0.3, -90.0 * kDeg, 0.0, -90.0 * kDeg};
  const std::array<double, 6> delta{0.01, 0.02, -0.03, 0.5 * kDeg, -0.25 * kDeg, 1.0 * kDeg};

  const auto after = calib::apply_edge_delta(edge_bag, delta);
  for (std::size_t i = 0; i < after.size(); ++i) {
    EXPECT_DOUBLE_EQ(after[i], edge_bag[i] + delta[i]) << "axis " << i;
  }
  const auto expected =
    calib::make_transform({after[0], after[1], after[2]}, {after[3], after[4], after[5]});
  const auto actual = calib::edge_transform(edge_bag, delta);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual[i], expected[i]) << "element " << i;
  }
  // And it is genuinely NOT the right-multiplied factor the report used to
  // disagree with, so this test would fail if that form ever came back.
  const auto right_multiplied = calib::mat4_multiply(
    calib::make_transform(
      {edge_bag[0], edge_bag[1], edge_bag[2]}, {edge_bag[3], edge_bag[4], edge_bag[5]}),
    calib::make_transform({delta[0], delta[1], delta[2]}, {delta[3], delta[4], delta[5]}));
  EXPECT_GT(std::abs(actual[0] - right_multiplied[0]), 1e-6);
}

TEST(ExtrinsicRefineTest, EvaluateSampleCostsMatchesSerialNidCost)
{
  // Three samples at different trajectory poses, evaluated on a 4-way pool
  // (each sample's points projected as four ranges) against nid_cost run
  // serially per sample. Exact agreement is asserted because projection is
  // per point, the depth cull's nearest depth is a min-reduction and the
  // histograms count integers — none of it depends on the split.
  const auto cam = test_camera();
  const auto params = test_params();
  std::vector<calib::CalibSample> samples;
  for (const double yaw : {0.0, 0.2 * kDeg, -0.3 * kDeg}) {
    auto sample = make_correlated_sample(cam, params.nid.bins);
    sample.t_world_trajframe = calib::make_transform({0.0, 0.0, 0.0}, {0.0, 0.0, yaw});
    samples.push_back(std::move(sample));
  }
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.edge_bag = {0.01, 0, 0, 0, 0, 0.5 * kDeg};
  chain.t_child_camoptical = calib::identity_mat4();
  const std::array<double, 6> delta{0.0, 0.005, 0.0, 0.0, 0.0, -0.2 * kDeg};

  bagwiz::core::WorkerPool pool{4};
  const auto pooled = calib::evaluate_sample_costs(samples, cam, chain, delta, params.nid, &pool);
  const auto inline_costs =
    calib::evaluate_sample_costs(samples, cam, chain, delta, params.nid, nullptr);
  ASSERT_EQ(pooled.size(), samples.size());
  const calib::Mat4 t_trajframe_cam = calib::mat4_multiply(
    chain.t_trajframe_parent,
    calib::mat4_multiply(calib::edge_transform(chain.edge_bag, delta), chain.t_child_camoptical));
  for (std::size_t s = 0; s < samples.size(); ++s) {
    const calib::Mat4 t_cam_world =
      calib::rigid_inverse(calib::mat4_multiply(samples[s].t_world_trajframe, t_trajframe_cam));
    const auto serial = calib::nid_cost(samples[s], cam, t_cam_world, params.nid);
    ASSERT_TRUE(serial.has_value()) << s;
    ASSERT_TRUE(pooled[s].has_value()) << s;
    EXPECT_EQ(*pooled[s], *serial) << s;
    ASSERT_TRUE(inline_costs[s].has_value()) << s;
    EXPECT_EQ(*inline_costs[s], *serial) << s;
  }
}

TEST(ExtrinsicRefineTest, RefineResultDoesNotDependOnThePool)
{
  // The whole refinement (default --fix auto) on the calling thread and on a
  // 4-way pool: the same delta, the same NIDs, the same curvature evidence.
  // Exact agreement is asserted for the reason EvaluateSampleCostsMatches-
  // SerialNidCost gives — every evaluated cost is identical — and the
  // optimizer is deterministic over identical costs.
  const auto cam = test_camera();
  const auto params = test_params();
  const std::vector<calib::CalibSample> samples{make_correlated_sample(cam, params.nid.bins)};
  calib::EdgeChain chain;
  chain.t_trajframe_parent = calib::identity_mat4();
  chain.edge_bag = {0, 0, 0, 0, 0, 1.0 * kDeg};
  chain.t_child_camoptical = calib::identity_mat4();

  bagwiz::core::WorkerPool pool{4};
  const auto pooled = calib::refine_extrinsic(samples, cam, chain, params, &pool);
  const auto serial = calib::refine_extrinsic(samples, cam, chain, params, nullptr);
  ASSERT_TRUE(pooled.ok) << pooled.error;
  ASSERT_TRUE(serial.ok) << serial.error;
  EXPECT_EQ(pooled.delta, serial.delta);
  EXPECT_EQ(pooled.nid_before, serial.nid_before);
  EXPECT_EQ(pooled.nid_after, serial.nid_after);
  EXPECT_EQ(pooled.samples_used, serial.samples_used);
  EXPECT_EQ(pooled.observability, serial.observability);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    EXPECT_EQ(pooled.curvature[axis].mean, serial.curvature[axis].mean) << axis;
    EXPECT_EQ(pooled.curvature[axis].std_error, serial.curvature[axis].std_error) << axis;
    EXPECT_EQ(pooled.curvature[axis].pairs, serial.curvature[axis].pairs) << axis;
  }
  ASSERT_EQ(pooled.auto_held.size(), serial.auto_held.size());
  for (std::size_t i = 0; i < pooled.auto_held.size(); ++i) {
    EXPECT_EQ(pooled.auto_held[i].unit, serial.auto_held[i].unit) << i;
  }
}
