// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Final full-trajectory batch BA (visual_refinement.hpp): recovery of
// endpoint-degraded seeds, run-to-run determinism, and the fallback behaviors
// (track-less keyframes, pre-init observations, no observations at all). The
// synthetic world mirrors cloud_mapper_test.cpp's camera-only harness: world
// z-up, a forward-looking camera (optical z along world +x) watching a field
// of landmarks ahead (non-planar by design — a single plane is two-fold
// ambiguous for monocular reconstruction), and a linear diagonal trajectory at
// constant velocity, so the ground-truth IMU stream is a constant specific
// force (plus sensor bias) and the trajectory interpolation the fold falls
// back to is exact at every observation stamp.

#include "visual_refinement.hpp"  // NOLINT(build/include_subdir) src-local header

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{

using bagwiz::core::slam::BackpropImu;
using bagwiz::core::slam::VisualObservation;
using bagwiz::core::slam::vio::refine_keyframe_poses;
using bagwiz::core::slam::vio::RefinementConfig;
using bagwiz::core::slam::vio::RefinementKeyframe;
using bagwiz::core::slam::vio::RefinementResult;

// Referencing one gtsam_points symbol here keeps libgtsam_points.so.1 the
// binary's first-recorded shared library (this test's own objects otherwise
// resolve libgtsam.so.4 symbols first). That NEEDED order — the one
// scan_match_fill_test has naturally — is load-bearing on this toolchain:
// with libgtsam.so.4 first, the dynamic loader eagerly resolves
// libcephes-gtsam.so.1's GLOB_DAT reference to libm's IFUNC `sin` before
// libm.so.6 is relocated and the binary segfaults at startup.
const gtsam_points::PointCloudCPU::Ptr kLoaderOrderProbe(new gtsam_points::PointCloudCPU);

constexpr std::int64_t kBaseStampNs = 1'000'000'000'000'000'000LL;
constexpr std::int64_t kPeriodNs = 100'000'000;  // 10 Hz keyframes
constexpr int kNumKeyframes = 24;

// The forward-looking optical mount (camera z along world +x) — the same
// matrix as cloud_mapper_test.cpp's forward_camera_pose().
Eigen::Isometry3d forward_camera_rotation()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  return pose;
}

// Ground-truth T_world_cam0 at `elapsed_ns` past the trajectory start:
// translation (s, s, 0.5) with s = 1.0 * t (constant velocity), rotation the
// constant forward mount.
Eigen::Isometry3d gt_pose(std::int64_t elapsed_ns)
{
  const double s = 1.0e-9 * static_cast<double>(elapsed_ns);
  Eigen::Isometry3d pose = forward_camera_rotation();
  pose.translation() = Eigen::Vector3d(s, s, 0.5);
  return pose;
}

// 20 landmarks ahead of the rig, deliberately NON-planar: monocular
// reconstruction from views of a single plane is two-fold ambiguous (the
// classic planar degeneracy), which would give the BA a second valid solution
// to slide into. Varying the depth per landmark removes it. All stay visible
// from every keyframe (camera looks along +x; z_cam >= 12 - 2.3 > 0).
std::vector<Eigen::Vector3d> wall_landmarks()
{
  std::vector<Eigen::Vector3d> landmarks;
  std::size_t i = 0;
  for (const double y : {-5.0, -2.5, 0.0, 2.5, 5.0}) {
    for (const double z : {1.0, 2.0, 3.0, 4.0}) {
      landmarks.emplace_back(12.0 + static_cast<double>(i % 5), y, z);
      ++i;
    }
  }
  return landmarks;
}

// The IMU stream the batch BA preintegrates, at 200 Hz: the ground-truth
// specific force R^T * -g_world (the motion is constant-velocity and
// constant-rotation, so the body rate is zero), plus a constant sensor bias
// the solver's bias states must absorb — an exactly-zero bias would leave the
// bias random walk and the ImuFactor's bias correction unexercised.
constexpr std::int64_t kImuPeriodNs = 5'000'000;

std::vector<BackpropImu> make_imu_stream(
  std::int64_t from_ns, std::int64_t to_ns, const Eigen::Vector3d & acc_bias,
  const Eigen::Vector3d & gyro_bias)
{
  const Eigen::Vector3d specific_force =
    forward_camera_rotation().linear().transpose() * Eigen::Vector3d(0.0, 0.0, 9.81) + acc_bias;
  std::vector<BackpropImu> stream;
  for (std::int64_t stamp = from_ns; stamp <= to_ns; stamp += kImuPeriodNs) {
    BackpropImu sample;
    sample.stamp = 1.0e-9 * static_cast<double>(stamp);
    sample.linear_acceleration = specific_force;
    sample.angular_velocity = gyro_bias;
    stream.push_back(sample);
  }
  return stream;
}

// The default stream's small constant biases.
const Eigen::Vector3d kSmallAccBias(0.02, -0.01, 0.03);
const Eigen::Vector3d kSmallGyroBias(0.002, -0.001, 0.0015);

// The stream covering one keyframe list: from just before the first keyframe
// (preintegration starts mid-stream) to comfortably past the last (the last
// keyframe's ownership window folds against IMU predictions).
std::vector<BackpropImu> imu_for(const std::vector<RefinementKeyframe> & keyframes)
{
  return make_imu_stream(
    keyframes.front().stamp_ns - 10'000'000, keyframes.back().stamp_ns + 100'000'000, kSmallAccBias,
    kSmallGyroBias);
}

RefinementConfig make_config()
{
  RefinementConfig config;
  // Single camera; in camera-only mode the "lidar" frame IS cam0's optical
  // frame, so the rig extrinsic is the identity.
  config.t_lidar_cams.push_back(Eigen::Isometry3d::Identity());
  config.obs_sigma = 1.0e-3;
  config.max_obs_per_track = 0;  // keep every observation
  return config;
}

// The keyframe list: kNumKeyframes at 10 Hz with ground-truth poses.
std::vector<RefinementKeyframe> make_keyframes()
{
  std::vector<RefinementKeyframe> keyframes;
  for (int i = 0; i < kNumKeyframes; ++i) {
    const std::int64_t stamp = kBaseStampNs + static_cast<std::int64_t>(i) * kPeriodNs;
    keyframes.push_back(RefinementKeyframe{stamp, gt_pose(stamp - kBaseStampNs)});
  }
  return keyframes;
}

// All observations of the landmarks: two per keyframe per LIVE track (at the
// keyframe's own stamp and half a period later), projected through the
// ground-truth pose at each stamp. Tracks are SHORT-LIVED and staggered
// (track tr lives across keyframes [s, s+life)) — deliberately NOT full-life:
// a track that spans the whole trajectory can re-triangulate its landmark to
// absorb any smooth global pose warp (monocular BA's low-frequency gauge
// weakness), which would leave the seed's ramp error nearly unobservable;
// limited, overlapping spans are what pin the local geometry. No track is
// alive at the LAST keyframe (the stagger keeps every start <=
// kNumKeyframes - kTrackLife - 1), so it owns no observations — the
// end-of-bag situation this pass targets, where the terminal keyframe is
// constrained only by the IMU chain. rgb encodes the track id.
constexpr int kTrackLife = 8;  // keyframes

std::size_t track_start(std::size_t track)
{
  return (track * 7) % (kNumKeyframes - kTrackLife);
}

std::vector<VisualObservation> make_observations(const std::vector<RefinementKeyframe> & keyframes)
{
  const std::vector<Eigen::Vector3d> landmarks = wall_landmarks();
  std::vector<VisualObservation> observations;
  for (std::size_t kf = 0; kf < keyframes.size(); ++kf) {
    const std::int64_t stamp = keyframes[kf].stamp_ns;
    for (int j = 0; j < 2; ++j) {
      const std::int64_t obs_stamp = stamp + static_cast<std::int64_t>(j) * (kPeriodNs / 2);
      const Eigen::Isometry3d t_cam_world = gt_pose(obs_stamp - kBaseStampNs).inverse();
      for (std::size_t track = 0; track < landmarks.size(); ++track) {
        if (kf < track_start(track) || kf >= track_start(track) + kTrackLife) {
          continue;  // track not alive in this keyframe window
        }
        const Eigen::Vector3d p_cam = t_cam_world * landmarks[track];
        if (p_cam.z() <= 0.0) {
          continue;
        }
        VisualObservation obs;
        obs.camera_id = 0;
        obs.track_id = track;
        obs.stamp_ns = obs_stamp;
        obs.x = p_cam.x() / p_cam.z();
        obs.y = p_cam.y() / p_cam.z();
        obs.rgb = {static_cast<std::uint8_t>(track), 20, 30};
        observations.push_back(obs);
      }
    }
  }
  return observations;
}

// Inject a smooth ramp error into the seed: zero at the middle keyframe,
// growing linearly to 0.3 m / ~3 deg at both ends — the shape the refinement
// exists to fix (real endpoint error is locally smooth drift in the absolute
// placement, not in the inter-keyframe motion). The IMU stream stays
// ground-truth-consistent, exactly what lets the batch BA pull the placement
// back.
void degrade_endpoints(std::vector<RefinementKeyframe> & keyframes)
{
  const std::size_t mid = keyframes.size() / 2;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    const double ramp = static_cast<double>(i > mid ? i - mid : mid - i) / static_cast<double>(mid);
    Eigen::Isometry3d & pose = keyframes[i].T_world_lidar;
    pose.translation().y() += 0.3 * ramp;
    pose.linear() = Eigen::AngleAxisd(0.05 * ramp, Eigen::Vector3d::UnitZ()) * pose.linear();
  }
}

// The end-to-end claim: with the seed's endpoints degraded by a smooth drift
// ramp (0.3 m / ~3 deg at both ends, zero at the middle), the batch BA —
// visual factors for the local geometry, the IMU chain for the drift-free
// metric constraint, one hard anchor at the middle keyframe — pulls the
// endpoint error back down to the centimetre level, while the zero-error
// middle is not degraded, and the landmark map re-triangulated at the refined
// poses is strictly better than one triangulated at the seed.
TEST(VisualRefinement, RecoversDegradedEndpoints)
{
  std::vector<RefinementKeyframe> keyframes = make_keyframes();
  const std::vector<VisualObservation> observations = make_observations(keyframes);
  const std::vector<BackpropImu> imu = imu_for(keyframes);
  degrade_endpoints(keyframes);

  // Premise check: the degradation is large enough to matter.
  const double seed_error_start =
    (keyframes.front().T_world_lidar.translation() - gt_pose(0).translation()).norm();
  EXPECT_GT(seed_error_start, 0.1);

  const RefinementResult result =
    refine_keyframe_poses(observations, imu, keyframes, make_config());

  ASSERT_EQ(result.refined.size(), keyframes.size());
  EXPECT_EQ(result.factors, wall_landmarks().size());
  // Every emitted track has 16 observations over its 8-keyframe span, well
  // above min_track_obs, so every observation enters a factor.
  EXPECT_EQ(result.observations_used, observations.size());
  EXPECT_GE(result.rounds, 2);

  // Endpoint recovery: at least 2.5x better than the seed at both ends.
  const double refined_error_start =
    (result.refined.front().translation() - gt_pose(0).translation()).norm();
  const double refined_error_end = (result.refined.back().translation() -
                                    gt_pose(keyframes.back().stamp_ns - kBaseStampNs).translation())
                                     .norm();
  EXPECT_LT(refined_error_start, seed_error_start / 2.5)
    << "refined start error: " << refined_error_start;
  EXPECT_LT(refined_error_end, seed_error_start / 2.5)
    << "refined end error: " << refined_error_end;

  // The middle (zero seed error) must not be degraded: mean translation error
  // over keyframes 8..16 stays at the centimetre level (neighbouring warps
  // pull it slightly), and the hard-anchored keyframe does not move at all.
  double middle_sum = 0.0;
  int middle_count = 0;
  for (std::size_t i = 8; i <= 16; ++i) {
    const Eigen::Isometry3d gt = gt_pose(keyframes[i].stamp_ns - kBaseStampNs);
    middle_sum += (result.refined[i].translation() - gt.translation()).norm();
    ++middle_count;
  }
  EXPECT_LT(middle_sum / middle_count, 0.02) << "middle mean error: " << middle_sum / middle_count;
  const std::size_t anchor = keyframes.size() / 2;
  const Eigen::Isometry3d gt_anchor = gt_pose(keyframes[anchor].stamp_ns - kBaseStampNs);
  EXPECT_LT((result.refined[anchor].translation() - gt_anchor.translation()).norm(), 1.0e-5);

  // Landmarks: one per track, rgb from the track's first observation (track
  // keys sort by (camera_id, track_id), so landmark i belongs to track i).
  ASSERT_EQ(result.landmarks.size(), wall_landmarks().size());
  const std::vector<Eigen::Vector3d> landmarks = wall_landmarks();
  for (std::size_t i = 0; i < result.landmarks.size(); ++i) {
    EXPECT_EQ(
      result.landmarks[i].rgb, (std::array<std::uint8_t, 3>{static_cast<std::uint8_t>(i), 20, 30}));
  }
  // The triangulation gate is itself the verdict on pose quality: at the
  // degraded seed poses, most tracks' rays no longer intersect within the
  // reprojection gate and are dropped from the export...
  const std::vector<Eigen::Isometry3d> seed_poses = [&keyframes] {
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve(keyframes.size());
    for (const auto & kf : keyframes) {
      poses.push_back(kf.T_world_lidar);
    }
    return poses;
  }();
  const std::vector<bagwiz::core::slam::visual::Landmark> seed_landmarks =
    bagwiz::core::slam::vio::triangulate_keyframe_landmarks(
      observations, keyframes, seed_poses, make_config());
  EXPECT_LT(seed_landmarks.size(), result.landmarks.size() / 2);
  // ...while the refined poses pass for every track. The absolute accuracy is
  // bounded loosely: each track triangulates from its 8-keyframe span (~1.1 m
  // baseline at 12-16 m depth), so the residual pose error's bearing
  // component is amplified into depth by ~depth^2/baseline — decimetre pose
  // recovery reads as sub-metre landmark error here.
  for (std::size_t i = 0; i < result.landmarks.size(); ++i) {
    const Eigen::Vector3d p(
      result.landmarks[i].point[0], result.landmarks[i].point[1], result.landmarks[i].point[2]);
    EXPECT_LT((p - landmarks[i]).norm(), 1.0) << "landmark " << i;
  }
}

// A real-scale IMU bias (|acc bias| ~ 0.36 m/s^2, gyro ~ 1.9e-3 rad/s — the
// magnitudes measured on vehicle data) is beyond what a zero-seeded solve can
// re-derive through the bias random walk within two dozen keyframes: the
// trajectory warps to absorb the unmodeled specific force instead. Seeding
// the per-keyframe velocity/bias states from the odometry's own estimates
// (RefinementKeyframe::has_state_seed) keeps the solve at the right point:
// endpoint recovery works as in the small-bias case.
TEST(VisualRefinement, StateSeedsAbsorbLargeImuBias)
{
  const Eigen::Vector3d acc_bias(0.3623, -0.0428, -0.0147);
  const Eigen::Vector3d gyro_bias(-0.000085, 0.000557, -0.001781);
  std::vector<RefinementKeyframe> keyframes = make_keyframes();
  const std::vector<VisualObservation> observations = make_observations(keyframes);
  const std::vector<BackpropImu> imu = make_imu_stream(
    keyframes.front().stamp_ns - 10'000'000, keyframes.back().stamp_ns + 100'000'000, acc_bias,
    gyro_bias);
  degrade_endpoints(keyframes);

  const double seed_error_start =
    (keyframes.front().T_world_lidar.translation() - gt_pose(0).translation()).norm();
  const Eigen::Vector3d gt_end = gt_pose(keyframes.back().stamp_ns - kBaseStampNs).translation();

  // The zero-seeded failure mode, kept as the baseline of the comparison.
  const RefinementResult zero_seeded =
    refine_keyframe_poses(observations, imu, keyframes, make_config());
  const double zero_seeded_start =
    (zero_seeded.refined.front().translation() - gt_pose(0).translation()).norm();

  // Seeded run: bias at the true value and velocity at the constant-velocity
  // ground truth — what a healthy odometry's marginalized frames carry.
  std::vector<RefinementKeyframe> seeded_keyframes = keyframes;  // same degraded poses
  for (RefinementKeyframe & kf : seeded_keyframes) {
    kf.v_world = Eigen::Vector3d(1.0, 1.0, 0.0);
    kf.bias << acc_bias, gyro_bias;
    kf.has_state_seed = true;
  }
  const RefinementResult seeded =
    refine_keyframe_poses(observations, imu, seeded_keyframes, make_config());
  const double seeded_start =
    (seeded.refined.front().translation() - gt_pose(0).translation()).norm();
  const double seeded_end = (seeded.refined.back().translation() - gt_end).norm();

  EXPECT_LT(seeded_start, seed_error_start / 2.5) << "seeded start error: " << seeded_start;
  EXPECT_LT(seeded_end, seed_error_start / 2.5) << "seeded end error: " << seeded_end;
  // ...and the seeded run must be strictly better than the zero-seeded one at
  // the start keyframe (the bias seed is the whole point of the exercise).
  EXPECT_LT(seeded_start, zero_seeded_start)
    << "seeded " << seeded_start << " m vs zero-seeded " << zero_seeded_start << " m";
}

// Determinism: two runs over identical input must agree BIT-FOR-BIT. The
// agreement is algorithmically guaranteed, not observed: the graph is built
// in sorted track-key order with stamp-sorted measurements from immutable
// input, and gtsam's batch LM is single-threaded, so both runs replay the
// identical floating-point instruction sequence (no work-split-dependent
// accumulation order; see AGENTS.md "Numerical Reproducibility").
TEST(VisualRefinement, DeterministicAcrossRuns)
{
  std::vector<RefinementKeyframe> keyframes = make_keyframes();
  const std::vector<VisualObservation> observations = make_observations(keyframes);
  const std::vector<BackpropImu> imu = imu_for(keyframes);
  degrade_endpoints(keyframes);

  const RefinementResult first = refine_keyframe_poses(observations, imu, keyframes, make_config());
  const RefinementResult second =
    refine_keyframe_poses(observations, imu, keyframes, make_config());

  ASSERT_EQ(first.refined.size(), second.refined.size());
  ASSERT_EQ(first.landmarks.size(), second.landmarks.size());
  for (std::size_t i = 0; i < first.refined.size(); ++i) {
    EXPECT_EQ(first.refined[i].matrix(), second.refined[i].matrix()) << "keyframe " << i;
  }
  for (std::size_t i = 0; i < first.landmarks.size(); ++i) {
    EXPECT_EQ(first.landmarks[i].point, second.landmarks[i].point) << "landmark " << i;
  }
}

// A keyframe no observation folds to is untouched by every visual factor, but
// the IMU chain still constrains it: preintegration asserts the true relative
// motion to both neighbours, so a perturbed track-less keyframe is carried
// back toward the ground truth (the online odometry's own behavior on a
// feature-less span). Contrast with NoObservationsKeepsSeeds below: with NO
// IMU samples either, there is no information to move it.
TEST(VisualRefinement, KeyframeWithoutObservationsFollowsImu)
{
  constexpr int kSmall = 6;
  std::vector<RefinementKeyframe> keyframes;
  for (int i = 0; i < kSmall; ++i) {
    const std::int64_t stamp = kBaseStampNs + static_cast<std::int64_t>(i) * kPeriodNs;
    keyframes.push_back(RefinementKeyframe{stamp, gt_pose(stamp - kBaseStampNs)});
  }
  // Perturb the observation-less keyframe so "carried toward the truth" is
  // distinguishable from "stayed at seed".
  keyframes[2].T_world_lidar.translation().x() += 0.13;
  keyframes[2].T_world_lidar.translation().y() -= 0.07;
  const Eigen::Isometry3d seed_pose = keyframes[2].T_world_lidar;

  // Observations for every keyframe window EXCEPT [kf2, kf3) — keyframe 2's
  // ownership span — so no factor references X(2).
  const std::vector<Eigen::Vector3d> landmarks = wall_landmarks();
  std::vector<VisualObservation> observations;
  for (int kf = 0; kf < kSmall; ++kf) {
    if (kf == 2) {
      continue;
    }
    const std::int64_t stamp = keyframes[static_cast<std::size_t>(kf)].stamp_ns;
    const Eigen::Isometry3d t_cam_world = gt_pose(stamp - kBaseStampNs).inverse();
    for (std::size_t track = 0; track < landmarks.size(); ++track) {
      const Eigen::Vector3d p_cam = t_cam_world * landmarks[track];
      VisualObservation obs;
      obs.camera_id = 0;
      obs.track_id = track;
      obs.stamp_ns = stamp;
      obs.x = p_cam.x() / p_cam.z();
      obs.y = p_cam.y() / p_cam.z();
      observations.push_back(obs);
    }
  }
  const std::vector<BackpropImu> imu = imu_for(keyframes);

  const RefinementResult result =
    refine_keyframe_poses(observations, imu, keyframes, make_config());

  ASSERT_EQ(result.refined.size(), keyframes.size());
  // The track-less keyframe must NOT stay at its perturbed seed: the IMU
  // chain disagrees with it.
  const double moved = (result.refined[2].translation() - seed_pose.translation()).norm();
  EXPECT_GT(moved, 0.02) << "the IMU chain should pull the track-less keyframe off its seed";
  // ...and where it lands is near the ground truth, the only placement
  // consistent with both IMU intervals.
  const Eigen::Isometry3d gt = gt_pose(keyframes[2].stamp_ns - kBaseStampNs);
  const double gt_error = (result.refined[2].translation() - gt.translation()).norm();
  const double seed_error = (seed_pose.translation() - gt.translation()).norm();
  EXPECT_LT(gt_error, seed_error / 2.0)
    << "refined " << gt_error << " m from ground truth vs seed " << seed_error << " m";
}

// Observations stamped BEFORE the first keyframe precede gravity alignment —
// the odometry never saw them, and the refinement must skip them too (their
// fold span would be unbounded). Adding them must change nothing.
TEST(VisualRefinement, PreInitObservationsAreSkipped)
{
  constexpr int kSmall = 6;
  std::vector<RefinementKeyframe> keyframes;
  for (int i = 0; i < kSmall; ++i) {
    const std::int64_t stamp = kBaseStampNs + static_cast<std::int64_t>(i) * kPeriodNs;
    keyframes.push_back(RefinementKeyframe{stamp, gt_pose(stamp - kBaseStampNs)});
  }
  const std::vector<Eigen::Vector3d> landmarks = wall_landmarks();
  std::vector<VisualObservation> observations;
  for (int kf = 0; kf < kSmall; ++kf) {
    const std::int64_t stamp = keyframes[static_cast<std::size_t>(kf)].stamp_ns;
    const Eigen::Isometry3d t_cam_world = gt_pose(stamp - kBaseStampNs).inverse();
    for (std::size_t track = 0; track < landmarks.size(); ++track) {
      const Eigen::Vector3d p_cam = t_cam_world * landmarks[track];
      VisualObservation obs;
      obs.camera_id = 0;
      obs.track_id = track;
      obs.stamp_ns = stamp;
      obs.x = p_cam.x() / p_cam.z();
      obs.y = p_cam.y() / p_cam.z();
      observations.push_back(obs);
    }
  }
  const std::vector<BackpropImu> imu = imu_for(keyframes);

  const RefinementResult clean = refine_keyframe_poses(observations, imu, keyframes, make_config());

  std::vector<VisualObservation> with_preinit = observations;
  for (std::size_t track = 0; track < landmarks.size(); ++track) {
    VisualObservation obs;
    obs.camera_id = 0;
    obs.track_id = track;
    obs.stamp_ns = kBaseStampNs - 50'000'000;  // before the first keyframe
    obs.x = 0.1;
    obs.y = 0.1;
    with_preinit.push_back(obs);
  }
  const RefinementResult skipped =
    refine_keyframe_poses(with_preinit, imu, keyframes, make_config());

  EXPECT_EQ(skipped.observations_used, clean.observations_used);
  ASSERT_EQ(skipped.refined.size(), clean.refined.size());
  for (std::size_t i = 0; i < clean.refined.size(); ++i) {
    EXPECT_EQ(skipped.refined[i].matrix(), clean.refined[i].matrix()) << "keyframe " << i;
  }
}

// With no observations AND no IMU samples the graph is priors only, every
// residual is zero at the seed, and the refinement is the identity. Empty
// keyframes are safe too.
TEST(VisualRefinement, NoObservationsKeepsSeeds)
{
  const std::vector<RefinementKeyframe> keyframes = make_keyframes();
  const RefinementResult result = refine_keyframe_poses({}, {}, keyframes, make_config());
  ASSERT_EQ(result.refined.size(), keyframes.size());
  EXPECT_EQ(result.factors, 0u);
  EXPECT_TRUE(result.landmarks.empty());
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    EXPECT_EQ(result.refined[i].matrix(), keyframes[i].T_world_lidar.matrix()) << "keyframe " << i;
  }

  const RefinementResult empty = refine_keyframe_poses({}, {}, {}, make_config());
  EXPECT_TRUE(empty.refined.empty());
  EXPECT_TRUE(empty.landmarks.empty());
}

}  // namespace
