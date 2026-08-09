// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__SLAM__VISUAL_REFINEMENT_HPP_
#define CORE__SLAM__VISUAL_REFINEMENT_HPP_

#include "bagwiz/core/slam/visual_observation.hpp"
#include "bagwiz/core/slam/warmup_fill.hpp"
#include "visual_factors.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

// Src-local, like visual_factors.hpp and visual_odometry_window.hpp: the
// final full-trajectory batch bundle adjustment of the camera-only mode
// (CloudMapperConfig::visual_final_ba), run once in finish() after the global
// optimization. Not installed; only cloud_mapper.cpp links this TU.
//
// WHY THIS EXISTS — the camera-only trajectory is weakest at its two ends,
// and the stock pipeline cannot reach the error there:
//  - Start: the odometry anchors its first keyframe at the IMU-only gravity
//    alignment with a TIGHT pose prior (visual_odometry_window.cpp), so an
//    imperfect initialization is frozen into the exported trajectory.
//  - End: the keyframes flushed at end-of-sequence carry only young feature
//    tracks (a factor needs >= 2 keyframes), so they are near-IMU dead
//    reckoning, and unlike mid-run keyframes they never get re-solved with
//    "future" data.
//  - Globally: the camera-only global layer is a thin relative-pose layer
//    (no scan-matching factors anywhere) whose visual factors only cover
//    tracks crossing a submap boundary — rare at both ends — and each
//    keyframe's pose is frozen relative to its submap origin, so the global
//    optimization moves only whole submaps and cannot correct intra-submap
//    error at all.
// Re-estimating EVERY keyframe jointly against ALL observations AND the IMU
// removes the one-sidedness: the first and last keyframes are constrained by
// every track they participate in, exactly like mid-run ones.
//
// HOW — the graph is the odometry window solver's (visual_odometry_window.cpp)
// at full-trajectory scale: per-keyframe pose/velocity/bias states, IMU
// factors between consecutive keyframes (preintegrated from the raw stream
// the mapper buffers in camera-only mode), a bias random walk, and one smart
// rig-projection factor per qualifying track with per-observation rig poses
// folded from the owning keyframe by IMU prediction — the same fold the
// window solver performs. One hard prior on the single middle keyframe fixes
// the gauge (the endpoint evaluation is SE(3)-relative, so the gauge spot
// does not matter; the middle is the best-conditioned point). A priori it
// may seem that smart factors plus an IMU-free relative-pose chain (the
// LiDAR-side pattern) would do — it does NOT: a smooth trajectory warp is
// absorbable by re-triangulating the landmarks (monocular BA's low-frequency
// gauge weakness), so vision-only variants wander along that valley, while
// metric constraints that merely re-assert the seed (seed-delta between
// factors) either under-correct or, re-fitted round over round, diverge. The
// IMU preintegration is the metric, landmark-independent, drift-free
// constraint that makes the problem well-posed — the same reason the online
// odometry works.
//
// The fold deltas come from IMU prediction at the CURRENT estimate (rebuilt
// every round), not from trajectory interpolation: an interpolation-based
// fold re-baked from the just-solved trajectory was measured to amplify the
// solver's low-curvature wander into divergence. The fold therefore runs a
// few solve rounds until the deltas converge with the poses.
//
// The velocity/bias states are seeded from the odometry's own marginalized
// keyframe estimates (RefinementKeyframe::has_state_seed), and the gauge
// priors pin those seeded values: the odometry's export is consistent with
// the bias-CORRECTED IMU, while the preintegration here runs on the raw
// stream, so a zero-seeded bias would present the solve with a systematic
// per-hop acceleration error (0.1-0.4 m/s^2 on a real unit) that only a
// trajectory warp can absorb within the random-walk budget.
//
// Every round solves the FULL graph (visual + IMU factors together) — an
// IMU-only warm-up round, which would snap the seed onto the IMU-consistent
// manifold before the visuals enter, is rejected on measurement: without the
// visual factors the graph has a free direction (constant IMU bias <->
// linear-in-time trajectory ramp, broken only by the loose anchor bias
// prior), and the solve slides along it — tens of degrees of false yaw over a
// minute-scale run. The biases are observable only through vision+IMU
// jointly, exactly as in the online odometry.
namespace bagwiz::core::slam::vio
{

// Tuning for refine_keyframe_poses(). Poses are in the "lidar"-role frame
// (the first camera's optical frame in camera-only mode), matching
// visual::SubmapView's convention; t_lidar_cams follows
// CloudMapperConfig::visual_cameras (p_lidar = T * p_cam). The SOLVE runs in
// the IMU frame, because gtsam's ImuFactor requires the state frame to be the
// IMU's: seeds are converted with t_lidar_imu (CloudMapperConfig::t_lidar_imu,
// = T_cam0_imu in camera-only mode) and the rig extrinsics become
// t_imu_cams = t_lidar_imu^-1 * t_lidar_cams — the same conversion
// make_estimator() applies for the online odometry — and the refined poses
// are converted back before export. The IMU-noise and prior sigmas mirror the
// window solver's own defaults (its IMUIntegration GlobalConfig fallback and
// its edge noises), so the offline solve shares the online one's statistical
// model.
struct RefinementConfig
{
  std::vector<Eigen::Isometry3d> t_lidar_cams;                    // per camera_id
  Eigen::Isometry3d t_lidar_imu = Eigen::Isometry3d::Identity();  // p_lidar = T * p_imu
  double obs_sigma = 1.0e-3;                                      // normalized units
  int min_track_obs = 3;                                          // shorter tracks emit no factor
  int max_obs_per_track = 16;                                     // <= 0 keeps every observation
  double bias_random_walk_sigma = 1.0e-3;                         // between consecutive biases
  double anchor_velocity_sigma = 5.0;  // m/s, loose: the seed FD velocity
  double anchor_bias_sigma = 0.1;      // loose, at zero bias (window solver's edge noise)
  int max_iterations = 50;             // Levenberg-Marquardt cap per round
  int max_rounds = 8;                  // solve rounds; ~2-4 needed
  // Early-stop: when no keyframe moves by more than this between two rounds,
  // the fold deltas have converged with the poses (further rounds only replay
  // the same solve). Deterministic: same input -> same round count.
  double round_convergence_trans = 1.0e-4;  // m
};

// One keyframe of the camera-only trajectory to refine: its stamp and its
// current (seed) world pose. Keyframes must be ascending by stamp.
struct RefinementKeyframe
{
  std::int64_t stamp_ns = 0;
  Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
  // Optional seeds for the velocity/bias states — the odometry's own
  // marginalized-frame estimates (v_world_imu / imu_bias in the IMU frame's
  // world). The bias seed matters on real IMUs: the odometry's export is
  // consistent with the bias-CORRECTED stream, and a real accelerometer bias
  // can reach 0.1-0.4 m/s^2. Zero-seeding would force the solve to re-derive
  // that bias through the random walk and anchor priors — far slower than the
  // walk budget allows — and the trajectory warps to absorb the remainder
  // (measured on vehicle data: a zero-seeded solve bends a 25 s run by tens
  // of meters where the seeded one stays put). With has_state_seed=false the
  // velocity falls back to finite differences over the seed poses and the
  // bias to zero.
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> bias = Eigen::Matrix<double, 6, 1>::Zero();
  bool has_state_seed = false;
};

struct RefinementResult
{
  std::vector<Eigen::Isometry3d> refined;  // parallel to the input keyframes
  // The sparse landmark set re-triangulated at the REFINED poses with the
  // same quality gates as visual::triangulate_landmarks, in deterministic
  // track-key order. Triangulating at the refined poses (instead of reusing
  // the odometry-era landmarks) is what lifts the endpoint regions of the
  // exported map together with the trajectory.
  std::vector<visual::Landmark> landmarks;
  std::size_t factors = 0;            // smart factors emitted (one per used track)
  std::size_t observations_used = 0;  // measurements the factors carry
  int rounds = 0;                     // solve rounds actually run
};

// Re-estimate every keyframe state by a batch Levenberg-Marquardt solve of
// the window-solver-shaped graph (see the header) built from ALL of
// `observations` and the raw `imu_samples` (ascending, covering the
// keyframes' timespan; with an empty span the IMU factors degrade to the
// loose fallbacks the window solver uses), seeded from the keyframes'
// current poses and their velocity/bias state seeds where carried
// (RefinementKeyframe::has_state_seed). Deterministic: the graph is built in
// sorted track-key order with stamp-sorted measurements and solved by gtsam's
// single-threaded batch LM, so identical inputs produce bit-identical output.
//
// Observation ownership: an observation belongs to the last keyframe with
// stamp <= its own (the GroupingBuffer's anchor-window rule); observations
// before the first keyframe are SKIPPED — the odometry never saw them (they
// precede gravity alignment).
RefinementResult refine_keyframe_poses(
  std::span<const VisualObservation> observations, std::span<const BackpropImu> imu_samples,
  const std::vector<RefinementKeyframe> & keyframes, const RefinementConfig & config);

// The landmark half of refine_keyframe_poses(), usable on its own:
// re-triangulate every qualifying track at `poses` (the caller's — typically
// a refined trajectory, or the seed for a baseline comparison) with the same
// quality gates as visual::triangulate_landmarks (>= min_track_obs
// observations, no two-keyframe requirement), in deterministic track-key
// order. keyframes must be the same stamps `poses` was estimated on, parallel
// to `poses`. NOTE: the fold here falls back to trajectory interpolation
// (there is no state estimate to IMU-predict from); refine_keyframe_poses
// uses IMU-predicted folds internally, so landmark sets can differ slightly
// between the two folds at low parallax.
std::vector<visual::Landmark> triangulate_keyframe_landmarks(
  std::span<const VisualObservation> observations,
  const std::vector<RefinementKeyframe> & keyframes, const std::vector<Eigen::Isometry3d> & poses,
  const RefinementConfig & config);

}  // namespace bagwiz::core::slam::vio

#endif  // CORE__SLAM__VISUAL_REFINEMENT_HPP_
