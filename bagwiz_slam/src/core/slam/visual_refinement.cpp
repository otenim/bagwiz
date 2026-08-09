// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_refinement.hpp"  // NOLINT(build/include_subdir) src-local header

#include <glim/common/imu_integration.hpp>

#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core::slam::vio
{

namespace
{

double to_sec(std::int64_t stamp_ns)
{
  return 1.0e-9 * static_cast<double>(stamp_ns);
}

// One observation folded to its owning keyframe: the per-observation constant
// rig pose (keyframe-local motion delta * camera extrinsic), the measurement,
// and what the landmark export needs (stamp + sampled color). Same shape as
// visual_odometry_window.cpp's FoldedObs, keyed by keyframe vector index
// instead of gtsam key index.
struct FoldedObs
{
  std::size_t kf = 0;
  Eigen::Isometry3d body_P_sensor = Eigen::Isometry3d::Identity();
  gtsam::Point2 measurement;
  std::array<std::uint8_t, 3> rgb{};
  std::int64_t stamp_ns = 0;
};

// SE(3) interpolation at t (seconds) over stamp-sorted (times, poses),
// clamped at both ends. Used for the trajectory-interpolation fallback fold
// and for scanning IMU-rate pose predictions.
Eigen::Isometry3d interpolate_pose(
  const std::vector<double> & times, const std::vector<Eigen::Isometry3d> & poses, double t)
{
  if (t <= times.front()) {
    return poses.front();
  }
  if (t >= times.back()) {
    return poses.back();
  }

  const auto hi_it = std::upper_bound(times.begin(), times.end(), t);
  const auto hi = static_cast<std::size_t>(hi_it - times.begin());
  const std::size_t lo = hi - 1;
  const double t0 = times[lo];
  const double t1 = times[hi];
  const double alpha = (t - t0) / (t1 - t0);

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
    poses[lo].translation() + alpha * (poses[hi].translation() - poses[lo].translation());
  result.linear() = Eigen::Quaterniond(poses[lo].rotation())
                      .slerp(alpha, Eigen::Quaterniond(poses[hi].rotation()))
                      .toRotationMatrix();
  return result;
}

// Per-keyframe estimated state carried across solve rounds. The pose frame is
// the SOLVED body frame: the IMU frame inside refine_keyframe_poses (gtsam's
// ImuFactor requires the state frame to be the IMU's), the caller's
// lidar-role frame in triangulate_keyframe_landmarks (no IMU involved there).
struct FrameState
{
  Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
  Eigen::Vector3d v_world = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 1> bias = Eigen::Matrix<double, 6, 1>::Zero();
};

// Ownership index of `stamp_ns`: the last keyframe stamped at or before it,
// or -1 when it precedes the first keyframe (a pre-init observation the
// odometry never saw; see the header).
std::ptrdiff_t owner_keyframe(const std::vector<std::int64_t> & kf_stamps_ns, std::int64_t stamp_ns)
{
  if (stamp_ns < kf_stamps_ns.front()) {
    return -1;
  }
  const auto next_it = std::upper_bound(kf_stamps_ns.begin(), kf_stamps_ns.end(), stamp_ns);
  return static_cast<std::ptrdiff_t>(next_it - kf_stamps_ns.begin()) - 1;
}

using TrackMap = std::unordered_map<visual::TrackKey, std::vector<FoldedObs>, visual::TrackKeyHash>;

// Fold every observation to its owning keyframe, grouped by track. The
// keyframe-local motion delta comes from IMU prediction across the owning
// window at the CURRENT state estimate (exactly the window solver's fold in
// add_visual_factors: integrate the window span at the keyframe's nav/bias,
// then delta = pred_anchor^-1 * pred_obs), refreshed every round as the
// estimate moves. Observations whose window has too few IMU samples to
// integrate — and every observation when `imu` is empty — fall back to
// trajectory interpolation between the bracketing keyframes (the offline
// counterpart; bounded by the keyframe displacement gate). `t_body_cams` are
// the rig extrinsics in the states' body frame (p_body = T * p_cam).
TrackMap fold_observations(
  std::span<const VisualObservation> observations,
  const std::vector<RefinementKeyframe> & keyframes, const std::vector<FrameState> & states,
  std::span<const BackpropImu> imu, const std::vector<Eigen::Isometry3d> & t_body_cams)
{
  const std::size_t n = keyframes.size();
  std::vector<std::int64_t> stamps_ns;
  std::vector<double> times;
  std::vector<Eigen::Isometry3d> poses;
  stamps_ns.reserve(n);
  times.reserve(n);
  poses.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    stamps_ns.push_back(keyframes[i].stamp_ns);
    times.push_back(to_sec(keyframes[i].stamp_ns));
    poses.push_back(states[i].T_world_body);
  }

  // Observations grouped by owner keyframe first (skipping pre-init ones).
  std::vector<std::vector<const VisualObservation *>> by_owner(n);
  for (const VisualObservation & obs : observations) {
    if (obs.camera_id < 0 || static_cast<std::size_t>(obs.camera_id) >= t_body_cams.size()) {
      continue;  // no extrinsic for this camera: nothing to fold or project through
    }
    const std::ptrdiff_t owner = owner_keyframe(stamps_ns, obs.stamp_ns);
    if (owner < 0) {
      continue;
    }
    by_owner[static_cast<std::size_t>(owner)].push_back(&obs);
  }

  // The fold's IMU instance (separate from the preintegration one): refilled
  // per round and consumed in ascending, contiguous window order — the usage
  // pattern IMUIntegration is built for (the window solver's).
  glim::IMUIntegration imu_integration;
  for (const BackpropImu & sample : imu) {
    imu_integration.insert_imu(sample.stamp, sample.linear_acceleration, sample.angular_velocity);
  }

  TrackMap tracks;
  for (std::size_t i = 0; i < n; ++i) {
    const std::vector<const VisualObservation *> & owned = by_owner[i];
    if (owned.empty()) {
      continue;
    }

    // IMU-predict the rig poses across this keyframe's ownership window
    // [t_i, t_next) — for the last keyframe, its last owned observation's
    // stamp (the displacement gate bounds how far past that anything sits).
    const double window_end = (i + 1 < n) ? times[i + 1] : to_sec(owned.back()->stamp_ns);
    std::vector<double> pred_times;
    std::vector<Eigen::Isometry3d> pred_poses;
    if (window_end > times[i]) {
      imu_integration.integrate_imu(
        times[i], window_end,
        gtsam::NavState(gtsam::Pose3(states[i].T_world_body.matrix()), states[i].v_world),
        gtsam::imuBias::ConstantBias(states[i].bias), pred_times, pred_poses);
    }
    // Usable only with >= 2 predicted poses to interpolate between; otherwise
    // (empty IMU span, or a window shorter than one IMU period) fold against
    // the trajectory interpolation instead.
    const bool imu_fold = pred_poses.size() >= 2;

    for (const VisualObservation * obs : owned) {
      Eigen::Isometry3d delta;
      if (imu_fold) {
        const Eigen::Isometry3d pred_obs =
          interpolate_pose(pred_times, pred_poses, to_sec(obs->stamp_ns));
        delta = pred_poses.front().inverse() * pred_obs;
      } else {
        delta = poses[i].inverse() * interpolate_pose(times, poses, to_sec(obs->stamp_ns));
      }
      const Eigen::Isometry3d body_p_sensor =
        delta * t_body_cams[static_cast<std::size_t>(obs->camera_id)];
      tracks[visual::track_key(*obs)].push_back(
        FoldedObs{i, body_p_sensor, gtsam::Point2(obs->x, obs->y), obs->rgb, obs->stamp_ns});
    }
  }

  // Per-camera arrival is stamp-ordered but multi-camera batches interleave;
  // the measurement order inside a factor is part of the built graph, so make
  // it deterministic.
  for (auto & entry : tracks) {
    std::sort(
      entry.second.begin(), entry.second.end(),
      [](const FoldedObs & a, const FoldedObs & b) { return a.stamp_ns < b.stamp_ns; });
  }
  return tracks;
}

// Thin a long track to max_obs evenly spaced observations (first and last
// kept), in place — the same stride recipe as visual_factors.cpp's
// subsample(), applied to the folded observations. Because the stride depends
// only on the track's size, every fold selects the same subset.
void subsample_track(std::vector<FoldedObs> & track, int max_obs)
{
  if (max_obs <= 0 || track.size() <= static_cast<std::size_t>(max_obs)) {
    return;
  }
  const auto limit = static_cast<std::size_t>(max_obs);
  if (limit < 2) {
    track = {track.front()};
    return;
  }
  std::vector<FoldedObs> kept;
  kept.reserve(limit);
  const double step = static_cast<double>(track.size() - 1) / static_cast<double>(limit - 1);
  for (std::size_t i = 0; i < limit; ++i) {
    kept.push_back(track[static_cast<std::size_t>(std::llround(static_cast<double>(i) * step))]);
  }
  track = std::move(kept);
}

// Triangulate one track in the world frame at the given keyframe poses. Same
// recipe as visual_factors.cpp's triangulate_world, keyed per keyframe.
gtsam::TriangulationResult triangulate_track(
  const std::vector<FoldedObs> & track, const std::vector<Eigen::Isometry3d> & poses,
  const gtsam::Cal3_S2::shared_ptr & calibration, const gtsam::TriangulationParameters & params)
{
  gtsam::CameraSet<visual::RigCamera> cameras;
  visual::RigCamera::MeasurementVector measurements;
  cameras.reserve(track.size());
  measurements.reserve(track.size());
  for (const FoldedObs & obs : track) {
    const Eigen::Isometry3d t_world_cam = poses[obs.kf] * obs.body_P_sensor;
    cameras.emplace_back(gtsam::Pose3(t_world_cam.matrix()), calibration);
    measurements.push_back(obs.measurement);
  }
  return gtsam::triangulateSafe(cameras, measurements, params);
}

// Preintegrate each consecutive keyframe interval from the raw stream (once:
// gtsam::ImuFactor applies first-order bias corrections internally, so
// re-integrating whenever the bias estimate moves is unnecessary — the
// window solver's own recipe). Parallel to keyframes minus one; nullopt
// where the interval has fewer than 2 samples (the caller substitutes the
// window solver's loose fallbacks).
std::vector<std::optional<gtsam::PreintegratedImuMeasurements>> preintegrate_pairs(
  std::span<const BackpropImu> imu, const std::vector<RefinementKeyframe> & keyframes)
{
  std::vector<std::optional<gtsam::PreintegratedImuMeasurements>> pims(keyframes.size() - 1);
  glim::IMUIntegration imu_integration;
  for (const BackpropImu & sample : imu) {
    imu_integration.insert_imu(sample.stamp, sample.linear_acceleration, sample.angular_velocity);
  }
  const gtsam::imuBias::ConstantBias zero_bias;
  for (std::size_t i = 0; i + 1 < keyframes.size(); ++i) {
    int num_integrated = 0;
    imu_integration.integrate_imu(
      to_sec(keyframes[i].stamp_ns), to_sec(keyframes[i + 1].stamp_ns), zero_bias, &num_integrated);
    if (num_integrated >= 2) {
      pims[i] = gtsam::PreintegratedImuMeasurements(imu_integration.integrated_measurements());
    }
  }
  return pims;
}

}  // namespace

RefinementResult refine_keyframe_poses(
  std::span<const VisualObservation> observations, std::span<const BackpropImu> imu_samples,
  const std::vector<RefinementKeyframe> & keyframes, const RefinementConfig & config)
{
  using gtsam::symbol_shorthand::B;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;

  RefinementResult result;
  if (keyframes.empty()) {
    return result;
  }
  const std::size_t n = keyframes.size();

  // The solve runs in the IMU frame (the ImuFactor's state-frame
  // requirement): seeds and the gauge anchor convert with t_lidar_imu, the
  // rig extrinsics with its inverse — the same conversion make_estimator()
  // applies for the online odometry — and the refined poses convert back at
  // the end.
  const Eigen::Isometry3d t_imu_lidar = config.t_lidar_imu.inverse();
  std::vector<Eigen::Isometry3d> t_imu_cams;
  t_imu_cams.reserve(config.t_lidar_cams.size());
  for (const Eigen::Isometry3d & t : config.t_lidar_cams) {
    t_imu_cams.push_back(t_imu_lidar * t);
  }
  std::vector<Eigen::Isometry3d> seed_imu(n);
  for (std::size_t i = 0; i < n; ++i) {
    seed_imu[i] = keyframes[i].T_world_lidar * config.t_lidar_imu;
  }

  // Seed states: the seed poses, and per-keyframe velocity/bias from the
  // odometry's own marginalized-frame estimates where the caller carried them
  // through (has_state_seed). Without those the velocity falls back to finite
  // differences over the seed poses (locally accurate — drift is in the
  // absolute placement, not in the local motion) and the bias to zero.
  std::vector<FrameState> current(n);
  for (std::size_t i = 0; i < n; ++i) {
    current[i].T_world_body = seed_imu[i];
    if (keyframes[i].has_state_seed) {
      current[i].v_world = keyframes[i].v_world;
      current[i].bias = keyframes[i].bias;
    }
  }
  // Finite-difference velocity fallback for the keyframes without a state
  // seed.
  std::vector<Eigen::Vector3d> fd_velocity(n, Eigen::Vector3d::Zero());
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double dt = to_sec(keyframes[i + 1].stamp_ns - keyframes[i].stamp_ns);
    if (dt > 0.0) {
      const Eigen::Vector3d v = (seed_imu[i + 1].translation() - seed_imu[i].translation()) / dt;
      fd_velocity[i] = v;
      fd_velocity[i + 1] = v;  // refined below for interior keyframes
    }
  }
  for (std::size_t i = 1; i + 1 < n; ++i) {
    fd_velocity[i] = 0.5 * (fd_velocity[i - 1] + fd_velocity[i]);
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!keyframes[i].has_state_seed) {
      current[i].v_world = fd_velocity[i];
    }
  }
  // The anchor velocity/bias priors pin the SEEDED state, constant across
  // rounds — captured here because `current` is overwritten by each solve.
  const std::size_t anchor = n / 2;
  const Eigen::Vector3d anchor_seed_velocity = current[anchor].v_world;
  const Eigen::Matrix<double, 6, 1> anchor_seed_bias = current[anchor].bias;

  const auto pims = preintegrate_pairs(imu_samples, keyframes);

  const auto calibration = visual::normalized_calibration();
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, config.obs_sigma);
  const auto factor_params = visual::make_smart_projection_params();
  const auto min_obs = static_cast<std::size_t>(config.min_track_obs);

  TrackMap last_tracks;
  for (int round = 0; round < config.max_rounds; ++round) {
    const auto tracks =
      fold_observations(observations, keyframes, current, imu_samples, t_imu_cams);

    // The emitted factor order becomes part of the built graph, so walk
    // tracks in key order instead of the hash map's (same rule as the window
    // solver).
    std::vector<visual::TrackKey> keys;
    keys.reserve(tracks.size());
    for (const auto & entry : tracks) {
      keys.push_back(entry.first);
    }
    std::sort(keys.begin(), keys.end());

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    for (std::size_t i = 0; i < n; ++i) {
      values.insert(X(i), gtsam::Pose3(current[i].T_world_body.matrix()));
      values.insert(V(i), current[i].v_world);
      values.insert(B(i), gtsam::imuBias::ConstantBias(current[i].bias));
    }

    std::size_t factors = 0;
    std::size_t observations_used = 0;
    for (const visual::TrackKey & key : keys) {
      std::vector<FoldedObs> track = tracks.at(key);
      subsample_track(track, config.max_obs_per_track);

      std::unordered_set<std::size_t> distinct_kfs;
      for (const FoldedObs & obs : track) {
        distinct_kfs.insert(obs.kf);
      }
      if (distinct_kfs.size() < 2 || track.size() < min_obs) {
        continue;  // constrains nothing, or too short to trust
      }

      // One rig camera per observation, its pose the folded per-observation
      // extrinsic — keyed per keyframe, with observations from the same
      // keyframe repeating the pose key under distinct entry indices (the rig
      // factor's documented non-unique-keys mode), exactly the window
      // solver's shape.
      auto rig = std::make_shared<gtsam::CameraSet<visual::RigCamera>>();
      rig->reserve(track.size());
      auto factor = std::make_shared<visual::RigFactor>(noise, rig, factor_params);
      for (std::size_t i = 0; i < track.size(); ++i) {
        rig->emplace_back(gtsam::Pose3(track[i].body_P_sensor.matrix()), calibration);
        factor->add(track[i].measurement, X(track[i].kf), i);
      }
      graph.add(factor);
      ++factors;
      observations_used += track.size();
    }

    // IMU chain (the window solver's exact shape): ImuFactor where the
    // preintegration has >= 2 samples, the same loose fallbacks where not.
    for (std::size_t i = 0; i + 1 < n; ++i) {
      if (pims[i].has_value()) {
        graph.emplace_shared<gtsam::ImuFactor>(X(i), V(i), X(i + 1), V(i + 1), B(i), *pims[i]);
      } else {
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
          V(i), V(i + 1), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0));
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(i), X(i + 1),
          gtsam::Pose3(current[i].T_world_body.matrix())
            .between(gtsam::Pose3(current[i + 1].T_world_body.matrix())),
          gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
      }
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(i), B(i + 1), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Isotropic::Sigma(6, config.bias_random_walk_sigma));
    }

    // Gauge: one hard pose prior on the single middle keyframe — an endpoint
    // anchor would re-freeze exactly the poses this pass exists to unpin —
    // plus the window solver's loose edge noises there for velocity and bias,
    // both AT THE SEEDED STATE (the odometry's own estimate when carried
    // through, else the FD velocity and zero bias): the bias prior in
    // particular must pin the seeded value, not zero — pinning a real 0.1+
    // m/s^2 bias at zero would reintroduce the warp the seeding exists to
    // prevent.
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(anchor), gtsam::Pose3(seed_imu[anchor].matrix()),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-6));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      V(anchor), anchor_seed_velocity,
      gtsam::noiseModel::Isotropic::Sigma(3, config.anchor_velocity_sigma));
    graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(anchor), gtsam::imuBias::ConstantBias(anchor_seed_bias),
      gtsam::noiseModel::Isotropic::Sigma(6, config.anchor_bias_sigma));

    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(config.max_iterations);
    gtsam::Values solved;
    try {
      solved = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
    } catch (const gtsam::IndeterminantLinearSystemException &) {
      // Defensive (mirrors the window solver): keep the pre-round states
      // rather than propagate a rank-deficiency out of finish().
      solved = values;
    }

    double max_move = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Isometry3d next(solved.at<gtsam::Pose3>(X(i)).matrix());
      max_move =
        std::max(max_move, (next.translation() - current[i].T_world_body.translation()).norm());
      current[i].T_world_body = next;
      current[i].v_world = solved.at<gtsam::Vector3>(V(i));
      current[i].bias = solved.at<gtsam::imuBias::ConstantBias>(B(i)).vector();
    }
    result.rounds = round + 1;
    result.factors = factors;
    result.observations_used = observations_used;
    last_tracks = tracks;
    if (max_move < config.round_convergence_trans) {
      break;
    }
  }

  // Back to the lidar-role frame for the export.
  result.refined.reserve(n);
  for (const FrameState & state : current) {
    result.refined.push_back(state.T_world_body * t_imu_lidar);
  }

  // Re-triangulate the sparse landmark set at the final fold (consistent with
  // the solved states), with the same gates as visual::triangulate_landmarks:
  // >= min_track_obs observations, no two-keyframe requirement — a
  // single-keyframe track with parallax is valid map geometry. The
  // triangulation recipe (rank tolerance sized for normalized coordinates,
  // 150 m distance cap, 3-sigma reprojection gate) is the shared one from
  // visual_factors.cpp. The solved poses stay in the IMU frame here: the
  // fold's rig poses do.
  std::vector<Eigen::Isometry3d> refined_body;
  refined_body.reserve(n);
  for (const FrameState & state : current) {
    refined_body.push_back(state.T_world_body);
  }
  std::vector<visual::TrackKey> keys;
  keys.reserve(last_tracks.size());
  for (const auto & entry : last_tracks) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());
  const gtsam::TriangulationParameters tri_params(
    1.0e-9, /*enableEPI=*/false, 150.0, 3.0 * config.obs_sigma);
  for (const visual::TrackKey & key : keys) {
    std::vector<FoldedObs> track = last_tracks.at(key);
    subsample_track(track, config.max_obs_per_track);
    if (track.size() < min_obs) {
      continue;
    }
    const gtsam::TriangulationResult point =
      triangulate_track(track, refined_body, calibration, tri_params);
    if (!point.valid()) {
      continue;
    }
    result.landmarks.push_back(
      visual::Landmark{
        {static_cast<float>(point->x()), static_cast<float>(point->y()),
         static_cast<float>(point->z())},
        track.front().rgb});
  }
  return result;
}

std::vector<visual::Landmark> triangulate_keyframe_landmarks(
  std::span<const VisualObservation> observations,
  const std::vector<RefinementKeyframe> & keyframes, const std::vector<Eigen::Isometry3d> & poses,
  const RefinementConfig & config)
{
  std::vector<visual::Landmark> landmarks;
  if (keyframes.empty()) {
    return landmarks;
  }
  const auto calibration = visual::normalized_calibration();
  const auto min_obs = static_cast<std::size_t>(config.min_track_obs);
  // Trajectory-interpolation fold (see the header note): no state estimate
  // exists to IMU-predict from on this path. Poses stay in the caller's
  // lidar-role frame, so the fold takes the lidar-frame rig extrinsics.
  std::vector<FrameState> states(keyframes.size());
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    states[i].T_world_body = poses[i];
  }
  const auto tracks = fold_observations(observations, keyframes, states, {}, config.t_lidar_cams);
  std::vector<visual::TrackKey> keys;
  keys.reserve(tracks.size());
  for (const auto & entry : tracks) {
    keys.push_back(entry.first);
  }
  std::sort(keys.begin(), keys.end());
  const gtsam::TriangulationParameters tri_params(
    1.0e-9, /*enableEPI=*/false, 150.0, 3.0 * config.obs_sigma);
  for (const visual::TrackKey & key : keys) {
    std::vector<FoldedObs> track = tracks.at(key);
    subsample_track(track, config.max_obs_per_track);
    if (track.size() < min_obs) {
      continue;
    }
    const gtsam::TriangulationResult point =
      triangulate_track(track, poses, calibration, tri_params);
    if (!point.valid()) {
      continue;
    }
    landmarks.push_back(
      visual::Landmark{
        {static_cast<float>(point->x()), static_cast<float>(point->y()),
         static_cast<float>(point->z())},
        track.front().rgb});
  }
  return landmarks;
}

}  // namespace bagwiz::core::slam::vio
