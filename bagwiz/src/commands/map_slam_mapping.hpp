// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_MAPPING_HPP_
#define COMMANDS__MAP_SLAM_MAPPING_HPP_

#include "bagwiz/commands/map_slam.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

// Internals of `map slam`'s mapping run, split out of map_slam.cpp so the
// config fill, the progress setup, the output writing, and the run summary
// can be unit-tested without driving a SLAM run. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Validate the mode-level flag combinations the per-option CLI checks cannot
// express: --pcd is optional since camera-only mode (issue #376 Phase 3), but
// exactly one of the LiDAR or camera input modes must be fully specified.
// Returns the first violation found as a human-readable message, or an empty
// string when the combination is valid. Pure over MapSlamArgs (no bag access)
// so it can be unit-tested directly; run_map_slam calls it before any bag
// work. Rules (--pcd absent == camera-only mode, which --cam must drive):
//   - neither --pcd nor --cam: no SLAM input at all;
//   - --upsample without --imu: it resamples the odometry's per-frame IMU-rate
//     chains, which a LiDAR-only run never estimates (checked in both modes);
//   - --color without --pcd: colorization needs the LiDAR map and its
//     dynamic-occluder oracle;
//   - camera-only without --imu: the odometry is visual-INERTIAL (gravity
//     alignment and the async-rig folding both need the IMU);
//   - camera-only with --backend cuda: the visual-inertial odometry is
//     CPU-only;
//   - camera-only with --remove-dynamic / --remove-outliers: both are
//     LiDAR-map post-processors (ray-cast scans / dense-neighborhood filter)
//     and meaningless on a sparse landmark map.
[[nodiscard]] std::string validate_mode_flags(const MapSlamArgs & args);

// Whether a PointCloud2 frame_id looks like a vehicle or world frame rather
// than a single sensor's own frame (exact match against the conventional
// names: base_link, base_footprint, map, odom). Such a frame means the topic
// is a concatenated/fused cloud whose per-point emitter is unknown, so
// `--remove-dynamic` with the dufomap method would ray-cast from wrong
// origins and silently corrupt the map — the caller warns and points at
// --dynamic-method erasor2. Heuristic on purpose (frame naming is
// convention), hence a warning, never a refusal.
[[nodiscard]] bool is_vehicle_like_frame(std::string_view frame_id);

// Median of the positive consecutive deltas of `stamps_ns` (nanoseconds,
// arrival order), or 0 when fewer than two stamps or no positive delta
// exists. The camera-only anchor period derivation: robust to frame drops (a
// doubled delta) and stamp jitter, where span/count averaging is not.
[[nodiscard]] std::int64_t median_frame_period_ns(std::span<const std::int64_t> stamps_ns);

// Fill the CloudMapperConfig from the parsed CLI arguments.
// `gnss_antenna_offset` is the antenna lever-arm (T_cloud_gnss.translation)
// the caller resolved from the bag's static TF so the GNSS prior constrains
// the sensor origin, not the antenna; a zero offset (GNSS off, or the TF
// absent) reproduces the raw-antenna behavior.
// `visual_cameras` is the --cam extrinsic table (cloud frame <- camera optical
// frame) in --cam listing order, so a row index is the
// VisualObservation::camera_id the visual frontends stamp; empty (no --cam)
// leaves the visual constraints off.
// `visual_anchor_period_ns` is the resolved camera-only anchor-window period
// (issue #17: --visual-anchor-period, or the anchor topic's derived median
// frame period) — required explicitly so no caller can silently ride the
// CloudMapperConfig default; ignored by the mapper outside camera-only mode.
// `upsample_period_ns` is the resolved --upsample rate (0 = off), likewise
// required explicitly rather than defaulted.
[[nodiscard]] core::slam::CloudMapperConfig build_mapper_config(
  const MapSlamArgs & args, const std::optional<core::slam::SensorTransform> & t_lidar_imu,
  bool use_gpu, const std::array<double, 3> & gnss_antenna_offset,
  std::span<const core::slam::SensorTransform> visual_cameras, std::int64_t visual_anchor_period_ns,
  std::int64_t upsample_period_ns);

// Inputs for the ScanProgress construction: whether the live bar renders at
// all, and the total message count it runs against (0 = indeterminate).
struct ScanProgressSetup
{
  bool enabled = false;
  std::int64_t total_msgs = 0;
};

// Resolve the read+feed phase's progress setup. The bar renders only on an
// interactive stderr, with NO_COLOR unset and --no-progress not passed. The
// total is the number of messages the read loop will stream; a bag-stats
// failure only forfeits the determinate bar (warned).
[[nodiscard]] ScanProgressSetup resolve_scan_progress(
  io::BagReader & reader, const MapSlamArgs & args, bool stderr_is_tty, const char * logger);

// Result of finalize_with_spinner(): the finished map plus finish()'s
// wall-clock time in seconds.
struct FinalizeResult
{
  core::slam::CloudMap map;
  double seconds = 0.0;
};

// Run the blocking finalization (global optimization + endpoint window fill +
// map export), which exposes no per-step progress, under an indeterminate
// spinner, then log the timing breakdown.
[[nodiscard]] FinalizeResult finalize_with_spinner(
  core::slam::CloudMapper & mapper, bool progress_on, const char * logger);

// Remove isolated points from the finished map in place (radius outlier
// removal): a point survives when at least `min_neighbors` OTHER map points
// lie within `radius` meters. map.points and, when present, the parallel
// map.intensities are compacted together with their order preserved. Returns
// the number of removed points; radius <= 0 or min_neighbors <= 0 removes
// nothing (the CLI enforces positive values, this is only the last line of
// defense).
[[nodiscard]] std::size_t remove_isolated_map_points(
  core::slam::CloudMap & map, double radius, int min_neighbors, int num_threads);

// Write traj.tum (`trajectory`) and map.pcd (`points` + `intensities` +
// `colors`) to their paths. The map stream is opened BEFORE the trajectory
// file is committed so an unwritable map path fails before either file is
// touched (rather than leaving an orphaned trajectory behind), and it is
// flushed and closed before this returns — an open ofstream keeps the final
// partial block in its user-space buffer, so a later --viewer serve must not
// see the file still open. Returns false on any failure (logged).
[[nodiscard]] bool write_map_outputs(
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  std::span<const core::TrajectoryPose> trajectory, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities, std::span<const std::array<std::uint8_t, 3>> colors,
  const char * logger);

// Log the end-of-run summary: the written pose/point counts, then the
// start/end fill outcomes and the GNSS constraint application (only for the
// features the run actually requested).
void log_mapping_summary(
  const core::slam::CloudMap & map, const MapSlamArgs & args, std::int64_t scans,
  std::int64_t skipped, std::int64_t imu_count, std::int64_t gnss_count,
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  const char * logger);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_MAPPING_HPP_
