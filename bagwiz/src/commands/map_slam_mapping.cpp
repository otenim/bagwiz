// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_mapping.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/kdtree.hpp"
#include "bagwiz/core/pointcloud/outlier_removal.hpp"
#include "bagwiz/core/slam/point_cloud_io.hpp"
#include "bagwiz/core/slam/progress_bar.hpp"
#include "bagwiz/io/page_cache.hpp"
#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <string>
#include <vector>

namespace bagwiz::commands
{
namespace
{

// " + N IMU samples" when IMU mode ran, otherwise empty.
std::string imu_suffix(const MapSlamArgs & args, std::int64_t imu_count)
{
  if (args.imu_topic.empty()) {
    return "";
  }
  return fmt::format(" + {} IMU samples", imu_count);
}

}  // namespace

std::string validate_mode_flags(const MapSlamArgs & args)
{
  const bool camera_only = args.cloud_topic.empty();
  if (camera_only && args.cam_topics.empty()) {
    return "either --pcd (LiDAR SLAM) or --cam (camera-only visual-inertial SLAM) is required";
  }
  // Checked before the mode split: --upsample is valid in both modes, and in
  // camera-only mode --imu is required anyway, so this can only fire with --pcd.
  if (!args.upsample.empty() && args.imu_topic.empty()) {
    return "--upsample needs --imu: the poses it resamples are the odometry's per-frame "
           "IMU-rate chains, which a LiDAR-only run never estimates";
  }
  if (!camera_only) {
    // The --dynamic-* tuning options each affect exactly one removal method;
    // passing one to the other method would be a silent no-op, so refuse it.
    // Checked in LiDAR mode only: in camera-only mode --remove-dynamic itself
    // is rejected below, which is the guidance that matters there.
    if (args.dynamic_method == "erasor2" && args.dynamic_dufomap_tuning_given) {
      return "--dynamic-res / --dynamic-ds / --dynamic-dp tune the dufomap method's "
             "free-space grid and have no effect with --dynamic-method erasor2";
    }
    if (args.dynamic_method != "erasor2" && args.dynamic_sensor_height_given) {
      return "--dynamic-sensor-height anchors the erasor2 method's height band and has "
             "no effect with the dufomap method (pass --dynamic-method erasor2)";
    }
    if (!args.visual_anchor_period.empty()) {
      return "--visual-anchor-period sets the camera-only grouping's anchor window and "
             "has no effect with --pcd (the LiDAR modes group nothing by camera period)";
    }
    return "";  // LiDAR mode: every camera/feature flag keeps its existing meaning
  }
  if (!args.color_topics.empty()) {
    return "--color requires --pcd (colorization needs the LiDAR map and its "
           "dynamic-occluder oracle); the camera-only map is rgb-colored from its own "
           "track observations instead";
  }
  if (args.imu_topic.empty()) {
    return "camera-only mode (--cam without --pcd) requires --imu: the odometry is "
           "visual-INERTIAL — gravity alignment and the asynchronous-camera folding both "
           "integrate the IMU stream";
  }
  if (args.backend == "cuda") {
    return "camera-only mode (--cam without --pcd) does not support --backend cuda: the "
           "visual-inertial odometry is CPU-only (use --backend auto/cpu)";
  }
  if (args.remove_dynamic) {
    return "--remove-dynamic requires --pcd (it ray-casts the LiDAR scans); the "
           "camera-only map is a sparse landmark set with no scan ghosts to remove";
  }
  if (args.remove_outliers) {
    return "--remove-outliers requires --pcd (a dense-map neighborhood filter); it would "
           "decimate the camera-only sparse landmark map";
  }
  return "";
}

// cppcheck-suppress passedByValue  // std::string_view is a cheap value type
bool is_vehicle_like_frame(std::string_view frame_id)
{
  return frame_id == "base_link" || frame_id == "base_footprint" || frame_id == "map" ||
         frame_id == "odom";
}

std::int64_t median_frame_period_ns(std::span<const std::int64_t> stamps_ns)
{
  if (stamps_ns.size() < 2) {
    return 0;
  }
  std::vector<std::int64_t> deltas;
  deltas.reserve(stamps_ns.size() - 1);
  for (std::size_t i = 1; i < stamps_ns.size(); ++i) {
    const std::int64_t delta = stamps_ns[i] - stamps_ns[i - 1];
    if (delta > 0) {
      deltas.push_back(delta);
    }
  }
  if (deltas.empty()) {
    return 0;
  }
  // nth_element instead of a full sort: only the median position matters.
  const std::size_t mid = deltas.size() / 2;
  std::nth_element(deltas.begin(), deltas.begin() + static_cast<std::ptrdiff_t>(mid), deltas.end());
  return deltas[mid];
}

core::slam::CloudMapperConfig build_mapper_config(
  const MapSlamArgs & args, const std::optional<core::slam::SensorTransform> & t_lidar_imu,
  bool use_gpu, const std::array<double, 3> & gnss_antenna_offset,
  std::span<const core::slam::SensorTransform> visual_cameras,
  const std::int64_t visual_anchor_period_ns, const std::int64_t upsample_period_ns)
{
  core::slam::CloudMapperConfig config;
  config.input_resolution = args.input_resolution;
  config.range_min = args.range_min;
  config.range_max = args.range_max;
  config.fill_min_inlier_fraction = args.fill_min_inlier_fraction;
  config.submap_max_keyframes = args.submap_max_keyframes;
  config.remove_dynamic_points = args.remove_dynamic;
  config.dynamic_voxel_size = args.dynamic_resolution;
  config.dynamic_sensor_offset = args.dynamic_sensor_offset;
  config.dynamic_neighborhood = args.dynamic_neighborhood;
  // The CLI restricts --dynamic-method to the two member names, so anything
  // that is not "erasor2" is the dufomap default.
  config.dynamic_method = args.dynamic_method == "erasor2"
                            ? core::slam::DynamicRemovalMethod::kErasor2
                            : core::slam::DynamicRemovalMethod::kDufomap;
  config.dynamic_erasor.sensor_height = args.dynamic_sensor_height;
  config.t_lidar_imu = t_lidar_imu;
  config.num_threads = resolve_threads(args.num_threads);
  config.enable_gnss = !args.gnss_topic.empty();
  config.use_gpu = use_gpu;
  // The fill scan-matches the window scans against the optimized map, so it runs
  // in LiDAR-only mode too; --imu only adds the IMU init/fallback path inside the
  // mapper. Gated solely on the fill toggles, not on the IMU topic.
  config.fill_start = args.fill_start;
  config.fill_end = args.fill_end;
  config.gnss_antenna_offset = gnss_antenna_offset;
  config.visual_cameras.assign(visual_cameras.begin(), visual_cameras.end());
  // No --pcd == camera-only visual-inertial SLAM (issue #376 Phase 3): the
  // mapper swaps its odometry layer for the visual-inertial estimator and
  // exports a sparse landmark map. validate_mode_flags (called first by
  // run_map_slam) guarantees --cam and --imu are set in this mode.
  config.camera_only = args.cloud_topic.empty();
  // Resolved by the caller — --visual-anchor-period or the anchor camera
  // topic's derived median frame period (issue #17); the mapper ignores it
  // outside camera-only mode.
  config.visual_anchor_period_ns = visual_anchor_period_ns;
  // Resolved by the caller from --upsample (period or hz); 0 leaves traj.tum at
  // one pose per scan and CloudMap::trajectory_dense empty.
  config.upsample_period_ns = upsample_period_ns;
  return config;
}

ScanProgressSetup resolve_scan_progress(
  io::BagReader & reader, const MapSlamArgs & args, bool stderr_is_tty, const char * logger)
{
  // Live progress bar (stderr) for the long read+feed phase. Auto-suppressed
  // off a TTY / under NO_COLOR / with --no-progress (progress_enabled), so it
  // never spams a pipe or log. The total is the number of messages the read
  // loop will stream; a stats failure only forfeits the determinate bar.
  ScanProgressSetup setup;
  setup.enabled = core::slam::progress_enabled(
    stderr_is_tty, std::getenv("NO_COLOR") != nullptr, args.no_progress);
  if (setup.enabled) {
    // The read loop streams the cloud topic in LiDAR modes; camera-only mode
    // (--cam without --pcd) streams IMU + cameras only.
    std::vector<std::string> progress_topics;
    if (!args.cloud_topic.empty()) {
      progress_topics.push_back(args.cloud_topic);
    }
    if (!args.imu_topic.empty()) {
      progress_topics.push_back(args.imu_topic);
    }
    if (!args.gnss_topic.empty()) {
      progress_topics.push_back(args.gnss_topic);
    }
    // The --cam images stream through this same pass (the --color topics do
    // not: they are read again in the later colorize pass, which has its own
    // progress reporting).
    for (const std::string & topic : args.cam_topics) {
      progress_topics.push_back(topic);
    }
    try {
      const auto topic_counts = reader.compute_topic_counts(progress_topics);
      setup.total_msgs = core::slam::progress_total(topic_counts, progress_topics);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_WARN(
        logger, "Could not read bag stats for the progress bar (%s); using an indeterminate bar",
        e.what());
    }
  }
  return setup;
}

FinalizeResult finalize_with_spinner(
  core::slam::CloudMapper & mapper, bool progress_on, const char * logger)
{
  // finish() runs the blocking finalization (global optimization + endpoint
  // window fill + map export) with no per-step progress; animate an indeterminate
  // spinner on a worker thread until it returns.
  FinalizeResult result;
  const auto finalize_start = std::chrono::steady_clock::now();
  {
    core::slam::FinalizeSpinner spinner("Finalizing map", progress_on);
    result.map = mapper.finish();
  }
  result.seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - finalize_start).count();
  // Log the breakdown, not just the total: the endpoint fill (up to a full
  // odometry smoother window of scan registrations), not the iSAM2 update,
  // dominates finalization on LiDAR-only runs, and a bare total reads as
  // "the optimizer is slow".
  BAGWIZ_LOG_INFO(
    logger,
    "Finalization took %.1fs (global optimization %.1fs, endpoint fill %.1fs, "
    "map export %.1fs)",
    result.seconds, result.map.optimize_seconds, result.map.window_fill_seconds,
    result.map.export_seconds);
  return result;
}

std::size_t remove_isolated_map_points(
  core::slam::CloudMap & map, double radius, int min_neighbors, int num_threads)
{
  if (map.points.empty()) {
    return 0;
  }
  std::vector<std::uint8_t> keep(map.points.size(), 1);
  std::size_t removed = 0;
  {
    // The tree references map.points, so it must be gone before the
    // compaction below mutates them.
    const core::pointcloud::KdTree tree(map.points);
    removed = core::pointcloud::mark_radius_outliers(
      map.points, tree, radius, min_neighbors, keep, num_threads);
  }
  if (removed == 0) {
    return 0;
  }
  // Stable in-place compaction keeps points and intensities parallel.
  const bool has_intensities = map.intensities.size() == map.points.size();
  std::size_t write = 0;
  for (std::size_t i = 0; i < map.points.size(); ++i) {
    if (keep[i] == 0) {
      continue;
    }
    map.points[write] = map.points[i];
    if (has_intensities) {
      map.intensities[write] = map.intensities[i];
    }
    ++write;
  }
  map.points.resize(write);
  if (has_intensities) {
    map.intensities.resize(write);
  }
  return removed;
}

bool write_map_outputs(
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  std::span<const core::TrajectoryPose> trajectory, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities, std::span<const std::array<std::uint8_t, 3>> colors,
  const char * logger)
{
  // Open the map stream before committing the trajectory so an unwritable
  // map path fails before either file is touched (rather than leaving an
  // orphaned trajectory behind).
  std::ofstream map_out(map_path, std::ios::binary);
  if (!map_out) {
    BAGWIZ_LOG_ERROR(logger, "could not open %s for writing", map_path.c_str());
    return false;
  }

  std::ofstream traj_out(trajectory_path, std::ios::binary);
  if (!traj_out) {
    BAGWIZ_LOG_ERROR(logger, "could not open %s for writing", trajectory_path.c_str());
    return false;
  }
  core::write_tum(traj_out, trajectory);
  if (!traj_out.good()) {
    BAGWIZ_LOG_ERROR(logger, "write failed: %s", trajectory_path.c_str());
    return false;
  }

  core::slam::write_pcd(map_out, points, intensities, colors);
  // Flush and close before the good() check and before --viewer serves the file.
  // An open ofstream keeps the final partial (<8 KiB) block in its user-space
  // buffer, so until the stream is destroyed the on-disk file is short of its
  // own header's vertex count. serve_map_viewer() (called by the caller, below)
  // blocks while map_out is still in scope, so without this close it would read
  // a too-small file_size, send a truncated body, and the browser's PCD loader
  // would fail with "Offset is outside the bounds of the DataView". close()
  // also surfaces a flush failure (e.g. disk full) through good() below, which
  // the prior mid-write good() check could not see.
  map_out.close();
  if (!map_out.good()) {
    BAGWIZ_LOG_ERROR(logger, "write failed: %s", map_path.c_str());
    return false;
  }
  // Record the map for the exit-time page-cache drop; a multi-GB map.pcd
  // otherwise lingers in the page cache after the process exits.
  io::register_written_file(map_path);
  return true;
}

void log_mapping_summary(
  const core::slam::CloudMap & map, const MapSlamArgs & args, std::int64_t scans,
  std::int64_t skipped, std::int64_t imu_count, std::int64_t gnss_count,
  const std::filesystem::path & trajectory_path, const std::filesystem::path & map_path,
  const char * logger)
{
  if (args.cloud_topic.empty()) {
    // Camera-only mode: the "scans" are the visual-inertial odometry's
    // keyframes, and the map is the sparse landmark set.
    BAGWIZ_LOG_INFO(
      logger,
      "Wrote %zu optimized trajectory poses and a %zu-landmark sparse map from %" PRId64
      " visual keyframe(s)%s to %s and %s",
      map.trajectory.size(), map.points.size(), map.visual_odom_keyframe_count,
      imu_suffix(args, imu_count).c_str(), trajectory_path.string().c_str(),
      map_path.string().c_str());
    if (map.visual_dropped_observation_count > 0) {
      BAGWIZ_LOG_WARN(
        logger,
        "%" PRId64
        " visual observation(s) were dropped by the cross-camera grouping (stamps the "
        "anchor camera's frames left uncovered — mid-run frame drops on the first --cam "
        "topic, or the benign case: other cameras' frames before its first / after its "
        "last frame at the bag's edges)",
        map.visual_dropped_observation_count);
    }
  } else {
    BAGWIZ_LOG_INFO(
      logger,
      "Wrote %zu optimized trajectory poses and a %zu-point map from %zu scans%s (%zu skipped) "
      "to %s and %s",
      map.trajectory.size(), map.points.size(), scans, imu_suffix(args, imu_count).c_str(), skipped,
      trajectory_path.string().c_str(), map_path.string().c_str());
  }

  // --upsample: say how much of the exported file is resampled rather than
  // solved, and name the spans the grid could not cover (the fill windows and
  // any IMU dropout), which stay at scan density.
  if (!map.trajectory_dense.empty()) {
    BAGWIZ_LOG_INFO(
      logger, "Upsampled the trajectory to %zu poses (%zu added by --upsample %s)",
      map.trajectory_dense.size(), map.upsample_grid_poses, args.upsample.c_str());
    if (map.upsample_uncovered_gaps > 0) {
      BAGWIZ_LOG_INFO(
        logger,
        "%zu span(s) kept their scan-rate density: no IMU-rate poses cover them (the "
        "endpoint fill windows, or an IMU gap)",
        map.upsample_uncovered_gaps);
    }
  }

  // The endpoint fills scan-match LiDAR window scans; they are force-disabled
  // in camera-only mode, so only report them in LiDAR modes.
  if (args.fill_start && !args.cloud_topic.empty()) {
    if (map.filled_start_pose_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Filled %zu initialization-window pose(s) by scan-matching",
        map.filled_start_pose_count);
    } else if (map.warmup_overflowed) {
      BAGWIZ_LOG_INFO(
        logger,
        "Initialization-window fill abandoned: the pre-init scan buffer overflowed before "
        "odometry converged (a very long static/slow start)");
    } else {
      BAGWIZ_LOG_INFO(
        logger,
        "No initialization-window poses filled (odometry started immediately, or no "
        "pre-init scans)");
    }
  }

  if (args.fill_end && !args.cloud_topic.empty()) {
    if (map.filled_end_pose_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Filled %zu cooldown-window pose(s) by scan-matching", map.filled_end_pose_count);
    } else {
      BAGWIZ_LOG_INFO(
        logger,
        "No cooldown-window poses filled (no trailing scans past the last estimated "
        "frame)");
    }
  }

  if (!args.gnss_topic.empty()) {
    if (map.gnss_factor_count > 0) {
      BAGWIZ_LOG_INFO(
        logger, "Applied %zu GNSS constraint(s) from %" PRId64 " fix(es) on '%s'",
        map.gnss_factor_count, gnss_count, args.gnss_topic.c_str());
    } else {
      // GNSS was requested but the alignment could not initialize: the map is
      // still valid, just unconstrained by GNSS. Warn rather than fail.
      BAGWIZ_LOG_WARN(
        logger,
        "GNSS topic '%s' yielded no constraints (%s fix(es) read); the global optimization ran "
        "without GNSS. Likely too little motion (baseline) or no temporal overlap between GNSS "
        "and the submaps.",
        args.gnss_topic.c_str(), std::to_string(gnss_count).c_str());
    }
  }

  if (!args.cam_topics.empty()) {
    if (map.visual_factor_count > 0) {
      BAGWIZ_LOG_INFO(
        logger,
        "Applied %" PRId64 " visual constraint(s) from %" PRId64
        " track(s) across %zu --cam "
        "camera(s); the 'visual constraints:' line above breaks the tracks down",
        map.visual_factor_count, map.visual_track_count, args.cam_topics.size());
    } else {
      // --cam was requested but no track survived to a factor: the map is still
      // valid, just unconstrained by the cameras. Warn rather than fail, as
      // GNSS does above.
      BAGWIZ_LOG_WARN(
        logger,
        "--cam camera(s) yielded no visual constraints (%" PRId64
        " track(s) read); the global optimization ran without them. Likely too little "
        "parallax, tracks confined to a single submap, or landmarks the LiDAR-support gate "
        "rejected — see the 'visual constraints:' line above for the breakdown.",
        map.visual_track_count);
    }
  }
}

}  // namespace bagwiz::commands
