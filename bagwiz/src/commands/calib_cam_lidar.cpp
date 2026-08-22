// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/atomic_write.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/pointcloud/point_cloud_io.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"
#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "calib_cam_lidar_offset.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "pcd_undistort_common.hpp"    // NOLINT(build/include_subdir) src-local shared header
#include "worker_threads.hpp"          // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

// Wires the pure helpers in calib_cam_lidar_common.{hpp,cpp} and the
// calibration core in bagwiz_pointcloud's core::calib namespace into
// `bagwiz calib cam-lidar`'s actual bag-driving run. Everything the
// command needs comes out of the one bag: the --pose topic feeds the --of ->
// --ref trajectory (the same builder `pcd undistort` uses), the --pcd topic's
// clouds are accumulated into the map through that trajectory, and the --cam
// images are sampled against it. Structure mirrors tf_static_dump.cpp: small
// phase helpers, one error path per failure (log + return 1), success writes
// the output YAML and prints the summary/--json report to stdout.
namespace bagwiz::commands
{
namespace
{

constexpr const char * kLogger = "bagwiz.cmd.calib.cam_lidar";
// Sample picks are shrunk this far in from each end of the trajectory span so
// interpolate_trajectory always has bracketing poses on both sides.
constexpr std::int64_t kSampleMarginNs = 3'000'000'000LL;

// Private copies of the two image type strings (is_supported_image_type()/
// to_packed_raster() cover the decode side; this file additionally needs to
// peek at just the header stamp during the cheap phase-6 scan, so it
// re-checks the type here too). Mirrors kImageTopicTypes in
// bagwiz/include/bagwiz/commands/topic_types.hpp together with
// generate_video_common.cpp's and packed_raster.cpp's copies — keep all four
// in sync by hand.
constexpr std::string_view kImageMsgType = "sensor_msgs/msg/Image";
constexpr std::string_view kCompressedImageMsgType = "sensor_msgs/msg/CompressedImage";
// The one type --pcd accepts. Mirrors the private kPointCloud2Type copies
// listed in topic_types.hpp.
constexpr std::string_view kPointCloud2MsgType = "sensor_msgs/msg/PointCloud2";

// ---- small formatting helpers -------------------------------------------

std::string sorted_frames_csv(const tf2::BufferCore & buffer)
{
  std::vector<std::string> frames = buffer.getAllFrameNames();
  std::sort(frames.begin(), frames.end());
  return core::join_csv(frames);
}

std::string edges_csv(const std::vector<std::pair<std::string, std::string>> & edges)
{
  std::vector<std::string> rendered;
  rendered.reserve(edges.size());
  for (const auto & [parent, child] : edges) {
    rendered.push_back(fmt::format("{} -> {}", parent, child));
  }
  return core::join_csv(rendered);
}

// ---- phase 2: --pcd / --pose topics -----------------------------------------

// Validates the --pcd topic (present, PointCloud2) and the --pose topic
// (present, one of the pose types `pcd undistort --pose` accepts) against the
// bag's topic list, logging the command's errors on the first failure.
// Returns the pose topic's TopicInfo (aliasing the reader's internal list) on
// success, nullptr after logging otherwise.
const io::TopicInfo * validate_pcd_and_pose_topics(
  const CalibCamLidarArgs & args, io::BagReader & reader)
{
  const io::TopicInfo * pcd_ti =
    io::find_topic_or_log(reader, args.pcd_topic, args.input_path, kLogger);
  if (pcd_ti == nullptr) {
    return nullptr;
  }
  if (pcd_ti->type != kPointCloud2MsgType) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' has type '%s', which `--pcd` cannot read; expected %s.",
      args.pcd_topic.c_str(), pcd_ti->type.c_str(), std::string(kPointCloud2MsgType).c_str());
    return nullptr;
  }
  const io::TopicInfo * pose_ti =
    io::find_topic_or_log(reader, args.pose_topic, args.input_path, kLogger);
  if (pose_ti == nullptr) {
    return nullptr;
  }
  if (!is_supported_pose_topic_type(pose_ti->type)) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Topic '%s' has type '%s', which `--pose` cannot read; expected tf2_msgs/msg/TFMessage, "
      "nav_msgs/msg/Odometry, geometry_msgs/msg/PoseStamped, or "
      "geometry_msgs/msg/PoseWithCovarianceStamped.",
      args.pose_topic.c_str(), pose_ti->type.c_str());
    return nullptr;
  }
  return pose_ti;
}

// ---- phase 2: image topic + CameraInfo --------------------------------------

std::optional<core::image::CameraInfo> resolve_image_and_cam_info(
  const CalibCamLidarArgs & args, io::BagReader & reader)
{
  const io::TopicInfo * image_topic =
    io::find_topic_or_log(reader, args.cam_topic, args.input_path, kLogger);
  if (image_topic == nullptr) {
    return std::nullopt;
  }
  if (!core::image::is_supported_image_type(image_topic->type)) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Topic '%s' has type '%s', which `calib cam-lidar` cannot read; expected "
      "sensor_msgs/msg/Image or sensor_msgs/msg/CompressedImage.",
      args.cam_topic.c_str(), image_topic->type.c_str());
    return std::nullopt;
  }

  std::string cam_info_topic;
  if (args.cam_info_given) {
    if (const auto err =
          core::camera_info::validate_camera_info_topic(args.input_path, args.cam_info_topic);
        err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return std::nullopt;
    }
    cam_info_topic = args.cam_info_topic;
  } else {
    const auto resolved =
      core::camera_info::resolve_camera_info_topic(args.cam_topic, reader.topics());
    if (!resolved.topic.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "%s. Pass it explicitly with --cam-info.",
        resolved.error.value_or("could not resolve a CameraInfo topic").c_str());
      return std::nullopt;
    }
    cam_info_topic = *resolved.topic;
  }

  auto ci = core::camera_info::load_camera_info(args.input_path, cam_info_topic);
  if (!ci.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
    return std::nullopt;
  }
  if (ci.info->frame_id.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "CameraInfo topic '%s' has an empty header.frame_id (needed as the optical frame).",
      cam_info_topic.c_str());
    return std::nullopt;
  }

  return std::move(*ci.info);
}

// ---- phase 3: trajectory -----------------------------------------------------

// Builds the --of -> --ref trajectory from the --pose topic (the same builder
// `pcd undistort` runs) and requires at least two poses so
// interpolate_trajectory has something to bracket. The TF buffer the builder
// fills is deliberately dropped afterwards: for a TFMessage --pose topic it
// carries replayed dynamic edges, while the chain resolution below works off
// a static-only buffer.
std::optional<std::vector<core::TrajectoryPose>> build_trajectory(
  const CalibCamLidarArgs & args, const io::TopicInfo & pose_ti)
{
  tf2::BufferCore buffer{std::chrono::hours(24 * 365)};
  auto built = build_sorted_of_ref_trajectory(
    args.input_path, pose_ti, args.ref_frame, args.of_frame, /*motion_is_twist=*/false, buffer,
    kLogger);
  if (!built.ok()) {
    if (!built.error.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", built.error.c_str());
    } else {
      BAGWIZ_LOG_ERROR(
        kLogger, "No poses decoded from --pose topic '%s'.", args.pose_topic.c_str());
    }
    return std::nullopt;
  }
  if (built.trajectory.size() < 2) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Trajectory from --pose topic '%s' has fewer than 2 poses; cannot interpolate.",
      args.pose_topic.c_str());
    return std::nullopt;
  }
  return std::move(built.trajectory);
}

// ---- phase 4: static TF chain ---------------------------------------------

// mat4_from_quat over a geometry_msgs Transform, naming the offending leg when
// the recorded quaternion cannot be normalized (zero norm, or a non-finite
// component) rather than letting a NaN-filled matrix poison the whole chain.
std::optional<core::calib::Mat4> mat4_of_transform(
  const geometry_msgs::msg::Transform & transform, const char * leg)
{
  auto m = mat4_from_quat(
    transform.translation.x, transform.translation.y, transform.translation.z, transform.rotation.x,
    transform.rotation.y, transform.rotation.z, transform.rotation.w);
  if (!m.has_value()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "The rotation recorded for %s is not a usable quaternion (zero norm or non-finite).",
      leg);
  }
  return m;
}

// The two fixed legs of the edited edge's chain, plus the edge's own bag value
// as the six scalars the delta is added to (EdgeChain::edge_bag). The edge is
// sourced from a static topic's own transforms rather than the resolved
// buffer, so the report's "bag value" matches what a later `tf static update`
// would see. `tf_buffer` is the caller's static-only buffer (the trajectory
// builder's own buffer may carry replayed dynamic edges from a TFMessage
// --pose topic, which must not leak into this chain).
std::optional<core::calib::EdgeChain> resolve_edge_chain(
  const CalibCamLidarArgs & args, const tf2::BufferCore & tf_buffer,
  const std::string & optical_frame)
{
  const std::vector<std::string> chain_frames =
    core::resolve_chain(tf_buffer, args.of_frame, optical_frame, tf2::TimePointZero);
  if (chain_frames.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "No static TF path from --of '%s' to the image's optical frame '%s'. Available "
      "static frames: %s",
      args.of_frame.c_str(), optical_frame.c_str(), sorted_frames_csv(tf_buffer).c_str());
    return std::nullopt;
  }
  const auto chain_edges = core::chain_to_edges(tf_buffer, chain_frames, tf2::TimePointZero);
  const bool edge_on_chain = std::any_of(
    chain_edges.begin(), chain_edges.end(), [&](const std::pair<std::string, std::string> & e) {
      return e.first == args.parent_frame && e.second == args.child_frame;
    });
  if (!edge_on_chain) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--parent '%s' --child '%s' is not an edge on the chain from --of '%s' to '%s'. "
      "Chain edges: %s",
      args.parent_frame.c_str(), args.child_frame.c_str(), args.of_frame.c_str(),
      optical_frame.c_str(), edges_csv(chain_edges).c_str());
    return std::nullopt;
  }

  std::vector<core::StaticTopicTransforms> static_topics;
  try {
    static_topics =
      core::collect_static_tf(args.input_path, core::StaticTfRead::kFirstMessagePerTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Failed to read static TF from %s: %s", args.input_path.c_str(), e.what());
    return std::nullopt;
  }
  const geometry_msgs::msg::TransformStamped * edge_record = nullptr;
  for (const auto & topic : static_topics) {
    for (const auto & t : topic.transforms) {
      if (t.header.frame_id == args.parent_frame && t.child_frame_id == args.child_frame) {
        edge_record = &t;
        break;
      }
    }
    if (edge_record != nullptr) {
      break;
    }
  }
  if (edge_record == nullptr) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--parent '%s' --child '%s' is on the TF chain but not directly recorded on any static TF "
      "topic (e.g. /tf_static); nothing to refine.",
      args.parent_frame.c_str(), args.child_frame.c_str());
    return std::nullopt;
  }

  core::calib::EdgeChain chain;
  const auto edge_mat = mat4_of_transform(edge_record->transform, "the edited static edge");
  if (!edge_mat.has_value()) {
    return std::nullopt;
  }
  // The delta is added to these six scalars (core::calib::apply_edge_delta),
  // so the edge is carried as scalars from here on and never as a matrix the
  // report and the emitted YAML could re-derive differently.
  const auto edge_t = core::calib::translation_of(*edge_mat);
  const auto edge_rpy = core::calib::rpy_of(*edge_mat);
  chain.edge_bag = {edge_t[0], edge_t[1], edge_t[2], edge_rpy[0], edge_rpy[1], edge_rpy[2]};
  try {
    const auto tf_parent =
      tf_buffer.lookupTransform(args.of_frame, args.parent_frame, tf2::TimePointZero);
    const auto tf_child =
      tf_buffer.lookupTransform(args.child_frame, optical_frame, tf2::TimePointZero);
    const auto parent_mat = mat4_of_transform(tf_parent.transform, "the --of to --parent leg");
    const auto child_mat =
      mat4_of_transform(tf_child.transform, "the --child to camera-optical leg");
    if (!parent_mat.has_value() || !child_mat.has_value()) {
      return std::nullopt;
    }
    chain.t_trajframe_parent = *parent_mat;
    chain.t_child_camoptical = *child_mat;
  } catch (const tf2::TransformException & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not resolve the chain around the edited edge: %s", e.what());
    return std::nullopt;
  }
  return chain;
}

// ---- phase 5: map accumulation ----------------------------------------------

// Accumulates the --pcd topic's clouds into one map in the --ref frame. Each
// cloud is placed by T_ref_of(header.stamp) * T_of_cloud (the extrinsic from
// the static TF tree, cached per distinct cloud frame); clouds with a real
// sweep (a usable, non-uniform per-point time field) are deskewed to their
// header stamp first. Points outside every sample's view (the `views`
// frustum union) are dropped as they go, so the grid only ever holds what
// the samples can look at. The map needs intensity for NID — a cloud
// without it is a hard error, as is an unparseable message. The per-point
// work of every cloud runs on `pool`.
std::optional<core::pointcloud::PcdCloud> accumulate_map(
  const CalibCamLidarArgs & args, std::span<const core::TrajectoryPose> poses,
  const tf2::BufferCore & static_buffer, std::span<const SampleViewFrustum> views,
  core::WorkerPool & pool)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = {args.pcd_topic};
  reader->set_filter(filter);

  // One grid partition per worker, so the insertion pass runs as wide as the
  // placement pass.
  MapAccumulator map{args.voxel_size, pool.size()};
  MapAccumulationStats stats;
  MapAccumulationContext context{views, &pool};
  std::unordered_map<std::string, std::optional<geometry_msgs::msg::Transform>> extrinsic_by_frame;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to parse a cloud on '%s': %s", args.pcd_topic.c_str(),
          parsed.error.c_str());
        return std::nullopt;
      }
      const std::string & cloud_frame = parsed.cloud->frame_id;
      auto [ext_it, inserted] = extrinsic_by_frame.try_emplace(cloud_frame);
      if (inserted) {
        if (cloud_frame == args.of_frame) {
          ext_it->second = std::nullopt;  // identity: the cloud frame already is --of
        } else {
          try {
            ext_it->second =
              static_buffer.lookupTransform(args.of_frame, cloud_frame, tf2::TimePointZero)
                .transform;
          } catch (const tf2::TransformException & e) {
            BAGWIZ_LOG_ERROR(
              kLogger,
              "No static TF path from --of '%s' to the cloud frame '%s' (needed to place the "
              "--pcd clouds): %s",
              args.of_frame.c_str(), cloud_frame.c_str(), e.what());
            return std::nullopt;
          }
        }
      }
      if (const auto err = accumulate_cloud_into_map(
            map, std::move(*parsed.cloud), poses, ext_it->second, stats, context);
          err.has_value()) {
        BAGWIZ_LOG_ERROR(kLogger, "Topic '%s': %s.", args.pcd_topic.c_str(), err->c_str());
        return std::nullopt;
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Error reading topic '%s': %s", args.pcd_topic.c_str(), e.what());
    return std::nullopt;
  }

  if (stats.clouds_read == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "Topic '%s' carries no messages.", args.pcd_topic.c_str());
    return std::nullopt;
  }
  if (map.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "No usable map points: all %" PRIu64
      " cloud(s) on '%s' fell outside the --pose trajectory's time span.",
      stats.clouds_skipped_out_of_span, args.pcd_topic.c_str());
    return std::nullopt;
  }
  if (stats.clouds_skipped_out_of_span > 0) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "%" PRIu64 " of %" PRIu64
      " cloud(s) on '%s' fell outside the --pose trajectory's time span; skipped them.",
      stats.clouds_skipped_out_of_span, stats.clouds_read, args.pcd_topic.c_str());
  }
  if (stats.points_clamped_out_of_span > 0) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "%" PRIu64
      " point(s) fell outside the trajectory's time span while deskewing; their poses were "
      "clamped to the trajectory endpoints.",
      stats.points_clamped_out_of_span);
  }
  if (stats.points_dropped_nonfinite > 0) {
    BAGWIZ_LOG_WARN(
      kLogger, "Dropped %" PRIu64 " non-finite point(s) from the accumulated map.",
      stats.points_dropped_nonfinite);
  }
  if (stats.points_culled_out_of_view > 0) {
    BAGWIZ_LOG_INFO(
      kLogger,
      "Culled %" PRIu64
      " point(s) no image sample can see; the map covers the union of the sample views only.",
      stats.points_culled_out_of_view);
  }
  // Both counts, because the gap between them is the whole point of the grid:
  // a driving platform re-measures each surface once per sweep, and the map
  // keeps one point per voxel out of all of them.
  if (args.voxel_size > 0.0) {
    BAGWIZ_LOG_INFO(
      kLogger,
      "Map: %zu point(s) on a %.3f m voxel grid from %" PRIu64 " cloud(s) on '%s' (%" PRIu64
      " point(s) read, %" PRIu64 " deskewed).",
      map.size(), args.voxel_size, stats.clouds_read, args.pcd_topic.c_str(), stats.points_added,
      stats.clouds_deskewed);
  } else {
    BAGWIZ_LOG_INFO(
      kLogger, "Map: %zu point(s) from %" PRIu64 " cloud(s) on '%s' (%" PRIu64 " deskewed).",
      map.size(), stats.clouds_read, args.pcd_topic.c_str(), stats.clouds_deskewed);
  }
  return map.finish(&pool);
}

// ---- phase 6: cheap stamp scan ---------------------------------------------

// header.stamp for `type`/`payload` (0 when the message's own header carries
// none, e.g. an unset builtin_interfaces/Time, or the payload fails to
// parse). Only the two image types this command reads are recognized; any
// other type reports unset.
// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::int64_t header_stamp_ns_of(std::string_view type, std::span<const std::byte> payload)
{
  if (type == kCompressedImageMsgType) {
    const auto view = core::image::extract_compressed_image(payload);
    return view.ok() ? view.image->header_stamp_ns : 0;
  }
  if (type == kImageMsgType) {
    const auto view = core::image::extract_raw_image(payload);
    return view.ok() ? view.image->header_stamp_ns : 0;
  }
  return 0;
}

std::optional<std::vector<std::int64_t>> scan_image_stamps(
  const CalibCamLidarArgs & args, io::BagReader & reader)
{
  io::ReadFilter filter;
  filter.topics = {args.cam_topic};
  reader.set_filter(filter);

  std::vector<std::int64_t> stamps;
  std::size_t fallback_count = 0;
  io::RawMessage raw;
  try {
    while (reader.next(raw)) {
      const std::int64_t header_stamp_ns = header_stamp_ns_of(raw.topic->type, raw.payload);
      if (header_stamp_ns != 0) {
        stamps.push_back(header_stamp_ns);
      } else {
        stamps.push_back(raw.timestamp_ns);
        ++fallback_count;
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Error reading topic '%s': %s", args.cam_topic.c_str(), e.what());
    return std::nullopt;
  }
  if (stamps.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "Topic '%s' carries no messages.", args.cam_topic.c_str());
    return std::nullopt;
  }
  if (fallback_count > 0) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "%zu of %zu image message(s) on '%s' had no header stamp; using bag record time for those.",
      fallback_count, stamps.size(), args.cam_topic.c_str());
  }
  return stamps;
}

// ---- phase 6b: keyframe-gated sample picking -------------------------------

// Cap on how many members of one gate interval are decoded and
// sharpness-scored. A long stationary interval can hold hundreds of
// near-identical frames; scoring a small evenly-spaced subset finds a sharp
// one without decoding them all.
constexpr std::size_t kMaxScoredPerInterval = 8;

// n indices evenly spread over [0, count), strictly increasing; all of them
// when count <= n. The interval-level counterpart of pick_sample_indices'
// linspace over eligible stamps.
std::vector<std::size_t> evenly_spaced(std::size_t count, std::size_t n)
{
  std::vector<std::size_t> out;
  if (count == 0 || n == 0) {
    return out;
  }
  if (count <= n) {
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
      out[i] = i;
    }
    return out;
  }
  out.reserve(n);
  std::size_t last = count;  // sentinel: no index emitted yet
  for (std::size_t i = 0; i < n; ++i) {
    const double a = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    const auto idx = static_cast<std::size_t>(a * static_cast<double>(count - 1));
    if (idx != last) {
      out.push_back(idx);
      last = idx;
    }
  }
  return out;
}

// Result of the keyframe-gated pick: a flat ascending pick list (indices into
// the image-stamp list) plus, parallel to it, the gate interval each pick
// belongs to. After decoding, each interval keeps only its sharpest member.
struct GatedPicks
{
  std::vector<std::size_t> picks;
  std::vector<std::size_t> group_of;  // parallel to picks
  std::size_t interval_count = 0;     // gate intervals found (pre-selection)
};

// Pose-gate the eligible frames into keyframe intervals, pick args.samples of
// those intervals evenly, and nominate up to kMaxScoredPerInterval candidate
// frames per picked interval. Returns nullopt when fewer than 3 intervals
// exist (a stationary or near-stationary recording) — the caller falls back
// to plain even-time picking with a warning.
std::optional<GatedPicks> keyframe_gated_picks(
  const CalibCamLidarArgs & args, std::span<const std::int64_t> stamps,
  std::span<const core::TrajectoryPose> poses, std::int64_t margin_ns)
{
  const auto eligible = eligible_sample_indices(
    stamps, poses.front().timestamp_ns, poses.back().timestamp_ns, margin_ns);

  // Pose per eligible frame; frames the trajectory cannot interpolate are
  // left out of the gate entirely (assemble_samples would drop them anyway).
  std::vector<std::size_t> gated_eligible;
  std::vector<core::calib::Mat4> gated_poses;
  gated_eligible.reserve(eligible.size());
  gated_poses.reserve(eligible.size());
  for (const auto idx : eligible) {
    const auto pose = interpolate_trajectory(poses, stamps[idx]);
    if (pose.has_value()) {
      gated_eligible.push_back(idx);
      gated_poses.push_back(*pose);
    }
  }

  const auto intervals =
    pose_gate_intervals(gated_poses, args.keyframe_dist, args.keyframe_rot_deg * M_PI / 180.0);
  if (intervals.size() < 3) {
    return std::nullopt;
  }

  GatedPicks out;
  out.interval_count = intervals.size();
  const auto chosen = evenly_spaced(intervals.size(), static_cast<std::size_t>(args.samples));
  for (std::size_t g = 0; g < chosen.size(); ++g) {
    const auto [begin, end] = intervals[chosen[g]];
    for (const auto member : evenly_spaced(end - begin, kMaxScoredPerInterval)) {
      out.picks.push_back(gated_eligible[begin + member]);
      out.group_of.push_back(g);
    }
  }
  return out;
}

// ---- phase 7: decode the picked samples ------------------------------------

struct DecodedSample
{
  core::calib::GrayImage gray;
  std::int64_t stamp_ns = 0;
  // Index into the picks list this sample decoded from, so the keyframe-gated
  // path can map a decoded frame back to its gate interval even when other
  // picks were dropped by decode failures.
  std::size_t pick_ordinal = 0;
};

std::optional<std::vector<DecodedSample>> decode_picked_samples(
  const CalibCamLidarArgs & args, std::span<const std::size_t> picks,
  std::span<const std::int64_t> stamps)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = {args.cam_topic};
  reader->set_filter(filter);

  std::vector<DecodedSample> decoded;
  decoded.reserve(picks.size());
  io::RawMessage raw;
  try {
    std::size_t pick_i = 0;
    std::size_t idx = 0;
    while (pick_i < picks.size() && reader->next(raw)) {
      if (idx != picks[pick_i]) {
        ++idx;
        continue;
      }
      const auto packed = core::image::to_packed_raster(raw.topic->type, raw.payload);
      if (!packed.ok()) {
        BAGWIZ_LOG_WARN(
          kLogger, "Dropping sample at %" PRId64 " ns: failed to decode: %s", stamps[idx],
          packed.error.c_str());
        ++idx;
        ++pick_i;
        continue;
      }
      DecodedSample sample;
      sample.gray = core::calib::gray_from_bgr24(
        packed.raster->bgr, packed.raster->width, packed.raster->height);
      sample.stamp_ns = stamps[idx];
      sample.pick_ordinal = pick_i;
      decoded.push_back(std::move(sample));
      ++idx;
      ++pick_i;
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Error reading topic '%s': %s", args.cam_topic.c_str(), e.what());
    return std::nullopt;
  }
  return decoded;
}

// Reduce decoded candidates to one per gate interval: the sharpest member
// (gray_sharpness), matching `map slam --color-keyframe-blur`'s policy that a
// motion-blurred frame is worse than a redundant one.
std::vector<DecodedSample> sharpest_per_group(
  std::vector<DecodedSample> && decoded, std::span<const std::size_t> group_of)
{
  std::map<std::size_t, std::pair<double, std::size_t>> best;  // group -> (score, decoded idx)
  for (std::size_t i = 0; i < decoded.size(); ++i) {
    const double score = gray_sharpness(decoded[i].gray);
    const std::size_t group = group_of[decoded[i].pick_ordinal];
    const auto it = best.find(group);
    if (it == best.end() || score > it->second.first) {
      best[group] = {score, i};
    }
  }
  std::vector<DecodedSample> reduced;
  reduced.reserve(best.size());
  for (auto & entry : best) {
    reduced.push_back(std::move(decoded[entry.second.second]));
  }
  return reduced;
}

// ---- phase 8: pre-cull candidates + assemble CalibSamples ------------------

// Pixel-bound padding around the initial-estimate projection: the trust
// region lets refine_extrinsic move the extrinsic by up to max_rot_rad /
// max_trans, which shifts where a map point lands on the image, so the
// pre-cull at delta=0 has to keep a margin wide enough that a point the
// optimizer might need is not thrown away before it ever sees it. +64 is
// slack for the depth-cull cell size and general roundoff.
int precull_pad_px(
  const core::calib::CameraModel & cam, double max_rot_rad, double max_trans, double min_depth)
{
  return static_cast<int>(std::ceil(cam.k[0] * (max_rot_rad + max_trans / min_depth))) + 64;
}

// Project `cloud`'s points through `t_cam_world` (the initial, delta=0
// estimate) and keep the ones that land within the depth window and a
// pad_px-widened pixel bound, appending each kept point's raw intensity to
// `candidate_intensities` (parallel to sample.points_world). The intensities
// of ALL samples' candidates are later histogram-equalized as one union, so
// the NID bins are decided by exactly the points NID scores — not by the
// map's coverage, which the frustum cull, the voxel size, or the map source
// would otherwise leak into the binning.
void fill_precull_candidates(
  const core::pointcloud::PcdCloud & cloud, const core::calib::CameraModel & cam,
  const core::calib::Mat4 & t_cam_world, double min_depth, double max_depth, int pad_px,
  core::calib::CalibSample & sample, std::vector<float> & candidate_intensities)
{
  const double lo_u = -static_cast<double>(pad_px);
  const double lo_v = -static_cast<double>(pad_px);
  const double hi_u = static_cast<double>(cam.width) + pad_px;
  const double hi_v = static_cast<double>(cam.height) + pad_px;
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto & p = cloud.points[i];
    const auto pc = core::calib::transform_point(
      t_cam_world,
      {static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])});
    if (pc[2] < min_depth || pc[2] > max_depth) {
      continue;
    }
    const auto nd = core::image::distort_normalized(pc[0] / pc[2], pc[1] / pc[2], cam.model, cam.d);
    const double u = cam.k[0] * nd.x + cam.k[2];
    const double v = cam.k[4] * nd.y + cam.k[5];
    if (u < lo_u || v < lo_v || u >= hi_u || v >= hi_v) {
      continue;
    }
    sample.points_world.push_back(p);
    candidate_intensities.push_back(cloud.intensities[i]);
  }
}

// The frustum union the map accumulation culls against: one view per decoded
// sample, in the --ref frame. The bounds are the pinhole projection of the
// precull-padded pixel rectangle, widened by a fixed margin so that a point
// the distortion model pushes INTO the padded rect is still kept — the cull
// must be a superset of the exact per-sample predicate, and a few extra
// points only cost memory. Samples whose stamp falls outside the
// trajectory get no view; assemble_samples drops them anyway.
std::vector<SampleViewFrustum> build_sample_frusta(
  const CalibCamLidarArgs & args, std::span<const DecodedSample> decoded,
  std::span<const core::TrajectoryPose> poses, const core::calib::CameraModel & cam,
  const core::calib::Mat4 & t_trajframe_cam0)
{
  const double max_rot_rad = args.max_rot_deg * M_PI / 180.0;
  const int pad_px = precull_pad_px(cam, max_rot_rad, args.max_trans, args.min_depth);
  const auto widen = [](double lo, double hi) {
    const double mid = 0.5 * (lo + hi);
    const double half = 0.5 * (hi - lo) * 1.1 + 0.02;
    return std::pair{mid - half, mid + half};
  };
  const auto [lo_xn, hi_xn] = widen(
    (-static_cast<double>(pad_px) - cam.k[2]) / cam.k[0],
    (static_cast<double>(cam.width) + pad_px - cam.k[2]) / cam.k[0]);
  const auto [lo_yn, hi_yn] = widen(
    (-static_cast<double>(pad_px) - cam.k[5]) / cam.k[4],
    (static_cast<double>(cam.height) + pad_px - cam.k[5]) / cam.k[4]);

  std::vector<SampleViewFrustum> views;
  views.reserve(decoded.size());
  for (const auto & d : decoded) {
    const auto pose = interpolate_trajectory(poses, d.stamp_ns);
    if (!pose.has_value()) {
      continue;
    }
    SampleViewFrustum v;
    v.t_cam_ref = core::calib::rigid_inverse(core::calib::mat4_multiply(*pose, t_trajframe_cam0));
    v.lo_xn = lo_xn;
    v.hi_xn = hi_xn;
    v.lo_yn = lo_yn;
    v.hi_yn = hi_yn;
    v.lo_depth = args.min_depth;
    v.hi_depth = args.max_depth;
    views.push_back(v);
  }
  return views;
}

// The per-sample pre-cull runs on `pool`, one sample per task: every sample
// projects the whole map on its own, and the outcome per sample depends on
// that sample alone. The dropped-sample warnings and the candidate union are
// then assembled in sample order, exactly as a serial loop would have.
std::optional<std::vector<core::calib::CalibSample>> assemble_samples(
  const CalibCamLidarArgs & args, std::span<const DecodedSample> decoded,
  std::span<const core::TrajectoryPose> poses, const core::pointcloud::PcdCloud & cloud,
  const core::calib::CameraModel & cam, const core::calib::Mat4 & t_trajframe_cam0,
  core::WorkerPool & pool)
{
  const double max_rot_rad = args.max_rot_deg * M_PI / 180.0;
  const int pad_px = precull_pad_px(cam, max_rot_rad, args.max_trans, args.min_depth);
  const core::calib::NidParams default_nid;

  enum class Drop { kNone, kImageSize, kOutsideTrajectory, kTooFewCandidates };
  struct Candidate
  {
    core::calib::CalibSample sample;
    std::vector<float> intensities;  // parallel to sample.points_world
    Drop drop = Drop::kNone;
  };
  std::vector<Candidate> candidates(decoded.size());
  pool.parallel_for(decoded.size(), [&](std::size_t i) {
    const auto & d = decoded[i];
    auto & c = candidates[i];
    if (d.gray.width != cam.width || d.gray.height != cam.height) {
      c.drop = Drop::kImageSize;
      return;
    }
    const auto pose = interpolate_trajectory(poses, d.stamp_ns);
    if (!pose.has_value()) {
      c.drop = Drop::kOutsideTrajectory;
      return;
    }
    const core::calib::Mat4 t_cam_world0 =
      core::calib::rigid_inverse(core::calib::mat4_multiply(*pose, t_trajframe_cam0));
    c.sample.image = d.gray;
    c.sample.t_world_trajframe = *pose;
    fill_precull_candidates(
      cloud, cam, t_cam_world0, args.min_depth, args.max_depth, pad_px, c.sample, c.intensities);
    if (c.sample.points_world.size() < default_nid.min_points) {
      c.drop = Drop::kTooFewCandidates;
    }
  });

  std::vector<core::calib::CalibSample> samples;
  samples.reserve(decoded.size());
  std::vector<float> candidate_intensities;  // union over samples, in sample order
  std::vector<std::size_t> sample_sizes;
  for (std::size_t i = 0; i < decoded.size(); ++i) {
    const auto & d = decoded[i];
    auto & c = candidates[i];
    switch (c.drop) {
      case Drop::kImageSize:
        BAGWIZ_LOG_WARN(
          kLogger,
          "Dropping sample at %" PRId64 " ns: image size %ux%u does not match CameraInfo %ux%u.",
          d.stamp_ns, d.gray.width, d.gray.height, cam.width, cam.height);
        continue;
      case Drop::kOutsideTrajectory:
        BAGWIZ_LOG_WARN(
          kLogger,
          "Dropping sample at %" PRId64 " ns: outside the trajectory's interpolation range.",
          d.stamp_ns);
        continue;
      case Drop::kTooFewCandidates:
        BAGWIZ_LOG_WARN(
          kLogger,
          "Dropping sample at %" PRId64 " ns: only %zu candidate map point(s) project into view.",
          d.stamp_ns, c.sample.points_world.size());
        continue;
      case Drop::kNone:
        break;
    }
    candidate_intensities.insert(
      candidate_intensities.end(), c.intensities.begin(), c.intensities.end());
    sample_sizes.push_back(c.sample.points_world.size());
    samples.push_back(std::move(c.sample));
  }
  if (samples.size() < 3) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Only %zu of %zu decoded sample(s) survived pre-culling; at least 3 are needed. Check "
      "--of, the TF chain, and --min-depth/--max-depth.",
      samples.size(), decoded.size());
    return std::nullopt;
  }
  // Histogram-equalize the union of candidate intensities: the NID bins are
  // decided by exactly the points NID scores, so the binning cannot drift
  // with the map's coverage (frustum culling, the voxel size, or the map
  // source) — only with what the samples actually look at.
  const auto bins = core::calib::equalize_intensity_bins(candidate_intensities, args.nid_bins);
  std::size_t pos = 0;
  for (std::size_t s = 0; s < samples.size(); ++s) {
    samples[s].intensity_bins.assign(
      bins.begin() + static_cast<std::ptrdiff_t>(pos),
      bins.begin() + static_cast<std::ptrdiff_t>(pos + sample_sizes[s]));
    pos += sample_sizes[s];
  }
  return samples;
}

}  // namespace

int run_calib_cam_lidar(const CalibCamLidarArgs & args)
{
  // 1. Cross-field validation, the output path included. The check is
  // non-destructive — the existing entry is removed only at the write site in
  // step 10 — so it runs here rather than after the refinement: a collision
  // that would otherwise surface minutes later now costs one stat.
  if (const auto err = validate_calibrate_flags(args); !err.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
    return 1;
  }
  const auto fix_spec = parse_fix_spec(args.fix_axes).first;
  const std::filesystem::path out_path =
    args.output_path.empty()
      ? std::filesystem::path(default_calib_cam_lidar_output_path(args.input_path))
      : std::filesystem::path(args.output_path);
  if (const auto r = core::check_output_path_free(out_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }
  // One worker pool for the whole run, shared by every pass with per-point or
  // per-sample work, sized by -j/--threads (0 = the hardware concurrency).
  core::WorkerPool pool{resolve_num_threads(args.threads, std::thread::hardware_concurrency())};

  // 2. Topics: --pcd (PointCloud2), --pose (a supported pose type), and the
  // --cam image topic + its CameraInfo.
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  const io::TopicInfo * pose_ti = validate_pcd_and_pose_topics(args, *reader);
  if (pose_ti == nullptr) {
    return 1;
  }
  const auto cam_info = resolve_image_and_cam_info(args, *reader);
  if (!cam_info.has_value()) {
    return 1;
  }

  // 3. Trajectory from the --pose topic (T_ref_of, sorted).
  auto poses = build_trajectory(args, *pose_ti);
  if (!poses.has_value()) {
    return 1;
  }

  // 3b. --skip-start / --skip-end: trim the trajectory to the bag's time
  // extent minus the skipped durations. Everything downstream keys off the
  // trajectory span — image-sample eligibility, the per-cloud span check, and
  // deskew clamping — so this one trim excludes the skipped ranges from the
  // whole estimation. The first/last surviving pose can sit up to one pose
  // period inside the window, so a sliver of the requested window at each end
  // is treated as out-of-span; consistent with the existing out-of-span skip.
  const auto skip_ns = parse_skip_durations(args).first;  // validated in step 1
  if (skip_ns[0] > 0 || skip_ns[1] > 0) {
    const auto extent = reader->compute_time_extent();
    if (!extent.has_data) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--skip-start/--skip-end need the bag's time extent, which '%s' does not provide.",
        args.input_path.c_str());
      return 1;
    }
    const std::int64_t window_lo = extent.start_ns + skip_ns[0];
    const std::int64_t window_hi = extent.end_ns - skip_ns[1];
    if (window_lo >= window_hi) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--skip-start plus --skip-end cover the whole bag (bag duration %.3f s).",
        static_cast<double>(extent.end_ns - extent.start_ns) / 1e9);
      return 1;
    }
    std::erase_if(*poses, [&](const core::TrajectoryPose & p) {
      return p.timestamp_ns < window_lo || p.timestamp_ns > window_hi;
    });
    if (poses->size() < 2) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--skip-start/--skip-end leave fewer than 2 trajectory pose(s) inside the window; cannot "
        "interpolate.");
      return 1;
    }
    BAGWIZ_LOG_INFO(
      kLogger,
      "Skipping [bag start, bag start + %.3f s) and (bag end - %.3f s, bag end]: trajectory "
      "trimmed to %zu pose(s) spanning %.3f s.",
      static_cast<double>(skip_ns[0]) / 1e9, static_cast<double>(skip_ns[1]) / 1e9, poses->size(),
      static_cast<double>(poses->back().timestamp_ns - poses->front().timestamp_ns) / 1e9);
  }

  // 4. Static TF chain around the edited edge, off a static-only buffer the
  // map accumulation also resolves the cloud extrinsic from.
  tf2::BufferCore static_buffer{std::chrono::hours(24 * 365)};
  if (const auto err = core::load_static_tf_buffer(args.input_path, static_buffer);
      err.has_value()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
    return 1;
  }
  const auto chain = resolve_edge_chain(args, static_buffer, cam_info->frame_id);
  if (!chain.has_value()) {
    return 1;
  }
  // The initial (delta=0) camera pose chain, shared by the offset estimate,
  // the frustum cull and the candidate assembly.
  const core::calib::Mat4 t_trajframe_cam0 = core::calib::mat4_multiply(
    chain->t_trajframe_parent,
    core::calib::mat4_multiply(
      core::calib::edge_transform(chain->edge_bag, {}), chain->t_child_camoptical));

  // 5. Cheap stamp scan + sample picking (reuses `reader`; no next() has run
  // on it yet, so set_filter here is still legal). Ahead of the map pass:
  // picking needs only image stamps and the trajectory, and the picks decide
  // which part of the scene the map must cover at all.
  auto stamps = scan_image_stamps(args, *reader);
  if (!stamps.has_value()) {
    return 1;
  }

  // 5a. --cam-offset: the manual value, or under `auto` the estimate measured
  // from these images against the trajectory (or the --imu gyro). Applied to
  // every stamp here, at the one place the image times enter the command, so
  // every consumer — sample eligibility and picking, the keyframe gate, the
  // frustum cull, the pre-cull and each sample's trajectory pose — sees the
  // same shifted time and the image stamped t is placed at pose(t + offset).
  // A stamp that fell back to the bag record time is shifted the same way:
  // the offset corrects the image's time whatever its source.
  CamOffsetReport cam_offset;
  {
    const CamOffsetSpec spec = parse_cam_offset(args).first;  // validated in step 1
    if (spec.auto_estimate) {
      const auto estimate = estimate_cam_offset(
        CamOffsetEstimateInput{args, *stamps, *poses, *cam_info, t_trajframe_cam0, static_buffer});
      if (!estimate.has_value()) {
        return 1;
      }
      cam_offset.estimate = *estimate;
      cam_offset.applied_ns = estimate->offset_ns;
    } else {
      cam_offset.applied_ns = spec.offset_ns;
    }
    if (cam_offset.applied_ns != 0) {
      for (auto & stamp_ns : *stamps) {
        stamp_ns += cam_offset.applied_ns;
      }
      BAGWIZ_LOG_INFO(
        kLogger,
        "Applying --cam-offset %+.3f ms to %zu image stamp(s) on '%s': each image is placed at "
        "pose(stamp + offset).",
        static_cast<double>(cam_offset.applied_ns) / 1e6, stamps->size(), args.cam_topic.c_str());
    }
  }
  std::vector<std::size_t> picks;
  std::vector<std::size_t> group_of;  // gated mode only; parallel to picks
  bool gated = args.keyframe_dist > 0.0 || args.keyframe_rot_deg > 0.0;
  if (gated) {
    const auto gp = keyframe_gated_picks(args, *stamps, *poses, kSampleMarginNs);
    if (gp.has_value()) {
      picks = gp->picks;
      group_of = gp->group_of;
      BAGWIZ_LOG_INFO(
        kLogger,
        "Keyframe gate: %zu interval(s); scoring %zu candidate frame(s) for up to %d sample(s).",
        gp->interval_count, picks.size(), args.samples);
    } else {
      gated = false;
      BAGWIZ_LOG_WARN(
        kLogger,
        "Keyframe gate found fewer than 3 intervals (near-stationary trajectory); falling back "
        "to even time spacing.");
    }
  }
  if (!gated) {
    picks = pick_sample_indices(
      *stamps, poses->front().timestamp_ns, poses->back().timestamp_ns, args.samples,
      kSampleMarginNs);
  }
  if (picks.size() < 3) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Only %zu image sample(s) fall inside the trajectory span (with margin); at least 3 are "
      "needed. Image stamps span [%" PRId64 ", %" PRId64 "] ns, trajectory spans [%" PRId64
      ", %" PRId64 "] ns.",
      picks.size(), stamps->front(), stamps->back(), poses->front().timestamp_ns,
      poses->back().timestamp_ns);
    return 1;
  }

  // 6. Decode only the picked messages + build the camera model from the
  // first one. In gated mode every interval's candidates are decoded and the
  // sharpest one per interval survives.
  auto decoded = decode_picked_samples(args, picks, *stamps);
  if (!decoded.has_value()) {
    return 1;  // decode_picked_samples already logged the specific read error.
  }
  if (gated) {
    *decoded = sharpest_per_group(std::move(*decoded), group_of);
  }
  if (decoded->size() < 3) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Only %zu of %zu picked sample(s) decoded successfully; at least 3 are needed.",
      decoded->size(), picks.size());
    return 1;
  }
  const auto & first = decoded->front();
  const core::image::CameraInfo scaled_cam_info =
    core::image::camera_info_for_size(*cam_info, first.gray.width, first.gray.height);
  core::calib::CameraModel cam;
  cam.k = scaled_cam_info.k;
  cam.model = core::image::select_distortion_model(scaled_cam_info.distortion_model);
  cam.d = scaled_cam_info.d;
  cam.width = first.gray.width;
  cam.height = first.gray.height;

  // 7. Map: accumulate the --pcd topic's clouds into the --ref frame,
  // dropping points no picked sample can see as they arrive.
  const auto frusta = build_sample_frusta(args, *decoded, *poses, cam, t_trajframe_cam0);
  const auto cloud = accumulate_map(args, *poses, static_buffer, frusta, pool);
  if (!cloud.has_value()) {
    return 1;
  }

  // 8. Pre-cull candidates from the map and assemble the per-sample structs.
  const auto samples =
    assemble_samples(args, *decoded, *poses, *cloud, cam, t_trajframe_cam0, pool);
  if (!samples.has_value()) {
    return 1;
  }

  // 9. Refine.
  core::calib::RefineParams params;
  params.nid.bins = args.nid_bins;
  params.nid.min_depth = args.min_depth;
  params.nid.max_depth = args.max_depth;
  params.fixed = fix_spec.fixed;
  params.auto_fix = fix_spec.auto_fix;
  params.max_trans = args.max_trans;
  params.max_rot = args.max_rot_deg * M_PI / 180.0;
  const auto result = core::calib::refine_extrinsic(*samples, cam, *chain, params);
  if (!result.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "Refinement failed: %s", result.error.c_str());
    return 1;
  }

  // 10. Output: the refined edge as a one-transform static-TF-tree YAML. The
  // refined edge is the bag's six scalars plus the delta, axis by axis — the
  // exact composition refine_extrinsic minimized over and the one
  // render_calibrate_summary/json print, so the file and the report cannot
  // describe different edges.
  const std::array<double, 6> & edge_before = chain->edge_bag;
  const auto edge_after = core::calib::apply_edge_delta(edge_before, result.delta);

  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = args.parent_frame;
  ts.child_frame_id = args.child_frame;
  ts.transform.translation.x = edge_after[0];
  ts.transform.translation.y = edge_after[1];
  ts.transform.translation.z = edge_after[2];
  ts.transform.rotation = core::rpy_to_quaternion({edge_after[3], edge_after[4], edge_after[5]});

  const std::string yaml = core::emit_static_tf_tree_yaml(
    std::span<const geometry_msgs::msg::TransformStamped>(&ts, 1),
    args.input_path.string() + " (bagwiz calib cam-lidar)");

  if (const auto r = core::prepare_output_path(out_path, args.overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }
  std::string write_error;
  if (!core::write_file_atomically(out_path, yaml, write_error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", out_path.c_str(), write_error.c_str());
    return 1;
  }

  const std::string report =
    args.json ? render_calibrate_json(args, result, edge_before, cam_offset)
              : render_calibrate_summary(args, result, edge_before, out_path.string(), cam_offset);
  fmt::print(stdout, "{}{}", report, args.json ? "\n" : "");
  if (std::fflush(stdout) != 0) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to write the calibration report to stdout");
    return 1;
  }
  BAGWIZ_LOG_INFO(
    kLogger, "calib cam-lidar: wrote '%s' (%d sample(s), NID %.6f -> %.6f).", out_path.c_str(),
    result.samples_used, result.nid_before, result.nid_after);
  return 0;
}

}  // namespace bagwiz::commands
