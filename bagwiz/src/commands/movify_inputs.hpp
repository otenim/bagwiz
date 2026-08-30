// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_INPUTS_HPP_
#define COMMANDS__MOVIFY_INPUTS_HPP_

#include "bagwiz/commands/movify_video.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

// The input side of `movify cam`: the --pcd / --cam-info entry parsing, the
// pre-flight validation of every topic, the pass-1 scan (frame rate and
// point-cloud index), the pass-2 geometry (camera infos + TF), and the encode
// reader. CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// ---- per-view bindings ----------------------------------------------------------

// --pcd entries after topic expansion: a bare value (already glob-expanded by
// the CLI layer) projects onto every view; an "<image_topic>=<pcd_topic>"
// entry projects onto that view only.
struct PcdBindings
{
  std::vector<std::string> global_topics;
  std::unordered_map<std::string, std::vector<std::string>> per_view;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Split --pcd entries into global topics and per-view bindings. Errors: an
// empty half, or an <image_topic> that is not one of `image_topics`.
[[nodiscard]] PcdBindings parse_pcd_bindings(
  std::span<const std::string> entries, std::span<const std::string> image_topics);

// --cam-info entries: a bare value applies to every view, an
// "<image_topic>=<info_topic>" entry overrides one view.
struct CamInfoEntries
{
  std::optional<std::string> global_topic;
  std::unordered_map<std::string, std::string> per_view;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Split --cam-info entries into the global topic and per-view overrides.
// Errors: an empty half, an <image_topic> outside `image_topics`, a duplicate
// override for one view, or more than one bare (global) value.
[[nodiscard]] CamInfoEntries parse_cam_info_entries(
  std::span<const std::string> entries, std::span<const std::string> image_topics);

// ---- input validation -------------------------------------------------------

// One view's resolved inputs: the image topic and its message type, the
// camera-info topic when the view needs one (nullopt when the run needs none
// for this view or none could be derived), and the point-cloud topics
// projected onto this view (global topics first, then the view's own
// bindings, duplicates removed).
struct ViewInput
{
  std::string topic;
  std::string topic_type;
  std::optional<std::string> camera_info_topic;
  std::vector<std::string> pcd_topics;
};

// Outcome of validate_video_inputs(). `views` is parallel to args.topics and
// `grid` is the resolved layout. `error` is empty on success; on failure it
// holds the message that was already logged.
struct VideoInputValidation
{
  std::vector<ViewInput> views;
  GridSpec grid;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// The command's pre-flight checks: grid parse, duplicate/empty topic
// rejection, every image topic's presence + renderable type, --pcd and
// --cam-info entry parsing, per-view cam-info resolution (explicit entry or
// derivation from the image topic name) and the cam-info requirement of
// rectification / --pcd, and every point-cloud topic's presence + type. Logs
// the command's errors and returns on the first failure.
[[nodiscard]] VideoInputValidation validate_video_inputs(const MovifyVideoArgs & args);

// Whether a validated view renders rectified: rectification must be in effect
// (the default, unless --no-rectify) and the view's camera info must have
// resolved. --no-rectify wins even with --pcd — the projection then targets
// the raw image, applying the camera's lens distortion model instead of
// assuming a rectified one.
[[nodiscard]] bool view_rectifies(bool rectify_requested, const ViewInput & view) noexcept;

// ---- pass 1: frame-rate + point-cloud scan -----------------------------------

// Timestamps + count for a single topic, gathered by a payload-free scan.
struct TopicSpan
{
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
  std::uint64_t count = 0;
};

// Outcome of scan_video_inputs(). pcd_topics is the deduplicated union of
// every view's point-cloud topics in first-use order; pcd_spans and
// pcd_topic_has_stamps are parallel to it, and the pcd_spans entries are
// owned here (the cloud sources move them out). `error` is empty on success;
// on failure it holds the message that was already logged.
struct VideoInputScan
{
  // The primary (first) view's message span, which drives the frame rate.
  TopicSpan span;
  core::video::FrameRate fps;
  std::vector<std::string> pcd_topics;
  std::vector<core::pointcloud::PointCloudIndex> pcd_spans;
  // Per pcd topic: whether it can be matched by capture time (every cloud
  // carried a header.stamp). Topics that fall back to record time are matched
  // by record time on both sides so the overlay stays in one clock.
  std::vector<bool> pcd_topic_has_stamps;
  double global_property_min = 0.0;
  double global_property_max = 0.0;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Pass 1: derive the frame rate from the primary topic's message timestamps,
// require every view's topic to carry at least one message, and, when
// point-cloud overlay topics are given, scan each for its index and the
// selected property's global min/max. Logs the command's errors and returns
// with !ok() on the first failure.
[[nodiscard]] VideoInputScan scan_video_inputs(
  const MovifyVideoArgs & args, const VideoInputValidation & validation);

// ---- pass-2 geometry ---------------------------------------------------------

// The camera infos (one per view, UNSCALED — each view's renderer applies its
// own scale) and TF buffer the encode loop needs for rectification / --pcd,
// loaded up front so a failure aborts before the encode. camera_infos[i] is
// set iff view i resolved a camera-info topic; the TF buffer iff any view
// projects point clouds. Filled via an out parameter because tf2::BufferCore
// is immobile (it owns a mutex), so this struct cannot be returned by value.
struct VideoGeometry
{
  std::vector<std::optional<core::image::CameraInfo>> camera_infos;
  std::optional<tf2::BufferCore> tf_buffer;
};

// Load the pass-2 geometry into `out`: camera info from each view's resolved
// topic, and the bag's TF when point-cloud overlay topics are present.
// Returns "" on success; on failure logs and returns the message.
[[nodiscard]] std::string load_video_geometry(
  const MovifyVideoArgs & args, const VideoInputValidation & validation, VideoGeometry & out);

// ---- pass 2: the encode reader ------------------------------------------------

// Open the input bag for the encode pass, restricted to the primary image
// topic (every other view reads through its own NearestMessageSource). Logs
// "failed to open ..." and returns nullptr on failure.
[[nodiscard]] std::unique_ptr<io::BagReader> open_encode_reader(const MovifyVideoArgs & args);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_INPUTS_HPP_
