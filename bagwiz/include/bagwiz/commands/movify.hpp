// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__MOVIFY_HPP_
#define BAGWIZ__COMMANDS__MOVIFY_HPP_

#include "bagwiz/core/pointcloud/cloud_view.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz movify`. Populated by MovifyCommand's CLI wiring
// (src/commands/movify.cpp) and consumed by run_movify. Kept in a header so
// the run function and the source check can be exercised directly from tests
// without driving the CLI parser.
struct MovifyArgs
{
  MovifyArgs() = default;
  MovifyArgs(
    std::filesystem::path input_path_arg, std::string cam_topic_arg,
    std::filesystem::path output_path_arg, bool overwrite_arg)
  : input_path(std::move(input_path_arg)),
    cam_topics{std::move(cam_topic_arg)},
    output_path(std::move(output_path_arg)),
    overwrite(overwrite_arg)
  {
  }

  std::filesystem::path input_path;
  // Image topics to render as camera panels, in grid order (left to right,
  // top to bottom). Together with pcd_topics and gnss_topic at least one
  // panel is required.
  std::vector<std::string> cam_topics;
  std::filesystem::path output_path;
  // Replace a pre-existing <output>. Without it, an existing output path stops
  // the run before any work is done.
  bool overwrite = false;
  // The topic whose messages define the output frames — one frame per
  // message, its message rate as the frame rate, and its panel's render size
  // as the grid's cell size. Must be one of cam_topics, pcd_topics or
  // gnss_topic; unset picks the first camera panel, else the first
  // point-cloud panel, else the map panel.
  std::optional<std::string> clock;
  // Grid layout as "<cols>x<rows>" (e.g. "2x2"). Empty derives a near-square
  // grid from the panel count.
  std::string grid;
  // Camera-info entries: a bare value applies to every camera panel, an
  // "<image_topic>=<info_topic>" entry overrides one panel. Panels without an
  // entry derive the topic from the image topic name.
  std::vector<std::string> camera_info_entries;
  // Rectify each frame (OpenCV lens-distortion remap) using the resolved
  // camera info. Requested unless --no-rectify opts out; the CLI has no
  // opt-in flag, since this initializer already carries the default. A panel
  // whose camera info cannot be resolved renders unrectified with a warning,
  // unless it projects point clouds (those always require a camera info).
  // "rectify" is the image-side term throughout bagwiz; the point-cloud
  // motion correction keeps the name `pcd undistort`.
  bool rectify = true;
  // Scale the clock panel's frame by this factor while preserving aspect
  // ratio, which sets the cell size (camera clocks only).
  float resize_scale = 1.0f;
  // Fix the composed output width in pixels, deriving the cell size from the
  // grid columns and the clock panel's aspect ratio. Mutually exclusive with
  // resize_scale.
  std::optional<std::uint32_t> width;

  // Point clouds projected onto the camera panels (--cam-pcd). A bare entry
  // (a literal name or '*' glob) projects onto every panel; an
  // "<image_topic>=<pcd_selector>" entry projects onto that panel only.
  std::vector<std::string> cam_pcd_entries;
  // Coloring shared by the camera overlays and the point-cloud panels.
  core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
  std::optional<double> property_min;
  std::optional<double> property_max;
  core::pointcloud::ColorScheme colorscheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 2;
  // Opacity of the camera overlays (the point-cloud panels draw on black).
  float alpha = 1.0f;

  // Point-cloud panels (--pcd): every listed topic is drawn into one panel per
  // view, each cloud transformed at its own stamp into `frame` (unset: the
  // first topic's own frame).
  std::vector<std::string> pcd_topics;
  std::vector<core::pointcloud::CloudProjection> views{
    core::pointcloud::CloudProjection::kPerspective};
  std::optional<std::string> frame;
  // BEV half-extent in meters. Unset: the 95th percentile of the ground
  // distances of the first cloud of the first topic (see kBevAutoRangeQuantile).
  std::optional<double> range_m;
  // Perspective camera on a sphere of radius dist_m around the view frame's
  // origin, looking at it.
  double elev_deg = 20.0;
  double azim_deg = 180.0;
  double dist_m = 30.0;

  // Map panel (--gnss): a NavSatFix topic drawn as the vehicle's track in a
  // local East-North-Up plan view, after the point-cloud panels. `map_range_m`
  // makes the panel follow the current fix at +-range; unset fits the whole
  // track into the panel.
  std::optional<std::string> gnss_topic;
  std::optional<double> map_range_m;

  // Internal toggle for the parallel per-panel pipeline. When false the
  // synchronous loop is used, which composes the same frames without worker
  // threads. Not exposed on the CLI; tests set this directly.
  bool enable_parallel_pipeline = true;
};

// Classification of whether a topic can be rendered to video.
enum class VideoSourceStatus {
  kOk,               // topic present and a supported message type
  kInputUnopenable,  // the bag could not be opened
  kTopicNotFound,    // no topic by that name in the bag
  kUnsupportedType,  // topic present but its message type is not renderable
};

// Outcome of check_video_source(). `topic_type` is set whenever the topic was
// found (kOk or kUnsupportedType); `message` is a human-readable, log-verbatim
// explanation on any non-kOk status. Never throws.
struct VideoSourceCheck
{
  VideoSourceStatus status = VideoSourceStatus::kInputUnopenable;
  std::string topic_type;
  std::string message;

  [[nodiscard]] bool ok() const noexcept { return status == VideoSourceStatus::kOk; }
};

// Resolve `topic` against the bag at `input` and classify whether it can be
// rendered as a camera panel: the message type must be sensor_msgs/msg/Image
// or sensor_msgs/msg/CompressedImage. Reads only the topic list, never
// payloads.
[[nodiscard]] VideoSourceCheck check_video_source(
  const std::filesystem::path & input, const std::string & topic);

// Render the panels named by `args` (camera panels from cam_topics,
// point-cloud panels from pcd_topics, the map panel from gnss_topic) from
// `args.input_path` to a video at
// `args.output_path`, inferring the container/codec from the output extension
// and the frame rate from the clock topic's message timestamps. The panels are
// arranged in a grid (see `grid`); every panel other than the clock shows the
// message of its topic whose bag record time is nearest the clock message's.
// Returns a process exit code: 0 on success, 1 on any error.
int run_movify(const MovifyArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__MOVIFY_HPP_
