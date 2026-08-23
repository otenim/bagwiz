// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_
#define BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_

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

// Arguments for `bagwiz generate video cam`. Populated by GenerateCommand's CLI
// wiring (src/commands/generate.cpp) and consumed by run_generate_video. Kept
// in a header so the run function and the source check can be exercised
// directly from tests without driving the CLI parser.
struct GenerateVideoArgs
{
  GenerateVideoArgs() = default;
  GenerateVideoArgs(
    std::filesystem::path input_path_arg, std::string topic_arg,
    std::filesystem::path output_path_arg, bool overwrite_arg)
  : input_path(std::move(input_path_arg)),
    topics{std::move(topic_arg)},
    output_path(std::move(output_path_arg)),
    overwrite(overwrite_arg)
  {
  }

  std::filesystem::path input_path;
  // Image topics to render, in grid order (left to right, top to bottom). The
  // first topic is primary: it drives the frame rate and the output timing,
  // and its (resized) frame size fixes the grid's cell size.
  std::vector<std::string> topics;
  std::filesystem::path output_path;
  // Replace a pre-existing <output>. Without it, an existing output path stops
  // the run before any work is done.
  bool overwrite = false;
  // Grid layout as "<cols>x<rows>" (e.g. "2x2"). Empty derives a near-square
  // grid from the topic count.
  std::string grid;
  // Camera-info entries: a bare value applies to every view, an
  // "<image_topic>=<info_topic>" entry overrides one view. Views without an
  // entry derive the topic from the image topic name.
  std::vector<std::string> camera_info_entries;
  // Rectify each frame (OpenCV lens-distortion remap) using the resolved
  // camera info. On by default; --no-rectify opts out. A view whose camera
  // info cannot be resolved renders unrectified with a warning, unless it
  // projects point clouds (those always require a camera info).
  // "rectify" is the image-side term throughout bagwiz; the point-cloud
  // motion correction keeps the name `pcd undistort`.
  bool rectify = true;
  // Scale output dimensions by this factor while preserving aspect ratio.
  float resize_scale = 1.0f;
  // Fix the composed output width in pixels, deriving the cell size from the
  // grid columns and the primary frame's aspect ratio. Mutually exclusive
  // with resize_scale.
  std::optional<std::uint32_t> width;

  // Point-cloud overlay options. A bare entry (a literal name or '*' glob)
  // projects onto every view; an "<image_topic>=<pcd_selector>" entry projects
  // onto that view only.
  std::vector<std::string> pointcloud_topics;
  core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
  std::optional<double> property_min;
  std::optional<double> property_max;
  core::pointcloud::ColorScheme colorscheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 2;
  float alpha = 1.0f;
  // Internal toggle for the threaded point-cloud projection pipeline. When false
  // the synchronous path is used, which keeps output bit-for-bit identical to the
  // pre-threading implementation. Not exposed on the CLI; tests set this directly.
  bool enable_threaded_projection = true;
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
// rendered to video: the message type must be sensor_msgs/msg/Image or
// sensor_msgs/msg/CompressedImage. Reads only the topic list, never payloads.
[[nodiscard]] VideoSourceCheck check_video_source(
  const std::filesystem::path & input, const std::string & topic);

// Render `args.topics` from `args.input_path` to a video at `args.output_path`,
// inferring the container/codec from the output extension and the frame rate
// from the first (primary) topic's message timestamps. With several topics the
// views are arranged in a grid (see `grid`); each non-primary view shows the
// frame whose bag record time is nearest the primary frame's. Returns a process
// exit code: 0 on success, 1 on any error. Renders raw sensor_msgs/msg/Image
// (bgr8 / rgb8) and sensor_msgs/msg/CompressedImage (JPEG / PNG, decoded to BGR
// before encoding).
int run_generate_video(const GenerateVideoArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_
