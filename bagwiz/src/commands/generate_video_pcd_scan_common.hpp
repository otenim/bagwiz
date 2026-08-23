// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__GENERATE_VIDEO_PCD_SCAN_COMMON_HPP_
#define COMMANDS__GENERATE_VIDEO_PCD_SCAN_COMMON_HPP_

#include "bagwiz/commands/generate_video_pcd_scan.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstdint>
#include <optional>
#include <string>

// Internals of `generate video pcd-scan`, split out of
// generate_video_pcd_scan.cpp so the validation, frame-rate, pass-1 scan, and
// auto-range units can be unit-tested without driving the full command.
// CLI-internal: this header lives with the command sources and is not
// installed. The tmp-file lifecycle, output-path validation, and summary line
// are shared with `generate video cam` through generate_video_common.hpp.
namespace bagwiz::commands
{

// ---- input validation -------------------------------------------------------

// Outcome of validate_pcd_scan_inputs(). On success `first_cloud` holds the
// topic's first message (already parsed, reused by pass 1 for the auto-range
// estimate) and `time_field` the per-point time field resolved on it. `error`
// is empty on success; on failure it holds the message that was already
// logged.
struct PcdScanValidation
{
  core::pointcloud::PointCloud2 first_cloud;
  core::pointcloud::PointTimeField time_field;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// The command's pre-flight checks: source topic presence + PointCloud2 type,
// even canvas dimensions (H.264), positive steps / --range / --dist values,
// and — on the topic's first message — x/y/z fields and a recognised
// per-point time field (required; there is no array-order fallback). Logs the
// command's errors and returns on the first failure.
[[nodiscard]] PcdScanValidation validate_pcd_scan_inputs(const GenerateVideoPcdScanArgs & args);

// ---- frame rate -------------------------------------------------------------

// The output frame rate and the step count that produces it.
struct ScanFrameRate
{
  core::video::FrameRate fps;
  std::uint32_t steps = 1;  // effective steps; <= the requested count
};

// fps = cloud_fps * steps, so one sweep spans `steps` video frames and the
// animation plays in real time. When the product would exceed
// core::video::kMaxFps, the step count is reduced to the largest value that
// stays within the cap (never below 1).
[[nodiscard]] ScanFrameRate derive_scan_frame_rate(
  core::video::FrameRate cloud_fps, std::uint32_t requested_steps);

// ---- pass 1: topic span + auto range -----------------------------------------

// Pass 1: stream the topic's messages reading only their timestamps (no
// payload decode) to learn the count and time span for the frame-rate
// estimate. Returns "" on success; on failure logs and returns the message.
[[nodiscard]] std::string scan_pcd_scan_span(
  const std::filesystem::path & input, const std::string & topic, TopicSpan & out);

// The largest finite XY distance (meters) in `cloud`, used as the default
// --range. Returns nullopt when the cloud has no finite point.
[[nodiscard]] std::optional<double> auto_range_from_cloud(
  const core::pointcloud::PointCloud2 & cloud);

}  // namespace bagwiz::commands

#endif  // COMMANDS__GENERATE_VIDEO_PCD_SCAN_COMMON_HPP_
