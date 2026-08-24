// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_PCD_SCAN_COMMON_HPP_
#define COMMANDS__MOVIFY_PCD_SCAN_COMMON_HPP_

#include "bagwiz/commands/movify_pcd_scan.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstdint>
#include <optional>
#include <string>

// Internals of `movify scan`, split out of
// movify_pcd_scan.cpp so the validation, frame-rate, pass-1 scan, and
// auto-range units can be unit-tested without driving the full command.
// CLI-internal: this header lives with the command sources and is not
// installed. The tmp-file lifecycle, output-path validation, and summary line
// are shared with `movify cam` through movify_video_common.hpp.
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
// even canvas dimensions (H.264), a valid --fps / --speed / --range / --dist,
// and — on the topic's first message — x/y/z fields and a recognised
// per-point time field (required; there is no array-order fallback). Logs the
// command's errors and returns on the first failure.
[[nodiscard]] PcdScanValidation validate_pcd_scan_inputs(const MovifyPcdScanArgs & args);

// ---- sweep timing -------------------------------------------------------------

// Video frames rendered per sweep: round(fps / (cloud_fps * speed)), never
// below 1. `cloud_fps` is the topic's message rate from derive_frame_rate.
// Below 1 (fps < cloud rate * speed) each sweep gets exactly one frame, so
// the animation plays slower than the requested speed — the caller warns.
[[nodiscard]] std::uint32_t scan_frames_per_sweep(
  core::video::FrameRate cloud_fps, std::uint32_t fps, double speed);

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

#endif  // COMMANDS__MOVIFY_PCD_SCAN_COMMON_HPP_
