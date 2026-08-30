// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/movify.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "movify_camera_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_cloud_panel.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "movify_cloud_source.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_direct.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_inputs.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_panel.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_output.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_pipeline.hpp"      // NOLINT(build/include_subdir) src-local shared header

#include <filesystem>
#include <string>
#include <thread>
#include <utility>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

// Close the stream, move it into place, and log the summary line. Returns
// the process exit code.
int finish_and_report(
  VideoFrameEncoder & encoder, const std::string & clock_topic,
  const std::filesystem::path & tmp_path, const MovifyArgs & args, core::video::FrameRate fps)
{
  if (const auto err =
        finish_video_encode(encoder, clock_topic, tmp_path, args.output_path, args.overwrite);
      !err.empty()) {
    return 1;
  }
  log_video_summary(args.output_path, encoder.written(), encoder.width(), encoder.height(), fps);
  return 0;
}
}  // namespace

int run_movify(const MovifyArgs & args)
{
  // Validate the source topics, camera infos, point-cloud and GNSS topics,
  // the grid, the clock, and the output path before touching anything
  // expensive.
  const auto validation = validate_video_inputs(args);
  if (!validation.ok()) {
    return 1;
  }
  if (const auto err = validate_video_output_path(args.output_path, args.overwrite); !err.empty()) {
    return 1;
  }
  const std::string & clock_topic = clock_topic_of(validation);

  // Pass 1: derive the frame rate, scan the point-cloud topics, and load the
  // GNSS track.
  auto scan = scan_video_inputs(args, validation);
  if (!scan.ok()) {
    return 1;
  }

  // Load camera infos + TF before pass 2 so a failure aborts before the encode.
  VideoGeometry geometry;
  if (const auto err = load_video_geometry(args, validation, geometry); !err.empty()) {
    return 1;
  }

  // Pass 2: decode + encode to a sibling temp path, renamed into place on
  // success. The guard removes the temp on any error exit, so no partial
  // output and no leftover temp survive a failure.
  const std::filesystem::path tmp_path = partial_tmp_path_for(args.output_path);
  PartialFileGuard guard(tmp_path);
  auto reader = open_encode_reader(args.input_path, clock_topic);
  if (!reader) {
    return 1;
  }

  VideoFrameEncoder encoder(
    tmp_path, scan.fps, core::video::VideoEncoderOptions{args.encoder, args.preset});

  // One JPEG camera shown as decoded skips the composed canvas: its frames'
  // planes go to the encoder as they decode, several frames ahead.
  if (can_stream_camera_direct(args, validation)) {
    const unsigned int slots =
      direct_decode_slots(args.enable_parallel_pipeline, std::thread::hardware_concurrency());
    BAGWIZ_LOG_INFO(
      kLogger, "streaming '%s' as decoded, %u frame(s) ahead.", clock_topic.c_str(), slots);
    if (run_direct_encode_pass(*reader, clock_topic, encoder, slots) != 0) {
      return 1;
    }
    return finish_and_report(encoder, clock_topic, tmp_path, args, scan.fps);
  }

  // The point-cloud sources every panel projects from (they take the pass-1
  // index entries out of `scan`), then the panels themselves in grid order,
  // the clock among them: the camera panels, one point-cloud panel per view,
  // then the map panel (which takes the pass-1 track out of `scan`).
  CloudSources clouds(
    args.input_path, scan, geometry.tf_buffer.has_value() ? &*geometry.tf_buffer : nullptr);
  auto panels = build_camera_panels(args, validation, scan, geometry, clouds);
  if (!panels.has_value()) {
    return 1;
  }
  auto cloud_panels = build_cloud_panels(args, validation, scan, clouds, geometry.pose.get());
  if (!cloud_panels.has_value()) {
    return 1;
  }
  for (auto & panel : *cloud_panels) {
    panels->push_back(std::move(panel));
  }
  if (validation.gnss_topic.has_value()) {
    panels->push_back(build_map_panel(args, validation, std::move(*scan.map_track)));
  }
  const bool parallel = should_use_parallel_pipeline(
    panels->size(), !scan.pcd_topics.empty(), args.enable_parallel_pipeline, scan.span.count,
    std::thread::hardware_concurrency());

  if (
    run_encode_pass(
      *reader, *panels, validation.grid, parallel, validation.clock, clock_topic, encoder) != 0) {
    return 1;
  }
  return finish_and_report(encoder, clock_topic, tmp_path, args, scan.fps);
}

}  // namespace bagwiz::commands
