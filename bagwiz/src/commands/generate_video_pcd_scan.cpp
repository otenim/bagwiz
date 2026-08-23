// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video_pcd_scan.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/scan_pattern.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "generate_video_pcd_scan_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";

// One sweep of the encode loop: sort the cloud's points by firing time, then
// emit `steps` cumulative frames, drawing only the points that fired since the
// previous frame onto the persistent canvas.
//
// The canvas is reused across write_frame() calls: write_frame() converts the
// pixels into the encoder's own buffers (swscale) before returning, so it
// never retains the caller's buffer.
int encode_sweep(
  const core::pointcloud::PointCloud2 & cloud, const core::pointcloud::PointTimeField & time_field,
  const core::pointcloud::ScanPatternView & view, std::uint32_t steps,
  core::pointcloud::ColorScheme scheme, std::uint32_t point_size,
  core::video::VideoEncoder & encoder, std::uint64_t & written)
{
  const auto times = core::pointcloud::extract_scan_times(cloud, time_field);
  if (!times.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", times.error.c_str());
    return 1;
  }
  const auto order = core::pointcloud::sorted_scan_indices(times.times);

  core::image::PackedRaster canvas;
  canvas.width = view.width;
  canvas.height = view.height;
  canvas.encoding = "bgr8";
  canvas.bgr.assign(static_cast<std::size_t>(view.width) * view.height * 3, std::byte{0});

  if (order.empty()) {
    // No finite per-point time in this sweep: keep the timeline by emitting
    // blank frames.
    for (std::uint32_t k = 0; k < steps; ++k) {
      if (const auto err = encoder.write_frame(
            canvas.bgr, canvas.width * 3, core::video::SourcePixelFormat::kBgr8);
          !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
        return 1;
      }
      ++written;
    }
    return 0;
  }

  // Sorted per-point times; the sub-frame thresholds binary-search this.
  std::vector<double> sorted_times;
  sorted_times.reserve(order.size());
  for (const auto i : order) {
    sorted_times.push_back(times.times[i]);
  }
  const double t_min = sorted_times.front();
  const double t_max = sorted_times.back();
  const double span = t_max - t_min;
  // ColorMapper maps max <= min to the LUT start, so a zero span is safe.
  const double value_max = span;

  std::size_t drawn = 0;
  for (std::uint32_t k = 1; k <= steps; ++k) {
    const double threshold = t_min + span * static_cast<double>(k) / static_cast<double>(steps);
    const std::size_t count = static_cast<std::size_t>(
      std::upper_bound(sorted_times.begin(), sorted_times.end(), threshold) - sorted_times.begin());
    if (count > drawn) {
      const auto projected = core::pointcloud::project_scan_points(
        cloud, view, std::span<const std::uint32_t>(order.data() + drawn, count - drawn),
        times.times, t_min);
      if (!projected.ok()) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", projected.error.c_str());
        return 1;
      }
      if (const auto err = core::pointcloud::overlay_projected_points(
            canvas, projected.points, 0.0, value_max, scheme, point_size, 1.0f, canvas);
          !err.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
        return 1;
      }
      drawn = count;
    }
    if (const auto err =
          encoder.write_frame(canvas.bgr, canvas.width * 3, core::video::SourcePixelFormat::kBgr8);
        !err.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
      return 1;
    }
    ++written;
  }
  return 0;
}

}  // namespace

int run_generate_video_pcd_scan(const GenerateVideoPcdScanArgs & args)
{
  // Validate the source topic, its point layout, and the numeric options
  // before touching anything expensive.
  const auto validation = validate_pcd_scan_inputs(args);
  if (!validation.ok()) {
    return 1;
  }
  if (const auto err = validate_video_output_path(args.output_path, args.overwrite); !err.empty()) {
    return 1;
  }

  // Pass 1: derive the cloud rate from the topic's message timestamps.
  TopicSpan span;
  if (const auto err = scan_pcd_scan_span(args.input_path, args.topic, span); !err.empty()) {
    return 1;
  }
  const auto cloud_fps = core::video::derive_frame_rate(span.first_ns, span.last_ns, span.count);
  const auto scan_rate = derive_scan_frame_rate(cloud_fps, args.steps, args.speed);
  if (scan_rate.steps != args.steps) {
    BAGWIZ_LOG_WARN(
      kLogger, "--steps reduced from %u to %u to keep the output frame rate within %d fps.",
      args.steps, scan_rate.steps, core::video::kMaxFps);
  }
  if (scan_rate.fps.num == core::video::kMinFps && scan_rate.fps.den == 1) {
    const double wanted =
      static_cast<double>(cloud_fps.num) * scan_rate.steps * args.speed / cloud_fps.den;
    if (wanted < core::video::kMinFps) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "cloud rate x --steps x --speed is %.3g fps, below the %d fps floor; the output "
        "frame rate is clamped to %d fps and the animation plays faster than requested.",
        wanted, core::video::kMinFps, core::video::kMinFps);
    }
  }

  // Resolve the BEV view extent: --range defaults to the largest finite XY
  // distance in the first cloud. The perspective view does not use it.
  double range_m = args.range_m.value_or(0.0);
  if (args.view == core::pointcloud::ScanPatternProjection::kBev && !args.range_m.has_value()) {
    const auto auto_range = auto_range_from_cloud(validation.first_cloud);
    if (!auto_range.has_value() || *auto_range <= 0.0) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "could not determine --range from the first cloud of topic '%s' (no finite points); "
        "pass --range explicitly.",
        args.topic.c_str());
      return 1;
    }
    range_m = *auto_range;
  }
  core::pointcloud::ScanPatternView view;
  view.projection = args.view;
  view.width = args.width;
  view.height = args.height;
  view.range_m = range_m;
  view.elev_deg = args.elev_deg;
  view.azim_deg = args.azim_deg;
  view.dist_m = args.dist_m;

  // Pass 2: encode to a sibling temp path, renamed into place on success. The
  // guard removes the temp on any error exit, so no partial output and no
  // leftover temp survive a failure.
  const std::filesystem::path tmp_path = partial_tmp_path_for(args.output_path);
  PartialFileGuard guard(tmp_path);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
    return 1;
  }
  io::ReadFilter filter;
  filter.topics.push_back(args.topic);
  reader->set_filter(filter);

  auto opened = core::video::open_video_encoder(
    tmp_path, args.width, args.height, scan_rate.fps.num, scan_rate.fps.den);
  if (!opened.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
    return 1;
  }

  std::uint64_t written = 0;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "failed to parse a cloud of topic '%s': %s", args.topic.c_str(),
          parsed.error.c_str());
        return 1;
      }
      // The field layout is expected to be stable across the bag (validation
      // resolved it on the first cloud), but a mid-bag layout change must not
      // silently mis-order the animation.
      const auto time_field = core::pointcloud::find_point_time_field(*parsed.cloud);
      if (!time_field.has_value()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "topic '%s' lost its per-point time field mid-bag.", args.topic.c_str());
        return 1;
      }
      if (
        encode_sweep(
          *parsed.cloud, *time_field, view, scan_rate.steps, args.colorscheme, args.point_size,
          *opened.encoder, written) != 0) {
        return 1;
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", args.topic.c_str(), e.what());
    return 1;
  }

  if (written == 0) {
    // Pass 1 saw messages, so a frameless pass 2 means the bag changed
    // between passes.
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' yielded no frames in the encode pass.", args.topic.c_str());
    return 1;
  }
  if (const auto err = opened.encoder->finish(); !err.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", err.c_str());
    return 1;
  }
  if (const auto err = finalize_video_output(tmp_path, args.output_path, args.overwrite);
      !err.empty()) {
    return 1;
  }

  const double fps_value =
    static_cast<double>(scan_rate.fps.num) / static_cast<double>(scan_rate.fps.den);
  BAGWIZ_LOG_INFO(
    kLogger, "generate video scan: wrote %" PRIu64 " frame(s) to %s (%ux%u bgr8 @ %.3g fps).",
    written, args.output_path.string().c_str(), args.width, args.height, fps_value);
  return 0;
}

}  // namespace bagwiz::commands
