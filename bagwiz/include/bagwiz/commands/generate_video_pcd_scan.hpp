// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__GENERATE_VIDEO_PCD_SCAN_HPP_
#define BAGWIZ__COMMANDS__GENERATE_VIDEO_PCD_SCAN_HPP_

#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/scan_pattern.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

namespace bagwiz::commands
{

// Arguments for `bagwiz generate video scan`. Populated by GenerateCommand's
// CLI wiring (src/commands/generate.cpp) and consumed by
// run_generate_video_pcd_scan. Kept in a header so the run function can be
// exercised directly from tests without driving the CLI parser.
struct GenerateVideoPcdScanArgs
{
  GenerateVideoPcdScanArgs() = default;
  GenerateVideoPcdScanArgs(
    std::filesystem::path input_path_arg, std::string topic_arg,
    std::filesystem::path output_path_arg, bool overwrite_arg)
  : input_path(std::move(input_path_arg)),
    topic(std::move(topic_arg)),
    output_path(std::move(output_path_arg)),
    overwrite(overwrite_arg)
  {
  }

  std::filesystem::path input_path;
  std::string topic;
  std::filesystem::path output_path;
  // Replace a pre-existing <output>. Without it, an existing output path stops
  // the run before any work is done.
  bool overwrite = false;

  // Projection space of the animation.
  core::pointcloud::ScanPatternProjection view = core::pointcloud::ScanPatternProjection::kBev;
  // Canvas size in pixels. Both must be even (H.264 requires even dimensions).
  std::uint32_t width = 1280;
  std::uint32_t height = 720;
  // Video frames rendered per sweep. The output frame rate is the cloud rate
  // times this, so the animation plays in real time.
  std::uint32_t steps = 10;
  // BEV half-extent in meters. In the 3D view it is not read directly; only
  // the default dist_m (2.5x the range) derives from it. Unset = auto: the
  // largest finite XY distance in the first cloud.
  std::optional<double> range_m;
  // Perspective camera (view = kPerspective only): position on a sphere of
  // radius dist_m around the sensor, looking at it. dist_m unset = 2.5x range.
  double elev_deg = 30.0;
  double azim_deg = 180.0;
  std::optional<double> dist_m;

  core::pointcloud::ColorScheme colorscheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 2;
};

// Render the scan pattern of `args.topic` (sensor_msgs/msg/PointCloud2) from
// `args.input_path` to a video at `args.output_path`: within each sweep the
// points appear in firing order — read from the cloud's per-point time field,
// which is required — colored by their sweep-relative time. The output frame
// rate is the cloud rate times `args.steps`, so the animation plays in real
// time. Returns a process exit code: 0 on success, 1 on any error.
int run_generate_video_pcd_scan(const GenerateVideoPcdScanArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__GENERATE_VIDEO_PCD_SCAN_HPP_
