// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__MAP_FILTER_HPP_
#define BAGWIZ__COMMANDS__MAP_FILTER_HPP_

#include <filesystem>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz map filter removert`. Populated by MapCommand's CLI
// wiring (src/commands/map.cpp) and consumed by run_map_filter_removert. Kept in
// a header so the run function can be exercised directly from tests without
// driving the CLI parser.
struct MapFilterRemovertArgs
{
  // Map to filter: a .pcd file or a directory containing map.pcd.
  std::filesystem::path map_path;
  // Source bag that carries the original PointCloud2 scans.
  std::filesystem::path input_path;
  // PointCloud2 topic whose scans will be reprojected into the world frame.
  std::string cloud_topic;
  // TUM trajectory produced by `bagwiz map slam` (one pose per scan).
  std::filesystem::path traj_path;
  // Output map path (a .pcd file) or directory (receives map.pcd).
  std::filesystem::path output_path;

  // Removert-style filter tunables; see bagwiz/core/slam/cloud_filters.hpp.
  bool enable_revert = true;
  double vertical_fov_deg = 50.0;
  double horizontal_fov_deg = 360.0;
  std::vector<double> remove_resolutions = {2.0};
  std::vector<double> revert_resolutions = {1.0};
  double adaptive_coeff = 0.05;
  double valid_diff_upper_bound = 200.0;

  // Overwrite the output file(s) if they already exist.
  bool overwrite = false;
  // Disable the live progress bar.
  bool no_progress = false;
};

// Apply an original Removert-style dynamic-point filter to an existing map.pcd.
// The optimized trajectory is used to project each raw scan from <input_path>'s
// <cloud_topic> into the world frame; the merged map points are then classified
// as static/dynamic against those scan views. The filtered map is written to
// <output_path>.
//
// Returns a process exit code: 0 on success, 1 on any error (input open failure,
// missing topic/type mismatch, missing trajectory, bad PCD, or a read/write
// error).
int run_map_filter_removert(const MapFilterRemovertArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__MAP_FILTER_HPP_
