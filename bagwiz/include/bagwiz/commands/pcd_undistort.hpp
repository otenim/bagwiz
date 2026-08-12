// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_
#define BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz pcd undistort`. Deskews one or more PointCloud2
// topics using a pose or twist topic as the motion source, resolved through the
// bag's static TF (no dynamic /tf; SLAM-free).
struct PcdUndistortArgs
{
  std::filesystem::path input_path;      // -i,--input
  std::string pose_topic;                // --pose (motion source)
  std::string twist_topic;               // --twist (motion source; exactly one of --pose/--twist)
  std::vector<std::string> pcd_topics;   // --pcd (>=1)
  std::optional<std::string> ref_frame;  // --ref; empty => "map"
  std::optional<std::string> of_frame;   // --of;  empty => "base_link"
  std::optional<std::filesystem::path> output_path;  // -o; empty => in-place
  bool overwrite = false;                            // -w
  std::optional<int> threads;  // -j,--threads; omit/0 => hardware concurrency, 1 => sync
  bool no_extrap = false;      // --no-extrap; disable trajectory extrapolation
  // --max-extrap-duration; per-side cap on the trajectory extrapolation,
  // parsed with core::parse_duration_ns (no unit = ms). Empty => 1s.
  std::optional<std::string> max_extrap_duration;
  // --compression; mcap chunk codec for the output: "zstd", "lz4", or
  // "none". Empty => the storage default (zstd). mcap outputs only.
  std::optional<std::string> compression;
  // --compression-level; encoder effort for the chosen (or default) codec:
  // "fastest", "fast", "default", "slow", or "slowest". Empty => "default".
  // Rejected together with --compression none.
  std::optional<std::string> compression_level;
};

// Execute `bagwiz pcd undistort`. Returns a process exit code.
int run_pcd_undistort(const PcdUndistortArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_
