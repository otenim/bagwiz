// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PCD_COMPRESS_HPP_
#define BAGWIZ__COMMANDS__PCD_COMPRESS_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz pcd compress`. Replaces each selected
// sensor_msgs/msg/PointCloud2 topic with a
// point_cloud_interfaces/msg/CompressedPointCloud2 topic holding a
// Draco-encoded point payload (format "draco").
struct PcdCompressArgs
{
  std::filesystem::path input_path;  // -i,--input
  // -t,--topics; PointCloud2 topics to compress (literal names or '*' globs,
  // already expanded before run()). Empty => every PointCloud2 topic in the bag.
  std::vector<std::string> topics;
  // --as; overrides the output topic name. Only valid with exactly one
  // selected topic. Empty => "<input topic>/draco".
  std::string output_topic;
  std::optional<std::filesystem::path> output_path;  // -o; empty => in-place
  // -f,--force; replaces a pre-existing output topic in the input bag.
  bool force = false;
  // -w,--overwrite; overwrites an existing -o path.
  bool overwrite = false;
  std::optional<int> threads;  // -j,--threads; omit/0 => hardware concurrency, 1 => sync
  // --lossless; no quantization, floats round-trip bit-exact. Default: lossy
  // (14-bit quantization per attribute type).
  bool lossless = false;
  // --position-bits / --normal-bits / --color-bits / --generic-bits; per-type
  // quantization bit counts (1..31) overriding the 14-bit default. Rejected
  // together with --lossless. tex_coord keeps the default (no flag).
  std::optional<int> position_bits;
  std::optional<int> normal_bits;
  std::optional<int> color_bits;
  std::optional<int> generic_bits;
};

// Execute `bagwiz pcd compress`. Returns a process exit code.
int run_pcd_compress(const PcdCompressArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PCD_COMPRESS_HPP_
