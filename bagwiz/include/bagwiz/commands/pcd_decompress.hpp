// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PCD_DECOMPRESS_HPP_
#define BAGWIZ__COMMANDS__PCD_DECOMPRESS_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz pcd decompress`. Replaces each selected
// point_cloud_interfaces/msg/CompressedPointCloud2 topic carrying
// format "draco" with the reconstructed sensor_msgs/msg/PointCloud2 topic.
struct PcdDecompressArgs
{
  std::filesystem::path input_path;  // -i,--input
  // -t,--topics; CompressedPointCloud2 topics to decompress (literal names or
  // '*' globs, already expanded before run()). Empty => every
  // CompressedPointCloud2 topic in the bag.
  std::vector<std::string> topics;
  // --as; overrides the output topic name. Only valid with exactly one
  // selected topic. Empty => the input topic name with a trailing "/draco"
  // stripped (an input topic without that suffix is an error).
  std::string output_topic;
  std::optional<std::filesystem::path> output_path;  // -o; empty => in-place
  // -f,--force; replaces a pre-existing output topic in the input bag.
  bool force = false;
  // -w,--overwrite; overwrites an existing -o path.
  bool overwrite = false;
  std::optional<int> threads;  // -j,--threads; omit/0 => hardware concurrency, 1 => sync
};

// Execute `bagwiz pcd decompress`. Returns a process exit code.
int run_pcd_decompress(const PcdDecompressArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PCD_DECOMPRESS_HPP_
