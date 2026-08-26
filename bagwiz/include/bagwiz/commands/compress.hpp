// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__COMPRESS_HPP_
#define BAGWIZ__COMMANDS__COMPRESS_HPP_

#include <filesystem>
#include <string>

namespace bagwiz::commands
{

// Arguments for `bagwiz compress`. Populated by CompressCommand's CLI wiring
// (src/commands/compress.cpp) and consumed by run_compress. Kept in a header
// so the run function can be exercised directly from tests without driving
// the CLI parser.
struct CompressArgs
{
  std::filesystem::path input_path;
  std::filesystem::path output_path;
  // Compression mode: "auto" (per-storage default), "file", "message", or
  // "none" (decompress). For MCAP outputs, "file" is chunk compression and
  // "message" is rejected (rosbag2 defines no per-message mode for MCAP).
  // For SQLite3 outputs, "message" is per-message zstd and "file" is the
  // whole-shard .db3.zstd envelope; both require a directory-layout output.
  std::string mode = "auto";
  // Compression codec: "zstd" or "lz4". lz4 is valid only for MCAP chunk
  // compression (rosbag2 defines zstd alone for SQLite3 storage).
  std::string codec = "zstd";
  // Encoder effort: "", "fastest", "fast", "default", "slow", "slowest".
  // Empty selects the codec-appropriate default.
  std::string level;
  // Target storage backend: "" (resolve like `convert format`: output
  // extension, then the input's detected storage), "mcap", or "sqlite3".
  std::string storage;
  // Replace any pre-existing output_path.
  bool overwrite = false;
};

// Re-encode the input bag with the requested compression (or decompress it
// with mode "none") into a new bag at output_path. Returns a process exit
// code: 0 on success, 1 on any error (unresolvable storage, an invalid
// mode/codec/level combination, input open failure, output collision, or a
// declare/read/write/close error).
int run_compress(const CompressArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__COMPRESS_HPP_
