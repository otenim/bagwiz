// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__INFO_HPP_
#define BAGWIZ__COMMANDS__INFO_HPP_

#include <filesystem>

namespace bagwiz::commands
{

// Arguments for `bagwiz info`. Populated by InfoCommand's CLI wiring
// (src/commands/info.cpp) and consumed by run_info. Kept in a header so the
// run function can be exercised directly from tests without driving the CLI
// parser.
struct InfoArgs
{
  std::filesystem::path input_path;
  // Append one row per storage file (shard) after the summary block: its
  // size, message count, start time and duration.
  bool long_listing = false;
  // Print sizes as raw byte counts instead of the default human-readable
  // units (1024-based, e.g. "4.0K", "1.2M").
  bool bytes = false;
};

// Print the bag-level metadata of `args.input_path` as one `Key: value` line
// per field: layout, storage backend, compression, file count and on-disk
// size, metadata.yaml presence, topic count, message count, start, end and
// duration. Every value comes from the bag's own summaries (metadata.yaml,
// the MCAP summary, a .db3's `metadata` row) and the file system; a bag that
// does not summarise a field reports it as unknown rather than scanning for
// it. Returns a process exit code: 0 on success, 1 when the bag cannot be
// described (missing path, unparseable metadata.yaml).
int run_info(const InfoArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__INFO_HPP_
