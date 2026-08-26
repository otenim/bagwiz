// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__DU_HPP_
#define BAGWIZ__COMMANDS__DU_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz du`. Populated by DuCommand's CLI wiring
// (src/commands/du.cpp) and consumed by run_du. Kept in a header so the run
// function can be exercised directly from tests without driving the CLI
// parser.
struct DuArgs
{
  std::filesystem::path input_path;
  // Topic selectors to report on, already expanded to literal topic names by
  // the CLI's topic-slot pass (see commands/topic_option.hpp). Empty: report
  // every topic in the bag.
  std::vector<std::string> topics;
  // Print sizes as raw byte counts instead of the default human-readable
  // units (1024-based, e.g. "4.0K", "1.2M").
  bool bytes = false;
  // Aggregate topics by their first N name components, du(1) --max-depth
  // style: depth 1 groups "/sensing/lidar" under "/sensing". A topic already
  // at or above the depth keeps its full name; 0 prints only the total row.
  // Unset: one row per topic.
  std::optional<int> depth;
};

// Report each topic's total serialized payload size in `args.input_path`,
// one row per topic sorted by size descending, followed by a `total` row.
// With `args.depth` set, rows are instead the per-depth-group aggregates of
// the topics' first N name components (0: only the total row). The size is
// the sum of the uncompressed serialized payload bytes (the logical message
// size), not the on-disk footprint, which per-topic chunk compression makes
// unrecoverable. Requires a full scan of the bag's messages. Returns a
// process exit code: 0 on success, 1 on any error (input open failure, a
// selector naming no topic, or a read error).
int run_du(const DuArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__DU_HPP_
