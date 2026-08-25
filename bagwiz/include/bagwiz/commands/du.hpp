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
  // Print sizes in human-readable units (1024-based, e.g. "4.0K", "1.2M")
  // instead of raw byte counts.
  bool human = false;
};

// Report each topic's total serialized payload size in `args.input_path`,
// one row per topic sorted by size descending, followed by a `total` row.
// The size is the sum of the uncompressed serialized payload bytes (the
// logical message size), not the on-disk footprint, which per-topic chunk
// compression makes unrecoverable. Requires a full scan of the bag's
// messages. Returns a process exit code: 0 on success, 1 on any error
// (input open failure, a selector naming no topic, or a read error).
int run_du(const DuArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__DU_HPP_
