// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__STAMP_SYNC_HPP_
#define BAGWIZ__COMMANDS__STAMP_SYNC_HPP_

#include <filesystem>
#include <optional>

namespace bagwiz::commands
{

// Arguments for `bagwiz stamp sync`. Populated by StampCommand's CLI wiring
// (src/commands/stamp.cpp) and consumed by run_stamp_sync. Kept in a header so
// the run function can be exercised directly from tests without driving the
// CLI parser.
struct StampSyncArgs
{
  std::filesystem::path input_path;
  // Empty: rewrite <input> in place. Set: write the result to this new bag and
  // leave <input> untouched.
  std::optional<std::filesystem::path> output_path;
  // Replace a pre-existing output_path (no effect in in-place mode).
  bool overwrite = false;
};

// Overwrite each message's header.stamp with its receive (log) time, on every
// topic whose type leads with a std_msgs/Header (the ROS 2 stamped-message
// convention — the same classification `trim --stamp header` uses). Messages
// on other topics are copied verbatim. Returns a process exit code: 0 on
// success, 1 on any error (input open failure, no headered topic in the bag,
// output collision, or a read/write/close error).
int run_stamp_sync(const StampSyncArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__STAMP_SYNC_HPP_
