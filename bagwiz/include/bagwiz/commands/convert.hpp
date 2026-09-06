// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__CONVERT_HPP_
#define BAGWIZ__COMMANDS__CONVERT_HPP_

#include <filesystem>
#include <string>

namespace bagwiz::commands
{

// Arguments for `bagwiz convert format`. Populated by ConvertCommand's CLI
// wiring (src/commands/convert.cpp) and consumed by run_convert_format. Kept
// in a header so the run function can be exercised directly from tests
// without driving the CLI parser.
struct ConvertFormatArgs
{
  std::filesystem::path input_path;
  // The new bag: a `.mcap` / `.db3` path is a single file, anything else a
  // directory. Always required — `convert format` has no in-place mode, a
  // repack that changes nothing being a plain `cp`.
  std::filesystem::path output_path;
  // Target storage backend: "" (the output extension, then the input's
  // detected storage), "mcap", or "sqlite3".
  std::string storage;
  // Replace any pre-existing output_path.
  bool overwrite = false;
};

// Repack the input bag into output_path, converting between storage backends
// and/or file/directory layouts while carrying the input's compression over.
// Returns a process exit code: 0 on success, 1 on any error (unresolvable
// storage, a same-storage same-layout repack, output collision, input open
// failure, or a declare/read/write/close error).
int run_convert_format(const ConvertFormatArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__CONVERT_HPP_
