// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TRAJ_COMMON_HPP_
#define COMMANDS__TRAJ_COMMON_HPP_

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <memory>
#include <span>
#include <string>

// Shared internals of `traj dump`, split out of traj.cpp so the decoder-open
// and TUM-write steps can be unit-tested without driving the full command.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// Open the decoder for `topic` on the already-open `reader`. On failure the
// command's usual error is logged to `logger` (with the decoder factory's
// reason when the topic exists but no backend could open it) and nullptr is
// returned.
[[nodiscard]] std::unique_ptr<core::decoder::Decoder> open_topic_decoder(
  io::BagReader & reader, const std::string & topic, const char * logger);

// Write `poses` to `output_path` in the TUM trajectory format. The output path
// is claimed here (core::prepare_output_path) rather than by the caller, so a
// run that fails before it has poses to write cannot delete a pre-existing file
// under `overwrite`. On a collision or an open failure the error is logged to
// `logger` and false is returned.
[[nodiscard]] bool write_tum_file(
  const std::filesystem::path & output_path, std::span<const core::TrajectoryPose> poses,
  bool overwrite, const char * logger);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TRAJ_COMMON_HPP_
