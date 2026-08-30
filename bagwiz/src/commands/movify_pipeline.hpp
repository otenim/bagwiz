// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_PIPELINE_HPP_
#define COMMANDS__MOVIFY_PIPELINE_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_output.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"   // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// The encode pass of `movify`: the loop that turns each clock message into
// one output frame by driving every panel through select() and render() and
// handing the composed canvas to the encoder — once synchronously, once with
// the panels' work spread across worker threads. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// The parallel per-panel pipeline is only worthwhile when there is work to
// spread across workers (several panels, or point-cloud projection) and enough
// frames to hide the per-tick job-launch overhead.
[[nodiscard]] bool should_use_parallel_pipeline(
  std::size_t panel_count, bool has_pointcloud_topics, bool enable_parallel,
  std::uint64_t frame_count, unsigned int hardware_concurrency);

// Run the encode pass: `reader` yields the clock topic's messages, one per
// output tick; panels[0] is the clock panel and the rest fill the grid in
// order. Per tick every panel selects its input and renders its cell, then
// the composed grid is encoded. The first tick's clock selection fixes the
// cell size (and reports an oversized output). `parallel` picks the
// worker-thread loop, which overlaps a tick's panel jobs with the previous
// tick's encode, over the synchronous one; both compose identical frames.
// `clock_topic` names the clock in log lines. Returns a process exit code;
// errors are logged inside.
[[nodiscard]] int run_encode_pass(
  io::BagReader & reader, std::vector<std::unique_ptr<Panel>> & panels, GridSpec grid,
  bool parallel, const std::string & clock_topic, VideoFrameEncoder & encoder);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_PIPELINE_HPP_
