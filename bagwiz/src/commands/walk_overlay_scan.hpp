// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_OVERLAY_SCAN_HPP_
#define COMMANDS__WALK_OVERLAY_SCAN_HPP_

#include "bagwiz/core/pointcloud/fetcher.hpp"

#include <tf2/buffer_core.hpp>

#include <atomic>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

// One-pass bag scan behind `bagwiz walk`'s point-cloud overlay
// initialization. A single streaming pass collects everything the overlay
// needs up front — the TF buffer and the per-topic cloud timestamps — so the
// cost stays at one messages-table walk instead of one walk per input.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// Outcome of scan_overlay_inputs(). `error` is empty on success; on failure
// the other fields must be treated as invalid. When the scan was cancelled
// via `cancel`, the function returns early with `error` still empty — the
// canceller knows it cancelled and discards the result.
struct OverlayScanResult
{
  tf2::BufferCore tf_buffer;
  // Parallel to the `pcd_topics` argument: index entries of each selected
  // topic, in bag (record-time) order. stamp_ns is the cloud's own
  // header.stamp, read from the leading bytes of its payload, falling back to
  // record_ns when the source left the stamp unset.
  std::vector<std::vector<core::pointcloud::PointCloudIndexEntry>> entries;
  // Parallel to `entries`: true when *every* message of that topic carried a
  // header.stamp, so its stamp_ns axis is a pure capture-time clock (see
  // core::pointcloud::PointCloudIndex::header_stamps_present). False means the
  // axis mixes two clocks and the topic must be matched by record time.
  std::vector<bool> header_stamps_present;
  std::string error;
};

// Scan `input` once, decoding every TFMessage topic into `out.tf_buffer` and
// collecting the timestamps of each topic in `pcd_topics` into `out.entries`.
// Only the leading std_msgs/Header stamp of each cloud is parsed — the point
// data is never decoded — so the pass costs one messages-table walk plus the
// payload read of the selected cloud topics.
//
// `cancel` is polled per message; when set, the scan aborts early.
// `progress` (may be empty) is invoked with a 0..1 fraction derived from the
// bag's time extent, throttled to whole-percent changes. Fails (via
// `out.error`) when the bag holds no TFMessage topic, a TF message fails to
// decode, or a selected topic has no messages.
//
// Threading: safe to run on a worker thread — it opens its own reader and
// touches only its arguments.
void scan_overlay_inputs(
  const std::filesystem::path & input, const std::vector<std::string> & pcd_topics,
  const std::atomic<bool> & cancel, const std::function<void(double)> & progress,
  OverlayScanResult & out);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_OVERLAY_SCAN_HPP_
