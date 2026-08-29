// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_CLOUD_SOURCE_HPP_
#define COMMANDS__MOVIFY_CLOUD_SOURCE_HPP_

#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "movify_inputs.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// The point-cloud topics `movify`'s panels draw, shared across panels and
// ticks: one fetcher per unique topic behind a mutex (PointCloudFetcher is
// not thread-safe, and the parallel loop's panel jobs fetch concurrently), so
// each distinct cloud is loaded from the bag at most once per run — the
// fetcher's cache covers both cross-panel and cross-tick reuse. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

class CloudSources
{
public:
  // Take the pass-1 index entries out of `scan` (they are moved) and open one
  // fetcher per topic, in `scan.pcd_topics` order. `tf_buffer` is the bag's
  // TF the projections look up (null when no panel projects).
  CloudSources(
    const std::filesystem::path & input, VideoInputScan & scan, tf2::BufferCore * tf_buffer);

  [[nodiscard]] std::size_t size() const noexcept { return sources_.size(); }

  // Whether topic `index` can be matched by capture time (every cloud carried
  // a header.stamp — see PointCloudIndex::header_stamps_present).
  [[nodiscard]] bool has_header_stamps(std::size_t index) const { return has_stamps_[index]; }

  [[nodiscard]] tf2::BufferCore * tf_buffer() const noexcept { return tf_buffer_; }

  // The cloud of topic `index` whose `key` clock is nearest `target_ns`
  // (target_ns expressed in that same clock), shared so concurrent panels can
  // hold clouds simultaneously. Null with `error` set on a load failure.
  [[nodiscard]] std::shared_ptr<const core::pointcloud::PointCloud2> fetch(
    std::size_t index, std::int64_t target_ns, core::pointcloud::PointCloudMatchKey key,
    std::string & error);

private:
  // One topic's fetcher plus the mutex serializing it.
  struct Source
  {
    Source(
      const std::filesystem::path & input, std::string topic,
      std::vector<core::pointcloud::PointCloudIndexEntry> entries);

    core::pointcloud::PointCloudFetcher fetcher;
    std::mutex mutex;
  };

  std::vector<std::unique_ptr<Source>> sources_;
  std::vector<bool> has_stamps_;
  tf2::BufferCore * tf_buffer_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_CLOUD_SOURCE_HPP_
