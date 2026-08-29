// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_cloud_source.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

CloudSources::Source::Source(
  const std::filesystem::path & input, std::string topic,
  std::vector<core::pointcloud::PointCloudIndexEntry> entries)
: fetcher(input, std::move(topic), std::move(entries))
{
}

CloudSources::CloudSources(
  const std::filesystem::path & input, VideoInputScan & scan, tf2::BufferCore * tf_buffer)
: has_stamps_(scan.pcd_topic_has_stamps), tf_buffer_(tf_buffer)
{
  sources_.reserve(scan.pcd_topics.size());
  for (std::size_t i = 0; i < scan.pcd_topics.size(); ++i) {
    sources_.push_back(
      std::make_unique<Source>(input, scan.pcd_topics[i], std::move(scan.pcd_spans[i].entries)));
  }
}

std::shared_ptr<const core::pointcloud::PointCloud2> CloudSources::fetch(
  std::size_t index, std::int64_t target_ns, core::pointcloud::PointCloudMatchKey key,
  std::string & error)
{
  Source & source = *sources_[index];
  const std::lock_guard<std::mutex> lock(source.mutex);
  return source.fetcher.fetch_shared(target_ns, key, error);
}

}  // namespace bagwiz::commands
