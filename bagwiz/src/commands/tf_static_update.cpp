// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_update.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iterator>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.update";

struct UpdateCounts
{
  std::uint64_t added = 0;
  std::uint64_t updated = 0;
  std::uint64_t reparented = 0;
};

}  // namespace

int run_tf_static_update(
  const std::filesystem::path & input_path, const std::filesystem::path & yaml_path,
  const std::string & topic, const std::optional<std::filesystem::path> & output_path,
  bool overwrite)
{
  const auto parsed = core::parse_static_tf_tree_yaml(yaml_path);
  if (!parsed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load static TF from '%s': %s", yaml_path.c_str(), parsed.error.c_str());
    return 1;
  }
  // Same note as `tf static join`: a deeper nesting level is a grouping heading,
  // not a chain link, so name the keys that parented nothing.
  if (!parsed.grouping_frames.empty()) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "'%s': %s named a grouping level, not a parent frame, so no transform was created for it. "
      "Only the level directly above a transform is its parent.",
      yaml_path.c_str(), core::join_csv(parsed.grouping_frames).c_str());
  }
  return run_tf_static_update(input_path, *parsed.transforms, topic, output_path, overwrite);
}

int run_tf_static_update(
  const std::filesystem::path & input_path,
  const std::vector<geometry_msgs::msg::TransformStamped> & transforms, const std::string & topic,
  const std::optional<std::filesystem::path> & output_path, bool overwrite)
{
  // Same guard as `tf static join`: CLI11 accepts an explicit empty string for
  // an option with a default, which would declare an unnameable topic.
  if (topic.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "-t/--topic must be a non-empty topic name.");
    return 1;
  }
  if (!core::is_static_tf_topic(topic)) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "Topic '%s' does not have 'tf_static' as its final path segment, so bagwiz's static-TF "
      "readers will treat it as a dynamic TF topic and skip it.",
      topic.c_str());
  }

  // Load the merged static tree, keeping the per-topic split so updates land in
  // the topic that carries each edge. The whole topic is read: a later message
  // may carry an edge the first one does not.
  std::vector<core::StaticTopicTransforms> topics;
  try {
    topics = core::collect_static_tf(input_path, core::StaticTfRead::kWholeTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to load static TF from %s: %s", input_path.c_str(), e.what());
    return 1;
  }

  UpdateCounts counts;
  std::unordered_set<std::string> touched;
  for (const auto & edge : transforms) {
    bool found = false;
    for (auto & st : topics) {
      for (auto & t : st.transforms) {
        if (t.child_frame_id != edge.child_frame_id) {
          continue;
        }
        if (t.header.frame_id != edge.header.frame_id) {
          BAGWIZ_LOG_WARN(
            kLogger, "Re-parenting '%s': '%s' -> '%s' on topic '%s'.", edge.child_frame_id.c_str(),
            t.header.frame_id.c_str(), edge.header.frame_id.c_str(), st.name.c_str());
          ++counts.reparented;
        }
        t.header.frame_id = edge.header.frame_id;
        t.transform = edge.transform;
        touched.insert(st.name);
        found = true;
      }
    }
    if (found) {
      ++counts.updated;
      continue;
    }
    // New child: it joins `topic`, which is declared by the pass when the bag
    // does not have it yet.
    auto it =
      std::find_if(topics.begin(), topics.end(), [&](const auto & st) { return st.name == topic; });
    if (it == topics.end()) {
      topics.push_back({topic, {}});
      it = std::prev(topics.end());
    }
    it->transforms.push_back(edge);
    touched.insert(topic);
    ++counts.added;
  }

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "tf static update: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "tf static update: pass failed; aborting in-place swap";

  StaticTfInjectOptions inject_opts;
  inject_opts.logger = kLogger;
  inject_opts.label = "tf static update";
  inject_opts.profile_label = "tf_static_update";

  const int rc = rewrite_touched_static_topics(
    input_path, topics, touched, output_path, overwrite, rewrite_opts, inject_opts);
  if (rc == 0 && !touched.empty()) {
    BAGWIZ_LOG_INFO(
      kLogger,
      "tf static update: added %" PRIu64 ", updated %" PRIu64 " (re-parented %" PRIu64
      ") across %zu topic(s).",
      counts.added, counts.updated, counts.reparented, touched.size());
  }
  return rc;
}

}  // namespace bagwiz::commands
