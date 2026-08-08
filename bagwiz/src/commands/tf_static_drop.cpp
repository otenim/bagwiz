// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_drop.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.drop";

// True when any topic carries an edge whose child is `frame`.
bool is_child_frame(
  const std::vector<core::StaticTopicTransforms> & topics, const std::string & frame)
{
  for (const auto & st : topics) {
    for (const auto & t : st.transforms) {
      if (t.child_frame_id == frame) {
        return true;
      }
    }
  }
  return false;
}

// True when any topic carries an edge whose parent is `frame`.
bool is_parent_frame(
  const std::vector<core::StaticTopicTransforms> & topics, const std::string & frame)
{
  for (const auto & st : topics) {
    for (const auto & t : st.transforms) {
      if (t.header.frame_id == frame) {
        return true;
      }
    }
  }
  return false;
}

// Every child frame in the merged tree, sorted, for error messages.
std::vector<std::string> all_child_frames(const std::vector<core::StaticTopicTransforms> & topics)
{
  std::vector<std::string> children;
  for (const auto & st : topics) {
    for (const auto & t : st.transforms) {
      children.push_back(t.child_frame_id);
    }
  }
  std::sort(children.begin(), children.end());
  children.erase(std::unique(children.begin(), children.end()), children.end());
  return children;
}

// The frames of the subtrees rooted at `roots`, roots included, over the
// merged parent -> child adjacency. Computed against the tree as loaded,
// before any edit is applied, so several --frame roots where one is another's
// descendant resolve against the same snapshot.
std::unordered_set<std::string> subtree_frames(
  const std::vector<core::StaticTopicTransforms> & topics, const std::vector<std::string> & roots)
{
  std::unordered_set<std::string> doomed;
  std::vector<std::string> stack = roots;
  while (!stack.empty()) {
    const std::string cur = stack.back();
    stack.pop_back();
    if (!doomed.insert(cur).second) {
      continue;
    }
    for (const auto & st : topics) {
      for (const auto & t : st.transforms) {
        if (t.header.frame_id == cur) {
          stack.push_back(t.child_frame_id);
        }
      }
    }
  }
  return doomed;
}

// Remove every transform whose child is in `doomed` from each topic, logging
// each removed edge. Topics that lost at least one edge are added to `touched`.
// Returns the number of edges removed.
std::uint64_t remove_doomed_edges(
  std::vector<core::StaticTopicTransforms> & topics, const std::unordered_set<std::string> & doomed,
  std::unordered_set<std::string> & touched, const char * logger)
{
  std::uint64_t removed = 0;
  for (auto & st : topics) {
    std::vector<geometry_msgs::msg::TransformStamped> kept;
    kept.reserve(st.transforms.size());
    for (const auto & t : st.transforms) {
      if (doomed.count(t.child_frame_id) == 0) {
        kept.push_back(t);
        continue;
      }
      BAGWIZ_LOG_WARN(
        logger, "Dropping edge '%s' -> '%s' on topic '%s'.", t.header.frame_id.c_str(),
        t.child_frame_id.c_str(), st.name.c_str());
      ++removed;
    }
    if (kept.size() != st.transforms.size()) {
      st.transforms = std::move(kept);
      touched.insert(st.name);
    }
  }
  return removed;
}

}  // namespace

int run_tf_static_drop(
  const std::filesystem::path & input_path, const std::vector<std::string> & frames,
  const std::optional<std::filesystem::path> & output_path, bool overwrite)
{
  if (frames.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "Nothing to do: pass at least one --frame <frame> to drop.");
    return 1;
  }
  for (const auto & frame : frames) {
    if (frame.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Every --frame argument must be a non-empty frame id.");
      return 1;
    }
  }

  // Load the merged static tree, keeping the per-topic split so each removal
  // lands in the topic that carries the edge. The whole topic is read: a later
  // message may carry an edge the first one does not.
  std::vector<core::StaticTopicTransforms> topics;
  try {
    topics = core::collect_static_tf(input_path, core::StaticTfRead::kWholeTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to load static TF from %s: %s", input_path.c_str(), e.what());
    return 1;
  }

  // Every --frame is validated against the tree as loaded, then all removals are
  // applied together, so a frame inside another's subtree is not reported
  // missing just because its ancestor was listed first.
  std::vector<std::string> roots;
  {
    std::unordered_set<std::string> seen;
    for (const auto & frame : frames) {
      if (seen.insert(frame).second) {
        roots.push_back(frame);
      }
    }
  }
  for (const auto & frame : roots) {
    if (is_child_frame(topics, frame)) {
      continue;
    }
    if (is_parent_frame(topics, frame)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "--frame names a child frame (the edge above it is removed), but '%s' is a root of the "
        "static tree: it parents edges and has no parent itself.",
        frame.c_str());
    } else {
      BAGWIZ_LOG_ERROR(
        kLogger, "Frame '%s' is not present in the bag's static TF tree.", frame.c_str());
    }
    BAGWIZ_LOG_ERROR(
      kLogger, "Available static child frames: %s",
      core::join_csv(all_child_frames(topics)).c_str());
    return 1;
  }

  std::unordered_set<std::string> touched;
  const auto doomed = subtree_frames(topics, roots);
  const std::uint64_t pruned = remove_doomed_edges(topics, doomed, touched, kLogger);

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "tf static drop: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "tf static drop: pass failed; aborting in-place swap";

  StaticTfInjectOptions inject_opts;
  inject_opts.logger = kLogger;
  inject_opts.label = "tf static drop";
  inject_opts.profile_label = "tf_static_drop";

  const int rc = rewrite_touched_static_topics(
    input_path, topics, touched, output_path, overwrite, rewrite_opts, inject_opts);
  if (rc == 0 && !touched.empty()) {
    BAGWIZ_LOG_INFO(
      kLogger, "tf static drop: dropped %" PRIu64 " edge(s) across %zu topic(s).", pruned,
      touched.size());
  }
  return rc;
}

}  // namespace bagwiz::commands
