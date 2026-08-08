// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_edit.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/tf/tf_forest_check.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.edit";

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
// before any edit is applied, so several --prune frames where one is another's
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
        logger, "Pruning edge '%s' -> '%s' on topic '%s'.", t.header.frame_id.c_str(),
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

struct EditCounts
{
  std::uint64_t pruned = 0;
  std::uint64_t added = 0;
  std::uint64_t updated = 0;
  std::uint64_t reparented = 0;
};

}  // namespace

int run_tf_static_edit(
  const std::filesystem::path & input_path, const std::optional<std::filesystem::path> & yaml_path,
  const std::vector<std::string> & prune_frames, const std::string & topic,
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
      "Topic '%s' does not end with 'tf_static', so bagwiz's static-TF readers will treat it as a "
      "dynamic TF topic and skip it.",
      topic.c_str());
  }
  if (!yaml_path.has_value() && prune_frames.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "Nothing to do: pass --yaml and/or --prune.");
    return 1;
  }
  for (const auto & frame : prune_frames) {
    if (frame.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Every --prune argument must be a non-empty frame id.");
      return 1;
    }
  }

  // 1. Load the merged static tree, keeping the per-topic split so edits land
  //    in the topic that carries each edge. The whole topic is read: a later
  //    message may carry an edge the first one does not.
  std::vector<core::StaticTopicTransforms> topics;
  try {
    topics = core::collect_static_tf(input_path, core::StaticTfRead::kWholeTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to load static TF from %s: %s", input_path.c_str(), e.what());
    return 1;
  }

  EditCounts counts;
  std::unordered_set<std::string> touched;

  // 2. Prune phase. Every --prune frame is validated against the tree as
  //    loaded, then all removals are applied together, so a prune frame inside
  //    another's subtree is not reported missing just because its ancestor was
  //    listed first.
  if (!prune_frames.empty()) {
    std::vector<std::string> roots;
    {
      std::unordered_set<std::string> seen;
      for (const auto & frame : prune_frames) {
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
          "--prune names a child frame (the edge above it is removed), but '%s' is a root of the "
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
    const auto doomed = subtree_frames(topics, roots);
    counts.pruned = remove_doomed_edges(topics, doomed, touched, kLogger);
  }

  // 3. Add/update phase, applied after pruning: pruning a frame and re-adding
  //    it via --yaml in one run replaces that subtree with the config's.
  if (yaml_path.has_value()) {
    const auto parsed = core::parse_static_tf_tree_yaml(*yaml_path);
    if (!parsed.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Could not load static TF from '%s': %s", yaml_path->c_str(),
        parsed.error.c_str());
      return 1;
    }
    // Same note as `tf static join`: a deeper nesting level is a grouping
    // heading, not a chain link, so name the keys that parented nothing.
    if (!parsed.grouping_frames.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "'%s': %s named a grouping level, not a parent frame, so no transform was created for it. "
        "Only the level directly above a transform is its parent.",
        yaml_path->c_str(), core::join_csv(parsed.grouping_frames).c_str());
    }
    for (const auto & edge : *parsed.transforms) {
      bool found = false;
      for (auto & st : topics) {
        for (auto & t : st.transforms) {
          if (t.child_frame_id != edge.child_frame_id) {
            continue;
          }
          if (t.header.frame_id != edge.header.frame_id) {
            BAGWIZ_LOG_WARN(
              kLogger, "Re-parenting '%s': '%s' -> '%s' on topic '%s'.",
              edge.child_frame_id.c_str(), t.header.frame_id.c_str(), edge.header.frame_id.c_str(),
              st.name.c_str());
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
      // New child: it joins `topic`, which is declared by the pass when the
      // bag does not have it yet.
      auto it = std::find_if(
        topics.begin(), topics.end(), [&](const auto & st) { return st.name == topic; });
      if (it == topics.end()) {
        topics.push_back({topic, {}});
        it = std::prev(topics.end());
      }
      it->transforms.push_back(edge);
      touched.insert(topic);
      ++counts.added;
    }
  }

  // 4. The merged result must still be a forest: the YAML is validated on its
  //    own by the parse, but an update that re-parents an existing frame can
  //    close a cycle against edges the bag already carries.
  std::set<std::pair<std::string, std::string>> merged_edges;
  for (const auto & st : topics) {
    for (const auto & t : st.transforms) {
      merged_edges.emplace(t.header.frame_id, t.child_frame_id);
    }
  }
  if (const auto err = core::validate_tf_forest(merged_edges, "after applying the edit")) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
    return 1;
  }

  // 5. Rewrite, in bag topic order (a brand-new -t sorts last, as appended).
  std::vector<core::StaticTopicTransforms> touched_topics;
  for (const auto & st : topics) {
    if (touched.count(st.name) != 0) {
      touched_topics.push_back(st);
    }
  }
  if (touched_topics.empty()) {
    // Unreachable in practice (every accepted edit touches a topic), but a
    // no-op must not rewrite the bag for nothing.
    BAGWIZ_LOG_INFO(kLogger, "tf static edit: no edge changed; nothing to write.");
    return 0;
  }

  // 6. -o vs in-place dispatch, shared with the other rewrite-style commands.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "tf static edit: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "tf static edit: pass failed; aborting in-place swap";

  StaticTfInjectOptions inject_opts;
  inject_opts.logger = kLogger;
  inject_opts.label = "tf static edit";
  inject_opts.profile_label = "tf_static_edit";

  const int rc = core::run_bag_rewrite(
    input_path, output_path, overwrite, rewrite_opts, [&](const io::WriterFactory & open_writer) {
      return edit_static_tf_pass(input_path, touched_topics, inject_opts, open_writer);
    });
  if (rc == 0) {
    BAGWIZ_LOG_INFO(
      kLogger,
      "tf static edit: pruned %" PRIu64 " edge(s), added %" PRIu64 ", updated %" PRIu64
      " (re-parented %" PRIu64 ") across %zu topic(s).",
      counts.pruned, counts.added, counts.updated, counts.reparented, touched_topics.size());
  }
  return rc;
}

}  // namespace bagwiz::commands
