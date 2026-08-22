// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_BUFFER_LOADER_HPP_
#define BAGWIZ__CORE__TF__TF_BUFFER_LOADER_HPP_

#include "bagwiz/core/tf/tf_merge_check.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core
{

// Load every tf2_msgs/msg/TFMessage topic from `input` into `buffer`.
// Returns std::nullopt on success, or an error string on failure.
[[nodiscard]] std::optional<std::string> load_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer);

// The bag's static transforms — every TransformStamped on every static TF
// topic (…/tf_static), in read order — with `read` deciding how much of each
// topic is read: StaticTfRead::kWholeTopic (every message, so a value a later
// message re-publishes wins once fed into a buffer) or kFirstMessagePerTopic
// (stop at each static topic's first message and stop reading the bag once
// every static topic has produced one; static TF is latched, so the first
// message is normally the whole tree, and the rest of a long recording is
// republications this mode never reads). `error` is set, and `transforms`
// empty, on the failures load_static_tf_buffer reports: no static topic, a
// decoder or decode failure, an IO error.
struct StaticTfTransforms
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  std::string error;
  [[nodiscard]] bool ok() const { return error.empty(); }
};
[[nodiscard]] StaticTfTransforms load_static_tf_transforms(
  const std::filesystem::path & input, StaticTfRead read = StaticTfRead::kWholeTopic);

// Feed `transforms` into `buffer` as static transforms (authority "bagwiz"),
// in order — what load_static_tf_buffer does with what it read, exposed so a
// caller that needs the static tree in more than one buffer reads the bag
// once.
void set_static_transforms(
  tf2::BufferCore & buffer, std::span<const geometry_msgs::msg::TransformStamped> transforms);

// Load ONLY the bag's static TF topics (…/tf_static) into `buffer` as static
// transforms (dynamic /tf is ignored): load_static_tf_transforms with `read`,
// then set_static_transforms. Returns std::nullopt on success, else an error
// string (no static topic, decode failure, IO error).
[[nodiscard]] std::optional<std::string> load_static_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer,
  StaticTfRead read = StaticTfRead::kWholeTopic);

// One observed (parent, child) edge with its stamp, recorded for the
// transforms republished on a nominated input topic. Used by trajectory
// sampling, which filters the input topic's edges by chain membership after
// the chain has been resolved.
struct TfInputEdge
{
  std::string frame_id;
  std::string child_frame_id;
  std::int64_t stamp_ns = 0;
};

// Side outputs of replay_tf_topics. Every pointer member is optional:
// nullptr means "do not collect this output". Set only what the caller needs:
//
//   buffer            every transform is fed with the correct static/dynamic
//                     flag (authority "bagwiz").
//   conflict_checker  every transform with both frame ids non-empty is run
//                     through the checker; the first conflict throws
//                     std::runtime_error ("TF merge conflict: ...").
//   edges_by_topic    distinct (parent, child) edges keyed by source topic
//                     (transforms with an empty frame id are skipped).
//   stamps            every transform's header stamp, in read order.
//   input_topic +     transforms whose topic equals `input_topic` are
//   input_edges       additionally recorded as TfInputEdge (stamp included);
//                     `input_topic` is ignored when `input_edges` is null.
struct TfReplayOutputs
{
  tf2::BufferCore * buffer = nullptr;
  TfMergeConflictChecker * conflict_checker = nullptr;
  std::map<std::string, std::set<std::pair<std::string, std::string>>> * edges_by_topic = nullptr;
  std::vector<tf2::TimePoint> * stamps = nullptr;
  std::string input_topic;
  std::vector<TfInputEdge> * input_edges = nullptr;
};

// Replay the given TF topics of the already-open `reader` once, decoding with
// one decoder per topic (per-topic schema_text differences are handled by the
// decoder factory) and feeding the requested `outputs`. Sets the reader's
// filter to `tf_topics`. Throws std::runtime_error when a decoder cannot be
// opened ("Could not open decoder for TF topic '<name>': ..."), when a message
// fails to decode ("Failed to decode TF message on '<name>': ..."), or when
// the conflict checker reports a contradiction.
//
// Decoding goes through the unified open_decoder() path, so for MCAP inputs
// the schema-driven backend does the work and tf2_msgs does not need to be on
// AMENT_PREFIX_PATH at runtime; only its header-only struct definition is
// required at build time (via extract_tf_message ->
// geometry_msgs::msg::TransformStamped).
void replay_tf_topics(
  io::BagReader & reader, const std::vector<TfTopic> & tf_topics, const TfReplayOutputs & outputs);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_BUFFER_LOADER_HPP_
