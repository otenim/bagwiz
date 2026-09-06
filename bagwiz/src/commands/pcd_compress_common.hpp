// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__PCD_COMPRESS_COMMON_HPP_
#define COMMANDS__PCD_COMPRESS_COMMON_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// The message-transform rewrite pass shared by `pcd compress` and
// `pcd decompress` (src-local, like cam_info_common). Both commands replace
// selected topics with a per-message transform of their payloads — Draco
// encode in one direction, decode in the other — while copying every other
// topic through byte-identically and preserving message order at any thread
// count. What differs between them (topic selection, output naming, the
// transform itself, the summary wording) stays in the command sources.
namespace bagwiz::commands
{

// The outcome of transforming one message payload.
struct PcdTransformResult
{
  // The transformed payload, written to the mapped output topic. Ignored when
  // passthrough_reason or error is non-empty.
  std::vector<std::byte> payload;
  // Non-empty: this message cannot be transformed; copy its original bytes to
  // the ORIGINAL topic instead (declared lazily on first use, since the
  // selected topics are otherwise replaced and dropped) and log the reason at
  // WARN once per topic.
  std::string passthrough_reason;
  // Non-empty: fatal; the pass aborts with this error.
  std::string error;
};

// Transforms one message payload of a selected topic into the output payload.
// Called on worker threads when num_threads > 1, so the function must be
// thread-safe (both commands' transforms are pure functions of the payload).
using PcdTransformFn =
  std::function<PcdTransformResult(const std::string & topic, std::span<const std::byte> payload)>;

// Per-topic tallies of a transform pass, keyed by input topic name.
struct PcdTransformTopicStats
{
  std::uint64_t messages = 0;     // transformed messages
  std::uint64_t in_bytes = 0;     // their input payload bytes
  std::uint64_t out_bytes = 0;    // their output payload bytes
  std::uint64_t passthrough = 0;  // messages copied to the original topic instead
};
using PcdTransformStats = std::map<std::string, PcdTransformTopicStats>;

// The resolved topic plan of one transform pass: which input topics are
// transformed, under which output name each one lands, and which input topics
// are dropped from copy-through (the selected ones, plus any pre-existing
// topic a --force'd output name replaces).
struct PcdTransformPlan
{
  // Selected input topics in bag declaration order, with their output names.
  std::vector<std::pair<const io::TopicInfo *, std::string>> mappings;
  std::unordered_map<std::string, std::string> output_by_input;
  std::unordered_set<std::string> suppress;
  std::vector<io::TopicInfo> declare_outputs;
};

// Build the plan shared by `pcd compress` and `pcd decompress`:
//   * topics: -t selectors (already glob-expanded) or, when empty, every bag
//     topic of `input_type`. Each named topic must exist and carry
//     `input_type`.
//   * output names: `as` when given (requires exactly one selected topic),
//     else `default_name(topic)` per topic.
//   * an output name that already exists in the bag requires `force` (the
//     pre-existing topic is then dropped from copy-through). An output name
//     colliding with another selected topic is an error even with force.
// `output_type` describes the declared output topics (QoS is copied from each
// input topic); the schema text is resolved once from $AMENT_PREFIX_PATH via
// core::resolve_message_definition, falling back to no self-description with
// a WARN (the declare_reader_topics convention).
// Returns nullopt after logging the specific error.
[[nodiscard]] std::optional<PcdTransformPlan> plan_pcd_transform(
  const io::BagReader & reader, const std::vector<std::string> & topics, const std::string & as,
  bool force, const std::string & input_type, const std::string & output_type,
  const std::function<std::string(const std::string &)> & default_name,
  const std::filesystem::path & input_path, const char * logger, const char * cmd);

// Run the transform pass over the bag: declare every input topic except
// `suppress` plus every topic in `declare_outputs`, stream the input, copy
// unselected messages through byte-identically, and replace each message on a
// selected topic (a key of `output_by_input`) with transform()'s result,
// written to the mapped output topic. The reader `topic_reader` must stay
// alive for the whole pass (its TopicInfo list backs lazy passthrough
// declarations). Message order in the output matches the input at any
// num_threads; num_threads <= 1 runs synchronously on the caller thread.
// Returns a process exit code and fills `stats`.
int run_pcd_transform_pass(
  const io::WriterFactory & factory, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & suppress,
  const std::unordered_map<std::string, std::string> & output_by_input,
  const std::vector<io::TopicInfo> & declare_outputs, const PcdTransformFn & transform,
  int num_threads, PcdTransformStats & stats, const char * logger, const char * cmd);

}  // namespace bagwiz::commands

#endif  // COMMANDS__PCD_COMPRESS_COMMON_HPP_
