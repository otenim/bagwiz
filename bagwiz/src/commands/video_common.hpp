// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__VIDEO_COMMON_HPP_
#define COMMANDS__VIDEO_COMMON_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Shared by `video encode` and `video decode`: mapping the selected source
// topics onto the topics they produce, and the topic declarations of the
// rewrite pass. Each subcommand keeps its own per-message conversion.
namespace bagwiz::commands
{

// One source topic and the topic its converted messages are written to.
struct VideoTopicPlan
{
  std::string source;
  std::string source_type;
  std::string output;
};

struct VideoTopicPlanRequest
{
  const io::BagReader * reader = nullptr;
  std::span<const std::string> sources;
  // The message types a source may have.
  std::span<const std::string_view> allowed_types;
  // --as: the output name; only valid with exactly one source.
  std::optional<std::string> as_topic;
  bool keep_inputs = false;
  // Derives the output name of a source when --as is not given.
  std::function<std::string(const std::string &)> default_output;
  const char * command_label = nullptr;  // e.g. "video encode", for messages
  const char * logger = nullptr;
};

// Resolve every source to a plan entry, checking that each source exists in
// the bag with an accepted type and that no output name collides with a
// topic that survives the rewrite (an output may reuse its own source's
// name only when the source is being replaced). Logs every problem it
// finds and returns nullopt when there is any.
[[nodiscard]] std::optional<std::vector<VideoTopicPlan>> plan_video_topics(
  const VideoTopicPlanRequest & request);

// Declare the pass's topics on `writer`: every topic of `reader` except the
// sources being replaced, then one output topic per plan entry as built by
// `make_output`. Backfills embedded schemas first so MCAP outputs stay
// self-describing. Returns false after logging when a declaration fails.
[[nodiscard]] bool declare_video_pass_topics(
  io::BagReader & reader, io::BagWriter & writer, std::span<const VideoTopicPlan> plan,
  bool keep_inputs, const std::function<io::TopicInfo(const VideoTopicPlan &)> & make_output,
  const char * logger);

// Per-source progress of a pass, for the closing summary line.
struct VideoTopicCounters
{
  std::uint64_t messages_in = 0;       // source messages seen
  std::uint64_t messages_out = 0;      // converted messages written
  std::uint64_t bytes_in = 0;          // source payload bytes
  std::uint64_t bytes_out = 0;         // converted payload bytes
  std::uint64_t messages_dropped = 0;  // source messages that produced no output
};

// "<in> -> <out> bytes (x.xx%)" style ratio text for the summary.
[[nodiscard]] std::string describe_size_change(std::uint64_t bytes_in, std::uint64_t bytes_out);

}  // namespace bagwiz::commands

#endif  // COMMANDS__VIDEO_COMMON_HPP_
