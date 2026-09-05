// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

std::string join_types(std::span<const std::string_view> types)
{
  std::string joined;
  for (const auto & t : types) {
    joined += (joined.empty() ? "" : ", ");
    joined += t;
  }
  return joined;
}

// Look every source up in the bag and check its type. Returns the entries
// (output names still empty) or nullopt after logging each problem.
std::optional<std::vector<VideoTopicPlan>> resolve_sources(const VideoTopicPlanRequest & request)
{
  std::vector<VideoTopicPlan> plan;
  std::unordered_set<std::string> seen;
  bool all_ok = true;
  for (const auto & source : request.sources) {
    if (!seen.insert(source).second) {
      continue;  // a selector matched the same topic twice
    }
    const io::TopicInfo * info = io::find_topic(*request.reader, source);
    if (info == nullptr) {
      BAGWIZ_LOG_ERROR(
        request.logger, "%s: topic '%s' is not present in the bag.", request.command_label,
        source.c_str());
      all_ok = false;
      continue;
    }
    const bool accepted = std::find(
                            request.allowed_types.begin(), request.allowed_types.end(),
                            std::string_view{info->type}) != request.allowed_types.end();
    if (!accepted) {
      BAGWIZ_LOG_ERROR(
        request.logger, "%s: topic '%s' has type %s; expected one of: %s.", request.command_label,
        source.c_str(), info->type.c_str(), join_types(request.allowed_types).c_str());
      all_ok = false;
      continue;
    }
    plan.push_back(VideoTopicPlan{source, info->type, {}});
  }
  if (!all_ok) {
    return std::nullopt;
  }
  return plan;
}

// Check every output name against the topics that survive the rewrite and
// against the other outputs. Logs each collision.
bool outputs_are_free(const VideoTopicPlanRequest & request, std::span<const VideoTopicPlan> plan)
{
  std::unordered_set<std::string> sources;
  for (const auto & entry : plan) {
    sources.insert(entry.source);
  }
  std::unordered_set<std::string> outputs;
  bool all_ok = true;
  for (const auto & entry : plan) {
    if (!outputs.insert(entry.output).second) {
      BAGWIZ_LOG_ERROR(
        request.logger, "%s: two source topics would both be written to '%s'.",
        request.command_label, entry.output.c_str());
      all_ok = false;
      continue;
    }
    const bool replaces_own_source = entry.output == entry.source && !request.keep_inputs;
    if (replaces_own_source) {
      continue;
    }
    const bool is_dropped_source = sources.count(entry.output) != 0 && !request.keep_inputs;
    const bool exists = io::find_topic(*request.reader, entry.output) != nullptr;
    if (exists && !is_dropped_source) {
      BAGWIZ_LOG_ERROR(
        request.logger,
        "%s: output topic '%s' already exists in the bag; pass --as to choose another name%s.",
        request.command_label, entry.output.c_str(),
        entry.output == entry.source ? " (or drop --keep-inputs to replace the source)" : "");
      all_ok = false;
    } else if (is_dropped_source) {
      BAGWIZ_LOG_ERROR(
        request.logger,
        "%s: output topic '%s' for '%s' is the name of another source topic being replaced.",
        request.command_label, entry.output.c_str(), entry.source.c_str());
      all_ok = false;
    }
  }
  return all_ok;
}

}  // namespace

std::optional<std::vector<VideoTopicPlan>> plan_video_topics(const VideoTopicPlanRequest & request)
{
  if (request.sources.empty()) {
    BAGWIZ_LOG_ERROR(request.logger, "%s: no source topic selected.", request.command_label);
    return std::nullopt;
  }
  auto plan = resolve_sources(request);
  if (!plan.has_value()) {
    return std::nullopt;
  }
  if (request.as_topic.has_value() && plan->size() != 1) {
    BAGWIZ_LOG_ERROR(
      request.logger,
      "%s: --as names one output topic, but %zu source topics were selected; drop --as to "
      "derive one output name per source.",
      request.command_label, plan->size());
    return std::nullopt;
  }
  for (auto & entry : *plan) {
    entry.output =
      request.as_topic.has_value() ? *request.as_topic : request.default_output(entry.source);
    if (entry.output.empty()) {
      BAGWIZ_LOG_ERROR(
        request.logger, "%s: the output topic name must not be empty.", request.command_label);
      return std::nullopt;
    }
  }
  if (!outputs_are_free(request, *plan)) {
    return std::nullopt;
  }
  return plan;
}

bool declare_video_pass_topics(
  io::BagReader & reader, io::BagWriter & writer, std::span<const VideoTopicPlan> plan,
  bool keep_inputs, const std::function<io::TopicInfo(const VideoTopicPlan &)> & make_output,
  const char * logger)
{
  std::unordered_set<std::string> dropped;
  if (!keep_inputs) {
    for (const auto & entry : plan) {
      dropped.insert(entry.source);
    }
  }
  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // copied topics (no-op for single-file readers and SQLite3 inputs).
  reader.populate_schemas();
  for (const auto & t : reader.topics()) {
    if (dropped.count(t.name) != 0) {
      continue;
    }
    try {
      writer.declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(logger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return false;
    }
  }
  for (const auto & entry : plan) {
    try {
      writer.declare_topic(make_output(entry));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        logger, "declare_topic failed for new topic '%s': %s", entry.output.c_str(), e.what());
      return false;
    }
  }
  return true;
}

std::string describe_size_change(std::uint64_t bytes_in, std::uint64_t bytes_out)
{
  std::string text = std::to_string(bytes_in) + " -> " + std::to_string(bytes_out) + " bytes";
  if (bytes_in > 0) {
    const double percent = 100.0 * static_cast<double>(bytes_out) / static_cast<double>(bytes_in);
    std::array<char, 32> buf{};
    std::snprintf(buf.data(), buf.size(), " (%.1f%%)", percent);
    text += buf.data();
  }
  return text;
}

}  // namespace bagwiz::commands
