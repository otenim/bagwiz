// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/bag_topic_plan.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

// Set every transform's header.stamp to `stamp_ns` so the injected static TF
// carries the destination's start time rather than whatever the source held.
void restamp_transforms(
  std::vector<geometry_msgs::msg::TransformStamped> & transforms, std::int64_t stamp_ns)
{
  const auto sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  const auto nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  for (auto & t : transforms) {
    t.header.stamp.sec = sec;
    t.header.stamp.nanosec = nanosec;
  }
}

// The plan for declaring/suppressing one static topic against the destination
// bag.
struct TopicWritePlan
{
  std::unordered_set<std::string> suppress;     // copy-time drop set (replace)
  std::vector<std::string> declare_new_topics;  // topics absent from dst
};

// Decide, for every topic being injected, whether it is declared fresh, kept,
// replaced (suppress + re-append), or aborts the run. `force` is the caller's
// replace_existing_topic. Returns false (after logging) on an unresolved
// conflict.
bool plan_topic_writes(
  std::span<const io::TopicInfo> dst_topics,
  const std::unordered_map<std::string, std::int64_t> & dst_counts,
  const std::vector<core::StaticTopicTransforms> & topics, bool force, const char * logger,
  TopicWritePlan & plan_out)
{
  for (const auto & st : topics) {
    std::int64_t existing_count = 0;
    if (auto it = dst_counts.find(st.name); it != dst_counts.end()) {
      existing_count = it->second;
    }
    const auto decision =
      core::decide_topic_write(dst_topics, st.name, kTfMessageType, existing_count, force);
    switch (decision.action) {
      case core::TopicWriteAction::kConflictAbort:
      case core::TopicWriteAction::kTypeMismatch:
        BAGWIZ_LOG_ERROR(logger, "%s", decision.reason.c_str());
        return false;
      case core::TopicWriteAction::kDeclareAndSuppress:
        BAGWIZ_LOG_WARN(logger, "%s", decision.reason.c_str());
        plan_out.suppress.insert(st.name);
        break;
      case core::TopicWriteAction::kDeclareNew:
        plan_out.declare_new_topics.push_back(st.name);
        break;
      case core::TopicWriteAction::kDeclareKeep:
        // Topic exists but is empty: keep its declaration, just append.
        break;
    }
  }
  return true;
}

// Declare every topic the destination bag already carries. Returns false after
// logging on a declare failure.
bool declare_existing_topics(
  io::BagWriter & writer, const std::vector<io::TopicInfo> & dst_topics_with_schemas,
  const char * logger)
{
  for (const auto & t : dst_topics_with_schemas) {
    try {
      writer.declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(logger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return false;
    }
  }
  return true;
}

// Serialize and write one TFMessage per entry in `topics`, stamped (receive
// time and header.stamp) at `stamp_ns`. Entries with no transforms write
// nothing. Returns the number of messages written, or std::nullopt after
// logging a serialize/write failure.
std::optional<std::uint64_t> write_synthesized_tf_messages(
  io::BagWriter & writer, const std::vector<core::StaticTopicTransforms> & topics,
  std::int64_t stamp_ns, const char * logger)
{
  std::uint64_t written = 0;
  core::TfMessageSerializer tf_serializer;
  for (const auto & st : topics) {
    if (st.transforms.empty()) {
      continue;
    }
    auto transforms = st.transforms;
    restamp_transforms(transforms, stamp_ns);
    std::vector<std::byte> payload;
    try {
      tf_serializer.serialize_many(
        std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
        payload);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        logger, "Failed to serialize TFMessage for topic '%s': %s", st.name.c_str(), e.what());
      return std::nullopt;
    }
    try {
      writer.write(st.name, stamp_ns, std::span<const std::byte>(payload.data(), payload.size()));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        logger, "Failed to write TFMessage on '%s' at stamp %" PRId64 ": %s", st.name.c_str(),
        stamp_ns, e.what());
      return std::nullopt;
    }
    ++written;
  }
  return written;
}

}  // namespace

int inject_static_tf_pass(
  const std::filesystem::path & dst_path, const std::vector<core::StaticTopicTransforms> & topics,
  const StaticTfInjectOptions & options, const io::WriterFactory & open_writer)
{
  const char * logger = options.logger;

  auto reader = io::open_read_or_log(dst_path, logger);
  if (!reader) {
    return 1;
  }
  io::BagReader::TimeExtent time_extent;
  try {
    time_extent = reader->compute_time_extent();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to compute time extent on %s: %s", dst_path.c_str(), e.what());
    return 1;
  }
  const std::int64_t start_ns = time_extent.start_ns;

  std::vector<std::string> topic_names;
  topic_names.reserve(topics.size());
  for (const auto & st : topics) {
    topic_names.push_back(st.name);
  }

  std::unordered_map<std::string, std::int64_t> dst_counts;
  try {
    dst_counts = reader->compute_topic_counts(topic_names);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      logger, "Failed to compute topic counts on %s: %s", dst_path.c_str(), e.what());
    return 1;
  }

  // Snapshot the destination's topic list before conflict detection; the
  // reader's span may be invalidated by subsequent operations.
  const std::vector<io::TopicInfo> dst_topics(reader->topics().begin(), reader->topics().end());

  TopicWritePlan plan;
  if (!plan_topic_writes(
        dst_topics, dst_counts, topics, options.replace_existing_topic, logger, plan)) {
    return 1;
  }

  auto writer = io::open_write_or_log(open_writer, logger);
  if (!writer) {
    return 1;
  }

  // Schemas are only needed once we start streaming messages. Deferring
  // avoids opening shard 0 for bags that abort early due to a topic conflict.
  reader->populate_schemas();

  // Snapshot the destination's topic list after schema backfill so the output
  // writer receives embedded schemas.
  const std::vector<io::TopicInfo> dst_topics_with_schemas(
    reader->topics().begin(), reader->topics().end());

  // Declare every existing destination topic, then a synthesised TopicInfo for
  // each brand-new static topic being introduced.
  if (!declare_existing_topics(*writer, dst_topics_with_schemas, logger)) {
    return 1;
  }
  for (const auto & name : plan.declare_new_topics) {
    try {
      writer->declare_topic(core::make_tf_message_topic_info(name));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        logger, "declare_topic failed for new topic '%s': %s", name.c_str(), e.what());
      return 1;
    }
  }

  // Emit the synthesized messages BEFORE the stream copy. They are stamped at
  // the destination's start time, so writing them afterwards would leave each
  // one holding the bag's lowest timestamp at the highest storage position —
  // the only rows whose physical order disagrees with their time. Consumers
  // that read a .db3 in row order rather than by timestamp (Foxglove's readers
  // issue their message query without an ORDER BY) would then receive the
  // static TF last, after everything it is supposed to precede.
  const auto injected = write_synthesized_tf_messages(*writer, topics, start_ns, logger);
  if (!injected) {
    return 1;
  }

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, plan.suppress, options.profile_label,
      core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Stream copy from %s failed: %s", dst_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, logger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    logger,
    "%s: copied %" PRIu64 " message(s), suppressed %" PRIu64 ", injected %" PRIu64
    " static TF topic(s) at stamp %" PRId64 ".",
    options.label.c_str(), counts.copied, counts.suppressed, *injected, start_ns);
  return 0;
}

int edit_static_tf_pass(
  const std::filesystem::path & dst_path, const std::vector<core::StaticTopicTransforms> & touched,
  const StaticTfInjectOptions & options, const io::WriterFactory & open_writer)
{
  const char * logger = options.logger;

  auto reader = io::open_read_or_log(dst_path, logger);
  if (!reader) {
    return 1;
  }
  io::BagReader::TimeExtent time_extent;
  try {
    time_extent = reader->compute_time_extent();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to compute time extent on %s: %s", dst_path.c_str(), e.what());
    return 1;
  }
  const std::int64_t start_ns = time_extent.start_ns;

  // Snapshot the destination's topic list before planning; the reader's span
  // may be invalidated by subsequent operations.
  const std::vector<io::TopicInfo> dst_topics(reader->topics().begin(), reader->topics().end());

  // Plan each touched topic against what the destination has: an existing one
  // is suppressed (its messages are replaced by the edited set), an absent one
  // is declared new. Unlike inject_static_tf_pass there is no conflict to
  // force past — the caller selected the touched set explicitly — but a touched
  // topic that exists as something other than a static TFMessage topic was
  // never loaded by the edit, and rewriting it would destroy messages the
  // command knows nothing about.
  std::unordered_set<std::string> suppress;
  std::vector<std::string> declare_new_topics;
  for (const auto & st : touched) {
    const io::TopicInfo * existing = nullptr;
    for (const auto & t : dst_topics) {
      if (t.name == st.name) {
        existing = &t;
        break;
      }
    }
    if (existing == nullptr) {
      declare_new_topics.push_back(st.name);
      continue;
    }
    if (existing->type != kTfMessageType || !core::is_static_tf_topic(st.name)) {
      BAGWIZ_LOG_ERROR(
        logger,
        "Topic '%s' exists in the destination with type '%s'; tf static edit rewrites only "
        "static tf2_msgs/msg/TFMessage topics.",
        st.name.c_str(), existing->type.c_str());
      return 1;
    }
    suppress.insert(st.name);
  }

  auto writer = io::open_write_or_log(open_writer, logger);
  if (!writer) {
    return 1;
  }

  reader->populate_schemas();

  const std::vector<io::TopicInfo> dst_topics_with_schemas(
    reader->topics().begin(), reader->topics().end());

  if (!declare_existing_topics(*writer, dst_topics_with_schemas, logger)) {
    return 1;
  }
  for (const auto & name : declare_new_topics) {
    try {
      writer->declare_topic(core::make_tf_message_topic_info(name));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        logger, "declare_topic failed for new topic '%s': %s", name.c_str(), e.what());
      return 1;
    }
  }

  // A topic pruned down to no edges keeps its declaration but carries no
  // message — a declared-but-empty static topic is how a bag says "no static
  // TF here" without losing the topic's metadata.
  for (const auto & st : touched) {
    if (st.transforms.empty()) {
      BAGWIZ_LOG_INFO(
        logger,
        "Topic '%s' has no static transforms left; keeping its declaration without "
        "messages.",
        st.name.c_str());
    }
  }

  // Write the edited messages BEFORE the stream copy, for the row-order reason
  // inject_static_tf_pass documents.
  const auto rewritten = write_synthesized_tf_messages(*writer, touched, start_ns, logger);
  if (!rewritten) {
    return 1;
  }

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, suppress, options.profile_label, core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Stream copy from %s failed: %s", dst_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, logger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    logger,
    "%s: copied %" PRIu64 " message(s), suppressed %" PRIu64 ", rewrote %" PRIu64
    " static TF topic(s) at stamp %" PRId64 ".",
    options.label.c_str(), counts.copied, counts.suppressed, *rewritten, start_ns);
  return 0;
}

}  // namespace bagwiz::commands
