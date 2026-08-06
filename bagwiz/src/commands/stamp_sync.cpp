// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/stamp_sync.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pipeline/backend_select.hpp"
#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "trim_stamp.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.stamp.sync";

// Overwrites the leading header.stamp of every message on a headered topic
// with the message's receive time, forwarding every other message verbatim. A
// headered payload the stamp cannot be written into (truncated below the
// 12-byte encapsulation + stamp, or a receive time outside the
// builtin_interfaces/Time range) is also forwarded verbatim and tallied for
// the post-run warning.
class StampSyncProcessor : public core::pipeline::Processor
{
public:
  // `headered` is the caller-owned set of topics whose type leads with a
  // std_msgs/Header; it must outlive the processor.
  explicit StampSyncProcessor(const std::unordered_set<std::string> & headered)
  : headered_(headered)
  {
  }

  // Messages whose stamp could not be written (see class comment). Read after
  // the pipeline run completes.
  [[nodiscard]] std::uint64_t unpatchable_count() const { return unpatchable_; }

  [[nodiscard]] core::pipeline::Emit route(const std::string & in_topic) const override
  {
    return core::pipeline::Emit{true, in_topic};
  }

  [[nodiscard]] bool transforms() const override { return true; }

  [[nodiscard]] core::pipeline::TransformAction transform(
    const io::RawMessage & msg, std::vector<std::byte> & out) const override
  {
    if (headered_.find(msg.topic->name) == headered_.end()) {
      return core::pipeline::TransformAction::kPassthrough;
    }
    out.assign(msg.payload.begin(), msg.payload.end());
    if (!write_leading_header_stamp_ns(out, msg.timestamp_ns)) {
      ++unpatchable_;
      return core::pipeline::TransformAction::kPassthrough;
    }
    return core::pipeline::TransformAction::kWrite;
  }

private:
  const std::unordered_set<std::string> & headered_;
  // Mutable because transform() is const per the Processor contract, which
  // also guarantees transform() runs on the single producer thread — so this
  // is the sole writer and needs no synchronization.
  mutable std::uint64_t unpatchable_ = 0;
};

// One full pass: open `input_path`, declare every topic verbatim (names and
// types are unchanged), then stream-copy — writing each headered message's
// receive time into its header.stamp and forwarding everything else untouched.
// The writer factory is parameterised so the in-place path can hand in a tmp
// location.
int execute_pass(
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & headered,
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(input_path, kLogger);
  if (!reader) {
    return 1;
  }
  // populate_schemas() so the declared topics carry their embedded schema
  // text and the output stays self-describing.
  reader->populate_schemas();

  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  for (const auto & t : topics) {
    try {
      writer->declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  StampSyncProcessor processor(headered);
  core::pipeline::RewriteCounts counts;
  try {
    auto backend = core::pipeline::make_backend(core::pipeline::BackendKind::Pipelined);
    counts = core::pipeline::run_pipeline(*reader, *writer, processor, *backend, "stamp sync");
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "stamp sync read/write failed: %s", e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  const std::uint64_t forwarded = counts.copied - counts.transformed;
  BAGWIZ_LOG_INFO(
    kLogger,
    "stamp sync: wrote the receive time into header.stamp on %" PRIu64
    " message(s) across %zu headered topic(s), copied %" PRIu64 " other message(s) verbatim.",
    counts.transformed, headered.size(), forwarded);
  if (processor.unpatchable_count() > 0) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "%" PRIu64
      " message(s) on headered topic(s) were copied verbatim instead: payload too short for a "
      "leading header.stamp, or receive time outside the builtin_interfaces/Time range.",
      processor.unpatchable_count());
  }
  return 0;
}

}  // namespace

int run_stamp_sync(const StampSyncArgs & args)
{
  // 1. Inspect the bag and classify every topic by leading-Header presence.
  //    populate_schemas() first, so MCAP shard readers carry the embedded
  //    schema text the classification prefers over $AMENT_PREFIX_PATH lookup.
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();
  const std::vector<io::TopicInfo> topics(reader->topics().begin(), reader->topics().end());
  const HeaderedTopics classified = classify_headered_topics(topics);

  if (!classified.unresolved_types.empty()) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "stamp sync: %zu message type(s) could not be classified for header.stamp (e.g. '%s'); "
      "their topics are copied verbatim.",
      classified.unresolved_types.size(), classified.unresolved_types.front().c_str());
  }
  if (classified.topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Bag has no topic whose type leads with a std_msgs/Header; nothing to sync. The input is "
      "left untouched.");
    return 1;
  }

  // Release the inspection reader before opening the read/write pass.
  reader.reset();

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a fresh bag (format/layout resolved from the output path's
  //    extension) and leaves <input> untouched; otherwise <input> is rewritten
  //    atomically via a sibling tmp, preserving its storage identity.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error = "Could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "stamp sync: pass failed; aborting in-place swap";
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer) {
      return execute_pass(args.input_path, classified.topics, open_writer);
    });
}

}  // namespace bagwiz::commands
