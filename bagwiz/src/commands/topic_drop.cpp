// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_drop.hpp"

#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/bag_passthrough.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.topic";

// Try the mcap chunk pass-through first; when it declines, declare every input
// topic except those in `drop`, push the survivors down as the reader's filter,
// and stream-copy the bag with the same set suppressed as a backstop. Shared by
// the in-place and -o modes; the writer factory is injected so the rewrite
// dispatch (core::run_bag_rewrite) can supply a tmp path. Returns a process
// exit code.
int execute_drop_pass(
  const TopicDropArgs & args, const std::unordered_set<std::string> & drop,
  const io::WriterFactory & open_writer, const core::RewriteTarget & target)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }

  // Neither path surfaces the suppressed messages (the pass-through never
  // reads them, the filtered reader never yields them), so report the
  // suppressed total from the bag's statistics instead (0, after the
  // backend's own warning, when the input carries none).
  const auto suppressed = core::count_topic_messages(*reader, drop);
  if (!suppressed.has_value()) {
    BAGWIZ_LOG_WARN(
      kLogger, "Could not compute the suppressed message count for %s; reporting 0.",
      args.input_path.c_str());
  }

  // Chunk pass-through fast path: mcap chunks untouched by the drop are
  // copied byte-for-byte, preserving the input's chunk compression. Falls
  // back to the decoded stream copy below whenever the input, the target, or
  // the edit is ineligible.
  {
    core::PassthroughEdit edit;
    edit.suppress_topics = drop;
    if (const auto pt = core::try_bag_passthrough_rewrite(args.input_path, target, edit, kLogger)) {
      std::size_t kept = 0;
      for (const auto & t : reader->topics()) {
        if (drop.count(t.name) == 0) {
          ++kept;
        }
      }
      BAGWIZ_LOG_INFO(
        kLogger,
        "topic drop: kept %zu topic(s), dropped %zu; copied %" PRIu64
        " message(s), suppressed %" PRId64 ".",
        kept, drop.size(), pt->copied, suppressed.value_or(0));
      return 0;
    }
  }

  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // surviving topics (no-op for single-file readers and SQLite3 inputs).
  reader->populate_schemas();

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  // A dropped topic disappears entirely from the output: it is neither declared
  // nor carries any message. Every other topic is declared verbatim.
  std::size_t kept = 0;
  for (const auto & t : reader->topics()) {
    if (drop.count(t.name) != 0) {
      continue;
    }
    try {
      writer->declare_topic(t);
      ++kept;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  // Push the surviving topics down to the reader: the indexed MCAP path can
  // then prune chunks whose messages all belong to dropped topics (and the
  // SQLite backend filters in SQL). Command-level for the same reason as
  // `topic keep`: bag_copy_filtered must not install reader filters itself.
  // When every topic is dropped the survivor list is empty, which ReadFilter
  // treats as "all topics" — skip the push-down and let the suppress set do
  // the (correct) work alone.
  io::ReadFilter filter;
  for (const auto & t : reader->topics()) {
    if (drop.count(t.name) == 0) {
      filter.topics.push_back(t.name);
    }
  }
  if (!filter.topics.empty()) {
    reader->set_filter(filter);
  }

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, drop, "topic drop", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "topic drop: kept %zu topic(s), dropped %zu; copied %" PRIu64 " message(s), suppressed %" PRId64
    ".",
    kept, drop.size(), counts.copied, suppressed.value_or(0));
  return 0;
}

}  // namespace

int run_topic_drop(const TopicDropArgs & args)
{
  // 0. Guard the public entry point. The CLI marks <topics> ->required(), but
  //    run_topic_drop is also called directly from tests; an empty selector
  //    list would otherwise build an empty `drop` set below and silently copy
  //    the bag unchanged. Fail fast instead.
  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic drop: no topic selector given; nothing to remove.");
    return 1;
  }

  // Claim -o before touching the bag. The check is non-destructive (the
  // dispatch below removes the existing entry), so it only moves the collision
  // verdict ahead of the read.
  if (args.output_path.has_value()) {
    if (const auto r = core::check_output_path_free(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }

  // 1. Snapshot the bag's topic list up front so the "all topics matched"
  //    warning below can compare against it. The topic list is snapshotted
  //    into a vector because the reader's span is invalidated once the reader
  //    is destroyed below.
  std::vector<std::string> topic_names;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    topic_names = io::snapshot_topic_names(*reader);
  }

  // Selectors were expanded before run() (see commands/topic_option.hpp), so
  // args.topics is already the literal list of topics to remove — the CLI's
  // -t/--topics slot sets TopicSlotSpec::require_present, so every entry is
  // supposed to already name a real topic by the time run() is reached.
  const std::unordered_set<std::string> drop(args.topics.begin(), args.topics.end());

  // Precondition assert, not selector validation: require_present (above) is
  // where "does this name a real topic" is supposed to be decided, once, for
  // every command that opts in. run_topic_drop is also called directly from
  // tests, bypassing that pass entirely, so this backstops the specific
  // failure mode that motivated require_present in the first place — a
  // name that reaches here without matching a real topic would otherwise
  // silently drop nothing instead of erroring.
  for (const auto & name : drop) {
    if (std::find(topic_names.begin(), topic_names.end(), name) == topic_names.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "topic drop: internal precondition failed: '%s' is not a topic in %s; expansion should "
        "have rejected this before run() was reached.",
        name.c_str(), args.input_path.c_str());
      return 1;
    }
  }

  if (drop.size() == topic_names.size()) {
    BAGWIZ_LOG_WARN(
      kLogger, "all %zu topic(s) matched; the output bag will contain no topics.",
      topic_names.size());
  }

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a new bag and leaves <input> untouched; otherwise <input> is
  //    rewritten atomically via a sibling tmp, preserving its storage format
  //    and layout.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "topic drop: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "topic drop: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer, const core::RewriteTarget & target) {
      return execute_drop_pass(args, drop, open_writer, target);
    });
}

}  // namespace bagwiz::commands
