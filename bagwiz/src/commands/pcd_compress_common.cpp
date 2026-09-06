// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_compress_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cinttypes>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

// One selected-topic message handed off from the bag reader to a worker
// thread. `frozen` shares the reader's chunk backing (BagReader::freeze)
// instead of copying the bytes; it pins that whole chunk while the job is in
// flight, so peak pinned memory is bounded by ParallelContext::max_in_flight
// chunks.
struct TransformJob
{
  std::size_t seq = 0;
  std::string topic;  // input topic
  std::int64_t timestamp_ns = 0;
  io::FrozenMessage frozen;
};

// One output message waiting in the in-order completion map for the
// collector. `frozen` is either the frozen input bytes (copy-through and
// pass-through messages) or a freshly serialized payload the item owns.
struct TransformOutputItem
{
  std::string topic;        // the topic to write: mapped output, or the original on pass-through
  std::string input_topic;  // the selected input topic (stats + WARN dedup)
  std::int64_t timestamp_ns = 0;
  io::FrozenMessage frozen;
  std::uint64_t in_bytes = 0;
  std::string passthrough_reason;  // non-empty: original bytes under the original topic
  std::optional<std::string> error;
};

// Shared state for the parallel reader / worker pool / collector pipeline
// (same shape as pcd undistort's ParallelContext).
struct ParallelContext
{
  std::mutex mutex;
  std::condition_variable cv;
  std::queue<TransformJob> job_queue;
  std::map<std::size_t, TransformOutputItem> completed;
  std::size_t next_output_seq = 0;
  std::size_t total_submitted = 0;
  std::size_t in_flight = 0;
  std::size_t max_in_flight = 0;
  bool stop = false;
  bool reader_done = false;
};

// The collector-side bookkeeping shared by the sync and parallel passes: both
// drain TransformOutputItems in bag order, so the write path, the lazy
// pass-through declaration, the once-per-topic WARN, and the stats are
// implemented exactly once.
class OutputDrainer
{
public:
  OutputDrainer(
    io::BagWriter & writer, const io::BagReader & topic_reader, PcdTransformStats & stats,
    const char * logger, const char * cmd)
  : writer_(writer), topic_reader_(topic_reader), stats_(stats), logger_(logger), cmd_(cmd)
  {
  }

  // Write one item. Returns false when the item carries a fatal error (the
  // caller aborts the pass).
  bool drain(TransformOutputItem & item)
  {
    if (item.error.has_value()) {
      BAGWIZ_LOG_ERROR(
        logger_, "%s: transform failed on '%s': %s", cmd_, item.input_topic.c_str(),
        item.error->c_str());
      return false;
    }
    auto & topic_stats = stats_[item.input_topic];
    if (!item.passthrough_reason.empty()) {
      // The original topic was replaced by its transform target, so it is not
      // declared up front; declare it on first use. The TopicInfo lookup
      // always hits: the message came from this bag.
      if (declared_passthrough_.insert(item.topic).second) {
        const io::TopicInfo * info = io::find_topic(topic_reader_, item.topic);
        if (info != nullptr) {
          writer_.declare_topic(*info);
        }
      }
      if (warned_.insert(item.input_topic).second) {
        BAGWIZ_LOG_WARN(
          logger_,
          "%s: cannot transform a message on '%s': %s; copying it through under the "
          "original topic (further such messages on this topic are counted, not logged)",
          cmd_, item.input_topic.c_str(), item.passthrough_reason.c_str());
      }
      ++topic_stats.passthrough;
    } else {
      ++topic_stats.messages;
      topic_stats.in_bytes += item.in_bytes;
      topic_stats.out_bytes += static_cast<std::uint64_t>(item.frozen.payload.size());
    }
    writer_.write(item.topic, item.timestamp_ns, item.frozen.payload);
    return true;
  }

private:
  io::BagWriter & writer_;
  const io::BagReader & topic_reader_;
  PcdTransformStats & stats_;
  const char * logger_;
  const char * cmd_;
  std::unordered_set<std::string> declared_passthrough_;
  std::unordered_set<std::string> warned_;
};

// Transform one message on a worker thread; touches only local state plus the
// (thread-safe by contract) transform function. `output_topic` is the mapped
// output topic the transformed payload is written to.
TransformOutputItem process_transform_job(
  TransformJob job, const PcdTransformFn & transform, const std::string & output_topic)
{
  TransformOutputItem item;
  item.topic = output_topic;
  item.input_topic = job.topic;
  item.timestamp_ns = job.timestamp_ns;
  item.in_bytes = static_cast<std::uint64_t>(job.frozen.payload.size());

  try {
    PcdTransformResult result = transform(job.topic, job.frozen.payload);
    if (!result.error.empty()) {
      item.error = std::move(result.error);
    } else if (!result.passthrough_reason.empty()) {
      item.topic = job.topic;  // pass through under the ORIGINAL topic
      item.passthrough_reason = std::move(result.passthrough_reason);
      item.frozen = std::move(job.frozen);
    } else {
      item.frozen = io::own_payload(std::move(result.payload));
    }
  } catch (const std::exception & e) {
    item.error = std::string("exception: ") + e.what();
  }
  return item;
}

// Parallel pass: one reader thread, a fixed-size std::jthread worker pool
// running transform(), and one collector thread that alone calls
// writer.write(), draining strictly by submission order. Copy-through
// messages bypass the job queue and go straight into the in-order completion
// map, so the output message order is identical to the synchronous pass and
// independent of the thread count.
int run_parallel_transform_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & suppress,
  const std::unordered_map<std::string, std::string> & output_by_input,
  const std::vector<io::TopicInfo> & declare_outputs, const PcdTransformFn & transform,
  int num_threads, PcdTransformStats & stats, const char * logger, const char * cmd)
{
  for (const auto & t : topic_reader.topics()) {
    if (suppress.count(t.name) != 0) {
      continue;
    }
    writer.declare_topic(t);
  }
  for (const auto & out : declare_outputs) {
    writer.declare_topic(out);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  ParallelContext ctx;
  ctx.max_in_flight = static_cast<std::size_t>(num_threads) * 3;

  int collector_status = 0;

  auto worker = [&]() {
    while (true) {
      TransformJob job;
      {
        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.stop || !ctx.job_queue.empty(); });
        if (ctx.stop && ctx.job_queue.empty()) {
          return;
        }
        job = std::move(ctx.job_queue.front());
        ctx.job_queue.pop();
      }

      const std::size_t seq = job.seq;
      const std::string & output_topic = output_by_input.at(job.topic);
      TransformOutputItem item = process_transform_job(std::move(job), transform, output_topic);

      {
        std::lock_guard lock(ctx.mutex);
        ctx.completed.emplace(seq, std::move(item));
      }
      ctx.cv.notify_all();
    }
  };

  auto collector = [&]() {
    OutputDrainer drainer(writer, topic_reader, stats, logger, cmd);
    try {
      while (true) {
        TransformOutputItem item;
        {
          std::unique_lock lock(ctx.mutex);
          ctx.cv.wait(lock, [&] {
            return ctx.completed.count(ctx.next_output_seq) != 0 ||
                   (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted);
          });

          if (ctx.reader_done && ctx.next_output_seq == ctx.total_submitted) {
            break;
          }
          auto it = ctx.completed.find(ctx.next_output_seq);
          if (it == ctx.completed.end()) {
            continue;
          }
          item = std::move(it->second);
          ctx.completed.erase(it);
          ++ctx.next_output_seq;
          --ctx.in_flight;
        }
        ctx.cv.notify_all();

        if (!drainer.drain(item)) {
          collector_status = 1;
          {
            std::lock_guard lock(ctx.mutex);
            ctx.stop = true;
          }
          ctx.cv.notify_all();
          try {
            writer.close();
          } catch (...) {
            // A writer close error is secondary to the transform error already reported.
          }
          return;
        }
      }

      if (!io::close_writer_or_log(writer, logger)) {
        collector_status = 1;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(logger, "%s: collector error: %s", cmd, e.what());
      collector_status = 1;
      {
        std::lock_guard lock(ctx.mutex);
        ctx.stop = true;
      }
      ctx.cv.notify_all();
    }
  };

  std::vector<std::jthread> workers;
  workers.reserve(num_threads);
  for (int i = 0; i < num_threads; ++i) {
    workers.emplace_back(worker);
  }
  std::jthread collector_thread(collector);

  // Copy-through messages keep their bag position: they join the same
  // in-order completion map the workers fill, so the collector sees one
  // interleaved stream in submission order.
  io::RawMessage raw;
  try {
    while (true) {
      const bool got = rd->next(raw);
      if (!got) {
        break;
      }
      const std::string & name = raw.topic->name;
      const auto selected = output_by_input.find(name);

      if (selected == output_by_input.end() && suppress.count(name) != 0) {
        continue;  // a replaced topic's messages are dropped, not copied
      }

      if (selected != output_by_input.end()) {
        TransformJob job;
        job.topic = name;
        job.timestamp_ns = raw.timestamp_ns;
        job.frozen = rd->freeze(raw);

        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.in_flight < ctx.max_in_flight || ctx.stop; });
        if (ctx.stop) {
          break;
        }
        job.seq = ctx.total_submitted++;
        ++ctx.in_flight;
        ctx.job_queue.push(std::move(job));
        lock.unlock();
        ctx.cv.notify_all();
      } else {
        TransformOutputItem item;
        item.topic = name;
        item.timestamp_ns = raw.timestamp_ns;
        item.frozen = rd->freeze(raw);

        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.in_flight < ctx.max_in_flight || ctx.stop; });
        if (ctx.stop) {
          break;
        }
        const std::size_t seq = ctx.total_submitted++;
        ++ctx.in_flight;
        ctx.completed.emplace(seq, std::move(item));
        lock.unlock();
        ctx.cv.notify_all();
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "%s: read error: %s", cmd, e.what());
    {
      std::lock_guard lock(ctx.mutex);
      ctx.reader_done = true;
      ctx.stop = true;
    }
    ctx.cv.notify_all();
    for (auto & t : workers) {
      t.join();
    }
    ctx.cv.notify_all();
    collector_thread.join();
    return 1;
  }

  {
    std::lock_guard lock(ctx.mutex);
    ctx.reader_done = true;
    ctx.stop = true;
  }
  ctx.cv.notify_all();

  for (auto & t : workers) {
    t.join();
  }

  ctx.cv.notify_all();
  collector_thread.join();

  return collector_status;
}

// Synchronous pass (num_threads <= 1): same declare + reopen + stream shape
// as run_parallel_transform_pass, but transforms each message inline on the
// reader thread. Output message order is trivially the bag's order.
int run_sync_transform_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & suppress,
  const std::unordered_map<std::string, std::string> & output_by_input,
  const std::vector<io::TopicInfo> & declare_outputs, const PcdTransformFn & transform,
  PcdTransformStats & stats, const char * logger, const char * cmd)
{
  for (const auto & t : topic_reader.topics()) {
    if (suppress.count(t.name) != 0) {
      continue;
    }
    writer.declare_topic(t);
  }
  for (const auto & out : declare_outputs) {
    writer.declare_topic(out);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  OutputDrainer drainer(writer, topic_reader, stats, logger, cmd);
  io::RawMessage raw;
  while (true) {
    bool got = false;
    try {
      got = rd->next(raw);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(logger, "%s: read error: %s", cmd, e.what());
      return 1;
    }
    if (!got) {
      break;
    }
    const std::string & name = raw.topic->name;
    const auto selected = output_by_input.find(name);

    if (selected == output_by_input.end()) {
      if (suppress.count(name) == 0) {
        writer.write(name, raw.timestamp_ns, raw.payload);
      }
      continue;
    }

    TransformJob job;
    job.topic = name;
    job.timestamp_ns = raw.timestamp_ns;
    job.frozen = rd->freeze(raw);
    TransformOutputItem item = process_transform_job(std::move(job), transform, selected->second);
    if (!drainer.drain(item)) {
      return 1;
    }
  }

  if (!io::close_writer_or_log(writer, logger)) {
    return 1;
  }
  return 0;
}

}  // namespace

std::optional<PcdTransformPlan> plan_pcd_transform(
  const io::BagReader & reader, const std::vector<std::string> & topics, const std::string & as,
  bool force, const std::string & input_type, const std::string & output_type,
  const std::function<std::string(const std::string &)> & default_name,
  const std::filesystem::path & input_path, const char * logger, const char * cmd)
{
  PcdTransformPlan plan;

  // ---- select the input topics -------------------------------------------
  if (topics.empty()) {
    for (const auto & t : reader.topics()) {
      if (t.type == input_type) {
        plan.mappings.emplace_back(&t, "");
      }
    }
    if (plan.mappings.empty()) {
      BAGWIZ_LOG_ERROR(
        logger, "%s: no %s topics in %s", cmd, input_type.c_str(), input_path.c_str());
      return std::nullopt;
    }
  } else {
    std::unordered_set<std::string> seen;
    for (const auto & name : topics) {
      if (!seen.insert(name).second) {
        continue;  // a repeated selector names the same topic twice
      }
      const io::TopicInfo * info = io::find_topic_or_log(reader, name, input_path, logger);
      if (info == nullptr) {
        return std::nullopt;
      }
      if (info->type != input_type) {
        BAGWIZ_LOG_ERROR(
          logger, "Topic '%s' is %s, expected %s", name.c_str(), info->type.c_str(),
          input_type.c_str());
        return std::nullopt;
      }
      plan.mappings.emplace_back(info, "");
    }
  }

  // ---- resolve output names ------------------------------------------------
  if (!as.empty()) {
    if (plan.mappings.size() != 1) {
      BAGWIZ_LOG_ERROR(
        logger, "%s: --as requires exactly one selected topic (got %zu)", cmd,
        plan.mappings.size());
      return std::nullopt;
    }
    plan.mappings.front().second = as;
  } else {
    for (auto & mapping : plan.mappings) {
      mapping.second = default_name(mapping.first->name);
      if (mapping.second.empty()) {
        BAGWIZ_LOG_ERROR(
          logger, "%s: cannot derive an output topic name from '%s'; pass --as to name it", cmd,
          mapping.first->name.c_str());
        return std::nullopt;
      }
    }
  }

  // ---- collisions -----------------------------------------------------------
  std::unordered_set<std::string> selected_names;
  for (const auto & [input_info, out_name] : plan.mappings) {
    selected_names.insert(input_info->name);
  }
  std::unordered_set<std::string> out_names;
  for (const auto & [input_info, out_name] : plan.mappings) {
    if (!out_names.insert(out_name).second) {
      BAGWIZ_LOG_ERROR(
        logger, "%s: two selected topics map to the same output topic '%s'", cmd, out_name.c_str());
      return std::nullopt;
    }
    if (out_name == input_info->name) {
      // The transform REPLACES the input topic, so the output must get a new
      // name: under the same name a pass-through message would force a second
      // declaration carrying the original type — one topic, two types.
      BAGWIZ_LOG_ERROR(
        logger, "%s: output topic '%s' is the selected input topic itself; name it with --as", cmd,
        out_name.c_str());
      return std::nullopt;
    }
    if (selected_names.count(out_name) != 0 && out_name != input_info->name) {
      BAGWIZ_LOG_ERROR(
        logger,
        "%s: output topic '%s' would overwrite another selected input topic; drop it from the "
        "selection or rename the output with --as",
        cmd, out_name.c_str());
      return std::nullopt;
    }
    if (io::find_topic(reader, out_name) != nullptr) {
      if (!force) {
        BAGWIZ_LOG_ERROR(
          logger, "Output topic '%s' already exists in %s; pass --force to replace it",
          out_name.c_str(), input_path.c_str());
        return std::nullopt;
      }
      plan.suppress.insert(out_name);
    }
  }

  // ---- output topic declarations --------------------------------------------
  // Resolve the output type's .msg definition once (cached per process) so
  // MCAP outputs stay self-describing. An unresolvable type is declared
  // without self-description, the declare_reader_topics convention.
  auto resolved = core::resolve_message_definition(output_type);
  if (resolved.text.empty()) {
    BAGWIZ_LOG_WARN(
      logger, "no .msg on disk for type '%s'; writing without self-description for this type",
      output_type.c_str());
  }
  for (const auto & [input_info, out_name] : plan.mappings) {
    plan.output_by_input.emplace(input_info->name, out_name);
    plan.suppress.insert(input_info->name);
    io::TopicInfo out;
    out.name = out_name;
    out.type = output_type;
    out.serialization_format = "cdr";
    out.offered_qos_profiles = input_info->offered_qos_profiles;
    out.schema_text = resolved.text;
    out.schema_encoding = resolved.encoding;
    plan.declare_outputs.push_back(std::move(out));
  }
  return plan;
}

int run_pcd_transform_pass(
  const io::WriterFactory & factory, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & suppress,
  const std::unordered_map<std::string, std::string> & output_by_input,
  const std::vector<io::TopicInfo> & declare_outputs, const PcdTransformFn & transform,
  int num_threads, PcdTransformStats & stats, const char * logger, const char * cmd)
{
  auto writer = io::open_write_or_log(factory, logger);
  if (!writer) {
    return 1;
  }
  if (num_threads <= 1) {
    return run_sync_transform_pass(
      *writer, topic_reader, input_path, suppress, output_by_input, declare_outputs, transform,
      stats, logger, cmd);
  }
  return run_parallel_transform_pass(
    *writer, topic_reader, input_path, suppress, output_by_input, declare_outputs, transform,
    num_threads, stats, logger, cmd);
}

}  // namespace bagwiz::commands
