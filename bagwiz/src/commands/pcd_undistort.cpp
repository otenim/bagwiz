// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_undistort.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/duration_parse.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pipeline/stage_profiler.hpp"
#include "bagwiz/core/pointcloud/deskew.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <span>
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

constexpr const char * kLogger = "bagwiz.cmd.pcd";
// BAGWIZ_PROFILE report label, shared by the sync and parallel passes.
constexpr const char * kProfileLabel = "pcd_undistort";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr const char * kDefaultRefFrame = "map";
constexpr const char * kDefaultOfFrame = "base_link";

// One PointCloud2 message handed off from the bag reader to a worker thread.
// `frozen` shares the reader's chunk backing (BagReader::freeze) instead of
// copying the bytes; it pins that whole chunk while the job is in flight, so
// peak pinned memory is bounded by ParallelContext::max_in_flight chunks.
struct DeskewJob
{
  std::size_t seq;
  std::string topic;
  std::int64_t timestamp_ns;
  io::FrozenMessage frozen;
  std::optional<geometry_msgs::msg::Transform> extrinsic;
};

// One output message waiting in the in-order completion map for the collector.
// `frozen` is either the frozen input bytes (copy-through and undecodable
// pass-through messages) or a freshly serialized cloud the item owns; the
// collector writes both the same way.
struct OutputItem
{
  std::string topic;
  std::int64_t timestamp_ns;
  io::FrozenMessage frozen;
  std::uint64_t in_bytes = 0;  // input payload size, for the profiler's byte counters
  std::optional<std::string> error;
};

// Shared state for the parallel reader / worker pool / collector pipeline.
struct ParallelContext
{
  std::mutex mutex;
  std::condition_variable cv;
  std::queue<DeskewJob> job_queue;
  std::map<std::size_t, OutputItem> completed;
  std::size_t next_output_seq = 0;
  std::size_t total_submitted = 0;
  std::size_t in_flight = 0;
  std::size_t max_in_flight = 0;
  bool stop = false;
  bool reader_done = false;
};

// Warn about endpoint clamping in a deskewed cloud. A clamped point (or
// reference stamp) is deskewed against a pose at a different time than its
// own — up to no correction at all — while its per-point time is still
// rewritten to the reference value, so this must never be silent.
void log_out_of_span_warnings(const core::pointcloud::DeskewCdrResult & res, const char * topic)
{
  if (res.ref_out_of_span) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "pcd undistort: cloud on '%s': header.stamp is outside the motion trajectory's time "
      "span; deskewed against the clamped endpoint pose",
      topic);
  }
  if (res.points_out_of_span > 0) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "pcd undistort: cloud on '%s': %" PRIu64
      " point(s) fell outside the motion trajectory's time span; their poses were clamped "
      "to the trajectory endpoints",
      topic, res.points_out_of_span);
  }
}

// Deskew a single cloud by patching one owned copy of its CDR payload in
// place — no parse-side data copy and no re-serialize. Runs on a worker
// thread and only touches local state plus the read-only trajectory span.
OutputItem process_deskew_job(DeskewJob job, std::span<const core::TrajectoryPose> trajectory)
{
  OutputItem item;
  item.topic = job.topic;
  item.timestamp_ns = job.timestamp_ns;
  item.in_bytes = static_cast<std::uint64_t>(job.frozen.payload.size());

  try {
    // The one copy this path still makes: the frozen payload may alias the
    // reader's chunk buffer, which is shared with the prefetcher and the other
    // messages in that chunk, so patching it in place is not an option.
    std::vector<std::byte> patched(job.frozen.payload.begin(), job.frozen.payload.end());
    const auto res = core::pointcloud::deskew_pointcloud2_cdr(
      std::span<std::byte>(patched.data(), patched.size()), trajectory, job.extrinsic);
    if (!res.parse_error.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "pcd undistort: skipping undecodable cloud on '%s': %s", job.topic.c_str(),
        res.parse_error.c_str());
      item.frozen = std::move(job.frozen);
    } else if (!res.error.empty()) {
      item.error = res.error;
    } else {
      if (res.points_deskewed == 0 && res.points_total > 0) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "pcd undistort: cloud on '%s' had nothing deskewed of %" PRIu64
          " point(s) (no_time=%" PRIu64 ", no_pose=%" PRIu64 ", nonfinite=%" PRIu64
          "); passed through un-deskewed",
          job.topic.c_str(), res.points_total, res.points_no_time, res.points_no_pose,
          res.points_nonfinite);
      }
      log_out_of_span_warnings(res, job.topic.c_str());
      item.frozen = io::own_payload(std::move(patched));
    }
  } catch (const std::exception & e) {
    item.error = std::string("exception: ") + e.what();
  }
  return item;
}

// Parallel version of Pass 2.  One reader thread, one collector thread that
// alone calls writer.write(), and a fixed-size std::jthread worker pool that
// deskews PointCloud2 messages.  Non-pcd messages bypass the job queue and go
// straight into the in-order completion map.  Output message order and bytes
// are identical to the synchronous path; note the summary cloud count is not
// (this path counts every submitted `--pcd` message, including undecodable
// ones).  Reports per-stage timings on success when BAGWIZ_PROFILE is enabled.
int run_parallel_undistort_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & pcd_set,
  const ExtrinsicMap & extrinsics, std::span<const core::TrajectoryPose> trajectory,
  int num_threads, std::uint64_t & total_clouds)
{
  for (const auto & t : topic_reader.topics()) {
    writer.declare_topic(t);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  // StageProfiler has no internal locking, so each thread accumulates into
  // its own instance: this reader thread times kRead, each worker times
  // kProcess, and the collector times kWrite. The collector's profiler also
  // owns every message/byte counter -- it is the one thread that sees each
  // output message exactly once. The totals are merged after the joins into
  // a single report.
  const bool profiling = core::pipeline::profile_value_enabled(std::getenv("BAGWIZ_PROFILE"));
  core::pipeline::StageProfiler read_prof(profiling);
  core::pipeline::StageProfiler write_prof(profiling);
  std::vector<core::pipeline::StageProfiler> worker_profs;
  worker_profs.reserve(static_cast<std::size_t>(num_threads));
  for (int i = 0; i < num_threads; ++i) {
    worker_profs.emplace_back(profiling);
  }
  const auto started_at = std::chrono::steady_clock::now();

  ParallelContext ctx;
  ctx.max_in_flight = static_cast<std::size_t>(num_threads) * 3;

  int collector_status = 0;

  auto worker = [&](core::pipeline::StageProfiler & prof) {
    while (true) {
      DeskewJob job;
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
      OutputItem item;
      {
        auto s = prof.time(core::pipeline::Stage::kProcess);
        item = process_deskew_job(std::move(job), trajectory);
      }

      {
        std::lock_guard lock(ctx.mutex);
        ctx.completed.emplace(seq, std::move(item));
      }
      ctx.cv.notify_all();
    }
  };

  auto collector = [&]() {
    try {
      while (true) {
        OutputItem item;
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

        if (item.error.has_value()) {
          BAGWIZ_LOG_ERROR(
            kLogger, "pcd undistort: deskew failed on '%s': %s", item.topic.c_str(),
            item.error->c_str());
          collector_status = 1;
          {
            std::lock_guard lock(ctx.mutex);
            ctx.stop = true;
          }
          ctx.cv.notify_all();
          try {
            writer.close();
          } catch (...) {
            // A writer close error is secondary to the deskew error already reported.
          }
          return;
        }

        {
          auto s = write_prof.time(core::pipeline::Stage::kWrite);
          writer.write(item.topic, item.timestamp_ns, item.frozen.payload);
        }
        write_prof.add_message(
          item.in_bytes, static_cast<std::uint64_t>(item.frozen.payload.size()));
      }

      // close() drains the writer's remaining compression backlog, so its
      // time belongs to the write stage.
      bool closed = false;
      {
        auto s = write_prof.time(core::pipeline::Stage::kWrite);
        closed = io::close_writer_or_log(writer, kLogger);
      }
      if (!closed) {
        collector_status = 1;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: collector error: %s", e.what());
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
    workers.emplace_back(worker, std::ref(worker_profs[static_cast<std::size_t>(i)]));
  }
  std::jthread collector_thread(collector);

  io::RawMessage raw;
  try {
    while (true) {
      bool got = false;
      {
        auto s = read_prof.time(core::pipeline::Stage::kRead);
        got = rd->next(raw);
      }
      if (!got) {
        break;
      }
      const std::string & name = raw.topic->name;
      const bool is_pcd = pcd_set.count(name) != 0;

      if (is_pcd) {
        DeskewJob job;
        job.topic = name;
        job.timestamp_ns = raw.timestamp_ns;
        {
          // freeze() shares the reader's chunk backing instead of copying the
          // payload; it stays on this reader thread's critical path, so it is
          // charged to the read stage.
          auto s = read_prof.time(core::pipeline::Stage::kRead);
          job.frozen = rd->freeze(raw);
        }
        job.extrinsic = extrinsics.at(name);

        std::unique_lock lock(ctx.mutex);
        ctx.cv.wait(lock, [&] { return ctx.in_flight < ctx.max_in_flight || ctx.stop; });
        if (ctx.stop) {
          break;
        }
        job.seq = ctx.total_submitted++;
        ++ctx.in_flight;
        ctx.job_queue.push(std::move(job));
        ++total_clouds;
        lock.unlock();
        ctx.cv.notify_all();
      } else {
        // Copy-through message: freeze the reader's chunk backing and hand it
        // to the collector through the in-order completion map. The frozen
        // payload keeps the bytes alive until the collector writes and drops
        // the item, so neither a payload copy nor the reader-blocking
        // rendezvous this used to need is required.
        OutputItem item;
        item.topic = name;
        item.timestamp_ns = raw.timestamp_ns;
        {
          // Charged to the read stage for the same reason as the pcd freeze
          // above.
          auto s = read_prof.time(core::pipeline::Stage::kRead);
          item.frozen = rd->freeze(raw);
        }
        item.in_bytes = static_cast<std::uint64_t>(item.frozen.payload.size());

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
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: read error: %s", e.what());
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

  if (collector_status == 0) {
    // Merge the reader's and workers' timings into the collector's profiler
    // (which holds the message/byte counters) for one report. A failed pass
    // aborts the in-place swap, so its truncated timings are not reported.
    write_prof.add(
      core::pipeline::Stage::kRead, std::chrono::nanoseconds(read_prof.totals().read_ns));
    for (const auto & wp : worker_profs) {
      write_prof.add(
        core::pipeline::Stage::kProcess, std::chrono::nanoseconds(wp.totals().process_ns));
    }
    write_prof.set_elapsed(std::chrono::steady_clock::now() - started_at);
    write_prof.report(kProfileLabel);
  }

  return collector_status;
}

// Synchronous version of Pass 2 (num_threads <= 1): same declare + reopen +
// stream shape as run_parallel_undistort_pass, but deskews each cloud inline
// on the reader thread. Output message order is trivially the bag's order.
// Reports per-stage timings on success when BAGWIZ_PROFILE is enabled.
int run_sync_undistort_pass(
  io::BagWriter & writer, const io::BagReader & topic_reader,
  const std::filesystem::path & input_path, const std::unordered_set<std::string> & pcd_set,
  const ExtrinsicMap & extrinsics, std::span<const core::TrajectoryPose> trajectory,
  std::uint64_t & total_clouds)
{
  for (const auto & t : topic_reader.topics()) {
    writer.declare_topic(t);
  }

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  core::pipeline::StageProfiler prof;
  const auto started_at = std::chrono::steady_clock::now();

  std::vector<std::byte> patch_buf;  // reused across clouds
  io::RawMessage raw;
  while (true) {
    bool got = false;
    {
      auto s = prof.time(core::pipeline::Stage::kRead);
      got = rd->next(raw);
    }
    if (!got) {
      break;
    }
    const std::string & name = raw.topic->name;
    const auto in_size = static_cast<std::uint64_t>(raw.payload.size());
    if (pcd_set.count(name) == 0) {
      {
        auto s = prof.time(core::pipeline::Stage::kWrite);
        writer.write(name, raw.timestamp_ns, raw.payload);
      }
      prof.add_message(in_size, in_size);
      continue;
    }
    // Deskew by patching one owned copy of the CDR payload in place — no
    // parse-side data copy and no re-serialize.
    const auto res = [&] {
      auto s = prof.time(core::pipeline::Stage::kProcess);
      patch_buf.assign(raw.payload.begin(), raw.payload.end());
      return core::pointcloud::deskew_pointcloud2_cdr(
        std::span<std::byte>(patch_buf.data(), patch_buf.size()), trajectory, extrinsics.at(name));
    }();
    if (!res.parse_error.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "pcd undistort: skipping undecodable cloud on '%s': %s", name.c_str(),
        res.parse_error.c_str());
      {
        auto s = prof.time(core::pipeline::Stage::kWrite);
        writer.write(name, raw.timestamp_ns, raw.payload);
      }
      prof.add_message(in_size, in_size);
      continue;
    }
    if (!res.error.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: deskew failed on '%s': %s", name.c_str(), res.error.c_str());
      return 1;
    }
    // The upfront per-topic check only guarantees the FIRST cloud on this
    // topic had a usable time field; a heterogeneous stream could still
    // hand deskew_pointcloud2 a later cloud that ends up moving nothing
    // (no usable time/pose/finite xyz on any point). ok() is still true —
    // the cloud passes through verbatim by design — but that must not be
    // silent, or a bug upstream of this topic could go unnoticed.
    if (res.points_deskewed == 0 && res.points_total > 0) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "pcd undistort: cloud on '%s' had nothing deskewed of %" PRIu64
        " point(s) (no_time=%" PRIu64 ", no_pose=%" PRIu64 ", nonfinite=%" PRIu64
        "); passed through un-deskewed",
        name.c_str(), res.points_total, res.points_no_time, res.points_no_pose,
        res.points_nonfinite);
    }
    log_out_of_span_warnings(res, name.c_str());
    {
      auto s = prof.time(core::pipeline::Stage::kWrite);
      writer.write(name, raw.timestamp_ns, patch_buf);
    }
    prof.add_message(in_size, static_cast<std::uint64_t>(patch_buf.size()));
    ++total_clouds;
  }

  // close() drains the writer's remaining compression backlog, so its time
  // belongs to the write stage.
  bool closed = false;
  {
    auto s = prof.time(core::pipeline::Stage::kWrite);
    closed = io::close_writer_or_log(writer, kLogger);
  }
  if (!closed) {
    return 1;
  }
  prof.set_elapsed(std::chrono::steady_clock::now() - started_at);
  prof.report(kProfileLabel);
  return 0;
}

// Run Pass 2 through the shared -o vs in-place rewrite dispatch, picking the
// sync or parallel pass by thread count. Unlike the other rewrite commands,
// pcd undistort keeps the storage default (zstd) for MCAP compression rather
// than forcing "none" — or forwards the user's --compression choice.
int dispatch_undistort_pass(
  const PcdUndistortArgs & args, const io::BagReader & topic_reader,
  const std::unordered_set<std::string> & pcd_set, const ExtrinsicMap & extrinsics,
  std::span<const core::TrajectoryPose> trajectory, int num_threads, std::uint64_t & total_clouds)
{
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "pcd undistort: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "pcd undistort: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  rewrite_opts.mcap_compression = args.compression.value_or("");
  rewrite_opts.mcap_compression_level = args.compression_level.value_or("");
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) {
      auto writer = io::open_write_or_log(factory, kLogger);
      if (!writer) {
        return 1;
      }
      if (num_threads <= 1) {
        return run_sync_undistort_pass(
          *writer, topic_reader, args.input_path, pcd_set, extrinsics, trajectory, total_clouds);
      }
      return run_parallel_undistort_pass(
        *writer, topic_reader, args.input_path, pcd_set, extrinsics, trajectory, num_threads,
        total_clouds);
    });
}

}  // namespace

int run_pcd_undistort(const PcdUndistortArgs & args)
{
  // ---- validate arguments + topics ------------------------------------------
  if (args.pcd_topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: --pcd needs at least 1 topic");
    return 1;
  }
  // CLI11 enforces --pose/--twist are not both given (excludes); the runner
  // enforces the other half of the xor.
  const bool motion_is_twist = !args.twist_topic.empty();
  if (!motion_is_twist && args.pose_topic.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "pcd undistort: exactly one of --pose / --twist is required");
    return 1;
  }
  const std::string & motion_topic = motion_is_twist ? args.twist_topic : args.pose_topic;
  const std::string ref = args.ref_frame.value_or(kDefaultRefFrame);
  const std::string of = args.of_frame.value_or(kDefaultOfFrame);
  if (motion_is_twist && args.ref_frame.has_value()) {
    // A twist source only carries relative motion, so the frame the trajectory
    // is expressed in never enters the deskew math.
    BAGWIZ_LOG_WARN(kLogger, "pcd undistort: --ref has no effect with --twist; ignoring it");
  }
  std::int64_t max_extrap_ns = 1'000'000'000;  // --max-extrap-duration default: 1s
  if (args.max_extrap_duration.has_value()) {
    const auto dur = core::parse_duration_ns(*args.max_extrap_duration);
    if (!dur.has_value() || *dur < 0) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "pcd undistort: --max-extrap-duration '%s' is not a valid non-negative duration "
        "(ns/us/ms/s, no unit = ms; e.g. 500ms)",
        args.max_extrap_duration->c_str());
      return 1;
    }
    max_extrap_ns = *dur;
  }
  // CLI11 already restricts the flag values, but the runner is also a direct
  // API entry (tests, future callers), so validate here too and fail fast.
  if (args.compression.has_value()) {
    const std::string & c = *args.compression;
    if (c != "zstd" && c != "lz4" && c != "none") {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: --compression '%s' is not one of zstd, lz4, none", c.c_str());
      return 1;
    }
  }
  if (args.compression_level.has_value()) {
    const std::string & l = *args.compression_level;
    if (l != "fastest" && l != "fast" && l != "default" && l != "slow" && l != "slowest") {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "pcd undistort: --compression-level '%s' is not one of fastest, fast, default, slow, "
        "slowest",
        l.c_str());
      return 1;
    }
    if (args.compression.has_value() && *args.compression == "none") {
      BAGWIZ_LOG_ERROR(
        kLogger, "pcd undistort: --compression-level has no effect with --compression none");
      return 1;
    }
  }
  if (args.compression.has_value() || args.compression_level.has_value()) {
    // The flags configure mcap chunk compression; reject them when the
    // output resolves to another storage format. An undetectable in-place
    // input falls through to the dispatch's own format error instead.
    const io::Format out_format = args.output_path.has_value()
                                    ? io::resolve_write_layout(
                                        *args.output_path, io::create_options_inheriting_format(
                                                             args.input_path, *args.output_path))
                                        .format
                                    : io::detect_format(args.input_path);
    if (out_format == io::Format::Sqlite3) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "pcd undistort: --compression/--compression-level apply only to mcap outputs; this "
        "output is SQLite3 (.db3)");
      return 1;
    }
  }

  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();
  const io::TopicInfo * motion_ti = validate_undistort_topics(
    *reader, motion_topic, motion_is_twist, args.pcd_topics, args.input_path, kLogger);
  if (motion_ti == nullptr) {
    return 1;
  }

  // ---- Pass 1: static TF + the --of -> --ref trajectory ---------------------
  tf2::BufferCore buffer{kTfBufferCacheTime};
  auto built = build_sorted_of_ref_trajectory(
    args.input_path, *motion_ti, ref, of, motion_is_twist, buffer, kLogger);
  if (!built.ok()) {
    return 1;
  }
  std::vector<core::TrajectoryPose> trajectory = std::move(built.trajectory);

  // ---- peek + validate each --pcd topic's first cloud, then extrinsics ------
  const auto pcd_state = peek_pcd_topic_states(args.input_path, args.pcd_topics, kLogger);
  if (!pcd_state.has_value() || !validate_pcd_topic_states(args.pcd_topics, *pcd_state, kLogger)) {
    return 1;
  }

  // ---- extend the trajectory over clouds outside its time span ---------------
  // The trajectory only spans the motion source's first..last sample, and
  // deskew clamps out-of-span lookups to the endpoint poses — for a cloud
  // wholly before the first sample that silently means no correction at all
  // (ref and points clamp to the same pose). Extrapolate the endpoints at
  // constant velocity to cover the --pcd clouds instead, but refuse gaps
  // beyond --max-extrap-duration: constant-velocity extrapolation is only
  // trustworthy near the endpoint. --no-extrap (or a 0 duration) keeps the
  // old clamping behaviour; the out-of-span warnings then flag the affected
  // clouds during Pass 2.
  if (!args.no_extrap && max_extrap_ns > 0) {
    std::optional<std::int64_t> needed_start;
    std::int64_t max_sweep_ns = 0;
    for (const auto & topic : args.pcd_topics) {
      const auto & span = pcd_state->at(topic).time_span;
      if (!span.has_value()) {
        continue;
      }
      if (!needed_start.has_value() || span->min_ns < *needed_start) {
        needed_start = span->min_ns;
      }
      max_sweep_ns = std::max(max_sweep_ns, span->max_ns - span->min_ns);
    }
    if (needed_start.has_value()) {
      // The tail must additionally cover the last clouds' sweeps, whose stamps
      // are unknown without a full bag scan; padding by the longest first-cloud
      // sweep covers the common case, and the out-of-span warnings catch the
      // rest.
      const std::int64_t needed_end = trajectory.back().timestamp_ns + max_sweep_ns;
      const std::int64_t head_ns = trajectory.front().timestamp_ns - *needed_start;
      const std::int64_t tail_ns = needed_end - trajectory.back().timestamp_ns;
      if (head_ns > max_extrap_ns) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "pcd undistort: the earliest --pcd cloud starts %.3f s before the motion "
          "trajectory's first sample, exceeding --max-extrap-duration (%.3f s); raise "
          "--max-extrap-duration to extrapolate that far, or pass --no-extrap to deskew "
          "against the clamped endpoint poses instead",
          head_ns / 1.0e9, max_extrap_ns / 1.0e9);
        return 1;
      }
      if (tail_ns > max_extrap_ns) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "pcd undistort: covering one sweep past the motion trajectory's last sample "
          "needs %.3f s of extrapolation, exceeding --max-extrap-duration (%.3f s); raise "
          "--max-extrap-duration to extrapolate that far, or pass --no-extrap to deskew "
          "against the clamped endpoint poses instead",
          tail_ns / 1.0e9, max_extrap_ns / 1.0e9);
        return 1;
      }
      core::extend_trajectory_to_span(trajectory, *needed_start, needed_end);
      if (head_ns > 0) {
        BAGWIZ_LOG_INFO(
          kLogger,
          "pcd undistort: extrapolated the motion trajectory %.1f ms before its first "
          "sample to cover the earliest --pcd cloud",
          head_ns / 1.0e6);
      }
      if (tail_ns > 0) {
        BAGWIZ_LOG_INFO(
          kLogger,
          "pcd undistort: extrapolated the motion trajectory %.1f ms past its last sample "
          "to cover the latest --pcd clouds' sweeps",
          tail_ns / 1.0e6);
      }
    }
  }

  const auto extrinsics = resolve_pcd_extrinsics(buffer, of, args.pcd_topics, *pcd_state, kLogger);
  if (!extrinsics.has_value()) {
    return 1;
  }

  // ---- Pass 2: copy-through + deskew (-o vs in-place shared dispatch) -------
  const std::unordered_set<std::string> pcd_set(args.pcd_topics.begin(), args.pcd_topics.end());
  std::uint64_t total_clouds = 0;
  const int num_threads =
    resolve_num_threads(args.threads.value_or(8), std::thread::hardware_concurrency());
  const int status = dispatch_undistort_pass(
    args, *reader, pcd_set, *extrinsics, trajectory, num_threads, total_clouds);
  if (status != 0) {
    return status;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "pcd undistort: deskewed %" PRIu64 " cloud(s) across %zu topic(s)", total_clouds,
    args.pcd_topics.size());
  return 0;
}

}  // namespace bagwiz::commands
