// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "sqlite3_payload_sizes.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/io/sqlite3_helpers.hpp"

#include <sqlite3.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{

// Rowid slices below this many rows are not worth their own connection and
// statement, so a small bag ends up on a single serial slice.
constexpr std::int64_t kMinRowsPerSlice = 4096;

// Slices per worker. More slices than workers keeps the tail short when the
// rows in one rowid range happen to be denser than in another.
constexpr std::int64_t kSlicesPerWorker = 4;

// Half-open rowid range [begin_id, end_id).
struct RowidSlice
{
  std::int64_t begin_id = 0;
  std::int64_t end_id = 0;
};

SqlitePtr open_readonly(const std::filesystem::path & path)
{
  auto db =
    sqlite_open_or_throw(path.string(), SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX, "sqlite3 open");
  // Read-only scanning tuning, best-effort as in the streaming reader.
  sqlite3_exec(db.get(), "PRAGMA query_only = 1;", nullptr, nullptr, nullptr);
  sqlite3_exec(db.get(), "PRAGMA cache_size = -16384;", nullptr, nullptr, nullptr);
  return db;
}

// The rowid extent of the (optionally restricted) row set, or nullopt when it
// is empty. Both ends come off the rowid B-tree in O(1).
std::optional<RowidSlice> rowid_extent(sqlite3 * db, const std::string & topic_clause)
{
  std::string sql = "SELECT MIN(id), MAX(id) FROM messages";
  if (!topic_clause.empty()) {
    sql += " WHERE " + topic_clause;
  }
  auto stmt = sqlite_prepare_or_throw(db, sql.c_str());
  if (sqlite3_step(stmt.get()) != SQLITE_ROW || sqlite3_column_type(stmt.get(), 0) == SQLITE_NULL) {
    return std::nullopt;
  }
  const std::int64_t max_id = sqlite3_column_int64(stmt.get(), 1);
  if (max_id == std::numeric_limits<std::int64_t>::max()) {
    // The half-open end below would wrap, and the slices would then cover no
    // rows at all. No rosbag2 writer produces such a rowid; refuse rather than
    // report an empty bag.
    throw std::runtime_error("messages.id reaches INT64_MAX");
  }
  return RowidSlice{sqlite3_column_int64(stmt.get(), 0), max_id + 1};
}

// Split `extent` into contiguous rowid ranges, uniform in rowid. Returns a
// single slice whenever splitting would not pay for itself.
std::vector<RowidSlice> build_slices(const RowidSlice & extent, int num_threads)
{
  const std::int64_t span = extent.end_id - extent.begin_id;
  const std::int64_t by_workers = std::max<std::int64_t>(num_threads, 1) * kSlicesPerWorker;
  const std::int64_t count =
    std::clamp<std::int64_t>(std::min(by_workers, span / kMinRowsPerSlice), 1, by_workers);

  // Spread the remainder over the leading slices instead of scaling `span` by
  // the slice index, which would overflow on a pathological rowid range. The
  // boundaries stay contiguous and the last one ends exactly at end_id.
  const std::int64_t per_slice = span / count;
  const std::int64_t remainder = span % count;
  const auto boundary = [&](std::int64_t i) {
    return extent.begin_id + (per_slice * i) + std::min(i, remainder);
  };

  std::vector<RowidSlice> slices;
  slices.reserve(static_cast<std::size_t>(count));
  for (std::int64_t i = 0; i < count; ++i) {
    slices.push_back({boundary(i), boundary(i + 1)});
  }
  return slices;
}

std::string slice_sql(const RowidSlice & slice, const std::string & topic_clause)
{
  // NOT INDEXED pins the plan to the rowid range scan this function depends
  // on. `id` is the table's INTEGER PRIMARY KEY, so the range is still served
  // off the rowid B-tree; what the hint rules out is the planner reaching for
  // the (topic_id, timestamp) index older bagwiz versions wrote, which would
  // turn a sequential range into one row lookup per index entry.
  std::string sql = "SELECT topic_id, LENGTH(data) FROM messages NOT INDEXED WHERE id >= " +
                    std::to_string(slice.begin_id) + " AND id < " + std::to_string(slice.end_id);
  if (!topic_clause.empty()) {
    sql += " AND " + topic_clause;
  }
  return sql;
}

// Sum one slice into `sizes`. Returns an error string, empty on success.
std::string scan_slice(
  sqlite3 * db, const RowidSlice & slice, const std::string & topic_clause,
  std::unordered_map<std::int64_t, std::uint64_t> & sizes)
{
  auto stmt = sqlite_prepare_or_throw(db, slice_sql(slice, topic_clause).c_str());
  for (;;) {
    const int rc = sqlite3_step(stmt.get());
    if (rc == SQLITE_DONE) {
      return {};
    }
    if (rc != SQLITE_ROW) {
      return "payload length scan failed: " + sqlite_errmsg(db);
    }
    sizes[sqlite3_column_int64(stmt.get(), 0)] +=
      static_cast<std::uint64_t>(sqlite3_column_int64(stmt.get(), 1));
  }
}

}  // namespace

TopicPayloadSizes sum_payload_lengths(
  const std::filesystem::path & path, const std::string & topic_clause, int num_threads)
{
  TopicPayloadSizes result;
  std::vector<RowidSlice> slices;
  try {
    auto db = open_readonly(path);
    const auto extent = rowid_extent(db.get(), topic_clause);
    if (!extent) {
      return result;  // no matching rows: every topic is absent, i.e. zero
    }
    slices = build_slices(*extent, num_threads);
  } catch (const std::exception & e) {
    result.error = e.what();
    return result;
  }

  const std::size_t workers = std::min<std::size_t>(std::max(num_threads, 1), slices.size());
  std::vector<std::unordered_map<std::int64_t, std::uint64_t>> partial_sizes(workers);
  std::vector<std::string> partial_errors(workers);
  std::atomic<std::size_t> next_slice{0};
  std::atomic<bool> failed{false};

  const auto worker = [&](std::size_t slot) {
    // jthread bodies must not throw; a failed open or prepare has to surface
    // as this slot's error instead of terminating the process.
    try {
      auto db = open_readonly(path);
      for (;;) {
        const std::size_t i = next_slice.fetch_add(1);
        if (i >= slices.size() || failed.load()) {
          return;
        }
        auto error = scan_slice(db.get(), slices[i], topic_clause, partial_sizes[slot]);
        if (!error.empty()) {
          partial_errors[slot] = std::move(error);
          failed.store(true);
          return;
        }
      }
    } catch (const std::exception & e) {
      partial_errors[slot] = std::string("payload length scan failed: ") + e.what();
      failed.store(true);
    }
  };

  {
    std::vector<std::jthread> pool;
    pool.reserve(workers - 1);
    for (std::size_t w = 1; w < workers; ++w) {
      pool.emplace_back([&worker, w]() { worker(w); });
    }
    worker(0);
  }

  for (const auto & error : partial_errors) {
    if (!error.empty()) {
      result.error = error;
      return result;
    }
  }
  for (const auto & partial : partial_sizes) {
    // cppcheck-suppress unassignedVariable
    for (const auto & [topic_id, bytes] : partial) {
      result.per_topic_id[topic_id] += bytes;
    }
  }
  return result;
}

}  // namespace bagwiz::io::detail
