// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__SQLITE3_PAYLOAD_SIZES_HPP_
#define IO__SQLITE3_PAYLOAD_SIZES_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>

// Src-local building block of `bagwiz du`: per-topic totals of the bytes each
// row's `messages.data` BLOB occupies — a plain payload or a MESSAGE-mode
// zstd frame alike — read out of one .db3's row headers instead of out of the
// BLOBs themselves.
namespace bagwiz::io::detail
{

// Per-topic-id sum of stored `messages.data` BLOB bytes. Meaningful only when
// `error` is empty.
struct TopicPayloadSizes
{
  std::unordered_map<std::int64_t, std::uint64_t> per_topic_id;
  std::string error;
};

// Sum each topic's `messages.data` lengths without reading a single payload.
//
// SQLite answers LENGTH() on a BLOB column from the row header alone and never
// visits the overflow pages the payload itself lives on, so this touches only
// the table's leaf pages — under a tenth of a payload-heavy bag. The sum must
// stay out of SQL: `SUM(LENGTH(data)) ... GROUP BY topic_id` groups through a
// temp B-tree whose sorter records carry `data`, which materializes every
// payload and reads the whole file (measured 66s versus 0.2s on a 12 GiB bag).
//
// Those leaf pages are scattered across the file, so the scan is latency-bound
// rather than bandwidth-bound and splits well: `num_threads` workers, each with
// its own read-only connection, take disjoint rowid ranges. `num_threads` <= 1,
// or a bag too small to be worth splitting, scans serially on the caller's
// thread.
//
// `topic_clause` is an optional SQL restriction on the rows to sum, e.g.
// "topic_id IN (1,2)"; empty sums every topic.
TopicPayloadSizes sum_payload_lengths(
  const std::filesystem::path & path, const std::string & topic_clause, int num_threads);

}  // namespace bagwiz::io::detail

#endif  // IO__SQLITE3_PAYLOAD_SIZES_HPP_
