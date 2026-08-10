// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_INDEXED_STREAM_HPP_
#define IO__MCAP_INDEXED_STREAM_HPP_

#include "mcap_chunk_prefetch.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "mcap_read_job_compat.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/reader.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Src-local parallel replacement for the LogTimeOrder indexed read path:
// replicates mcap::IndexedMessageReader's emission order exactly (the same
// mcap_compat::ReadJobQueue discipline over the same job keys) while the
// chunk decompression itself runs ahead on a ChunkPrefetcher worker pool
// instead of synchronously on the iterating thread.
namespace bagwiz::io::detail
{

class ParallelIndexedStream
{
public:
  struct Options
  {
    std::optional<std::uint64_t> start_ns;  // inclusive, like ReadMessageOptions::startTime
    std::optional<std::uint64_t> end_ns;    // exclusive, like ReadMessageOptions::endTime
    std::function<bool(std::string_view)> topic_filter;  // null => all topics
    int num_threads = 8;                                 // decompress workers
  };

  // One emitted message. `payload` points into the retained decompressed
  // chunk buffer and stays valid until the next next() call.
  struct Message
  {
    std::uint16_t channel_id = 0;
    std::uint64_t log_time = 0;
    std::span<const std::byte> payload;
  };

  // True when every chunk of `reader`'s summary can be served by this stream
  // (zstd or uncompressed chunks only — lz4 bags use the fallback path).
  [[nodiscard]] static bool supported(const mcap::McapReader & reader);

  // `reader` must have a loaded summary with chunk indexes; it is only used
  // for its summary (channels + chunk indexes) — file I/O runs on the
  // prefetcher's own handles.
  ParallelIndexedStream(
    const std::filesystem::path & path, const mcap::McapReader & reader, Options options);

  // Emits the next message in exactly the order mcap's IndexedMessageReader
  // (LogTimeOrder) would. Returns false at end of stream or on error; a
  // non-empty error() distinguishes the latter.
  [[nodiscard]] bool next(Message & out);

  [[nodiscard]] const std::string & error() const { return error_; }

  // Shared ownership of the chunk buffer backing the payload emitted by the
  // most recent successful next() call; null before the first emission.
  // BagReader::freeze() uses this to alias the buffer instead of copying the
  // payload.
  [[nodiscard]] std::shared_ptr<const void> last_payload_owner() const { return last_owner_; }

private:
  // One retained decompressed chunk. The slot is reused once all its messages
  // were consumed AND a later chunk claims the slot (so the last emitted
  // payload stays valid until the next next() call, per the contract). The
  // buffer is shared: freezing a message out of this slot extends the
  // buffer's life past eviction, and the last owner returns it to the
  // prefetcher's pool (see ChunkBufferPool::share).
  struct Slot
  {
    std::shared_ptr<std::vector<std::byte>> records;
    std::size_t offset = 0;  // records blob start within `records`
    std::size_t size = 0;    // records blob size in bytes
    std::uint64_t chunk_start_offset = 0;
    std::size_t unread = 0;

    [[nodiscard]] const std::byte * data() const { return records->data() + offset; }
  };

  [[nodiscard]] std::size_t find_free_slot();
  void ingest_chunk(const mcap_compat::DecompressChunkJob & job);

  Options options_;
  std::unordered_set<std::uint16_t> selected_channels_;  // empty => no filter
  bool has_filter_ = false;
  mcap_compat::ReadJobQueue queue_{false};
  std::vector<Slot> slots_;
  std::unordered_map<std::uint64_t, std::size_t> schedule_index_by_offset_;
  // Declared before prefetcher_ so the pool outlives the workers that draw
  // buffers from it (members destruct in reverse declaration order).
  std::shared_ptr<ChunkBufferPool> pool_ = std::make_shared<ChunkBufferPool>();
  std::unique_ptr<ChunkPrefetcher> prefetcher_;
  std::shared_ptr<const void> last_owner_;  // chunk buffer of the last emission
  std::string error_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_INDEXED_STREAM_HPP_
