// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_CHUNK_PREFETCH_HPP_
#define IO__MCAP_CHUNK_PREFETCH_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Src-local building block of the parallel indexed mcap read path: a small
// worker pool that reads and decompresses one mcap file's chunk records ahead
// of the consumer, in a fixed schedule order, with a bounded lookahead window.
namespace bagwiz::io::detail
{

// Thread-safe pool of multi-MiB chunk buffers, shared between a
// ChunkPrefetcher's workers and the ParallelIndexedStream consuming them.
// Held via shared_ptr so that buffers frozen out of the stream (see
// BagReader::freeze) can return themselves to the pool when their last owner
// lets go — or free themselves normally when the pool is already gone, which
// is what lets a FrozenMessage outlive the reader.
class ChunkBufferPool : public std::enable_shared_from_this<ChunkBufferPool>
{
public:
  // Pop a pooled buffer (empty vector when the pool is dry).
  [[nodiscard]] std::vector<std::byte> take();

  // Return a buffer for reuse by the workers. Keeps the steady state free of
  // large allocations (and of the page faults + zero-fill that come with
  // fresh multi-MB buffers).
  void recycle(std::vector<std::byte> && buf);

  // Move `buf` into a shared_ptr whose deleter returns the buffer to this
  // pool when the last shared owner is released (and simply frees it once
  // the pool is gone).
  [[nodiscard]] std::shared_ptr<std::vector<std::byte>> share(std::vector<std::byte> && buf);

private:
  std::mutex mutex_;
  std::vector<std::vector<std::byte>> buffers_;
};

// One chunk record to prefetch, straight from the bag's ChunkIndex summary.
struct ChunkRef
{
  std::uint64_t start_offset = 0;  // file offset of the chunk record
  std::uint64_t length = 0;        // full record length (opcode + len + body)
};

// One decompressed chunk, ready for record iteration. The records blob
// starts `offset` bytes into `records` (non-zero when the buffer is the raw
// chunk record handed over without a copy for an uncompressed chunk).
// `data()`/`size` are only meaningful when `error` is empty.
struct PrefetchedChunk
{
  std::vector<std::byte> records;  // backing buffer holding the records blob
  std::size_t offset = 0;          // blob start within `records`
  std::size_t size = 0;            // blob size in bytes
  std::string error;               // non-empty => read/decompress failed

  [[nodiscard]] const std::byte * data() const { return records.data() + offset; }
};

// Decompresses `schedule`'s chunks on `num_threads` workers, each with its own
// file handle, at most a bounded lookahead ahead of the consumer. The consumer
// calls get(0), get(1), ... in ascending order; each call blocks until that
// chunk is ready and moves the buffer out. Destruction cancels outstanding
// work and joins the workers, so early consumer exit is safe.
class ChunkPrefetcher
{
public:
  ChunkPrefetcher(
    std::filesystem::path path, std::vector<ChunkRef> schedule, int num_threads,
    std::shared_ptr<ChunkBufferPool> pool);
  ~ChunkPrefetcher();

  ChunkPrefetcher(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher & operator=(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher(ChunkPrefetcher &&) = delete;
  ChunkPrefetcher & operator=(ChunkPrefetcher &&) = delete;

  [[nodiscard]] std::size_t size() const { return schedule_.size(); }

  // Blocking hand-off of schedule entry `index`; must be called with
  // ascending indexes (0, 1, ...), each exactly once.
  [[nodiscard]] PrefetchedChunk get(std::size_t index);

private:
  void worker_loop();

  const std::filesystem::path path_;
  const std::vector<ChunkRef> schedule_;
  const std::size_t lookahead_;
  const std::shared_ptr<ChunkBufferPool> pool_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::size_t next_claim_ = 0;  // next schedule index a worker may take
  std::size_t consumed_ = 0;    // entries already handed to the consumer
  bool cancel_ = false;
  std::string worker_fatal_error_;  // non-empty => a worker failed to start
  std::map<std::size_t, PrefetchedChunk> ready_;

  std::vector<std::jthread> workers_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_CHUNK_PREFETCH_HPP_
