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
#include <utility>
#include <vector>

// Src-local building block of the parallel indexed mcap read path: a small
// worker pool that reads and decompresses one mcap file's chunk records ahead
// of the consumer, in a fixed schedule order, with a bounded lookahead window.
namespace bagwiz::io::detail
{

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

// Thread-safe pool of reusable chunk buffers, shared (via shared_ptr) between
// the prefetcher's workers and the retained-chunk deleters built by
// make_retained_chunk(). Keeping it a standalone shared object lets a
// retained chunk return its buffer for reuse no matter which thread drops the
// last reference — and, when the consumer outlives the prefetcher, lets the
// pool itself expire once the last holder is gone.
class ChunkBufferPool
{
public:
  // Pop a pooled buffer; an empty (capacity-0) vector when the pool is dry.
  [[nodiscard]] std::vector<std::byte> take()
  {
    std::lock_guard lock(mutex_);
    if (buffers_.empty()) {
      return {};
    }
    std::vector<std::byte> buf = std::move(buffers_.back());
    buffers_.pop_back();
    return buf;
  }

  // Return a buffer for reuse. Capacity-0 buffers are dropped: pooling them
  // would hand workers an allocation-free vector that still has to allocate.
  void put(std::vector<std::byte> && buf)
  {
    if (buf.capacity() == 0) {
      return;
    }
    std::lock_guard lock(mutex_);
    buffers_.push_back(std::move(buf));
  }

private:
  std::mutex mutex_;
  std::vector<std::vector<std::byte>> buffers_;
};

// Wrap a prefetched chunk in a shared handle whose deleter returns the
// backing buffer to `pool` when the last reference drops. This is what lets
// a consumer keep a chunk's bytes alive past the stream's own slot reuse
// (BagReader::retain_payload) without taking the buffer out of circulation
// for good.
[[nodiscard]] std::shared_ptr<const PrefetchedChunk> make_retained_chunk(
  PrefetchedChunk && chunk, std::shared_ptr<ChunkBufferPool> pool);

// Decompresses `schedule`'s chunks on `num_threads` workers, each with its own
// file handle, at most a bounded lookahead ahead of the consumer. The consumer
// calls get(0), get(1), ... in ascending order; each call blocks until that
// chunk is ready and moves the buffer out. Destruction cancels outstanding
// work and joins the workers, so early consumer exit is safe.
class ChunkPrefetcher
{
public:
  ChunkPrefetcher(std::filesystem::path path, std::vector<ChunkRef> schedule, int num_threads);
  ~ChunkPrefetcher();

  ChunkPrefetcher(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher & operator=(const ChunkPrefetcher &) = delete;
  ChunkPrefetcher(ChunkPrefetcher &&) = delete;
  ChunkPrefetcher & operator=(ChunkPrefetcher &&) = delete;

  [[nodiscard]] std::size_t size() const { return schedule_.size(); }

  // Blocking hand-off of schedule entry `index`; must be called with
  // ascending indexes (0, 1, ...), each exactly once.
  [[nodiscard]] PrefetchedChunk get(std::size_t index);

  // The buffer pool the workers draw from. Consumed chunks return their
  // buffers to it through make_retained_chunk()'s deleter, keeping the
  // steady state free of large allocations (and of the page faults +
  // zero-fill that come with fresh multi-MB buffers).
  [[nodiscard]] const std::shared_ptr<ChunkBufferPool> & pool() const { return pool_; }

private:
  void worker_loop();

  const std::filesystem::path path_;
  const std::vector<ChunkRef> schedule_;
  const std::size_t lookahead_;
  const std::shared_ptr<ChunkBufferPool> pool_ = std::make_shared<ChunkBufferPool>();

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
