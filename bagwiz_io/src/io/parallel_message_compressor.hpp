// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__PARALLEL_MESSAGE_COMPRESSOR_HPP_
#define IO__PARALLEL_MESSAGE_COMPRESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace bagwiz::io::detail
{

// A worker pool that compresses message payloads concurrently (one bare zstd
// frame per payload, the rosbag2 MESSAGE-mode shape) while the caller drains
// completions strictly in submission order. Built for the sqlite3 MESSAGE-mode
// writer, whose inserts into the messages table must stay in arrival order.
//
// The pool never inserts or reorders anything itself: submit() appends a job,
// workers compress jobs as they find them, and try_take()/take_blocking()
// return the oldest submitted job only once its frame is ready. Per-message
// frames depend only on the payload bytes, so the drained stream is
// byte-identical for any worker count.
//
// submit()/try_take()/take_blocking()/finish() are caller-thread-only (the
// BagWriter contract); workers never call them. Payload bytes are pinned by
// `owner` until the job's frame is ready, so the caller may submit spans it
// does not own as long as ownership rides along.
class ParallelMessageCompressor
{
public:
  // `workers` >= 2 (the caller keeps a serial path for 0/1). `level` is a
  // zstd level (0 = library default). Throws std::runtime_error on allocation
  // failure.
  ParallelMessageCompressor(int workers, int level);
  ~ParallelMessageCompressor();  // finish() on best effort; never throws

  ParallelMessageCompressor(const ParallelMessageCompressor &) = delete;
  ParallelMessageCompressor & operator=(const ParallelMessageCompressor &) = delete;
  ParallelMessageCompressor(ParallelMessageCompressor &&) = delete;
  ParallelMessageCompressor & operator=(ParallelMessageCompressor &&) = delete;

  // Insert context carried opaquely through the pool so the caller can bind
  // the completed frame without its own reorder buffer.
  struct Job
  {
    int64_t topic_id = 0;
    int64_t timestamp_ns = 0;
    std::vector<std::byte> compressed;
  };

  // Queue one payload for compression. `owner` pins the payload's backing
  // store; pass nullptr only for payloads that outlive finish(). Throws if a
  // worker has latched a compression error.
  void submit(
    int64_t topic_id, int64_t timestamp_ns, std::span<const std::byte> payload,
    std::shared_ptr<const void> owner);

  // Move the oldest job's frame into `out` once it is ready; returns false
  // when the oldest job is still compressing. take_blocking() waits. Both
  // throw a latched worker error.
  bool try_take(Job & out);
  void take_blocking(Job & out);

  // Outstanding (submitted but not yet taken) payload bytes — the caller
  // enforces its in-flight cap against this.
  std::uint64_t in_flight_bytes() const;
  bool empty() const;

  // Stop the pool after every job has been taken; joins the workers and
  // rethrows a latched worker error. Must not be called with jobs
  // outstanding.
  void finish();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::io::detail

#endif  // IO__PARALLEL_MESSAGE_COMPRESSOR_HPP_
