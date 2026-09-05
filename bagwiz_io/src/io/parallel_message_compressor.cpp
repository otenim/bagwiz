// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "parallel_message_compressor.hpp"  // NOLINT(build/include_subdir) src-local header

#include "bagwiz/io/message_compressor.hpp"

#include <condition_variable>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

class ParallelMessageCompressor::Impl
{
public:
  Impl(int workers, int level)
  {
    // One compressor per worker, built here so an allocation failure surfaces
    // from the constructor instead of inside a thread body.
    compressors_.reserve(static_cast<std::size_t>(workers));
    for (int i = 0; i < workers; ++i) {
      compressors_.emplace_back("zstd", level);
    }
    workers_.reserve(static_cast<std::size_t>(workers));
    for (int i = 0; i < workers; ++i) {
      workers_.emplace_back(
        [this, i]() { worker_loop(compressors_[static_cast<std::size_t>(i)]); });
    }
  }

  ~Impl()
  {
    if (!finished_) {
      try {
        cancel();
      } catch (...) {
        // Destructors must not throw.
      }
    }
  }

  void submit(
    int64_t topic_id, int64_t timestamp_ns, std::span<const std::byte> payload,
    std::shared_ptr<const void> owner)
  {
    throw_if_failed();
    Slot slot;
    slot.topic_id = topic_id;
    slot.timestamp_ns = timestamp_ns;
    slot.payload = payload.data();
    slot.payload_size = payload.size();
    slot.owner = std::move(owner);
    {
      std::lock_guard lock(mutex_);
      in_flight_bytes_ += slot.payload_size;
      slots_.push_back(std::move(slot));
    }
    cv_.notify_all();
  }

  bool try_take(Job & out)
  {
    std::lock_guard lock(mutex_);
    throw_if_failed_locked();
    if (slots_.empty() || !slots_.front().ready) {
      return false;
    }
    take_front_locked(out);
    return true;
  }

  void take_blocking(Job & out)
  {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [&] { return !error_.empty() || (!slots_.empty() && slots_.front().ready); });
    throw_if_failed_locked();
    take_front_locked(out);
  }

  std::uint64_t in_flight_bytes() const
  {
    std::lock_guard lock(mutex_);
    return in_flight_bytes_;
  }

  bool empty() const
  {
    std::lock_guard lock(mutex_);
    return slots_.empty();
  }

  void finish()
  {
    {
      std::lock_guard lock(mutex_);
      finished_ = true;
      stop_ = true;
    }
    cv_.notify_all();
    for (auto & w : workers_) {
      if (w.joinable()) {
        w.join();
      }
    }
    throw_if_failed();
  }

private:
  struct Slot
  {
    int64_t topic_id = 0;
    int64_t timestamp_ns = 0;
    const std::byte * payload = nullptr;
    std::uint64_t payload_size = 0;
    std::shared_ptr<const void> owner;
    bool claimed = false;  // a worker owns this slot's compression
    bool ready = false;    // compressed is final and may be taken
    std::vector<std::byte> compressed;
  };

  void cancel()
  {
    {
      std::lock_guard lock(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    for (auto & w : workers_) {
      if (w.joinable()) {
        w.join();
      }
    }
  }

  void take_front_locked(Job & out)
  {
    Slot & front = slots_.front();
    out.topic_id = front.topic_id;
    out.timestamp_ns = front.timestamp_ns;
    out.compressed = std::move(front.compressed);
    in_flight_bytes_ -= front.payload_size;
    slots_.pop_front();
  }

  void throw_if_failed() const
  {
    std::lock_guard lock(mutex_);
    throw_if_failed_locked();
  }

  void throw_if_failed_locked() const
  {
    if (!error_.empty()) {
      throw std::runtime_error(error_);
    }
  }

  void worker_loop(MessageCompressor & compressor)
  {
    while (true) {
      Slot * slot = nullptr;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [&] { return stop_ || !error_.empty() || find_unclaimed() != nullptr; });
        if (stop_ || !error_.empty()) {
          return;
        }
        slot = find_unclaimed();
        slot->claimed = true;
      }

      // A slot's fields are owned by the queue and touched by exactly one
      // thread at a time: the caller until `claimed`, this worker until
      // `ready`, the caller again afterwards.
      try {
        const auto frame = compressor.compress({slot->payload, slot->payload_size});
        slot->compressed.assign(frame.begin(), frame.end());
      } catch (const std::exception & e) {
        std::lock_guard lock(mutex_);
        if (error_.empty()) {
          error_ = std::string("message compression failed: ") + e.what();
        }
        slot->ready = true;
        cv_.notify_all();
        return;
      }

      // Release the pinned payload backing before marking the slot ready:
      // once `ready` is set the caller may take and destroy the slot, so
      // everything the worker still touches must happen beforehand.
      slot->owner.reset();
      slot->payload = nullptr;
      {
        std::lock_guard lock(mutex_);
        slot->ready = true;
      }
      cv_.notify_all();
    }
  }

  Slot * find_unclaimed()
  {
    for (auto & slot : slots_) {
      if (!slot.claimed) {
        return &slot;
      }
    }
    return nullptr;
  }

  std::vector<MessageCompressor> compressors_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Slot> slots_;
  std::uint64_t in_flight_bytes_ = 0;
  bool stop_ = false;
  bool finished_ = false;
  std::string error_;

  std::vector<std::jthread> workers_;
};

ParallelMessageCompressor::ParallelMessageCompressor(int workers, int level)
: impl_(std::make_unique<Impl>(workers, level))
{
}

ParallelMessageCompressor::~ParallelMessageCompressor() = default;

void ParallelMessageCompressor::submit(
  int64_t topic_id, int64_t timestamp_ns, std::span<const std::byte> payload,
  std::shared_ptr<const void> owner)
{
  impl_->submit(topic_id, timestamp_ns, payload, std::move(owner));
}

bool ParallelMessageCompressor::try_take(Job & out)
{
  return impl_->try_take(out);
}

void ParallelMessageCompressor::take_blocking(Job & out)
{
  impl_->take_blocking(out);
}

std::uint64_t ParallelMessageCompressor::in_flight_bytes() const
{
  return impl_->in_flight_bytes();
}

bool ParallelMessageCompressor::empty() const
{
  return impl_->empty();
}

void ParallelMessageCompressor::finish()
{
  impl_->finish();
}

}  // namespace bagwiz::io::detail
