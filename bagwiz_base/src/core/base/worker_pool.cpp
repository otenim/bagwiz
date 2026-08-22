// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/worker_pool.hpp"

#include <atomic>
#include <exception>
#include <memory>
#include <utility>

namespace bagwiz::core
{

// One published loop. Workers hold it through a shared_ptr, so a worker that
// wakes late still sees a consistent (fn, n, next) triple — its claims fail
// against this loop's own `next` — and never touches a newer loop's body.
struct WorkerPool::Loop
{
  std::function<void(std::size_t)> fn;  // a copy: outlives the caller's temporary
  std::size_t n = 0;
  std::atomic<std::size_t> next{0};      // next index to hand out
  std::atomic<std::size_t> finished{0};  // indices done (run or skipped)
  std::atomic<bool> failed{false};
  std::exception_ptr error;  // the first exception; guarded by the pool mutex
};

WorkerPool::WorkerPool(int threads)
{
  const int worker_count = threads > 1 ? threads - 1 : 0;
  workers_.reserve(static_cast<std::size_t>(worker_count));
  for (int i = 0; i < worker_count; ++i) {
    workers_.emplace_back([this] { worker_main(); });
  }
}

WorkerPool::~WorkerPool()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  for (auto & worker : workers_) {
    worker.join();
  }
}

int WorkerPool::size() const
{
  return static_cast<int>(workers_.size()) + 1;
}

void WorkerPool::worker_main()
{
  std::uint64_t seen = 0;
  for (;;) {
    std::shared_ptr<Loop> loop;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
      if (stop_) {
        return;
      }
      seen = generation_;
      loop = current_;
    }
    if (loop) {
      run_loop(*loop);
    }
  }
}

void WorkerPool::run_loop(Loop & loop)
{
  for (;;) {
    const std::size_t i = loop.next.fetch_add(1);
    if (i >= loop.n) {
      return;
    }
    if (!loop.failed.load()) {
      try {
        loop.fn(i);
      } catch (...) {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!loop.error) {
          loop.error = std::current_exception();
        }
        loop.failed.store(true);
      }
    }
    if (loop.finished.fetch_add(1) + 1 == loop.n) {
      // Notify under the mutex so the caller's predicate check and this
      // wake-up cannot interleave into a lost notification.
      const std::lock_guard<std::mutex> lock(mutex_);
      done_.notify_all();
    }
  }
}

void WorkerPool::parallel_for(std::size_t n, const std::function<void(std::size_t)> & fn)
{
  if (n == 0) {
    return;
  }
  if (workers_.empty()) {
    for (std::size_t i = 0; i < n; ++i) {
      fn(i);
    }
    return;
  }
  auto loop = std::make_shared<Loop>();
  loop->fn = fn;
  loop->n = n;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    current_ = loop;
    ++generation_;
  }
  wake_.notify_all();
  run_loop(*loop);
  {
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [&] { return loop->finished.load() == n; });
    current_.reset();
  }
  if (loop->error) {
    std::rethrow_exception(loop->error);
  }
}

}  // namespace bagwiz::core
