// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__WORKER_POOL_HPP_
#define BAGWIZ__CORE__BASE__WORKER_POOL_HPP_

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace bagwiz::core
{

// A fixed set of worker threads, started once and reused for every
// index-parallel loop a command runs, so a pass that evaluates thousands of
// short loops — one per cloud, one per cost evaluation — does not pay a thread
// creation per loop. The pool holds no queue beyond the loop in flight:
// parallel_for publishes one loop, every worker plus the calling thread claim
// indices from it until none are left, and the call returns only once every
// index has finished, so a loop body may capture references into the caller's
// stack.
class WorkerPool
{
public:
  // `threads` is a loop's total parallelism: the calling thread plus
  // threads - 1 workers. A value of 1 or less starts no worker thread, and
  // every loop then runs inline on the caller — the synchronous mode.
  explicit WorkerPool(int threads);
  ~WorkerPool();
  WorkerPool(const WorkerPool &) = delete;
  WorkerPool & operator=(const WorkerPool &) = delete;

  // Total parallelism of a loop: the workers plus the calling thread, >= 1.
  [[nodiscard]] int size() const;

  // Run fn(i) for every i in [0, n) and return once every call has finished.
  // Indices are handed out one at a time to whichever thread asks next, so
  // uneven work balances itself, and the calling thread takes its share. An
  // exception thrown by fn stops the loop — indices not yet started are
  // skipped — and the first one thrown is rethrown to the caller after the
  // calls already running have finished. Not reentrant: fn must not call
  // parallel_for on the same pool.
  void parallel_for(std::size_t n, const std::function<void(std::size_t)> & fn);

private:
  struct Loop;
  void worker_main();
  void run_loop(Loop & loop);

  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable wake_;  // workers: a new loop was published, or stop
  std::condition_variable done_;  // caller: the loop's last index finished
  std::shared_ptr<Loop> current_;
  std::uint64_t generation_ = 0;  // bumped per published loop
  bool stop_ = false;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__WORKER_POOL_HPP_
