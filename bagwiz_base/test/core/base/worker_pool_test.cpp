// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/worker_pool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{

using bagwiz::core::WorkerPool;

TEST(WorkerPoolTest, RunsEveryIndexExactlyOnce)
{
  WorkerPool pool{4};
  constexpr std::size_t kN = 1000;
  std::vector<std::atomic<int>> hits(kN);
  pool.parallel_for(kN, [&](std::size_t i) { hits[i].fetch_add(1); });
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(hits[i].load(), 1) << "index " << i;
  }
}

TEST(WorkerPoolTest, SizeIsTheRequestedParallelismClampedToOne)
{
  EXPECT_EQ(WorkerPool{0}.size(), 1);
  EXPECT_EQ(WorkerPool{1}.size(), 1);
  EXPECT_EQ(WorkerPool{4}.size(), 4);
}

TEST(WorkerPoolTest, SingleThreadedPoolRunsInline)
{
  WorkerPool pool{1};
  const auto caller = std::this_thread::get_id();
  std::vector<std::thread::id> ids(8);
  pool.parallel_for(8, [&](std::size_t i) { ids[i] = std::this_thread::get_id(); });
  for (const auto & id : ids) {
    EXPECT_EQ(id, caller);
  }
}

TEST(WorkerPoolTest, UsesMoreThanOneThread)
{
  // Every task waits for a second task to be running at the same time; with
  // a single thread the first task would sit out the deadline alone.
  WorkerPool pool{4};
  std::atomic<int> arrived{0};
  std::atomic<bool> met{false};
  pool.parallel_for(4, [&](std::size_t) {
    arrived.fetch_add(1);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (arrived.load() < 2 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::yield();
    }
    if (arrived.load() >= 2) {
      met.store(true);
    }
  });
  EXPECT_TRUE(met.load());
}

TEST(WorkerPoolTest, PropagatesTheExceptionAndStaysUsable)
{
  WorkerPool pool{3};
  try {
    pool.parallel_for(64, [](std::size_t i) {
      if (i == 7) {
        throw std::runtime_error("index seven");
      }
    });
    FAIL() << "expected the loop to throw";
  } catch (const std::runtime_error & e) {
    EXPECT_STREQ(e.what(), "index seven");
  }
  std::atomic<int> count{0};
  pool.parallel_for(100, [&](std::size_t) { count.fetch_add(1); });
  EXPECT_EQ(count.load(), 100);
}

TEST(WorkerPoolTest, InlineLoopSkipsTheIndicesAfterAFailure)
{
  WorkerPool pool{1};
  std::vector<std::size_t> ran;
  EXPECT_THROW(
    pool.parallel_for(
      10,
      [&](std::size_t i) {
        if (i == 3) {
          throw std::runtime_error("stop");
        }
        ran.push_back(i);
      }),
    std::runtime_error);
  EXPECT_EQ(ran, (std::vector<std::size_t>{0, 1, 2}));
}

TEST(WorkerPoolTest, EmptyRangeDoesNothing)
{
  WorkerPool pool{4};
  bool called = false;
  pool.parallel_for(0, [&](std::size_t) { called = true; });
  EXPECT_FALSE(called);
}

TEST(WorkerPoolTest, ReusableAcrossManyLoops)
{
  WorkerPool pool{4};
  std::atomic<std::uint64_t> sum{0};
  for (int round = 0; round < 200; ++round) {
    pool.parallel_for(37, [&](std::size_t i) { sum.fetch_add(i); });
  }
  EXPECT_EQ(sum.load(), 200ULL * (36ULL * 37ULL / 2ULL));
}

}  // namespace
