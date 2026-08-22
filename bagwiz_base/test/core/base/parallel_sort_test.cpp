// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/parallel_sort.hpp"

#include "bagwiz/core/base/worker_pool.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::parallel_sort;
using bagwiz::core::WorkerPool;

// Deterministic pseudo-random integers from a fixed-seed LCG.
class Lcg
{
public:
  explicit Lcg(std::uint64_t seed) : state_(seed) {}
  std::uint64_t next()
  {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    return state_ >> 11;
  }

private:
  std::uint64_t state_;
};

TEST(ParallelSortTest, SortsLargeInputWithDuplicatesLikeStdSort)
{
  // 300,001 integers (no multiple of the range count) drawn from a small range
  // so duplicates are plentiful: the sorted sequence is the one std::sort
  // produces, because equal integers are indistinguishable.
  Lcg rng{1};
  std::vector<int> items;
  for (int i = 0; i < 300001; ++i) {
    items.push_back(static_cast<int>(rng.next() % 5000) - 2500);
  }
  auto expected = items;
  std::sort(expected.begin(), expected.end());
  WorkerPool pool{4};
  parallel_sort(items, &pool, std::less<int>{});
  EXPECT_EQ(items, expected);
}

TEST(ParallelSortTest, SortsPairsByAComparatorOverATotalOrder)
{
  // Unique (a, b) keys under a lexicographic comparator: a total order, so the
  // result is exactly std::sort's.
  Lcg rng{2};
  std::vector<std::pair<int, int>> items;
  for (int i = 0; i < 200000; ++i) {
    items.push_back({static_cast<int>(rng.next() % 1000), i});
  }
  auto expected = items;
  const auto less = [](const std::pair<int, int> & a, const std::pair<int, int> & b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  };
  std::sort(expected.begin(), expected.end(), less);
  WorkerPool pool{3};
  parallel_sort(items, &pool, less);
  EXPECT_EQ(items, expected);
}

TEST(ParallelSortTest, SmallInputAndNoPoolFallBackToStdSort)
{
  std::vector<int> small{5, 3, 9, 1, 3};
  WorkerPool pool{4};
  parallel_sort(small, &pool, std::less<int>{});
  EXPECT_EQ(small, (std::vector<int>{1, 3, 3, 5, 9}));

  Lcg rng{3};
  std::vector<int> items;
  for (int i = 0; i < 100000; ++i) {
    items.push_back(static_cast<int>(rng.next() % 100));
  }
  auto expected = items;
  std::sort(expected.begin(), expected.end());
  parallel_sort(items, nullptr, std::less<int>{});
  EXPECT_EQ(items, expected);
}

TEST(ParallelSortTest, AlreadySortedAndConstantInputsStaySorted)
{
  // Degenerate inputs: every sample equal (one range takes everything) and a
  // pre-sorted run.
  std::vector<int> constant(100000, 7);
  WorkerPool pool{4};
  parallel_sort(constant, &pool, std::less<int>{});
  EXPECT_TRUE(std::all_of(constant.begin(), constant.end(), [](int v) { return v == 7; }));
  EXPECT_EQ(constant.size(), 100000U);

  std::vector<int> sorted(100000);
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    sorted[i] = static_cast<int>(i);
  }
  parallel_sort(sorted, &pool, std::less<int>{});
  EXPECT_TRUE(std::is_sorted(sorted.begin(), sorted.end()));
  EXPECT_EQ(sorted.front(), 0);
  EXPECT_EQ(sorted.back(), 99999);
}

}  // namespace
