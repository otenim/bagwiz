// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__PARALLEL_SORT_HPP_
#define BAGWIZ__CORE__BASE__PARALLEL_SORT_HPP_

#include "bagwiz/core/base/worker_pool.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace bagwiz::core
{

// Items below which parallel_sort sorts on the calling thread even when given
// a pool, and the stride at which it samples the input to balance its ranges.
inline constexpr std::size_t kParallelSortMinItems = 1U << 16U;
inline constexpr std::size_t kParallelSortSampleStride = 1024;

// Sort `items` under `less` on `pool`: sample every kParallelSortSampleStride-th
// item, take the sample's quantiles as range bounds, scatter the items into
// their ranges (one contiguous slice per range of a second buffer, filled by
// the workers chunk by chunk), sort every range in place on its own worker,
// and hand the buffer back as `items`. The ranges are disjoint under `less`
// and laid out in order, each sorted inside, so the result is sorted exactly
// as std::sort would sort it — elements equivalent under `less` may land in a
// different relative order, which std::sort does not pin down either; a
// caller that needs a particular order among equivalents sorts by a total
// order. Peak memory is the input plus one buffer of the same size. A small
// input, or a null / single-threaded pool, goes straight to std::sort. T must
// be default-constructible (the second buffer is value-initialized).
template <typename T, typename Less>
void parallel_sort(std::vector<T> & items, WorkerPool * pool, Less less)
{
  const std::size_t n = items.size();
  if (pool == nullptr || pool->size() <= 1 || n < kParallelSortMinItems) {
    std::sort(items.begin(), items.end(), less);
    return;
  }
  const auto ranges = static_cast<std::size_t>(pool->size());
  const std::size_t chunks = ranges;
  const std::size_t chunk = (n + chunks - 1) / chunks;

  // Range bounds: the quantiles of a strided sample. Range r holds the items
  // with bounds[r-1] <= x < bounds[r] under `less` (open-ended at both ends);
  // equivalent items always share a range.
  std::vector<T> sample;
  sample.reserve(n / kParallelSortSampleStride + 1);
  for (std::size_t i = 0; i < n; i += kParallelSortSampleStride) {
    sample.push_back(items[i]);
  }
  std::sort(sample.begin(), sample.end(), less);
  std::vector<T> bounds;
  bounds.reserve(ranges - 1);
  for (std::size_t r = 1; r < ranges; ++r) {
    bounds.push_back(sample[(r * sample.size()) / ranges]);
  }

  // Pass 1, per chunk: the range of every item and the per-chunk range counts.
  std::vector<std::uint16_t> range_of(n);
  std::vector<std::vector<std::size_t>> counts(chunks, std::vector<std::size_t>(ranges, 0));
  pool->parallel_for(chunks, [&](std::size_t c) {
    const std::size_t begin = std::min(n, c * chunk);
    const std::size_t end = std::min(n, begin + chunk);
    for (std::size_t i = begin; i < end; ++i) {
      const auto r = static_cast<std::uint16_t>(
        std::upper_bound(bounds.begin(), bounds.end(), items[i], less) - bounds.begin());
      range_of[i] = r;
      ++counts[c][r];
    }
  });

  // Where every range starts, and where every chunk's share of it starts.
  std::vector<std::size_t> base(ranges + 1, 0);
  for (std::size_t r = 0; r < ranges; ++r) {
    base[r + 1] = base[r];
    for (std::size_t c = 0; c < chunks; ++c) {
      base[r + 1] += counts[c][r];
    }
  }
  std::vector<std::vector<std::size_t>> cursor(chunks, std::vector<std::size_t>(ranges, 0));
  for (std::size_t r = 0; r < ranges; ++r) {
    std::size_t next = base[r];
    for (std::size_t c = 0; c < chunks; ++c) {
      cursor[c][r] = next;
      next += counts[c][r];
    }
  }

  // Pass 2, per chunk: scatter into the ranges. Each chunk writes only its own
  // share of every range, so the writes never overlap.
  std::vector<T> scattered(n);
  pool->parallel_for(chunks, [&](std::size_t c) {
    const std::size_t begin = std::min(n, c * chunk);
    const std::size_t end = std::min(n, begin + chunk);
    for (std::size_t i = begin; i < end; ++i) {
      scattered[cursor[c][range_of[i]]++] = items[i];
    }
  });

  // Pass 3, per range: sort in place.
  pool->parallel_for(ranges, [&](std::size_t r) {
    std::sort(
      scattered.begin() + static_cast<std::ptrdiff_t>(base[r]),
      scattered.begin() + static_cast<std::ptrdiff_t>(base[r + 1]), less);
  });
  items.swap(scattered);
}

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__PARALLEL_SORT_HPP_
