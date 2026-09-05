// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Unit coverage for the MESSAGE-mode compression pool: submission-order drain,
// frame-level agreement with the serial MessageCompressor across worker
// counts, and the in-flight byte accounting the caller's cap relies on.

#include "io/parallel_message_compressor.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/message_compressor.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace
{

using bagwiz::io::detail::ParallelMessageCompressor;

// Compressible pattern with the seed mixed in (zstd shrinks it well).
std::vector<std::byte> pattern_payload(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed + static_cast<int>(i) / 7) & 0xFF);
  }
  return out;
}

// Incompressible xorshift PRNG output.
std::vector<std::byte> random_payload(std::uint64_t & state, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    out[i] = static_cast<std::byte>(state & 0xFF);
  }
  return out;
}

// Run the pool over `payloads` with `workers` workers, interleaving an
// opportunistic drain the way SqliteFileWriter does, and return the drained
// jobs in drain order.
std::vector<ParallelMessageCompressor::Job> run_pool(
  int workers, const std::vector<std::vector<std::byte>> & payloads)
{
  ParallelMessageCompressor pool(workers, /*level=*/0);
  std::vector<ParallelMessageCompressor::Job> out;
  for (std::size_t i = 0; i < payloads.size(); ++i) {
    const auto & p = payloads[i];
    // Hand ownership over the way the pipeline's frozen messages arrive.
    auto frozen = bagwiz::io::own_payload(std::vector<std::byte>(p.begin(), p.end()));
    pool.submit(
      static_cast<std::int64_t>(i % 3 + 1), static_cast<std::int64_t>(1000 + i), frozen.payload,
      std::move(frozen.owner));
    ParallelMessageCompressor::Job job;
    while (pool.try_take(job)) {
      out.push_back(std::move(job));
    }
  }
  while (!pool.empty()) {
    ParallelMessageCompressor::Job job;
    pool.take_blocking(job);
    out.push_back(std::move(job));
  }
  pool.finish();
  return out;
}

std::vector<std::vector<std::byte>> mixed_payloads(std::size_t count)
{
  std::vector<std::vector<std::byte>> payloads;
  std::uint64_t state = 0x9E3779B97F4A7C15ull;
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t size = (i % 7 == 0) ? (i * 997) % (256 * 1024) : 64 + (i * 37) % 4096;
    payloads.push_back(
      (i % 5 == 4) ? random_payload(state, size) : pattern_payload(static_cast<int>(i), size));
  }
  return payloads;
}

// Exact byte equality is asserted here because each message's frame is a
// per-element computation that reads only immutable input (zstd is
// deterministic for a fixed level and payload) and the drain order is fixed
// to submission order — neither depends on the worker count or schedule.
TEST(ParallelMessageCompressorTest, DrainsInSubmissionOrderMatchingSerialFrames)
{
  const auto payloads = mixed_payloads(400);

  bagwiz::io::MessageCompressor serial("zstd", 0);
  std::vector<std::vector<std::byte>> reference;
  for (const auto & p : payloads) {
    const auto frame = serial.compress(p);
    reference.emplace_back(frame.begin(), frame.end());
  }

  for (const int workers : {2, 4, 8}) {
    const auto out = run_pool(workers, payloads);
    ASSERT_EQ(out.size(), payloads.size()) << "workers=" << workers;
    for (std::size_t i = 0; i < out.size(); ++i) {
      EXPECT_EQ(out[i].topic_id, static_cast<std::int64_t>(i % 3 + 1)) << "workers=" << workers;
      EXPECT_EQ(out[i].timestamp_ns, static_cast<std::int64_t>(1000 + i)) << "workers=" << workers;
      EXPECT_EQ(out[i].compressed, reference[i]) << "workers=" << workers << " payload index=" << i;
    }
  }
}

TEST(ParallelMessageCompressorTest, EmptyPayloadRoundTrips)
{
  ParallelMessageCompressor pool(2, 0);
  pool.submit(1, 42, {}, nullptr);
  ParallelMessageCompressor::Job job;
  pool.take_blocking(job);
  pool.finish();

  bagwiz::io::MessageCompressor serial("zstd", 0);
  const auto frame = serial.compress({});
  EXPECT_EQ(job.compressed, std::vector<std::byte>(frame.begin(), frame.end()));
  EXPECT_EQ(job.topic_id, 1);
  EXPECT_EQ(job.timestamp_ns, 42);
}

TEST(ParallelMessageCompressorTest, InFlightBytesTrackOutstandingPayloads)
{
  ParallelMessageCompressor pool(2, 0);
  std::uint64_t expected = 0;
  std::vector<bagwiz::io::FrozenMessage> keep;
  for (const std::size_t size : {100UL, 200UL, 300UL}) {
    auto frozen = bagwiz::io::own_payload(pattern_payload(1, size));
    expected += size;
    pool.submit(1, 1, frozen.payload, std::move(frozen.owner));
  }
  // Submissions may already be compressed but not yet taken; in-flight bytes
  // only drop at take time.
  EXPECT_EQ(pool.in_flight_bytes(), expected);

  ParallelMessageCompressor::Job job;
  pool.take_blocking(job);
  EXPECT_EQ(pool.in_flight_bytes(), expected - 100);
  while (!pool.empty()) {
    pool.take_blocking(job);
  }
  EXPECT_EQ(pool.in_flight_bytes(), 0U);
  pool.finish();
}

}  // namespace
