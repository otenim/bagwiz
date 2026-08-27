// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/message_compressor.hpp"

#include "bagwiz/io/file_decompressor.hpp"  // is_zstd_magic
#include "bagwiz/io/message_decompressor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <stdexcept>
#include <vector>

namespace
{

std::vector<std::byte> to_bytes(std::initializer_list<std::uint8_t> v)
{
  std::vector<std::byte> out;
  out.reserve(v.size());
  for (const auto b : v) {
    out.push_back(static_cast<std::byte>(b));
  }
  return out;
}

bool spans_equal(std::span<const std::byte> a, std::span<const std::byte> b)
{
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

TEST(MessageCompressorTest, RoundTripsThroughMessageDecompressor)
{
  // The pair shares one contract — a bare single zstd frame per message, the
  // rosbag2 MESSAGE-mode byte shape — so the decompressor reading back
  // exactly what the compressor wrote is the whole point.
  bagwiz::io::MessageCompressor compressor("zstd");
  bagwiz::io::MessageDecompressor decompressor("zstd");

  const auto payload = to_bytes({0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33});
  // Copy out of the compressor's internal buffer before decompress() (both
  // classes invalidate their span on the next call).
  const auto compressed = compressor.compress(payload);
  const std::vector<std::byte> compressed_copy(compressed.begin(), compressed.end());

  ASSERT_FALSE(compressed_copy.empty());
  EXPECT_NE(compressed_copy, payload) << "payload was stored uncompressed";
  EXPECT_TRUE(bagwiz::io::is_zstd_magic(compressed_copy))
    << "rosbag2 MESSAGE-mode payloads are bare zstd frames";

  const auto restored = decompressor.decompress(compressed_copy);
  EXPECT_TRUE(spans_equal(restored, payload));
}

TEST(MessageCompressorTest, RoundTripsEmptyPayload)
{
  bagwiz::io::MessageCompressor compressor("zstd");
  bagwiz::io::MessageDecompressor decompressor("zstd");

  const auto compressed = compressor.compress({});
  const std::vector<std::byte> compressed_copy(compressed.begin(), compressed.end());
  const auto restored = decompressor.decompress(compressed_copy);
  EXPECT_TRUE(restored.empty());
}

TEST(MessageCompressorTest, LevelChangesOutputButNotContract)
{
  // A higher effort level must never change the byte contract — both levels
  // still decompress to the same payload through the shared decompressor.
  const auto payload = to_bytes({0x01, 0x02, 0x03, 0x04});

  bagwiz::io::MessageCompressor fast("zstd", 1);
  bagwiz::io::MessageCompressor slow("zstd", 19);
  bagwiz::io::MessageDecompressor decompressor("zstd");

  const auto fast_frame = fast.compress(payload);
  const std::vector<std::byte> fast_copy(fast_frame.begin(), fast_frame.end());
  const auto slow_frame = slow.compress(payload);
  const std::vector<std::byte> slow_copy(slow_frame.begin(), slow_frame.end());

  EXPECT_TRUE(spans_equal(decompressor.decompress(fast_copy), payload));
  EXPECT_TRUE(spans_equal(decompressor.decompress(slow_copy), payload));
}

TEST(MessageCompressorTest, RejectsUnsupportedFormat)
{
  EXPECT_THROW(bagwiz::io::MessageCompressor("lz4"), std::runtime_error);
}

TEST(MessageCompressorTest, RejectsOutOfRangeLevel)
{
  EXPECT_THROW(bagwiz::io::MessageCompressor("zstd", -1), std::runtime_error);
  EXPECT_THROW(bagwiz::io::MessageCompressor("zstd", 100), std::runtime_error);
}

TEST(ZstdLevelFromNameTest, MapsAdvertisedNamesMonotonically)
{
  using bagwiz::io::zstd_level_from_name;
  EXPECT_EQ(zstd_level_from_name(""), 0);
  EXPECT_EQ(zstd_level_from_name("default"), 0);
  EXPECT_EQ(zstd_level_from_name("fastest"), 1);
  EXPECT_EQ(zstd_level_from_name("fast"), 2);
  EXPECT_EQ(zstd_level_from_name("slow"), 9);
  EXPECT_EQ(zstd_level_from_name("slowest"), 19);
}

TEST(ZstdLevelFromNameTest, RejectsUnknownName)
{
  EXPECT_THROW(bagwiz::io::zstd_level_from_name("ludicrous"), std::runtime_error);
}

}  // namespace
