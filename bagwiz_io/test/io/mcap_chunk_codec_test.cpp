// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/mcap_chunk_codec.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include <mcap/writer.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

// The chunk pass-through re-encodes the chunks an edit touches with the
// chunk's own codec. The level it picks must be the one the mcap writer picks
// for an unset level, so a rewrite never mixes two lz4 modes in one bag.
namespace
{

using bagwiz::io::detail::compress_chunk_records;
using bagwiz::io::detail::unset_compression_level;

// A records blob with enough repetition for lz4's fast and high-compression
// modes to choose different matches, so their outputs can be told apart.
std::vector<std::byte> compressible_records()
{
  std::string text;
  for (int i = 0; i < 400; ++i) {
    text += "record " + std::to_string(i % 37) + " payload payload payload " +
            std::to_string((i * 7) % 101) + "\n";
  }
  std::vector<std::byte> out;
  out.reserve(text.size());
  for (const char c : text) {
    out.push_back(static_cast<std::byte>(c));
  }
  return out;
}

std::vector<std::byte> encode_with(
  mcap::IChunkWriter & writer, const std::vector<std::byte> & records)
{
  writer.write(records.data(), records.size());
  writer.end();
  const std::byte * data = writer.compressedData();
  return {data, data + writer.compressedSize()};
}

}  // namespace

// One rule, shared by the writer and the pass-through: an unset level means
// lz4's fast mode (mcap maps lz4 + Default onto LZ4-HC, several times slower
// than zstd's default for a larger output) and zstd's own default.
TEST(McapChunkCodec, UnsetLevelIsFastForLz4AndDefaultForZstd)
{
  EXPECT_EQ(unset_compression_level(mcap::Compression::Lz4), mcap::CompressionLevel::Fastest);
  EXPECT_EQ(unset_compression_level(mcap::Compression::Zstd), mcap::CompressionLevel::Default);
}

TEST(McapChunkCodec, ReencodesLz4ChunksInFastMode)
{
  const auto records = compressible_records();
  mcap::LZ4Writer fast(mcap::CompressionLevel::Fastest, records.size());
  mcap::LZ4Writer high_compression(mcap::CompressionLevel::Default, records.size());
  const auto expected = encode_with(fast, records);
  ASSERT_NE(expected, encode_with(high_compression, records))
    << "the fixture cannot tell lz4's two modes apart";

  EXPECT_EQ(compress_chunk_records(records, "lz4"), expected);
}

TEST(McapChunkCodec, ReencodesZstdChunksAtTheDefaultLevel)
{
  const auto records = compressible_records();
  mcap::ZStdWriter writer(mcap::CompressionLevel::Default, records.size());

  EXPECT_EQ(compress_chunk_records(records, "zstd"), encode_with(writer, records));
}
