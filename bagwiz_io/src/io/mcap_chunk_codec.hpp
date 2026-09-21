// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__MCAP_CHUNK_CODEC_HPP_
#define IO__MCAP_CHUNK_CODEC_HPP_

#include <mcap/types.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Src-local codec for whole mcap Chunk records: parse and decompress one raw
// Chunk record into its records blob, and compress a records blob back with a
// named codec. Shared by the parallel indexed read path (ChunkPrefetcher) and
// the chunk pass-through rewrite engine.
namespace bagwiz::io::detail
{

// Parsed header fields plus the decompressed records blob of one mcap Chunk
// record.
struct DecodedChunk
{
  std::uint64_t message_start_time = 0;
  std::uint64_t message_end_time = 0;
  std::string compression;         // "" / "none" / "zstd" / "lz4"
  std::vector<std::byte> records;  // decompressed records blob
  std::string error;               // non-empty => parse/decompress failed
};

// Parse one raw chunk record (opcode + length prefix included) and produce
// its decompressed records blob. Supports uncompressed, zstd, and lz4 chunks.
// Never throws; failures land in `error` with `records` cleared.
DecodedChunk decompress_chunk_record(std::span<const std::byte> record);

// The encoder level a chunk codec runs at when no level was named: the one
// rule the mcap writer and the pass-through's re-encode share, so a rewrite
// never mixes two levels of one codec in a bag. mcap maps lz4 +
// CompressionLevel::Default onto LZ4-HC, which measures several times slower
// than zstd's default for a larger output — the opposite of what choosing
// lz4 means — so lz4 gets its fast mode; every other codec gets Default.
mcap::CompressionLevel unset_compression_level(mcap::Compression codec);

// Compress a records blob with `compression` ("" / "none" returns a verbatim
// copy; "zstd" and "lz4" compress at unset_compression_level()). Throws
// std::runtime_error on an unknown codec name or a compressor failure.
std::vector<std::byte> compress_chunk_records(
  std::span<const std::byte> records, std::string_view compression);

}  // namespace bagwiz::io::detail

#endif  // IO__MCAP_CHUNK_CODEC_HPP_
