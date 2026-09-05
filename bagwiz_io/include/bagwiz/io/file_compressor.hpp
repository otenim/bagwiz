// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__FILE_COMPRESSOR_HPP_
#define BAGWIZ__IO__FILE_COMPRESSOR_HPP_

#include <filesystem>

namespace bagwiz::io
{

// Stream-compress the whole file at `src` into `dst` as a single zstd frame —
// the write-side mirror of decompress_zstd_file_to_temp() in
// file_decompressor.hpp. Used to emit rosbag2 `compression_mode: FILE`
// (whole-database `.db3.zstd` envelope) bags: the finished plain shard is
// compressed in one pass and then unlinked by the caller.
//
// Compression runs multi-threaded when BAGWIZ_WRITE_THREADS selects two or
// more workers (the knob shared with the parallel mcap write path); MT frames
// are plain zstd on read, so consumers need nothing new. The knob's 0/1
// serial value keeps the legacy single-threaded byte layout.
//
// `level` is a zstd level (1..22; 0 selects ZSTD_defaultCLevel()).
// `dst` is created with truncation and must not equal `src`. On any failure
// the function throws std::runtime_error and removes a partially written
// `dst`, so callers never observe a torn envelope.
void compress_file_to_zstd(
  const std::filesystem::path & src, const std::filesystem::path & dst, int level = 0);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__FILE_COMPRESSOR_HPP_
