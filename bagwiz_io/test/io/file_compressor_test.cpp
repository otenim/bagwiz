// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Unit coverage for compress_file_to_zstd, the whole-file zstd envelope writer
// behind rosbag2 compression_mode: FILE. The worker count comes from
// BAGWIZ_WRITE_THREADS via resolve_write_threads(): 0 or 1 keeps the
// single-threaded streaming layout, >= 2 enables zstd's multi-threaded job
// splitting (ZSTD_c_nbWorkers). Multi-threading changes the frame's internal
// block layout — the compressed bytes differ from the single-threaded output
// — but the frame stays a stock single zstd frame that decompresses to the
// same bytes, so the round-trip assertions below compare decompressed content
// rather than compressed bytes.

#include "bagwiz/io/file_compressor.hpp"

#include <gtest/gtest.h>
#include <zstd.h>

#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// 8 MiB of blocky, moderately compressible data: varied enough that zstd's
// multi-threaded job splitting (job size ~512 KiB at the default level)
// produces a different frame layout than the single-threaded stream.
std::vector<std::byte> make_input()
{
  constexpr std::size_t kSize = 8U * 1024U * 1024U;
  std::vector<std::byte> data(kSize);
  for (std::size_t i = 0; i < kSize; ++i) {
    data[i] = static_cast<std::byte>(((i / 4096) * 31 + (i % 4096) / 64) % 251);
  }
  return data;
}

void write_file(const std::filesystem::path & path, const std::vector<std::byte> & bytes)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(out.good());
  out.write(
    reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  out.flush();
  ASSERT_TRUE(out.good());
}

std::vector<std::byte> read_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  const std::vector<char> chars{
    std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  std::vector<std::byte> out(chars.size());
  std::memcpy(out.data(), chars.data(), chars.size());
  return out;
}

struct ZstdDStreamDeleter
{
  void operator()(ZSTD_DStream * ctx) const noexcept
  {
    if (ctx != nullptr) {
      ZSTD_freeDStream(ctx);
    }
  }
};

// Streaming decompress: the envelope frame carries no content-size field (the
// streaming writer learns the total only at the end), so one-shot
// ZSTD_decompress cannot size its output buffer.
std::vector<std::byte> zstd_decompress(const std::vector<std::byte> & compressed)
{
  std::unique_ptr<ZSTD_DStream, ZstdDStreamDeleter> dctx{ZSTD_createDStream()};
  if (dctx == nullptr) {
    throw std::runtime_error("failed to allocate ZSTD_DStream");
  }
  ZSTD_initDStream(dctx.get());

  std::vector<std::byte> out_buf(ZSTD_DStreamOutSize());
  std::vector<std::byte> out;
  ZSTD_inBuffer input{compressed.data(), compressed.size(), 0};
  std::size_t ret = 1;
  while (input.pos < input.size) {
    ZSTD_outBuffer output{out_buf.data(), out_buf.size(), 0};
    ret = ZSTD_decompressStream(dctx.get(), &output, &input);
    if (ZSTD_isError(ret) != 0U) {
      throw std::runtime_error(std::string("zstd decompress failed: ") + ZSTD_getErrorName(ret));
    }
    out.insert(out.end(), out_buf.data(), out_buf.data() + output.pos);
  }
  if (ret != 0) {
    throw std::runtime_error("truncated zstd frame");
  }
  return out;
}

std::vector<std::byte> compress_with_threads(
  const std::filesystem::path & dir, const std::string & name, const std::vector<std::byte> & input,
  const char * write_threads)
{
  ::setenv("BAGWIZ_WRITE_THREADS", write_threads, 1);
  const auto src = dir / (name + ".bin");
  const auto dst = dir / (name + ".zstd");
  write_file(src, input);
  bagwiz::io::compress_file_to_zstd(src, dst, /*level=*/3);
  return read_file(dst);
}

class FileCompressorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_file_compressor_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
  }
  void TearDown() override
  {
    ::unsetenv("BAGWIZ_WRITE_THREADS");
    std::filesystem::remove_all(tmp_);
  }

  std::filesystem::path tmp_;
};

}  // namespace

TEST_F(FileCompressorTest, MultiThreadedOutputRoundTripsToInput)
{
  const auto input = make_input();
  const auto compressed = compress_with_threads(tmp_, "mt", input, "4");

  EXPECT_EQ(zstd_decompress(compressed), input);
}

// A worker count >= 2 must actually reach ZSTD_c_nbWorkers: zstd's
// multi-threaded mode splits the input into independent compression jobs,
// which changes the frame's block layout relative to the single-threaded
// stream. Both layouts decompress to the same bytes.
TEST_F(FileCompressorTest, WorkerCountChangesCompressedLayout)
{
  const auto input = make_input();
  const auto serial = compress_with_threads(tmp_, "serial", input, "1");
  const auto parallel = compress_with_threads(tmp_, "parallel", input, "4");

  EXPECT_NE(serial, parallel);
  EXPECT_EQ(zstd_decompress(serial), input);
  EXPECT_EQ(zstd_decompress(parallel), input);
}
