// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/file_compressor.hpp"

#include "mcap_parallel_chunk_writer.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <zstd.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

struct ZstdCStreamDeleter
{
  void operator()(ZSTD_CStream * ctx) const noexcept
  {
    if (ctx != nullptr) {
      ZSTD_freeCStream(ctx);
    }
  }
};
using ZstdCStreamPtr = std::unique_ptr<ZSTD_CStream, ZstdCStreamDeleter>;

// Owns `path` for the duration of the compression: on destruction without
// release(), the path is unlinked. Backs the "no torn envelope" guarantee of
// compress_file_to_zstd — any throw between creating `dst` and finishing the
// frame removes the partial output.
class PartialOutputGuard
{
public:
  explicit PartialOutputGuard(std::filesystem::path path) : path_(std::move(path)) {}
  ~PartialOutputGuard()
  {
    if (!path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(path_, ec);
    }
  }
  PartialOutputGuard(const PartialOutputGuard &) = delete;
  PartialOutputGuard & operator=(const PartialOutputGuard &) = delete;

  // The output finished cleanly; keep the file.
  void release() noexcept { path_.clear(); }

private:
  std::filesystem::path path_;
};

}  // namespace

void compress_file_to_zstd(
  const std::filesystem::path & src, const std::filesystem::path & dst, int level)
{
  if (src == dst) {
    throw std::runtime_error("zstd compress: source and destination must differ: " + src.string());
  }

  std::ifstream in(src, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open file for zstd compression: " + src.string());
  }

  ZstdCStreamPtr cctx{ZSTD_createCStream()};
  if (cctx == nullptr) {
    throw std::runtime_error("failed to allocate ZSTD_CStream for " + src.string());
  }
  const auto init_rc = ZSTD_CCtx_setParameter(cctx.get(), ZSTD_c_compressionLevel, level);
  if (ZSTD_isError(init_rc) != 0U) {
    throw std::runtime_error(
      std::string("failed to set zstd level for ") + src.string() + ": " +
      ZSTD_getErrorName(init_rc));
  }
  // Whole-database envelopes are large enough that single-threaded streaming
  // dominates the writer's close(): fan compression out over zstd's worker
  // pool instead. 0 or 1 workers keeps the single-threaded layout; >= 2 lets
  // zstd split the input into parallel jobs, which changes the frame's block
  // layout but not what it decompresses to.
  const int threads = detail::resolve_write_threads();
  if (threads >= 2) {
    const auto workers_rc = ZSTD_CCtx_setParameter(cctx.get(), ZSTD_c_nbWorkers, threads);
    if (ZSTD_isError(workers_rc) != 0U) {
      throw std::runtime_error(
        std::string("failed to set zstd worker count for ") + src.string() + ": " +
        ZSTD_getErrorName(workers_rc));
    }
  }

  std::ofstream out(dst, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open zstd output for writing: " + dst.string());
  }
  // From here on, a throw must not leave a partial `dst` behind.
  PartialOutputGuard partial(dst);

  const std::size_t in_chunk = ZSTD_CStreamInSize();
  const std::size_t out_chunk = ZSTD_CStreamOutSize();
  std::vector<std::byte> in_buf(in_chunk);
  std::vector<std::byte> out_buf(out_chunk);

  auto write_out = [&](std::size_t bytes) {
    out.write(reinterpret_cast<const char *>(out_buf.data()), static_cast<std::streamsize>(bytes));
    if (!out) {
      throw std::runtime_error("failed writing zstd output to " + dst.string());
    }
  };

  for (;;) {
    in.read(reinterpret_cast<char *>(in_buf.data()), static_cast<std::streamsize>(in_buf.size()));
    const auto got = in.gcount();
    if (got < 0) {
      throw std::runtime_error("failed reading from " + src.string());
    }
    const bool last_chunk = got == 0;

    ZSTD_inBuffer input{in_buf.data(), static_cast<std::size_t>(got), 0};
    while (input.pos < input.size) {
      ZSTD_outBuffer output{out_buf.data(), out_buf.size(), 0};
      const auto ret = ZSTD_compressStream2(
        cctx.get(), &output, &input, last_chunk ? ZSTD_e_end : ZSTD_e_continue);
      if (ZSTD_isError(ret) != 0U) {
        throw std::runtime_error(
          std::string("zstd compress failed for ") + src.string() + ": " + ZSTD_getErrorName(ret));
      }
      write_out(output.pos);
    }

    if (last_chunk) {
      // Drain the frame trailer: e_end must be called until it returns 0 so
      // the epilogue (and any buffered block) is fully flushed.
      for (;;) {
        ZSTD_outBuffer output{out_buf.data(), out_buf.size(), 0};
        const auto ret = ZSTD_compressStream2(cctx.get(), &output, &input, ZSTD_e_end);
        if (ZSTD_isError(ret) != 0U) {
          throw std::runtime_error(
            std::string("zstd compress finalization failed for ") + src.string() + ": " +
            ZSTD_getErrorName(ret));
        }
        write_out(output.pos);
        if (ret == 0) {
          break;
        }
      }
      break;
    }
  }

  out.flush();
  if (!out) {
    throw std::runtime_error("failed to flush zstd output to " + dst.string());
  }
  out.close();
  if (!out) {
    throw std::runtime_error("failed to close zstd output " + dst.string());
  }

  partial.release();
}

}  // namespace bagwiz::io
