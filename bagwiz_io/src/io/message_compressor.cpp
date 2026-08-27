// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/message_compressor.hpp"

#include <zstd.h>

#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{

constexpr std::string_view kZstd = "zstd";

struct ZstdCCtxDeleter
{
  void operator()(ZSTD_CCtx * ctx) const noexcept
  {
    if (ctx != nullptr) {
      ZSTD_freeCCtx(ctx);
    }
  }
};

using ZstdCCtxPtr = std::unique_ptr<ZSTD_CCtx, ZstdCCtxDeleter>;

}  // namespace

// Hide the zstd C type behind a thin opaque struct so the public header does
// not have to expose <zstd.h>. Mirrors MessageDecompressorState: only the
// long-lived compression context lives here; the reusable output buffer
// stays on MessageCompressor itself (it must stay accessible to the public
// span return value).
struct MessageCompressorState
{
  ZstdCCtxPtr context;
};

MessageCompressor::MessageCompressor(std::string_view format, int level)
: format_(format), state_(std::make_unique<MessageCompressorState>())
{
  if (format_ != kZstd) {
    throw std::runtime_error(
      "MessageCompressor: unsupported compression_format '" + format_ +
      "' (only 'zstd' is implemented for rosbag2 MESSAGE-mode)");
  }
  if (level < 0 || level > ZSTD_maxCLevel()) {
    throw std::runtime_error(
      "MessageCompressor: zstd level " + std::to_string(level) + " out of range (0.." +
      std::to_string(ZSTD_maxCLevel()) + "; 0 selects the library default)");
  }
  state_->context = ZstdCCtxPtr{ZSTD_createCCtx()};
  if (state_->context == nullptr) {
    throw std::runtime_error("MessageCompressor: failed to allocate ZSTD_CCtx");
  }
  const auto rc = ZSTD_CCtx_setParameter(state_->context.get(), ZSTD_c_compressionLevel, level);
  if (ZSTD_isError(rc) != 0U) {
    throw std::runtime_error(
      std::string{"MessageCompressor: failed to set zstd level: "} + ZSTD_getErrorName(rc));
  }
}

MessageCompressor::~MessageCompressor() = default;
MessageCompressor::MessageCompressor(MessageCompressor &&) noexcept = default;
MessageCompressor & MessageCompressor::operator=(MessageCompressor &&) noexcept = default;

std::span<const std::byte> MessageCompressor::compress(std::span<const std::byte> payload)
{
  // One-shot frame per message, matching rosbag2_compression_zstd's
  // ZstdCompressor::compress_serialized_bag_message (a bare zstd frame, no
  // extra header). The content-size field ends up embedded, which is what
  // lets MessageDecompressor take its single-shot fast path.
  out_buffer_.resize(ZSTD_compressBound(payload.size()));
  const auto written = ZSTD_compress2(
    state_->context.get(), out_buffer_.data(), out_buffer_.size(), payload.data(), payload.size());
  if (ZSTD_isError(written) != 0U) {
    throw std::runtime_error(
      std::string{"MessageCompressor: zstd compress failed: "} + ZSTD_getErrorName(written));
  }
  out_buffer_.resize(written);
  return {out_buffer_.data(), out_buffer_.size()};
}

int zstd_level_from_name(std::string_view name)
{
  if (name.empty() || name == "default") {
    return 0;  // zstd maps level 0 onto ZSTD_defaultCLevel()
  }
  if (name == "fastest") {
    return 1;
  }
  if (name == "fast") {
    return 2;
  }
  if (name == "slow") {
    return 9;
  }
  if (name == "slowest") {
    return 19;
  }
  throw std::runtime_error("unknown compression level: " + std::string(name));
}

}  // namespace bagwiz::io
