// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__MESSAGE_COMPRESSOR_HPP_
#define BAGWIZ__IO__MESSAGE_COMPRESSOR_HPP_

#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::io
{

// Forward declaration so the zstd C type does not leak into clients of this
// header. The deleter is defined in the implementation file.
struct MessageCompressorState;

// Per-message compressor for writing rosbag2 `compression_mode: MESSAGE`
// bags — the exact mirror of MessageDecompressor.
//
// rosbag2's per-message compression path (see rosbag2_compression_zstd's
// ZstdCompressor::compress_serialized_bag_message) wraps each message's
// serialized payload in a single zstd frame with no extra header. This class
// produces that byte shape: callers hand in the raw serialized payload and
// receive the compressed frame, ready to store as `messages.data` in SQLite3
// storage. MessageDecompressor reads these frames back.
//
// The returned span is **invalidated by the next call to compress()**;
// copy it if you need to outlive the next call.
//
// The class is move-only (Rule of Five): the ZSTD_CCtx held internally is
// expensive to allocate and cheap to reuse, matching upstream rosbag2's
// long-lived context pattern.
class MessageCompressor
{
public:
  // Construct a compressor for `format` at zstd level `level` (1..22; 0, the
  // default, selects ZSTD_defaultCLevel()). Only "zstd" is supported today;
  // anything else throws std::runtime_error with a clear diagnostic so the
  // factory can fail fast rather than silently producing wrong bytes.
  explicit MessageCompressor(std::string_view format, int level = 0);

  ~MessageCompressor();

  MessageCompressor(const MessageCompressor &) = delete;
  MessageCompressor & operator=(const MessageCompressor &) = delete;
  MessageCompressor(MessageCompressor &&) noexcept;
  MessageCompressor & operator=(MessageCompressor &&) noexcept;

  // Compresses `payload` into a single zstd frame. The returned span points
  // into an internal buffer owned by this object and is invalidated by the
  // next call to compress(). Throws std::runtime_error on a zstd failure.
  [[nodiscard]] std::span<const std::byte> compress(std::span<const std::byte> payload);

  // The format string this compressor was constructed with (e.g. "zstd").
  // Useful for diagnostics; never empty after construction.
  [[nodiscard]] const std::string & format() const noexcept { return format_; }

private:
  std::string format_;
  std::unique_ptr<MessageCompressorState> state_;
  std::vector<std::byte> out_buffer_;
};

// Map a CLI-facing compression level name ("fastest", "fast", "default",
// "slow", "slowest"; "" selects ZSTD_defaultCLevel()) onto a numeric zstd
// level. Shared by every zstd write path (per-message and whole-file) so the
// names mean the same thing everywhere. Throws std::runtime_error on an
// unknown name.
//
// The mapping stays monotone in the names' advertised order:
//   fastest = 1, fast = 2, default = ZSTD_defaultCLevel() (3), slow = 9,
//   slowest = 19.
[[nodiscard]] int zstd_level_from_name(std::string_view name);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__MESSAGE_COMPRESSOR_HPP_
