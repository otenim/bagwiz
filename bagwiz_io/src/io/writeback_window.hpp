// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__WRITEBACK_WINDOW_HPP_
#define IO__WRITEBACK_WINDOW_HPP_

#include "env_tuning.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstdint>
#include <filesystem>

namespace bagwiz::io::detail
{

// Bounds the dirty-page backlog of a sequentially written output file.
//
// Without this, a bag rewrite can leave its whole output sitting dirty in the
// page cache — the default dirty ratio scales with RAM, so on a host with
// hundreds of GB of memory the kernel never throttles the writer — where the
// pages both inflate buff/cache and set up a long writeback stall later.
// note_offset() is called with the current file offset after each write;
// every time a further `interval_bytes` have been written, the window issues
// sync_file_range(SYNC_FILE_RANGE_WRITE) for the interval just completed,
// starting writeback without waiting for it. finish() waits for the bounded
// remainder (SYNC_FILE_RANGE_WAIT_AFTER — a couple of intervals at most, so
// the wait stays short).
//
// The window needs no access to the writer's own fd: it opens a separate
// management fd on the path, and page-cache state is per-inode. Every
// operation is a best-effort no-op when the open failed or interval_bytes is
// 0. Linux-only; other platforms compile to no-ops. Not thread-safe: call
// note_offset()/finish() from the single thread that writes the file.
class WritebackWindow
{
public:
  WritebackWindow(const std::filesystem::path & path, std::uint64_t interval_bytes) noexcept;
  // Closes the management fd. Deliberately does NOT finish(): on an error
  // path the output is incomplete, so there is no bounded remainder worth
  // waiting for.
  ~WritebackWindow();

  WritebackWindow(const WritebackWindow &) = delete;
  WritebackWindow & operator=(const WritebackWindow &) = delete;
  WritebackWindow(WritebackWindow &&) = delete;
  WritebackWindow & operator=(WritebackWindow &&) = delete;

  // Records that the bytes below `offset` have been written, and issues
  // sync_file_range() for every interval completed since the last call.
  void note_offset(std::uint64_t offset) noexcept;

  // Flushes the bounded remainder and closes the fd. Idempotent. When the
  // window is disabled or its fd could not be opened, this is a no-op beyond
  // closing the fd.
  void finish() noexcept;

private:
  int fd_ = -1;
  std::uint64_t interval_ = 0;
  std::uint64_t synced_below_ = 0;  // [0, synced_below_): sync_file_range issued
  bool finished_ = false;
};

// Writeback interval for the streaming writers, from
// BAGWIZ_WRITEBACK_INTERVAL_BYTES. Defaults to 256 MiB; 0 disables the window.
inline std::uint64_t resolve_writeback_interval_bytes(const char * logger)
{
  constexpr std::int64_t kDefault = 256ll * 1024 * 1024;
  constexpr std::int64_t kMax = 16ll * 1024 * 1024 * 1024;
  return static_cast<std::uint64_t>(
    resolve_env_int("BAGWIZ_WRITEBACK_INTERVAL_BYTES", kDefault, 0, kMax, logger));
}

}  // namespace bagwiz::io::detail

#endif  // IO__WRITEBACK_WINDOW_HPP_
