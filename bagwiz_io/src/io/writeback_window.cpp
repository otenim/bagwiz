// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// sync_file_range() is a GNU extension; make sure its declaration is visible
// even if the compiler driver did not predefine _GNU_SOURCE. This must come
// before any system header.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "writeback_window.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/io/page_cache.hpp"

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif

namespace bagwiz::io::detail
{

WritebackWindow::WritebackWindow(
  const std::filesystem::path & path, std::uint64_t interval_bytes) noexcept
: path_(path), interval_(interval_bytes)
{
#ifdef __linux__
  if (interval_ == 0) {
    return;
  }
  // O_WRONLY because sync_file_range() expects a writable fd. The file must
  // already exist — the writer creates it before constructing the window.
  fd_ = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  // A failure here is fine: every method no-ops on fd_ < 0 and the exit-time
  // drop remains as the fallback.
#endif
}

WritebackWindow::~WritebackWindow()
{
#ifdef __linux__
  if (fd_ >= 0) {
    ::close(fd_);
  }
#endif
}

void WritebackWindow::note_offset(std::uint64_t offset) noexcept
{
#ifdef __linux__
  if (fd_ < 0 || finished_ || interval_ == 0) {
    return;
  }
  while (synced_below_ + interval_ <= offset) {
    // Start writeback of the interval that just completed; do not wait.
    ::sync_file_range(fd_, synced_below_, interval_, SYNC_FILE_RANGE_WRITE);
    synced_below_ += interval_;
  }
  if (!page_cache_drop_enabled()) {
    return;  // writeback pacing only: the master switch keeps the pages cached
  }
  // Drop every interval whose sync_file_range was issued at least one interval
  // ago: it has had a full interval of writeback time, so its pages are clean
  // (if still dirty, DONTNEED simply skips them and finish() gets another
  // chance). At most two intervals remain in flight or dirty at any time.
  while (dropped_below_ + 2 * interval_ <= synced_below_) {
    ::posix_fadvise(fd_, dropped_below_, interval_, POSIX_FADV_DONTNEED);
    dropped_below_ += interval_;
  }
#endif
}

void WritebackWindow::finish() noexcept
{
#ifdef __linux__
  if (fd_ < 0 || finished_) {
    return;
  }
  finished_ = true;
  if (page_cache_drop_enabled()) {
    // Wait for the remainder — bounded to a couple of intervals because
    // note_offset() has been starting writeback continuously — then drop the
    // whole file (length 0 covers to end-of-file for both calls).
    ::sync_file_range(fd_, 0, 0, SYNC_FILE_RANGE_WRITE | SYNC_FILE_RANGE_WAIT_AFTER);
    ::posix_fadvise(fd_, 0, 0, POSIX_FADV_DONTNEED);
    unregister_written_file(path_);
  }
  ::close(fd_);
  fd_ = -1;
#endif
}

}  // namespace bagwiz::io::detail
