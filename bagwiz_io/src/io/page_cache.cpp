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

#include "bagwiz/io/page_cache.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <string>
#include <system_error>
#include <unordered_set>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#endif

namespace bagwiz::io
{

namespace
{

struct Registry
{
  std::mutex mutex;
  std::unordered_set<std::filesystem::path> read_files;
  std::unordered_set<std::filesystem::path> written_files;
};

Registry & registry() noexcept
{
  // Intentionally leaked: drop_registered_file_caches() runs from std::atexit,
  // interleaved with static destruction, so the registry must never be
  // destroyed.
  static Registry * const reg = new Registry;
  return *reg;
}

// std::once_flag has a trivial destructor, so static storage is safe in the
// atexit context (unlike the registry contents).
std::once_flag install_once;

// Collapse `.`/`..` and symlinks so the same file registered through different
// spellings deduplicates. Falls back to the spelling given when canonicalization
// fails (the drop is best-effort anyway).
std::filesystem::path normalize(const std::filesystem::path & path) noexcept
{
  std::error_code ec;
  auto canon = std::filesystem::weakly_canonical(path, ec);
  return ec ? path : canon;
}

#ifdef __linux__
void drop_read_file(const std::filesystem::path & path) noexcept
{
  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    return;
  }
  // Length 0 covers the whole file. Only clean pages are dropped, which is
  // exactly what a read-only file has — no I/O is triggered.
  ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
  ::close(fd);
}

void drop_written_file(const std::filesystem::path & path) noexcept
{
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    return;
  }
  // Start writeback of everything still dirty, but do not wait: on hosts with
  // hundreds of GB of RAM the default dirty ratio lets gigabytes of a freshly
  // written file stay dirty, and a synchronous flush would stall process exit.
  // DONTNEED then drops the pages that are already clean; the kernel writes
  // back the remainder in the background.
  ::sync_file_range(fd, 0, 0, SYNC_FILE_RANGE_WRITE);
  ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
  ::close(fd);
}

// Applies drop_file to `path`, expanding a directory (a directory-layout bag)
// to the regular files directly inside — shard files and metadata.yaml. No
// recursion: bagwiz layouts never nest bags.
template <typename DropFn>
void drop_path(const std::filesystem::path & path, DropFn drop_file) noexcept
{
  std::error_code ec;
  if (!std::filesystem::is_directory(path, ec)) {
    drop_file(path);
    return;
  }
  std::filesystem::directory_iterator it(path, ec);
  const std::filesystem::directory_iterator end;
  for (; !ec && it != end; it.increment(ec)) {
    if (it->is_regular_file(ec)) {
      drop_file(it->path());
    }
  }
}
#endif  // __linux__

}  // namespace

// Same truth convention as the other on/off switches (BAGWIZ_PASSTHROUGH et
// al.): unset or empty keeps the default (enabled); "0"/"off"/"false"/"no",
// case-insensitive, disable.
bool page_cache_drop_enabled() noexcept
{
  const char * raw = std::getenv("BAGWIZ_PAGE_CACHE_DROP");
  if (raw == nullptr || *raw == '\0') {
    return true;
  }
  std::string value(raw);
  for (auto & c : value) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return value != "0" && value != "off" && value != "false" && value != "no";
}

void register_read_file(const std::filesystem::path & path) noexcept
{
  try {
    if (!page_cache_drop_enabled()) {
      return;
    }
    std::call_once(install_once, [] { std::atexit(&drop_registered_file_caches); });
    auto & reg = registry();
    std::lock_guard lock(reg.mutex);
    reg.read_files.insert(normalize(path));
  } catch (...) {
    // Best-effort: registration must never fail the command.
  }
}

void register_written_file(const std::filesystem::path & path) noexcept
{
  try {
    if (!page_cache_drop_enabled()) {
      return;
    }
    std::call_once(install_once, [] { std::atexit(&drop_registered_file_caches); });
    auto & reg = registry();
    std::lock_guard lock(reg.mutex);
    reg.written_files.insert(normalize(path));
  } catch (...) {
    // Best-effort: registration must never fail the command.
  }
}

void unregister_written_file(const std::filesystem::path & path) noexcept
{
  try {
    auto & reg = registry();
    std::lock_guard lock(reg.mutex);
    reg.written_files.erase(normalize(path));
  } catch (...) {
    // Best-effort: a leftover entry only repeats a harmless drop at exit.
  }
}

void drop_registered_file_caches() noexcept
{
  try {
    // Drain first, even when disabled, so tests that toggle the switch never
    // leak stale entries into a later drop.
    auto & reg = registry();
    std::unordered_set<std::filesystem::path> reads;
    std::unordered_set<std::filesystem::path> writtens;
    {
      std::lock_guard lock(reg.mutex);
      reads.swap(reg.read_files);
      writtens.swap(reg.written_files);
    }
#ifdef __linux__
    if (!page_cache_drop_enabled()) {
      return;
    }
    for (const auto & path : reads) {
      if (writtens.count(path) != 0) {
        continue;  // a path in both sets is handled as written below
      }
      drop_path(path, &drop_read_file);
    }
    for (const auto & path : writtens) {
      drop_path(path, &drop_written_file);
    }
#endif
  } catch (...) {
    // atexit context: never propagate.
  }
}

}  // namespace bagwiz::io
