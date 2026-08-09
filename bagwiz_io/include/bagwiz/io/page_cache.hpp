// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__PAGE_CACHE_HPP_
#define BAGWIZ__IO__PAGE_CACHE_HPP_

#include <filesystem>

namespace bagwiz::io
{

// Page-cache hygiene for batch workloads.
//
// A bagwiz process reads and writes each bag file exactly once, yet every byte
// passes through the kernel page cache and stays there after the process
// exits. In a batch loop over many bags (a shell `for` running one bagwiz
// process per bag, for example) those dead pages accumulate until free memory
// runs out, at which point reclaim and writeback stalls slow the whole batch
// down.
//
// The functions below let the I/O layer record which files it touched; at
// process exit (a std::atexit handler installed on first registration) the
// recorded pages are handed back to the kernel with
// posix_fadvise(POSIX_FADV_DONTNEED), preceded by a sync_file_range() nudge
// for written files. Dropping at exit — rather than while reading — keeps
// multi-pass commands fast: `map slam` and `generate video` re-open the same
// bag several times within one process and rely on the cache for every pass
// after the first.
//
// Everything is best-effort: registration and dropping never fail the command
// (a drop error only means the kernel keeps the pages, which is the behavior
// bagwiz has always had). Set BAGWIZ_PAGE_CACHE_DROP=0 (or off/false/no) to
// restore the pre-hygiene behavior. Implemented for Linux only; on other
// platforms registration and dropping compile to no-ops. Directory paths are
// expanded to the regular files directly inside at drop time, which covers
// directory-layout bags (shard files + metadata.yaml).

// Records `path` as read by this process; its clean pages are dropped at
// process exit. open_read() calls this, so every bag read through the
// BagReader factory is covered.
void register_read_file(const std::filesystem::path & path) noexcept;

// Whether the cache drop is currently enabled: false when
// BAGWIZ_PAGE_CACHE_DROP is set to 0/off/false/no (case-insensitive), true
// otherwise. Read on every use, so a change takes effect mid-process. Besides
// the exit-time pass, the streaming writers consult this for their drop step
// (their sync_file_range() writeback pacing stays on regardless — it evicts
// nothing).
bool page_cache_drop_enabled() noexcept;

// Records `path` as written by this process; at process exit its writeback is
// started with sync_file_range() (without waiting) and its already-clean
// pages are dropped. open_write() calls this; writers that bypass open_write()
// register themselves.
void register_written_file(const std::filesystem::path & path) noexcept;

// Removes `path` from the written-file set. A writer that manages its own
// writeback and drop (detail::WritebackWindow) calls this once finished, so
// the exit-time pass does not repeat the work.
void unregister_written_file(const std::filesystem::path & path) noexcept;

// Drops the page cache of every registered file and clears the registry. This
// is the std::atexit handler body; tests also call it directly. Runs before or
// after static destruction depending on registration order, so it must not
// touch any object with static storage duration — and therefore never logs.
void drop_registered_file_caches() noexcept;

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__PAGE_CACHE_HPP_
