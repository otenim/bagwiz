// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/bag/bag_inplace.hpp"

#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace bagwiz::core
{

namespace
{

// Name marker for the staging directory. Tests assert on it to prove no
// staging is left behind.
constexpr const char * kStagingPrefix = ".bagwiz-inplace-tmp-";

// Drop a trailing separator. A user-typed "drive_dir/" parses as an EMPTY
// filename() with "drive_dir" as the parent_path(), so every path derived
// from it lands *inside* the bag rather than beside it — which would put the
// staging directory inside the very bag the swap then removes, destroying the
// original and leaving nothing in its place. lexically_normal() also collapses
// "a//" and resolves "a/b/..", so one strip is enough afterwards.
std::filesystem::path strip_trailing_separator(const std::filesystem::path & path)
{
  const auto normalized = path.lexically_normal();
  return (normalized.filename().empty() && normalized.has_parent_path()) ? normalized.parent_path()
                                                                         : normalized;
}

// The staging directory beside `final_path`, plus the path inside it that the
// replacement bag is written to.
//
// The unique suffix goes on the DIRECTORY, never on the bag: the bag is
// staged under its own real name. That matters because the directory writers
// name their shard after the directory they are handed ("<dirname>_0.mcap")
// and record that name in metadata.yaml, so staging at
// "<name>.bagwiz-inplace-tmp-N" would bake the staging name into the bag that
// gets swapped in. Single-file bags keep their .mcap / .db3 extension for the
// same reason. pid + steady_clock ticks keep the directory from colliding
// with a concurrent invocation over the same parent.
struct TmpStaging
{
  std::filesystem::path dir;
  std::filesystem::path bag;
};

TmpStaging make_tmp_staging(const std::filesystem::path & final_path)
{
  const auto parent =
    final_path.has_parent_path() ? final_path.parent_path() : std::filesystem::current_path();
  const auto pid = static_cast<std::uint64_t>(::getpid());
  const auto ticks =
    static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto dir = parent / (kStagingPrefix + std::to_string(pid) + "-" + std::to_string(ticks));
  return {dir, dir / final_path.filename()};
}

// RAII guard that removes the staging directory on destruction. Unconditional
// by design: on the unhappy path it takes the half-written bag down with it,
// and on the happy path the bag has already been renamed out, so all that is
// left to remove is the empty directory.
class TmpDirGuard
{
public:
  explicit TmpDirGuard(std::filesystem::path path) : path_(std::move(path)) {}

  ~TmpDirGuard()
  {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  TmpDirGuard(const TmpDirGuard &) = delete;
  TmpDirGuard & operator=(const TmpDirGuard &) = delete;
  TmpDirGuard(TmpDirGuard &&) = delete;
  TmpDirGuard & operator=(TmpDirGuard &&) = delete;

  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
};

}  // namespace

void write_bag_inplace(
  const std::filesystem::path & final_path,
  const std::function<void(const std::filesystem::path & tmp_path)> & writer_fn)
{
  // Everything below derives from the normalised path: the swap removes and
  // renames onto it, and the staging directory is placed beside it.
  const auto target = strip_trailing_separator(final_path);

  // A path with no name of its own ("/", ".", "..") names a directory the
  // swap would remove wholesale rather than a bag inside it. Refuse before
  // anything is created or deleted.
  const auto name = target.filename().string();
  if (name.empty() || name == "." || name == "..") {
    throw std::runtime_error(
      "write_bag_inplace: refusing to rewrite '" + final_path.string() +
      "' in place: it does not name a bag");
  }

  if (!std::filesystem::exists(target)) {
    throw std::runtime_error("write_bag_inplace: final_path does not exist: " + target.string());
  }

  const auto staging = make_tmp_staging(target);
  // Guard first, create second: a create_directories that throws part-way
  // still gets cleaned up on the way out.
  TmpDirGuard guard(staging.dir);
  std::filesystem::create_directories(staging.dir);

  // Hand off to the caller. Any exception escapes; the guard wipes the
  // staging directory on the way out.
  writer_fn(staging.bag);

  // Sanity check: writer_fn must have materialised something at the
  // staged path. Without this check, a no-op writer would silently delete
  // final_path on the next step.
  if (!std::filesystem::exists(staging.bag)) {
    throw std::runtime_error(
      "write_bag_inplace: writer_fn returned successfully but produced no output at " +
      staging.bag.string());
  }

  // Simple one-shot swap. There is a brief window between remove_all
  // and rename during which a process crash leaves no bag at
  // final_path; the helper's contract documents this trade-off.
  std::error_code ec;
  std::filesystem::remove_all(target, ec);
  if (ec) {
    throw std::runtime_error(
      "write_bag_inplace: failed to remove final_path '" + target.string() + "': " + ec.message());
  }
  std::filesystem::rename(staging.bag, target, ec);
  if (ec) {
    // remove_all already wiped final_path; we cannot recover it from
    // here. Surface the situation explicitly so the caller can log a
    // recovery hint.
    throw std::runtime_error(
      "write_bag_inplace: failed to rename tmp '" + staging.bag.string() + "' to '" +
      target.string() + "' (original bag has been deleted): " + ec.message());
  }
  // The guard removes the staging directory, now empty, on the way out.
}

}  // namespace bagwiz::core
