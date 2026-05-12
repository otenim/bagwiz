// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/atomic_replace.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>

namespace bagwiz::io
{

namespace
{

std::filesystem::path make_sibling_temp(const std::filesystem::path & target, std::string_view tag)
{
  const auto stem = target.filename().string();
  const auto ts = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::steady_clock::now().time_since_epoch())
                                               .count());
  std::string leaf;
  leaf.reserve(stem.size() + tag.size() + 32);
  leaf.append(".")
    .append(stem)
    .append(".bagwiz-")
    .append(tag)
    .append("-")
    .append(std::to_string(ts));
  std::filesystem::path parent = target.parent_path();
  if (parent.empty()) {
    parent = std::filesystem::current_path();
  }
  return parent / leaf;
}

}  // namespace

void atomic_replace(const std::filesystem::path & staged, const std::filesystem::path & target)
{
  std::error_code ec;

  if (!std::filesystem::exists(staged, ec) || ec) {
    throw std::system_error(
      ec ? ec : std::make_error_code(std::errc::no_such_file_or_directory),
      "atomic_replace: staged path does not exist: " + staged.string());
  }

  // Fast path: target absent → straight rename. We still try to use rename
  // (atomic when same FS), and fall back to a copy+remove only on
  // cross-device errors.
  if (!std::filesystem::exists(target, ec)) {
    ec.clear();
    std::filesystem::rename(staged, target, ec);
    if (!ec) {
      return;
    }
    if (ec == std::errc::cross_device_link) {
      std::filesystem::copy(
        staged, target,
        std::filesystem::copy_options::recursive |
          std::filesystem::copy_options::overwrite_existing,
        ec);
      if (ec) {
        throw std::system_error(ec, "atomic_replace: cross-device copy failed: " + staged.string());
      }
      std::error_code rm_ec;
      std::filesystem::remove_all(staged, rm_ec);
      return;
    }
    throw std::system_error(ec, "atomic_replace: rename failed: " + staged.string());
  }
  ec.clear();

  // Move the existing target aside so the staged path can take its name.
  // Both moves are renames on the same FS, so each is atomic on POSIX; the
  // composite operation is not, hence the rollback below.
  const auto backup = make_sibling_temp(target, "backup");

  std::filesystem::rename(target, backup, ec);
  if (ec) {
    throw std::system_error(ec, "atomic_replace: failed to move target aside: " + target.string());
  }

  std::filesystem::rename(staged, target, ec);
  if (ec) {
    // Rollback: put the original target back before propagating the error
    // so the caller observes the pre-replace state.
    std::error_code rb_ec;
    std::filesystem::rename(backup, target, rb_ec);
    throw std::system_error(
      ec, "atomic_replace: failed to move staged into place: " + staged.string());
  }

  std::error_code rm_ec;
  std::filesystem::remove_all(backup, rm_ec);
  // Best-effort cleanup; the swap already succeeded.
}

}  // namespace bagwiz::io
