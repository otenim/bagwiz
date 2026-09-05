// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__STORAGE_SNIFF_HPP_
#define IO__STORAGE_SNIFF_HPP_

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <filesystem>
#include <string>

// Src-local helpers that read a bag's storage shape from its first bytes, its
// extension, or its metadata.yaml declaration — without opening a reader.
// Shared by the open_read() factory (bag_factory.cpp) and describe_bag()
// (bag_describe.cpp) so both classify a bag the same way.
namespace bagwiz::io::detail
{

// ASCII-lowercased copy of `s`, for case-insensitive comparison of the
// compression mode / format strings (rosbag2 writes uppercase enum names,
// bagwiz writes lowercase).
std::string to_lower_copy(std::string s);

// Magic-byte sniff: opens `path`, reads up to 16 bytes, and matches the
// MCAP / SQLite3 prefix. Returns Format::Auto on any failure (open error,
// short read, no match) so callers can fall through to higher-level
// diagnostics.
Format sniff_file_magic(const std::filesystem::path & path) noexcept;

// Resolve the storage format hidden inside a single-file zstd envelope from
// its extension alone, by stripping a trailing `.zstd` and re-inferring
// (e.g. `foo.db3.zstd` -> Sqlite3, `foo.mcap.zstd` -> Mcap). Returns
// Format::Auto when the inner extension is not recognised. Cheap and never
// touches file contents.
Format infer_inner_format_from_zstd_extension(const std::filesystem::path & path) noexcept;

// True when `md` describes a rosbag2 `compression_mode: FILE` whole-database
// zstd envelope over sqlite3 storage (the `.db3.zstd` case). Case-insensitive
// on the mode string to tolerate both rosbag2's uppercase enum names and
// bagwiz's lowercase output.
bool is_sqlite3_file_zstd_envelope(const BagMetadata & md);

}  // namespace bagwiz::io::detail

#endif  // IO__STORAGE_SNIFF_HPP_
