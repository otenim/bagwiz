// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "storage_sniff.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <array>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

namespace bagwiz::io::detail
{

namespace
{
// MCAP magic prefix: 0x89, 'M', 'C', 'A', 'P', 0x30
constexpr std::array<unsigned char, 6> kMcapMagic = {0x89, 'M', 'C', 'A', 'P', '0'};

// SQLite3 header prefix (first 16 bytes are "SQLite format 3\0").
constexpr const char * kSqliteMagic = "SQLite format 3";
}  // namespace

std::string to_lower_copy(std::string s)
{
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

Format sniff_file_magic(const std::filesystem::path & path) noexcept
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return Format::Auto;
  }
  std::array<char, 16> buf{};
  f.read(buf.data(), buf.size());
  const auto read = f.gcount();
  if (read < 0) {
    return Format::Auto;
  }
  const auto bytes = static_cast<std::size_t>(read);

  if (bytes >= kMcapMagic.size()) {
    if (std::memcmp(buf.data(), kMcapMagic.data(), kMcapMagic.size()) == 0) {
      return Format::Mcap;
    }
  }
  const auto sqlite_len = std::strlen(kSqliteMagic);
  if (bytes >= sqlite_len) {
    if (std::memcmp(buf.data(), kSqliteMagic, sqlite_len) == 0) {
      return Format::Sqlite3;
    }
  }
  return Format::Auto;
}

Format infer_inner_format_from_zstd_extension(const std::filesystem::path & path) noexcept
{
  if (to_lower_copy(path.extension().string()) != ".zstd") {
    return Format::Auto;
  }
  return infer_format_from_extension(path.stem());
}

bool is_sqlite3_file_zstd_envelope(const BagMetadata & md)
{
  return to_lower_copy(md.compression_mode) == "file" && md.storage_identifier == "sqlite3";
}

}  // namespace bagwiz::io::detail
