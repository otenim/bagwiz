// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__BAG_DESCRIBE_HPP_
#define BAGWIZ__IO__BAG_DESCRIBE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::io
{

// One storage file of a bag: a directory bag's shard, or the single file
// itself.
struct BagFileDescription
{
  // Relative to the bag directory for a shard; the file's own name for a
  // single-file bag.
  std::filesystem::path path;
  // On-disk bytes. nullopt when metadata.yaml lists the file but it is not
  // on disk.
  std::optional<std::uint64_t> size_bytes;
  // The file's own message-time extent and count, when a summary records
  // them (a metadata.yaml `files:` entry, an MCAP shard's Statistics record).
  // nullopt otherwise; never derived by reading messages.
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
  std::optional<std::int64_t> message_count;
};

// How a bag's bytes are compressed, in the vocabulary `bagwiz compress`
// uses.
struct BagCompression
{
  // "none": plain storage. "chunk": MCAP chunk compression. "message":
  // rosbag2 MESSAGE mode (per-message frames). "file": rosbag2 FILE mode
  // (whole-shard `.db3.zstd` envelope). "unknown": the codec could not be
  // read without scanning (an MCAP with no summary).
  std::string mode;
  // Codecs in use, sorted and unique: {"zstd"}, {"lz4"}; several when an
  // MCAP mixes codecs across its chunks. Empty for "none" and "unknown".
  std::vector<std::string> codecs;
  // MCAP chunk totals from the chunk index, so callers can report a ratio.
  // nullopt for every other mode.
  std::optional<std::uint64_t> compressed_bytes;
  std::optional<std::uint64_t> uncompressed_bytes;
};

// Bag-level facts read from a bag's own summaries and its file system
// entries alone: metadata.yaml, the MCAP summary section, a .db3's `metadata`
// table and timestamp index, and stat(2). Nothing here comes from reading
// message records, so what a bag does not summarise stays unknown
// (nullopt / empty optional) rather than being computed.
struct BagDescription
{
  std::filesystem::path path;
  Layout layout = Layout::Auto;  // SingleFile or Directory
  Format format = Format::Auto;  // Mcap or Sqlite3; Auto when undetectable
  BagCompression compression;
  // The bag's storage files in play order; one entry for a single file.
  std::vector<BagFileDescription> files;
  std::uint64_t size_bytes = 0;  // sum of files[].size_bytes
  // Directory bags only: whether metadata.yaml was present, and the
  // `version` / `ros_distro` it declares (nullopt / empty when it does not).
  bool has_metadata_yaml = false;
  std::optional<int> metadata_version;
  std::string ros_distro;
  // Topic names and types. nullopt when the topic list is not reachable
  // without reading messages (a bare `.db3.zstd` envelope, an MCAP with no
  // summary).
  std::optional<std::vector<TopicInfo>> topics;
  // Bag-level summary. message_count is nullopt when no summary states it
  // (a humble-era .db3 without a `metadata` row). start_ns / end_ns are
  // nullopt when unknown or when the bag holds no messages.
  std::optional<std::int64_t> message_count;
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
};

// Describe the bag at `path` without reading message records. Throws
// std::runtime_error when `path` does not exist, or when a directory bag's
// metadata.yaml cannot be parsed. A file whose format cannot be recognised
// is returned with Format::Auto rather than rejected.
BagDescription describe_bag(const std::filesystem::path & path);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__BAG_DESCRIBE_HPP_
