// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_describe.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_decompressor.hpp"
#include "bagwiz/io/metadata_computer.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "bagwiz/io/sqlite3_helpers.hpp"
#include "storage_sniff.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/reader.hpp>

#include <sqlite3.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace bagwiz::io
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.describe";

// What one storage file's own summary structures say about it: the MCAP
// summary section, or a .db3's `topics` table, `metadata` row and timestamp
// index. Every field stays unset when the file does not record it; nothing
// is scanned to fill it in.
struct FileProbe
{
  // The file's summary was found and parsed (an MCAP summary section, a .db3
  // `metadata` row). False leaves the codec / count unknown.
  bool summary_ok = false;
  std::optional<std::vector<TopicInfo>> topics;
  std::optional<std::int64_t> message_count;
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
  // MCAP chunk index totals. `codecs` holds one entry per chunk, "" for a
  // chunk stored plain.
  std::vector<std::string> codecs;
  std::uint64_t compressed_bytes = 0;
  std::uint64_t uncompressed_bytes = 0;
  // A .db3's embedded `metadata` row, when it has a usable one.
  std::optional<BagMetadata> embedded;
};

std::optional<std::uint64_t> file_size_or_nullopt(const std::filesystem::path & path)
{
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(size);
}

void sort_by_name(std::vector<TopicInfo> & topics)
{
  std::sort(topics.begin(), topics.end(), [](const TopicInfo & a, const TopicInfo & b) {
    return a.name < b.name;
  });
}

Format format_from_identifier(const std::string & storage_identifier)
{
  if (storage_identifier == "mcap") {
    return Format::Mcap;
  }
  if (storage_identifier == "sqlite3") {
    return Format::Sqlite3;
  }
  return Format::Auto;
}

// ---------------------------------------------------------------------------
// MCAP: everything comes from the summary section. NoFallbackScan keeps a bag
// without one (never finalized) from being scanned; it reads as unknown.
// ---------------------------------------------------------------------------
FileProbe probe_mcap(const std::filesystem::path & file)
{
  FileProbe probe;
  mcap::McapReader reader;
  if (const auto status = reader.open(file.string()); !status.ok()) {
    BAGWIZ_LOG_WARN(kLogger, "%s: mcap open failed: %s", file.c_str(), status.message.c_str());
    return probe;
  }
  if (const auto status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
      !status.ok()) {
    BAGWIZ_LOG_INFO(
      kLogger, "%s: no readable summary section (%s); contents reported as unknown", file.c_str(),
      status.message.c_str());
    return probe;
  }
  probe.summary_ok = true;

  std::vector<TopicInfo> topics;
  const auto channels = reader.channels();
  const auto schemas = reader.schemas();
  topics.reserve(channels.size());
  for (const auto & entry : channels) {
    const auto & channel = entry.second;
    TopicInfo info;
    info.name = channel->topic;
    info.serialization_format = channel->messageEncoding;
    if (const auto schema_it = schemas.find(channel->schemaId); schema_it != schemas.end()) {
      info.type = schema_it->second->name;
      info.schema_encoding = schema_it->second->encoding;
    }
    topics.push_back(std::move(info));
  }
  sort_by_name(topics);
  probe.topics = std::move(topics);

  if (const auto & statistics = reader.statistics(); statistics.has_value()) {
    probe.message_count = static_cast<std::int64_t>(statistics->messageCount);
    if (statistics->messageCount > 0) {
      probe.start_ns = static_cast<std::int64_t>(statistics->messageStartTime);
      probe.end_ns = static_cast<std::int64_t>(statistics->messageEndTime);
    }
  }

  for (const auto & chunk : reader.chunkIndexes()) {
    probe.codecs.push_back(chunk.compression);
    probe.compressed_bytes += chunk.compressedSize;
    probe.uncompressed_bytes += chunk.uncompressedSize;
  }
  return probe;
}

// ---------------------------------------------------------------------------
// SQLite3: the `topics` table lists topics, the `metadata` row (rosbag2 iron+
// and every bagwiz writer) carries the summary, and the timestamp index
// answers the extent when that row is absent. The count is never recomputed:
// a humble-era .db3 without the row reads as count unknown.
// ---------------------------------------------------------------------------

// True on SQLITE_ROW, false on SQLITE_DONE; throws on any other result so a
// broken database is reported rather than read as empty.
bool step_or_throw(sqlite3 * db, sqlite3_stmt * stmt)
{
  const int rc = sqlite3_step(stmt);
  if (rc == SQLITE_ROW) {
    return true;
  }
  if (rc == SQLITE_DONE) {
    return false;
  }
  throw std::runtime_error("sqlite3_step failed: " + detail::sqlite_errmsg(db));
}

std::string column_text(sqlite3_stmt * stmt, int column)
{
  const unsigned char * text = sqlite3_column_text(stmt, column);
  if (text == nullptr) {
    return {};
  }
  return reinterpret_cast<const char *>(
    text);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
}

bool table_exists(sqlite3 * db, const char * table)
{
  auto stmt = detail::sqlite_prepare_or_throw(
    db, "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?");
  sqlite3_bind_text(stmt.get(), 1, table, -1, SQLITE_STATIC);
  return step_or_throw(db, stmt.get());
}

FileProbe probe_sqlite3(const std::filesystem::path & file)
{
  FileProbe probe;
  auto db =
    detail::sqlite_open_or_throw(file.string(), SQLITE_OPEN_READONLY, "sqlite3 open for describe");

  std::vector<TopicInfo> topics;
  {
    auto stmt = detail::sqlite_prepare_or_throw(
      db.get(), "SELECT name, type, serialization_format FROM topics ORDER BY id");
    while (step_or_throw(db.get(), stmt.get())) {
      TopicInfo info;
      info.name = column_text(stmt.get(), 0);
      info.type = column_text(stmt.get(), 1);
      info.serialization_format = column_text(stmt.get(), 2);
      topics.push_back(std::move(info));
    }
  }
  sort_by_name(topics);
  probe.topics = std::move(topics);

  // Last row wins: rosbag2 inserts a template row at open and the final
  // summary at close. parse_metadata_yaml_body rejects the template.
  if (table_exists(db.get(), "metadata")) {
    auto stmt = detail::sqlite_prepare_or_throw(
      db.get(), "SELECT metadata FROM metadata ORDER BY id DESC LIMIT 1");
    if (step_or_throw(db.get(), stmt.get())) {
      if (auto md = parse_metadata_yaml_body(column_text(stmt.get(), 0)); md.has_value()) {
        probe.summary_ok = true;
        probe.message_count = md->total_messages;
        if (md->total_messages > 0) {
          probe.start_ns = md->start_ns;
          probe.end_ns = md->end_ns;
        }
        probe.embedded = std::move(md);
      }
    }
  }

  // MIN/MAX over an indexed column read the two ends of timestamp_idx; a
  // COUNT(*) would walk the whole index, so the count stays unknown here.
  if (!probe.summary_ok) {
    auto stmt = detail::sqlite_prepare_or_throw(
      db.get(), "SELECT MIN(timestamp), MAX(timestamp) FROM messages");
    if (step_or_throw(db.get(), stmt.get()) && sqlite3_column_type(stmt.get(), 0) != SQLITE_NULL) {
      probe.start_ns = sqlite3_column_int64(stmt.get(), 0);
      probe.end_ns = sqlite3_column_int64(stmt.get(), 1);
    }
  }
  return probe;
}

// Probe one shard of a directory bag. A shard that cannot be opened (listed
// in metadata.yaml but missing, or corrupt) is logged and reads as unknown
// rather than failing the whole description.
FileProbe probe_shard(const std::filesystem::path & shard, Format format)
{
  try {
    return format == Format::Mcap ? probe_mcap(shard) : probe_sqlite3(shard);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_WARN(kLogger, "%s: not probed: %s", shard.c_str(), e.what());
    return FileProbe{};
  }
}

// ---------------------------------------------------------------------------
// Folding several shards' probes into bag-level answers. Each field is
// all-or-nothing: a value derived from some shards but not others would read
// as a fact about the whole bag, so it stays unknown instead.
// ---------------------------------------------------------------------------
std::optional<std::vector<TopicInfo>> union_topics(const std::vector<FileProbe> & probes)
{
  std::vector<TopicInfo> topics;
  for (const auto & probe : probes) {
    if (!probe.topics.has_value()) {
      return std::nullopt;
    }
    for (const auto & topic : *probe.topics) {
      const bool seen = std::any_of(topics.begin(), topics.end(), [&topic](const TopicInfo & t) {
        return t.name == topic.name;
      });
      if (!seen) {
        topics.push_back(topic);
      }
    }
  }
  sort_by_name(topics);
  return topics;
}

std::optional<std::int64_t> sum_counts(const std::vector<FileProbe> & probes)
{
  std::int64_t total = 0;
  for (const auto & probe : probes) {
    if (!probe.message_count.has_value()) {
      return std::nullopt;
    }
    total += *probe.message_count;
  }
  return total;
}

struct Extent
{
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
};

// The bag's extent spans its shards' extents. A shard known to hold no
// messages contributes nothing; a shard whose extent is unknown makes the
// bag's unknown.
Extent fold_extent(const std::vector<FileProbe> & probes)
{
  Extent extent;
  for (const auto & probe : probes) {
    const bool known_empty = probe.message_count.has_value() && *probe.message_count == 0;
    if (known_empty) {
      continue;
    }
    if (!probe.start_ns.has_value() || !probe.end_ns.has_value()) {
      return Extent{};
    }
    extent.start_ns =
      extent.start_ns ? std::min(*extent.start_ns, *probe.start_ns) : *probe.start_ns;
    extent.end_ns = extent.end_ns ? std::max(*extent.end_ns, *probe.end_ns) : *probe.end_ns;
  }
  return extent;
}

// MCAP chunk compression across every shard: the sorted, unique, non-empty
// codec names, with the chunk index byte totals behind them. Any shard whose
// summary could not be read leaves the codec unknown.
BagCompression fold_mcap_compression(const std::vector<FileProbe> & probes)
{
  BagCompression compression;
  std::vector<std::string> codecs;
  std::uint64_t compressed = 0;
  std::uint64_t uncompressed = 0;
  for (const auto & probe : probes) {
    if (!probe.summary_ok) {
      compression.mode = "unknown";
      return compression;
    }
    for (const auto & codec : probe.codecs) {
      if (!codec.empty()) {
        codecs.push_back(codec);
      }
    }
    compressed += probe.compressed_bytes;
    uncompressed += probe.uncompressed_bytes;
  }
  std::sort(codecs.begin(), codecs.end());
  codecs.erase(std::unique(codecs.begin(), codecs.end()), codecs.end());
  if (codecs.empty()) {
    compression.mode = "none";
    return compression;
  }
  compression.mode = "chunk";
  compression.codecs = std::move(codecs);
  compression.compressed_bytes = compressed;
  compression.uncompressed_bytes = uncompressed;
  return compression;
}

// rosbag2-layer compression as metadata.yaml (or a .db3 `metadata` row)
// declares it. nullopt when the declaration names no compression, so the
// caller falls through to the storage's own (MCAP chunk) compression.
std::optional<BagCompression> declared_compression(const BagMetadata & md, Format format)
{
  const std::string mode = detail::to_lower_copy(md.compression_mode);
  const std::string codec = detail::to_lower_copy(md.compression_format);
  BagCompression compression;
  if (mode == "message") {
    compression.mode = "message";
  } else if (mode == "file" && format == Format::Sqlite3) {
    // FILE mode over MCAP is a declaration rosbag2 tolerates but never acts
    // on (libmcap decompresses chunks itself), so it is not a compression
    // of the bag; only the sqlite3 `.db3.zstd` envelope is.
    compression.mode = "file";
  } else {
    return std::nullopt;
  }
  if (!codec.empty()) {
    compression.codecs.push_back(codec);
  }
  return compression;
}

// The `files:` entry for the shard at `rel`, if metadata.yaml has one. A
// FILE-mode bag lists the on-disk `.db3.zstd` name in relative_file_paths but
// the logical `.db3` name under files[].path, so both spellings are tried.
const BagMetadataFile * find_file_entry(const BagMetadata & md, const std::filesystem::path & rel)
{
  const std::filesystem::path logical =
    rel.extension() == ".zstd" ? rel.parent_path() / rel.stem() : rel;
  for (const auto & entry : md.files) {
    if (entry.path == rel || entry.path == logical) {
      return &entry;
    }
  }
  return nullptr;
}

BagDescription describe_directory(const std::filesystem::path & dir)
{
  BagDescription d;
  d.path = dir;
  d.layout = Layout::Directory;

  // metadata.yaml when present, otherwise the same reconstruction open_read()
  // uses: a directory listing plus a magic-byte sniff, no shard opened.
  const auto yaml_path = dir / "metadata.yaml";
  std::error_code ec;
  d.has_metadata_yaml = std::filesystem::exists(yaml_path, ec);
  const BagMetadata md =
    d.has_metadata_yaml ? load_metadata_yaml(yaml_path) : MetadataComputer::compute(dir);
  d.metadata_version = md.version;
  d.ros_distro = md.ros_distro;
  d.format = format_from_identifier(md.storage_identifier);

  for (const auto & rel : md.relative_file_paths) {
    BagFileDescription file;
    file.path = rel;
    file.size_bytes = file_size_or_nullopt(dir / rel);
    if (const auto * entry = find_file_entry(md, rel); entry != nullptr && entry->has_summary) {
      file.message_count = entry->message_count;
      if (entry->message_count > 0) {
        file.start_ns = entry->start_ns;
        file.end_ns = entry->end_ns;
      }
    }
    d.size_bytes += file.size_bytes.value_or(0);
    d.files.push_back(std::move(file));
  }

  if (!md.topics.empty()) {
    d.topics = md.topics;
  }
  if (md.has_summary) {
    d.message_count = md.total_messages;
    if (md.total_messages > 0) {
      d.start_ns = md.start_ns;
      d.end_ns = md.end_ns;
    }
  }

  // Shards are opened only for what metadata.yaml cannot say. An MCAP's
  // chunk codec lives in its chunk index alone, so MCAP shards are always
  // summarised (a footer read each). A sqlite3 shard is opened only when the
  // topic list or the summary is missing, and never when it is a `.db3.zstd`
  // envelope, which cannot be read without decompressing it whole.
  const auto declared = declared_compression(md, d.format);
  const bool envelope = declared.has_value() && declared->mode == "file";
  const bool summary_missing = !d.topics.has_value() || !md.has_summary;
  const bool probe_shards =
    d.format == Format::Mcap || (d.format == Format::Sqlite3 && !envelope && summary_missing);

  std::vector<FileProbe> probes;
  if (probe_shards) {
    probes.reserve(d.files.size());
    for (auto & file : d.files) {
      probes.push_back(probe_shard(dir / file.path, d.format));
      const auto & probe = probes.back();
      if (!file.message_count.has_value()) {
        file.message_count = probe.message_count;
      }
      if (!file.start_ns.has_value()) {
        file.start_ns = probe.start_ns;
        file.end_ns = probe.end_ns;
      }
    }
    if (!d.topics.has_value()) {
      d.topics = union_topics(probes);
    }
    if (!md.has_summary) {
      d.message_count = sum_counts(probes);
      const auto extent = fold_extent(probes);
      d.start_ns = extent.start_ns;
      d.end_ns = extent.end_ns;
    }
  }

  if (declared.has_value()) {
    d.compression = *declared;
  } else if (d.format == Format::Mcap) {
    d.compression = fold_mcap_compression(probes);
  } else {
    d.compression.mode = "none";
  }
  return d;
}

// A single file is the whole bag, so its probe answers both the bag-level
// fields and the file's own row.
void apply_single_file_probe(BagDescription & d, const FileProbe & probe)
{
  d.topics = probe.topics;
  d.message_count = probe.message_count;
  d.start_ns = probe.start_ns;
  d.end_ns = probe.end_ns;
  auto & self = d.files.front();
  self.message_count = probe.message_count;
  self.start_ns = probe.start_ns;
  self.end_ns = probe.end_ns;
}

BagDescription describe_single_file(const std::filesystem::path & file)
{
  BagDescription d;
  d.path = file;
  d.layout = Layout::SingleFile;
  BagFileDescription self;
  self.path = file.filename();
  self.size_bytes = file_size_or_nullopt(file);
  d.size_bytes = self.size_bytes.value_or(0);
  d.files.push_back(std::move(self));

  const Format sniffed = detail::sniff_file_magic(file);
  if (sniffed == Format::Auto && is_zstd_file(file)) {
    // A bare FILE-mode envelope. Its contents (topics, summary) sit behind a
    // whole-database decompression, so the description stops at the file.
    d.format = detail::infer_inner_format_from_zstd_extension(file);
    d.compression.mode = "file";
    d.compression.codecs = {"zstd"};
    return d;
  }
  d.format = sniffed;

  if (d.format == Format::Mcap) {
    const auto probe = probe_mcap(file);
    apply_single_file_probe(d, probe);
    d.compression = fold_mcap_compression({probe});
    return d;
  }
  if (d.format == Format::Sqlite3) {
    const auto probe = probe_sqlite3(file);
    apply_single_file_probe(d, probe);
    // A shard lifted out of a MESSAGE-mode bag declares that mode in its own
    // `metadata` row; a plain .db3 declares nothing.
    const auto declared =
      probe.embedded.has_value() ? declared_compression(*probe.embedded, d.format) : std::nullopt;
    if (declared.has_value() && declared->mode == "message") {
      d.compression = *declared;
    } else {
      d.compression.mode = "none";
    }
    return d;
  }
  d.compression.mode = "unknown";
  return d;
}

}  // namespace

BagDescription describe_bag(const std::filesystem::path & path)
{
  std::error_code ec;
  const bool exists = std::filesystem::exists(path, ec);
  if (ec || !exists) {
    throw std::runtime_error("path does not exist: " + path.string());
  }
  if (std::filesystem::is_directory(path, ec)) {
    return describe_directory(path);
  }
  return describe_single_file(path);
}

}  // namespace bagwiz::io
