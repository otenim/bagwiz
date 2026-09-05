// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/mcap_writer.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"
#include "mcap_parallel_chunk_writer.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <mcap/writer.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::io::detail
{

namespace
{
constexpr const char * kLogger = "bagwiz.io.mcap";

// Transparent string hashing so per-message channel/count lookups can find()
// by string_view without materializing a std::string per message.
struct StringHash
{
  using is_transparent = void;
  size_t operator()(std::string_view v) const { return std::hash<std::string_view>{}(v); }
};
template <typename V>
using StringMap = std::unordered_map<std::string, V, StringHash, std::equal_to<>>;

mcap::Compression parse_compression(std::string_view name)
{
  if (name == "zstd") {
    return mcap::Compression::Zstd;
  }
  if (name == "lz4") {
    return mcap::Compression::Lz4;
  }
  if (name.empty() || name == "none") {
    return mcap::Compression::None;
  }
  throw std::runtime_error("unknown mcap compression: " + std::string(name));
}

mcap::CompressionLevel parse_compression_level(std::string_view name, mcap::Compression codec)
{
  if (name.empty()) {
    // Codec-appropriate default. mcap maps lz4 + CompressionLevel::Default
    // onto LZ4-HC, which measures several times slower than zstd's default
    // for a larger output — the opposite of what choosing lz4 means — so an
    // unset level selects lz4's fast mode instead. An explicit "default"
    // still forces CompressionLevel::Default for any codec.
    return codec == mcap::Compression::Lz4 ? mcap::CompressionLevel::Fastest
                                           : mcap::CompressionLevel::Default;
  }
  if (name == "default") {
    return mcap::CompressionLevel::Default;
  }
  if (name == "fastest") {
    return mcap::CompressionLevel::Fastest;
  }
  if (name == "fast") {
    return mcap::CompressionLevel::Fast;
  }
  if (name == "slow") {
    return mcap::CompressionLevel::Slow;
  }
  if (name == "slowest") {
    return mcap::CompressionLevel::Slowest;
  }
  throw std::runtime_error("unknown mcap compression level: " + std::string(name));
}

// ---------------------------------------------------------------------------
// Single .mcap file writer.
// ---------------------------------------------------------------------------
class McapFileWriter : public BagWriter
{
public:
  McapFileWriter(const std::filesystem::path & path, const CreateOptions & options)
  {
    const auto compression = parse_compression(options.mcap_compression);
    const auto level = parse_compression_level(options.mcap_compression_level, compression);
    // Compressed output goes through the parallel chunk writer when write
    // threads are available: chunk compression is the write path's CPU
    // bottleneck and parallelizes across chunks. Uncompressed output has no
    // chunk encode to parallelize and stays on the serial libmcap writer.
    if (compression != mcap::Compression::None && resolve_write_threads() > 1) {
      parallel_ = std::make_unique<ParallelChunkMcapWriter>(
        path, compression == mcap::Compression::Zstd ? "zstd" : "lz4", level,
        options.mcap_chunk_size, resolve_write_threads());
      return;
    }

    mcap::McapWriterOptions wopts("ros2");
    wopts.compression = compression;
    wopts.compressionLevel = level;
    wopts.chunkSize = options.mcap_chunk_size;
    // Chunk CRCs cost a CRC32 pass over every written byte, and the common
    // readers (libmcap, rosbag2, foxglove) do not validate them on their
    // default read path — on multi-GB rewrites that is seconds of pure
    // overhead. Skip them; the end-of-file summary CRC stays on.
    wopts.noChunkCRC = true;

    const auto status = writer_.open(path.string(), wopts);
    if (!status.ok()) {
      throw std::runtime_error(
        "mcap writer open failed for " + path.string() + ": " + status.message);
    }
  }

  ~McapFileWriter() override
  {
    if (!closed_) {
      try {
        if (parallel_) {
          parallel_->close();
        } else {
          writer_.close();
        }
      } catch (...) {
        // Destructors must not throw; mcap::close itself can't throw either
        // but a user subclass or future revision might. Swallow silently.
      }
    }
  }

  McapFileWriter(const McapFileWriter &) = delete;
  McapFileWriter & operator=(const McapFileWriter &) = delete;
  McapFileWriter(McapFileWriter &&) = delete;
  McapFileWriter & operator=(McapFileWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    mcap::SchemaId schema_id{};
    if (auto it = type_to_schema_.find(topic.type); it != type_to_schema_.end()) {
      schema_id = it->second;
    } else {
      // Embed the message definition when the caller provided one. The
      // encoding defaults to "ros2msg" because that is what every ROS 2
      // toolchain (rosbag2, foxglove, mcap_ros2) emits today; if a caller
      // ever passes raw IDL, they must set schema_encoding explicitly.
      //
      // When schema_text is empty (caller has no definition handy),
      // also emit an empty encoding. Pairing `encoding="ros2msg"` with
      // `data=""` is misleading: strict readers (e.g. rosbags-convert)
      // parse the empty payload as a zero-field type and conflict it
      // with their built-in `builtin_interfaces/msg/Time` definition,
      // surfacing as `TypesysError("...already present with different
      // definition.")`. An empty encoding is the MCAP convention for
      // "no schema known" — readers fall back to their default
      // typestore instead of treating it as a malformed schema.
      const bool has_text = !topic.schema_text.empty();
      const std::string encoding =
        has_text ? (topic.schema_encoding.empty() ? std::string("ros2msg") : topic.schema_encoding)
                 : std::string{};
      mcap::Schema schema(topic.type, encoding, topic.schema_text);
      if (parallel_) {
        // The parallel writer serializes records verbatim, so ids are
        // assigned here — same order libmcap would: sequential from 1.
        schema.id = next_schema_id_++;
        parallel_->write_schema(schema);
      } else {
        writer_.addSchema(schema);
      }
      schema_id = schema.id;
      type_to_schema_[topic.type] = schema_id;
    }

    mcap::Channel channel(topic.name, topic.serialization_format, schema_id);
    if (!topic.offered_qos_profiles.empty()) {
      channel.metadata["offered_qos_profiles"] = topic.offered_qos_profiles;
    }
    if (parallel_) {
      channel.id = next_channel_id_++;
      parallel_->write_channel(channel);
    } else {
      writer_.addChannel(channel);
    }
    topic_to_channel_[topic.name] = channel.id;
  }

  void write(
    // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    const mcap::ChannelId channel_id = lookup_channel(topic);
    mcap::Message msg;
    msg.channelId = channel_id;
    msg.sequence = 0;
    msg.logTime = static_cast<mcap::Timestamp>(timestamp_ns);
    msg.publishTime = msg.logTime;
    msg.data = reinterpret_cast<const std::byte *>(payload.data());
    msg.dataSize = payload.size();

    if (parallel_) {
      parallel_->write_message(msg);
      return;
    }
    const auto status = writer_.write(msg);
    if (!status.ok()) {
      throw std::runtime_error("mcap write failed: " + status.message);
    }
  }

  // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
  void write_frozen(std::string_view topic, FrozenMessage msg) override
  {
    // The parallel writer pins the payload through the shared owner instead of
    // copying it into a staging buffer. Without an owner the span's lifetime
    // is call-scoped, so fall back to the copying entry point.
    if (parallel_ && msg.owner) {
      parallel_->write_message_owned(
        lookup_channel(topic), static_cast<mcap::Timestamp>(msg.timestamp_ns), msg.payload,
        std::move(msg.owner));
      return;
    }
    write(topic, msg.timestamp_ns, msg.payload);
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    if (parallel_) {
      parallel_->close();
    } else {
      writer_.close();
    }
    closed_ = true;
  }

private:
  mcap::ChannelId lookup_channel(std::string_view topic) const
  {
    const auto it = topic_to_channel_.find(topic);
    if (it == topic_to_channel_.end()) {
      throw std::runtime_error(
        "mcap write on undeclared topic: " + std::string(topic) + " (call declare_topic() first)");
    }
    return it->second;
  }

  mcap::McapWriter writer_;
  std::unique_ptr<ParallelChunkMcapWriter> parallel_;
  mcap::SchemaId next_schema_id_ = 1;
  mcap::ChannelId next_channel_id_ = 1;
  StringMap<mcap::ChannelId> topic_to_channel_;
  std::unordered_map<std::string, mcap::SchemaId> type_to_schema_;
  bool closed_ = false;
};

// ---------------------------------------------------------------------------
// Directory writer: wraps a single McapFileWriter shard and emits a
// metadata.yaml compatible with rosbag2's expected schema on close().
// ---------------------------------------------------------------------------
class McapDirectoryWriter final : public BagWriter
{
public:
  McapDirectoryWriter(const std::filesystem::path & dir, const CreateOptions & options) : dir_(dir)
  {
    std::filesystem::create_directories(dir);

    // rosbag2 uses "<dirname>_<index>.mcap" for shards; match that layout so
    // the output is interchangeable with `ros2 bag record`.
    const auto stem = dir.filename().string();
    shard_rel_ = stem + "_0.mcap";
    inner_ = std::make_unique<McapFileWriter>(dir_ / shard_rel_, options);
  }

  ~McapDirectoryWriter() override
  {
    if (!closed_) {
      try {
        McapDirectoryWriter::close();
      } catch (const std::exception & e) {
        BAGWIZ_LOG_WARN(kLogger, "McapDirectoryWriter close failed: %s", e.what());
      } catch (...) {
        // Never throw from destructor.
      }
    }
  }

  McapDirectoryWriter(const McapDirectoryWriter &) = delete;
  McapDirectoryWriter & operator=(const McapDirectoryWriter &) = delete;
  McapDirectoryWriter(McapDirectoryWriter &&) = delete;
  McapDirectoryWriter & operator=(McapDirectoryWriter &&) = delete;

  void declare_topic(const TopicInfo & topic) override
  {
    inner_->declare_topic(topic);
    topics_.push_back(topic);
    topic_counts_[topic.name] = 0;
  }

  void write(
    // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
    std::string_view topic, int64_t timestamp_ns, std::span<const std::byte> payload) override
  {
    inner_->write(topic, timestamp_ns, payload);
    // Every written topic was declared (inner_ throws otherwise), so the
    // string_view find always hits the entry declare_topic() inserted.
    ++topic_counts_.find(topic)->second;
    ++total_messages_;
    if (timestamp_ns < start_ns_) {
      start_ns_ = timestamp_ns;
    }
    if (timestamp_ns > end_ns_) {
      end_ns_ = timestamp_ns;
    }
  }

  // cppcheck-suppress passedByValue  // std::string_view is a cheap value type
  void write_frozen(std::string_view topic, FrozenMessage msg) override
  {
    const auto timestamp_ns = msg.timestamp_ns;
    inner_->write_frozen(topic, std::move(msg));
    ++topic_counts_.find(topic)->second;
    ++total_messages_;
    if (timestamp_ns < start_ns_) {
      start_ns_ = timestamp_ns;
    }
    if (timestamp_ns > end_ns_) {
      end_ns_ = timestamp_ns;
    }
  }

  void close() override
  {
    if (closed_) {
      return;
    }
    inner_->close();
    // The metadata `compression_format` / `compression_mode` pair describes
    // rosbag2's own compression layer (rosbag2_compression), not the storage
    // plugin's internal compression, so both stay empty here however the
    // chunks were written. Declaring the chunk codec there instead makes
    // rosbag2 open the bag through SequentialCompressionReader, which expands
    // the named file as a whole-file zstd envelope and fails on an ordinary
    // .mcap ("ZSTD decompression error: Unknown frame descriptor"); for lz4 it
    // fails earlier still, since rosbag2 registers no lz4 plugin. rosbag2's
    // own mcap writer leaves the pair empty for the same reason. Nothing is
    // lost: MCAP records each chunk's codec inside the file, which is where
    // both libmcap and bagwiz's chunk pass-through read it from.
    MetadataYamlInfo info;
    info.storage_identifier = "mcap";
    info.topics = topics_;
    info.per_topic_counts =
      std::unordered_map<std::string, int64_t>(topic_counts_.begin(), topic_counts_.end());
    info.total_messages = total_messages_;
    info.start_ns = start_ns_;
    info.end_ns = end_ns_;
    info.shard_relative_path = shard_rel_;
    write_metadata_yaml(dir_, info);
    closed_ = true;
  }

private:
  std::filesystem::path dir_;
  std::string shard_rel_;
  std::unique_ptr<McapFileWriter> inner_;

  std::vector<TopicInfo> topics_;
  StringMap<int64_t> topic_counts_;
  int64_t total_messages_ = 0;
  int64_t start_ns_ = std::numeric_limits<int64_t>::max();
  int64_t end_ns_ = std::numeric_limits<int64_t>::min();
  bool closed_ = false;
};

}  // namespace

std::unique_ptr<BagWriter> create_mcap_file(
  const std::filesystem::path & path, const CreateOptions & options)
{
  return std::make_unique<McapFileWriter>(path, options);
}

std::unique_ptr<BagWriter> create_mcap_directory(
  const std::filesystem::path & dir, const CreateOptions & options)
{
  return std::make_unique<McapDirectoryWriter>(dir, options);
}

}  // namespace bagwiz::io::detail
