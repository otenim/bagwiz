// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/file_decompressor.hpp"

#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include <gtest/gtest.h>
#include <sqlite3.h>
#include <zstd.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

// BagReader::compute_topic_sizes() answers `bagwiz du`: each topic's bytes on
// disk, derived from the bag's indexes and row headers rather than from its
// payloads. On every bag shape it accepts, the answer must match a reference
// that walks the actual records and applies the documented charging rule; on
// the shapes it cannot answer it must decline (nullopt) so the caller scans.
namespace
{

using SizeMap = std::unordered_map<std::string, std::uint64_t>;

// Every mcap record is a 1-byte opcode plus a uint64 content length. A Message
// record's content is a 22-byte header (channel id, sequence, log time,
// publish time) followed by the payload. A MessageIndex record's content is
// the channel id (2), the entry array's byte length (4), then one 16-byte
// (log time, offset) entry per message.
constexpr std::uint64_t kRecordPrefixBytes = 9;
constexpr std::uint64_t kMessageHeaderBytes = 22;
constexpr std::uint64_t kMessageIndexHeaderBytes = 2 + 4;
constexpr std::uint64_t kMessageIndexEntryBytes = 16;

bagwiz::io::TopicInfo topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/ByteMultiArray";
  t.serialization_format = "cdr";
  return t;
}

// A sawtooth byte pattern: distinct per seed, and compressible, so a compressed
// bag is visibly smaller than its payloads.
std::vector<std::byte> payload_bytes(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed * 31 + static_cast<int>(i)) & 0xFF);
  }
  return out;
}

std::uint64_t sum_of(const SizeMap & sizes)
{
  std::uint64_t total = 0;
  for (const auto & [name, bytes] : sizes) {
    total += bytes;
  }
  return total;
}

// The logical payload sizes a full message scan sees: what `du` used to
// report, and the upper bound a compressed bag's on-disk answer must undercut.
SizeMap payload_sizes_by_scan(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  SizeMap sizes;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    sizes[msg.topic->name] += msg.payload.size();
  }
  return sizes;
}

std::optional<SizeMap> sizes_by_index(
  const std::filesystem::path & path, const std::vector<std::string> & topics = {})
{
  auto reader = bagwiz::io::open_read(path);
  return reader->compute_topic_sizes(topics);
}

// One chunk's share charged to a channel: the channel's uncompressed record
// bytes scaled by the chunk's compression ratio, to the nearest byte. An
// uncompressed chunk (equal sizes) is charged exactly.
std::uint64_t prorate(std::uint64_t bytes, std::uint64_t compressed, std::uint64_t uncompressed)
{
  if (compressed == uncompressed) {
    return bytes;
  }
  return static_cast<std::uint64_t>(std::llround(
    static_cast<long double>(bytes) * static_cast<long double>(compressed) /
    static_cast<long double>(uncompressed)));
}

// One record inside a chunk's records blob, as the reference walk sees it.
struct ChunkRecord
{
  std::uint64_t offset = 0;
  // Set for a Message record; a Schema / Channel declaration record has none.
  std::optional<std::uint16_t> channel;
  // A Message record's size from its own fields, cross-checked against the
  // gap to the next record so the walk's arithmetic is itself verified.
  std::uint64_t declared_size = 0;
};

// Reference for an mcap: walk every record with libmcap's typed reader and
// charge bytes by the rule `du` documents. A Message record's bytes go to its
// channel; a Schema / Channel record written mid-chunk goes to the channel of
// the message before it (nothing, ahead of the first message); each chunk's
// charges are prorated by its compression ratio; a MessageIndex record's bytes
// go to its channel in full. Chunk record headers, top-level declarations and
// the summary section are charged to no topic.
SizeMap sizes_by_record_walk(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("open failed: " + path.string());
  }
  mcap::FileStreamReader file(stream);

  std::unordered_map<std::uint16_t, std::string> topic_of;
  std::unordered_map<std::uint16_t, std::uint64_t> per_channel;
  std::vector<ChunkRecord> chunk_records;
  std::uint64_t chunk_compressed = 0;
  std::uint64_t chunk_uncompressed = 0;

  // For a record inside a chunk, libmcap's typed reader passes the chunk's
  // file offset as the ByteOffset argument and the record's offset within the
  // chunk's uncompressed records blob as the optional one; a top-level record
  // gets no optional.
  mcap::TypedRecordReader reader(file, sizeof(mcap::Magic));
  reader.onChunk = [&](const mcap::Chunk & chunk, mcap::ByteOffset /*offset*/) {
    chunk_records.clear();
    chunk_compressed = chunk.compressedSize;
    chunk_uncompressed = chunk.uncompressedSize;
  };
  reader.onSchema = [&](
                      mcap::SchemaPtr /*schema*/, mcap::ByteOffset /*chunk_start*/,
                      std::optional<mcap::ByteOffset> in_chunk) {
    if (in_chunk.has_value()) {
      chunk_records.push_back({*in_chunk, std::nullopt, 0});
    }
  };
  reader.onChannel = [&](
                       mcap::ChannelPtr channel, mcap::ByteOffset /*chunk_start*/,
                       std::optional<mcap::ByteOffset> in_chunk) {
    topic_of[channel->id] = channel->topic;
    if (in_chunk.has_value()) {
      chunk_records.push_back({*in_chunk, std::nullopt, 0});
    }
  };
  reader.onMessage = [&](
                       const mcap::Message & message, mcap::ByteOffset /*chunk_start*/,
                       std::optional<mcap::ByteOffset> in_chunk) {
    if (!in_chunk.has_value()) {
      throw std::runtime_error("reference walk expects every message inside a chunk");
    }
    chunk_records.push_back(
      {*in_chunk, message.channelId, kRecordPrefixBytes + kMessageHeaderBytes + message.dataSize});
  };
  reader.onChunkEnd = [&](mcap::ByteOffset /*offset*/) {
    std::unordered_map<std::uint16_t, std::uint64_t> in_chunk;
    std::optional<std::uint16_t> owner;
    for (std::size_t i = 0; i < chunk_records.size(); ++i) {
      const auto & record = chunk_records[i];
      const std::uint64_t end =
        i + 1 < chunk_records.size() ? chunk_records[i + 1].offset : chunk_uncompressed;
      const std::uint64_t bytes = end - record.offset;
      if (record.channel.has_value()) {
        if (bytes != record.declared_size) {
          throw std::runtime_error("reference walk: message record gap != declared size");
        }
        owner = record.channel;
      }
      if (owner.has_value()) {
        in_chunk[*owner] += bytes;
      }
    }
    for (const auto & [channel, bytes] : in_chunk) {
      per_channel[channel] += prorate(bytes, chunk_compressed, chunk_uncompressed);
    }
  };
  reader.onMessageIndex = [&](const mcap::MessageIndex & index, mcap::ByteOffset /*offset*/) {
    per_channel[index.channelId] += kRecordPrefixBytes + kMessageIndexHeaderBytes +
                                    kMessageIndexEntryBytes * index.records.size();
  };
  while (reader.next()) {
  }
  if (!reader.status().ok()) {
    throw std::runtime_error("reference walk failed: " + reader.status().message);
  }

  SizeMap result;
  for (const auto & [channel, bytes] : per_channel) {
    result[topic_of.at(channel)] += bytes;
  }
  return result;
}

// Reference for one .db3: the stored BLOB length of every row, per topic —
// the bytes a row's payload occupies, compressed or not.
SizeMap sizes_by_row_lengths(const std::filesystem::path & db3)
{
  sqlite3 * raw = nullptr;
  if (sqlite3_open_v2(db3.string().c_str(), &raw, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite open failed: " + db3.string());
  }
  std::unique_ptr<sqlite3, decltype(&sqlite3_close)> db(raw, &sqlite3_close);
  sqlite3_stmt * stmt_raw = nullptr;
  const char * sql =
    "SELECT t.name, LENGTH(m.data) FROM messages m JOIN topics t ON t.id = m.topic_id";
  if (sqlite3_prepare_v2(db.get(), sql, -1, &stmt_raw, nullptr) != SQLITE_OK) {
    throw std::runtime_error("sqlite prepare failed");
  }
  std::unique_ptr<sqlite3_stmt, decltype(&sqlite3_finalize)> stmt(stmt_raw, &sqlite3_finalize);
  SizeMap sizes;
  while (sqlite3_step(stmt.get()) == SQLITE_ROW) {
    const auto * name = reinterpret_cast<const char *>(sqlite3_column_text(stmt.get(), 0));
    sizes[name] += static_cast<std::uint64_t>(sqlite3_column_int64(stmt.get(), 1));
  }
  return sizes;
}

// The single shard of a directory bag, by extension.
std::filesystem::path only_shard(const std::filesystem::path & dir, const std::string & suffix)
{
  std::filesystem::path found;
  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
    const auto name = entry.path().filename().string();
    if (
      name.size() >= suffix.size() &&
      name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
      if (!found.empty()) {
        throw std::runtime_error("more than one shard in " + dir.string());
      }
      found = entry.path();
    }
  }
  if (found.empty()) {
    throw std::runtime_error("no " + suffix + " shard in " + dir.string());
  }
  return found;
}

// A bag with wildly uneven message sizes on three topics, so a per-topic
// mix-up cannot pass unnoticed. `mcap_chunk_size` small enough to spread the
// messages over many chunks.
std::filesystem::path write_bag(
  const std::filesystem::path & path, const bagwiz::io::CreateOptions & options)
{
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/big"));
  writer->declare_topic(topic_info("/small"));
  writer->declare_topic(topic_info("/silent"));

  for (int i = 0; i < 40; ++i) {
    writer->write(
      "/big", 1'000'000'000LL + i, std::span<const std::byte>(payload_bytes(i, 20'000 + i)));
    writer->write("/small", 1'000'000'001LL + i, std::span<const std::byte>(payload_bytes(i, 7)));
  }
  writer->close();
  return path;
}

bagwiz::io::CreateOptions mcap_options(const std::string & compression)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = compression;
  options.mcap_chunk_size = 64 * 1024;
  return options;
}

bagwiz::io::CreateOptions sqlite_options()
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;
  return options;
}

bagwiz::io::CreateOptions sqlite_dir_options(const std::string & mode)
{
  auto options = sqlite_options();
  options.layout = bagwiz::io::Layout::Directory;
  options.sqlite3_compression_mode = mode;
  options.sqlite3_compression_format = "zstd";
  return options;
}

std::vector<std::byte> zstd_compress(const std::vector<std::byte> & plain)
{
  std::vector<std::byte> compressed(ZSTD_compressBound(plain.size()));
  const std::size_t written = ZSTD_compress(
    compressed.data(), compressed.size(), plain.data(), plain.size(), /*compressionLevel=*/1);
  if (ZSTD_isError(written) != 0U) {
    throw std::runtime_error(ZSTD_getErrorName(written));
  }
  compressed.resize(written);
  return compressed;
}

// Overwrite `length` bytes at `offset` with zeros.
void zero_region(const std::filesystem::path & path, std::uint64_t offset, std::uint64_t length)
{
  std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
  ASSERT_TRUE(file.is_open());
  const std::vector<char> zeros(static_cast<std::size_t>(length), '\0');
  file.seekp(static_cast<std::streamoff>(offset));
  file.write(zeros.data(), static_cast<std::streamsize>(zeros.size()));
}

std::vector<mcap::ChunkIndex> chunk_indexes_of(const std::filesystem::path & path)
{
  mcap::McapReader reader;
  if (const auto status = reader.open(path.string()); !status.ok()) {
    throw std::runtime_error(status.message);
  }
  if (const auto status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
      !status.ok()) {
    throw std::runtime_error(status.message);
  }
  return reader.chunkIndexes();
}

class TopicSizesTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_topic_sizes_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_, ec);
  }

  std::filesystem::path tmp_;
};

// An uncompressed chunk stores every message record verbatim, so each topic
// is charged exactly its record bytes (prefix, header and payload) plus its
// message index records — more than its payloads alone.
TEST_F(TopicSizesTest, UncompressedMcapChargesRecordAndIndexBytes)
{
  const auto path = write_bag(tmp_ / "none.mcap", mcap_options("none"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_record_walk(path));

  const auto payloads = payload_sizes_by_scan(path);
  EXPECT_GT(indexed->at("/big"), payloads.at("/big"));
  EXPECT_GT(indexed->at("/small"), payloads.at("/small"));
}

// A compressed chunk's bytes are split among the topics it holds in
// proportion to their uncompressed record bytes, so a compressible bag's
// answer lands below its payload total — the file-size drop `du` exists to
// show.
TEST_F(TopicSizesTest, ZstdMcapProratesCompressedChunkBytes)
{
  const auto path = write_bag(tmp_ / "zstd.mcap", mcap_options("zstd"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_record_walk(path));

  EXPECT_LT(indexed->at("/big"), payload_sizes_by_scan(path).at("/big"));
  EXPECT_LE(sum_of(*indexed), std::filesystem::file_size(path));
}

TEST_F(TopicSizesTest, Lz4McapProratesCompressedChunkBytes)
{
  const auto path = write_bag(tmp_ / "lz4.mcap", mcap_options("lz4"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_record_walk(path));

  EXPECT_LT(indexed->at("/big"), payload_sizes_by_scan(path).at("/big"));
  EXPECT_LE(sum_of(*indexed), std::filesystem::file_size(path));
}

// The answer comes from the chunk and message indexes alone: with every
// chunk's records blob zeroed — payloads, record headers, compressed frames,
// all of it — the sizes must not change. Reading any record byte would either
// change the answer or fail on the wrecked frames.
TEST_F(TopicSizesTest, McapSizesNeedNoRecordBytes)
{
  for (const auto & compression : {"none", "zstd"}) {
    const auto path =
      write_bag(tmp_ / (std::string(compression) + "_zeroed.mcap"), mcap_options(compression));
    const auto before = sizes_by_index(path);
    ASSERT_TRUE(before.has_value());

    // A chunk record's body ahead of its records blob: message start/end time
    // (8+8), uncompressed size (8), uncompressed CRC (4), the compression
    // string (4 + its bytes), and the records blob's length prefix (8).
    for (const auto & index : chunk_indexes_of(path)) {
      const std::uint64_t blob_offset = index.chunkStartOffset + kRecordPrefixBytes + 8 + 8 + 8 +
                                        4 + 4 + index.compression.size() + 8;
      zero_region(path, blob_offset, index.compressedSize);
    }

    const auto after = sizes_by_index(path);
    ASSERT_TRUE(after.has_value()) << compression;
    EXPECT_EQ(*after, *before) << compression;
  }
}

// A plain .db3 stores each payload as its row's BLOB, so a topic's bytes are
// its rows' BLOB lengths.
TEST_F(TopicSizesTest, Sqlite3ReportsStoredBlobLengths)
{
  const auto path = write_bag(tmp_ / "bag.db3", sqlite_options());
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_row_lengths(path));
  EXPECT_EQ(*indexed, payload_sizes_by_scan(path));
}

// rosbag2 `compression_mode: MESSAGE` stores each payload as a zstd frame in
// its row, so the stored BLOB length is exactly the on-disk size: the reader
// answers from the row headers and the answer is below the decompressed
// payload total.
TEST_F(TopicSizesTest, MessageCompressedSqlite3ReportsStoredBlobLengths)
{
  const auto path = write_bag(tmp_ / "message_compressed", sqlite_dir_options("message"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_row_lengths(only_shard(path, ".db3")));
  EXPECT_LT(indexed->at("/big"), payload_sizes_by_scan(path).at("/big"));
}

// rosbag2 `compression_mode: FILE` wraps the whole .db3 in one zstd frame, so
// no per-topic byte count exists on disk. The reader scales each topic's
// stored BLOB lengths by the envelope's compression ratio (envelope bytes over
// decompressed database bytes), to the nearest byte.
TEST_F(TopicSizesTest, FileCompressedSqlite3ScalesByEnvelopeRatio)
{
  const auto path = write_bag(tmp_ / "file_compressed", sqlite_dir_options("file"));
  const auto envelope = only_shard(path, ".db3.zstd");
  const auto plain = bagwiz::io::decompress_zstd_file_to_temp(envelope);
  const std::uint64_t envelope_bytes = std::filesystem::file_size(envelope);
  const std::uint64_t database_bytes = std::filesystem::file_size(plain.path());
  ASSERT_LT(envelope_bytes, database_bytes);

  SizeMap expected;
  for (const auto & [name, bytes] : sizes_by_row_lengths(plain.path())) {
    expected[name] = prorate(bytes, envelope_bytes, database_bytes);
  }

  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, expected);
}

// rosbag2 MESSAGE-mode over mcap storage: each message record carries a zstd
// frame as its payload. The records are what is on disk, so the reader answers
// from the indexes exactly as for a plain mcap, below the decompressed total.
TEST_F(TopicSizesTest, MessageCompressedMcapReportsStoredRecordBytes)
{
  const auto dir = tmp_ / "message_compressed_mcap";
  std::filesystem::create_directories(dir);
  const auto shard = dir / "shard_0.mcap";
  {
    mcap::McapWriter writer;
    mcap::McapWriterOptions options("ros2");
    options.compression = mcap::Compression::None;
    ASSERT_TRUE(writer.open(shard.string(), options).ok());
    mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "");
    writer.addSchema(schema);
    mcap::Channel channel("/big", "cdr", schema.id);
    writer.addChannel(channel);
    for (int i = 0; i < 8; ++i) {
      const auto frame = zstd_compress(payload_bytes(i, 20'000));
      mcap::Message message;
      message.channelId = channel.id;
      message.sequence = static_cast<std::uint32_t>(i);
      message.logTime = 1'000'000'000 + static_cast<mcap::Timestamp>(i);
      message.publishTime = message.logTime;
      message.data = frame.data();
      message.dataSize = frame.size();
      ASSERT_TRUE(writer.write(message).ok());
    }
    writer.close();
  }
  {
    std::ofstream yaml(dir / "metadata.yaml");
    yaml << "rosbag2_bagfile_information:\n"
            "  version: 5\n"
            "  storage_identifier: mcap\n"
            "  duration:\n"
            "    nanoseconds: 7\n"
            "  starting_time:\n"
            "    nanoseconds_since_epoch: 1000000000\n"
            "  message_count: 8\n"
            "  topics_with_message_count:\n"
            "    - topic_metadata:\n"
            "        name: /big\n"
            "        type: std_msgs/msg/ByteMultiArray\n"
            "        serialization_format: cdr\n"
            "        offered_qos_profiles: \"\"\n"
            "      message_count: 8\n"
            "  compression_format: zstd\n"
            "  compression_mode: MESSAGE\n"
            "  relative_file_paths:\n"
            "    - shard_0.mcap\n"
            "  files:\n"
            "    - path: shard_0.mcap\n"
            "      starting_time:\n"
            "        nanoseconds_since_epoch: 1000000000\n"
            "      duration:\n"
            "        nanoseconds: 7\n"
            "      message_count: 8\n";
  }

  const auto indexed = sizes_by_index(dir);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_record_walk(shard));
  EXPECT_LT(indexed->at("/big"), payload_sizes_by_scan(dir).at("/big"));
}

// A selection must isolate exactly the named topics: the unselected ones are
// absent from the result, not merely zero, and the selected ones keep the
// bytes they have in a full report — on every shape, including the compressed
// chunks whose bytes are shared with unselected topics.
TEST_F(TopicSizesTest, TopicSelectionKeepsFullReportBytes)
{
  const std::vector<std::pair<std::string, bagwiz::io::CreateOptions>> shapes{
    {"none.mcap", mcap_options("none")},
    {"zstd.mcap", mcap_options("zstd")},
    {"bag.db3", sqlite_options()},
    {"message_compressed", sqlite_dir_options("message")},
    {"file_compressed", sqlite_dir_options("file")},
  };
  for (const auto & [name, options] : shapes) {
    const auto path = write_bag(tmp_ / name, options);
    const auto full = sizes_by_index(path);
    ASSERT_TRUE(full.has_value()) << name;

    const auto selected = sizes_by_index(path, {"/small"});
    ASSERT_TRUE(selected.has_value()) << name;
    EXPECT_EQ(selected->at("/small"), full->at("/small")) << name;
    EXPECT_EQ(selected->count("/big"), 0U) << name;
  }
}

TEST_F(TopicSizesTest, EmptyBagReportsNothing)
{
  const auto path = tmp_ / "empty.mcap";
  auto writer = bagwiz::io::open_write(path, mcap_options("none"));
  writer->declare_topic(topic_info("/silent"));
  writer->close();

  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_TRUE(indexed->empty());
}

// An unchunked mcap has no message index to read offsets from. The reader must
// decline so the caller scans instead of reporting nothing.
TEST_F(TopicSizesTest, UnchunkedMcapDeclines)
{
  const auto path = tmp_ / "unchunked.mcap";
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("ros2");
  options.noChunking = true;
  ASSERT_TRUE(writer.open(path.string(), options).ok());

  mcap::Schema schema("std_msgs/msg/ByteMultiArray", "ros2msg", "");
  writer.addSchema(schema);
  mcap::Channel channel("/big", "cdr", schema.id);
  writer.addChannel(channel);
  const auto payload = payload_bytes(1, 512);
  mcap::Message message;
  message.channelId = channel.id;
  message.sequence = 0;
  message.logTime = 1'000'000'000;
  message.publishTime = message.logTime;
  message.data = payload.data();
  message.dataSize = payload.size();
  ASSERT_TRUE(writer.write(message).ok());
  writer.close();

  EXPECT_FALSE(sizes_by_index(path).has_value());
}

// libmcap emits a channel's Schema and Channel records into the chunk that
// first carries one of its messages, between other channels' messages. The
// reader never reads records, so it cannot tell those bytes from the message
// ahead of them: they are charged to the topic of that preceding message. The
// error is bounded by the bag's declaration records — a few KiB — and this
// test pins the rule rather than hiding it. This bag declares its second topic
// only after the first one's messages, to put such records mid-chunk.
TEST_F(TopicSizesTest, MidChunkDeclarationChargedToPrecedingTopic)
{
  const auto path = tmp_ / "late_channel.mcap";
  mcap::McapWriter writer;
  mcap::McapWriterOptions options("ros2");
  options.compression = mcap::Compression::None;
  options.chunkSize = 8 * 1024 * 1024;  // one chunk holds everything
  ASSERT_TRUE(writer.open(path.string(), options).ok());

  mcap::Schema schema_a("std_msgs/msg/ByteMultiArray", "ros2msg", "a");
  writer.addSchema(schema_a);
  mcap::Channel channel_a("/first", "cdr", schema_a.id);
  writer.addChannel(channel_a);

  SizeMap record_bytes;
  const auto write_one = [&](const mcap::Channel & channel, const std::string & name, int seed) {
    const auto payload = payload_bytes(seed, 64 + static_cast<std::size_t>(seed));
    mcap::Message message;
    message.channelId = channel.id;
    message.sequence = static_cast<std::uint32_t>(seed);
    message.logTime = 1'000'000'000 + static_cast<mcap::Timestamp>(seed);
    message.publishTime = message.logTime;
    message.data = payload.data();
    message.dataSize = payload.size();
    ASSERT_TRUE(writer.write(message).ok());
    record_bytes[name] += kRecordPrefixBytes + kMessageHeaderBytes + payload.size();
  };

  for (int i = 0; i < 4; ++i) {
    write_one(channel_a, "/first", i);
  }
  // Declared only now: its Schema and Channel records land between messages.
  mcap::Schema schema_b("std_msgs/msg/ByteMultiArray", "ros2msg", "bbbbbbbbbbbbbbbb");
  writer.addSchema(schema_b);
  mcap::Channel channel_b("/second", "cdr", schema_b.id);
  writer.addChannel(channel_b);
  for (int i = 4; i < 8; ++i) {
    write_one(channel_b, "/second", i);
  }
  writer.close();

  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_record_walk(path));

  // Each topic: 4 messages, one MessageIndex record of 4 entries.
  const std::uint64_t index_bytes =
    kRecordPrefixBytes + kMessageIndexHeaderBytes + 4 * kMessageIndexEntryBytes;
  EXPECT_EQ(indexed->at("/second"), record_bytes.at("/second") + index_bytes);
  // /first carries its own records plus /second's mid-chunk declarations.
  EXPECT_GT(indexed->at("/first"), record_bytes.at("/first") + index_bytes);
}

// A message index that does not parse must be refused, not silently
// mis-counted: the caller falls back to the scan.
TEST_F(TopicSizesTest, CorruptedMessageIndexDeclines)
{
  const auto path = write_bag(tmp_ / "corrupt.mcap", mcap_options("none"));
  ASSERT_TRUE(sizes_by_index(path).has_value());

  const auto indexes = chunk_indexes_of(path);
  ASSERT_FALSE(indexes.empty());
  const auto & first = indexes.front();
  std::uint64_t block_offset = first.messageIndexOffsets.begin()->second;
  for (const auto & [channel_id, offset] : first.messageIndexOffsets) {
    block_offset = std::min<std::uint64_t>(block_offset, offset);
  }
  zero_region(path, block_offset, first.messageIndexLength);

  EXPECT_FALSE(sizes_by_index(path).has_value());
}

}  // namespace
