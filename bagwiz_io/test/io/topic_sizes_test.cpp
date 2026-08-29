// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <mcap/writer.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

// BagReader::compute_topic_sizes() reads each message's declared length
// instead of its payload, so its answer must be byte-for-byte what a full scan
// would produce — on every bag shape it accepts — and it must decline (nullopt)
// on the shapes it cannot answer exactly.
namespace
{

using SizeMap = std::unordered_map<std::string, std::uint64_t>;

bagwiz::io::TopicInfo topic_info(const std::string & name)
{
  bagwiz::io::TopicInfo t;
  t.name = name;
  t.type = "std_msgs/msg/ByteMultiArray";
  t.serialization_format = "cdr";
  return t;
}

std::vector<std::byte> payload_bytes(int seed, std::size_t size)
{
  std::vector<std::byte> out(size);
  for (std::size_t i = 0; i < size; ++i) {
    out[i] = static_cast<std::byte>((seed * 31 + static_cast<int>(i)) & 0xFF);
  }
  return out;
}

// What the answer has to match: the payload sizes a full message scan sees.
SizeMap sizes_by_scan(
  const std::filesystem::path & path, const std::vector<std::string> & topics = {})
{
  auto reader = bagwiz::io::open_read(path);
  if (!topics.empty()) {
    bagwiz::io::ReadFilter filter;
    filter.topics = topics;
    reader->set_filter(filter);
  }
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

// A bag with wildly uneven message sizes on three topics, so a per-topic
// mix-up cannot pass unnoticed. `chunk_size` small enough to spread the
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

TEST_F(TopicSizesTest, MatchesScanOnUncompressedMcap)
{
  const auto path = write_bag(tmp_ / "none.mcap", mcap_options("none"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_scan(path));
}

TEST_F(TopicSizesTest, MatchesScanOnZstdMcap)
{
  const auto path = write_bag(tmp_ / "zstd.mcap", mcap_options("zstd"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_scan(path));
}

TEST_F(TopicSizesTest, MatchesScanOnLz4Mcap)
{
  const auto path = write_bag(tmp_ / "lz4.mcap", mcap_options("lz4"));
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_scan(path));
}

TEST_F(TopicSizesTest, MatchesScanOnSqlite3)
{
  const auto path = write_bag(tmp_ / "bag.db3", sqlite_options());
  const auto indexed = sizes_by_index(path);
  ASSERT_TRUE(indexed.has_value());
  EXPECT_EQ(*indexed, sizes_by_scan(path));
}

// A selection must isolate exactly the named topics: the unselected ones are
// absent from the result, not merely zero, and the selected ones keep the
// totals they have in a full report.
TEST_F(TopicSizesTest, TopicSelectionMatchesFilteredScan)
{
  for (const auto & options : {mcap_options("none"), mcap_options("zstd"), sqlite_options()}) {
    const auto path = write_bag(
      tmp_ /
        ("selected_" + std::to_string(static_cast<int>(options.format)) + options.mcap_compression),
      options);
    const std::vector<std::string> selection{"/small"};
    const auto indexed = sizes_by_index(path, selection);
    ASSERT_TRUE(indexed.has_value());
    EXPECT_EQ(*indexed, sizes_by_scan(path, selection));
    EXPECT_EQ(indexed->count("/big"), 0U);
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

// rosbag2 `compression_mode: MESSAGE` stores each payload compressed, so the
// stored length is the compressed one. The reader must decline rather than
// report it as the payload size.
TEST_F(TopicSizesTest, MessageCompressedSqliteDeclines)
{
  auto options = sqlite_options();
  options.layout = bagwiz::io::Layout::Directory;
  options.sqlite3_compression_mode = "message";
  options.sqlite3_compression_format = "zstd";
  const auto path = write_bag(tmp_ / "message_compressed", options);

  EXPECT_FALSE(sizes_by_index(path).has_value());
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

// mcap writers emit a channel's Schema and Channel records into the chunk that
// first carries one of its messages, so a chunk can hold records that are not
// messages, interleaved between them. Deriving a message's size from the gap
// to the next one would silently absorb those bytes; reading the record's own
// length prefix — what the reader does — does not. This bag declares its
// second topic only after the first one's messages, to put such a record in
// the middle of a chunk.
TEST_F(TopicSizesTest, ChannelDeclaredMidChunkDoesNotInflateSizes)
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

  SizeMap expected;
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
    expected[name] += payload.size();
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
  EXPECT_EQ(*indexed, expected);
  EXPECT_EQ(*indexed, sizes_by_scan(path));
}

// A file whose chunk index promises message records where there are none must
// be refused, not silently mis-counted: the caller falls back to the scan.
TEST_F(TopicSizesTest, CorruptedMessageRecordDeclines)
{
  const auto path = write_bag(tmp_ / "corrupt.mcap", mcap_options("none"));
  ASSERT_TRUE(sizes_by_index(path).has_value());

  // Blank the whole data section: every message opcode the index points at is
  // now a zero byte.
  {
    std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
    ASSERT_TRUE(file.is_open());
    const auto size = std::filesystem::file_size(path);
    const std::vector<char> zeros(static_cast<std::size_t>(size) / 2, '\0');
    file.seekp(64);
    file.write(zeros.data(), static_cast<std::streamsize>(zeros.size()));
  }

  EXPECT_FALSE(sizes_by_index(path).has_value());
}

}  // namespace
