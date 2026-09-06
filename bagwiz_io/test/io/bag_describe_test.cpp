// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_describe.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

using bagwiz::io::BagDescription;
using bagwiz::io::CreateOptions;
using bagwiz::io::describe_bag;
using bagwiz::io::Format;
using bagwiz::io::Layout;
using bagwiz::io::TopicInfo;

constexpr std::int64_t kFirstStampNs = 1'000'000'000LL;
constexpr std::int64_t kMiddleStampNs = 2'000'000'000LL;
constexpr std::int64_t kLastStampNs = 3'500'000'000LL;
constexpr std::int64_t kMessageCount = 3;

TopicInfo make_topic(std::string name, std::string type)
{
  TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> view(const std::vector<std::byte> & bytes)
{
  return std::span<const std::byte>(bytes.data(), bytes.size());
}

// Two topics; three messages spanning [kFirstStampNs, kLastStampNs] unless
// `with_messages` is false, in which case the topics are declared and the bag
// is closed empty. The large payload is a run of one byte so a compressed
// fixture measurably shrinks.
void write_fixture(
  const std::filesystem::path & path, const CreateOptions & options, bool with_messages = true)
{
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/a", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/b", "sensor_msgs/msg/Image"));
  if (with_messages) {
    const std::vector<std::byte> small(64, std::byte{0x41});
    const std::vector<std::byte> large(4096, std::byte{0x42});
    writer->write("/a", kFirstStampNs, view(small));
    writer->write("/b", kMiddleStampNs, view(large));
    writer->write("/a", kLastStampNs, view(small));
  }
  writer->close();
}

CreateOptions mcap_options(Layout layout, std::string compression)
{
  CreateOptions options;
  options.format = Format::Mcap;
  options.layout = layout;
  options.mcap_compression = std::move(compression);
  return options;
}

CreateOptions sqlite3_options(Layout layout, std::string mode = "", std::string format = "")
{
  CreateOptions options;
  options.format = Format::Sqlite3;
  options.layout = layout;
  options.sqlite3_compression_mode = std::move(mode);
  options.sqlite3_compression_format = std::move(format);
  return options;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void write_file(const std::filesystem::path & path, const std::string & text)
{
  std::ofstream out(path, std::ios::trunc);
  out << text;
}

void expect_fixture_summary(const BagDescription & d)
{
  ASSERT_TRUE(d.topics.has_value());
  EXPECT_EQ(d.topics->size(), 2u);
  EXPECT_EQ(d.message_count, kMessageCount);
  EXPECT_EQ(d.start_ns, kFirstStampNs);
  EXPECT_EQ(d.end_ns, kLastStampNs);
}

class BagDescribeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_describe_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(BagDescribeTest, McapDirectoryWithZstdChunksReportsChunkCompression)
{
  const auto bag = tmp_dir_ / "zstd_dir";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.layout, Layout::Directory);
  EXPECT_EQ(d.format, Format::Mcap);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  ASSERT_TRUE(d.compression.compressed_bytes.has_value());
  ASSERT_TRUE(d.compression.uncompressed_bytes.has_value());
  EXPECT_LT(*d.compression.compressed_bytes, *d.compression.uncompressed_bytes);

  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].path, std::filesystem::path("zstd_dir_0.mcap"));
  ASSERT_TRUE(d.files[0].size_bytes.has_value());
  EXPECT_EQ(*d.files[0].size_bytes, std::filesystem::file_size(bag / "zstd_dir_0.mcap"));
  EXPECT_EQ(d.size_bytes, *d.files[0].size_bytes);
  EXPECT_EQ(d.files[0].message_count, kMessageCount);
  EXPECT_EQ(d.files[0].start_ns, kFirstStampNs);
  EXPECT_EQ(d.files[0].end_ns, kLastStampNs);

  EXPECT_TRUE(d.has_metadata_yaml);
  EXPECT_EQ(d.metadata_version, 5);
  EXPECT_TRUE(d.ros_distro.empty());
  expect_fixture_summary(d);
}

TEST_F(BagDescribeTest, McapSingleFileUncompressedReportsNone)
{
  const auto bag = tmp_dir_ / "plain.mcap";
  write_fixture(bag, mcap_options(Layout::SingleFile, "none"));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.layout, Layout::SingleFile);
  EXPECT_EQ(d.format, Format::Mcap);
  EXPECT_EQ(d.compression.mode, "none");
  EXPECT_TRUE(d.compression.codecs.empty());
  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].path, std::filesystem::path("plain.mcap"));
  EXPECT_EQ(d.size_bytes, std::filesystem::file_size(bag));
  // The single file is the whole bag, so its own row carries the summary.
  EXPECT_EQ(d.files[0].message_count, kMessageCount);
  EXPECT_EQ(d.files[0].start_ns, kFirstStampNs);
  EXPECT_EQ(d.files[0].end_ns, kLastStampNs);
  EXPECT_FALSE(d.has_metadata_yaml);
  EXPECT_FALSE(d.metadata_version.has_value());
  expect_fixture_summary(d);
}

TEST_F(BagDescribeTest, McapSingleFileLz4ReportsLz4Chunks)
{
  const auto bag = tmp_dir_ / "lz4.mcap";
  write_fixture(bag, mcap_options(Layout::SingleFile, "lz4"));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"lz4"});
}

// A bag that was never finalized has no summary section. Nothing is scanned
// to make up for it: the codec, topics and counts all read unknown, while the
// file system facts are still reported.
TEST_F(BagDescribeTest, McapWithoutSummaryReportsUnknownsInsteadOfScanning)
{
  const auto bag = tmp_dir_ / "truncated.mcap";
  write_fixture(bag, mcap_options(Layout::SingleFile, "none"));
  const auto full_size = std::filesystem::file_size(bag);
  std::filesystem::resize_file(bag, full_size - 64);

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.format, Format::Mcap);
  EXPECT_EQ(d.compression.mode, "unknown");
  EXPECT_FALSE(d.topics.has_value());
  EXPECT_FALSE(d.message_count.has_value());
  EXPECT_FALSE(d.start_ns.has_value());
  EXPECT_FALSE(d.end_ns.has_value());
  EXPECT_EQ(d.size_bytes, full_size - 64);
}

TEST_F(BagDescribeTest, Sqlite3DirectoryAnswersFromMetadataYaml)
{
  const auto bag = tmp_dir_ / "plain_dir";
  write_fixture(bag, sqlite3_options(Layout::Directory));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.layout, Layout::Directory);
  EXPECT_EQ(d.format, Format::Sqlite3);
  EXPECT_EQ(d.compression.mode, "none");
  EXPECT_TRUE(d.compression.codecs.empty());
  EXPECT_FALSE(d.compression.compressed_bytes.has_value());
  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].path, std::filesystem::path("plain_dir_0.db3"));
  // Per-file timing comes from metadata.yaml's `files:` entry; no shard is
  // opened for it.
  EXPECT_EQ(d.files[0].message_count, kMessageCount);
  EXPECT_EQ(d.files[0].start_ns, kFirstStampNs);
  EXPECT_EQ(d.files[0].end_ns, kLastStampNs);
  EXPECT_TRUE(d.has_metadata_yaml);
  expect_fixture_summary(d);
}

TEST_F(BagDescribeTest, Sqlite3DirectoryMessageModeReportsPerMessageZstd)
{
  const auto bag = tmp_dir_ / "message_dir";
  write_fixture(bag, sqlite3_options(Layout::Directory, "message", "zstd"));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.compression.mode, "message");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  expect_fixture_summary(d);
}

TEST_F(BagDescribeTest, Sqlite3DirectoryFileModeReportsEnvelopeWithoutDecompressing)
{
  const auto bag = tmp_dir_ / "file_dir";
  write_fixture(bag, sqlite3_options(Layout::Directory, "file", "zstd"));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.compression.mode, "file");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].path.extension(), ".zstd");
  ASSERT_TRUE(d.files[0].size_bytes.has_value());
  EXPECT_EQ(*d.files[0].size_bytes, std::filesystem::file_size(bag / d.files[0].path));
  expect_fixture_summary(d);
}

// A bare `.db3.zstd` shard carries no summary outside its envelope, and
// opening the envelope means decompressing the whole database. The
// description stops at what the file system and the extension say.
TEST_F(BagDescribeTest, Sqlite3EnvelopeAloneLeavesContentsUnknown)
{
  const auto dir = tmp_dir_ / "file_dir";
  write_fixture(dir, sqlite3_options(Layout::Directory, "file", "zstd"));
  const auto shard = dir / "file_dir_0.db3.zstd";
  ASSERT_TRUE(std::filesystem::exists(shard));

  const auto d = describe_bag(shard);

  EXPECT_EQ(d.layout, Layout::SingleFile);
  EXPECT_EQ(d.format, Format::Sqlite3);
  EXPECT_EQ(d.compression.mode, "file");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  EXPECT_FALSE(d.topics.has_value());
  EXPECT_FALSE(d.message_count.has_value());
  EXPECT_FALSE(d.start_ns.has_value());
  EXPECT_EQ(d.size_bytes, std::filesystem::file_size(shard));
}

TEST_F(BagDescribeTest, Sqlite3SingleFileAnswersFromEmbeddedMetadataRow)
{
  const auto bag = tmp_dir_ / "single.db3";
  write_fixture(bag, sqlite3_options(Layout::SingleFile));

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.layout, Layout::SingleFile);
  EXPECT_EQ(d.format, Format::Sqlite3);
  EXPECT_EQ(d.compression.mode, "none");
  EXPECT_FALSE(d.has_metadata_yaml);
  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].message_count, kMessageCount);
  EXPECT_EQ(d.files[0].start_ns, kFirstStampNs);
  EXPECT_EQ(d.files[0].end_ns, kLastStampNs);
  expect_fixture_summary(d);
}

// A humble-era recording has an empty (or absent) `metadata` table. The
// topic list and the time extent are still cheap — the topics table and the
// timestamp index — but the message count would need a table scan, so it
// stays unknown.
TEST_F(BagDescribeTest, Sqlite3SingleFileWithoutMetadataRowLeavesCountUnknown)
{
  const auto bag = tmp_dir_ / "humble.db3";
  write_fixture(bag, sqlite3_options(Layout::SingleFile));
  {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open(bag.c_str(), &db), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db, "DROP TABLE metadata", nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(db);
  }

  const auto d = describe_bag(bag);

  ASSERT_TRUE(d.topics.has_value());
  EXPECT_EQ(d.topics->size(), 2u);
  EXPECT_FALSE(d.message_count.has_value());
  EXPECT_EQ(d.start_ns, kFirstStampNs);
  EXPECT_EQ(d.end_ns, kLastStampNs);
}

TEST_F(BagDescribeTest, DirectoryWithoutMetadataYamlIsReconstructedFromShards)
{
  const auto bag = tmp_dir_ / "no_yaml";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"));
  ASSERT_TRUE(std::filesystem::remove(bag / "metadata.yaml"));

  const auto d = describe_bag(bag);

  EXPECT_FALSE(d.has_metadata_yaml);
  EXPECT_FALSE(d.metadata_version.has_value());
  EXPECT_EQ(d.format, Format::Mcap);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  ASSERT_EQ(d.files.size(), 1u);
  EXPECT_EQ(d.files[0].path, std::filesystem::path("no_yaml_0.mcap"));
  expect_fixture_summary(d);
}

TEST_F(BagDescribeTest, MetadataYamlVersionAndRosDistroAreReported)
{
  const auto bag = tmp_dir_ / "distro_dir";
  write_fixture(bag, sqlite3_options(Layout::Directory));
  const auto yaml_path = bag / "metadata.yaml";
  auto yaml = read_file(yaml_path);
  const auto pos = yaml.find("version: 5");
  ASSERT_NE(pos, std::string::npos) << yaml;
  yaml.replace(pos, std::string("version: 5").size(), "version: 8\n  ros_distro: jazzy");
  write_file(yaml_path, yaml);

  const auto d = describe_bag(bag);

  EXPECT_EQ(d.metadata_version, 8);
  EXPECT_EQ(d.ros_distro, "jazzy");
}

TEST_F(BagDescribeTest, EmptyBagHasZeroMessagesAndNoExtent)
{
  const auto bag = tmp_dir_ / "empty_dir";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"), /*with_messages=*/false);

  const auto d = describe_bag(bag);

  ASSERT_TRUE(d.topics.has_value());
  EXPECT_EQ(d.topics->size(), 2u);
  EXPECT_EQ(d.message_count, 0);
  EXPECT_FALSE(d.start_ns.has_value());
  EXPECT_FALSE(d.end_ns.has_value());
}

TEST_F(BagDescribeTest, NonexistentPathThrows)
{
  EXPECT_THROW(describe_bag(tmp_dir_ / "missing"), std::runtime_error);
}

}  // namespace
