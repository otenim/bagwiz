// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/file_decompressor.hpp"  // is_zstd_file
#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

// Deterministic 4-byte payload; bagwiz writers do not interpret the payload,
// so any byte sequence is valid for round-trip testing.
constexpr std::array<std::uint8_t, 4> kPayload{0xDE, 0xAD, 0xBE, 0xEF};

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

void write_fixture(const std::filesystem::path & path, const bagwiz::io::CreateOptions & options)
{
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/foo", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/bar", "std_msgs/msg/Int32"));
  const std::vector<std::pair<std::string, int64_t>> messages = {
    {"/foo", 1'000'000'000LL},
    {"/foo", 1'000'000'001LL},
    {"/foo", 1'000'000'002LL},
    {"/bar", 2'000'000'000LL},
    {"/bar", 2'000'000'001LL}};
  for (const auto & [topic, ts] : messages) {
    writer->write(
      topic, ts,
      std::span<const std::byte>(
        reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          kPayload.data()),
        kPayload.size()));
  }
  writer->close();
}

void verify_round_trip(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);

  int foo_count = 0;
  int bar_count = 0;
  bagwiz::io::RawMessage msg;
  while (reader->next(msg)) {
    ASSERT_EQ(msg.payload.size(), kPayload.size());
    EXPECT_TRUE(
      std::equal(
        msg.payload.begin(), msg.payload.end(),
        reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          kPayload.data())))
      << "payload bytes did not round-trip on " << msg.topic->name;
    if (msg.topic->name == "/foo") {
      ++foo_count;
    } else if (msg.topic->name == "/bar") {
      ++bar_count;
    }
  }
  EXPECT_EQ(foo_count, 3);
  EXPECT_EQ(bar_count, 2);
}

bagwiz::io::CreateOptions sqlite3_dir_options(
  const std::string & mode, const std::string & format, const std::string & level = "")
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::Directory;
  options.sqlite3_compression_mode = mode;
  options.sqlite3_compression_format = format;
  options.sqlite3_compression_level = level;
  return options;
}

class Sqlite3CompressionWriteTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_sqlite3_compress_test_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(Sqlite3CompressionWriteTest, MessageModeDirectoryRoundTrips)
{
  const auto dir = tmp_dir_ / "message_mode";
  write_fixture(dir, sqlite3_dir_options("message", "zstd"));

  // metadata.yaml carries the rosbag2 MESSAGE-mode declaration, pointing at
  // the plain (un-enveloped) shard name.
  const auto md = bagwiz::io::load_metadata_yaml(dir / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0], dir.filename().string() + "_0.db3");

  // The shard on disk really holds zstd frames, not raw payloads.
  const auto shard = dir / md.relative_file_paths[0];
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(shard.string().c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(sqlite3_prepare_v2(db, "SELECT data FROM messages", -1, &stmt, nullptr), SQLITE_OK);
  int rows = 0;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ++rows;
    const auto * blob = static_cast<const std::byte *>(sqlite3_column_blob(stmt, 0));
    const int size = sqlite3_column_bytes(stmt, 0);
    ASSERT_GT(size, 4);
    EXPECT_TRUE(bagwiz::io::is_zstd_magic({blob, static_cast<std::size_t>(size)}))
      << "messages.data is not a bare zstd frame";
  }
  sqlite3_finalize(stmt);
  sqlite3_close(db);
  EXPECT_EQ(rows, 5);

  // And the shared reader path (metadata-driven MessageDecompressor) returns
  // the original bytes.
  verify_round_trip(dir);
}

TEST_F(Sqlite3CompressionWriteTest, MessageModeEmbeddedRowDeclaresCompression)
{
  // The shard's embedded `metadata` row (rosbag2 iron+ self-description)
  // must agree with metadata.yaml — a jazzy reader parsing the row should
  // see the same MESSAGE-mode declaration.
  const auto dir = tmp_dir_ / "message_embedded";
  write_fixture(dir, sqlite3_dir_options("message", "zstd"));

  const auto shard = dir / (dir.filename().string() + "_0.db3");
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(shard.string().c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(sqlite3_prepare_v2(db, "SELECT metadata FROM metadata", -1, &stmt, nullptr), SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const std::string yaml = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
  sqlite3_finalize(stmt);
  sqlite3_close(db);

  const auto info = YAML::Load(yaml);
  EXPECT_EQ(info["compression_format"].as<std::string>(), "zstd");
  EXPECT_EQ(info["compression_mode"].as<std::string>(), "message");
}

TEST_F(Sqlite3CompressionWriteTest, FileModeDirectoryRoundTrips)
{
  const auto dir = tmp_dir_ / "file_mode";
  write_fixture(dir, sqlite3_dir_options("file", "zstd"));

  // metadata.yaml declares the FILE-mode envelope and points at the
  // compressed shard; the plain shard must not remain on disk.
  const auto md = bagwiz::io::load_metadata_yaml(dir / "metadata.yaml");
  EXPECT_EQ(md.compression_mode, "file");
  EXPECT_EQ(md.compression_format, "zstd");
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0], dir.filename().string() + "_0.db3.zstd");

  const auto envelope = dir / md.relative_file_paths[0];
  EXPECT_TRUE(std::filesystem::exists(envelope));
  EXPECT_TRUE(bagwiz::io::is_zstd_file(envelope));
  EXPECT_FALSE(std::filesystem::exists(dir / (dir.filename().string() + "_0.db3")))
    << "plain shard was not removed after envelope compression";

  // The envelope-decompressing reader path returns the original bytes.
  verify_round_trip(dir);
}

TEST_F(Sqlite3CompressionWriteTest, FileModeEnvelopeEmbedsPlainMetadataRow)
{
  // The .db3 inside the envelope was written as plain storage, so its
  // embedded `metadata` row must NOT claim compression — after a reader
  // expands the envelope, the row describes the bytes it sits in.
  const auto dir = tmp_dir_ / "file_embedded";
  write_fixture(dir, sqlite3_dir_options("file", "zstd"));

  const auto temp =
    bagwiz::io::decompress_zstd_file_to_temp(dir / (dir.filename().string() + "_0.db3.zstd"));
  sqlite3 * db = nullptr;
  ASSERT_EQ(sqlite3_open(temp.path().string().c_str(), &db), SQLITE_OK);
  sqlite3_stmt * stmt = nullptr;
  ASSERT_EQ(sqlite3_prepare_v2(db, "SELECT metadata FROM metadata", -1, &stmt, nullptr), SQLITE_OK);
  ASSERT_EQ(sqlite3_step(stmt), SQLITE_ROW);
  const std::string yaml = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
  sqlite3_finalize(stmt);
  sqlite3_close(db);

  const auto info = YAML::Load(yaml);
  EXPECT_EQ(info["compression_format"].as<std::string>(), "");
  EXPECT_EQ(info["compression_mode"].as<std::string>(), "");
}

TEST_F(Sqlite3CompressionWriteTest, ExplicitNoneMatchesHistoricalLayout)
{
  // --mode none must produce byte-for-byte the historical plain layout:
  // empty compression fields and an untouched shard name.
  const auto dir = tmp_dir_ / "none_mode";
  write_fixture(dir, sqlite3_dir_options("none", "none"));

  const auto md = bagwiz::io::load_metadata_yaml(dir / "metadata.yaml");
  EXPECT_TRUE(
    md.compression_mode.empty() || md.compression_mode == "NONE" || md.compression_mode == "none");
  EXPECT_TRUE(md.compression_format.empty());
  ASSERT_EQ(md.relative_file_paths.size(), 1U);
  EXPECT_EQ(md.relative_file_paths[0], dir.filename().string() + "_0.db3");
  verify_round_trip(dir);
}

TEST_F(Sqlite3CompressionWriteTest, LevelNamesAreAccepted)
{
  const auto dir = tmp_dir_ / "leveled";
  write_fixture(dir, sqlite3_dir_options("message", "zstd", "slowest"));
  verify_round_trip(dir);
}

TEST_F(Sqlite3CompressionWriteTest, SingleFileRejectsCompression)
{
  // A bare .db3 has no metadata.yaml to declare compression, and bagwiz's
  // single-file reader has no decompressor hook — writing one would produce
  // a bag bagwiz itself misreads.
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;

  options.sqlite3_compression_mode = "message";
  options.sqlite3_compression_format = "zstd";
  EXPECT_THROW(bagwiz::io::open_write(tmp_dir_ / "message.db3", options), std::runtime_error);

  options.sqlite3_compression_mode = "file";
  EXPECT_THROW(bagwiz::io::open_write(tmp_dir_ / "file.db3", options), std::runtime_error);
}

TEST_F(Sqlite3CompressionWriteTest, RejectsNonZstdFormat)
{
  EXPECT_THROW(
    write_fixture(tmp_dir_ / "lz4", sqlite3_dir_options("message", "lz4")), std::runtime_error);
}

TEST_F(Sqlite3CompressionWriteTest, RejectsUnknownMode)
{
  EXPECT_THROW(
    write_fixture(tmp_dir_ / "chunk", sqlite3_dir_options("chunk", "zstd")), std::runtime_error);
}

TEST_F(Sqlite3CompressionWriteTest, ModeAndFormatMustAgree)
{
  // A format without a mode (or vice versa) is a caller bug; fail fast at
  // open_write() rather than guessing.
  EXPECT_THROW(
    write_fixture(tmp_dir_ / "format_only", sqlite3_dir_options("none", "zstd")),
    std::runtime_error);
  EXPECT_THROW(
    write_fixture(tmp_dir_ / "mode_only", sqlite3_dir_options("message", "none")),
    std::runtime_error);
}

TEST_F(Sqlite3CompressionWriteTest, RejectsUnknownLevelName)
{
  EXPECT_THROW(
    write_fixture(tmp_dir_ / "badlevel", sqlite3_dir_options("message", "zstd", "ludicrous")),
    std::runtime_error);
}

}  // namespace
