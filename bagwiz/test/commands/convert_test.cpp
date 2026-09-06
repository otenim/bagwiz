// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/convert.hpp"

#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr std::int64_t kT0 = 1'000'000'000LL;
constexpr std::int64_t kSecond = 1'000'000'000LL;

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

// Large, repetitive payloads: a codec that lands on the chunks has to
// shrink them, or libmcap stores the chunk plain and the output would read
// as uncompressed whatever the writer was asked for.
std::span<const std::byte> payload_view()
{
  static const std::vector<std::byte> bytes(4096, std::byte{0x42});
  return {bytes.data(), bytes.size()};
}

bagwiz::io::CreateOptions mcap_opts(bagwiz::io::Layout layout, const std::string & codec)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = layout;
  opts.mcap_compression = codec;
  return opts;
}

bagwiz::io::CreateOptions sqlite3_opts(
  bagwiz::io::Layout layout, const std::string & mode = "", const std::string & format = "")
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Sqlite3;
  opts.layout = layout;
  opts.sqlite3_compression_mode = mode;
  opts.sqlite3_compression_format = format;
  return opts;
}

// /fast with 5 messages, /slow with 2, written through `opts`.
std::filesystem::path build_input(
  const std::filesystem::path & path, const bagwiz::io::CreateOptions & opts)
{
  auto writer = bagwiz::io::open_write(path, opts);
  writer->declare_topic(make_topic("/fast", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/slow", "std_msgs/msg/String"));
  for (int i = 0; i <= 4; ++i) {
    writer->write("/fast", kT0 + i * kSecond, payload_view());
  }
  writer->write("/slow", kT0 + kSecond / 2, payload_view());
  writer->write("/slow", kT0 + 2 * kSecond + kSecond / 2, payload_view());
  writer->close();
  return path;
}

// Per-topic message counts of the bag at `path`, asserting every payload
// round-trips byte-for-byte.
std::map<std::string, int> collect_counts(const std::filesystem::path & path)
{
  auto reader = bagwiz::io::open_read(path);
  std::map<std::string, int> counts;
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    EXPECT_EQ(raw.payload.size(), payload_view().size());
    ++counts[raw.topic->name];
  }
  return counts;
}

void expect_fixture_intact(const std::filesystem::path & path)
{
  const auto counts = collect_counts(path);
  ASSERT_EQ(counts.size(), 2U);
  EXPECT_EQ(counts.at("/fast"), 5);
  EXPECT_EQ(counts.at("/slow"), 2);
}

bagwiz::commands::ConvertFormatArgs make_args(
  const std::filesystem::path & input, const std::filesystem::path & output)
{
  bagwiz::commands::ConvertFormatArgs args;
  args.input_path = input;
  args.output_path = output;
  return args;
}

class ConvertTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_convert_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

// ---------------------------------------------------------------------------
// Storage and layout resolution.
// ---------------------------------------------------------------------------

TEST_F(ConvertTest, DirectoryOutputInheritsTheInputsStorage)
{
  const auto in = build_input(tmp_dir_ / "in.db3", sqlite3_opts(bagwiz::io::Layout::SingleFile));
  const auto out = tmp_dir_ / "out_dir";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  ASSERT_TRUE(std::filesystem::is_directory(out));
  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Sqlite3);
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, OutputExtensionPicksTheStorage)
{
  const auto in = build_input(tmp_dir_ / "in.db3", sqlite3_opts(bagwiz::io::Layout::SingleFile));
  const auto out = tmp_dir_ / "out.mcap";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  ASSERT_FALSE(std::filesystem::is_directory(out));
  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Mcap);
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, StorageFlagOutranksTheInputForADirectoryOutput)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "none"));
  const auto out = tmp_dir_ / "out_dir";

  auto args = make_args(in, out);
  args.storage = "sqlite3";
  ASSERT_EQ(bagwiz::commands::run_convert_format(args), 0);

  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Sqlite3);
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, DirectoryInputToSingleFileTakesTheExtensionsStorage)
{
  const auto in =
    build_input(tmp_dir_ / "in_dir", mcap_opts(bagwiz::io::Layout::Directory, "none"));
  const auto out = tmp_dir_ / "out.db3";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  ASSERT_FALSE(std::filesystem::is_directory(out));
  EXPECT_EQ(bagwiz::io::detect_format(out), bagwiz::io::Format::Sqlite3);
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, SameStorageAndLayoutIsRejected)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "none"));
  const auto out = tmp_dir_ / "out.mcap";

  EXPECT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 1);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(ConvertTest, ExistingOutputRequiresOverwrite)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "none"));
  const auto out = tmp_dir_ / "out_dir";
  std::filesystem::create_directories(out);

  EXPECT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 1);

  auto args = make_args(in, out);
  args.overwrite = true;
  ASSERT_EQ(bagwiz::commands::run_convert_format(args), 0);
  expect_fixture_intact(out);
}

// ---------------------------------------------------------------------------
// Compression is carried over, translated to the target storage. The codec
// on the output is read back the way `bagwiz info` reads it.
// ---------------------------------------------------------------------------

TEST_F(ConvertTest, ZstdMcapFileToDirectoryKeepsZstdChunks)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));
  const auto out = tmp_dir_ / "out_dir";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  const auto d = bagwiz::io::describe_bag(out);
  EXPECT_EQ(d.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  // metadata.yaml never records chunk compression: rosbag2 would treat it as
  // a whole-file envelope and fail to open the bag.
  const auto md = bagwiz::io::load_metadata_yaml(out / "metadata.yaml");
  EXPECT_TRUE(md.compression_format.empty());
  EXPECT_TRUE(md.compression_mode.empty());
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, Lz4McapDirectoryToFileKeepsLz4Chunks)
{
  const auto in = build_input(tmp_dir_ / "in_dir", mcap_opts(bagwiz::io::Layout::Directory, "lz4"));
  const auto out = tmp_dir_ / "out.mcap";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  const auto d = bagwiz::io::describe_bag(out);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"lz4"});
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, PlainMcapStaysPlainAcrossTheRepack)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "none"));
  const auto out = tmp_dir_ / "out_dir";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  EXPECT_EQ(bagwiz::io::describe_bag(out).compression.mode, "none");
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, ZstdMcapToSqlite3DirectoryBecomesMessageMode)
{
  const auto in =
    build_input(tmp_dir_ / "in.mcap", mcap_opts(bagwiz::io::Layout::SingleFile, "zstd"));
  const auto out = tmp_dir_ / "out_dir";

  auto args = make_args(in, out);
  args.storage = "sqlite3";
  ASSERT_EQ(bagwiz::commands::run_convert_format(args), 0);

  const auto md = bagwiz::io::load_metadata_yaml(out / "metadata.yaml");
  EXPECT_EQ(md.storage_identifier, "sqlite3");
  EXPECT_EQ(md.compression_mode, "message");
  EXPECT_EQ(md.compression_format, "zstd");
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, MessageModeSqlite3DirectoryToMcapBecomesZstdChunks)
{
  const auto in = build_input(
    tmp_dir_ / "in_dir", sqlite3_opts(bagwiz::io::Layout::Directory, "message", "zstd"));
  const auto out = tmp_dir_ / "out.mcap";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  const auto d = bagwiz::io::describe_bag(out);
  EXPECT_EQ(d.format, bagwiz::io::Format::Mcap);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, FileModeSqlite3DirectoryToMcapBecomesZstdChunks)
{
  const auto in =
    build_input(tmp_dir_ / "in_dir", sqlite3_opts(bagwiz::io::Layout::Directory, "file", "zstd"));
  const auto out = tmp_dir_ / "out.mcap";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  const auto d = bagwiz::io::describe_bag(out);
  EXPECT_EQ(d.compression.mode, "chunk");
  EXPECT_EQ(d.compression.codecs, std::vector<std::string>{"zstd"});
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, MessageModeSqlite3DirectoryToSingleFileIsWrittenPlain)
{
  // A bare .db3 cannot carry compression: rosbag2 reads the mode from
  // metadata.yaml alone. The repack still succeeds, plain, with a warning.
  const auto in = build_input(
    tmp_dir_ / "in_dir", sqlite3_opts(bagwiz::io::Layout::Directory, "message", "zstd"));
  const auto out = tmp_dir_ / "out.db3";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  ASSERT_FALSE(std::filesystem::is_directory(out));
  EXPECT_EQ(bagwiz::io::describe_bag(out).compression.mode, "none");
  expect_fixture_intact(out);
}

TEST_F(ConvertTest, PlainSqlite3FileToMcapStaysPlain)
{
  const auto in = build_input(tmp_dir_ / "in.db3", sqlite3_opts(bagwiz::io::Layout::SingleFile));
  const auto out = tmp_dir_ / "out.mcap";

  ASSERT_EQ(bagwiz::commands::run_convert_format(make_args(in, out)), 0);

  EXPECT_EQ(bagwiz::io::describe_bag(out).compression.mode, "none");
  expect_fixture_intact(out);
}
