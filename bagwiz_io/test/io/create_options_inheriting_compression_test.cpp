// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// create_options_inheriting_compression against real bags of every
// compression shape bagwiz can write, checking that the knobs it fills
// reproduce the input's compression on the output storage the sibling
// helpers resolve. The cell-by-cell translation itself is covered by
// inherit_compression_test; these tests prove the plumbing from a bag on
// disk to the CreateOptions a writer consumes.

namespace
{

using bagwiz::io::CreateOptions;
using bagwiz::io::Format;
using bagwiz::io::Layout;
using bagwiz::io::TopicInfo;

TopicInfo make_topic(std::string name, std::string type)
{
  TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

// Compressible payloads: libmcap stores a chunk uncompressed when the codec
// does not shrink it, and a fixture whose chunks came out plain would read
// as an uncompressed input.
void write_fixture(const std::filesystem::path & path, const CreateOptions & options)
{
  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(make_topic("/a", "std_msgs/msg/String"));
  const std::vector<std::byte> payload(4096, std::byte{0x42});
  const std::span<const std::byte> view(payload.data(), payload.size());
  writer->write("/a", 1'000'000'000LL, view);
  writer->write("/a", 2'000'000'000LL, view);
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

// The options a -o rewrite composes before compression is decided: the
// output's format and layout, from its extension or else the reference. The
// compression knobs are seeded away from their defaults (zstd chunks, plain
// sqlite3), so a test expecting "zstd" or "none" back proves the helper
// wrote it rather than that it left the defaults alone.
CreateOptions output_options(
  const std::filesystem::path & reference, const std::filesystem::path & output)
{
  auto options = bagwiz::io::create_options_inheriting_format(reference, output);
  options.mcap_compression = "lz4";
  options.sqlite3_compression_mode = "message";
  options.sqlite3_compression_format = "zstd";
  return options;
}

class CreateOptionsInheritingCompressionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_create_options_inheriting_compression_" +
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

}  // namespace

// ---------------------------------------------------------------------------
// MCAP inputs.
// ---------------------------------------------------------------------------

TEST_F(CreateOptionsInheritingCompressionTest, ZstdMcapFileToMcapFileKeepsZstd)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "zstd"));
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "zstd");
  // Only the compression knobs change; the storage resolution passed in
  // comes back as it was.
  EXPECT_EQ(opts.format, Format::Auto);
  EXPECT_EQ(opts.layout, Layout::Auto);
}

TEST_F(CreateOptionsInheritingCompressionTest, Lz4McapFileToDirectoryKeepsLz4)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "lz4"));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "lz4");
  EXPECT_EQ(opts.format, Format::Mcap);
  EXPECT_EQ(opts.layout, Layout::Directory);
}

TEST_F(CreateOptionsInheritingCompressionTest, PlainMcapDirectoryToMcapFileStaysPlain)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, mcap_options(Layout::Directory, "none"));
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "none");
}

TEST_F(CreateOptionsInheritingCompressionTest, ZstdMcapToSqlite3DirectoryBecomesMessageMode)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "zstd"));
  const auto out = tmp_dir_ / "out_dir";

  // A command that pins the backend (`convert --storage sqlite3`) composes
  // Sqlite3 + Auto and lets the extension-less path resolve to a directory.
  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, sqlite3_options(Layout::Auto));

  EXPECT_EQ(opts.sqlite3_compression_mode, "message");
  EXPECT_EQ(opts.sqlite3_compression_format, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, Lz4McapToSqlite3DirectoryBecomesMessageModeZstd)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "lz4"));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, sqlite3_options(Layout::Auto));

  EXPECT_EQ(opts.sqlite3_compression_mode, "message");
  EXPECT_EQ(opts.sqlite3_compression_format, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, ZstdMcapToSqlite3FileIsWrittenPlain)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "zstd"));
  const auto out = tmp_dir_ / "out.db3";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  // The single-file sqlite3 writer refuses any compression, so the knobs
  // must come back as an explicit "none" pair a writer accepts.
  EXPECT_EQ(opts.sqlite3_compression_mode, "none");
  EXPECT_EQ(opts.sqlite3_compression_format, "none");
  EXPECT_NO_THROW(bagwiz::io::open_write(out, opts)->close());
}

TEST_F(CreateOptionsInheritingCompressionTest, PlainMcapToSqlite3FileStaysPlain)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "none"));
  const auto out = tmp_dir_ / "out.db3";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.sqlite3_compression_mode, "none");
  EXPECT_EQ(opts.sqlite3_compression_format, "none");
}

// ---------------------------------------------------------------------------
// SQLite3 inputs.
// ---------------------------------------------------------------------------

TEST_F(CreateOptionsInheritingCompressionTest, MessageModeDirectoryToDirectoryKeepsMessageMode)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory, "message", "zstd"));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.format, Format::Sqlite3);
  EXPECT_EQ(opts.layout, Layout::Directory);
  EXPECT_EQ(opts.sqlite3_compression_mode, "message");
  EXPECT_EQ(opts.sqlite3_compression_format, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, MessageModeDirectoryToMcapFileBecomesZstdChunks)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory, "message", "zstd"));
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, MessageModeDirectoryToSqlite3FileIsWrittenPlain)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory, "message", "zstd"));
  const auto out = tmp_dir_ / "out.db3";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.sqlite3_compression_mode, "none");
  EXPECT_EQ(opts.sqlite3_compression_format, "none");
}

TEST_F(CreateOptionsInheritingCompressionTest, FileModeDirectoryToDirectoryKeepsFileMode)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory, "file", "zstd"));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.format, Format::Sqlite3);
  EXPECT_EQ(opts.layout, Layout::Directory);
  EXPECT_EQ(opts.sqlite3_compression_mode, "file");
  EXPECT_EQ(opts.sqlite3_compression_format, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, FileModeDirectoryToMcapFileBecomesZstdChunks)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory, "file", "zstd"));
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, BareEnvelopeToDirectoryKeepsFileMode)
{
  // A bare `.db3.zstd` shard: the envelope says FILE mode without being
  // opened, and the directory writer can reproduce it.
  const auto dir = tmp_dir_ / "in_dir";
  write_fixture(dir, sqlite3_options(Layout::Directory, "file", "zstd"));
  const auto in = dir / "in_dir_0.db3.zstd";
  ASSERT_TRUE(std::filesystem::exists(in));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.format, Format::Sqlite3);
  EXPECT_EQ(opts.layout, Layout::Directory);
  EXPECT_EQ(opts.sqlite3_compression_mode, "file");
  EXPECT_EQ(opts.sqlite3_compression_format, "zstd");
}

TEST_F(CreateOptionsInheritingCompressionTest, PlainSqlite3DirectoryToMcapStaysPlain)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, sqlite3_options(Layout::Directory));
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "none");
}

TEST_F(CreateOptionsInheritingCompressionTest, PlainSqlite3FileToDirectoryStaysPlain)
{
  const auto in = tmp_dir_ / "in.db3";
  write_fixture(in, sqlite3_options(Layout::SingleFile));
  const auto out = tmp_dir_ / "out_dir";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.sqlite3_compression_mode, "none");
  EXPECT_EQ(opts.sqlite3_compression_format, "none");
}

// ---------------------------------------------------------------------------
// In-place shape: the output path is the input itself, storage pinned.
// ---------------------------------------------------------------------------

TEST_F(CreateOptionsInheritingCompressionTest, InPlaceOptionsKeepTheInputsCompression)
{
  const auto in = tmp_dir_ / "in_dir";
  write_fixture(in, mcap_options(Layout::Directory, "lz4"));

  const auto pinned = bagwiz::io::create_options_preserving_storage(in);
  const auto opts = bagwiz::io::create_options_inheriting_compression(in, in, pinned);

  EXPECT_EQ(opts.format, Format::Mcap);
  EXPECT_EQ(opts.layout, Layout::Directory);
  EXPECT_EQ(opts.mcap_compression, "lz4");
}

// ---------------------------------------------------------------------------
// Nothing readable to carry over: the output is written plain, never with
// compression the input may not have had.
// ---------------------------------------------------------------------------

TEST_F(CreateOptionsInheritingCompressionTest, McapWithoutSummaryIsWrittenPlain)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "none"));
  std::filesystem::resize_file(in, std::filesystem::file_size(in) - 64);
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "none");
}

TEST_F(CreateOptionsInheritingCompressionTest, McapWithoutSummaryToSqlite3DirectoryIsWrittenPlain)
{
  const auto in = tmp_dir_ / "in.mcap";
  write_fixture(in, mcap_options(Layout::SingleFile, "zstd"));
  std::filesystem::resize_file(in, std::filesystem::file_size(in) - 64);
  const auto out = tmp_dir_ / "out_dir";

  auto given = sqlite3_options(Layout::Auto, "message", "zstd");
  const auto opts = bagwiz::io::create_options_inheriting_compression(in, out, given);

  EXPECT_EQ(opts.sqlite3_compression_mode, "none");
  EXPECT_EQ(opts.sqlite3_compression_format, "none");
}

TEST_F(CreateOptionsInheritingCompressionTest, MissingReferenceIsWrittenPlain)
{
  const auto in = tmp_dir_ / "does_not_exist";
  const auto out = tmp_dir_ / "out.mcap";

  const auto opts =
    bagwiz::io::create_options_inheriting_compression(in, out, output_options(in, out));

  EXPECT_EQ(opts.mcap_compression, "none");
  // Only the compression knobs change; the storage resolution passed in
  // comes back as it was.
  EXPECT_EQ(opts.format, Format::Auto);
  EXPECT_EQ(opts.layout, Layout::Auto);
}
