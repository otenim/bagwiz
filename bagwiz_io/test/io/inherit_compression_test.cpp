// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "io/inherit_compression.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include "bagwiz/io/bag_describe.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <utility>
#include <vector>

// The pure translation table behind create_options_inheriting_compression:
// what `bagwiz info` reports for the input, crossed with the storage and
// layout the output resolves to. The table is small enough to spell out
// cell by cell, which is what makes the I/O wrapper's fixture tests
// readable — they only have to prove the plumbing.

namespace
{

using bagwiz::io::BagCompression;
using bagwiz::io::Format;
using bagwiz::io::Layout;
using bagwiz::io::ResolvedWriteLayout;
using bagwiz::io::detail::translate_compression;

BagCompression compression(std::string mode, std::vector<std::string> codecs = {})
{
  BagCompression c;
  c.mode = std::move(mode);
  c.codecs = std::move(codecs);
  return c;
}

constexpr ResolvedWriteLayout kMcapFile{Format::Mcap, Layout::SingleFile};
constexpr ResolvedWriteLayout kMcapDir{Format::Mcap, Layout::Directory};
constexpr ResolvedWriteLayout kSqliteFile{Format::Sqlite3, Layout::SingleFile};
constexpr ResolvedWriteLayout kSqliteDir{Format::Sqlite3, Layout::Directory};

}  // namespace

// ---------------------------------------------------------------------------
// MCAP outputs take a chunk codec.
// ---------------------------------------------------------------------------

TEST(TranslateCompression, PlainInputStaysPlainOnMcap)
{
  const auto r = translate_compression(compression("none"), kMcapFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "none");
  EXPECT_TRUE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, ZstdChunksCarryOverToMcapExactly)
{
  const auto r = translate_compression(compression("chunk", {"zstd"}), kMcapDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "zstd");
  EXPECT_TRUE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, Lz4ChunksCarryOverToMcapExactly)
{
  const auto r = translate_compression(compression("chunk", {"lz4"}), kMcapFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "lz4");
  EXPECT_TRUE(r->note.empty());
}

TEST(TranslateCompression, MixedChunkCodecsSettleOnZstdAndSaySo)
{
  // describe_bag lists the codecs sorted and unique, so a file mixing the
  // two reads {"lz4", "zstd"}. One writer takes one codec; zstd is bagwiz's
  // own default and the better ratio, and the choice is reported.
  const auto r = translate_compression(compression("chunk", {"lz4", "zstd"}), kMcapFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "zstd");
  EXPECT_FALSE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, Sqlite3MessageModeBecomesZstdChunksOnMcap)
{
  const auto r = translate_compression(compression("message", {"zstd"}), kMcapDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "zstd");
  EXPECT_FALSE(r->note.empty());  // a storage change, worth one INFO line
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, Sqlite3FileModeBecomesZstdChunksOnMcap)
{
  const auto r = translate_compression(compression("file", {"zstd"}), kMcapFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->mcap_compression, "zstd");
  EXPECT_FALSE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, McapOutputLeavesTheSqlite3KnobsAlone)
{
  const auto r = translate_compression(compression("chunk", {"zstd"}), kMcapFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(r->sqlite3_compression_mode.empty());
  EXPECT_TRUE(r->sqlite3_compression_format.empty());
}

// ---------------------------------------------------------------------------
// SQLite3 directory outputs take rosbag2's mode/format pair.
// ---------------------------------------------------------------------------

TEST(TranslateCompression, PlainInputStaysPlainOnSqlite3Directory)
{
  const auto r = translate_compression(compression("none"), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "none");
  EXPECT_EQ(r->sqlite3_compression_format, "none");
  EXPECT_TRUE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, MessageModeCarriesOverToSqlite3DirectoryExactly)
{
  const auto r = translate_compression(compression("message", {"zstd"}), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "message");
  EXPECT_EQ(r->sqlite3_compression_format, "zstd");
  EXPECT_TRUE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, FileModeCarriesOverToSqlite3DirectoryExactly)
{
  const auto r = translate_compression(compression("file", {"zstd"}), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "file");
  EXPECT_EQ(r->sqlite3_compression_format, "zstd");
  EXPECT_TRUE(r->note.empty());
}

TEST(TranslateCompression, ZstdChunksBecomeMessageModeOnSqlite3Directory)
{
  // rosbag2 defines no chunk compression for sqlite3; MESSAGE mode is the
  // one that keeps the shard readable in place, which is also what
  // `bagwiz compress --mode auto` picks for sqlite3.
  const auto r = translate_compression(compression("chunk", {"zstd"}), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "message");
  EXPECT_EQ(r->sqlite3_compression_format, "zstd");
  EXPECT_FALSE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, Lz4ChunksBecomeMessageModeZstdOnSqlite3Directory)
{
  // sqlite3 storage knows only zstd, so lz4 has to change codec on the way.
  const auto r = translate_compression(compression("chunk", {"lz4"}), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "message");
  EXPECT_EQ(r->sqlite3_compression_format, "zstd");
  EXPECT_NE(r->note.find("lz4"), std::string::npos);
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, Sqlite3OutputLeavesTheMcapKnobAlone)
{
  const auto r = translate_compression(compression("chunk", {"zstd"}), kSqliteDir);
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(r->mcap_compression.empty());
}

// ---------------------------------------------------------------------------
// A single-file .db3 cannot carry compression at all.
// ---------------------------------------------------------------------------

TEST(TranslateCompression, PlainInputStaysPlainOnSqlite3SingleFile)
{
  const auto r = translate_compression(compression("none"), kSqliteFile);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->sqlite3_compression_mode, "none");
  EXPECT_EQ(r->sqlite3_compression_format, "none");
  EXPECT_TRUE(r->note.empty());
  EXPECT_FALSE(r->dropped);
}

TEST(TranslateCompression, CompressedInputIsDroppedOnSqlite3SingleFile)
{
  for (const auto & input :
       {compression("chunk", {"zstd"}), compression("chunk", {"lz4"}),
        compression("message", {"zstd"}), compression("file", {"zstd"})}) {
    const auto r = translate_compression(input, kSqliteFile);
    ASSERT_TRUE(r.has_value()) << input.mode;
    EXPECT_EQ(r->sqlite3_compression_mode, "none") << input.mode;
    EXPECT_EQ(r->sqlite3_compression_format, "none") << input.mode;
    EXPECT_TRUE(r->dropped) << input.mode;
    EXPECT_FALSE(r->note.empty()) << input.mode;
  }
}

// ---------------------------------------------------------------------------
// Declarations without a codec, and no declaration at all.
// ---------------------------------------------------------------------------

TEST(TranslateCompression, RosbagModeWithoutACodecMeansZstd)
{
  // rosbag2 defines exactly one compression_format for its modes, so a
  // metadata.yaml that names the mode but not the format still means zstd.
  const auto mcap = translate_compression(compression("message"), kMcapFile);
  ASSERT_TRUE(mcap.has_value());
  EXPECT_EQ(mcap->mcap_compression, "zstd");

  const auto sqlite = translate_compression(compression("file"), kSqliteDir);
  ASSERT_TRUE(sqlite.has_value());
  EXPECT_EQ(sqlite->sqlite3_compression_mode, "file");
  EXPECT_EQ(sqlite->sqlite3_compression_format, "zstd");
}

TEST(TranslateCompression, UnknownCompressionCarriesNothingOver)
{
  // An mcap with no summary section: the codec is not readable without a
  // scan, and describe_bag does not scan. There is nothing to carry over.
  EXPECT_FALSE(translate_compression(compression("unknown"), kMcapFile).has_value());
  EXPECT_FALSE(translate_compression(compression("unknown"), kSqliteDir).has_value());
}
