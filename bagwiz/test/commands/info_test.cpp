// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/info.hpp"

#include "bagwiz/io/bag_io.hpp"
#include "format_units.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>
#include <sqlite3.h>

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

using bagwiz::commands::InfoArgs;
using bagwiz::commands::run_info;
using bagwiz::io::CreateOptions;
using bagwiz::io::Format;
using bagwiz::io::Layout;
using bagwiz::io::TopicInfo;

constexpr std::int64_t kFirstStampNs = 1'000'000'000LL;
constexpr std::int64_t kMiddleStampNs = 2'000'000'000LL;
constexpr std::int64_t kLastStampNs = 3'500'000'000LL;

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

// Two topics of two types; three messages spanning 2.5 s from t=1 s, unless
// `with_messages` is false.
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

struct Output
{
  int exit_code = 0;
  std::string text;
  // `Key: value` lines, with the value's surrounding spaces stripped. Only
  // the summary block's lines have a key; `-l` rows are left in `text`.
  std::map<std::string, std::string> fields;
};

Output run_captured(const InfoArgs & args)
{
  Output out;
  ::testing::internal::CaptureStdout();
  out.exit_code = run_info(args);
  out.text = ::testing::internal::GetCapturedStdout();

  std::size_t pos = 0;
  while (pos < out.text.size()) {
    const auto nl = out.text.find('\n', pos);
    const auto end = nl == std::string::npos ? out.text.size() : nl;
    const auto line = out.text.substr(pos, end - pos);
    pos = end + 1;
    const auto colon = line.find(':');
    if (colon == std::string::npos || colon == 0 || line.find(' ') < colon) {
      continue;
    }
    auto value = line.substr(colon + 1);
    value.erase(0, value.find_first_not_of(' '));
    value.erase(value.find_last_not_of(' ') + 1);
    out.fields[line.substr(0, colon)] = value;
  }
  return out;
}

class InfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_info_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(InfoTest, DirectoryMcapPrintsEveryField)
{
  const auto bag = tmp_dir_ / "drive";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"));

  const auto out = run_captured(InfoArgs{.input_path = bag});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  // Keys are padded to one column so the values line up.
  EXPECT_EQ(out.text.substr(0, out.text.find('\n')), "Path:         " + bag.string());
  EXPECT_EQ(out.fields.at("Layout"), "directory");
  EXPECT_EQ(out.fields.at("Storage"), "mcap");
  EXPECT_EQ(out.fields.at("Compression").rfind("zstd chunks (", 0), 0u) << out.text;
  EXPECT_EQ(out.fields.at("Compression").back(), ')');
  EXPECT_EQ(out.fields.at("Files"), "1");
  EXPECT_FALSE(out.fields.at("Size").empty());
  EXPECT_EQ(out.fields.at("Metadata"), "metadata.yaml version 5");
  EXPECT_EQ(out.fields.at("Topics"), "2 (2 types)");
  EXPECT_EQ(out.fields.at("Messages"), "3");
  EXPECT_EQ(out.fields.at("Start"), "1970-01-01 00:00:01.000000000 UTC (1.000000000)");
  EXPECT_EQ(out.fields.at("End"), "1970-01-01 00:00:03.500000000 UTC (3.500000000)");
  EXPECT_EQ(out.fields.at("Duration"), "2.500s");
}

TEST_F(InfoTest, SingleFileSqlite3HasNoMetadataYaml)
{
  const auto bag = tmp_dir_ / "single.db3";
  write_fixture(bag, sqlite3_options(Layout::SingleFile));

  const auto out = run_captured(InfoArgs{.input_path = bag});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Layout"), "single-file");
  EXPECT_EQ(out.fields.at("Storage"), "sqlite3");
  EXPECT_EQ(out.fields.at("Compression"), "none");
  EXPECT_EQ(out.fields.at("Metadata"), "none (single file)");
  EXPECT_EQ(out.fields.at("Messages"), "3");
}

TEST_F(InfoTest, CompressedSqlite3DirectoryNamesTheRosbag2Mode)
{
  const auto message_dir = tmp_dir_ / "message_dir";
  write_fixture(message_dir, sqlite3_options(Layout::Directory, "message", "zstd"));
  const auto file_dir = tmp_dir_ / "file_dir";
  write_fixture(file_dir, sqlite3_options(Layout::Directory, "file", "zstd"));

  const auto message_out = run_captured(InfoArgs{.input_path = message_dir});
  const auto file_out = run_captured(InfoArgs{.input_path = file_dir});

  ASSERT_EQ(message_out.exit_code, 0);
  ASSERT_EQ(file_out.exit_code, 0);
  EXPECT_EQ(message_out.fields.at("Compression"), "zstd per-message (rosbag2 MESSAGE mode)");
  EXPECT_EQ(file_out.fields.at("Compression"), "zstd whole-file envelope (rosbag2 FILE mode)");
}

// A humble-era .db3 has no `metadata` row: the count would need a table scan,
// so it reads unknown while the extent still comes from the timestamp index.
TEST_F(InfoTest, MissingSummaryReadsUnknownInsteadOfScanning)
{
  const auto bag = tmp_dir_ / "humble.db3";
  write_fixture(bag, sqlite3_options(Layout::SingleFile));
  {
    sqlite3 * db = nullptr;
    ASSERT_EQ(sqlite3_open(bag.c_str(), &db), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db, "DROP TABLE metadata", nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(db);
  }

  const auto out = run_captured(InfoArgs{.input_path = bag});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Topics"), "2 (2 types)");
  EXPECT_EQ(out.fields.at("Messages"), "unknown (no summary in the bag)");
  EXPECT_EQ(out.fields.at("Start"), "1970-01-01 00:00:01.000000000 UTC (1.000000000)");
  EXPECT_EQ(out.fields.at("Duration"), "2.500s");
}

TEST_F(InfoTest, EnvelopeAloneReadsTopicsUnknown)
{
  const auto dir = tmp_dir_ / "file_dir";
  write_fixture(dir, sqlite3_options(Layout::Directory, "file", "zstd"));

  const auto out = run_captured(InfoArgs{.input_path = dir / "file_dir_0.db3.zstd"});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Layout"), "single-file");
  EXPECT_EQ(out.fields.at("Compression"), "zstd whole-file envelope (rosbag2 FILE mode)");
  EXPECT_EQ(out.fields.at("Topics"), "unknown");
  EXPECT_EQ(out.fields.at("Messages"), "unknown (no summary in the bag)");
  EXPECT_EQ(out.fields.at("Start"), "unknown");
}

TEST_F(InfoTest, EmptyBagPrintsDashesForTheExtent)
{
  const auto bag = tmp_dir_ / "empty";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"), /*with_messages=*/false);

  const auto out = run_captured(InfoArgs{.input_path = bag});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Messages"), "0");
  EXPECT_EQ(out.fields.at("Start"), "-");
  EXPECT_EQ(out.fields.at("End"), "-");
  EXPECT_EQ(out.fields.at("Duration"), "-");
}

TEST_F(InfoTest, MissingMetadataYamlIsReported)
{
  const auto bag = tmp_dir_ / "no_yaml";
  write_fixture(bag, mcap_options(Layout::Directory, "none"));
  ASSERT_TRUE(std::filesystem::remove(bag / "metadata.yaml"));

  const auto out = run_captured(InfoArgs{.input_path = bag});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Metadata"), "metadata.yaml missing (reconstructed from the directory)");
  EXPECT_EQ(out.fields.at("Compression"), "none");
  EXPECT_EQ(out.fields.at("Messages"), "3");
}

TEST_F(InfoTest, BytesFlagPrintsRawSize)
{
  const auto bag = tmp_dir_ / "plain.mcap";
  write_fixture(bag, mcap_options(Layout::SingleFile, "none"));

  const auto out = run_captured(InfoArgs{.input_path = bag, .bytes = true});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_EQ(out.fields.at("Size"), std::to_string(std::filesystem::file_size(bag)));
}

TEST_F(InfoTest, LongListingAppendsOneRowPerFile)
{
  const auto bag = tmp_dir_ / "drive";
  write_fixture(bag, mcap_options(Layout::Directory, "zstd"));

  const auto out = run_captured(InfoArgs{.input_path = bag, .long_listing = true});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  EXPECT_NE(out.text.find("SIZE"), std::string::npos) << out.text;
  EXPECT_NE(out.text.find("MESSAGES"), std::string::npos) << out.text;
  const auto row = out.text.find("drive_0.mcap");
  ASSERT_NE(row, std::string::npos) << out.text;
  // The row carries the shard's own count, start and duration.
  const auto line_start = out.text.rfind('\n', row) + 1;
  const auto line = out.text.substr(line_start, out.text.find('\n', row) - line_start);
  EXPECT_NE(line.find(" 3 "), std::string::npos) << line;
  EXPECT_NE(line.find("1970-01-01 00:00:01.000000000 UTC (1.000000000)"), std::string::npos)
    << line;
  EXPECT_NE(line.find("2.500s"), std::string::npos) << line;
}

// A single file is the whole bag, so its `-l` row carries the bag's summary
// rather than dashes.
TEST_F(InfoTest, LongListingRowOfSingleFileCarriesTheSummary)
{
  const auto bag = tmp_dir_ / "plain.mcap";
  write_fixture(bag, mcap_options(Layout::SingleFile, "none"));

  const auto out = run_captured(InfoArgs{.input_path = bag, .long_listing = true});

  ASSERT_EQ(out.exit_code, 0) << out.text;
  // The row is the line after the table header (the `Path:` line ends with
  // the same file name, so it cannot anchor the search).
  const auto header = out.text.find("PATH\n");
  ASSERT_NE(header, std::string::npos) << out.text;
  const auto line_start = header + std::string("PATH\n").size();
  const auto line = out.text.substr(line_start, out.text.find('\n', line_start) - line_start);
  EXPECT_NE(line.find("plain.mcap"), std::string::npos) << line;
  EXPECT_NE(line.find(" 3 "), std::string::npos) << line;
  EXPECT_NE(line.find("1970-01-01 00:00:01.000000000 UTC (1.000000000)"), std::string::npos)
    << line;
  EXPECT_NE(line.find("2.500s"), std::string::npos) << line;
}

TEST_F(InfoTest, MissingInputFails)
{
  const auto out = run_captured(InfoArgs{.input_path = tmp_dir_ / "missing"});

  EXPECT_EQ(out.exit_code, 1);
  EXPECT_TRUE(out.text.empty());
}

TEST(FormatUnitsTest, FormatDurationRendersSecondsMinutesAndHours)
{
  using bagwiz::commands::format_duration;
  EXPECT_EQ(format_duration(0), "0.000s");
  EXPECT_EQ(format_duration(2'500'000'000LL), "2.500s");
  EXPECT_EQ(format_duration(62'500'000'000LL), "1m 02.500s");
  EXPECT_EQ(format_duration(3'723'456'000'000LL), "1h 02m 03.456s");
  EXPECT_EQ(format_duration(-1), "0.000s");
}

}  // namespace
