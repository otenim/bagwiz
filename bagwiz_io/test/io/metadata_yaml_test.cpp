// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/metadata_yaml.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace
{

constexpr std::int64_t kFirstShardStartNs = 1'000'000'000LL;
constexpr std::int64_t kFirstShardDurationNs = 1'000'000'000LL;
constexpr std::int64_t kSecondShardStartNs = 3'500'000'000LL;

class MetadataYamlTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_metadata_yaml_test_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path write_yaml(const std::string & body)
  {
    const auto path = tmp_dir_ / "metadata.yaml";
    std::ofstream out(path);
    out << body;
    return path;
  }

  std::filesystem::path tmp_dir_;
};

// The fields rosbag2 added after the base schema: `version`, `ros_distro`,
// and the per-file summaries under `files:`.
TEST_F(MetadataYamlTest, ParsesVersionRosDistroAndPerFileSummaries)
{
  const auto path = write_yaml(
    "rosbag2_bagfile_information:\n"
    "  version: 8\n"
    "  storage_identifier: mcap\n"
    "  ros_distro: jazzy\n"
    "  duration:\n"
    "    nanoseconds: 2500000000\n"
    "  starting_time:\n"
    "    nanoseconds_since_epoch: 1000000000\n"
    "  message_count: 3\n"
    "  topics_with_message_count: []\n"
    "  compression_format: \"\"\n"
    "  compression_mode: \"\"\n"
    "  relative_file_paths:\n"
    "    - bag_0.mcap\n"
    "    - bag_1.mcap\n"
    "  files:\n"
    "    - path: bag_0.mcap\n"
    "      starting_time:\n"
    "        nanoseconds_since_epoch: 1000000000\n"
    "      duration:\n"
    "        nanoseconds: 1000000000\n"
    "      message_count: 2\n"
    "    - path: bag_1.mcap\n"
    "      starting_time:\n"
    "        nanoseconds_since_epoch: 3500000000\n"
    "      duration:\n"
    "        nanoseconds: 0\n"
    "      message_count: 1\n");

  const auto md = bagwiz::io::load_metadata_yaml(path);

  EXPECT_EQ(md.version, 8);
  EXPECT_EQ(md.ros_distro, "jazzy");
  ASSERT_EQ(md.relative_file_paths.size(), 2u);
  ASSERT_EQ(md.files.size(), 2u);
  EXPECT_EQ(md.files[0].path, std::filesystem::path("bag_0.mcap"));
  EXPECT_TRUE(md.files[0].has_summary);
  EXPECT_EQ(md.files[0].start_ns, kFirstShardStartNs);
  EXPECT_EQ(md.files[0].end_ns, kFirstShardStartNs + kFirstShardDurationNs);
  EXPECT_EQ(md.files[0].message_count, 2);
  EXPECT_EQ(md.files[1].path, std::filesystem::path("bag_1.mcap"));
  EXPECT_TRUE(md.files[1].has_summary);
  EXPECT_EQ(md.files[1].start_ns, kSecondShardStartNs);
  EXPECT_EQ(md.files[1].end_ns, kSecondShardStartNs);
  EXPECT_EQ(md.files[1].message_count, 1);
}

TEST_F(MetadataYamlTest, LeavesOptionalFieldsUnsetWhenAbsent)
{
  const auto path = write_yaml(
    "rosbag2_bagfile_information:\n"
    "  storage_identifier: sqlite3\n"
    "  relative_file_paths:\n"
    "    - bag_0.db3\n");

  const auto md = bagwiz::io::load_metadata_yaml(path);

  EXPECT_FALSE(md.version.has_value());
  EXPECT_TRUE(md.ros_distro.empty());
  EXPECT_TRUE(md.files.empty());
  EXPECT_FALSE(md.has_summary);
}

TEST_F(MetadataYamlTest, FileEntryWithoutTimingKeepsPathOnly)
{
  const auto path = write_yaml(
    "rosbag2_bagfile_information:\n"
    "  version: 4\n"
    "  storage_identifier: sqlite3\n"
    "  files:\n"
    "    - path: bag_0.db3\n");

  const auto md = bagwiz::io::load_metadata_yaml(path);

  ASSERT_EQ(md.files.size(), 1u);
  EXPECT_EQ(md.files[0].path, std::filesystem::path("bag_0.db3"));
  EXPECT_FALSE(md.files[0].has_summary);
}

}  // namespace
