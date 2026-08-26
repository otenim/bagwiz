// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/du.hpp"

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::span<const std::byte> payload_view(const std::vector<std::byte> & bytes)
{
  return std::span<const std::byte>(bytes.data(), bytes.size());
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// Build an MCAP directory bag with four topics at mixed name depths:
//   /sensing/lidar/points  (2 messages x 1024 bytes = 2048)
//   /sensing/camera/image  (1 message  x 1536 bytes = 1536)
//   /perception/objects    (1 message  x    4 bytes =    4)
//   /silent                (declared, no messages   =    0)
std::filesystem::path build_input(const std::filesystem::path & dir)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic("/sensing/lidar/points", "sensor_msgs/msg/PointCloud2"));
  writer->declare_topic(make_topic("/sensing/camera/image", "sensor_msgs/msg/Image"));
  writer->declare_topic(make_topic("/perception/objects", "std_msgs/msg/String"));
  writer->declare_topic(make_topic("/silent", "std_msgs/msg/String"));

  const std::vector<std::byte> lidar_payload(1024, std::byte{0x01});
  const std::vector<std::byte> camera_payload(1536, std::byte{0x02});
  const std::vector<std::byte> objects_payload(4, std::byte{0x03});

  writer->write("/sensing/lidar/points", 1'000'000'000LL, payload_view(lidar_payload));
  writer->write("/sensing/camera/image", 2'000'000'000LL, payload_view(camera_payload));
  writer->write("/perception/objects", 3'000'000'000LL, payload_view(objects_payload));
  writer->write("/sensing/lidar/points", 4'000'000'000LL, payload_view(lidar_payload));
  writer->close();
  return path;
}

// Run run_du capturing stdout, and return the output with trailing spaces
// stripped from each line (column padding is not what these tests assert).
std::string run_captured(const bagwiz::commands::DuArgs & args, int & exit_code)
{
  ::testing::internal::CaptureStdout();
  exit_code = bagwiz::commands::run_du(args);
  std::string out = ::testing::internal::GetCapturedStdout();

  std::string stripped;
  std::size_t pos = 0;
  while (pos < out.size()) {
    const auto nl = out.find('\n', pos);
    const auto end = nl == std::string::npos ? out.size() : nl;
    auto line = out.substr(pos, end - pos);
    line.erase(line.find_last_not_of(' ') + 1);
    stripped += line;
    stripped += '\n';
    pos = end + 1;
  }
  return stripped;
}

class DuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_du_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
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

TEST_F(DuTest, ReportsEveryTopicSortedBySizeWithTotal)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  // Byte totals are integer sums over fixed synthetic payloads, so exact
  // equality is the per-element/integer-addition shape the numerical
  // reproducibility rules allow to be asserted exactly.
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "2.0K  57.1% /sensing/lidar/points\n"
    "1.5K  42.8% /sensing/camera/image\n"
    "   4   0.1% /perception/objects\n"
    "   0   0.0% /silent\n"
    "3.5K 100.0% total\n");
}

TEST_F(DuTest, RawByteSizes)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.bytes = true;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "2048  57.1% /sensing/lidar/points\n"
    "1536  42.8% /sensing/camera/image\n"
    "   4   0.1% /perception/objects\n"
    "   0   0.0% /silent\n"
    "3588 100.0% total\n");
}

TEST_F(DuTest, DepthOneGroupsByFirstNameComponent)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.depth = 1;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  // /sensing aggregates lidar + camera (3584 bytes); /perception and /silent
  // are leaves at the grouping depth and keep their names.
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "3.5K  99.9% /sensing\n"
    "   4   0.1% /perception\n"
    "   0   0.0% /silent\n"
    "3.5K 100.0% total\n");
}

TEST_F(DuTest, DepthTwoGroupsBySecondNameComponent)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.bytes = true;
  args.depth = 2;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  // /perception/objects sits exactly at depth 2 and /silent above it, so
  // both keep their full names.
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "2048  57.1% /sensing/lidar\n"
    "1536  42.8% /sensing/camera\n"
    "   4   0.1% /perception/objects\n"
    "   0   0.0% /silent\n"
    "3588 100.0% total\n");
}

TEST_F(DuTest, DepthZeroPrintsOnlyTotal)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.depth = 0;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "3.5K 100.0% total\n");
}

TEST_F(DuTest, DepthAggregatesFilteredTopics)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/lidar/points", "/sensing/camera/image"};
  args.depth = 1;

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  // The total covers only the selected topics.
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "3.5K 100.0% /sensing\n"
    "3.5K 100.0% total\n");
}

TEST_F(DuTest, TopicFilterNarrowsRowsAndTotal)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.topics = {"/sensing/camera/image"};

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE      % TOPIC\n"
    "1.5K 100.0% /sensing/camera/image\n"
    "1.5K 100.0% total\n");
}

TEST_F(DuTest, TopicFilterKeepsZeroMessageTopic)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.topics = {"/silent"};

  int exit_code = -1;
  const auto out = run_captured(args, exit_code);

  // Nothing was reported, so there is no total to take a share of: every
  // percentage reads 0.0%, the `total` row included.
  EXPECT_EQ(exit_code, 0);
  EXPECT_EQ(
    out,
    "SIZE    % TOPIC\n"
    "   0 0.0% /silent\n"
    "   0 0.0% total\n");
}

TEST_F(DuTest, UnknownTopicSelectorFails)
{
  const auto in_path = build_input(tmp_dir_);

  bagwiz::commands::DuArgs args;
  args.input_path = in_path;
  args.topics = {"/no/such/topic"};

  int exit_code = -1;
  ::testing::internal::CaptureStdout();
  exit_code = bagwiz::commands::run_du(args);
  ::testing::internal::GetCapturedStdout();

  EXPECT_EQ(exit_code, 1);
}

TEST_F(DuTest, MissingInputFails)
{
  bagwiz::commands::DuArgs args;
  args.input_path = tmp_dir_ / "does_not_exist";

  int exit_code = -1;
  ::testing::internal::CaptureStdout();
  exit_code = bagwiz::commands::run_du(args);
  ::testing::internal::GetCapturedStdout();

  EXPECT_EQ(exit_code, 1);
}

}  // namespace
