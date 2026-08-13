// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TOPIC_EXPAND_TEST_UTIL_HPP_
#define COMMANDS__TOPIC_EXPAND_TEST_UTIL_HPP_

// Shared bag-building helpers and fixture for expand_topic_selectors() tests
// (topic_expand.cpp), used by both topic_expand_test.cpp (independent-slot
// behavior: glob/literal/require_present/reject/dedupe/...) and
// topic_expand_pair_scope_test.cpp (cross-slot behavior: pair_value and
// scope). Anonymous-namespace helpers give each including .cpp its own
// private copy — same as when this all lived in one file — which keeps this
// header safe to include from more than one ament_add_gtest target without
// an ODR hazard; ExpandTopicSelectorsTest itself is a class definition, which
// the standard exempts from ODR as long as every definition is
// byte-identical, true here since both TUs get it from this same header.

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

constexpr std::array<std::uint8_t, 4> kPayload{0x01, 0x02, 0x03, 0x04};

std::span<const std::byte> payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size());
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

// Fresh scratch directory for a standalone TEST() that needs one bag and does
// not use ExpandTopicSelectorsTest's fixture (which owns a shared tmp_dir_
// removed in TearDown). The caller owns cleanup.
std::filesystem::path scratch_dir(const std::string & label)
{
  const auto dir = std::filesystem::temp_directory_path() /
                   ("bagwiz_topic_expand_" + label + "_" +
                    std::to_string(::testing::UnitTest::GetInstance()->random_seed()));
  std::filesystem::create_directories(dir);
  return dir;
}

std::filesystem::path write_bag(
  const std::filesystem::path & dir, const std::vector<bagwiz::io::TopicInfo> & topics)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";

  auto writer = bagwiz::io::open_write(dir, opts);
  for (const auto & t : topics) {
    writer->declare_topic(t);
  }
  std::int64_t stamp = 1000;
  for (const auto & t : topics) {
    writer->write(t.name, stamp++, payload_view());
  }
  writer->close();
  return dir;
}

// Bag with two PointCloud2 topics (declared right-before-left so the test can
// prove the result is sorted, not declaration-ordered) and one Image topic.
std::filesystem::path make_bag(const std::filesystem::path & dir)
{
  return write_bag(
    dir, {
           make_topic("/lidar/right/points", "sensor_msgs/msg/PointCloud2"),
           make_topic("/lidar/left/points", "sensor_msgs/msg/PointCloud2"),
           make_topic("/camera/image_raw", "sensor_msgs/msg/Image"),
         });
}

// Bag with three PointCloud2 topics named /a, /a1, /a2 (declared out of
// sorted order, same reason as make_bag() above): a literal '/a' matches only
// itself, a glob '/a*' matches all three sorted. Mirrors the dedupe-rule
// example in topic_expand.cpp's dedupe().
std::filesystem::path make_dedupe_bag(const std::filesystem::path & dir)
{
  return write_bag(
    dir, {
           make_topic("/a2", "sensor_msgs/msg/PointCloud2"),
           make_topic("/a", "sensor_msgs/msg/PointCloud2"),
           make_topic("/a1", "sensor_msgs/msg/PointCloud2"),
         });
}

}  // namespace

class ExpandTopicSelectorsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_topic_expand_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
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

#endif  // COMMANDS__TOPIC_EXPAND_TEST_UTIL_HPP_
