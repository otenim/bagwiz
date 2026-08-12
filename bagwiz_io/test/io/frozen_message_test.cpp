// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

// BagReader::freeze() upgrades the most recently returned RawMessage into an
// owning FrozenMessage whose payload stays valid across later next() calls
// and reader destruction. On the parallel indexed mcap path the upgrade must
// be zero-copy (the frozen span aliases the retained chunk buffer); on every
// other path it may copy, but the survival contract must hold identically.
namespace
{

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

constexpr int kMessageCount = 40;

std::vector<std::byte> expected_payload(int i)
{
  return payload_bytes(i, 64);
}

// Zstd-compressed with tiny chunks so nearly every message lands in its own
// chunk: freeze() must keep whole chunk buffers alive across eviction.
void write_mcap_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::SingleFile;
  options.mcap_compression = "zstd";
  options.mcap_chunk_size = 64;

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/a"));
  for (int i = 0; i < kMessageCount; ++i) {
    const auto p = expected_payload(i);
    writer->write("/a", 1000 + i * 10, std::span<const std::byte>(p.data(), p.size()));
  }
  writer->close();
}

void write_sqlite_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Sqlite3;
  options.layout = bagwiz::io::Layout::SingleFile;

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/a"));
  for (int i = 0; i < kMessageCount; ++i) {
    const auto p = expected_payload(i);
    writer->write("/a", 1000 + i * 10, std::span<const std::byte>(p.data(), p.size()));
  }
  writer->close();
}

// Directory-layout mcap bag split into several shards, so freeze() must work
// through the shard multiplexer.
void write_sharded_fixture(const std::filesystem::path & path)
{
  bagwiz::io::CreateOptions options;
  options.format = bagwiz::io::Format::Mcap;
  options.layout = bagwiz::io::Layout::Directory;
  options.mcap_compression = "zstd";
  options.mcap_chunk_size = 64;
  options.split_bytes = 512;  // force a handful of shards

  auto writer = bagwiz::io::open_write(path, options);
  writer->declare_topic(topic_info("/a"));
  for (int i = 0; i < kMessageCount; ++i) {
    const auto p = expected_payload(i);
    writer->write("/a", 1000 + i * 10, std::span<const std::byte>(p.data(), p.size()));
  }
  writer->close();
}

class FrozenMessageTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_ = std::filesystem::temp_directory_path() /
           ("bagwiz_frozen_" +
            std::to_string(::testing::UnitTest::GetInstance()->current_test_info()->line()));
    std::filesystem::remove_all(tmp_);
    std::filesystem::create_directories(tmp_);
  }
  void TearDown() override
  {
    ::unsetenv("BAGWIZ_READ_THREADS");
    std::filesystem::remove_all(tmp_);
  }

  std::filesystem::path tmp_;
};

// Freeze every message, then verify all payloads only after the iteration
// completed: each frozen payload must have survived every later next() call
// (and, on the parallel path, its chunk buffer's eviction and recycling).
void freeze_all_and_verify(const std::filesystem::path & bag, const char * read_threads)
{
  ::setenv("BAGWIZ_READ_THREADS", read_threads, 1);
  auto reader = bagwiz::io::open_read(bag);
  std::vector<bagwiz::io::FrozenMessage> frozen;
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    frozen.push_back(reader->freeze(raw));
  }
  ASSERT_EQ(frozen.size(), static_cast<std::size_t>(kMessageCount));
  for (int i = 0; i < kMessageCount; ++i) {
    const auto expected = expected_payload(i);
    const auto & f = frozen[static_cast<std::size_t>(i)];
    EXPECT_EQ(f.topic->name, "/a");
    EXPECT_EQ(f.timestamp_ns, 1000 + i * 10);
    ASSERT_EQ(f.payload.size(), expected.size());
    EXPECT_TRUE(std::equal(f.payload.begin(), f.payload.end(), expected.begin()));
  }
}

TEST_F(FrozenMessageTest, SerialMcapFreezeSurvivesAdvance)
{
  const auto bag = tmp_ / "serial.mcap";
  write_mcap_fixture(bag);
  freeze_all_and_verify(bag, "0");
}

TEST_F(FrozenMessageTest, ParallelMcapFreezeSurvivesAdvance)
{
  const auto bag = tmp_ / "parallel.mcap";
  write_mcap_fixture(bag);
  freeze_all_and_verify(bag, "4");
}

TEST_F(FrozenMessageTest, ParallelMcapFreezeIsZeroCopy)
{
  ::setenv("BAGWIZ_READ_THREADS", "4", 1);
  const auto bag = tmp_ / "zerocopy.mcap";
  write_mcap_fixture(bag);
  auto reader = bagwiz::io::open_read(bag);
  bagwiz::io::RawMessage raw;
  int seen = 0;
  while (reader->next(raw)) {
    const bagwiz::io::FrozenMessage frozen = reader->freeze(raw);
    EXPECT_EQ(frozen.payload.data(), raw.payload.data())
      << "freeze() on the parallel indexed path must alias the chunk buffer, not copy";
    EXPECT_NE(frozen.owner, nullptr);
    ++seen;
  }
  EXPECT_EQ(seen, kMessageCount);
}

TEST_F(FrozenMessageTest, FrozenPayloadSurvivesReaderDestruction)
{
  ::setenv("BAGWIZ_READ_THREADS", "4", 1);
  const auto bag = tmp_ / "outlive.mcap";
  write_mcap_fixture(bag);
  bagwiz::io::FrozenMessage frozen;
  {
    auto reader = bagwiz::io::open_read(bag);
    bagwiz::io::RawMessage raw;
    ASSERT_TRUE(reader->next(raw));
    frozen = reader->freeze(raw);
  }  // reader (and its chunk buffer pool) destroyed here
  const auto expected = expected_payload(0);
  ASSERT_EQ(frozen.payload.size(), expected.size());
  EXPECT_TRUE(std::equal(frozen.payload.begin(), frozen.payload.end(), expected.begin()));
}

TEST_F(FrozenMessageTest, ShardedMcapFreezeIsZeroCopy)
{
  ::setenv("BAGWIZ_READ_THREADS", "4", 1);
  const auto bag = tmp_ / "sharded";
  write_sharded_fixture(bag);
  auto reader = bagwiz::io::open_read(bag);
  std::vector<bagwiz::io::FrozenMessage> frozen;
  bagwiz::io::RawMessage raw;
  while (reader->next(raw)) {
    const bagwiz::io::FrozenMessage f = reader->freeze(raw);
    EXPECT_EQ(f.payload.data(), raw.payload.data())
      << "freeze() through the shard multiplexer must stay zero-copy";
    frozen.push_back(f);
  }
  ASSERT_EQ(frozen.size(), static_cast<std::size_t>(kMessageCount));
  for (int i = 0; i < kMessageCount; ++i) {
    const auto expected = expected_payload(i);
    const auto & f = frozen[static_cast<std::size_t>(i)];
    EXPECT_EQ(f.topic->name, "/a");
    ASSERT_EQ(f.payload.size(), expected.size());
    EXPECT_TRUE(std::equal(f.payload.begin(), f.payload.end(), expected.begin()));
  }
}

TEST_F(FrozenMessageTest, SqliteFreezeSurvivesAdvance)
{
  const auto bag = tmp_ / "serial.db3";
  write_sqlite_fixture(bag);
  freeze_all_and_verify(bag, "0");
  freeze_all_and_verify(bag, "4");
}

}  // namespace
