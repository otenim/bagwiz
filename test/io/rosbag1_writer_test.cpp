// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/rosbag1_writer.hpp"

#include "bagwiz/io/rosbag1_reader.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

std::filesystem::path tmp_bag_path(const std::string & name)
{
  const auto tmp = std::filesystem::temp_directory_path() / "bagwiz_writer_test";
  std::filesystem::create_directories(tmp);
  return tmp / name;
}

std::vector<std::byte> as_bytes(std::initializer_list<std::uint8_t> vs)
{
  std::vector<std::byte> out;
  out.reserve(vs.size());
  for (auto v : vs) {
    out.push_back(static_cast<std::byte>(v));
  }
  return out;
}

bool spans_equal(std::span<const std::byte> a, std::span<const std::byte> b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

TEST(Rosbag1Writer, EmptyBagIsReadable)
{
  const auto path = tmp_bag_path("empty.bag");
  std::filesystem::remove(path);

  {
    bagwiz::io::Rosbag1Writer w(path);
    w.close();
  }

  bagwiz::io::Rosbag1Reader r(path);
  bagwiz::io::Ros1Message msg;
  EXPECT_FALSE(r.next(msg));
  EXPECT_TRUE(r.connections().empty());
}

TEST(Rosbag1Writer, SingleTopicSingleMessage)
{
  const auto path = tmp_bag_path("single.bag");
  std::filesystem::remove(path);

  const auto payload = as_bytes({0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE});

  uint32_t cid;
  {
    bagwiz::io::Rosbag1Writer w(path);
    cid = w.declare_connection(
      "/chatter", "std_msgs/String", "992ce8a1687cec8c8bd883ec73ca41d1", "string data\n");
    w.write(cid, 1'700'000'000'000'000'000LL, payload);
    w.close();
  }

  bagwiz::io::Rosbag1Reader r(path);
  bagwiz::io::Ros1Message msg;
  ASSERT_TRUE(r.next(msg));
  EXPECT_EQ(msg.timestamp_ns, 1'700'000'000'000'000'000LL);
  EXPECT_TRUE(spans_equal(msg.payload, payload));
  EXPECT_FALSE(r.next(msg));

  ASSERT_EQ(r.connections().size(), 1u);
  EXPECT_EQ(r.connections()[0].topic, "/chatter");
  EXPECT_EQ(r.connections()[0].type, "std_msgs/String");
  EXPECT_EQ(r.connections()[0].md5sum, "992ce8a1687cec8c8bd883ec73ca41d1");
  EXPECT_EQ(r.connections()[0].message_definition, "string data\n");
}

TEST(Rosbag1Writer, MultipleTopicsRoundTrip)
{
  const auto path = tmp_bag_path("multi.bag");
  std::filesystem::remove(path);

  const auto p_a = as_bytes({0x01, 0x02, 0x03, 0x04});
  const auto p_b = as_bytes({0xAA, 0xBB});

  {
    bagwiz::io::Rosbag1Writer w(path);
    const auto a = w.declare_connection("/a", "std_msgs/Int32", "md5_a", "int32 data\n");
    const auto b = w.declare_connection("/b", "std_msgs/UInt8", "md5_b", "uint8 data\n");

    w.write(a, 1'000'000'000LL, p_a);
    w.write(b, 1'500'000'000LL, p_b);
    w.write(a, 2'000'000'000LL, p_a);
    w.close();
  }

  bagwiz::io::Rosbag1Reader r(path);

  // The reader registers connections incrementally as chunks are
  // walked, so collect (conn_id, time) tuples first and resolve topic
  // names from the final connections() snapshot at EOF.
  std::vector<std::pair<uint32_t, int64_t>> raw_seen;
  bagwiz::io::Ros1Message msg;
  while (r.next(msg)) {
    raw_seen.emplace_back(msg.conn_id, msg.timestamp_ns);
  }
  std::unordered_map<uint32_t, std::string> conn_topic;
  for (const auto & c : r.connections()) {
    conn_topic[c.conn_id] = c.topic;
  }
  std::vector<std::pair<std::string, int64_t>> seen;
  for (const auto & [cid, t] : raw_seen) {
    seen.emplace_back(conn_topic.at(cid), t);
  }
  EXPECT_EQ(seen.size(), 3u);
  EXPECT_EQ(seen[0].first, "/a");
  EXPECT_EQ(seen[0].second, 1'000'000'000LL);
  EXPECT_EQ(seen[1].first, "/b");
  EXPECT_EQ(seen[1].second, 1'500'000'000LL);
  EXPECT_EQ(seen[2].first, "/a");
  EXPECT_EQ(seen[2].second, 2'000'000'000LL);
}

TEST(Rosbag1Writer, ChunkSplitOnLargePayloads)
{
  // The writer flushes when chunk_data exceeds 4 MiB. Push enough to
  // cross that boundary several times and verify all messages survive
  // the round-trip.
  const auto path = tmp_bag_path("chunked.bag");
  std::filesystem::remove(path);

  const std::size_t kPayloadBytes = 512 * 1024;  // 512 KiB
  std::vector<std::byte> payload(kPayloadBytes);
  for (std::size_t i = 0; i < kPayloadBytes; ++i) {
    payload[i] = static_cast<std::byte>(i & 0xFF);
  }

  constexpr int kCount = 20;  // ~10 MiB total -> at least 2 chunk flushes

  {
    bagwiz::io::Rosbag1Writer w(path);
    const auto cid =
      w.declare_connection("/blob", "std_msgs/UInt8MultiArray", "md5_blob", "uint8[] data\n");
    for (int i = 0; i < kCount; ++i) {
      w.write(cid, static_cast<int64_t>(i + 1) * 1'000'000'000LL, payload);
    }
    w.close();
  }

  bagwiz::io::Rosbag1Reader r(path);
  int seen = 0;
  bagwiz::io::Ros1Message msg;
  while (r.next(msg)) {
    EXPECT_TRUE(spans_equal(msg.payload, payload));
    EXPECT_EQ(msg.timestamp_ns, static_cast<int64_t>(seen + 1) * 1'000'000'000LL);
    ++seen;
  }
  EXPECT_EQ(seen, kCount);
}

TEST(Rosbag1Writer, IdempotentDeclareConnection)
{
  // Same (topic, type, md5, def) returns the same conn_id on repeat.
  const auto path = tmp_bag_path("idempotent.bag");
  std::filesystem::remove(path);

  bagwiz::io::Rosbag1Writer w(path);
  const auto a = w.declare_connection("/x", "T", "m", "d");
  const auto b = w.declare_connection("/x", "T", "m", "d");
  EXPECT_EQ(a, b);

  // Different definition -> new conn_id.
  const auto c = w.declare_connection("/x", "T", "m", "d2");
  EXPECT_NE(a, c);

  w.close();
}
