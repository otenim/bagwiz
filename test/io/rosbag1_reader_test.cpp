// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/rosbag1_reader.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

// A tiny ROS 1 bag v2.0 builder that hand-crafts the on-disk layout.
// We only need the few records the production reader actually consumes:
// magic + BAG_HEADER + one CHUNK containing one CONNECTION and N
// MESSAGE_DATA records.
class Bag1Builder
{
public:
  // Append a u32 to `dst` in little-endian form.
  static void put_u32(std::vector<std::byte> & dst, std::uint32_t v)
  {
    for (int i = 0; i < 4; ++i) {
      dst.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFF));
    }
  }

  static void put_u8(std::vector<std::byte> & dst, std::uint8_t v)
  {
    dst.push_back(static_cast<std::byte>(v));
  }

  static void put_str(std::vector<std::byte> & dst, const std::string & s)
  {
    for (char c : s) {
      dst.push_back(static_cast<std::byte>(c));
    }
  }

  // Encode one "name=value" header field with its u32 length prefix.
  static void put_field_str(
    std::vector<std::byte> & dst, const std::string & name, const std::string & value)
  {
    const std::string body = name + "=" + value;
    put_u32(dst, static_cast<std::uint32_t>(body.size()));
    put_str(dst, body);
  }

  // "name=<raw bytes>" with `value` interpreted as binary.
  static void put_field_bytes(
    std::vector<std::byte> & dst, const std::string & name, const std::vector<std::byte> & value)
  {
    std::vector<std::byte> body;
    put_str(body, name + "=");
    body.insert(body.end(), value.begin(), value.end());
    put_u32(dst, static_cast<std::uint32_t>(body.size()));
    dst.insert(dst.end(), body.begin(), body.end());
  }

  // op field is one byte stored as a binary value.
  static void put_field_op(std::vector<std::byte> & dst, std::uint8_t op)
  {
    put_field_bytes(dst, "op", {static_cast<std::byte>(op)});
  }

  // u32 field stored as little-endian binary value.
  static void put_field_u32(std::vector<std::byte> & dst, const std::string & name, std::uint32_t v)
  {
    std::vector<std::byte> raw;
    put_u32(raw, v);
    put_field_bytes(dst, name, raw);
  }

  // u64 field stored as little-endian binary value.
  static void put_field_u64(std::vector<std::byte> & dst, const std::string & name, std::uint64_t v)
  {
    std::vector<std::byte> raw;
    for (int i = 0; i < 8; ++i) {
      raw.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFF));
    }
    put_field_bytes(dst, name, raw);
  }

  static void put_field_time(
    std::vector<std::byte> & dst, const std::string & name, std::uint32_t sec, std::uint32_t nsec)
  {
    std::vector<std::byte> raw;
    put_u32(raw, sec);
    put_u32(raw, nsec);
    put_field_bytes(dst, name, raw);
  }

  // Wrap a header + data pair as a record in the file body.
  static void put_record(
    std::vector<std::byte> & dst, const std::vector<std::byte> & header,
    const std::vector<std::byte> & data)
  {
    put_u32(dst, static_cast<std::uint32_t>(header.size()));
    dst.insert(dst.end(), header.begin(), header.end());
    put_u32(dst, static_cast<std::uint32_t>(data.size()));
    dst.insert(dst.end(), data.begin(), data.end());
  }
};

std::filesystem::path write_to_temp(const std::vector<std::byte> & bytes)
{
  auto path = std::filesystem::temp_directory_path() /
              ("bagwiz_rosbag1_test_" + std::to_string(::getpid()) + ".bag");
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  return path;
}

}  // namespace

TEST(Rosbag1Reader, ReadsConnectionsAndMessages)
{
  using B = Bag1Builder;
  std::vector<std::byte> file;

  // Magic.
  B::put_str(file, "#ROSBAG V2.0\n");

  // BAG_HEADER (op=0x03). index_pos / conn_count / chunk_count are not
  // load-bearing for our reader; data is just padding.
  {
    std::vector<std::byte> hdr;
    B::put_field_op(hdr, 0x03);
    B::put_field_u64(hdr, "index_pos", 0);
    B::put_field_u32(hdr, "conn_count", 1);
    B::put_field_u32(hdr, "chunk_count", 1);
    std::vector<std::byte> data(64, std::byte{0x20});  // arbitrary pad
    B::put_record(file, hdr, data);
  }

  // CHUNK (op=0x05). The chunk's `data` block holds inner CONNECTION
  // + MESSAGE_DATA records back-to-back.
  std::vector<std::byte> chunk_data;

  // CONNECTION record (op=0x07) for conn=42, topic=/chatter, type=std_msgs/String.
  {
    std::vector<std::byte> hdr;
    B::put_field_op(hdr, 0x07);
    B::put_field_u32(hdr, "conn", 42);
    B::put_field_str(hdr, "topic", "/chatter");

    std::vector<std::byte> data;
    B::put_field_str(data, "topic", "/chatter");
    B::put_field_str(data, "type", "std_msgs/String");
    B::put_field_str(data, "md5sum", "992ce8a1687cec8cc7c4e8a55a04a78a");
    B::put_field_str(data, "message_definition", "string data\n");

    B::put_record(chunk_data, hdr, data);
  }

  // Two MESSAGE_DATA records (op=0x02).
  for (int i = 0; i < 2; ++i) {
    std::vector<std::byte> hdr;
    B::put_field_op(hdr, 0x02);
    B::put_field_u32(hdr, "conn", 42);
    B::put_field_time(hdr, "time", 1700000000U + i, 0U);

    // ROS 1 std_msgs/String payload: u32 length + bytes.
    std::vector<std::byte> data;
    const std::string text = (i == 0) ? "hello" : "world";
    B::put_u32(data, static_cast<std::uint32_t>(text.size()));
    B::put_str(data, text);

    B::put_record(chunk_data, hdr, data);
  }

  // Wrap chunk_data into a CHUNK record on the outer file.
  {
    std::vector<std::byte> hdr;
    B::put_field_op(hdr, 0x05);
    B::put_field_str(hdr, "compression", "none");
    B::put_field_u32(hdr, "size", static_cast<std::uint32_t>(chunk_data.size()));
    B::put_record(file, hdr, chunk_data);
  }

  const auto path = write_to_temp(file);

  bagwiz::io::Rosbag1Reader reader(path);

  bagwiz::io::Ros1Message msg;

  ASSERT_TRUE(reader.next(msg));
  // After the first next() the connection has been observed.
  ASSERT_EQ(reader.connections().size(), 1u);
  EXPECT_EQ(reader.connections()[0].conn_id, 42u);
  EXPECT_EQ(reader.connections()[0].topic, "/chatter");
  EXPECT_EQ(reader.connections()[0].type, "std_msgs/String");

  EXPECT_EQ(msg.conn_id, 42u);
  EXPECT_EQ(msg.timestamp_ns, 1'700'000'000'000'000'000LL);
  EXPECT_EQ(msg.payload.size(), 4u + 5u);  // u32 length + "hello"

  ASSERT_TRUE(reader.next(msg));
  EXPECT_EQ(msg.timestamp_ns, 1'700'000'001'000'000'000LL);

  EXPECT_FALSE(reader.next(msg));

  std::filesystem::remove(path);
}

TEST(Rosbag1Reader, RejectsCompressedChunk)
{
  using B = Bag1Builder;
  std::vector<std::byte> file;
  B::put_str(file, "#ROSBAG V2.0\n");

  std::vector<std::byte> hdr;
  B::put_field_op(hdr, 0x05);
  B::put_field_str(hdr, "compression", "bz2");
  B::put_field_u32(hdr, "size", 0);
  std::vector<std::byte> data;  // empty payload — will not be parsed
  B::put_record(file, hdr, data);

  const auto path = write_to_temp(file);

  bagwiz::io::Rosbag1Reader reader(path);
  bagwiz::io::Ros1Message msg;
  EXPECT_THROW(reader.next(msg), std::runtime_error);

  std::filesystem::remove(path);
}

TEST(Rosbag1Reader, RejectsBadMagic)
{
  std::vector<std::byte> file;
  Bag1Builder::put_str(file, "NOT A BAG FILE!");
  const auto path = write_to_temp(file);
  EXPECT_THROW(bagwiz::io::Rosbag1Reader{path}, std::runtime_error);
  std::filesystem::remove(path);
}
