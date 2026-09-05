// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/compressed_video.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::image::extract_compressed_video;
using bagwiz::core::image::kCompressedVideoType;
using bagwiz::core::image::make_compressed_video_topic_info;
using bagwiz::core::image::serialize_compressed_video;

std::vector<std::byte> annexb_frame()
{
  // A start code followed by a few arbitrary payload bytes.
  return {std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x01},
          std::byte{0x65}, std::byte{0x88}, std::byte{0x84}};
}

}  // namespace

TEST(CompressedVideoTest, SerializeThenExtractRoundTrips)
{
  const auto frame = annexb_frame();
  const std::int64_t stamp_ns = 1'700'000'000'250'000'000LL;  // 1700000000.25 s

  const auto payload = serialize_compressed_video(stamp_ns, "cam0_optical", "h264", frame);
  const auto result = extract_compressed_video(payload);

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.video->timestamp_ns, stamp_ns);
  EXPECT_EQ(result.video->frame_id, "cam0_optical");
  EXPECT_EQ(result.video->format, "h264");
  ASSERT_EQ(result.video->data.size(), frame.size());
  EXPECT_TRUE(std::equal(frame.begin(), frame.end(), result.video->data.begin()));
  // The view borrows from the payload rather than copying it.
  EXPECT_GE(result.video->data.data(), payload.data());
  EXPECT_LT(result.video->data.data(), payload.data() + payload.size());
}

TEST(CompressedVideoTest, SerializedBytesFollowCdrLayout)
{
  const std::vector<std::byte> data{std::byte{0xAA}, std::byte{0xBB}};
  const auto payload = serialize_compressed_video(3'000'000'007LL, "f", "h265", data);

  // 4-byte encapsulation header (little-endian CDR), then the fields in
  // declaration order: int32 sec, uint32 nanosec, string frame_id,
  // uint8[] data (length-prefixed), string format.
  ASSERT_GE(payload.size(), 4U);
  EXPECT_EQ(payload[0], std::byte{0x00});
  EXPECT_EQ(payload[1], std::byte{0x01});
  bagwiz::core::cdr_walker::CdrReader reader(payload);
  EXPECT_EQ(reader.read_i32(), 3);
  EXPECT_EQ(reader.read_u32(), 7U);
  EXPECT_EQ(reader.read_string(), "f");
  EXPECT_EQ(reader.read_sequence_length(), 2U);
  const auto bytes = reader.read_bytes(2);
  EXPECT_EQ(bytes[0], std::byte{0xAA});
  EXPECT_EQ(bytes[1], std::byte{0xBB});
  EXPECT_EQ(reader.read_string(), "h265");
}

TEST(CompressedVideoTest, NegativeStampSplitsWithFloorSemantics)
{
  // builtin_interfaces/Time keeps nanosec in [0, 1e9): -1.7 s is
  // {sec: -2, nanosec: 3e8}, not {sec: -1, nanosec: -7e8}.
  const auto payload = serialize_compressed_video(-1'700'000'000LL, "", "h264", {});
  bagwiz::core::cdr_walker::CdrReader reader(payload);
  EXPECT_EQ(reader.read_i32(), -2);
  EXPECT_EQ(reader.read_u32(), 300'000'000U);

  const auto result = extract_compressed_video(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.video->timestamp_ns, -1'700'000'000LL);
  EXPECT_TRUE(result.video->frame_id.empty());
  EXPECT_TRUE(result.video->data.empty());
}

TEST(CompressedVideoTest, ExtractRejectsTruncatedPayload)
{
  const auto frame = annexb_frame();
  auto payload = serialize_compressed_video(1, "cam", "h264", frame);
  payload.resize(payload.size() - 3);  // cut into the format string

  const auto result = extract_compressed_video(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_NE(result.error.find("CompressedVideo"), std::string::npos) << result.error;
}

TEST(CompressedVideoTest, TopicInfoIsSelfDescribing)
{
  const auto info = make_compressed_video_topic_info("/cam0/video");

  EXPECT_EQ(info.name, "/cam0/video");
  EXPECT_EQ(info.type, kCompressedVideoType);
  EXPECT_EQ(info.type, "foxglove_msgs/msg/CompressedVideo");
  EXPECT_EQ(info.serialization_format, "cdr");
  EXPECT_EQ(info.schema_encoding, "ros2msg");
  // The field declarations, in order, followed by the dependency section
  // rosbag2 appends for builtin_interfaces/Time.
  const auto & schema = info.schema_text;
  const auto timestamp_at = schema.find("builtin_interfaces/Time timestamp\n");
  const auto frame_id_at = schema.find("string frame_id\n");
  const auto data_at = schema.find("uint8[] data\n");
  const auto format_at = schema.find("string format\n");
  const auto dep_at = schema.find(
    "================================================================================\n"
    "MSG: builtin_interfaces/Time\n");
  ASSERT_NE(timestamp_at, std::string::npos);
  ASSERT_NE(frame_id_at, std::string::npos);
  ASSERT_NE(data_at, std::string::npos);
  ASSERT_NE(format_at, std::string::npos);
  ASSERT_NE(dep_at, std::string::npos);
  EXPECT_LT(timestamp_at, frame_id_at);
  EXPECT_LT(frame_id_at, data_at);
  EXPECT_LT(data_at, format_at);
  EXPECT_LT(format_at, dep_at);
  EXPECT_NE(schema.find("int32 sec\n", dep_at), std::string::npos);
  EXPECT_NE(schema.find("uint32 nanosec\n", dep_at), std::string::npos);
  EXPECT_EQ(schema.back(), '\n');
}
