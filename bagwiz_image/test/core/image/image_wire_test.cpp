// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_wire.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/raw_image.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::image::extract_compressed_image;
using bagwiz::core::image::extract_raw_image;
using bagwiz::core::image::kCompressedImageType;
using bagwiz::core::image::kImageType;
using bagwiz::core::image::make_compressed_image_topic_info;
using bagwiz::core::image::make_image_topic_info;
using bagwiz::core::image::serialize_compressed_image;
using bagwiz::core::image::serialize_raw_image;

std::vector<std::byte> pixels(std::size_t n)
{
  std::vector<std::byte> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = static_cast<std::byte>(i & 0xFFU);
  }
  return out;
}

}  // namespace

TEST(ImageWireTest, RawImageRoundTripsThroughExtractor)
{
  const auto data = pixels(4 * 3 * 2);  // 4x2 bgr8, step 12
  const std::int64_t stamp_ns = 1'700'000'000'500'000'000LL;

  const auto payload = serialize_raw_image(stamp_ns, "cam0", 4, 2, "bgr8", 12, data);
  const auto result = extract_raw_image(payload);

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 4U);
  EXPECT_EQ(result.image->height, 2U);
  EXPECT_EQ(result.image->step, 12U);
  EXPECT_EQ(result.image->encoding, "bgr8");
  EXPECT_EQ(result.image->header_stamp_ns, stamp_ns);
  EXPECT_EQ(result.image->header_frame_id, "cam0");
  ASSERT_EQ(result.image->data.size(), data.size());
  EXPECT_TRUE(std::equal(data.begin(), data.end(), result.image->data.begin()));
}

TEST(ImageWireTest, RawImageWritesLittleEndianFlagAndFieldOrder)
{
  const auto data = pixels(3);
  const auto payload = serialize_raw_image(5, "f", 1, 1, "rgb8", 3, data);

  bagwiz::core::cdr_walker::CdrReader reader(payload);
  EXPECT_EQ(reader.read_i32(), 0);   // header.stamp.sec
  EXPECT_EQ(reader.read_u32(), 5U);  // header.stamp.nanosec
  EXPECT_EQ(reader.read_string(), "f");
  EXPECT_EQ(reader.read_u32(), 1U);  // height
  EXPECT_EQ(reader.read_u32(), 1U);  // width
  EXPECT_EQ(reader.read_string(), "rgb8");
  EXPECT_EQ(reader.read_u8(), 0U);   // is_bigendian
  EXPECT_EQ(reader.read_u32(), 3U);  // step
  EXPECT_EQ(reader.read_sequence_length(), 3U);
}

TEST(ImageWireTest, CompressedImageRoundTripsThroughExtractor)
{
  const std::vector<std::byte> blob{std::byte{0xFF}, std::byte{0xD8}, std::byte{0xFF}};
  const std::int64_t stamp_ns = 42;

  const auto payload = serialize_compressed_image(stamp_ns, "cam1", "jpeg", blob);
  const auto result = extract_compressed_image(payload);

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->format, "jpeg");
  EXPECT_EQ(result.image->header_stamp_ns, stamp_ns);
  EXPECT_EQ(result.image->header_frame_id, "cam1");
  ASSERT_EQ(result.image->data.size(), blob.size());
  EXPECT_TRUE(std::equal(blob.begin(), blob.end(), result.image->data.begin()));
}

TEST(ImageWireTest, TopicInfosNameTheSensorMsgsTypes)
{
  const auto raw = make_image_topic_info("/cam/image_raw");
  EXPECT_EQ(raw.name, "/cam/image_raw");
  EXPECT_EQ(raw.type, kImageType);
  EXPECT_EQ(raw.type, "sensor_msgs/msg/Image");
  EXPECT_EQ(raw.serialization_format, "cdr");

  const auto compressed = make_compressed_image_topic_info("/cam/image_raw/compressed");
  EXPECT_EQ(compressed.name, "/cam/image_raw/compressed");
  EXPECT_EQ(compressed.type, kCompressedImageType);
  EXPECT_EQ(compressed.type, "sensor_msgs/msg/CompressedImage");
  EXPECT_EQ(compressed.serialization_format, "cdr");

  // The schema is resolved from the installed sensor_msgs definitions (whose
  // field lines carry trailing comments, so only the declarations are
  // matched); when it resolves it must be the concatenated ros2msg form, and
  // when it does not the pair stays consistently empty so the writer records
  // "no schema".
  for (const auto & info : {raw, compressed}) {
    if (info.schema_text.empty()) {
      EXPECT_TRUE(info.schema_encoding.empty());
      continue;
    }
    EXPECT_EQ(info.schema_encoding, "ros2msg");
    EXPECT_NE(info.schema_text.find("std_msgs/Header header"), std::string::npos);
    EXPECT_NE(info.schema_text.find("MSG: std_msgs/Header\n"), std::string::npos);
    EXPECT_NE(info.schema_text.find("MSG: builtin_interfaces/Time\n"), std::string::npos);
  }
  if (!raw.schema_text.empty()) {
    EXPECT_NE(raw.schema_text.find("uint8[] data"), std::string::npos);
    EXPECT_NE(raw.schema_text.find("string encoding"), std::string::npos);
  }
  if (!compressed.schema_text.empty()) {
    EXPECT_NE(compressed.schema_text.find("string format"), std::string::npos);
  }
}
