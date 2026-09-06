// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/draco_codec.hpp"

#include "bagwiz/core/base/tolerance.hpp"
#include "bagwiz/core/pointcloud/compressed_pointcloud2.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::CompressedPointCloud2;
using bagwiz::core::pointcloud::DracoEncodeConfig;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;

// x, y, z (float32) + intensity (float32) + ring (uint16), tightly packed.
PointCloud2 make_cloud(std::uint32_t num_points)
{
  PointCloud2 cloud;
  cloud.timestamp_ns = 123456789;
  cloud.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = num_points;
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
    PointField{"z", 8, PointFieldType::kFloat32, 1},
    PointField{"intensity", 12, PointFieldType::kFloat32, 1},
    PointField{"ring", 16, PointFieldType::kUint16, 1},
  };
  cloud.point_step = 18;
  cloud.row_step = 18 * num_points;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;

  for (std::uint32_t i = 0; i < num_points; ++i) {
    // Keep coordinates within +-10 m so the 14-bit default quantization error
    // (range / 2^14 ~ 1.2 mm over a 20 m span) stays testable against
    // tolerance::kPointMeters.
    const float x = -10.0f + 0.01f * static_cast<float>(i % 2000);
    const float y = 5.0f - 0.007f * static_cast<float>(i % 1000);
    const float z = 0.5f + 0.003f * static_cast<float>(i % 500);
    const float intensity = static_cast<float>(i % 256);
    const std::uint16_t ring = static_cast<std::uint16_t>(i % 128);
    std::byte * p = cloud.data.data() + static_cast<std::size_t>(i) * cloud.point_step;
    std::memcpy(p + 0, &x, 4);
    std::memcpy(p + 4, &y, 4);
    std::memcpy(p + 8, &z, 4);
    std::memcpy(p + 12, &intensity, 4);
    std::memcpy(p + 16, &ring, 2);
  }
  return cloud;
}

float read_float(const std::vector<std::byte> & data, std::size_t point, std::uint32_t offset)
{
  float v;
  std::memcpy(&v, data.data() + point * 18 + offset, 4);
  return v;
}

}  // namespace

TEST(DracoCodec, LosslessRoundTripIsBitExact)
{
  // Exact agreement is asserted because sequential encoding without
  // quantization stores float and integer attribute values verbatim: each
  // point's bytes are a pure function of that point's input bytes, so the
  // round trip is a per-element computation over immutable input.
  const auto cloud = make_cloud(1000);
  DracoEncodeConfig config;
  config.lossless = true;

  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, config);
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  ASSERT_FALSE(encoded.data->empty());
  EXPECT_LT(encoded.data->size(), cloud.data.size());

  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(decoded.num_points, cloud.width);
  ASSERT_EQ(decoded.data->size(), cloud.data.size());
  EXPECT_EQ(*decoded.data, cloud.data);
}

TEST(DracoCodec, LossyRoundTripStaysWithinQuantizationTolerance)
{
  const auto cloud = make_cloud(1000);
  DracoEncodeConfig config;  // 14-bit quantization, the default

  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, config);
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  ASSERT_EQ(decoded.num_points, cloud.width);
  ASSERT_EQ(decoded.data->size(), cloud.data.size());

  for (std::uint32_t i = 0; i < cloud.width; ++i) {
    for (const std::uint32_t offset : {0u, 4u, 8u}) {
      EXPECT_NEAR(
        read_float(*decoded.data, i, offset), read_float(cloud.data, i, offset),
        bagwiz::core::base::tolerance::kPointMeters);
    }
    // uint16 ring is not quantized: exact.
    EXPECT_EQ(
      std::memcmp(decoded.data->data() + i * 18 + 16, cloud.data.data() + i * 18 + 16, 2), 0);
  }
}

TEST(DracoCodec, PackedRgbRoundTripIsBitExact)
{
  // rgb is a packed float32; the codec reinterprets its 4 bytes as 4 uint8
  // channels. Lossless mode must return the same bytes.
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 3;
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
    PointField{"z", 8, PointFieldType::kFloat32, 1},
    PointField{"rgb", 12, PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 16;
  cloud.row_step = 48;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;
  const float xyz[3][3] = {{0, 0, 0}, {1, 1, 1}, {2, 2, 2}};
  const std::uint8_t rgb[3][4] = {{255, 0, 0, 0}, {0, 255, 0, 0}, {0, 0, 255, 0}};
  for (std::uint32_t i = 0; i < 3; ++i) {
    std::byte * p = cloud.data.data() + i * cloud.point_step;
    std::memcpy(p, xyz[i], 12);
    std::memcpy(p + 12, rgb[i], 4);
  }

  DracoEncodeConfig config;
  config.lossless = true;
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, config);
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  ASSERT_EQ(decoded.num_points, 3u);
  EXPECT_EQ(*decoded.data, cloud.data);
}

TEST(DracoCodec, PackedRgbStaysExactInLossyMode)
{
  // The rgb tweak carries the packed bytes as uint8 channels, and Draco
  // quantizes float attributes only — so color bytes survive even the default
  // lossy mode unchanged (per-element byte copies of immutable input).
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 2;
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
    PointField{"z", 8, PointFieldType::kFloat32, 1},
    PointField{"rgb", 12, PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 16;
  cloud.row_step = 32;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;
  const float xyz[2][3] = {{0, 0, 0}, {1, 1, 1}};
  const std::uint8_t rgb[2][4] = {{10, 20, 30, 0}, {200, 100, 50, 0}};
  for (std::uint32_t i = 0; i < 2; ++i) {
    std::byte * p = cloud.data.data() + i * cloud.point_step;
    std::memcpy(p, xyz[i], 12);
    std::memcpy(p + 12, rgb[i], 4);
  }

  const auto encoded =
    bagwiz::core::pointcloud::encode_draco(cloud, DracoEncodeConfig{});  // lossy default
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  ASSERT_EQ(decoded.num_points, 2u);
  for (std::uint32_t i = 0; i < 2; ++i) {
    EXPECT_EQ(
      std::memcmp(decoded.data->data() + i * 16 + 12, cloud.data.data() + i * 16 + 12, 4), 0);
  }
}

TEST(DracoCodec, CloudHasNonFiniteScansValuesNotTheFlag)
{
  // Finite data flagged is_dense=false (the real-world Seyond/Autoware case):
  // not non-finite.
  auto cloud = make_cloud(64);
  cloud.is_dense = false;
  EXPECT_FALSE(bagwiz::core::pointcloud::cloud_has_non_finite(cloud));

  // A NaN in any float field (here: intensity, not just x/y/z) is caught.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(cloud.data.data() + 3 * cloud.point_step + 12, &nan, 4);
  EXPECT_TRUE(bagwiz::core::pointcloud::cloud_has_non_finite(cloud));
}

TEST(DracoCodec, EmptyCloudRoundTripsAsEmptyBuffer)
{
  const auto cloud = make_cloud(0);
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, DracoEncodeConfig{});
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  EXPECT_TRUE(encoded.data->empty());

  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(decoded.num_points, 0u);
  EXPECT_TRUE(decoded.data->empty());
}

TEST(DracoCodec, RejectsBigEndianCloud)
{
  auto cloud = make_cloud(4);
  cloud.is_bigendian = true;
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, DracoEncodeConfig{});
  EXPECT_FALSE(encoded.ok());
}

TEST(DracoCodec, RejectsRowPadding)
{
  // An organized cloud with row_step > width * point_step: attributes are read
  // point-contiguously, so encoding it would corrupt every row after the
  // first. The codec must reject it (the command then passes it through).
  auto cloud = make_cloud(4);
  cloud.height = 2;
  cloud.width = 2;
  cloud.row_step = 2 * cloud.point_step + 8;  // 8 padding bytes per row
  cloud.data.resize(cloud.row_step * 2);
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, DracoEncodeConfig{});
  EXPECT_FALSE(encoded.ok());
}

TEST(DracoCodec, AcceptsStaleRowStepOnSingleRowCloud)
{
  // A height=1 cloud whose row_step disagrees with width * point_step (stale
  // metadata from an upstream filter, seen in real Autoware recordings): the
  // row stride is not layout for a single row, so the codec encodes normally.
  // The round trip is still bit-exact (per-element copies of immutable input).
  auto cloud = make_cloud(8);
  cloud.row_step = 12345;  // stale
  DracoEncodeConfig config;
  config.lossless = true;
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, config);
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  const auto decoded =
    bagwiz::core::pointcloud::decode_draco(*encoded.data, cloud.fields, cloud.point_step);
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(*decoded.data, cloud.data);
}

TEST(DracoCodec, RejectsInvalidFieldLayoutEvenWhenEmpty)
{
  // A 0-point cloud whose fields extend past point_step must not enter the
  // bag: decode_draco validates the layout before its empty-buffer shortcut,
  // so encode_draco validates before its own empty shortcut too, keeping a
  // bagwiz-written bag always decodable by bagwiz.
  auto cloud = make_cloud(0);
  cloud.point_step = 4;  // x alone needs 4; y at offset 4 already overflows
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, DracoEncodeConfig{});
  EXPECT_FALSE(encoded.ok());
}

TEST(DracoCodec, RejectsFieldLayoutMismatchOnDecode)
{
  const auto cloud = make_cloud(8);
  DracoEncodeConfig config;
  config.lossless = true;
  const auto encoded = bagwiz::core::pointcloud::encode_draco(cloud, config);
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  auto wrong_fields = cloud.fields;
  wrong_fields.back().datatype = PointFieldType::kUint32;  // 2-byte ring widened to 4
  const auto decoded = bagwiz::core::pointcloud::decode_draco(*encoded.data, wrong_fields, 20);
  EXPECT_FALSE(decoded.ok());
}

TEST(CompressedPointCloud2Cdr, RoundTripIsByteExact)
{
  CompressedPointCloud2 message;
  message.timestamp_ns = 9876543210LL;
  message.frame_id = "lidar";
  message.height = 2;
  message.width = 64;
  message.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"ring", 16, PointFieldType::kUint16, 1},
  };
  message.point_step = 18;
  message.row_step = 18 * 64;
  message.compressed_data = {std::byte{0xDE}, std::byte{0xAD}, std::byte{0xBE}, std::byte{0xEF}};
  message.is_dense = true;
  message.format = "draco";

  const auto payload = bagwiz::core::pointcloud::serialize_compressed_pointcloud2(message);
  const auto parsed = bagwiz::core::pointcloud::parse_compressed_pointcloud2(payload);
  ASSERT_TRUE(parsed.ok()) << parsed.error;

  const auto & m = *parsed.message;
  EXPECT_EQ(m.timestamp_ns, message.timestamp_ns);
  EXPECT_EQ(m.frame_id, message.frame_id);
  EXPECT_EQ(m.height, message.height);
  EXPECT_EQ(m.width, message.width);
  ASSERT_EQ(m.fields.size(), 2u);
  EXPECT_EQ(m.fields[0].name, "x");
  EXPECT_EQ(m.fields[0].offset, 0u);
  EXPECT_EQ(m.fields[0].datatype, PointFieldType::kFloat32);
  EXPECT_EQ(m.fields[0].count, 1u);
  EXPECT_EQ(m.fields[1].name, "ring");
  EXPECT_EQ(m.fields[1].datatype, PointFieldType::kUint16);
  EXPECT_EQ(m.point_step, message.point_step);
  EXPECT_EQ(m.row_step, message.row_step);
  EXPECT_EQ(m.compressed_data, message.compressed_data);
  EXPECT_TRUE(m.is_dense);
  EXPECT_EQ(m.format, "draco");

  // Re-serializing the parsed message reproduces the same bytes: the codec is
  // byte-exact in both directions (per-element copies of immutable input).
  EXPECT_EQ(bagwiz::core::pointcloud::serialize_compressed_pointcloud2(m), payload);
}

TEST(CompressedPointCloud2Cdr, RejectsTruncatedPayload)
{
  CompressedPointCloud2 message;
  message.frame_id = "lidar";
  message.format = "draco";
  auto payload = bagwiz::core::pointcloud::serialize_compressed_pointcloud2(message);
  payload.resize(payload.size() / 2);
  const auto parsed = bagwiz::core::pointcloud::parse_compressed_pointcloud2(payload);
  EXPECT_FALSE(parsed.ok());
}
