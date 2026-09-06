// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__COMPRESSED_POINTCLOUD2_HPP_
#define BAGWIZ__CORE__POINTCLOUD__COMPRESSED_POINTCLOUD2_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// The full type name of the compressed-cloud message `pcd compress` writes and
// `pcd decompress` reads. It matches the point_cloud_transport ecosystem's
// message (same field layout), so bags written by bagwiz decompress with the
// standard draco_point_cloud_transport subscriber and vice versa.
inline constexpr const char * kCompressedPointCloud2TypeName =
  "point_cloud_interfaces/msg/CompressedPointCloud2";

// Dependency-free model of point_cloud_interfaces/msg/CompressedPointCloud2:
//
//   std_msgs/Header header
//   uint32 height
//   uint32 width
//   sensor_msgs/PointField[] fields
//   bool is_bigendian
//   uint32 point_step
//   uint32 row_step
//   uint8[] compressed_data
//   bool is_dense
//   string format                // codec name, e.g. "draco"
//
// height/width/fields/... describe the ORIGINAL uncompressed cloud, so a
// decoder can reconstruct the exact PointCloud2 layout (offsets, point_step)
// from this message alone.
struct CompressedPointCloud2
{
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::vector<std::byte> compressed_data;
  bool is_dense = false;
  std::string format;
};

struct CompressedPointCloud2Result
{
  std::optional<CompressedPointCloud2> message;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return message.has_value() && error.empty(); }
};

[[nodiscard]] CompressedPointCloud2Result parse_compressed_pointcloud2(
  std::span<const std::byte> payload);

// Inverse of parse_compressed_pointcloud2: plain little-endian CDR-1, so
// parse(serialize(m)) == m field-for-field and byte-for-byte in
// compressed_data (same contract as serialize_pointcloud2).
[[nodiscard]] std::vector<std::byte> serialize_compressed_pointcloud2(
  const CompressedPointCloud2 & message);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__COMPRESSED_POINTCLOUD2_HPP_
