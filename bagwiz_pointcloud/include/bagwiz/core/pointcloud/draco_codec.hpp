// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__DRACO_CODEC_HPP_
#define BAGWIZ__CORE__POINTCLOUD__DRACO_CODEC_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// The `format` value written into CompressedPointCloud2 messages produced by
// encode_draco, matching the draco_point_cloud_transport transport name.
inline constexpr const char * kDracoFormatName = "draco";

// Knobs of the Draco encoder. Defaults mirror draco_point_cloud_transport's
// defaults (speeds 7/7, 14 quantization bits per attribute type), except that
// deduplication is always off: bagwiz rewrites recorded bags, where point
// count and order are downstream semantics (deskew walks points in order), so
// the codec never reorders or drops points.
struct DracoEncodeConfig
{
  int encode_speed = 7;  // 0 = best ratio, 10 = fastest
  int decode_speed = 7;  // 0 = best ratio, 10 = fastest
  // false (default): float attributes are quantized to the per-type bit counts
  // below (lossy). true: no quantization, floats round-trip bit-exact.
  bool lossless = false;
  int quantization_position = 14;
  int quantization_normal = 14;
  int quantization_color = 14;
  int quantization_generic = 14;
};

struct DracoEncodeResult
{
  // The encoded bitstream. An empty (but present) vector is a 0-point cloud:
  // Draco cannot encode an empty cloud, so the empty case round-trips as an
  // empty buffer instead.
  std::optional<std::vector<std::byte>> data;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return data.has_value() && error.empty(); }
};

struct DracoDecodeResult
{
  // The reconstructed point blob: num_points * point_step bytes, each
  // attribute's bytes placed at the offset its PointField declares.
  std::optional<std::vector<std::byte>> data;
  std::uint32_t num_points = 0;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return data.has_value() && error.empty(); }
};

// Encode a PointCloud2's point payload with Draco. Only `data` (interpreted
// through fields/point_step/height/width) is compressed; the header metadata
// travels uncompressed in the enclosing CompressedPointCloud2 message.
//
// PointField name -> Draco attribute type follows draco_point_cloud_transport's
// table (x/y/z -> POSITION, rgb/rgba -> COLOR, nx/ny/nz -> NORMAL, ...), as does
// the rgb/rgba tweak (a packed float32 rgb field is carried as 4 uint8s).
// Big-endian clouds are rejected.
[[nodiscard]] DracoEncodeResult encode_draco(
  const PointCloud2 & cloud, const DracoEncodeConfig & config);

// Inverse of encode_draco: rebuild the point blob in the exact layout
// `fields` + `point_step` describe (the metadata stored in the enclosing
// CompressedPointCloud2). Attribute i of the Draco bitstream must correspond
// to fields[i], in the order encode_draco created them; a byte-size mismatch
// between an attribute and its field is an error.
[[nodiscard]] DracoDecodeResult decode_draco(
  std::span<const std::byte> data, const std::vector<PointField> & fields,
  std::uint32_t point_step);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__DRACO_CODEC_HPP_
