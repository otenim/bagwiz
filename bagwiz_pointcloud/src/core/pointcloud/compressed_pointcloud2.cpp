// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/compressed_pointcloud2.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"
#include "bagwiz/core/cdr_walker/cdr_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

// point_cloud_interfaces/msg/CompressedPointCloud2 CDR layout (CDR-1):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   uint32 height
//   uint32 width
//   PointField[] fields
//     string name
//     uint32 offset
//     uint8  datatype
//     uint32 count
//   bool   is_bigendian
//   uint32 point_step
//   uint32 row_step
//   uint8[] compressed_data      // length-prefixed
//   bool   is_dense
//   string format
CompressedPointCloud2Result parse_compressed_pointcloud2(std::span<const std::byte> payload)
{
  CompressedPointCloud2Result result;
  try {
    cdr_walker::CdrReader reader(payload);

    CompressedPointCloud2 message;
    const std::int32_t sec = reader.read_i32();
    const std::uint32_t nanosec = reader.read_u32();
    message.timestamp_ns = static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
    message.frame_id = reader.read_string();
    message.height = reader.read_u32();
    message.width = reader.read_u32();

    const std::uint32_t field_count = reader.read_sequence_length();
    message.fields.resize(field_count);
    for (std::uint32_t i = 0; i < field_count; ++i) {
      auto & f = message.fields[i];
      f.name = reader.read_string();
      f.offset = reader.read_u32();
      f.datatype = static_cast<PointFieldType>(reader.read_u8());
      f.count = reader.read_u32();
    }

    message.is_bigendian = reader.read_bool();
    message.point_step = reader.read_u32();
    message.row_step = reader.read_u32();

    const std::uint32_t data_len = reader.read_sequence_length();
    const auto data_span = reader.read_bytes(data_len);
    message.compressed_data.assign(data_span.begin(), data_span.end());

    message.is_dense = reader.read_bool();
    message.format = reader.read_string();
    result.message.emplace(std::move(message));
  } catch (const std::exception & e) {
    result.message.reset();
    result.error =
      std::string("failed to parse point_cloud_interfaces/msg/CompressedPointCloud2 payload: ") +
      e.what();
  }
  return result;
}

std::vector<std::byte> serialize_compressed_pointcloud2(const CompressedPointCloud2 & message)
{
  cdr_walker::CdrWriter writer;

  const std::int64_t ts = message.timestamp_ns;
  writer.write_i32(static_cast<std::int32_t>(ts / 1'000'000'000LL));
  writer.write_u32(static_cast<std::uint32_t>(ts % 1'000'000'000LL));
  writer.write_string(message.frame_id);
  writer.write_u32(message.height);
  writer.write_u32(message.width);

  writer.write_sequence_length(static_cast<std::uint32_t>(message.fields.size()));
  for (const auto & f : message.fields) {
    writer.write_string(f.name);
    writer.write_u32(f.offset);
    writer.write_u8(static_cast<std::uint8_t>(f.datatype));
    writer.write_u32(f.count);
  }

  writer.write_bool(message.is_bigendian);
  writer.write_u32(message.point_step);
  writer.write_u32(message.row_step);

  writer.write_sequence_length(static_cast<std::uint32_t>(message.compressed_data.size()));
  writer.write_bytes(message.compressed_data);

  writer.write_bool(message.is_dense);
  writer.write_string(message.format);
  return writer.take();
}

}  // namespace bagwiz::core::pointcloud
