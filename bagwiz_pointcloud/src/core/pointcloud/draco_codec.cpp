// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/draco_codec.hpp"

#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/metadata/geometry_metadata.h>
#include <draco/point_cloud/point_cloud_builder.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

// How one PointField becomes one Draco attribute. A field maps 1:1 to an
// attribute whose value occupies exactly the field's byte count, so decode is
// a byte copy per point — including the rgb/rgba tweak, where the packed
// float32 (4 bytes) is reinterpreted as 4 uint8 color channels.
struct FieldMapping
{
  draco::GeometryAttribute::Type attribute_type = draco::GeometryAttribute::INVALID;
  draco::DataType data_type = draco::DT_INVALID;
  int num_components = 0;
  std::size_t value_bytes = 0;  // per-point byte count of the attribute value
};

draco::GeometryAttribute::Type attribute_type_for(const std::string & name)
{
  // The name table draco_point_cloud_transport's publisher uses, so bags are
  // interchangeable with it in both directions.
  static const std::pair<const char *, draco::GeometryAttribute::Type> kTable[] = {
    {"x", draco::GeometryAttribute::POSITION},
    {"y", draco::GeometryAttribute::POSITION},
    {"z", draco::GeometryAttribute::POSITION},
    {"pos", draco::GeometryAttribute::POSITION},
    {"position", draco::GeometryAttribute::POSITION},
    {"vp_x", draco::GeometryAttribute::POSITION},
    {"vp_y", draco::GeometryAttribute::POSITION},
    {"vp_z", draco::GeometryAttribute::POSITION},
    {"rgb", draco::GeometryAttribute::COLOR},
    {"rgba", draco::GeometryAttribute::COLOR},
    {"r", draco::GeometryAttribute::COLOR},
    {"g", draco::GeometryAttribute::COLOR},
    {"b", draco::GeometryAttribute::COLOR},
    {"a", draco::GeometryAttribute::COLOR},
    {"nx", draco::GeometryAttribute::NORMAL},
    {"ny", draco::GeometryAttribute::NORMAL},
    {"nz", draco::GeometryAttribute::NORMAL},
    {"normal_x", draco::GeometryAttribute::NORMAL},
    {"normal_y", draco::GeometryAttribute::NORMAL},
    {"normal_z", draco::GeometryAttribute::NORMAL},
  };
  // cppcheck-suppress unassignedVariable
  for (const auto & [key, type] : kTable) {
    if (name == key) {
      return type;
    }
  }
  return draco::GeometryAttribute::GENERIC;
}

std::optional<FieldMapping> map_field(const PointField & field)
{
  FieldMapping mapping;
  mapping.attribute_type = attribute_type_for(field.name);

  switch (field.datatype) {
    case PointFieldType::kInt8:
      mapping.data_type = draco::DT_INT8;
      break;
    case PointFieldType::kUint8:
      mapping.data_type = draco::DT_UINT8;
      break;
    case PointFieldType::kInt16:
      mapping.data_type = draco::DT_INT16;
      break;
    case PointFieldType::kUint16:
      mapping.data_type = draco::DT_UINT16;
      break;
    case PointFieldType::kInt32:
      mapping.data_type = draco::DT_INT32;
      break;
    case PointFieldType::kUint32:
      mapping.data_type = draco::DT_UINT32;
      break;
    case PointFieldType::kFloat32:
      mapping.data_type = draco::DT_FLOAT32;
      break;
    case PointFieldType::kFloat64:
      mapping.data_type = draco::DT_FLOAT64;
      break;
    default:
      return std::nullopt;
  }
  mapping.num_components = static_cast<int>(field.count);

  // rgb/rgba carry packed color channels in one float: reinterpret the bytes
  // as 8-bit (float32) or 16-bit (float64) channels, matching
  // draco_point_cloud_transport.
  if (field.name == "rgb" || field.name == "rgba") {
    if (field.datatype == PointFieldType::kFloat32) {
      mapping.data_type = draco::DT_UINT8;
      mapping.num_components = 4 * static_cast<int>(field.count);
    } else if (field.datatype == PointFieldType::kFloat64) {
      mapping.data_type = draco::DT_UINT16;
      mapping.num_components = 4 * static_cast<int>(field.count);
    }
  }

  mapping.value_bytes =
    draco::DataTypeLength(mapping.data_type) * static_cast<std::size_t>(mapping.num_components);
  if (mapping.value_bytes != datatype_size(field.datatype) * field.count) {
    return std::nullopt;  // unreachable for the mappings above; kept as a guard
  }
  return mapping;
}

int quantization_bits_for(const DracoEncodeConfig & config, draco::GeometryAttribute::Type type)
{
  switch (type) {
    case draco::GeometryAttribute::POSITION:
      return config.quantization_position;
    case draco::GeometryAttribute::NORMAL:
      return config.quantization_normal;
    case draco::GeometryAttribute::COLOR:
      return config.quantization_color;
    // No PointField name maps to TEX_COORD (see attribute_type_for), so tex
    // coords take the generic bit count if one ever appears.
    default:
      return config.quantization_generic;
  }
}

}  // namespace

DracoEncodeResult encode_draco(const PointCloud2 & cloud, const DracoEncodeConfig & config)
{
  DracoEncodeResult result;
  if (cloud.is_bigendian) {
    result.error = "big-endian clouds are not supported by the draco codec";
    return result;
  }

  // Inter-row padding is not representable: attributes are read
  // point-contiguously with stride point_step, so a padded organized cloud
  // would corrupt every row after the first. Reject instead of mis-encoding.
  // row_step of a single-row cloud is not layout (it is often stale in real
  // recordings), so it is not checked here; compress normalizes it to
  // width * point_step in the stored metadata.
  if (cloud.height > 1 && cloud.row_step != cloud.width * cloud.point_step) {
    result.error = "row_step != width * point_step with height > 1 (row padding is not supported)";
    return result;
  }

  // Validate the whole field layout up front — including for empty clouds, so
  // a layout that decode_draco would later reject never enters the bag.
  std::vector<FieldMapping> mappings;
  mappings.reserve(cloud.fields.size());
  for (const auto & field : cloud.fields) {
    const auto mapping = map_field(field);
    if (!mapping.has_value()) {
      result.error = "field '" + field.name + "' has an unsupported datatype";
      return result;
    }
    if (field.offset + mapping->value_bytes > cloud.point_step) {
      result.error = "field '" + field.name + "' extends past point_step";
      return result;
    }
    mappings.push_back(*mapping);
  }

  const std::uint64_t num_points = static_cast<std::uint64_t>(cloud.height) * cloud.width;
  if (
    num_points > 0 && cloud.data.size() < static_cast<std::size_t>(num_points) * cloud.point_step) {
    result.error = "point data is smaller than height * width * point_step";
    return result;
  }

  // Draco cannot encode an empty cloud; the empty case round-trips as an
  // empty buffer (see decode_draco).
  if (num_points == 0) {
    result.data = std::vector<std::byte>{};
    return result;
  }

  draco::PointCloudBuilder builder;
  builder.Start(static_cast<draco::PointIndex::ValueType>(num_points));
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    const int att_id = builder.AddAttribute(
      mappings[i].attribute_type, mappings[i].num_components, mappings[i].data_type);
    builder.SetAttributeValuesForAllPoints(
      att_id, cloud.data.data() + cloud.fields[i].offset, static_cast<int>(cloud.point_step));
  }

  std::unique_ptr<draco::PointCloud> draco_cloud = builder.Finalize(false);
  if (
    draco_cloud == nullptr ||
    draco_cloud->num_points() != static_cast<draco::PointIndex::ValueType>(num_points)) {
    result.error = "conversion from PointCloud2 to draco::PointCloud failed";
    return result;
  }
  // draco_point_cloud_transport records this flag on the cloud; keep it so its
  // subscriber reads bagwiz-written streams unchanged.
  auto metadata = std::make_unique<draco::GeometryMetadata>();
  metadata->AddEntryInt("deduplicate", 0);
  draco_cloud->AddMetadata(std::move(metadata));

  draco::Encoder encoder;
  encoder.SetSpeedOptions(config.encode_speed, config.decode_speed);
  // Sequential encoding keeps point order and count intact; the kd-tree
  // alternative compresses better but reorders points, which recorded
  // downstream consumers (e.g. deskew) rely on.
  encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);
  if (!config.lossless) {
    for (const auto & field : cloud.fields) {
      const auto type = attribute_type_for(field.name);
      encoder.SetAttributeQuantization(type, quantization_bits_for(config, type));
    }
  }

  draco::EncoderBuffer buffer;
  const draco::Status status = encoder.EncodePointCloudToBuffer(*draco_cloud, &buffer);
  if (!status.ok()) {
    result.error = "draco encoder failed: " + std::string(status.error_msg());
    return result;
  }

  const auto * bytes = reinterpret_cast<const std::byte *>(buffer.data());
  result.data = std::vector<std::byte>(bytes, bytes + buffer.size());
  return result;
}

DracoDecodeResult decode_draco(
  std::span<const std::byte> data, const std::vector<PointField> & fields, std::uint32_t point_step)
{
  DracoDecodeResult result;

  std::vector<FieldMapping> mappings;
  mappings.reserve(fields.size());
  for (const auto & field : fields) {
    const auto mapping = map_field(field);
    if (!mapping.has_value()) {
      result.error = "field '" + field.name + "' has an unsupported datatype";
      return result;
    }
    if (field.offset + mapping->value_bytes > point_step) {
      result.error = "field '" + field.name + "' extends past point_step";
      return result;
    }
    mappings.push_back(*mapping);
  }

  // The empty-bitstream convention encode_draco uses for 0-point clouds.
  if (data.empty()) {
    result.data = std::vector<std::byte>{};
    result.num_points = 0;
    return result;
  }

  draco::DecoderBuffer decoder_buffer;
  decoder_buffer.Init(reinterpret_cast<const char *>(data.data()), data.size());
  draco::Decoder decoder;
  auto status_or = decoder.DecodePointCloudFromBuffer(&decoder_buffer);
  if (!status_or.ok()) {
    result.error = "draco decoder failed: " + std::string(status_or.status().error_msg());
    return result;
  }
  const std::unique_ptr<draco::PointCloud> & draco_cloud = status_or.value();

  const std::uint32_t num_points = draco_cloud->num_points();
  if (draco_cloud->num_attributes() != static_cast<int>(fields.size())) {
    result.error = "draco stream attribute count does not match the stored field count";
    return result;
  }

  result.data =
    std::vector<std::byte>(static_cast<std::size_t>(num_points) * point_step, std::byte{0});
  auto & out = *result.data;
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const draco::PointAttribute * attr = draco_cloud->attribute(static_cast<int>(i));
    if (attr == nullptr) {
      result.error = "draco stream is missing attribute " + std::to_string(i);
      result.data.reset();
      return result;
    }
    if (
      attr->data_type() != mappings[i].data_type ||
      attr->num_components() != mappings[i].num_components) {
      result.error =
        "draco attribute for field '" + fields[i].name + "' does not match the stored field layout";
      result.data.reset();
      return result;
    }
    for (std::uint32_t p = 0; p < num_points; ++p) {
      const std::uint8_t * src = attr->GetAddressOfMappedIndex(draco::PointIndex(p));
      std::memcpy(
        out.data() + static_cast<std::size_t>(p) * point_step + fields[i].offset, src,
        mappings[i].value_bytes);
    }
  }
  result.num_points = num_points;
  return result;
}

bool cloud_has_non_finite(const PointCloud2 & cloud)
{
  // Walk row by row so padded organized clouds (row_step > width *
  // point_step) are read correctly; such clouds are rejected by encode_draco
  // anyway, but this scan runs before that rejection.
  for (const auto & field : cloud.fields) {
    if (field.datatype != PointFieldType::kFloat32 && field.datatype != PointFieldType::kFloat64) {
      continue;
    }
    const std::size_t elem = datatype_size(field.datatype);
    const std::size_t field_bytes = elem * field.count;
    if (field.offset + field_bytes > cloud.point_step) {
      return true;  // broken layout: treat as unsafe
    }
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      const std::size_t row_base = static_cast<std::size_t>(row) * cloud.row_step;
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const std::size_t base =
          row_base + static_cast<std::size_t>(col) * cloud.point_step + field.offset;
        if (base + field_bytes > cloud.data.size()) {
          return true;
        }
        for (std::uint32_t c = 0; c < field.count; ++c) {
          const std::byte * v = cloud.data.data() + base + c * elem;
          if (field.datatype == PointFieldType::kFloat32) {
            float f;
            std::memcpy(&f, v, sizeof(f));
            if (!std::isfinite(f)) {
              return true;
            }
          } else {
            double d;
            std::memcpy(&d, v, sizeof(d));
            if (!std::isfinite(d)) {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

}  // namespace bagwiz::core::pointcloud
