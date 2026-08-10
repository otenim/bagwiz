// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace bagwiz::core::pointcloud
{

namespace
{

// Per-point time field names in glim's precedence order.
constexpr std::array<const char *, 4> kTimeFieldNames{"t", "time", "time_stamp", "timestamp"};

bool is_supported(PointFieldType datatype)
{
  return datatype == PointFieldType::kUint32 || datatype == PointFieldType::kFloat32 ||
         datatype == PointFieldType::kFloat64;
}

}  // namespace

std::optional<PointTimeField> find_point_time_field(const PointCloud2 & cloud)
{
  for (const auto * const name : kTimeFieldNames) {
    for (const auto & f : cloud.fields) {
      if (f.name == name && f.count == 1 && is_supported(f.datatype)) {
        return PointTimeField{f.offset, f.datatype};
      }
    }
  }
  return std::nullopt;
}

double point_time_seconds(const std::byte * field_bytes, const PointTimeField & field)
{
  switch (field.datatype) {
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(&v, field_bytes, sizeof(v));
      return static_cast<double>(v) / 1e9;  // nanoseconds -> seconds
    }
    case PointFieldType::kFloat32: {
      float v = 0.0F;
      std::memcpy(&v, field_bytes, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kFloat64: {
      double v = 0.0;
      std::memcpy(&v, field_bytes, sizeof(v));
      return v;
    }
    default:
      return 0.0;  // unreachable: find_point_time_field only returns supported datatypes
  }
}

std::optional<PointTimeSpan> absolute_point_time_span_ns(
  const PointCloud2 & cloud, const PointTimeField & field, std::int64_t t_ref_ns)
{
  if (
    cloud.point_step == 0 ||
    static_cast<std::size_t>(field.offset) + datatype_size(field.datatype) > cloud.point_step) {
    return std::nullopt;
  }
  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  if (
    static_cast<std::size_t>(cloud.width) * cloud.point_step > rstep ||
    cloud.data.size() < static_cast<std::size_t>(cloud.height) * rstep) {
    return std::nullopt;
  }

  // Classify relative vs absolute exactly like deskew_pointcloud2: one scan
  // for the largest magnitude, then the threshold shared with it.
  double max_abs_sec = 0.0;
  bool any_finite = false;
  for (std::uint32_t r = 0; r < cloud.height; ++r) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::byte * b = cloud.data.data() + static_cast<std::size_t>(r) * rstep +
                            static_cast<std::size_t>(col) * cloud.point_step + field.offset;
      const double s = point_time_seconds(b, field);
      if (std::isfinite(s)) {
        any_finite = true;
        max_abs_sec = std::max(max_abs_sec, std::abs(s));
      }
    }
  }
  if (!any_finite) {
    return std::nullopt;
  }
  const bool relative = max_abs_sec < kRelativeTimeThresholdSec;

  PointTimeSpan span;
  bool first = true;
  for (std::uint32_t r = 0; r < cloud.height; ++r) {
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::byte * b = cloud.data.data() + static_cast<std::size_t>(r) * rstep +
                            static_cast<std::size_t>(col) * cloud.point_step + field.offset;
      const double s = point_time_seconds(b, field);
      if (!std::isfinite(s)) {
        continue;
      }
      const std::int64_t t_ns = relative
                                  ? t_ref_ns + static_cast<std::int64_t>(std::llround(s * 1.0e9))
                                  : static_cast<std::int64_t>(std::llround(s * 1.0e9));
      if (first) {
        span.min_ns = t_ns;
        span.max_ns = t_ns;
        first = false;
      } else {
        span.min_ns = std::min(span.min_ns, t_ns);
        span.max_ns = std::max(span.max_ns, t_ns);
      }
    }
  }
  return span;
}

}  // namespace bagwiz::core::pointcloud
