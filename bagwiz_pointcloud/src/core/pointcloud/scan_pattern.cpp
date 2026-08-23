// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/scan_pattern.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

// Degrees -> radians.
double rad(double deg)
{
  return deg * M_PI / 180.0;
}

// Validate the point layout shared by every per-point walk here: the time
// field must fit in a point, and the data blob must cover height * width
// points at the declared strides. Returns "" on success.
std::string check_layout(const PointCloud2 & cloud)
{
  if (cloud.point_step == 0) {
    return "point cloud has point_step 0";
  }
  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  if (
    static_cast<std::size_t>(cloud.width) * cloud.point_step > rstep ||
    cloud.data.size() < static_cast<std::size_t>(cloud.height) * rstep) {
    return "point cloud data is smaller than height * width * point_step";
  }
  return "";
}

// First byte of point `i` (linear index over height * width) in the blob.
const std::byte * point_base(const PointCloud2 & cloud, std::uint32_t i)
{
  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  const std::uint32_t row = cloud.width != 0 ? i / cloud.width : 0;
  const std::uint32_t col = cloud.width != 0 ? i % cloud.width : 0;
  return cloud.data.data() + static_cast<std::size_t>(row) * rstep +
         static_cast<std::size_t>(col) * cloud.point_step;
}

// Read one numeric field of point `i` as a double. Mirrors the datatype
// coverage of projector.cpp's read_field.
double read_coord(const PointCloud2 & cloud, std::uint32_t i, const PointField & field)
{
  const std::byte * base = point_base(cloud, i) + field.offset;
  switch (field.datatype) {
    case PointFieldType::kFloat32: {
      float v = 0.0F;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kFloat64: {
      double v = 0.0;
      std::memcpy(&v, base, sizeof(v));
      return v;
    }
    case PointFieldType::kInt8: {
      std::int8_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint8: {
      std::uint8_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kInt16: {
      std::int16_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint16: {
      std::uint16_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kInt32: {
      std::int32_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
  }
  return 0.0;  // unreachable: PointFieldType is exhaustive
}

const PointField * find_field(const PointCloud2 & cloud, const char * name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

// Camera frame of the perspective view: position on the azim/elev/dist sphere
// around the origin, forward toward the origin, up as close to +z as the
// geometry allows. Writes the orthonormal basis (right, up, forward) and the
// camera position into `basis` / `pos` (each 3 doubles, x/y/z).
void perspective_camera(const ScanPatternView & view, double (&basis)[9], double (&pos)[3])
{
  const double az = rad(view.azim_deg);
  const double el = rad(view.elev_deg);
  pos[0] = view.dist_m * std::cos(el) * std::cos(az);
  pos[1] = view.dist_m * std::cos(el) * std::sin(az);
  pos[2] = view.dist_m * std::sin(el);

  double fwd[3] = {-pos[0], -pos[1], -pos[2]};
  const double flen = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
  for (auto & c : fwd) {
    c /= flen;
  }

  // World up degenerates when the camera looks straight down the z axis;
  // fall back to +y there.
  const double up_ref[3] = {0.0, 0.0, 1.0};
  const double up_alt[3] = {0.0, 1.0, 0.0};
  const double * up_w = std::abs(fwd[2]) > 0.999 ? up_alt : up_ref;

  // right = normalize(forward x up_w); up = right x forward.
  double right[3] = {
    fwd[1] * up_w[2] - fwd[2] * up_w[1], fwd[2] * up_w[0] - fwd[0] * up_w[2],
    fwd[0] * up_w[1] - fwd[1] * up_w[0]};
  const double rlen = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
  for (auto & c : right) {
    c /= rlen;
  }
  const double up[3] = {
    right[1] * fwd[2] - right[2] * fwd[1], right[2] * fwd[0] - right[0] * fwd[2],
    right[0] * fwd[1] - right[1] * fwd[0]};

  std::memcpy(&basis[0], right, 3 * sizeof(double));
  std::memcpy(&basis[3], up, 3 * sizeof(double));
  std::memcpy(&basis[6], fwd, 3 * sizeof(double));
}

}  // namespace

ScanTimesResult extract_scan_times(const PointCloud2 & cloud, const PointTimeField & field)
{
  ScanTimesResult result;
  if (const std::string err = check_layout(cloud); !err.empty()) {
    result.error = err;
    return result;
  }
  if (static_cast<std::size_t>(field.offset) + datatype_size(field.datatype) > cloud.point_step) {
    result.error = "per-point time field is declared out of bounds";
    return result;
  }

  const std::size_t n = static_cast<std::size_t>(cloud.height) * cloud.width;
  result.times.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    const std::byte * b = point_base(cloud, static_cast<std::uint32_t>(i)) + field.offset;
    result.times[i] = point_time_seconds(b, field);
  }
  return result;
}

std::vector<std::uint32_t> sorted_scan_indices(std::span<const double> times)
{
  std::vector<std::uint32_t> order;
  order.reserve(times.size());
  for (std::uint32_t i = 0; i < times.size(); ++i) {
    if (std::isfinite(times[i])) {
      order.push_back(i);
    }
  }
  std::sort(order.begin(), order.end(), [&times](std::uint32_t a, std::uint32_t b) {
    return times[a] < times[b];
  });
  return order;
}

ScanProjectionResult project_scan_points(
  const PointCloud2 & cloud, const ScanPatternView & view, std::span<const std::uint32_t> indices,
  std::span<const double> times, double t_min)
{
  ScanProjectionResult result;

  const PointField * field_x = find_field(cloud, "x");
  const PointField * field_y = find_field(cloud, "y");
  const PointField * field_z = find_field(cloud, "z");
  if (field_x == nullptr || field_y == nullptr || field_z == nullptr) {
    result.error = "point cloud is missing required x/y/z fields";
    return result;
  }
  if (const std::string err = check_layout(cloud); !err.empty()) {
    result.error = err;
    return result;
  }

  result.points.reserve(indices.size());
  for (const std::uint32_t i : indices) {
    const double x = read_coord(cloud, i, *field_x);
    const double y = read_coord(cloud, i, *field_y);
    const double z = read_coord(cloud, i, *field_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    std::optional<ProjectedPoint> projected;
    if (view.projection == ScanPatternProjection::kBev) {
      projected = project_bev(x, y, view);
    } else {
      projected = project_perspective(x, y, z, view);
    }
    if (!projected.has_value()) {
      continue;
    }
    projected->value = static_cast<float>(times[i] - t_min);
    result.points.push_back(*projected);
  }
  return result;
}

ProjectedPoint project_bev(double x, double y, const ScanPatternView & view)
{
  // The view is square in meters on both axes; scale by the smaller canvas
  // dimension so a meter is a meter in both directions.
  const double scale = (std::min(view.width, view.height) / 2.0) / view.range_m;
  ProjectedPoint p;
  p.u = static_cast<std::int32_t>(std::lround(view.width / 2.0 - y * scale));
  p.v = static_cast<std::int32_t>(std::lround(view.height / 2.0 - x * scale));
  p.depth = 0.0F;
  return p;
}

std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const ScanPatternView & view)
{
  double basis[9];
  double pos[3];
  perspective_camera(view, basis, pos);

  const double d[3] = {x - pos[0], y - pos[1], z - pos[2]};
  const double z_cam = d[0] * basis[6] + d[1] * basis[7] + d[2] * basis[8];
  if (z_cam <= 1.0e-6) {
    return std::nullopt;
  }
  const double x_cam = d[0] * basis[0] + d[1] * basis[1] + d[2] * basis[2];
  const double y_cam = d[0] * basis[3] + d[1] * basis[4] + d[2] * basis[5];

  const double focal = (view.height / 2.0) / std::tan(rad(kScanPatternFovDeg) / 2.0);
  ProjectedPoint p;
  p.u = static_cast<std::int32_t>(std::lround(focal * x_cam / z_cam + view.width / 2.0));
  p.v = static_cast<std::int32_t>(std::lround(view.height / 2.0 - focal * y_cam / z_cam));
  p.depth = static_cast<float>(z_cam);
  return p;
}

}  // namespace bagwiz::core::pointcloud
