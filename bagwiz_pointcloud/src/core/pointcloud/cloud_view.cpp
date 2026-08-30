// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
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

// Validate the point layout shared by every per-point walk here: the data
// blob must cover height * width points at the declared strides. Returns ""
// on success.
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

// The field named `name` when it fits inside a point, else nullptr.
const PointField * find_field(const PointCloud2 & cloud, const char * name)
{
  for (const auto & f : cloud.fields) {
    if (
      f.name == name &&
      static_cast<std::size_t>(f.offset) + datatype_size(f.datatype) <= cloud.point_step) {
      return &f;
    }
  }
  return nullptr;
}

// Read one numeric field of the point starting at `base` as a double,
// through memcpy so an unaligned field is read safely.
double read_field(const std::byte * base, const PointField & field)
{
  const std::byte * p = base + field.offset;
  switch (field.datatype) {
    case PointFieldType::kFloat32: {
      float v = 0.0F;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kFloat64: {
      double v = 0.0;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointFieldType::kInt8: {
      std::int8_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint8: {
      std::uint8_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kInt16: {
      std::int16_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint16: {
      std::uint16_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kInt32: {
      std::int32_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(&v, p, sizeof(v));
      return static_cast<double>(v);
    }
  }
  return 0.0;  // unreachable: PointFieldType is exhaustive
}

}  // namespace

ProjectedPoint project_bev(double x, double y, const CloudView & view)
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

PerspectiveCamera make_perspective_camera(const CloudView & view)
{
  // Position on the azim/elev/dist sphere around the origin, forward toward
  // the origin, up as close to +z as the geometry allows.
  PerspectiveCamera cam;
  const double az = rad(view.azim_deg);
  const double el = rad(view.elev_deg);
  cam.position = {
    view.dist_m * std::cos(el) * std::cos(az), view.dist_m * std::cos(el) * std::sin(az),
    view.dist_m * std::sin(el)};

  std::array<double, 3> fwd = {-cam.position[0], -cam.position[1], -cam.position[2]};
  const double flen = std::sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
  for (auto & c : fwd) {
    c /= flen;
  }

  // World up degenerates when the camera looks straight down the z axis;
  // fall back to +y there.
  const std::array<double, 3> up_ref = {0.0, 0.0, 1.0};
  const std::array<double, 3> up_alt = {0.0, 1.0, 0.0};
  const std::array<double, 3> & up_w = std::abs(fwd[2]) > 0.999 ? up_alt : up_ref;

  // right = normalize(forward x up_w); up = right x forward.
  std::array<double, 3> right = {
    fwd[1] * up_w[2] - fwd[2] * up_w[1], fwd[2] * up_w[0] - fwd[0] * up_w[2],
    fwd[0] * up_w[1] - fwd[1] * up_w[0]};
  const double rlen = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
  for (auto & c : right) {
    c /= rlen;
  }
  cam.right = right;
  cam.up = {
    right[1] * fwd[2] - right[2] * fwd[1], right[2] * fwd[0] - right[0] * fwd[2],
    right[0] * fwd[1] - right[1] * fwd[0]};
  cam.forward = fwd;
  cam.focal = (view.height / 2.0) / std::tan(rad(kCloudViewFovDeg) / 2.0);
  return cam;
}

std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const PerspectiveCamera & camera, const CloudView & view)
{
  const std::array<double, 3> d = {
    x - camera.position[0], y - camera.position[1], z - camera.position[2]};
  const double z_cam =
    d[0] * camera.forward[0] + d[1] * camera.forward[1] + d[2] * camera.forward[2];
  if (z_cam <= 1.0e-6) {
    return std::nullopt;
  }
  const double x_cam = d[0] * camera.right[0] + d[1] * camera.right[1] + d[2] * camera.right[2];
  const double y_cam = d[0] * camera.up[0] + d[1] * camera.up[1] + d[2] * camera.up[2];

  ProjectedPoint p;
  p.u = static_cast<std::int32_t>(std::lround(camera.focal * x_cam / z_cam + view.width / 2.0));
  p.v = static_cast<std::int32_t>(std::lround(view.height / 2.0 - camera.focal * y_cam / z_cam));
  p.depth = static_cast<float>(z_cam);
  return p;
}

std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const CloudView & view)
{
  return project_perspective(x, y, z, make_perspective_camera(view), view);
}

std::optional<double> bev_auto_range(
  const PointCloud2 & cloud, double fraction, std::string & error)
{
  if (!(fraction > 0.0) || fraction > 1.0) {
    error = "the auto-range fraction must be in (0, 1]";
    return std::nullopt;
  }
  const PointField * field_x = find_field(cloud, "x");
  const PointField * field_y = find_field(cloud, "y");
  if (field_x == nullptr || field_y == nullptr) {
    error = "point cloud is missing required x/y fields";
    return std::nullopt;
  }
  if (const std::string err = check_layout(cloud); !err.empty()) {
    error = err;
    return std::nullopt;
  }

  std::vector<double> distances;
  distances.reserve(static_cast<std::size_t>(cloud.height) * cloud.width);
  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::byte * row_base = cloud.data.data() + static_cast<std::size_t>(row) * rstep;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::byte * base = row_base + static_cast<std::size_t>(col) * cloud.point_step;
      const double x = read_field(base, *field_x);
      const double y = read_field(base, *field_y);
      if (std::isfinite(x) && std::isfinite(y)) {
        distances.push_back(std::hypot(x, y));
      }
    }
  }
  if (distances.empty()) {
    error = "point cloud has no finite points";
    return std::nullopt;
  }
  // Nearest rank: the smallest distance at or below which `fraction` of the
  // points fall.
  const auto rank =
    static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(distances.size())));
  const std::size_t index = std::clamp<std::size_t>(rank, 1, distances.size()) - 1;
  std::nth_element(
    distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(index), distances.end());
  const double range = distances[index];
  if (!(range > 0.0)) {
    error = "point cloud's points all sit at the origin";
    return std::nullopt;
  }
  return range;
}

CloudViewProjection project_cloud_to_view(
  const PointCloud2 & cloud, const RigidTransform & transform, const CloudView & view,
  PointCloudProperty property)
{
  CloudViewProjection result;

  const PointField * field_x = find_field(cloud, "x");
  const PointField * field_y = find_field(cloud, "y");
  const PointField * field_z = find_field(cloud, "z");
  if (field_x == nullptr || field_y == nullptr || field_z == nullptr) {
    result.error = "point cloud is missing required x/y/z fields";
    return result;
  }
  const PointField * field_intensity = nullptr;
  if (property == PointCloudProperty::kIntensity) {
    field_intensity = find_field(cloud, "intensity");
    if (field_intensity == nullptr) {
      result.error = "point cloud has no intensity field";
      return result;
    }
  }
  if (const std::string err = check_layout(cloud); !err.empty()) {
    result.error = err;
    return result;
  }

  const bool perspective = view.projection == CloudProjection::kPerspective;
  const PerspectiveCamera camera =
    perspective ? make_perspective_camera(view) : PerspectiveCamera{};
  const bool identity = transform.is_identity();
  const auto & r = transform.rotation;
  const auto & t = transform.translation;

  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  result.points.reserve(static_cast<std::size_t>(cloud.height) * cloud.width / 4);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::byte * row_base = cloud.data.data() + static_cast<std::size_t>(row) * rstep;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::byte * base = row_base + static_cast<std::size_t>(col) * cloud.point_step;
      const double x = read_field(base, *field_x);
      const double y = read_field(base, *field_y);
      const double z = read_field(base, *field_z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      // The point in the view frame.
      double vx = x;
      double vy = y;
      double vz = z;
      if (!identity) {
        vx = r[0] * x + r[1] * y + r[2] * z + t[0];
        vy = r[3] * x + r[4] * y + r[5] * z + t[1];
        vz = r[6] * x + r[7] * y + r[8] * z + t[2];
      }
      std::optional<ProjectedPoint> projected;
      if (perspective) {
        projected = project_perspective(vx, vy, vz, camera, view);
      } else {
        projected = project_bev(vx, vy, view);
        projected->depth = static_cast<float>(-vz);
      }
      if (
        !projected.has_value() || projected->u < 0 ||
        projected->u >= static_cast<std::int32_t>(view.width) || projected->v < 0 ||
        projected->v >= static_cast<std::int32_t>(view.height)) {
        continue;
      }
      double value = 0.0;
      switch (property) {
        case PointCloudProperty::kX:
          value = x;
          break;
        case PointCloudProperty::kY:
          value = y;
          break;
        case PointCloudProperty::kZ:
          value = z;
          break;
        case PointCloudProperty::kDistance:
          value = std::sqrt(x * x + y * y + z * z);
          break;
        case PointCloudProperty::kIntensity:
          value = read_field(base, *field_intensity);
          break;
      }
      projected->value = static_cast<float>(value);
      result.points.push_back(*projected);
    }
  }
  return result;
}

}  // namespace bagwiz::core::pointcloud
