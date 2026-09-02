// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

float read_field(
  const PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset, PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case PointFieldType::kFloat32:
      return *reinterpret_cast<const float *>(base);
    case PointFieldType::kFloat64:
      return static_cast<float>(*reinterpret_cast<const double *>(base));
    case PointFieldType::kInt8:
      return static_cast<float>(*reinterpret_cast<const std::int8_t *>(base));
    case PointFieldType::kUint8:
      return static_cast<float>(*reinterpret_cast<const std::uint8_t *>(base));
    case PointFieldType::kInt16:
      return static_cast<float>(*reinterpret_cast<const std::int16_t *>(base));
    case PointFieldType::kUint16:
      return static_cast<float>(*reinterpret_cast<const std::uint16_t *>(base));
    case PointFieldType::kInt32:
      return static_cast<float>(*reinterpret_cast<const std::int32_t *>(base));
    case PointFieldType::kUint32:
      return static_cast<float>(*reinterpret_cast<const std::uint32_t *>(base));
  }
  return 0.0f;
}

}  // namespace

// The camera model one projection uses: the intrinsics of the rectified or
// raw image, and the lens distortion the raw path applies.
class CameraProjection
{
public:
  CameraProjection(const image::CameraInfo & camera_info, bool use_rectified)
  : fx_(use_rectified ? camera_info.p[0] : camera_info.k[0]),
    fy_(use_rectified ? camera_info.p[5] : camera_info.k[4]),
    cx_(use_rectified ? camera_info.p[2] : camera_info.k[2]),
    cy_(use_rectified ? camera_info.p[6] : camera_info.k[5]),
    // When projecting onto the raw image (use_rectified=false) apply the
    // camera's lens distortion so points land where the distorted image
    // actually shows them. The rectified path uses camera_info.p, which
    // already assumes a rectified image, so it stays a plain pinhole
    // projection; with no distortion coefficients the raw path also reduces
    // to a plain pinhole.
    apply_distortion_(!use_rectified && !camera_info.d.empty()),
    distortion_model_(
      apply_distortion_ ? image::select_distortion_model(camera_info.distortion_model)
                        : image::DistortionModel::kNone),
    d_(camera_info.d)
  {
  }

  [[nodiscard]] std::optional<ImagePoint> project(
    double x, double y, double z, std::uint32_t image_width, std::uint32_t image_height,
    bool require_inside = true) const
  {
    if (z <= 0.0) {
      return std::nullopt;
    }
    double nx = x / z;
    double ny = y / z;
    if (apply_distortion_) {
      // Skip distortion fold-back artifacts: outside the model's valid domain
      // the forward distortion is non-injective, so far-off-axis points
      // (beyond the camera FOV) fold back into the image and must not be
      // drawn.
      const auto distorted = image::distort_for_raw_image(nx, ny, distortion_model_, d_, fx_, fy_);
      if (!distorted) {
        return std::nullopt;
      }
      nx = distorted->x;
      ny = distorted->y;
    }
    ImagePoint p;
    p.u = fx_ * nx + cx_;
    p.v = fy_ * ny + cy_;
    p.depth = z;
    if (require_inside && (p.u < 0.0 || p.u >= image_width || p.v < 0.0 || p.v >= image_height)) {
      return std::nullopt;
    }
    return p;
  }

private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  bool apply_distortion_;
  image::DistortionModel distortion_model_;
  std::vector<double> d_;  // a copy: the projection outlives no CameraInfo by accident
};

std::optional<ImagePoint> project_camera_point(
  double x, double y, double z, const image::CameraInfo & camera_info, std::uint32_t image_width,
  std::uint32_t image_height, bool use_rectified, bool require_inside)
{
  return CameraProjection(camera_info, use_rectified)
    .project(x, y, z, image_width, image_height, require_inside);
}

ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  PointCloudProperty property, bool use_rectified)
{
  ProjectionResult result;

  const auto off_x = cloud.field_offset("x");
  const auto off_y = cloud.field_offset("y");
  const auto off_z = cloud.field_offset("z");
  if (!off_x || !off_y || !off_z) {
    result.error = "point cloud is missing required x/y/z fields";
    return result;
  }

  const auto find_field = [&](const std::string & name) -> const PointField * {
    for (const auto & f : cloud.fields) {
      if (f.name == name) {
        return &f;
      }
    }
    return nullptr;
  };

  const PointField * field_x = find_field("x");
  const PointField * field_y = find_field("y");
  const PointField * field_z = find_field("z");

  const bool need_intensity = (property == PointCloudProperty::kIntensity);
  std::optional<std::uint32_t> off_intensity;
  const PointField * field_intensity = nullptr;
  if (need_intensity) {
    off_intensity = cloud.field_offset("intensity");
    if (!off_intensity) {
      result.error = "point cloud has no intensity field";
      return result;
    }
    field_intensity = find_field("intensity");
  }

  const CameraProjection camera(camera_info, use_rectified);

  // Clamp to the points the blob actually holds: a cloud whose data is
  // shorter than height * width * point_step (a stale header or a truncated
  // blob) projects its complete points instead of reading out of bounds.
  const std::uint64_t declared = static_cast<std::uint64_t>(cloud.height) * cloud.width;
  const std::uint64_t held = cloud.point_step != 0 ? cloud.data.size() / cloud.point_step : 0;
  const auto n = static_cast<std::uint32_t>(std::min(declared, held));
  result.points.reserve(n / 4);  // rough estimate

  for (std::uint32_t i = 0; i < n; ++i) {
    const float px = read_field(cloud, i, *off_x, field_x->datatype);
    const float py = read_field(cloud, i, *off_y, field_y->datatype);
    const float pz = read_field(cloud, i, *off_z, field_z->datatype);

    // Apply 4x4 transform: assume column-major storage matching tf2::StampedTransform.
    const double tx = transform[0] * px + transform[4] * py + transform[8] * pz + transform[12];
    const double ty = transform[1] * px + transform[5] * py + transform[9] * pz + transform[13];
    const double tz = transform[2] * px + transform[6] * py + transform[10] * pz + transform[14];

    const auto pixel = camera.project(tx, ty, tz, image_width, image_height);
    if (!pixel.has_value()) {
      continue;
    }
    const double u = pixel->u;
    const double v = pixel->v;

    float value = 0.0f;
    switch (property) {
      case PointCloudProperty::kX:
        value = px;
        break;
      case PointCloudProperty::kY:
        value = py;
        break;
      case PointCloudProperty::kZ:
        value = pz;
        break;
      case PointCloudProperty::kDistance:
        value = std::sqrt(px * px + py * py + pz * pz);
        break;
      case PointCloudProperty::kIntensity:
        value = read_field(cloud, i, *off_intensity, field_intensity->datatype);
        break;
    }

    ProjectedPoint pp;
    pp.u = static_cast<std::int32_t>(u);
    pp.v = static_cast<std::int32_t>(v);
    pp.depth = static_cast<float>(tz);
    pp.value = value;
    result.points.push_back(pp);
  }

  return result;
}

}  // namespace bagwiz::core::pointcloud
