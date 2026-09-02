// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

struct ProjectedPoint
{
  std::int32_t u = 0;
  std::int32_t v = 0;
  float depth = 0.0f;
  float value = 0.0f;
};

struct ProjectionResult
{
  std::vector<ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// A point projected onto the image, in pixels (sub-pixel; the image's
// top-left is 0,0), with its depth along the camera's optical axis.
struct ImagePoint
{
  double u = 0.0;
  double v = 0.0;
  double depth = 0.0;
};

// Project one point given in the camera frame (x right, y down, z forward)
// onto the image: a pinhole through `camera_info.p` when `use_rectified`,
// else through `camera_info.k` with the lens distortion applied so the point
// lands where the raw image shows it. nullopt for a point behind the camera,
// outside the distortion model's valid domain, or — unless `require_inside`
// is false (a polygon corner the drawing clips itself) — off the image.
[[nodiscard]] std::optional<ImagePoint> project_camera_point(
  double x, double y, double z, const image::CameraInfo & camera_info, std::uint32_t image_width,
  std::uint32_t image_height, bool use_rectified, bool require_inside = true);

// Project a point cloud onto an image.
// When `use_rectified` is true, the projection uses `camera_info.p` (the rectified projection
// matrix) instead of `camera_info.k`. This should be used when the target image has been
// rectified so the projected points align with the rectified image. Only the points the blob
// actually holds are projected: a blob shorter than height * width * point_step trims the
// walk to the complete points instead of reading out of bounds.
[[nodiscard]] ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  PointCloudProperty property, bool use_rectified = false);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
