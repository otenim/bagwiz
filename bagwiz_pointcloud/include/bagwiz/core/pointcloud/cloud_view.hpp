// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_
#define BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_

#include "bagwiz/core/pointcloud/projector.hpp"

#include <cstdint>
#include <optional>

// The 2D views a point cloud is rendered into when no camera image is
// involved: a top-down bird's-eye view (BEV) of the sensor's XY plane, or a
// perspective view from a virtual camera on a sphere around the sensor
// origin. Both map one sensor-frame point to a canvas pixel (a
// ProjectedPoint); colors and rasterization are left to the caller.
namespace bagwiz::core::pointcloud
{

// Which projection a CloudView applies.
enum class CloudProjection {
  kBev,          // top-down XY view centered on the origin
  kPerspective,  // pinhole view from a virtual camera looking at the origin
};

struct CloudView
{
  CloudProjection projection = CloudProjection::kBev;
  std::uint32_t width = 0;   // canvas width in pixels
  std::uint32_t height = 0;  // canvas height in pixels
  // BEV: half-extent of the view in meters — the canvas spans
  // [-range_m, +range_m] on both x (forward) and y (left).
  double range_m = 0.0;
  // Perspective only: camera position on a sphere of radius dist_m around the
  // origin (the sensor), looking at the origin. azim_deg is measured from the
  // +x axis around +z (azim 180 looks at the scene from behind the sensor),
  // elev_deg is the angle above the XY plane.
  double elev_deg = 30.0;
  double azim_deg = 180.0;
  double dist_m = 0.0;
};

// Vertical field of view of the perspective camera, in degrees. Fixed so the
// view has no zoom parameter; framing is controlled by dist_m alone.
inline constexpr double kCloudViewFovDeg = 60.0;

// Project one sensor-frame point to a canvas pixel (BEV): image up = +x
// (forward), image left = +y. The result is not clipped to the canvas —
// callers clip at draw time. `depth` is 0 and `value` is left at 0 for the
// caller to fill.
[[nodiscard]] ProjectedPoint project_bev(double x, double y, const CloudView & view);

// Project one sensor-frame point through the perspective camera. Returns
// nullopt when the point sits at or behind the camera. The result is not
// clipped to the canvas; `depth` is the camera-space depth in meters and
// `value` is left at 0 for the caller to fill.
[[nodiscard]] std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const CloudView & view);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_
