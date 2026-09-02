// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_
#define BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_

#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// The 2D views a point cloud is rendered into when no camera image is
// involved: a top-down bird's-eye view (BEV) of the view frame's XY plane, or
// a perspective view from a virtual camera on a sphere around the view
// frame's origin. Both map one point of the view frame to a canvas pixel (a
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

// Project one point of the view frame to a canvas pixel (BEV): image up = +x
// (forward), image left = +y. The result is not clipped to the canvas —
// callers clip at draw time. `depth` is 0 and `value` is left at 0 for the
// caller to fill.
[[nodiscard]] ProjectedPoint project_bev(double x, double y, const CloudView & view);

// The perspective camera of a CloudView, worked out once per view so a whole
// cloud projects without repeating the trigonometry per point: an
// orthonormal basis (right, up, forward, each a unit vector of the view
// frame), the camera position, and the focal length in pixels.
struct PerspectiveCamera
{
  std::array<double, 3> right{};
  std::array<double, 3> up{};
  std::array<double, 3> forward{};
  std::array<double, 3> position{};
  double focal = 0.0;
};

// The camera of `view`'s perspective projection (elev / azim / dist and the
// canvas height; the projection kind is not consulted).
[[nodiscard]] PerspectiveCamera make_perspective_camera(const CloudView & view);

// Project one point of the view frame through `camera`. Returns nullopt when
// the point sits at or behind the camera. The result is not clipped to the
// canvas; `depth` is the camera-space depth in meters and `value` is left at
// 0 for the caller to fill.
[[nodiscard]] std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const PerspectiveCamera & camera, const CloudView & view);

// project_perspective with the camera derived from `view` on each call; the
// overload above is the one to use over a whole cloud.
[[nodiscard]] std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const CloudView & view);

// Outcome of project_cloud_to_view(). `error` is empty on success.
struct CloudViewProjection
{
  std::vector<ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// The share of a cloud's points a BEV auto-range keeps inside the view: the
// half-extent is the 95th percentile of the points' ground distances, so a
// handful of far returns does not shrink the scene to a dot.
inline constexpr double kBevAutoRangeQuantile = 0.95;

// The BEV half-extent that keeps `fraction` (0 < fraction <= 1) of the
// cloud's finite points inside the view: the nearest-rank quantile of their
// ground (XY) distances. Only the points the blob actually holds are
// considered: a short blob or a stale row_step trims the walk to the
// complete points instead of failing. nullopt with `error` set when the blob
// cannot hold one complete point, no point is finite, or every point sits at
// the origin.
[[nodiscard]] std::optional<double> bev_auto_range(
  const PointCloud2 & cloud, double fraction, std::string & error);

// Project the finite points of `cloud` onto `view`: each point is moved by
// `transform` (the pose of the cloud's frame in the view frame, applied as
// p' = R p + t) and projected; points whose center lands outside the canvas
// are dropped. Each output point's `value` is the point's `property`
// (distance and the coordinates in the cloud's own frame, before the
// transform); its `depth` is the camera-space depth of the perspective view
// or, for the BEV, the negated height in the view frame — so a depth test
// that keeps the smallest depth shows the point nearest the camera, or the
// highest point of the bird's-eye view. Only the points the blob actually
// holds are projected: a short blob or a stale row_step trims the walk to
// the complete points instead of failing. Fails when the cloud lacks x/y/z
// fields (or intensity, when that is the property), or its blob cannot hold
// even one complete point.
[[nodiscard]] CloudViewProjection project_cloud_to_view(
  const PointCloud2 & cloud, const RigidTransform & transform, const CloudView & view,
  PointCloudProperty property);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__CLOUD_VIEW_HPP_
