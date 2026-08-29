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
#include <cstring>

namespace bagwiz::core::pointcloud
{

namespace
{

// Degrees -> radians.
double rad(double deg)
{
  return deg * M_PI / 180.0;
}

// Camera frame of the perspective view: position on the azim/elev/dist sphere
// around the origin, forward toward the origin, up as close to +z as the
// geometry allows. Writes the orthonormal basis (right, up, forward) and the
// camera position into `basis` / `pos` (each 3 doubles, x/y/z).
void perspective_camera(const CloudView & view, double (&basis)[9], double (&pos)[3])
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

std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const CloudView & view)
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

  const double focal = (view.height / 2.0) / std::tan(rad(kCloudViewFovDeg) / 2.0);
  ProjectedPoint p;
  p.u = static_cast<std::int32_t>(std::lround(focal * x_cam / z_cam + view.width / 2.0));
  p.v = static_cast<std::int32_t>(std::lround(view.height / 2.0 - focal * y_cam / z_cam));
  p.depth = static_cast<float>(z_cam);
  return p;
}

}  // namespace bagwiz::core::pointcloud
