// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__SCAN_PATTERN_HPP_
#define BAGWIZ__CORE__POINTCLOUD__SCAN_PATTERN_HPP_

#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

// Rendering core behind `bagwiz generate video scan`: turns one point
// cloud into a firing-order animation by (1) reading every point's per-point
// time, (2) sorting points by that time, and (3) projecting a cumulative
// prefix of the sorted points onto a 2D canvas — either a top-down BEV view
// or a perspective view from a fixed camera on a sphere around the sensor.
// Colors are left to the caller: each ProjectedPoint carries the point's
// sweep-relative time in `value`, which overlay_projected_points maps through
// a ColorScheme.
namespace bagwiz::core::pointcloud
{

// Which projection the scan-pattern renderer uses.
enum class ScanPatternProjection {
  kBev,          // top-down XY view centered on the sensor
  kPerspective,  // pinhole view from a fixed camera looking at the sensor
};

struct ScanPatternView
{
  ScanPatternProjection projection = ScanPatternProjection::kBev;
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
inline constexpr double kScanPatternFovDeg = 60.0;

// Outcome of extract_scan_times(). `times` is parallel to the cloud's points
// (height * width entries); non-finite per-point times (NaN or +-inf) are kept
// as-is so the entry stays aligned with its point. `error` is empty on
// success.
struct ScanTimesResult
{
  std::vector<double> times;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Read every point's per-point time as seconds (see point_time_seconds for the
// datatype handling). Whether the values are sweep-relative or epoch-absolute
// is irrelevant to callers: the animation normalizes by the cloud's own
// min/max, so no relative/absolute classification is applied here. Fails when
// the field is declared out of bounds or the point layout is inconsistent.
[[nodiscard]] ScanTimesResult extract_scan_times(
  const PointCloud2 & cloud, const PointTimeField & field);

// Indices into the cloud's points, sorted by ascending time. Entries whose
// time is not finite (NaN or +-inf) are excluded — those points never appear
// in the animation.
[[nodiscard]] std::vector<std::uint32_t> sorted_scan_indices(std::span<const double> times);

// Outcome of project_scan_points(). Points with non-finite coordinates, and
// points at or behind the perspective camera, are skipped silently.
// Projections outside the canvas are kept; overlay_projected_points clips
// them at draw time. `error` is empty on success.
struct ScanProjectionResult
{
  std::vector<ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Project the cloud points listed in `indices` onto the view. Each output
// point's `value` is the point's sweep-relative time (times[i] - t_min), ready
// for overlay_projected_points' property_min/property_max = [0, sweep span];
// its `depth` is the perspective camera-space depth (0 for BEV). Fails when
// the cloud lacks x/y/z fields or the point layout is inconsistent.
[[nodiscard]] ScanProjectionResult project_scan_points(
  const PointCloud2 & cloud, const ScanPatternView & view, std::span<const std::uint32_t> indices,
  std::span<const double> times, double t_min);

// Project one sensor-frame point to a canvas pixel (BEV): image up = +x
// (forward), image left = +y. The result is not clipped to the canvas —
// overlay_projected_points clips at draw time.
[[nodiscard]] ProjectedPoint project_bev(double x, double y, const ScanPatternView & view);

// Project one sensor-frame point through the perspective camera. Returns
// nullopt when the point sits at or behind the camera. The result is not
// clipped to the canvas.
[[nodiscard]] std::optional<ProjectedPoint> project_perspective(
  double x, double y, double z, const ScanPatternView & view);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__SCAN_PATTERN_HPP_
