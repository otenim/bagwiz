// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/scan_pattern.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::extract_scan_times;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointFieldType;
using bagwiz::core::pointcloud::PointTimeField;
using bagwiz::core::pointcloud::project_bev;
using bagwiz::core::pointcloud::project_perspective;
using bagwiz::core::pointcloud::project_scan_points;
using bagwiz::core::pointcloud::ScanPatternProjection;
using bagwiz::core::pointcloud::ScanPatternView;
using bagwiz::core::pointcloud::sorted_scan_indices;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// [x y z t] as 3x float32 + time field of the given type (point_step 16/20).
// `raw_times` holds the field's raw value per point: UINT32 nanoseconds,
// FLOAT32/64 seconds, or NaN to write a non-finite float.
PointCloud2 make_cloud(
  const std::vector<std::vector<float>> & xyz, PointFieldType time_type,
  const std::vector<double> & raw_times)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(xyz.size());
  c.point_step = time_type == PointFieldType::kFloat64 ? 20 : 16;
  c.row_step = c.point_step * c.width;
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, time_type, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < xyz.size(); ++i) {
    std::byte * base = c.data.data() + i * c.point_step;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const float v = xyz[i][axis];
      std::memcpy(base + axis * sizeof(float), &v, sizeof(float));
    }
    std::byte * tb = base + 12;
    switch (time_type) {
      case PointFieldType::kUint32: {
        const std::uint32_t v = static_cast<std::uint32_t>(raw_times[i]);
        std::memcpy(tb, &v, sizeof(v));
        break;
      }
      case PointFieldType::kFloat32: {
        const float v = static_cast<float>(raw_times[i]);
        std::memcpy(tb, &v, sizeof(v));
        break;
      }
      case PointFieldType::kFloat64: {
        const double v = raw_times[i];
        std::memcpy(tb, &v, sizeof(v));
        break;
      }
      default:
        break;
    }
  }
  return c;
}

ScanPatternView bev_view(std::uint32_t w, std::uint32_t h, double range)
{
  ScanPatternView v;
  v.projection = ScanPatternProjection::kBev;
  v.width = w;
  v.height = h;
  v.range_m = range;
  return v;
}

ScanPatternView persp_view(std::uint32_t w, std::uint32_t h, double elev, double azim, double dist)
{
  ScanPatternView v;
  v.projection = ScanPatternProjection::kPerspective;
  v.width = w;
  v.height = h;
  v.elev_deg = elev;
  v.azim_deg = azim;
  v.dist_m = dist;
  return v;
}

}  // namespace

TEST(ScanPatternTimes, ExtractsUint32AsSeconds)
{
  const auto cloud =
    make_cloud({{1, 0, 0}, {2, 0, 0}}, PointFieldType::kUint32, {10'000'000.0, 20'000'000.0});
  const auto r = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kUint32});
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.times.size(), 2u);
  EXPECT_NEAR(r.times[0], 0.01, 1e-12);
  EXPECT_NEAR(r.times[1], 0.02, 1e-12);
}

TEST(ScanPatternTimes, ExtractsFloat64)
{
  const auto cloud = make_cloud({{1, 0, 0}}, PointFieldType::kFloat64, {1.7e9});
  const auto r = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kFloat64});
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.times.size(), 1u);
  EXPECT_NEAR(r.times[0], 1.7e9, 1.0);
}

TEST(ScanPatternTimes, KeepsNonFiniteAsNaN)
{
  const auto cloud = make_cloud({{1, 0, 0}, {2, 0, 0}}, PointFieldType::kFloat32, {0.01, kNaN});
  const auto r = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kFloat32});
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.times.size(), 2u);
  EXPECT_NEAR(r.times[0], 0.01, 1e-6);
  EXPECT_TRUE(std::isnan(r.times[1]));
}

TEST(ScanPatternTimes, RejectsOutOfBoundsField)
{
  const auto cloud = make_cloud({{1, 0, 0}}, PointFieldType::kFloat32, {0.0});
  const auto r = extract_scan_times(cloud, PointTimeField{14, PointFieldType::kFloat32});
  EXPECT_FALSE(r.ok());
}

TEST(ScanPatternTimes, RejectsInconsistentLayout)
{
  auto cloud = make_cloud({{1, 0, 0}}, PointFieldType::kFloat32, {0.0});
  cloud.data.resize(4);  // too small for one point
  const auto r = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kFloat32});
  EXPECT_FALSE(r.ok());
}

TEST(ScanPatternOrder, SortsByTimeAndDropsNaN)
{
  const std::vector<double> times = {0.03, kNaN, 0.01, 0.02};
  const auto order = sorted_scan_indices(times);
  ASSERT_EQ(order.size(), 3u);
  EXPECT_EQ(order[0], 2u);
  EXPECT_EQ(order[1], 3u);
  EXPECT_EQ(order[2], 0u);
}

TEST(ScanPatternOrder, EmptyInput)
{
  EXPECT_TRUE(sorted_scan_indices({}).empty());
}

TEST(ScanPatternBev, CenterIsOrigin)
{
  const auto p = project_bev(0.0, 0.0, bev_view(200, 100, 50.0));
  EXPECT_EQ(p.u, 100);
  EXPECT_EQ(p.v, 50);
}

TEST(ScanPatternBev, ForwardIsUpLeftIsLeft)
{
  const auto view = bev_view(200, 100, 50.0);
  // scale = min(200,100)/2 / 50 = 1 px/m.
  const auto fwd = project_bev(10.0, 0.0, view);  // +x: 10 m forward
  EXPECT_EQ(fwd.u, 100);
  EXPECT_EQ(fwd.v, 40);                            // 10 px above center
  const auto left = project_bev(0.0, 10.0, view);  // +y: 10 m left
  EXPECT_EQ(left.u, 90);                           // 10 px left of center
  EXPECT_EQ(left.v, 50);
}

TEST(ScanPatternBev, UsesSmallerDimensionForScale)
{
  // Wide canvas: height still bounds the scale, so +-range on x fits exactly.
  const auto view = bev_view(400, 100, 50.0);  // scale = 50/50 = 1 px/m
  const auto edge = project_bev(50.0, 0.0, view);
  EXPECT_EQ(edge.u, 200);
  EXPECT_EQ(edge.v, 0);
}

TEST(ScanPatternPerspective, PointAheadOfCameraIsCentered)
{
  // Camera at azim 180, elev 0, dist 10 -> sits at (-10, 0, 0) looking down +x.
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  const auto p = project_perspective(10.0, 0.0, 0.0, view);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->u, 100);
  EXPECT_EQ(p->v, 50);
  EXPECT_NEAR(p->depth, 20.0, 1e-6);
}

TEST(ScanPatternPerspective, UpIsUpLeftIsLeft)
{
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  // 20 m ahead of the camera, offset in +z (up) and +y (scene left).
  const auto up = project_perspective(10.0, 0.0, 5.0, view);
  ASSERT_TRUE(up.has_value());
  EXPECT_EQ(up->u, 100);
  EXPECT_LT(up->v, 50);  // up on the image
  const auto left = project_perspective(10.0, 5.0, 0.0, view);
  ASSERT_TRUE(left.has_value());
  EXPECT_LT(left->u, 100);  // scene-left appears image-left (chase-cam view)
  EXPECT_EQ(left->v, 50);
}

TEST(ScanPatternPerspective, DropsPointsBehindCamera)
{
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  EXPECT_FALSE(project_perspective(-20.0, 0.0, 0.0, view).has_value());
}

TEST(ScanPatternPerspective, ElevatedCameraLooksDown)
{
  // elev 90 looks straight down the -z axis: the up fallback keeps the frame
  // non-degenerate with camera-up = +y, so a point at +y appears image-up.
  const auto view = persp_view(200, 100, 90.0, 0.0, 10.0);
  const auto p = project_perspective(0.0, 5.0, 0.0, view);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->u, 100);
  EXPECT_LT(p->v, 50);
}

TEST(ScanPatternProject, ProjectsInTimeOrderWithRelativeValues)
{
  const auto cloud =
    make_cloud({{10, 0, 0}, {0, 10, 0}, {0, 0, 5}}, PointFieldType::kFloat32, {0.02, 0.01, 0.03});
  const auto times = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kFloat32});
  ASSERT_TRUE(times.ok()) << times.error;
  const auto order = sorted_scan_indices(times.times);
  const double t_min = times.times[order.front()];

  const auto r = project_scan_points(cloud, bev_view(200, 100, 50.0), order, times.times, t_min);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 3u);
  // Order: point 1 (t=0.01), point 0 (t=0.02), point 2 (t=0.03).
  EXPECT_NEAR(r.points[0].value, 0.0, 1e-6);
  EXPECT_NEAR(r.points[1].value, 0.01, 1e-6);
  EXPECT_NEAR(r.points[2].value, 0.02, 1e-6);
  // First drawn is the +y point (image left), second the +x point (image up).
  EXPECT_EQ(r.points[0].u, 100 - 10);
  EXPECT_EQ(r.points[0].v, 50);
  EXPECT_EQ(r.points[1].u, 100);
  EXPECT_EQ(r.points[1].v, 50 - 10);
}

TEST(ScanPatternProject, SkipsNonFiniteCoordinates)
{
  const float fNaN = std::numeric_limits<float>::quiet_NaN();
  const auto cloud =
    make_cloud({{10, 0, 0}, {fNaN, 0, 0}, {0, 10, 0}}, PointFieldType::kFloat32, {0.0, 0.01, 0.02});
  const auto times = extract_scan_times(cloud, PointTimeField{12, PointFieldType::kFloat32});
  ASSERT_TRUE(times.ok()) << times.error;
  const auto order = sorted_scan_indices(times.times);

  const auto r = project_scan_points(cloud, bev_view(200, 100, 50.0), order, times.times, 0.0);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.points.size(), 2u);  // the NaN-x point is dropped
}

TEST(ScanPatternProject, RequiresXYZFields)
{
  auto cloud = make_cloud({{1, 0, 0}}, PointFieldType::kFloat32, {0.0});
  cloud.fields.pop_back();  // drop "t" so the field list is only x/y/z...
  cloud.fields.pop_back();  // ... then drop "z" too
  const std::vector<double> times = {0.0};
  const std::vector<std::uint32_t> order = {0};
  const auto r = project_scan_points(cloud, bev_view(200, 100, 50.0), order, times, 0.0);
  EXPECT_FALSE(r.ok());
}
