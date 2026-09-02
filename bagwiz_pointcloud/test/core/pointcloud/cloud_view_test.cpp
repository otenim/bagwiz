// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_view.hpp"

#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::bev_auto_range;
using bagwiz::core::pointcloud::CloudProjection;
using bagwiz::core::pointcloud::CloudView;
using bagwiz::core::pointcloud::kBevAutoRangeQuantile;
using bagwiz::core::pointcloud::make_perspective_camera;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointCloudProperty;
using bagwiz::core::pointcloud::PointFieldType;
using bagwiz::core::pointcloud::project_bev;
using bagwiz::core::pointcloud::project_cloud_to_view;
using bagwiz::core::pointcloud::project_perspective;
using bagwiz::core::pointcloud::RigidTransform;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

CloudView bev_view(std::uint32_t w, std::uint32_t h, double range)
{
  CloudView v;
  v.projection = CloudProjection::kBev;
  v.width = w;
  v.height = h;
  v.range_m = range;
  return v;
}

CloudView persp_view(std::uint32_t w, std::uint32_t h, double elev, double azim, double dist)
{
  CloudView v;
  v.projection = CloudProjection::kPerspective;
  v.width = w;
  v.height = h;
  v.elev_deg = elev;
  v.azim_deg = azim;
  v.dist_m = dist;
  return v;
}

// A cloud of [x y z intensity] float32 points (point_step 16).
PointCloud2 make_cloud(const std::vector<std::vector<float>> & points, bool with_intensity = true)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(points.size());
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
  };
  if (with_intensity) {
    c.fields.push_back({"intensity", 12, PointFieldType::kFloat32, 1});
  }
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::byte * base = c.data.data() + i * c.point_step;
    for (std::size_t k = 0; k < points[i].size(); ++k) {
      const float v = points[i][k];
      std::memcpy(base + k * sizeof(float), &v, sizeof(float));
    }
  }
  return c;
}

}  // namespace

TEST(CloudViewBev, CenterIsOrigin)
{
  const auto p = project_bev(0.0, 0.0, bev_view(200, 100, 50.0));
  EXPECT_EQ(p.u, 100);
  EXPECT_EQ(p.v, 50);
  EXPECT_FLOAT_EQ(p.depth, 0.0F);
}

TEST(CloudViewBev, ForwardIsUpLeftIsLeft)
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

TEST(CloudViewBev, UsesSmallerDimensionForScale)
{
  // Wide canvas: height still bounds the scale, so +-range on x fits exactly.
  const auto view = bev_view(400, 100, 50.0);  // scale = 50/50 = 1 px/m
  const auto edge = project_bev(50.0, 0.0, view);
  EXPECT_EQ(edge.u, 200);
  EXPECT_EQ(edge.v, 0);
}

TEST(CloudViewBev, KeepsPointsPastTheCanvas)
{
  // Clipping is the rasterizer's job: a point beyond the range still maps to
  // a (negative) pixel so the caller can decide what to do with it.
  const auto view = bev_view(200, 100, 50.0);
  const auto far = project_bev(80.0, 0.0, view);
  EXPECT_EQ(far.u, 100);
  EXPECT_EQ(far.v, -30);
}

TEST(CloudViewPerspective, PointAheadOfCameraIsCentered)
{
  // Camera at azim 180, elev 0, dist 10 -> sits at (-10, 0, 0) looking down +x.
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  const auto p = project_perspective(10.0, 0.0, 0.0, view);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->u, 100);
  EXPECT_EQ(p->v, 50);
  EXPECT_NEAR(p->depth, 20.0, 1e-6);
}

TEST(CloudViewPerspective, UpIsUpLeftIsLeft)
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

TEST(CloudViewPerspective, DropsPointsBehindCamera)
{
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  EXPECT_FALSE(project_perspective(-20.0, 0.0, 0.0, view).has_value());
}

TEST(CloudViewPerspective, DropsPointsAtTheCameraPlane)
{
  // A point exactly at the camera's own depth has no finite projection.
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  EXPECT_FALSE(project_perspective(-10.0, 3.0, 0.0, view).has_value());
}

TEST(CloudViewPerspective, ElevatedCameraLooksDown)
{
  // elev 90 looks straight down the -z axis: the up fallback keeps the frame
  // non-degenerate with camera-up = +y, so a point at +y appears image-up.
  const auto view = persp_view(200, 100, 90.0, 0.0, 10.0);
  const auto p = project_perspective(0.0, 5.0, 0.0, view);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(p->u, 100);
  EXPECT_LT(p->v, 50);
}

TEST(CloudViewPerspective, FartherPointsProjectCloserToTheCenter)
{
  // Same lateral offset at twice the depth lands at half the pixel offset.
  const auto view = persp_view(200, 100, 0.0, 180.0, 10.0);
  const auto near = project_perspective(10.0, 5.0, 0.0, view);  // depth 20
  const auto far = project_perspective(30.0, 5.0, 0.0, view);   // depth 40
  ASSERT_TRUE(near.has_value());
  ASSERT_TRUE(far.has_value());
  EXPECT_NEAR(far->depth, 40.0, 1e-6);
  EXPECT_EQ(100 - far->u, (100 - near->u) / 2);
}

// The camera worked out once per view projects exactly like the per-call
// overload, so a whole cloud can skip the trigonometry per point.
TEST(CloudViewPerspective, PrecomputedCameraMatchesThePerCallProjection)
{
  const auto view = persp_view(320, 240, 35.0, 120.0, 25.0);
  const auto camera = make_perspective_camera(view);
  EXPECT_NEAR(camera.focal, 120.0 / std::tan(30.0 * M_PI / 180.0), 1e-9);
  for (const auto & xyz : std::vector<std::vector<double>>{
         {5.0, 2.0, 1.0}, {-3.0, 8.0, -2.0}, {12.0, -4.0, 3.5}, {0.0, 0.0, 0.0}}) {
    const auto a = project_perspective(xyz[0], xyz[1], xyz[2], camera, view);
    const auto b = project_perspective(xyz[0], xyz[1], xyz[2], view);
    ASSERT_EQ(a.has_value(), b.has_value());
    if (a.has_value()) {
      EXPECT_EQ(a->u, b->u);
      EXPECT_EQ(a->v, b->v);
      EXPECT_FLOAT_EQ(a->depth, b->depth);
    }
  }
}

TEST(CloudViewProject, BevProjectsEveryFinitePointWithHeightAsNegatedDepth)
{
  // 200x100 at range 50: 1 px/m. Points at +x / +y / a NaN one.
  const auto cloud =
    make_cloud({{10.0F, 0.0F, 2.0F, 7.0F}, {0.0F, 10.0F, -1.0F, 3.0F}, {kNaN, 0.0F, 0.0F, 0.0F}});
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(200, 100, 50.0), PointCloudProperty::kZ);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 2u);
  EXPECT_EQ(r.points[0].u, 100);
  EXPECT_EQ(r.points[0].v, 40);
  EXPECT_FLOAT_EQ(r.points[0].depth, -2.0F);  // higher point = smaller depth
  EXPECT_FLOAT_EQ(r.points[0].value, 2.0F);
  EXPECT_EQ(r.points[1].u, 90);
  EXPECT_EQ(r.points[1].v, 50);
  EXPECT_FLOAT_EQ(r.points[1].depth, 1.0F);
  EXPECT_FLOAT_EQ(r.points[1].value, -1.0F);
}

TEST(CloudViewProject, DropsPointsOutsideTheCanvas)
{
  const auto cloud = make_cloud({{80.0F, 0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 0.0F, 0.0F}});
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(200, 100, 50.0), PointCloudProperty::kDistance);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 1u);
  EXPECT_NEAR(r.points[0].value, std::sqrt(2.0), 1e-6);
}

TEST(CloudViewProject, AppliesTheRigidTransformButColorsInTheCloudFrame)
{
  // Rotate 90 degrees about +z (x -> y) and shift 10 m along +x: the point
  // (5, 0, 0) lands at (10, 5, 0) in the view frame, so on a 1 px/m BEV it
  // sits 10 px above and 5 px left of center; its x value stays 5 (the
  // cloud's own coordinate) for coloring.
  RigidTransform tf;
  tf.rotation = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  tf.translation = {10.0, 0.0, 0.0};
  const auto cloud = make_cloud({{5.0F, 0.0F, 0.0F, 0.0F}});
  const auto r = project_cloud_to_view(cloud, tf, bev_view(200, 100, 50.0), PointCloudProperty::kX);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 1u);
  EXPECT_EQ(r.points[0].u, 95);
  EXPECT_EQ(r.points[0].v, 40);
  EXPECT_FLOAT_EQ(r.points[0].value, 5.0F);
}

TEST(CloudViewProject, PerspectiveDepthIsCameraDistance)
{
  const auto cloud = make_cloud({{10.0F, 0.0F, 0.0F, 42.0F}});
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, persp_view(200, 100, 0.0, 180.0, 10.0),
    PointCloudProperty::kIntensity);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 1u);
  EXPECT_EQ(r.points[0].u, 100);
  EXPECT_EQ(r.points[0].v, 50);
  EXPECT_NEAR(r.points[0].depth, 20.0, 1e-6);
  EXPECT_FLOAT_EQ(r.points[0].value, 42.0F);
}

TEST(CloudViewProject, RequiresXyzAndIntensityWhenAsked)
{
  auto no_xyz = make_cloud({{1.0F, 2.0F, 3.0F, 0.0F}});
  no_xyz.fields.erase(no_xyz.fields.begin() + 2);  // drop z
  EXPECT_FALSE(project_cloud_to_view(
                 no_xyz, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance)
                 .ok());
  const auto no_intensity = make_cloud({{1.0F, 2.0F, 3.0F}}, /*with_intensity=*/false);
  EXPECT_FALSE(
    project_cloud_to_view(
      no_intensity, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kIntensity)
      .ok());
  EXPECT_TRUE(
    project_cloud_to_view(
      no_intensity, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance)
      .ok());
}

TEST(CloudViewProject, ToleratesAStaleSmallRowStep)
{
  // A concatenated cloud whose row_step was left over from a source cloud
  // (smaller than width * point_step): the blob is densely packed, so the
  // stale stride is ignored in favor of dense packing.
  auto cloud = make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F, 0.0F}});
  cloud.row_step = cloud.point_step;  // stale: smaller than width * point_step
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 2);
  EXPECT_FLOAT_EQ(r.points[0].value, 1.0F);
  EXPECT_FLOAT_EQ(r.points[1].value, 2.0F);
}

TEST(CloudViewProject, ProjectsTheCompletePointsOfATruncatedBlob)
{
  auto cloud = make_cloud({{1.0F, 2.0F, 3.0F, 0.0F}, {4.0F, 5.0F, 6.0F, 0.0F}});
  cloud.data.resize(20);  // one full point plus a partial one
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 1);
  EXPECT_FLOAT_EQ(r.points[0].value, std::sqrt(14.0F));  // distance of (1, 2, 3)
}

TEST(CloudViewProject, ProjectsTheFullRowsAndPartialTailOfAShortOrganizedBlob)
{
  // An organized 2x2 cloud with padded rows whose blob holds one full row
  // and one point of the second: three of the four declared points project.
  auto cloud = make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F, 0.0F}});
  cloud.height = 2;
  cloud.width = 2;
  cloud.row_step = 40;                  // 8 bytes of padding after each 32-byte row
  cloud.data.assign(64, std::byte{0});  // one full row plus half a row
  const float xs[3] = {1.0F, 2.0F, 3.0F};
  const std::size_t offs[3] = {0, 16, 40};
  for (int i = 0; i < 3; ++i) {
    std::memcpy(cloud.data.data() + offs[i], &xs[i], sizeof(float));
  }
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 3);
  EXPECT_FLOAT_EQ(r.points[0].value, 1.0F);
  EXPECT_FLOAT_EQ(r.points[1].value, 2.0F);
  EXPECT_FLOAT_EQ(r.points[2].value, 3.0F);
}

TEST(CloudViewProject, IgnoresTrailingBytesBeyondTheDeclaredPoints)
{
  // The blob carries a third, fully valid point past the declared
  // height * width: it must not project. Fails if resolve_layout ever
  // stops clamping the walk to the declared point count.
  auto cloud = make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F, 0.0F}});
  cloud.data.resize(3 * cloud.point_step, std::byte{0});
  const float x = 5.0F;  // inside the canvas, so it would draw if read
  std::memcpy(cloud.data.data() + 2 * cloud.point_step, &x, sizeof(float));
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.points.size(), 2);
  EXPECT_FLOAT_EQ(r.points[0].value, 1.0F);
  EXPECT_FLOAT_EQ(r.points[1].value, 2.0F);
}

TEST(CloudViewProject, RejectsABlobTooSmallForOnePoint)
{
  auto cloud = make_cloud({{1.0F, 2.0F, 3.0F, 0.0F}, {4.0F, 5.0F, 6.0F, 0.0F}});
  cloud.data.resize(8);  // shorter than a single point
  const auto r = project_cloud_to_view(
    cloud, RigidTransform{}, bev_view(100, 100, 10.0), PointCloudProperty::kDistance);
  EXPECT_FALSE(r.ok());
}

// ---- bev_auto_range ------------------------------------------------------------

// Twenty points at ground distances 1..20 m (z varies but is not a ground
// distance): the nearest-rank quantiles are the 19th, 10th and 20th values.
TEST(BevAutoRange, IsTheNearestRankQuantileOfGroundDistances)
{
  std::vector<std::vector<float>> points;
  for (int i = 1; i <= 20; ++i) {
    points.push_back({static_cast<float>(i), 0.0F, static_cast<float>(100 - i), 0.0F});
  }
  const PointCloud2 cloud = make_cloud(points);
  std::string error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 0.95, error).value_or(-1.0), 19.0) << error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 0.5, error).value_or(-1.0), 10.0) << error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 1.0, error).value_or(-1.0), 20.0) << error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 0.01, error).value_or(-1.0), 1.0) << error;
  static_assert(kBevAutoRangeQuantile > 0.0 && kBevAutoRangeQuantile <= 1.0);
}

TEST(BevAutoRange, UsesTheGroundDistanceAndSkipsNonFinitePoints)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // (3, 4) is 5 m away on the ground whatever its height; the NaN point and
  // the infinite one are skipped.
  const PointCloud2 cloud = make_cloud(
    {{3.0F, 4.0F, 50.0F, 0.0F},
     {nan, 1.0F, 0.0F, 0.0F},
     {std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 0.0F}});
  std::string error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 1.0, error).value_or(-1.0), 5.0) << error;
}

TEST(BevAutoRange, ToleratesAStaleSmallRowStep)
{
  auto cloud = make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F, 0.0F}});
  cloud.row_step = cloud.point_step;  // stale: smaller than width * point_step
  std::string error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 1.0, error).value_or(-1.0), 2.0) << error;
}

TEST(BevAutoRange, IgnoresTrailingBytesBeyondTheDeclaredPoints)
{
  auto cloud = make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F, 0.0F}});
  cloud.data.resize(3 * cloud.point_step, std::byte{0});
  const float x = 100.0F;  // would dominate the quantile if read
  std::memcpy(cloud.data.data() + 2 * cloud.point_step, &x, sizeof(float));
  std::string error;
  EXPECT_DOUBLE_EQ(bev_auto_range(cloud, 1.0, error).value_or(-1.0), 2.0) << error;
}

TEST(BevAutoRange, RejectsCloudsWithoutAUsablePoint)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::string error;
  EXPECT_FALSE(bev_auto_range(make_cloud({{nan, nan, nan, 0.0F}}), 0.95, error).has_value());
  EXPECT_EQ(error, "point cloud has no finite points");
  EXPECT_FALSE(bev_auto_range(make_cloud({{0.0F, 0.0F, 7.0F, 0.0F}}), 0.95, error).has_value());
  EXPECT_EQ(error, "point cloud's points all sit at the origin");
  EXPECT_FALSE(bev_auto_range(make_cloud({{1.0F, 0.0F, 0.0F, 0.0F}}), 0.0, error).has_value());
  EXPECT_EQ(error, "the auto-range fraction must be in (0, 1]");
}
