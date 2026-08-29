// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_view.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

using bagwiz::core::pointcloud::CloudProjection;
using bagwiz::core::pointcloud::CloudView;
using bagwiz::core::pointcloud::project_bev;
using bagwiz::core::pointcloud::project_perspective;

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
