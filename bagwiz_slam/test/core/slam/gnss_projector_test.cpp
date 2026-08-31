// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_projector.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

// Unit test for GnssProjector: WGS84 lat/lon/alt -> local ENU meters around the
// first projected fix. Verifies the origin latches on the first call (yielding
// the zero vector) and that the East/North/Up axes carry the expected sign and
// magnitude. Links GeographicLib (alongside bagwiz_slam); no GLIM involved.
namespace
{
namespace slam = bagwiz::core::slam;

TEST(GnssProjector, FirstFixIsOriginThenRelativeEnu)
{
  slam::GnssProjector projector;
  EXPECT_FALSE(projector.has_origin());

  // First fix becomes the ENU origin -> the zero vector.
  const std::array<double, 3> origin = projector.project(35.0, 139.0, 0.0);
  EXPECT_TRUE(projector.has_origin());
  EXPECT_NEAR(origin[0], 0.0, 1e-6);
  EXPECT_NEAR(origin[1], 0.0, 1e-6);
  EXPECT_NEAR(origin[2], 0.0, 1e-6);

  // ~0.001 deg north: x (East) ~ 0, y (North) ~ 111 m, z ~ 0.
  const std::array<double, 3> north = projector.project(35.001, 139.0, 0.0);
  EXPECT_NEAR(north[0], 0.0, 1.0);
  EXPECT_GT(north[1], 100.0);
  EXPECT_LT(north[1], 120.0);
  EXPECT_NEAR(north[2], 0.0, 1.0);

  // ~0.001 deg east at lat 35: x (East) ~ cos(35) * 111 m ~ 91 m, y (North) ~ 0.
  const std::array<double, 3> east = projector.project(35.0, 139.001, 0.0);
  EXPECT_GT(east[0], 80.0);
  EXPECT_LT(east[0], 100.0);
  EXPECT_NEAR(east[1], 0.0, 1.0);

  // +10 m altitude maps to +z (Up).
  const std::array<double, 3> up = projector.project(35.0, 139.0, 10.0);
  EXPECT_NEAR(up[0], 0.0, 1e-3);
  EXPECT_NEAR(up[1], 0.0, 1e-3);
  EXPECT_NEAR(up[2], 10.0, 1e-3);
}

TEST(GnssProjector, OriginIsTheFirstFix)
{
  slam::GnssProjector projector;
  EXPECT_FALSE(projector.origin().has_value());
  (void)projector.project(35.0, 139.0, 40.0);
  const auto origin = projector.origin();
  ASSERT_TRUE(origin.has_value());
  EXPECT_DOUBLE_EQ((*origin)[0], 35.0);
  EXPECT_DOUBLE_EQ((*origin)[1], 139.0);
  EXPECT_DOUBLE_EQ((*origin)[2], 40.0);
}

TEST(GnssProjector, ReverseUndoesProject)
{
  slam::GnssProjector projector;
  (void)projector.project(35.0, 139.0, 40.0);
  const std::array<double, 3> enu = projector.project(35.001, 139.002, 45.0);
  const std::array<double, 3> fix = projector.reverse(enu[0], enu[1], enu[2]);
  EXPECT_NEAR(fix[0], 35.001, 1e-9);
  EXPECT_NEAR(fix[1], 139.002, 1e-9);
  EXPECT_NEAR(fix[2], 45.0, 1e-6);

  // The plane's own points: 100 m north of the origin is ~0.0009 deg of
  // latitude, on the origin's meridian.
  const std::array<double, 3> north = projector.reverse(0.0, 100.0, 0.0);
  EXPECT_GT(north[0], 35.0008);
  EXPECT_LT(north[0], 35.0010);
  EXPECT_NEAR(north[1], 139.0, 1e-9);
}

TEST(GnssProjector, ReverseBeforeAnyFixIsTheDatumOrigin)
{
  // Without a latched origin the solver sits at (0, 0, 0): the reverse of the
  // zero vector is that point, not a crash.
  slam::GnssProjector projector;
  const std::array<double, 3> fix = projector.reverse(0.0, 0.0, 0.0);
  EXPECT_NEAR(fix[0], 0.0, 1e-9);
  EXPECT_NEAR(fix[1], 0.0, 1e-9);
}

}  // namespace
