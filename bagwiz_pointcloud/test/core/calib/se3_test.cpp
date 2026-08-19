// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/se3.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace calib = bagwiz::core::calib;

TEST(Se3Test, MakeTransformRoundTripsRpyAndTranslation)
{
  const std::array<double, 3> xyz{1.5, -2.0, 0.25};
  const std::array<double, 3> rpy{0.1, -0.2, 0.7};
  const calib::Mat4 t = calib::make_transform(xyz, rpy);
  const auto xyz2 = calib::translation_of(t);
  const auto rpy2 = calib::rpy_of(t);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(xyz2[i], xyz[i], 1e-12);
    EXPECT_NEAR(rpy2[i], rpy[i], 1e-12);
  }
}

TEST(Se3Test, YawRotatesXTowardsY)
{
  // R = Rz(pi/2): the +x axis maps to +y (tf2 fixed-axis convention).
  const calib::Mat4 t = calib::make_transform({0, 0, 0}, {0, 0, M_PI / 2});
  const auto p = calib::transform_point(t, {1.0, 0.0, 0.0});
  EXPECT_NEAR(p[0], 0.0, 1e-12);
  EXPECT_NEAR(p[1], 1.0, 1e-12);
  EXPECT_NEAR(p[2], 0.0, 1e-12);
}

TEST(Se3Test, MultiplyAppliesRightFactorFirst)
{
  const calib::Mat4 rot = calib::make_transform({0, 0, 0}, {0, 0, M_PI / 2});
  const calib::Mat4 shift = calib::make_transform({1, 0, 0}, {0, 0, 0});
  // (rot * shift) p = rot(shift(p)): p=origin -> shift to (1,0,0) -> rotate to (0,1,0).
  const auto p = calib::transform_point(calib::mat4_multiply(rot, shift), {0, 0, 0});
  EXPECT_NEAR(p[0], 0.0, 1e-12);
  EXPECT_NEAR(p[1], 1.0, 1e-12);
}

TEST(Se3Test, RigidInverseUndoesTransform)
{
  const calib::Mat4 t = calib::make_transform({0.3, -1.0, 2.0}, {0.2, 0.4, -0.6});
  const calib::Mat4 id = calib::mat4_multiply(t, calib::rigid_inverse(t));
  const calib::Mat4 expect = calib::identity_mat4();
  for (int i = 0; i < 16; ++i) {
    EXPECT_NEAR(id[i], expect[i], 1e-12) << "index " << i;
  }
}

TEST(Se3Test, RotationAngleBetweenMeasuresRelativeRotation)
{
  const calib::Mat4 a = calib::make_transform({1, 2, 3}, {0, 0, 0.3});
  const calib::Mat4 b = calib::make_transform({-4, 0, 7}, {0, 0, 0.5});
  // Translation must not contribute; only the 0.2 rad relative yaw does.
  EXPECT_NEAR(calib::rotation_angle_between(a, b), 0.2, 1e-12);
  EXPECT_NEAR(calib::rotation_angle_between(a, a), 0.0, 1e-12);
}

TEST(Se3Test, RotationAngleBetweenIsAxisAgnostic)
{
  const calib::Mat4 a = calib::identity_mat4();
  const calib::Mat4 b = calib::make_transform({0, 0, 0}, {M_PI / 2, 0, 0});
  EXPECT_NEAR(calib::rotation_angle_between(a, b), M_PI / 2, 1e-12);
}
