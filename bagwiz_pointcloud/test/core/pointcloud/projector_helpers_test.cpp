// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector_helpers.hpp"

#include "bagwiz/core/pointcloud/cloud_transform.hpp"

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

namespace
{

using bagwiz::core::pointcloud::lookup_rigid_transform;
using bagwiz::core::pointcloud::RigidTransform;

constexpr std::int64_t kStampNs = 1'000'000'000LL;
constexpr double kTol = 1e-9;

// `child` sits at (x, y, z) in `parent`, yawed by `yaw_rad` about +z.
geometry_msgs::msg::TransformStamped make_tf(
  const std::string & parent, const std::string & child, double x, double y, double z,
  double yaw_rad)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.header.stamp.sec = 0;
  t.header.stamp.nanosec = 0;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(yaw_rad / 2.0);
  t.transform.rotation.w = std::cos(yaw_rad / 2.0);
  return t;
}

// p' = R p + t.
std::array<double, 3> apply(const RigidTransform & tf, double x, double y, double z)
{
  const auto & r = tf.rotation;
  const auto & t = tf.translation;
  return {
    r[0] * x + r[1] * y + r[2] * z + t[0], r[3] * x + r[4] * y + r[5] * z + t[1],
    r[6] * x + r[7] * y + r[8] * z + t[2]};
}

class LookupRigidTransformTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // The lidar sits 1 m ahead, 2 m left and 3 m up of the vehicle origin,
    // turned 90 degrees to the left.
    buffer_.setTransform(
      make_tf("base_link", "lidar", 1.0, 2.0, 3.0, M_PI / 2.0), "test", /*is_static=*/true);
  }

  tf2::BufferCore buffer_{std::chrono::seconds{10}};
  std::string error_;
};

TEST_F(LookupRigidTransformTest, MapsASourcePointIntoTheTargetFrame)
{
  const auto tf = lookup_rigid_transform(buffer_, "base_link", "lidar", kStampNs, error_);
  ASSERT_TRUE(tf.has_value()) << error_;
  EXPECT_TRUE(error_.empty());
  // A point 1 m ahead of the lidar (+x in lidar) points left in the vehicle
  // (+y), from the lidar's position.
  const auto p = apply(*tf, 1.0, 0.0, 0.0);
  EXPECT_NEAR(p[0], 1.0, kTol);
  EXPECT_NEAR(p[1], 3.0, kTol);
  EXPECT_NEAR(p[2], 3.0, kTol);
  // The lidar's own origin lands at its position in the vehicle.
  const auto o = apply(*tf, 0.0, 0.0, 0.0);
  EXPECT_NEAR(o[0], 1.0, kTol);
  EXPECT_NEAR(o[1], 2.0, kTol);
  EXPECT_NEAR(o[2], 3.0, kTol);
}

TEST_F(LookupRigidTransformTest, SwappingTheFramesGivesTheInverse)
{
  const auto forward = lookup_rigid_transform(buffer_, "base_link", "lidar", kStampNs, error_);
  const auto back = lookup_rigid_transform(buffer_, "lidar", "base_link", kStampNs, error_);
  ASSERT_TRUE(forward.has_value()) << error_;
  ASSERT_TRUE(back.has_value()) << error_;
  const auto there = apply(*forward, 0.5, -0.25, 2.0);
  const auto home = apply(*back, there[0], there[1], there[2]);
  EXPECT_NEAR(home[0], 0.5, kTol);
  EXPECT_NEAR(home[1], -0.25, kTol);
  EXPECT_NEAR(home[2], 2.0, kTol);
  // The vehicle origin seen from the lidar: -R^T t.
  const auto o = apply(*back, 0.0, 0.0, 0.0);
  EXPECT_NEAR(o[0], -2.0, kTol);
  EXPECT_NEAR(o[1], 1.0, kTol);
  EXPECT_NEAR(o[2], -3.0, kTol);
}

TEST_F(LookupRigidTransformTest, SameFrameIsTheIdentity)
{
  const auto tf = lookup_rigid_transform(buffer_, "lidar", "lidar", kStampNs, error_);
  ASSERT_TRUE(tf.has_value()) << error_;
  EXPECT_TRUE(tf->is_identity());
}

TEST_F(LookupRigidTransformTest, ReportsAnUnknownFrame)
{
  const auto tf = lookup_rigid_transform(buffer_, "base_link", "nowhere", kStampNs, error_);
  EXPECT_FALSE(tf.has_value());
  EXPECT_FALSE(error_.empty());
  EXPECT_NE(error_.find("nowhere"), std::string::npos) << error_;
}

}  // namespace
