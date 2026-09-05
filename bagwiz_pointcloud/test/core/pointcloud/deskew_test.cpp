// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/deskew.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::TrajectoryPose;
using bagwiz::core::pointcloud::deskew_pointcloud2;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;

// One point = [x y z t] as 4x float32 (point_step 16). `pts`: {x,y,z,t_seconds}.
PointCloud2 make_cloud_xyzt(const std::vector<std::array<float, 4>> & pts)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat32, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(float) * 4);
  }
  return c;
}

std::array<float, 3> xyz_at(const PointCloud2 & c, std::size_t i)
{
  std::array<float, 3> o{};
  std::memcpy(o.data(), c.data.data() + i * c.point_step, sizeof(float) * 3);
  return o;
}

float t_at(const PointCloud2 & c, std::size_t i)
{
  float v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 12, sizeof(float));
  return v;
}

// FLOAT64 variant of make_cloud_xyzt: one point = [x y z t] as 4x float64
// (point_step 32).
PointCloud2 make_cloud_xyzt_f64(const std::vector<std::array<double, 4>> & pts)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.point_step = 32;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat64, 1},
    {"y", 8, PointFieldType::kFloat64, 1},
    {"z", 16, PointFieldType::kFloat64, 1},
    {"t", 24, PointFieldType::kFloat64, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step, pts[i].data(), sizeof(double) * 4);
  }
  return c;
}

double x_at_f64(const PointCloud2 & c, std::size_t i)
{
  double v;
  std::memcpy(&v, c.data.data() + i * c.point_step, sizeof(double));
  return v;
}

double t_at_f64(const PointCloud2 & c, std::size_t i)
{
  double v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 24, sizeof(double));
  return v;
}

// x/y/z FLOAT32 (point_step 16) with a UINT32 nanosecond "t" field at offset 12,
// one point.
PointCloud2 make_cloud_xyz_u32time(float x, float y, float z, std::uint32_t t_ns)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.point_step = 16;
  c.row_step = c.point_step;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kUint32, 1},
  };
  c.data.resize(c.point_step);
  const std::array<float, 3> xyz{x, y, z};
  std::memcpy(c.data.data(), xyz.data(), sizeof(float) * 3);
  std::memcpy(c.data.data() + 12, &t_ns, sizeof(t_ns));
  return c;
}

std::uint32_t time_u32_at(const PointCloud2 & c, std::size_t i)
{
  std::uint32_t v;
  std::memcpy(&v, c.data.data() + i * c.point_step + 12, sizeof(v));
  return v;
}

}  // namespace

TEST(Deskew, PureTranslationMovesPointToRefPose)
{
  // Sensor at t=0 at origin, at t=0.1s at x=+2. Point captured at t=0.1s with local x=0.
  // In the ref (t=0) frame the sensor is +2 ahead, so the point maps to x=+2.
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}});  // relative time 0.1s
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, /*t_ref_ns=*/0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.0f, 1e-6);  // relative time reset to 0
}

TEST(Deskew, RefTimePointUnchanged)
{
  auto cloud = make_cloud_xyzt({{1.0f, 2.0f, 3.0f, 0.0f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 5, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f, 1e-5);  // t==t_ref -> no motion
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.0f, 1e-6);       // time field reset to 0 too
}

TEST(Deskew, RejectsBigEndian)
{
  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}});
  cloud.is_bigendian = true;
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("big-endian"), std::string::npos);
}

TEST(Deskew, NonFinitePointPassedThrough)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto cloud = make_cloud_xyzt({{nan, nan, nan, 0.05f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 9, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_nonfinite, 1u);
  EXPECT_TRUE(std::isnan(xyz_at(*r.cloud, 0)[0]));
}

TEST(Deskew, NoTimeFieldReturnsVerbatimWithCounter)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.point_step = 12;
  c.row_step = 12;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
  };
  c.data.assign(12, std::byte{0});
  auto r = deskew_pointcloud2(c, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_no_time, 1u);
  EXPECT_EQ(r.points_deskewed, 0u);
}

TEST(Deskew, Float64XyzAndTime)
{
  // Same pure-translation scenario as PureTranslationMovesPointToRefPose, but
  // xyz + time are stored as FLOAT64 (point_step 32) to exercise the F64
  // load/store and F64 write_ref_time paths.
  auto cloud = make_cloud_xyzt_f64({{0.0, 0.0, 0.0, 0.1}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(x_at_f64(*r.cloud, 0), 2.0, 1e-9);
  EXPECT_NEAR(t_at_f64(*r.cloud, 0), 0.0, 1e-12);  // relative time reset to 0
}

TEST(Deskew, Uint32NanosecondTimeResetsToZero)
{
  // Same pure-translation scenario, but the time field is UINT32 nanoseconds
  // (100'000'000 ns == 0.1s) instead of FLOAT32 seconds. write_ref_time's
  // UINT32 branch always writes 0 (ns-relative), regardless of the `relative`
  // flag, so this holds even though the flag is also true here.
  auto cloud = make_cloud_xyz_u32time(0.0f, 0.0f, 0.0f, 100'000'000u);
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_EQ(time_u32_at(*r.cloud, 0), 0u);
}

TEST(Deskew, KeepPointTimeMovesXyzButLeavesFloatTimeField)
{
  // PureTranslationMovesPointToRefPose with keep_point_time: xyz still lands
  // on the ref pose, but the FLOAT32 relative time keeps its captured value
  // instead of being reset to 0.
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj, std::nullopt, /*keep_point_time=*/true);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.1f, 1e-6);
}

TEST(Deskew, KeepPointTimeLeavesUint32TimeField)
{
  // Uint32NanosecondTimeResetsToZero with keep_point_time: the UINT32 branch
  // of write_ref_time is the one that would otherwise always write 0, so it
  // is checked separately from the FLOAT32 case above.
  auto cloud = make_cloud_xyz_u32time(0.0f, 0.0f, 0.0f, 100'000'000u);
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj, std::nullopt, /*keep_point_time=*/true);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_EQ(time_u32_at(*r.cloud, 0), 100'000'000u);
}

TEST(Deskew, KeepPointTimeLeavesNonFinitePointsUntouched)
{
  // A non-finite point is skipped before the time write either way, so
  // keep_point_time must not change how it is passed through or counted.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto cloud = make_cloud_xyzt({{nan, nan, nan, 0.05f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 9, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj, std::nullopt, /*keep_point_time=*/true);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_nonfinite, 1u);
  EXPECT_TRUE(std::isnan(xyz_at(*r.cloud, 0)[0]));
  EXPECT_NEAR(t_at(*r.cloud, 0), 0.05f, 1e-6);
}

TEST(Deskew, OrganizedCloudHeightTwoWidthOne)
{
  // height=2, width=1, row_step=point_step (no row padding): each row is one
  // point. Row 0 repeats PureTranslationMovesPointToRefPose's point; row 1
  // repeats RefTimePointUnchanged's point. Exercises the height/row_step
  // addressing (as opposed to a single unorganized row of width points).
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}, {1.0f, 2.0f, 3.0f, 0.0f}});
  cloud.height = 2;
  cloud.width = 1;
  cloud.row_step = cloud.point_step;
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_total, 2u);
  EXPECT_EQ(r.points_deskewed, 2u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);  // row 0: moved to ref pose
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 1.0f, 1e-5);  // row 1: t==t_ref, unchanged
}

TEST(Deskew, NonIdentityExtrinsicRotatesTheMotionDelta)
{
  // extrinsic E = +90 deg about Z, zero translation (cloud frame -> traj
  // frame). The trajectory is a pure +2m translation along X between t=0 and
  // t=0.1s (identity rotation), a motion delta d=(2,0,0) in the traj frame.
  //
  // A *translation-only* extrinsic would be a degenerate choice here:
  // translations commute, so conjugating a translation-only trajectory step
  // by a translation-only E leaves the result completely independent of E
  // (E cancels exactly) -- that would pass even if E were ignored entirely.
  // A rotating E is required to actually exercise the extrinsic plumbing.
  //
  // Working through p' = E^-1 * (T_ref^-1 * T(t_i)) * E * p analytically:
  // T_ref^-1 * T(t_i) is the pure translation (I, d); conjugating it by
  // E = (R_E, 0) gives the pure translation (I, R_E^-1 * d) -- E's rotation
  // cancels out of the result's rotation but rotates the translation delta.
  // So p' = p + R_E^-1 * d.
  //
  // R_E^-1 is -90 deg about Z: R_E^-1 * (2,0,0) = (0,-2,0) (the same
  // "R_z(90 deg) * (1,0,0) -> (0,1,0)" convention independently exercised by
  // ComposeTrajectoryPose.ReferenceBridgeRotatesIntoFromFrame in
  // trajectory_test.cpp, read in reverse). A local point at (1,0,0) therefore
  // lands at (1,0,0) + (0,-2,0) = (1,-2,0).
  constexpr double kSinPiOver4 = 0.7071067811865476;
  geometry_msgs::msg::Transform extrinsic;
  extrinsic.translation.x = 0.0;
  extrinsic.translation.y = 0.0;
  extrinsic.translation.z = 0.0;
  extrinsic.rotation.x = 0.0;
  extrinsic.rotation.y = 0.0;
  extrinsic.rotation.z = kSinPiOver4;
  extrinsic.rotation.w = kSinPiOver4;

  auto cloud = make_cloud_xyzt({{1.0f, 0.0f, 0.0f, 0.1f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj, extrinsic);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 1u);
  const auto xyz = xyz_at(*r.cloud, 0);
  EXPECT_NEAR(xyz[0], 1.0f, 1e-4);
  EXPECT_NEAR(xyz[1], -2.0f, 1e-4);
  EXPECT_NEAR(xyz[2], 0.0f, 1e-4);
}

TEST(Deskew, TimeFieldExceedingPointStepTreatedAsNoTime)
{
  // "t" declared as FLOAT64 (8 bytes) at offset 12 with point_step 16: the
  // field's own bytes [12,20) run 4 bytes past the point. Regression test for
  // an OOB read (the relative/absolute scan and point_time_seconds) and OOB
  // write (write_ref_time) this layout used to trigger -- corrupting the next
  // point's x and, for the last point, writing past the end of `data`
  // entirely. Built by hand (not make_cloud_xyzt, whose "t" is FLOAT32) so
  // the mismatched field/point_step combination is explicit.
  PointCloud2 c;
  c.height = 1;
  c.width = 2;
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat64, 1},  // offset 12 + size 8 = 20 > point_step 16
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  const std::array<float, 3> p0{1.0f, 2.0f, 3.0f};
  const std::array<float, 3> p1{4.0f, 5.0f, 6.0f};
  std::memcpy(c.data.data(), p0.data(), sizeof(float) * 3);
  std::memcpy(c.data.data() + c.point_step, p1.data(), sizeof(float) * 3);

  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(c, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_total, 2u);
  EXPECT_EQ(r.points_no_time, 2u);
  EXPECT_EQ(r.points_deskewed, 0u);
  const auto xyz0 = xyz_at(*r.cloud, 0);
  const auto xyz1 = xyz_at(*r.cloud, 1);
  EXPECT_NEAR(xyz0[0], 1.0f, 1e-6);
  EXPECT_NEAR(xyz0[1], 2.0f, 1e-6);
  EXPECT_NEAR(xyz0[2], 3.0f, 1e-6);
  EXPECT_NEAR(xyz1[0], 4.0f, 1e-6);  // unchanged: not corrupted by point 0's time write
  EXPECT_NEAR(xyz1[1], 5.0f, 1e-6);
  EXPECT_NEAR(xyz1[2], 6.0f, 1e-6);
}

TEST(Deskew, StaleRowStepSmallerThanWidthTimesPointStepIsDensePacked)
{
  // width=2, point_step=16 needs row_step >= 32, but row_step is 16: the
  // leftover of a narrower source cloud that a concatenation pipeline did not
  // update when it grew the width. Such a row_step cannot describe the blob
  // (a row would not fit in it), so the points are read densely packed --
  // the same rule the movify cloud walks apply -- and both are deskewed.
  // Same geometry as PureTranslationMovesPointToRefPose.
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}, {0.0f, 0.0f, 0.0f, 0.1f}});
  cloud.row_step = cloud.point_step;  // 16, but width*point_step = 32
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, /*t_ref_ns=*/0, traj);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.points_deskewed, 2u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 2.0f, 1e-4);
}

TEST(Deskew, OrganizedCloudWithStaleRowStepIsDensePackedToo)
{
  // The dense-packing fallback does not depend on height: height=2, width=2
  // with row_step 16 (< 32) is read as two dense rows of 32 bytes. All four
  // points move to the reference pose, including row 1's, which a stride of
  // 16 would have placed inside row 0.
  auto cloud = make_cloud_xyzt(
    {{0.0f, 0.0f, 0.0f, 0.1f},
     {0.0f, 0.0f, 0.0f, 0.1f},
     {0.0f, 0.0f, 0.0f, 0.1f},
     {0.0f, 0.0f, 0.0f, 0.1f}});
  cloud.height = 2;
  cloud.width = 2;
  cloud.row_step = cloud.point_step;  // 16, but width*point_step = 32
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, /*t_ref_ns=*/0, traj);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.points_deskewed, 4u);
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_NEAR(xyz_at(*r.cloud, i)[0], 2.0f, 1e-4) << "point " << i;
  }
}

TEST(Deskew, BlobShorterThanDeclaredPointsIsError)
{
  // Unlike the movify cloud walks, deskew rewrites the cloud, so a blob that
  // does not hold every declared point stays a hard error rather than being
  // trimmed: the output would otherwise be a cloud whose header promises
  // points its data does not carry.
  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}, {0, 0, 0, 0}});
  cloud.data.pop_back();
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("too small"), std::string::npos);
}

TEST(Deskew, HugeWidthTimesPointStepIsRejectedNotWrapped)
{
  // width * point_step (0x2000'0000 * 16 = 2^33) overflows 32 bits. The
  // stride must be computed in std::size_t so the cloud is rejected on data
  // size rather than wrapped into a small, accepted stride.
  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}, {0, 0, 0, 0}});
  cloud.width = 0x2000'0000U;
  cloud.row_step = 0;
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
}

TEST(Deskew, HeightTimesRowStrideDoesNotWrap)
{
  // A header declaring height 2 and a width * point_step just over 2^63
  // needs ~18.4 EB of data, but the product height * stride wraps std::size_t
  // to ~16 MB -- so a 20 MB blob would pass a multiplied size check and the
  // row-1 walk would then index ~9.2 EB past it. The check must compare
  // without multiplying.
  const std::uint32_t width = 3037012209U;
  const std::uint32_t point_step = 3036988791U;
  const std::size_t row_bytes = static_cast<std::size_t>(width) * point_step;
  ASSERT_GT(row_bytes, SIZE_MAX / 2);
  ASSERT_LT(static_cast<std::size_t>(2) * row_bytes, row_bytes);  // unsigned wrap

  auto cloud = make_cloud_xyzt({{0, 0, 0, 0}});
  cloud.height = 2;
  cloud.width = width;
  cloud.point_step = point_step;
  cloud.row_step = 0;
  cloud.data.assign(20U * 1024 * 1024, std::byte{0});
  auto r = deskew_pointcloud2(cloud, 0, std::vector<TrajectoryPose>{{0, 0, 0, 0, 0, 0, 0, 1}});
  EXPECT_FALSE(r.ok());
}

TEST(Deskew, OrganizedCloudWithRowPadding)
{
  // height=2, width=1, but row_step (32) is LARGER than width*point_step
  // (16): 16 bytes of padding after each row's single point. Complements
  // OrganizedCloudHeightTwoWidthOne (row_step == width*point_step, no
  // padding) by confirming the padding bytes are correctly skipped rather
  // than read as point data.
  PointCloud2 c;
  c.height = 2;
  c.width = 1;
  c.point_step = 16;
  c.row_step = 32;  // 16 bytes of padding per row
  c.is_bigendian = false;
  c.is_dense = true;
  c.frame_id = "lidar";
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"t", 12, PointFieldType::kFloat32, 1},
  };
  c.data.assign(static_cast<std::size_t>(c.row_step) * c.height, std::byte{0});
  const std::array<float, 4> row0{0.0f, 0.0f, 0.0f, 0.1f};  // PureTranslation-style point
  const std::array<float, 4> row1{1.0f, 2.0f, 3.0f, 0.0f};  // RefTimeUnchanged-style point
  std::memcpy(c.data.data(), row0.data(), sizeof(float) * 4);
  std::memcpy(c.data.data() + c.row_step, row1.data(), sizeof(float) * 4);

  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(c, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 2u);

  std::array<float, 3> xyz0{};
  std::memcpy(xyz0.data(), r.cloud->data.data(), sizeof(float) * 3);
  std::array<float, 3> xyz1{};
  std::memcpy(xyz1.data(), r.cloud->data.data() + c.row_step, sizeof(float) * 3);
  EXPECT_NEAR(xyz0[0], 2.0f, 1e-4);  // row 0: moved to ref pose
  EXPECT_NEAR(xyz1[0], 1.0f, 1e-5);  // row 1: t==t_ref, unchanged
}

TEST(Deskew, NonMonotonicPointTimes)
{
  // Point times jump backwards mid-scan (0.1 -> 0.05 -> 0.15 s relative):
  // exercises the trajectory lookup's non-monotone handling (each point must
  // still resolve its own pose, independent of scan order). The trajectory is
  // a pure +20 m/s X translation (2 m per 0.1 s), so a point at relative
  // time t_i maps to x + 20*t_i in the t_ref=0 frame.
  auto cloud = make_cloud_xyzt(
    {{1.0f, 0.0f, 0.0f, 0.1f}, {10.0f, 0.0f, 0.0f, 0.05f}, {100.0f, 0.0f, 0.0f, 0.15f}});
  std::vector<TrajectoryPose> traj{
    {0, 0, 0, 0, 0, 0, 0, 1},
    {100'000'000, 2, 0, 0, 0, 0, 0, 1},
    {200'000'000, 4, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 3u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f + 2.0f, 1e-4);    // t=0.1s: x + 20*0.1
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 10.0f + 1.0f, 1e-4);   // t=0.05s: x + 20*0.05
  EXPECT_NEAR(xyz_at(*r.cloud, 2)[0], 100.0f + 3.0f, 1e-4);  // t=0.15s: x + 20*0.15
}

TEST(Deskew, PointsBeforeSpanAreCountedOutOfSpanAndClamped)
{
  // Trajectory spans [0, 0.1s]; t_ref = 0. The point at relative time -0.05s
  // predates the first pose: it clamps to it and is counted out-of-span. The
  // clamped pose coincides with the ref pose (both are the trajectory front),
  // so the point's xyz is left unchanged. The in-span point at 0.02s moves
  // normally (20 m/s * 0.02 s = 0.4 m).
  auto cloud = make_cloud_xyzt({{1.0f, 0.0f, 0.0f, -0.05f}, {0.0f, 0.0f, 0.0f, 0.02f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 0, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.points_deskewed, 2u);
  EXPECT_EQ(r.points_out_of_span, 1u);
  EXPECT_FALSE(r.ref_out_of_span);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f, 1e-5);  // clamped pose == ref pose: no motion
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 0.4f, 1e-4);  // in-span: x + 20*0.02
}

TEST(Deskew, RefAndPointsPastSpanAreReportedOutOfSpan)
{
  // t_ref = 0.2s, past the last sample at 0.1s: the reference pose clamps to
  // the back pose, and both points (t_i = 0.2s / 0.25s) clamp likewise, so the
  // whole cloud is effectively left un-deskewed — reported, not silent.
  auto cloud = make_cloud_xyzt({{1.0f, 0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f, 0.05f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  auto r = deskew_pointcloud2(cloud, 200'000'000, traj);
  ASSERT_TRUE(r.ok());
  EXPECT_TRUE(r.ref_out_of_span);
  EXPECT_EQ(r.points_out_of_span, 2u);
  EXPECT_NEAR(xyz_at(*r.cloud, 0)[0], 1.0f, 1e-5);  // both poses clamp to back: no motion
  EXPECT_NEAR(xyz_at(*r.cloud, 1)[0], 2.0f, 1e-5);
}

// --- deskew_pointcloud2_cdr: the in-place serialized-payload variant ---------

namespace
{

using bagwiz::core::pointcloud::deskew_pointcloud2_cdr;
using bagwiz::core::pointcloud::parse_pointcloud2;
using bagwiz::core::pointcloud::serialize_pointcloud2;

// Deskew `cloud` through both paths -- parse -> deskew -> serialize, and the
// in-place CDR patch of the serialized payload -- and require byte-identical
// output plus identical counters. Exact equality is justified: both paths run
// the same per-point kernel over the same immutable inputs; the only
// difference is where the untouched bytes travel. The CDR variant reads its
// reference stamp from the message header, so the struct path is driven with
// cloud.timestamp_ns to match.
void expect_cdr_matches_struct_path(
  const PointCloud2 & cloud, const std::vector<TrajectoryPose> & traj,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic = std::nullopt,
  bool keep_point_time = false)
{
  const auto payload = serialize_pointcloud2(cloud);

  auto parsed = parse_pointcloud2(payload);
  ASSERT_TRUE(parsed.ok());
  auto ref = deskew_pointcloud2(
    std::move(*parsed.cloud), cloud.timestamp_ns, traj, extrinsic, keep_point_time);
  ASSERT_TRUE(ref.ok());
  const auto want = serialize_pointcloud2(*ref.cloud);

  std::vector<std::byte> patched = payload;
  const auto got = deskew_pointcloud2_cdr(
    std::span<std::byte>(patched.data(), patched.size()), traj, extrinsic, keep_point_time);
  ASSERT_TRUE(got.ok()) << got.parse_error << " / " << got.error;
  EXPECT_EQ(got.t_ref_ns, cloud.timestamp_ns);
  EXPECT_EQ(patched, want);
  EXPECT_EQ(got.points_total, ref.points_total);
  EXPECT_EQ(got.points_deskewed, ref.points_deskewed);
  EXPECT_EQ(got.points_no_time, ref.points_no_time);
  EXPECT_EQ(got.points_no_pose, ref.points_no_pose);
  EXPECT_EQ(got.points_nonfinite, ref.points_nonfinite);
  EXPECT_EQ(got.points_out_of_span, ref.points_out_of_span);
  EXPECT_EQ(got.ref_out_of_span, ref.ref_out_of_span);
}

}  // namespace

TEST(DeskewCdr, MatchesStructPathOnFloat32Cloud)
{
  // Mixed coverage in one cloud: a normal in-span point, a non-finite point,
  // an out-of-span point, and non-monotonic times; non-zero header stamp.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto cloud = make_cloud_xyzt(
    {{0.0f, 0.0f, 0.0f, 0.1f},
     {nan, nan, nan, 0.05f},
     {1.0f, 0.0f, 0.0f, -0.05f},
     {0.5f, 0.5f, 0.0f, 0.02f}});
  cloud.timestamp_ns = 2'000'000'000;
  std::vector<TrajectoryPose> traj{
    {2'000'000'000, 0, 0, 0, 0, 0, 0, 1}, {2'100'000'000, 2, 0, 0, 0, 0, 0.3826834, 0.9238795}};
  expect_cdr_matches_struct_path(cloud, traj);
}

TEST(DeskewCdr, MatchesStructPathOnStaleRowStepCloud)
{
  // The stale row_step travels verbatim through serialize/parse, so the CDR
  // path must apply the same dense-packing fallback as the struct path.
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}, {1.0f, 2.0f, 3.0f, 0.0f}});
  cloud.row_step = cloud.point_step;  // 16, but width*point_step = 32
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  expect_cdr_matches_struct_path(cloud, traj);
}

TEST(DeskewCdr, MatchesStructPathOnFloat64Cloud)
{
  auto cloud = make_cloud_xyzt_f64({{0.0, 0.0, 0.0, 0.1}, {1.0, 2.0, 3.0, 0.0}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  expect_cdr_matches_struct_path(cloud, traj);
}

TEST(DeskewCdr, MatchesStructPathOnU32TimeCloud)
{
  auto cloud = make_cloud_xyz_u32time(0.0f, 0.0f, 0.0f, 100'000'000u);
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  expect_cdr_matches_struct_path(cloud, traj);
}

TEST(DeskewCdr, MatchesStructPathWithExtrinsic)
{
  auto cloud = make_cloud_xyzt({{1.0f, 0.0f, 0.0f, 0.1f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  geometry_msgs::msg::Transform e;
  e.translation.x = 0.5;
  e.rotation.z = 0.7071068;
  e.rotation.w = 0.7071068;
  expect_cdr_matches_struct_path(cloud, traj, e);
}

// keep_point_time must reach the shared kernel identically through both
// entry points -- the command's parallel path runs the CDR variant, the
// struct variant backs the unit tests above.
TEST(DeskewCdr, MatchesStructPathWithKeepPointTime)
{
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}, {1.0f, 0.0f, 0.0f, -0.05f}});
  cloud.timestamp_ns = 2'000'000'000;
  std::vector<TrajectoryPose> traj{
    {2'000'000'000, 0, 0, 0, 0, 0, 0, 1}, {2'100'000'000, 2, 0, 0, 0, 0, 0, 1}};
  expect_cdr_matches_struct_path(cloud, traj, std::nullopt, /*keep_point_time=*/true);
}

// The bytes the flag is about: the patched payload's xyz moved, its per-point
// time did not. Asserted on the CDR path because that is what the command
// actually writes into the output bag.
TEST(DeskewCdr, KeepPointTimeLeavesTimeBytesInPatchedPayload)
{
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.1f}});
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};

  const auto payload = serialize_pointcloud2(cloud);
  std::vector<std::byte> patched = payload;
  const auto got = deskew_pointcloud2_cdr(
    std::span<std::byte>(patched.data(), patched.size()), traj, std::nullopt,
    /*keep_point_time=*/true);
  ASSERT_TRUE(got.ok()) << got.parse_error << " / " << got.error;
  EXPECT_EQ(got.points_deskewed, 1u);

  auto reparsed = parse_pointcloud2(patched);
  ASSERT_TRUE(reparsed.ok());
  EXPECT_NEAR(xyz_at(*reparsed.cloud, 0)[0], 2.0f, 1e-4);
  EXPECT_NEAR(t_at(*reparsed.cloud, 0), 0.1f, 1e-6);
}

TEST(DeskewCdr, NoTimeFieldLeavesPayloadVerbatimWithCounter)
{
  auto cloud = make_cloud_xyzt({{1.0f, 2.0f, 3.0f, 0.5f}});
  cloud.fields[3].name = "intensity";  // no recognised time field left
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}, {100'000'000, 2, 0, 0, 0, 0, 0, 1}};

  const auto payload = serialize_pointcloud2(cloud);
  std::vector<std::byte> patched = payload;
  const auto got =
    deskew_pointcloud2_cdr(std::span<std::byte>(patched.data(), patched.size()), traj);
  ASSERT_TRUE(got.ok());
  EXPECT_EQ(got.points_no_time, 1u);
  EXPECT_EQ(got.points_deskewed, 0u);
  EXPECT_EQ(patched, payload);  // nothing rewritten
}

TEST(DeskewCdr, UndecodablePayloadReportsParseErrorAndLeavesBytes)
{
  std::vector<std::byte> junk(7, std::byte{0x5A});
  const auto before = junk;
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}};
  const auto got = deskew_pointcloud2_cdr(std::span<std::byte>(junk.data(), junk.size()), traj);
  EXPECT_FALSE(got.ok());
  EXPECT_FALSE(got.parse_error.empty());
  EXPECT_TRUE(got.error.empty());
  EXPECT_EQ(junk, before);
}

TEST(DeskewCdr, RejectsBigEndianWithValidationError)
{
  auto cloud = make_cloud_xyzt({{0.0f, 0.0f, 0.0f, 0.0f}});
  cloud.is_bigendian = true;
  std::vector<TrajectoryPose> traj{{0, 0, 0, 0, 0, 0, 0, 1}};

  const auto payload = serialize_pointcloud2(cloud);
  std::vector<std::byte> patched = payload;
  const auto got =
    deskew_pointcloud2_cdr(std::span<std::byte>(patched.data(), patched.size()), traj);
  EXPECT_FALSE(got.ok());
  EXPECT_TRUE(got.parse_error.empty());
  EXPECT_FALSE(got.error.empty());
  EXPECT_EQ(patched, payload);
}
