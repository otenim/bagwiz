// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::TrajectoryPose;
using bagwiz::core::write_tum;

TEST(WriteTum, EmitsExpectedLineLayout)
{
  std::vector<TrajectoryPose> poses;
  // 1.5 s, identity rotation at (1, 2, 3).
  poses.push_back({1'500'000'000LL, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});
  // 2.75 s, a quarter turn about Z at (0, 0, 0).
  poses.push_back(
    {2'750'000'000LL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7071067811865475, 0.7071067811865475});

  std::ostringstream os;
  write_tum(os, poses);

  const std::string text = os.str();
  ASSERT_FALSE(text.empty());

  // Nanosecond-precision timestamp + 7 whitespace-separated values per line.
  EXPECT_NE(
    text.find(
      "1.500000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 "
      "0.000000000 1.000000000\n"),
    std::string::npos)
    << "got:\n"
    << text;
  EXPECT_NE(
    text.find(
      "2.750000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 "
      "0.707106781 0.707106781\n"),
    std::string::npos)
    << "got:\n"
    << text;
}

TEST(WriteTum, EmitsNothingForEmptyTrajectory)
{
  std::ostringstream os;
  std::vector<TrajectoryPose> empty;
  write_tum(os, empty);
  EXPECT_EQ(os.str(), "");
}

TEST(WriteTum, RestoresStreamFormatting)
{
  std::ostringstream os;
  os << 0.1;  // default precision
  const std::string before = os.str();

  os.str({});
  os.clear();
  write_tum(os, std::vector<TrajectoryPose>{{1'000'000'000LL, 0, 0, 0, 0, 0, 0, 1}});

  // After writing, the default precision should be restored for the
  // caller so they do not silently inherit fixed/9-digit formatting.
  os.str({});
  os.clear();
  os << 0.1;
  EXPECT_EQ(os.str(), before);
}

// --- Helpers for building decoded message shapes ----------------------

namespace cdr = bagwiz::core::cdr_walker;

cdr::Object make_xyz(double x, double y, double z)
{
  cdr::Object o;
  o.fields.emplace_back("x", cdr::Value{x});
  o.fields.emplace_back("y", cdr::Value{y});
  o.fields.emplace_back("z", cdr::Value{z});
  return o;
}

cdr::Object make_xyzw(double x, double y, double z, double w)
{
  auto o = make_xyz(x, y, z);
  o.fields.emplace_back("w", cdr::Value{w});
  return o;
}

cdr::Object make_header(std::int32_t sec, std::uint32_t nanosec, const std::string & frame_id)
{
  cdr::Object stamp;
  stamp.fields.emplace_back("sec", cdr::Value{sec});
  stamp.fields.emplace_back("nanosec", cdr::Value{nanosec});

  cdr::Object header;
  header.fields.emplace_back("stamp", cdr::Value{std::move(stamp)});
  header.fields.emplace_back("frame_id", cdr::Value{frame_id});
  return header;
}

}  // namespace

// --- extract_pose -----------------------------------------------------

TEST(ExtractPose, TransformStampedShape)
{
  // geometry_msgs/msg/TransformStamped:
  //   header { stamp, frame_id }
  //   string child_frame_id
  //   transform { translation, rotation }
  cdr::Object transform;
  transform.fields.emplace_back("translation", cdr::Value{make_xyz(1.0, 2.0, 3.0)});
  transform.fields.emplace_back("rotation", cdr::Value{make_xyzw(0.0, 0.0, 0.0, 1.0)});

  cdr::Object root;
  root.fields.emplace_back("header", cdr::Value{make_header(7, 500, "map")});
  root.fields.emplace_back("child_frame_id", cdr::Value{std::string{"base_link"}});
  root.fields.emplace_back("transform", cdr::Value{std::move(transform)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, /*fallback=*/123);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_TRUE(extraction->used_header_stamp);
  EXPECT_EQ(extraction->frame_id, "map");
  EXPECT_EQ(extraction->child_frame_id, "base_link");
  EXPECT_EQ(extraction->pose.timestamp_ns, 7'000'000'500LL);
  EXPECT_DOUBLE_EQ(extraction->pose.tx, 1.0);
  EXPECT_DOUBLE_EQ(extraction->pose.tz, 3.0);
  EXPECT_DOUBLE_EQ(extraction->pose.qw, 1.0);
}

TEST(ExtractPose, PoseStampedShape)
{
  // geometry_msgs/msg/PoseStamped: header + pose.{position, orientation}
  cdr::Object pose;
  pose.fields.emplace_back("position", cdr::Value{make_xyz(4.0, 5.0, 6.0)});
  pose.fields.emplace_back("orientation", cdr::Value{make_xyzw(0.0, 0.0, 0.0, 1.0)});

  cdr::Object root;
  root.fields.emplace_back("header", cdr::Value{make_header(0, 0, "odom")});
  root.fields.emplace_back("pose", cdr::Value{std::move(pose)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 999);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_TRUE(extraction->used_header_stamp);
  EXPECT_EQ(extraction->frame_id, "odom");
  EXPECT_TRUE(extraction->child_frame_id.empty());
  EXPECT_DOUBLE_EQ(extraction->pose.tx, 4.0);
  EXPECT_DOUBLE_EQ(extraction->pose.ty, 5.0);
}

TEST(ExtractPose, OdometryShape)
{
  // nav_msgs/msg/Odometry: header + child_frame_id + pose.pose.{position, orientation}
  cdr::Object inner_pose;
  inner_pose.fields.emplace_back("position", cdr::Value{make_xyz(1.0, 0.0, 0.0)});
  inner_pose.fields.emplace_back("orientation", cdr::Value{make_xyzw(0.0, 0.0, 0.7, 0.7)});

  cdr::Object pose;
  pose.fields.emplace_back("pose", cdr::Value{std::move(inner_pose)});
  // Odometry also carries a covariance array but extract_pose ignores it.

  cdr::Object root;
  root.fields.emplace_back("header", cdr::Value{make_header(1, 0, "world")});
  root.fields.emplace_back("child_frame_id", cdr::Value{std::string{"robot"}});
  root.fields.emplace_back("pose", cdr::Value{std::move(pose)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 0);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_EQ(extraction->child_frame_id, "robot");
  EXPECT_EQ(extraction->frame_id, "world");
  EXPECT_DOUBLE_EQ(extraction->pose.qz, 0.7);
}

TEST(ExtractPose, BareTransformUsesFallbackTimestamp)
{
  // geometry_msgs/msg/Transform — no header. Fallback timestamp must
  // flow through and used_header_stamp must be false.
  cdr::Object root;
  root.fields.emplace_back("translation", cdr::Value{make_xyz(0.5, 0.5, 0.5)});
  root.fields.emplace_back("rotation", cdr::Value{make_xyzw(0.0, 0.0, 0.0, 1.0)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 1'234'000'000);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_FALSE(extraction->used_header_stamp);
  EXPECT_TRUE(extraction->frame_id.empty());
  EXPECT_TRUE(extraction->child_frame_id.empty());
  EXPECT_EQ(extraction->pose.timestamp_ns, 1'234'000'000LL);
}

TEST(ExtractPose, BarePoseShape)
{
  cdr::Object root;
  root.fields.emplace_back("position", cdr::Value{make_xyz(0.0, 0.0, 1.0)});
  root.fields.emplace_back("orientation", cdr::Value{make_xyzw(0.0, 0.0, 0.0, 1.0)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 99);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_FALSE(extraction->used_header_stamp);
  EXPECT_DOUBLE_EQ(extraction->pose.tz, 1.0);
}

TEST(ExtractPose, AcceptsFloat32Fields)
{
  // Some writers emit float32 for pose fields; extract_pose must coerce.
  cdr::Object trans;
  trans.fields.emplace_back("x", cdr::Value{1.5F});
  trans.fields.emplace_back("y", cdr::Value{2.5F});
  trans.fields.emplace_back("z", cdr::Value{3.5F});

  cdr::Object rot;
  rot.fields.emplace_back("x", cdr::Value{0.0F});
  rot.fields.emplace_back("y", cdr::Value{0.0F});
  rot.fields.emplace_back("z", cdr::Value{0.0F});
  rot.fields.emplace_back("w", cdr::Value{1.0F});

  cdr::Object root;
  root.fields.emplace_back("translation", cdr::Value{std::move(trans)});
  root.fields.emplace_back("rotation", cdr::Value{std::move(rot)});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 0);
  ASSERT_TRUE(extraction.has_value());
  EXPECT_DOUBLE_EQ(extraction->pose.tx, 1.5);
  EXPECT_DOUBLE_EQ(extraction->pose.qw, 1.0);
}

TEST(ExtractPose, ReturnsNulloptForUnsupportedShape)
{
  // A message that has neither pose / transform / position / translation —
  // extract_pose should return nullopt rather than guess.
  cdr::Object root;
  root.fields.emplace_back("data", cdr::Value{std::string{"hello"}});

  const auto extraction = bagwiz::core::extract_pose(cdr::Value{root}, 0);
  EXPECT_FALSE(extraction.has_value());
}

TEST(ExtractPose, RejectsNonObjectRoot)
{
  // The Value contract for a decoded message is "Object at the root";
  // anything else is a usage error and yields nullopt.
  const auto extraction = bagwiz::core::extract_pose(cdr::Value{cdr::Sequence{}}, 0);
  EXPECT_FALSE(extraction.has_value());
}
