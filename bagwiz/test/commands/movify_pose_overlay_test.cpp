// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_pose_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::draw_pose_polylines;
using bagwiz::commands::draw_pose_tiles;
using bagwiz::commands::kPoseTileLengthRatio;
using bagwiz::commands::kPoseTileSpacingM;
using bagwiz::commands::load_pose_overlay;
using bagwiz::commands::pose_polyline_in_frame;
using bagwiz::commands::pose_runs;
using bagwiz::commands::pose_tiles_in_frame;
using bagwiz::commands::pose_ui_scale;
using bagwiz::commands::ProjectedPoseTile;
using bagwiz::test::kMovifyOdometryType;
using bagwiz::test::kMovifyPoseStampedType;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::movify_odometry_payload;
using bagwiz::test::movify_pose_stamped_payload;
using bagwiz::test::MovifyTmpDirTest;

constexpr std::int64_t kNs = 1'000'000'000LL;
constexpr double kTol = 1e-9;

// A static edge: `child` sits at (x, y, z) in `parent`, with no rotation.
geometry_msgs::msg::TransformStamped static_edge(
  const std::string & parent, const std::string & child, double x, double y, double z)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.child_frame_id = child;
  ts.transform.translation.x = x;
  ts.transform.translation.y = y;
  ts.transform.translation.z = z;
  ts.transform.rotation.w = 1.0;
  return ts;
}

class MovifyPoseOverlayTest : public MovifyTmpDirTest
{
protected:
  // /odom: base_link driving along +x in `map` at 1 m/s, one pose per
  // second from t = 1 s to t = 10 s (x = 1..10); /tf_static: the lidar 2 m
  // ahead of base_link.
  std::filesystem::path write_odometry_bag(bool with_static_tf = true)
  {
    const auto path = tmp_dir_ / "in.mcap";
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/odom", kMovifyOdometryType);
    if (with_static_tf) {
      w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
      const std::vector<geometry_msgs::msg::TransformStamped> edges{
        static_edge("base_link", "lidar", 2.0, 0.0, 0.0)};
      w->write("/tf_static", 0, bagwiz::core::serialize_tf_message(edges));
    }
    for (int i = 1; i <= 10; ++i) {
      w->write(
        "/odom", i * kNs, movify_odometry_payload(i * kNs, "map", "base_link", i, 0.0, 0.0, 0.0));
    }
    w->close();
    return path;
  }
};

TEST_F(MovifyPoseOverlayTest, LoadsAnOdometryTopicAsTheChildFramesTrajectory)
{
  const auto bag = write_odometry_bag();
  const auto loaded = load_pose_overlay(bag, "/odom", std::nullopt, 10.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  EXPECT_EQ(loaded.overlay->world_frame, "map");
  EXPECT_EQ(loaded.overlay->body_frame, "base_link");  // Odometry's child frame
  EXPECT_DOUBLE_EQ(loaded.overlay->window_s, 10.0);
  ASSERT_EQ(loaded.overlay->poses.size(), 10u);
  EXPECT_EQ(loaded.overlay->poses.front().timestamp_ns, kNs);
  EXPECT_DOUBLE_EQ(loaded.overlay->poses.front().tx, 1.0);
  EXPECT_DOUBLE_EQ(loaded.overlay->poses.back().tx, 10.0);
}

TEST_F(MovifyPoseOverlayTest, LoadsAPoseStampedTopicForTheAssertedBody)
{
  const auto path = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/pose", kMovifyPoseStampedType);
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const std::vector<geometry_msgs::msg::TransformStamped> edges{
      static_edge("base_link", "lidar", 2.0, 0.0, 0.0),
      static_edge("vehicle", "imu", 0.0, 0.0, 1.0)};
    w->write("/tf_static", 0, bagwiz::core::serialize_tf_message(edges));
    for (int i = 1; i <= 3; ++i) {
      w->write("/pose", i * kNs, movify_pose_stamped_payload(i * kNs, "map", 0.0, i, 0.0, 0.0));
    }
    w->close();
  }
  const auto loaded = load_pose_overlay(path, "/pose", std::string{"vehicle"}, 5.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  EXPECT_EQ(loaded.overlay->body_frame, "vehicle");
  ASSERT_EQ(loaded.overlay->poses.size(), 3u);
  EXPECT_DOUBLE_EQ(loaded.overlay->poses[2].ty, 3.0);
  // No --pose-of and no child frame: base_link is assumed.
  const auto assumed = load_pose_overlay(path, "/pose", std::nullopt, 5.0);
  ASSERT_TRUE(assumed.ok()) << assumed.error;
  EXPECT_EQ(assumed.overlay->body_frame, "base_link");
}

TEST_F(MovifyPoseOverlayTest, RejectsAMissingOrWrongTopic)
{
  const auto bag = write_odometry_bag();
  const auto missing = load_pose_overlay(bag, "/nowhere", std::nullopt, 10.0);
  EXPECT_FALSE(missing.ok());
  EXPECT_EQ(missing.error, "pose topic '/nowhere' not found in " + bag.string());
  const auto wrong = load_pose_overlay(bag, "/tf_static", std::nullopt, 10.0);
  EXPECT_FALSE(wrong.ok());
  EXPECT_NE(wrong.error.find("has type 'tf2_msgs/msg/TFMessage'"), std::string::npos)
    << wrong.error;
}

TEST_F(MovifyPoseOverlayTest, RejectsATopicWithoutMessages)
{
  const auto path = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/odom", kMovifyOdometryType);
    w->close();
  }
  const auto loaded = load_pose_overlay(path, "/odom", std::nullopt, 10.0);
  EXPECT_FALSE(loaded.ok());
  EXPECT_EQ(loaded.error, "topic '/odom' has no messages to render.");
}

// At t = 5.5 s the body sits at x = 5.5; with a +-2 s window the past runs
// from x = 4 (t = 4) up to the body and the future from the body to x = 7
// (t = 7). In the body's own frame the trajectory is the x axis.
TEST_F(MovifyPoseOverlayTest, PolylineInTheBodyFrameIsCenteredOnTheBody)
{
  const auto bag = write_odometry_bag();
  const auto loaded = load_pose_overlay(bag, "/odom", std::nullopt, 2.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto line = pose_polyline_in_frame(*loaded.overlay, "base_link", kNs * 5 + kNs / 2, error);
  ASSERT_TRUE(line.has_value()) << error;
  // past: t = 4, 5, then the body; future: the body, then t = 6, 7.
  ASSERT_EQ(line->past.size(), 3u);
  EXPECT_NEAR(line->past[0][0], -1.5, kTol);
  EXPECT_NEAR(line->past[1][0], -0.5, kTol);
  EXPECT_NEAR(line->past[2][0], 0.0, kTol);
  ASSERT_EQ(line->future.size(), 3u);
  EXPECT_NEAR(line->future[0][0], 0.0, kTol);
  EXPECT_NEAR(line->future[1][0], 0.5, kTol);
  EXPECT_NEAR(line->future[2][0], 1.5, kTol);
  for (const auto & p : line->past) {
    EXPECT_NEAR(p[1], 0.0, kTol);
    EXPECT_NEAR(p[2], 0.0, kTol);
  }
}

// In the lidar frame (2 m ahead of the body) everything shifts back by 2 m:
// the body sits at x = -2.
TEST_F(MovifyPoseOverlayTest, PolylineInAnotherFrameGoesThroughTheStaticTf)
{
  const auto bag = write_odometry_bag();
  const auto loaded = load_pose_overlay(bag, "/odom", std::nullopt, 2.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto line = pose_polyline_in_frame(*loaded.overlay, "lidar", kNs * 5, error);
  ASSERT_TRUE(line.has_value()) << error;
  EXPECT_NEAR(line->past.back()[0], -2.0, kTol);
  EXPECT_NEAR(line->future.front()[0], -2.0, kTol);
  EXPECT_NEAR(line->future.back()[0], 0.0, kTol);  // t = 7: x = 7, i.e. 2 m ahead of the body
  // A frame the static TF does not reach is an error.
  EXPECT_FALSE(pose_polyline_in_frame(*loaded.overlay, "nowhere", kNs * 5, error).has_value());
  EXPECT_NE(error.find("no static TF chain"), std::string::npos) << error;
}

// The body's yaw turns the world into the body frame: driving along +x while
// yawed 90 degrees left, the road ahead (+x in the world) lies to the body's
// right (-y).
TEST_F(MovifyPoseOverlayTest, PolylineFollowsTheBodyOrientation)
{
  const auto path = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/odom", kMovifyOdometryType);
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const std::vector<geometry_msgs::msg::TransformStamped> edges{
      static_edge("base_link", "lidar", 2.0, 0.0, 0.0)};
    w->write("/tf_static", 0, bagwiz::core::serialize_tf_message(edges));
    for (int i = 1; i <= 3; ++i) {
      w->write(
        "/odom", i * kNs,
        movify_odometry_payload(i * kNs, "map", "base_link", i, 0.0, 0.0, M_PI / 2.0));
    }
    w->close();
  }
  const auto loaded = load_pose_overlay(path, "/odom", std::nullopt, 5.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto line = pose_polyline_in_frame(*loaded.overlay, "base_link", 2 * kNs, error);
  ASSERT_TRUE(line.has_value()) << error;
  ASSERT_EQ(line->future.size(), 2u);
  EXPECT_NEAR(line->future[1][0], 0.0, kTol);
  EXPECT_NEAR(line->future[1][1], -1.0, kTol);
}

// A frame the bag's static TF does not know cannot be placed in any panel:
// loading stops with the frame named — an Odometry's child frame ("ins"),
// --pose-of or not, and the body a pose topic is taken as. A bag without
// static TF is the same case.
TEST_F(MovifyPoseOverlayTest, RejectsABodyFrameTheStaticTfDoesNotKnow)
{
  const auto path = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/odom", kMovifyOdometryType);
    movify_declare_topic(*w, "/pose", kMovifyPoseStampedType);
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const std::vector<geometry_msgs::msg::TransformStamped> edges{
      static_edge("base_link", "lidar", 2.0, 0.0, 0.0)};
    w->write("/tf_static", 0, bagwiz::core::serialize_tf_message(edges));
    for (int i = 1; i <= 3; ++i) {
      w->write("/odom", i * kNs, movify_odometry_payload(i * kNs, "map", "ins", i, 0.0, 0.0, 0.0));
      w->write("/pose", i * kNs, movify_pose_stamped_payload(i * kNs, "map", i, 0.0, 0.0, 0.0));
    }
    w->close();
  }
  for (const auto & body :
       {std::optional<std::string>{}, std::optional<std::string>{"base_link"}}) {
    const auto loaded = load_pose_overlay(path, "/odom", body, 5.0);
    EXPECT_FALSE(loaded.ok());
    EXPECT_NE(
      loaded.error.find("Odometry child frame 'ins' is not in the bag's static TF"),
      std::string::npos)
      << loaded.error;
    EXPECT_NE(loaded.error.find("tf static update"), std::string::npos) << loaded.error;
  }
  // The pose topic taken as an unknown body.
  const auto asserted = load_pose_overlay(path, "/pose", std::string{"vehicle"}, 5.0);
  EXPECT_FALSE(asserted.ok());
  EXPECT_NE(asserted.error.find("'vehicle' is not in the bag's static TF"), std::string::npos)
    << asserted.error;
  // ... while base_link (the default) is known.
  EXPECT_TRUE(load_pose_overlay(path, "/pose", std::nullopt, 5.0).ok());

  // No static TF at all: nothing is known.
  const auto bare = tmp_dir_ / "bare.mcap";
  {
    auto w = bagwiz::io::open_write(bare, movify_mcap_options());
    movify_declare_topic(*w, "/pose", kMovifyPoseStampedType);
    w->write("/pose", kNs, movify_pose_stamped_payload(kNs, "map", 0.0, 0.0, 0.0, 0.0));
    w->close();
  }
  const auto no_tf = load_pose_overlay(bare, "/pose", std::nullopt, 5.0);
  EXPECT_FALSE(no_tf.ok());
  EXPECT_NE(no_tf.error.find("'base_link' is not in the bag's static TF"), std::string::npos)
    << no_tf.error;
}

TEST(PoseRuns, BreakAtPointsThatDidNotProject)
{
  const std::vector<std::optional<cv::Point>> points{cv::Point(0, 0), cv::Point(1, 1), std::nullopt,
                                                     cv::Point(5, 5), std::nullopt,    std::nullopt,
                                                     cv::Point(7, 7), cv::Point(8, 8)};
  const auto runs = pose_runs(points);
  ASSERT_EQ(runs.size(), 3u);
  EXPECT_EQ(runs[0].size(), 2u);
  EXPECT_EQ(runs[1].size(), 1u);
  EXPECT_EQ(runs[2].size(), 2u);
  EXPECT_TRUE(pose_runs({}).empty());
}

TEST(DrawPosePolylines, DrawsTheRunsInTheirColors)
{
  cv::Mat canvas(40, 80, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_pose_polylines(
    canvas, {{cv::Point(0, 20), cv::Point(30, 20)}}, {{cv::Point(40, 20), cv::Point(79, 20)}},
    pose_ui_scale(720));
  const auto past = canvas.at<cv::Vec3b>(20, 15);
  const auto future = canvas.at<cv::Vec3b>(20, 60);
  EXPECT_EQ(past, cv::Vec3b(210, 210, 210));
  EXPECT_EQ(future, cv::Vec3b(0, 170, 255));
  EXPECT_EQ(canvas.at<cv::Vec3b>(5, 15), cv::Vec3b(0, 0, 0));  // untouched elsewhere
  // A one-point run draws nothing.
  cv::Mat blank(40, 80, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_pose_polylines(blank, {{cv::Point(10, 10)}}, {}, 1.0);
  EXPECT_EQ(blank.at<cv::Vec3b>(10, 10), cv::Vec3b(0, 0, 0));
}

// Plates draw farthest first whatever their order in the list: where a near
// plate ahead overlaps a far plate behind, the near one's color is on top.
TEST(DrawPoseTiles, DrawsFarPlatesFirst)
{
  cv::Mat canvas(40, 80, CV_8UC3, cv::Scalar(0, 0, 0));
  ProjectedPoseTile near;
  near.corners = {cv::Point(10, 10), cv::Point(40, 10), cv::Point(40, 30), cv::Point(10, 30)};
  near.ahead = true;
  near.fade = 1.0;
  near.depth = 5.0;
  ProjectedPoseTile far;
  far.corners = {cv::Point(30, 10), cv::Point(70, 10), cv::Point(70, 30), cv::Point(30, 30)};
  far.ahead = false;
  far.fade = 1.0;
  far.depth = 50.0;
  draw_pose_tiles(canvas, {near, far}, 1.0);  // the near plate listed first
  cv::Mat other(40, 80, CV_8UC3, cv::Scalar(0, 0, 0));
  draw_pose_tiles(other, {far, near}, 1.0);  // and last
  // Both orders give the same picture, with the near plate's orange on top
  // in the overlap (its red channel dominates the grey it covers).
  const auto overlap = canvas.at<cv::Vec3b>(20, 35);
  EXPECT_EQ(overlap, other.at<cv::Vec3b>(20, 35));
  EXPECT_GT(static_cast<int>(overlap[2]), static_cast<int>(overlap[0]) + 40);
}

TEST(PoseUiScale, FollowsTheCellHeightWithinBounds)
{
  EXPECT_DOUBLE_EQ(pose_ui_scale(720), 1.0);
  EXPECT_DOUBLE_EQ(pose_ui_scale(2160), 3.0);
  EXPECT_DOUBLE_EQ(pose_ui_scale(100), 0.5);
  EXPECT_DOUBLE_EQ(pose_ui_scale(10000), 4.0);
}

}  // namespace
