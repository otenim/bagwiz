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
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::draw_pose_tiles;
using bagwiz::commands::kPoseTileLengthRatio;
using bagwiz::commands::kPoseTileMaxOverhang;
using bagwiz::commands::kPoseTileSpacingM;
using bagwiz::commands::load_pose_overlay;
using bagwiz::commands::pose_tiles_in_frame;
using bagwiz::commands::pose_ui_scale;
using bagwiz::commands::PoseTile;
using bagwiz::commands::PoseTilePlacement;
using bagwiz::commands::project_pose_tiles;
using bagwiz::commands::ProjectedPoseCorner;
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

// At t = 5.5 s the body sits at x = 5.5; with a +-2 s window the path runs
// from x = 4 (t = 4) to x = 7 (t = 7), 1.5 m each side of the body. In the
// body's own frame one plate lies ahead along +x from the body and one
// behind along -x, each a tile length long and `width_m` wide across the
// path; the fade is 1 at the body.
TEST_F(MovifyPoseOverlayTest, TilesInTheBodyFrameStartAtTheBody)
{
  const auto bag = write_odometry_bag();
  const auto loaded = load_pose_overlay(bag, "/odom", std::nullopt, 2.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto tiles =
    pose_tiles_in_frame(*loaded.overlay, "base_link", kNs * 5 + kNs / 2, 2.0, error);
  ASSERT_TRUE(tiles.has_value()) << error;
  ASSERT_EQ(tiles->size(), 2u);
  const double len = kPoseTileSpacingM * kPoseTileLengthRatio;
  const auto & ahead = (*tiles)[0];
  EXPECT_TRUE(ahead.ahead);
  EXPECT_NEAR(ahead.fade, 1.0, kTol);
  EXPECT_NEAR(ahead.corners[0][0], 0.0, kTol);
  EXPECT_NEAR(ahead.corners[0][1], 1.0, kTol);
  EXPECT_NEAR(ahead.corners[1][0], len, kTol);
  EXPECT_NEAR(ahead.corners[1][1], 1.0, kTol);
  EXPECT_NEAR(ahead.corners[2][0], len, kTol);
  EXPECT_NEAR(ahead.corners[2][1], -1.0, kTol);
  EXPECT_NEAR(ahead.corners[3][0], 0.0, kTol);
  EXPECT_NEAR(ahead.corners[3][1], -1.0, kTol);
  const auto & behind = (*tiles)[1];
  EXPECT_FALSE(behind.ahead);
  EXPECT_NEAR(behind.corners[0][0], 0.0, kTol);
  EXPECT_NEAR(behind.corners[1][0], -len, kTol);
  EXPECT_NEAR(std::abs(behind.corners[0][1]), 1.0, kTol);
  for (const auto & tile : *tiles) {
    for (const auto & c : tile.corners) {
      EXPECT_NEAR(c[2], 0.0, kTol);
    }
  }
}

// In the lidar frame (2 m ahead of the body) everything shifts back by 2 m:
// the plate ahead starts at x = -2.
TEST_F(MovifyPoseOverlayTest, TilesInAnotherFrameGoThroughTheStaticTf)
{
  const auto bag = write_odometry_bag();
  const auto loaded = load_pose_overlay(bag, "/odom", std::nullopt, 2.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto tiles = pose_tiles_in_frame(*loaded.overlay, "lidar", kNs * 5, 2.0, error);
  ASSERT_TRUE(tiles.has_value()) << error;
  ASSERT_FALSE(tiles->empty());
  const auto & ahead = tiles->front();
  EXPECT_TRUE(ahead.ahead);
  EXPECT_NEAR(ahead.corners[0][0], -2.0, kTol);
  EXPECT_NEAR(ahead.corners[1][0], -2.0 + kPoseTileSpacingM * kPoseTileLengthRatio, kTol);
  // A frame the static TF does not reach is an error.
  EXPECT_FALSE(pose_tiles_in_frame(*loaded.overlay, "nowhere", kNs * 5, 2.0, error).has_value());
  EXPECT_NE(error.find("no static TF chain"), std::string::npos) << error;
}

// The body's yaw turns the world into the body frame: driving along +x while
// yawed 90 degrees left, the road ahead (+x in the world) lies to the body's
// right (-y), so the plate ahead runs along -y and spans +-1 m in x.
TEST_F(MovifyPoseOverlayTest, TilesFollowTheBodyOrientation)
{
  const auto path = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, "/odom", kMovifyOdometryType);
    w->declare_topic(bagwiz::core::make_tf_message_topic_info("/tf_static"));
    const std::vector<geometry_msgs::msg::TransformStamped> edges{
      static_edge("base_link", "lidar", 2.0, 0.0, 0.0)};
    w->write("/tf_static", 0, bagwiz::core::serialize_tf_message(edges));
    for (int i = 1; i <= 4; ++i) {
      w->write(
        "/odom", i * kNs,
        movify_odometry_payload(i * kNs, "map", "base_link", i, 0.0, 0.0, M_PI / 2.0));
    }
    w->close();
  }
  const auto loaded = load_pose_overlay(path, "/odom", std::nullopt, 5.0);
  ASSERT_TRUE(loaded.ok()) << loaded.error;
  std::string error;
  const auto tiles = pose_tiles_in_frame(*loaded.overlay, "base_link", 2 * kNs, 2.0, error);
  ASSERT_TRUE(tiles.has_value()) << error;
  ASSERT_FALSE(tiles->empty());
  const auto & ahead = tiles->front();
  EXPECT_TRUE(ahead.ahead);
  const double len = kPoseTileSpacingM * kPoseTileLengthRatio;
  EXPECT_NEAR(ahead.corners[0][0], 1.0, kTol);
  EXPECT_NEAR(ahead.corners[0][1], 0.0, kTol);
  EXPECT_NEAR(ahead.corners[1][0], 1.0, kTol);
  EXPECT_NEAR(ahead.corners[1][1], -len, kTol);
  EXPECT_NEAR(ahead.corners[2][0], -1.0, kTol);
  EXPECT_NEAR(ahead.corners[2][1], -len, kTol);
  EXPECT_NEAR(ahead.corners[3][0], -1.0, kTol);
  EXPECT_NEAR(ahead.corners[3][1], 0.0, kTol);
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

// A plate in the panel's frame: 1 m ahead of the origin, a tile long along
// +x and 2 m wide across it, on the ground.
PoseTile unit_tile()
{
  PoseTile tile;
  tile.corners = {{{1.0, 1.0, 0.0}, {2.0, 1.0, 0.0}, {2.0, -1.0, 0.0}, {1.0, -1.0, 0.0}}};
  tile.ahead = false;
  tile.fade = 0.5;
  return tile;
}

// A toy projection: x forward is the depth, y left runs left across the
// picture, z up runs up it, 10 px per meter about the picture's center; a
// point at or behind the camera's plane (x <= 0) does not project.
std::optional<ProjectedPoseCorner> toy_project(const std::array<double, 3> & p)
{
  if (p[0] <= 0.0) {
    return std::nullopt;
  }
  return ProjectedPoseCorner{50.0 - 10.0 * p[1], 50.0 - 10.0 * p[2], p[0]};
}

// Every corner projected lands in the cell, offset by the picture's place in
// it, with the plate's depth the mean of its corners' and its `ahead` / `fade`
// carried over.
TEST(ProjectPoseTiles, ProjectsEveryCornerIntoTheCell)
{
  PoseTilePlacement placement;
  placement.width = 100;
  placement.height = 100;
  placement.x_off = 7;
  placement.y_off = 3;
  const auto projected = project_pose_tiles({unit_tile()}, toy_project, placement);
  ASSERT_EQ(projected.size(), 1u);
  const auto & tile = projected.front();
  EXPECT_EQ(tile.corners[0], cv::Point(47, 53));
  EXPECT_EQ(tile.corners[1], cv::Point(47, 53));
  EXPECT_EQ(tile.corners[2], cv::Point(67, 53));
  EXPECT_EQ(tile.corners[3], cv::Point(67, 53));
  EXPECT_FALSE(tile.ahead);
  EXPECT_DOUBLE_EQ(tile.fade, 0.5);
  EXPECT_DOUBLE_EQ(tile.depth, 1.5);
}

// A plate with a corner that does not project (behind the camera) is dropped
// whole; the others stay.
TEST(ProjectPoseTiles, DropsAPlateWithACornerThatDoesNotProject)
{
  PoseTilePlacement placement;
  placement.width = 100;
  placement.height = 100;
  PoseTile behind = unit_tile();
  behind.corners[0][0] = -1.0;
  const auto projected = project_pose_tiles({behind, unit_tile()}, toy_project, placement);
  ASSERT_EQ(projected.size(), 1u);
  EXPECT_DOUBLE_EQ(projected.front().depth, 1.5);
}

// A corner shooting off past kPoseTileMaxOverhang picture sizes (grazing the
// camera's plane) drops the plate too, as does one that is not finite; a
// corner within the overhang, off the picture, is kept for the drawing to
// clip.
TEST(ProjectPoseTiles, DropsAPlateThatOverhangsThePictureTooFar)
{
  PoseTilePlacement placement;
  placement.width = 100;
  placement.height = 100;
  PoseTile far = unit_tile();
  far.corners[1][1] = (kPoseTileMaxOverhang + 1.0) * 10.0;  // u = -(overhang + 0.5) * 100
  PoseTile bad = unit_tile();
  bad.corners[2][2] = std::numeric_limits<double>::infinity();
  PoseTile edge = unit_tile();
  edge.corners[1][1] = (kPoseTileMaxOverhang - 1.0) * 10.0;  // u = -(overhang - 1.5) * 100
  const auto projected = project_pose_tiles({far, bad, edge}, toy_project, placement);
  ASSERT_EQ(projected.size(), 1u);
  EXPECT_EQ(
    projected.front().corners[1].x,
    static_cast<int>(std::lround(50.0 - (kPoseTileMaxOverhang - 1.0) * 100.0)));
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
