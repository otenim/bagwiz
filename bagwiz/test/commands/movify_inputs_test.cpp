// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_inputs.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/io/bag_io.hpp"
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <string>
#include <vector>

// Unit tests for the movify input side: per-view binding parsing, input
// validation, the rectification decision, the pass-1 scan, the pass-2
// geometry, and the encode reader. Exercises movify_inputs.hpp directly
// without driving the CLI.

namespace
{

using bagwiz::commands::clock_topic_of;
using bagwiz::commands::load_video_geometry;
using bagwiz::commands::MovifyArgs;
using bagwiz::commands::open_encode_reader;
using bagwiz::commands::parse_cam_info_entries;
using bagwiz::commands::parse_pcd_bindings;
using bagwiz::commands::scan_video_inputs;
using bagwiz::commands::validate_video_inputs;
using bagwiz::commands::VideoGeometry;
using bagwiz::commands::view_rectifies;
using bagwiz::commands::ViewInput;
using bagwiz::test::kMovifyCameraInfoType;
using bagwiz::test::kMovifyGarbagePayload;
using bagwiz::test::kMovifyImageType;
using bagwiz::test::kMovifyPointCloudType;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::movify_pointcloud2_payload;
using bagwiz::test::movify_write_cloud_bag;
using bagwiz::test::movify_write_image_bag;
using bagwiz::test::MovifyTmpDirTest;

// ---- parse_pcd_bindings -----------------------------------------------------

TEST(ParsePcdBindings, BareValuesAreGlobal)
{
  const std::vector<std::string> entries{"/points/front", "/points/rear"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_pcd_bindings(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.global_topics, entries);
  EXPECT_TRUE(r.per_view.empty());
}

TEST(ParsePcdBindings, PairBindsToOneView)
{
  const std::vector<std::string> entries{"/cam/a=/points/left", "/cam/a=/points/right"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_pcd_bindings(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_TRUE(r.global_topics.empty());
  ASSERT_EQ(r.per_view.count("/cam/a"), 1u);
  EXPECT_EQ(r.per_view.at("/cam/a"), std::vector<std::string>({"/points/left", "/points/right"}));
}

TEST(ParsePcdBindings, RejectsUnknownImageTopic)
{
  const std::vector<std::string> entries{"/cam/nope=/points"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_pcd_bindings(entries, images);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("not one of the --cam topics"), std::string::npos);
}

TEST(ParsePcdBindings, RejectsEmptyHalves)
{
  const std::vector<std::string> images{"/cam/a"};
  for (const char * entry : {"=/points", "/cam/a=", "="}) {
    const std::vector<std::string> entries{entry};
    EXPECT_FALSE(parse_pcd_bindings(entries, images).ok()) << entry;
  }
}

// ---- parse_cam_info_entries -------------------------------------------------

TEST(ParseCamInfoEntries, BareValueIsGlobal)
{
  const std::vector<std::string> entries{"/cam/camera_info"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_cam_info_entries(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_TRUE(r.global_topic.has_value());
  EXPECT_EQ(*r.global_topic, "/cam/camera_info");
  EXPECT_TRUE(r.per_view.empty());
}

TEST(ParseCamInfoEntries, PairOverridesOneView)
{
  const std::vector<std::string> entries{"/cam/a=/cam/a_info"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_cam_info_entries(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_FALSE(r.global_topic.has_value());
  ASSERT_EQ(r.per_view.count("/cam/a"), 1u);
  EXPECT_EQ(r.per_view.at("/cam/a"), "/cam/a_info");
}

TEST(ParseCamInfoEntries, RejectsSecondBareValue)
{
  const std::vector<std::string> entries{"/cam/info_a", "/cam/info_b"};
  const std::vector<std::string> images{"/cam/a"};
  EXPECT_FALSE(parse_cam_info_entries(entries, images).ok());
}

TEST(ParseCamInfoEntries, RejectsDuplicateOverride)
{
  const std::vector<std::string> entries{"/cam/a=/cam/i1", "/cam/a=/cam/i2"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_cam_info_entries(entries, images);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("duplicate override"), std::string::npos);
}

TEST(ParseCamInfoEntries, RejectsUnknownImageTopicAndEmptyHalves)
{
  const std::vector<std::string> images{"/cam/a"};
  for (const char * entry : {"/cam/nope=/cam/i", "=/cam/i", "/cam/a="}) {
    const std::vector<std::string> entries{entry};
    EXPECT_FALSE(parse_cam_info_entries(entries, images).ok()) << entry;
  }
}

// ---- view_rectifies -----------------------------------------------------------

TEST(ViewRectifies, RequiresRectifyRequested)
{
  ViewInput view;
  view.camera_info_topic = "/cam/camera_info";
  EXPECT_FALSE(view_rectifies(false, view));
}

TEST(ViewRectifies, RequiresResolvedCamInfo)
{
  ViewInput view;
  EXPECT_FALSE(view_rectifies(true, view));
}

TEST(ViewRectifies, RectifiesWhenRequestedAndCamInfoResolved)
{
  ViewInput view;
  view.camera_info_topic = "/cam/camera_info";
  EXPECT_TRUE(view_rectifies(true, view));
}

TEST(ViewRectifies, NoRectifyWinsOverPcd)
{
  // --cam-pcd does not force rectification: with --no-rectify the points are
  // projected onto the raw image using the camera's lens distortion instead.
  ViewInput view;
  view.camera_info_topic = "/cam/camera_info";
  view.pcd_topics = {"/points"};
  EXPECT_TRUE(view_rectifies(true, view));
  EXPECT_FALSE(view_rectifies(false, view));
}

// ---- validate_video_inputs --------------------------------------------------

TEST_F(MovifyTmpDirTest, ValidateInputsUnopenableInput)
{
  MovifyArgs args(tmp_dir_ / "does_not_exist.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_NE(v.error.find("failed to open"), std::string::npos);
}

TEST_F(MovifyTmpDirTest, ValidateInputsTopicNotFound)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/nope", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "topic '/nope' not found in " + bag.string());
}

TEST_F(MovifyTmpDirTest, ValidateInputsPlainImageTopicOk)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 1u);
  EXPECT_EQ(v.views[0].topic_type, kMovifyImageType);
  EXPECT_FALSE(v.views[0].camera_info_topic.has_value());
  EXPECT_EQ(v.grid.cols, 1u);
  EXPECT_EQ(v.grid.rows, 1u);
}

TEST_F(MovifyTmpDirTest, ValidateInputsDuplicateTopicFails)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/image");
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "topic '/cam/image' given more than once");
}

TEST_F(MovifyTmpDirTest, ValidateInputsGridTooSmallFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b", kMovifyImageType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b");
  args.grid = "1x1";
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_NE(v.error.find("1x1"), std::string::npos);
}

TEST_F(MovifyTmpDirTest, ValidateInputsWidthConflictsWithResize)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.width = 640;
  args.resize_scale = 0.5f;
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "--width and --resize are mutually exclusive.");
}

TEST_F(MovifyTmpDirTest, ValidateInputsWidthTooSmallForTheGridFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b", kMovifyImageType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b");
  args.width = 2;  // 2 px across 2 auto-grid columns leaves a sub-2-px cell
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_NE(v.error.find("too small"), std::string::npos);
}

// Rectification is on by default but degrades to a warning when no
// camera-info topic can be derived; validation succeeds and the view renders
// unrectified.
TEST_F(MovifyTmpDirTest, ValidateInputsRectifyWithoutCamInfoWarnsAndContinues)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 1u);
  EXPECT_FALSE(v.views[0].camera_info_topic.has_value());
}

// Point-cloud projection still hard-requires a camera-info topic.
TEST_F(MovifyTmpDirTest, ValidateInputsPcdWithoutCamInfoFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image", kMovifyImageType);
    movify_declare_topic(*w, "/points", kMovifyPointCloudType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.cam_pcd_entries = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "A camera-info topic is required for --cam-pcd, but none could be derived from "
    "'/cam/image'. Pass it explicitly with --cam-info /cam/image=<info_topic>.");
}

TEST_F(MovifyTmpDirTest, ValidateInputsDerivesCamInfoAndAcceptsPcd)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/camera_info", kMovifyCameraInfoType);
    movify_declare_topic(*w, "/points", kMovifyPointCloudType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.cam_pcd_entries = {"/points"};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 1u);
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/camera_info");
  EXPECT_EQ(v.views[0].pcd_topics, std::vector<std::string>({"/points"}));
}

TEST_F(MovifyTmpDirTest, ValidateInputsPerViewPcdBinding)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/a/camera_info", kMovifyCameraInfoType);
    movify_declare_topic(*w, "/cam/b/camera_info", kMovifyCameraInfoType);
    movify_declare_topic(*w, "/points/shared", kMovifyPointCloudType);
    movify_declare_topic(*w, "/points/a_only", kMovifyPointCloudType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a/image_raw", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b/image_raw");
  args.cam_pcd_entries = {"/points/shared", "/cam/a/image_raw=/points/a_only"};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 2u);
  EXPECT_EQ(v.views[0].pcd_topics, std::vector<std::string>({"/points/shared", "/points/a_only"}));
  EXPECT_EQ(v.views[1].pcd_topics, std::vector<std::string>({"/points/shared"}));
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/a/camera_info");
  EXPECT_EQ(v.views[1].camera_info_topic, "/cam/b/camera_info");
}

TEST_F(MovifyTmpDirTest, ValidateInputsPerViewCamInfoOverride)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/a/camera_info", kMovifyCameraInfoType);
    movify_declare_topic(*w, "/custom/b_info", kMovifyCameraInfoType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a/image_raw", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b/image_raw");
  args.camera_info_entries = {"/cam/b/image_raw=/custom/b_info"};
  args.rectify = true;
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/a/camera_info");
  EXPECT_EQ(v.views[1].camera_info_topic, "/custom/b_info");
}

TEST_F(MovifyTmpDirTest, ValidateInputsPcdTopicNotFoundFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/camera_info", kMovifyCameraInfoType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.cam_pcd_entries = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "pcd topic '/points' not found in " + bag.string());
}

TEST_F(MovifyTmpDirTest, ValidateInputsPcdTopicWrongTypeFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/camera_info", kMovifyCameraInfoType);
    movify_declare_topic(*w, "/points", kMovifyImageType);  // wrong type
    w->close();
  }
  MovifyArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.cam_pcd_entries = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "pcd topic '/points' has type 'sensor_msgs/msg/Image', expected sensor_msgs/msg/PointCloud2");
}

TEST_F(MovifyTmpDirTest, ValidateInputsExplicitCamInfoMissingFails)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.camera_info_entries = {"/cam/camera_info"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "camera_info topic '/cam/camera_info' not found in " + bag.string());
}

TEST_F(MovifyTmpDirTest, ValidateInputsClockDefaultsToTheFirstPanel)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b", kMovifyImageType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b");
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.clock, 0u);
  args.clock = "/cam/b";
  const auto named = validate_video_inputs(args);
  ASSERT_TRUE(named.ok()) << named.error;
  EXPECT_EQ(named.clock, 1u);
  EXPECT_EQ(named.views[1].topic, "/cam/b");
}

TEST_F(MovifyTmpDirTest, ValidateInputsWithoutCamTopicsFails)
{
  // The parser leaves --cam optional (the coming panel kinds are alternatives),
  // so an invocation with no panel at all is refused here.
  MovifyArgs args;
  args.input_path = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  args.output_path = tmp_dir_ / "out.avi";
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "nothing to render: pass at least one --cam or --pcd topic.");
}

TEST_F(MovifyTmpDirTest, ValidateInputsClockMustBeACamTopic)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.clock = "/cam/other";
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "--clock '/cam/other' is not one of the --cam or --pcd topics.");
}

// A point-cloud topic alone: one panel per view, the frame and (for a BEV
// view) the extent taken from the first cloud, and the topic as the clock.
TEST_F(MovifyTmpDirTest, ValidateInputsPointCloudPanelsAlone)
{
  const auto bag = movify_write_cloud_bag(tmp_dir_, "in.mcap", "/points", "lidar", 2);
  MovifyArgs args;
  args.input_path = bag;
  args.output_path = tmp_dir_ / "out.avi";
  args.pcd_topics = {"/points"};
  args.views = {
    bagwiz::core::pointcloud::CloudProjection::kPerspective,
    bagwiz::core::pointcloud::CloudProjection::kBev};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_TRUE(v.views.empty());
  EXPECT_EQ(v.pcd_topics, std::vector<std::string>({"/points"}));
  EXPECT_EQ(v.pcd_views.size(), 2u);
  EXPECT_EQ(v.frame, "lidar");
  EXPECT_DOUBLE_EQ(v.range_m, 1.0);  // the first cloud's farthest point
  ASSERT_TRUE(v.clock_pcd.has_value());
  EXPECT_EQ(*v.clock_pcd, 0u);
  EXPECT_EQ(v.clock, 0u);
  EXPECT_EQ(clock_topic_of(v), "/points");
  EXPECT_EQ(v.grid.cols, 2u);  // two panels
  EXPECT_EQ(v.grid.rows, 1u);
}

TEST_F(MovifyTmpDirTest, ValidateInputsHonorsFrameAndRange)
{
  const auto bag = movify_write_cloud_bag(tmp_dir_, "in.mcap", "/points", "lidar", 1);
  MovifyArgs args;
  args.input_path = bag;
  args.output_path = tmp_dir_ / "out.avi";
  args.pcd_topics = {"/points"};
  args.views = {bagwiz::core::pointcloud::CloudProjection::kBev};
  args.frame = "base_link";
  args.range_m = 80.0;
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.frame, "base_link");
  EXPECT_DOUBLE_EQ(v.range_m, 80.0);
}

TEST_F(MovifyTmpDirTest, ValidateInputsRejectsARepeatedView)
{
  const auto bag = movify_write_cloud_bag(tmp_dir_, "in.mcap", "/points", "lidar", 1);
  MovifyArgs args;
  args.input_path = bag;
  args.output_path = tmp_dir_ / "out.avi";
  args.pcd_topics = {"/points"};
  args.views = {
    bagwiz::core::pointcloud::CloudProjection::kBev,
    bagwiz::core::pointcloud::CloudProjection::kBev};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "--view names the same projection more than once.");
}

// A point-cloud topic may be the clock behind camera panels: it drives the
// ticks, and the first point-cloud panel (after the cameras) is the clock
// panel.
TEST_F(MovifyTmpDirTest, ValidateInputsClockMayBeAPointCloudTopic)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image", kMovifyImageType);
    movify_declare_topic(*w, "/points", kMovifyPointCloudType);
    const auto cloud = movify_pointcloud2_payload(1'000'000'000LL, "lidar", {{{1.0F, 0.0F, 0.0F}}});
    w->write("/points", 1'000'000'000LL, cloud);
    w->close();
  }
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.pcd_topics = {"/points"};
  args.clock = "/points";
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_TRUE(v.clock_pcd.has_value());
  EXPECT_EQ(v.clock, 1u);  // after the one camera panel
  EXPECT_EQ(clock_topic_of(v), "/points");
}

TEST_F(MovifyTmpDirTest, ValidateInputsPointCloudTopicWithoutMessagesFails)
{
  const auto bag = movify_write_cloud_bag(tmp_dir_, "in.mcap", "/points", "lidar", 0);
  MovifyArgs args;
  args.input_path = bag;
  args.output_path = tmp_dir_ / "out.avi";
  args.pcd_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "topic '/points' has no messages to render.");
}

// ---- scan_video_inputs ------------------------------------------------------

TEST_F(MovifyTmpDirTest, ScanEmptyTopicFails)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 0);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.error, "topic '/cam/image' has no messages to render.");
}

TEST_F(MovifyTmpDirTest, ScanEmptySecondaryTopicFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/a", kMovifyImageType);
    movify_declare_topic(*w, "/cam/b", kMovifyImageType);
    w->write("/cam/a", 1'000'000'000LL, kMovifyGarbagePayload);
    w->close();
  }
  MovifyArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.cam_topics.push_back("/cam/b");
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.error, "topic '/cam/b' has no messages to render.");
}

TEST_F(MovifyTmpDirTest, ScanDerivesSpanAndFps)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 3);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  ASSERT_TRUE(s.ok()) << s.error;
  EXPECT_EQ(s.span.count, 3);
  EXPECT_EQ(s.span.first_ns, 1'000'000'000LL);
  EXPECT_EQ(s.span.last_ns, 1'200'000'000LL);
  EXPECT_EQ(s.fps.num, 10);
  EXPECT_EQ(s.fps.den, 1);
  EXPECT_TRUE(s.pcd_topics.empty());
  EXPECT_TRUE(s.pcd_spans.empty());
  EXPECT_TRUE(s.pcd_topic_has_stamps.empty());
  EXPECT_EQ(s.global_property_min, 0.0);
  EXPECT_EQ(s.global_property_max, 0.0);
}

// ---- load_video_geometry ----------------------------------------------------

TEST_F(MovifyTmpDirTest, LoadVideoGeometryDefaultsToEmpty)
{
  const auto bag = movify_write_image_bag(tmp_dir_, "in.mcap", 1);
  MovifyArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  VideoGeometry g;
  EXPECT_EQ(load_video_geometry(args, v, g), "");
  ASSERT_EQ(g.camera_infos.size(), 1u);
  EXPECT_FALSE(g.camera_infos[0].has_value());
  EXPECT_FALSE(g.tf_buffer.has_value());
}

TEST_F(MovifyTmpDirTest, LoadVideoGeometryFailsWhenCamInfoUnreadable)
{
  // The cam-info topic is declared but carries no message to load.
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image_raw", kMovifyImageType);
    movify_declare_topic(*w, "/cam/camera_info", kMovifyCameraInfoType);
    w->close();
  }
  MovifyArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.rectify = true;
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  VideoGeometry g;
  EXPECT_FALSE(load_video_geometry(args, v, g).empty());
}

// ---- open_encode_reader -----------------------------------------------------

TEST_F(MovifyTmpDirTest, OpenEncodeReaderMissingBagReturnsNull)
{
  EXPECT_EQ(open_encode_reader(tmp_dir_ / "does_not_exist.mcap", "/cam/image"), nullptr);
}

TEST_F(MovifyTmpDirTest, OpenEncodeReaderFiltersToTheClockTopic)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/cam/image", kMovifyImageType);
    movify_declare_topic(*w, "/other", kMovifyImageType);
    w->write("/cam/image", 1'000'000'000LL, kMovifyGarbagePayload);
    w->write("/other", 1'000'000'000LL, kMovifyGarbagePayload);
    w->close();
  }
  auto reader = open_encode_reader(bag, "/cam/image");
  ASSERT_NE(reader, nullptr);
  bagwiz::io::RawMessage raw;
  ASSERT_TRUE(reader->next(raw));
  EXPECT_EQ(raw.topic->name, "/cam/image");
  EXPECT_FALSE(reader->next(raw));  // the other topic is filtered out of the encode reader
}

}  // namespace
