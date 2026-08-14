// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_edit.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/base/tolerance.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::apply_edge_to_buffer;
using bagwiz::commands::apply_edit_key;
using bagwiz::commands::carry_over_edits;
using bagwiz::commands::collect_editable_edges;
using bagwiz::commands::edit_step_preset;
using bagwiz::commands::edit_yaml;
using bagwiz::commands::EditableEdge;
using bagwiz::commands::edited_transform;
using bagwiz::commands::EditPose;
using bagwiz::commands::ExtrinsicEditState;
using bagwiz::commands::is_edited;
using bagwiz::commands::kEditStepDefaultIndex;
using bagwiz::commands::kEditStepPresetCount;
using bagwiz::commands::pose_from_transform;
using bagwiz::commands::pose_to_transform;
using bagwiz::core::KeyEvent;
using bagwiz::core::base::tolerance::kPointMeters;
using bagwiz::core::base::tolerance::kRotationRadians;

geometry_msgs::msg::TransformStamped make_edge(
  const std::string & parent, const std::string & child, double x,
  std::int64_t stamp_ns = 1'000'000'000LL)
{
  geometry_msgs::msg::TransformStamped ts;
  ts.header.frame_id = parent;
  ts.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  ts.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  ts.child_frame_id = child;
  ts.transform.translation.x = x;
  ts.transform.rotation.w = 1.0;
  return ts;
}

EditableEdge make_editable(const std::string & topic, geometry_msgs::msg::TransformStamped ts)
{
  EditableEdge edge;
  edge.topic = topic;
  edge.original = ts;
  edge.edited = pose_from_transform(ts.transform);
  return edge;
}

ExtrinsicEditState one_edge_state(double x = 1.0)
{
  ExtrinsicEditState state;
  state.edges.push_back(make_editable("/tf_static", make_edge("base_link", "lidar", x)));
  state.active = 0;
  return state;
}

TEST(WalkEditPose, RoundTripsThroughTransform)
{
  const EditPose pose{0.5, -1.25, 2.0, 0.1, -0.2, 0.3};
  const auto transform = pose_to_transform(pose);
  const auto back = pose_from_transform(transform);
  EXPECT_NEAR(back.x, pose.x, kPointMeters);
  EXPECT_NEAR(back.y, pose.y, kPointMeters);
  EXPECT_NEAR(back.z, pose.z, kPointMeters);
  EXPECT_NEAR(back.roll, pose.roll, kRotationRadians);
  EXPECT_NEAR(back.pitch, pose.pitch, kRotationRadians);
  EXPECT_NEAR(back.yaw, pose.yaw, kRotationRadians);
}

TEST(WalkEditPose, ZeroPoseIsIdentityTransform)
{
  const auto transform = pose_to_transform(EditPose{});
  EXPECT_DOUBLE_EQ(transform.translation.x, 0.0);
  EXPECT_DOUBLE_EQ(transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(transform.translation.z, 0.0);
  EXPECT_NEAR(transform.rotation.w, 1.0, kRotationRadians);
  EXPECT_NEAR(transform.rotation.x, 0.0, kRotationRadians);
  EXPECT_NEAR(transform.rotation.y, 0.0, kRotationRadians);
  EXPECT_NEAR(transform.rotation.z, 0.0, kRotationRadians);
}

TEST(WalkEditStep, PresetsArePinned)
{
  // Exact comparisons are justified: these are constant table entries, no
  // arithmetic is involved.
  EXPECT_EQ(kEditStepPresetCount, 3U);
  EXPECT_EQ(kEditStepDefaultIndex, 1U);
  EXPECT_DOUBLE_EQ(edit_step_preset(0).translation_m, 0.001);
  EXPECT_DOUBLE_EQ(edit_step_preset(0).rotation_rad, 0.0005);
  EXPECT_DOUBLE_EQ(edit_step_preset(1).translation_m, 0.01);
  EXPECT_DOUBLE_EQ(edit_step_preset(1).rotation_rad, 0.005);
  EXPECT_DOUBLE_EQ(edit_step_preset(2).translation_m, 0.1);
  EXPECT_DOUBLE_EQ(edit_step_preset(2).rotation_rad, 0.05);
}

TEST(WalkEditNudge, TranslationKeysMoveOneStep)
{
  // Exact comparisons are justified: the expected value repeats the same
  // per-element computation on the same immutable inputs.
  auto state = one_edge_state(1.0);
  const double step = edit_step_preset(state.step_index).translation_m;

  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.x, 1.0 + step);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXDown));
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXDown));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.x, 1.0 + step - step - step);

  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransYUp));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.y, step);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransZDown));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.z, -step);
}

TEST(WalkEditNudge, RotationKeysMoveOneStep)
{
  auto state = one_edge_state();
  const double step = edit_step_preset(state.step_index).rotation_rad;

  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditRollUp));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.roll, step);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditPitchDown));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.pitch, -step);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditYawUp));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.yaw, step);
}

TEST(WalkEditNudge, StepKeysClampAtPresetEnds)
{
  auto state = one_edge_state();
  ASSERT_EQ(state.step_index, 1U);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditStepUp));
  EXPECT_EQ(state.step_index, 2U);
  EXPECT_FALSE(apply_edit_key(state, KeyEvent::kEditStepUp));  // already coarsest
  EXPECT_EQ(state.step_index, 2U);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditStepDown));
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditStepDown));
  EXPECT_EQ(state.step_index, 0U);
  EXPECT_FALSE(apply_edit_key(state, KeyEvent::kEditStepDown));  // already finest
  EXPECT_EQ(state.step_index, 0U);
}

TEST(WalkEditNudge, ResetRestoresOriginalPose)
{
  auto state = one_edge_state(1.0);
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditYawUp));
  EXPECT_TRUE(is_edited(state.edges[0]));

  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditReset));
  EXPECT_FALSE(is_edited(state.edges[0]));
  EXPECT_DOUBLE_EQ(state.edges[0].edited.x, 1.0);
  EXPECT_DOUBLE_EQ(state.edges[0].edited.yaw, 0.0);
}

TEST(WalkEditNudge, IgnoresUnrelatedEventsAndEmptyState)
{
  ExtrinsicEditState empty;
  EXPECT_FALSE(apply_edit_key(empty, KeyEvent::kEditTransXUp));
  EXPECT_FALSE(apply_edit_key(empty, KeyEvent::kEditReset));
  // Step changes need no active edge; they still count as handled so the
  // info row repaints. Pin that exception.
  EXPECT_TRUE(apply_edit_key(empty, KeyEvent::kEditStepUp));

  auto state = one_edge_state();
  EXPECT_FALSE(apply_edit_key(state, KeyEvent::kNext));
  EXPECT_FALSE(apply_edit_key(state, KeyEvent::kToggleEditExtrinsic));
  EXPECT_FALSE(is_edited(state.edges[0]));
}

TEST(WalkEditNudge, ActsOnTheActiveEdgeOnly)
{
  auto state = one_edge_state(1.0);
  state.edges.push_back(make_editable("/tf_static", make_edge("base_link", "camera", 2.0)));
  state.active = 1;

  EXPECT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  EXPECT_FALSE(is_edited(state.edges[0]));
  EXPECT_TRUE(is_edited(state.edges[1]));
}

TEST(WalkEditTransform, EditedTransformPreservesEdgeIdentity)
{
  auto state = one_edge_state(1.0);
  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  const auto ts = edited_transform(state.edges[0]);

  EXPECT_EQ(ts.header.frame_id, "base_link");
  EXPECT_EQ(ts.child_frame_id, "lidar");
  EXPECT_EQ(ts.header.stamp.sec, state.edges[0].original.header.stamp.sec);
  EXPECT_EQ(ts.header.stamp.nanosec, state.edges[0].original.header.stamp.nanosec);
  const double step = edit_step_preset(state.step_index).translation_m;
  EXPECT_NEAR(ts.transform.translation.x, 1.0 + step, kPointMeters);
}

TEST(WalkEditBuffer, ApplyOverwritesTheStaticEdge)
{
  // The live-preview premise: re-setting a static (parent, child) pair
  // replaces the value every later lookup composes with.
  tf2::BufferCore buffer;
  buffer.setTransform(make_edge("base_link", "lidar", 1.0), "test", /*is_static=*/true);
  buffer.setTransform(make_edge("base_link", "camera", 2.0), "test", /*is_static=*/true);

  auto state = one_edge_state(1.0);
  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));  // x: 1.0 -> 1.01
  apply_edge_to_buffer(state.edges[0], buffer);

  const auto direct = buffer.lookupTransform("base_link", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(direct.transform.translation.x, 1.01, kPointMeters);
  // The chain the overlay projection actually resolves (camera <- lidar)
  // must see the nudge too: x_lidar_in_camera = 1.01 - 2.0.
  const auto chained = buffer.lookupTransform("camera", "lidar", tf2::TimePointZero);
  EXPECT_NEAR(chained.transform.translation.x, -0.99, kPointMeters);
}

// Buffer + static set used by the collection tests:
//   odom -> base_link   dynamic (on /tf, so NOT editable)
//   base_link -> lidar  static, carried by /tf_static
//   base_link -> lidar2 static, carried by /tf_static
//   odom -> camera      static, carried by /sensing/tf_static
struct CollectFixture
{
  tf2::BufferCore buffer;
  std::vector<bagwiz::core::StaticTopicTransforms> static_topics;
  tf2::TimePoint time{std::chrono::nanoseconds(1'000'000'000LL)};

  CollectFixture()
  {
    const auto lidar = make_edge("base_link", "lidar", 1.0);
    const auto lidar2 = make_edge("base_link", "lidar2", 1.5);
    const auto camera = make_edge("odom", "camera", 2.0);
    buffer.setTransform(lidar, "test", /*is_static=*/true);
    buffer.setTransform(lidar2, "test", /*is_static=*/true);
    buffer.setTransform(camera, "test", /*is_static=*/true);
    buffer.setTransform(make_edge("odom", "base_link", 5.0), "test", /*is_static=*/false);
    static_topics.push_back({"/tf_static", {lidar, lidar2}});
    static_topics.push_back({"/sensing/tf_static", {camera}});
  }
};

TEST(WalkEditCollect, FindsStaticChainEdgesAndSkipsDynamicOnes)
{
  CollectFixture fx;
  const auto edges =
    collect_editable_edges(fx.buffer, {"lidar"}, "camera", fx.time, fx.static_topics);

  // Chain lidar -> base_link -> odom -> camera: the two static edges in
  // chain order; the dynamic odom -> base_link edge is not editable.
  ASSERT_EQ(edges.size(), 2U);
  EXPECT_EQ(edges[0].topic, "/tf_static");
  EXPECT_EQ(edges[0].original.header.frame_id, "base_link");
  EXPECT_EQ(edges[0].original.child_frame_id, "lidar");
  EXPECT_DOUBLE_EQ(edges[0].edited.x, 1.0);
  EXPECT_FALSE(is_edited(edges[0]));
  EXPECT_EQ(edges[1].topic, "/sensing/tf_static");
  EXPECT_EQ(edges[1].original.header.frame_id, "odom");
  EXPECT_EQ(edges[1].original.child_frame_id, "camera");
}

TEST(WalkEditCollect, DeduplicatesEdgesSharedByChains)
{
  CollectFixture fx;
  const auto edges =
    collect_editable_edges(fx.buffer, {"lidar", "lidar2"}, "camera", fx.time, fx.static_topics);

  ASSERT_EQ(edges.size(), 3U);
  EXPECT_EQ(edges[0].original.child_frame_id, "lidar");
  EXPECT_EQ(edges[1].original.child_frame_id, "camera");
  EXPECT_EQ(edges[2].original.child_frame_id, "lidar2");
}

TEST(WalkEditCollect, UnknownFrameContributesNothing)
{
  CollectFixture fx;
  const auto edges =
    collect_editable_edges(fx.buffer, {"ghost"}, "camera", fx.time, fx.static_topics);
  EXPECT_TRUE(edges.empty());
}

TEST(WalkEditCarryOver, KeepsEditedValuesAndAppendsOrphans)
{
  auto edited_a = make_editable("/tf_static", make_edge("base_link", "lidar", 1.0));
  edited_a.edited.x = 9.0;
  auto edited_b = make_editable("/tf_static", make_edge("base_link", "gone", 3.0));
  edited_b.edited.yaw = 0.5;
  const auto untouched = make_editable("/sensing/tf_static", make_edge("odom", "camera", 2.0));

  std::vector<EditableEdge> fresh = {
    make_editable("/tf_static", make_edge("base_link", "lidar", 1.0)),
    make_editable("/sensing/tf_static", make_edge("odom", "camera", 2.0)),
  };
  carry_over_edits(fresh, {edited_a, untouched, edited_b});

  ASSERT_EQ(fresh.size(), 3U);
  // The matching edge inherited its edited pose ...
  EXPECT_DOUBLE_EQ(fresh[0].edited.x, 9.0);
  // ... the untouched previous edge changed nothing ...
  EXPECT_FALSE(is_edited(fresh[1]));
  // ... and the edited edge with no fresh counterpart was appended.
  EXPECT_EQ(fresh[2].original.child_frame_id, "gone");
  EXPECT_DOUBLE_EQ(fresh[2].edited.yaw, 0.5);
}

TEST(WalkEditFormat, EdgeLabelShowsEdgeTopicAndEditedMarker)
{
  auto edge = make_editable("/tf_static", make_edge("base_link", "lidar", 1.0));
  EXPECT_EQ(bagwiz::commands::edge_label(edge), "base_link -> lidar  [/tf_static]");
  edge.edited.x += 0.5;
  EXPECT_EQ(bagwiz::commands::edge_label(edge), "base_link -> lidar  [/tf_static]  (edited)");
}

TEST(WalkEditFormat, InfoTextShowsValuesDeltasAndStep)
{
  auto state = one_edge_state(1.0);
  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));  // x: 1.0 -> 1.01
  const std::string info = bagwiz::commands::edit_info_text(state);

  EXPECT_NE(info.find("edit: base_link->lidar"), std::string::npos) << info;
  // A nudged component carries its delta from the bag value ...
  EXPECT_NE(info.find("x: +1.0100 (+0.0100)"), std::string::npos) << info;
  // ... an untouched one shows the plain value only.
  EXPECT_NE(info.find("y: +0.0000"), std::string::npos) << info;
  EXPECT_EQ(info.find("y: +0.0000 ("), std::string::npos) << info;
  // Rotations render in degrees; 0.005 rad is 0.286 deg.
  EXPECT_NE(info.find("roll: +0.000°"), std::string::npos) << info;
  EXPECT_NE(info.find("step: 0.010m/0.286°"), std::string::npos) << info;
}

TEST(WalkEditFormat, InfoTextWithoutEdgeSaysSo)
{
  ExtrinsicEditState state;
  EXPECT_EQ(bagwiz::commands::edit_info_text(state), "edit: (no editable edge)");
}

TEST(WalkEditFormat, SummaryRendersEditedEdgesOnly)
{
  ExtrinsicEditState state;
  state.edges.push_back(make_editable("/tf_static", make_edge("base_link", "lidar", 1.0)));
  state.edges.push_back(make_editable("/sensing/tf_static", make_edge("odom", "camera", 2.0)));
  EXPECT_TRUE(bagwiz::commands::edit_summary(state).empty());

  state.active = 0;
  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  const std::string summary = bagwiz::commands::edit_summary(state);
  // The edge parent -> child reads as "pose of the child in the parent",
  // i.e. of=child ref=parent in format_transform_human's terms.
  EXPECT_NE(summary.find("of=lidar"), std::string::npos) << summary;
  EXPECT_NE(summary.find("ref=base_link"), std::string::npos) << summary;
  EXPECT_NE(summary.find("(static, topic /tf_static)"), std::string::npos) << summary;
  EXPECT_EQ(summary.find("camera"), std::string::npos) << summary;
}

TEST(WalkEditFormat, EscapesControlBytesFromBagContent)
{
  // Frame ids and topic names come from arbitrary bag payload content, and
  // the strings built here go straight to the terminal (picker rows, info
  // row, exit summary). A crafted id must not smuggle an escape sequence
  // through; the raw byte is replaced by its \xNN spelling.
  auto edge = make_editable(
    "/tf\x1Bstatic", make_edge(std::string("base\x1Blink"), std::string("lidar\x07"), 1.0));

  const std::string label = bagwiz::commands::edge_label(edge);
  EXPECT_EQ(label.find('\x1B'), std::string::npos) << label;
  EXPECT_EQ(label.find('\x07'), std::string::npos) << label;
  EXPECT_NE(label.find("\\x1b"), std::string::npos) << label;
  EXPECT_NE(label.find("\\x07"), std::string::npos) << label;

  ExtrinsicEditState state;
  state.edges.push_back(edge);
  state.active = 0;
  const std::string info = bagwiz::commands::edit_info_text(state);
  EXPECT_EQ(info.find('\x1B'), std::string::npos) << info;

  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));
  const std::string summary = bagwiz::commands::edit_summary(state);
  EXPECT_EQ(summary.find('\x1B'), std::string::npos) << summary;
  EXPECT_EQ(summary.find('\x07'), std::string::npos) << summary;
  // The summary's own line structure survives the escaping.
  EXPECT_NE(summary.find('\n'), std::string::npos) << summary;
}

class WalkEditYamlTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_walk_edit_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

TEST_F(WalkEditYamlTest, ExportsOnlyEditedEdgesAndParsesBack)
{
  ExtrinsicEditState state;
  state.edges.push_back(make_editable("/tf_static", make_edge("base_link", "lidar", 1.0)));
  state.edges.push_back(make_editable("/sensing/tf_static", make_edge("odom", "camera", 2.0)));
  state.active = 0;
  ASSERT_TRUE(apply_edit_key(state, KeyEvent::kEditTransXUp));  // lidar x: 1.0 -> 1.01

  const std::string yaml = edit_yaml(state, "test bag");
  ASSERT_FALSE(yaml.empty());
  EXPECT_EQ(yaml.find("camera"), std::string::npos) << yaml;

  const auto path = tmp_dir_ / "edit.yaml";
  std::ofstream(path) << yaml;
  const auto parsed = bagwiz::core::parse_static_tf_tree_yaml(path);
  ASSERT_TRUE(parsed.ok()) << parsed.error;
  ASSERT_EQ(parsed.transforms->size(), 1U);
  const auto & t = (*parsed.transforms)[0];
  EXPECT_EQ(t.header.frame_id, "base_link");
  EXPECT_EQ(t.child_frame_id, "lidar");
  EXPECT_NEAR(t.transform.translation.x, 1.01, kPointMeters);
}

TEST_F(WalkEditYamlTest, EmptyWhenNothingIsEdited)
{
  ExtrinsicEditState state;
  state.edges.push_back(make_editable("/tf_static", make_edge("base_link", "lidar", 1.0)));
  EXPECT_TRUE(edit_yaml(state, "test bag").empty());
}

}  // namespace
