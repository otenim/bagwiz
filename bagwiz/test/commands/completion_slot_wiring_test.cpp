// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Shell completion (completion.cpp's try_topic_completion()) reads every
// topic slot's `mode` and `allowed_types` in exactly one place. Nothing else
// in the codebase re-declares which flags take topics or what type they
// accept, so nothing else would fail if a production declaration silently
// lost its allowed_types or flipped its mode. This file is that regression
// net: one test per subcommand that declares topic slots, asserting `mode`
// for every slot and `allowed_types` for every slot that declares one — the
// exact two fields completion.cpp's try_topic_completion() reads.
//
// It also carries the wiring coverage nine literal-only slots lacked before
// this file existed (map slam's four, cam-info dump, tf static join/update,
// traj dump/join): their command test files link only their own command's
// sources, not the parent CLI file that owns configure(), so they never
// exercised topic_slots_of() at all. bagwiz_completion_test links every
// command source (see CMakeLists.txt), so this file can.

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "topic_slot_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <span>
#include <string>
#include <string_view>

namespace
{

using bagwiz::commands::Command;
using bagwiz::commands::Registry;
using bagwiz::commands::topic_slots_of;
using bagwiz::commands::TopicSelectorMode;
using bagwiz::commands::TopicSlot;
using bagwiz::test::slot_for;

// The registered command named `name`, or nullptr when none matches.
Command * command_named(const std::string & name)
{
  for (const auto & cmd : Registry::instance().all()) {
    if (cmd->name() == name) {
      return cmd.get();
    }
  }
  return nullptr;
}

// Asserts the two fields try_topic_completion() reads: `mode`, and
// `allowed_types`. `expected_types` defaults to an empty span for a slot
// that declares none. Compares `.data()` (which pointer, not just how many
// entries) against the named constant from topic_types.hpp the caller
// passes, so a slot accidentally wired to a different constant of the same
// size — e.g. kImuType swapped for kNavSatFixType, both size 1 — fails this
// check even though a size-only comparison would not catch it.
void expect_slot(
  const TopicSlot * slot, TopicSelectorMode mode,
  std::span<const std::string_view> expected_types = {})
{
  ASSERT_NE(slot, nullptr);
  EXPECT_EQ(slot->spec.mode, mode);
  EXPECT_EQ(slot->spec.allowed_types.data(), expected_types.data());
  EXPECT_EQ(slot->spec.allowed_types.size(), expected_types.size());
}

}  // namespace

TEST(CompletionSlotWiring, Walk)
{
  auto * cmd = command_named("walk");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"walk"};
  cmd->configure(app);
  const auto slots = topic_slots_of(app);

  expect_slot(slot_for(slots, "topic"), TopicSelectorMode::kLiteral);
  expect_slot(
    slot_for(slots, "cam-info"), TopicSelectorMode::kLiteral, bagwiz::commands::kCameraInfoType);
}

TEST(CompletionSlotWiring, TrajDumpAndJoin)
{
  auto * cmd = command_named("traj");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"traj"};
  cmd->configure(app);

  auto * dump = app.get_subcommand_no_throw("dump");
  ASSERT_NE(dump, nullptr);
  expect_slot(
    slot_for(topic_slots_of(*dump), "topic"), TopicSelectorMode::kLiteral,
    bagwiz::commands::kTrajDumpSupportedTypes);

  // `join`'s --as names the topic the trajectory is embedded under —
  // a new name, not a selection from the bag — so it declares no
  // allowed_types; only its mode is asserted.
  auto * join = app.get_subcommand_no_throw("join");
  ASSERT_NE(join, nullptr);
  expect_slot(slot_for(topic_slots_of(*join), "as"), TopicSelectorMode::kLiteral);
}

TEST(CompletionSlotWiring, TfTreeAndStatic)
{
  auto * cmd = command_named("tf");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"tf"};
  cmd->configure(app);

  auto * tree = app.get_subcommand_no_throw("tree");
  ASSERT_NE(tree, nullptr);
  expect_slot(
    slot_for(topic_slots_of(*tree), "topics"), TopicSelectorMode::kGlob,
    bagwiz::commands::kTfMessageTypes);

  auto * group = app.get_subcommand_no_throw("static");
  ASSERT_NE(group, nullptr);

  // `join`'s --as and `update`'s -t/--topic each name the topic newly added
  // transforms are embedded under — a new name, not a selection — so
  // neither declares allowed_types; only mode is asserted. Their completion
  // (offering the bag's *tf_static topics) is command-specific, not
  // registry-driven — see completion.cpp's complete_tf_static_update().
  auto * join = group->get_subcommand_no_throw("join");
  ASSERT_NE(join, nullptr);
  expect_slot(slot_for(topic_slots_of(*join), "as"), TopicSelectorMode::kLiteral);

  auto * update = group->get_subcommand_no_throw("update");
  ASSERT_NE(update, nullptr);
  expect_slot(slot_for(topic_slots_of(*update), "topic"), TopicSelectorMode::kLiteral);
}

TEST(CompletionSlotWiring, TopicDropKeepRename)
{
  auto * cmd = command_named("topic");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"topic"};
  cmd->configure(app);

  auto * drop = app.get_subcommand_no_throw("drop");
  ASSERT_NE(drop, nullptr);
  expect_slot(slot_for(topic_slots_of(*drop), "topics"), TopicSelectorMode::kGlob);

  auto * keep = app.get_subcommand_no_throw("keep");
  ASSERT_NE(keep, nullptr);
  expect_slot(slot_for(topic_slots_of(*keep), "topics"), TopicSelectorMode::kGlob);

  auto * rename = app.get_subcommand_no_throw("rename");
  ASSERT_NE(rename, nullptr);
  const auto rename_slots = topic_slots_of(*rename);
  expect_slot(slot_for(rename_slots, "src"), TopicSelectorMode::kLiteral);
  expect_slot(slot_for(rename_slots, "dst"), TopicSelectorMode::kLiteral);
}

TEST(CompletionSlotWiring, Movify)
{
  auto * cmd = command_named("movify");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"movify"};
  cmd->configure(app);

  const auto slots = topic_slots_of(app);
  expect_slot(slot_for(slots, "cam"), TopicSelectorMode::kGlob, bagwiz::commands::kImageTopicTypes);
  expect_slot(
    slot_for(slots, "clock"), TopicSelectorMode::kLiteral,
    bagwiz::commands::kMovifyClockTopicTypes);
  expect_slot(slot_for(slots, "pcd"), TopicSelectorMode::kGlob, bagwiz::commands::kPointCloud2Type);
  expect_slot(
    slot_for(slots, "gnss"), TopicSelectorMode::kLiteral, bagwiz::commands::kNavSatFixType);
  expect_slot(
    slot_for(slots, "pose"), TopicSelectorMode::kLiteral, bagwiz::commands::kMovifyPoseTopicTypes);
  // --cam-info takes a bare CameraInfo topic or an <image>=<info> pair; its
  // allowed_types name the pair's left half (like map slam --cam-info), and
  // its value completion defers to complete_movify() (pair_optional).
  expect_slot(
    slot_for(slots, "cam-info"), TopicSelectorMode::kLiteral, bagwiz::commands::kImageTopicTypes);
  expect_slot(
    slot_for(slots, "cam-pcd"), TopicSelectorMode::kGlob, bagwiz::commands::kPointCloud2Type);
}

TEST(CompletionSlotWiring, CamInfoReplaceRecomputePDump)
{
  auto * cmd = command_named("cam-info");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"cam-info"};
  cmd->configure(app);

  const auto & cam_info_types = bagwiz::commands::kCameraInfoType;
  auto * replace = app.get_subcommand_no_throw("replace");
  ASSERT_NE(replace, nullptr);
  expect_slot(
    slot_for(topic_slots_of(*replace), "topics"), TopicSelectorMode::kGlob, cam_info_types);

  auto * recompute_p = app.get_subcommand_no_throw("recompute-p");
  ASSERT_NE(recompute_p, nullptr);
  expect_slot(
    slot_for(topic_slots_of(*recompute_p), "topics"), TopicSelectorMode::kGlob, cam_info_types);

  auto * dump = app.get_subcommand_no_throw("dump");
  ASSERT_NE(dump, nullptr);
  expect_slot(
    slot_for(topic_slots_of(*dump), "topic"), TopicSelectorMode::kLiteral, cam_info_types);
}

TEST(CompletionSlotWiring, PcdConcatAndUndistort)
{
  auto * cmd = command_named("pcd");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"pcd"};
  cmd->configure(app);

  const auto & pcd_types = bagwiz::commands::kPointCloud2Type;
  auto * concat = app.get_subcommand_no_throw("concat");
  ASSERT_NE(concat, nullptr);
  const auto concat_slots = topic_slots_of(*concat);
  // --as names the new concatenated topic to create — a new name, not
  // a selection — so it declares no allowed_types.
  expect_slot(slot_for(concat_slots, "as"), TopicSelectorMode::kLiteral);
  expect_slot(slot_for(concat_slots, "pcd"), TopicSelectorMode::kGlob, pcd_types);
  // --stamp-offset is scoped to --pcd (see pcd.cpp's `.scope = pcd_opt`)
  // rather than carrying its own allowed_types; its completion stays
  // command-specific in completion.cpp for exactly that reason.
  expect_slot(slot_for(concat_slots, "stamp-offset"), TopicSelectorMode::kGlob);

  auto * undistort = app.get_subcommand_no_throw("undistort");
  ASSERT_NE(undistort, nullptr);
  const auto undistort_slots = topic_slots_of(*undistort);
  expect_slot(
    slot_for(undistort_slots, "pose"), TopicSelectorMode::kLiteral,
    bagwiz::commands::kUndistortPoseTopicTypes);
  expect_slot(
    slot_for(undistort_slots, "twist"), TopicSelectorMode::kLiteral,
    bagwiz::commands::kUndistortTwistTopicTypes);
  expect_slot(slot_for(undistort_slots, "pcd"), TopicSelectorMode::kGlob, pcd_types);
}

TEST(CompletionSlotWiring, CalibCamLidar)
{
  auto * cmd = command_named("calib");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"calib"};
  cmd->configure(app);

  auto * cam_lidar = app.get_subcommand_no_throw("cam-lidar");
  ASSERT_NE(cam_lidar, nullptr);
  const auto slots = topic_slots_of(*cam_lidar);
  expect_slot(
    slot_for(slots, "pcd"), TopicSelectorMode::kLiteral, bagwiz::commands::kPointCloud2Type);
  expect_slot(
    slot_for(slots, "pose"), TopicSelectorMode::kLiteral,
    bagwiz::commands::kUndistortPoseTopicTypes);
  expect_slot(
    slot_for(slots, "cam"), TopicSelectorMode::kLiteral, bagwiz::commands::kImageTopicTypes);
  expect_slot(
    slot_for(slots, "cam-info"), TopicSelectorMode::kLiteral, bagwiz::commands::kCameraInfoType);
}

TEST(CompletionSlotWiring, Trim)
{
  auto * cmd = command_named("trim");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"trim"};
  cmd->configure(app);
  expect_slot(slot_for(topic_slots_of(app), "align"), TopicSelectorMode::kGlob);
}

#ifdef BAGWIZ_WITH_SLAM
// `map` is registered only in a BAGWIZ_WITH_SLAM build (map.cpp is compiled
// only then — see CMakeLists.txt), so this wiring test is gated the same way
// the live completion assertions in completion_test.cpp are.
TEST(CompletionSlotWiring, MapSlam)
{
  auto * cmd = command_named("map");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"map"};
  cmd->configure(app);

  auto * slam = app.get_subcommand_no_throw("slam");
  ASSERT_NE(slam, nullptr);
  const auto slots = topic_slots_of(*slam);
  expect_slot(
    slot_for(slots, "pcd"), TopicSelectorMode::kLiteral, bagwiz::commands::kPointCloud2Type);
  expect_slot(slot_for(slots, "imu"), TopicSelectorMode::kLiteral, bagwiz::commands::kImuType);
  expect_slot(
    slot_for(slots, "gnss"), TopicSelectorMode::kLiteral, bagwiz::commands::kNavSatFixType);
  expect_slot(
    slot_for(slots, "color"), TopicSelectorMode::kGlob, bagwiz::commands::kImageTopicTypes);
  // --cam-info's <image_topic> half is one of --color's topics, always
  // image-typed; allowed_types makes that explicit for completion, even
  // though expansion never reads it (kLiteral without require_present).
  expect_slot(
    slot_for(slots, "cam-info"), TopicSelectorMode::kLiteral, bagwiz::commands::kImageTopicTypes);
}
#endif  // BAGWIZ_WITH_SLAM

TEST(CompletionSlotWiring, VideoEncodeAndDecode)
{
  auto * cmd = command_named("video");
  ASSERT_NE(cmd, nullptr);
  CLI::App app{"video"};
  cmd->configure(app);

  auto * encode = app.get_subcommand_no_throw("encode");
  ASSERT_NE(encode, nullptr);
  const auto encode_slots = topic_slots_of(*encode);
  expect_slot(
    slot_for(encode_slots, "topics"), TopicSelectorMode::kGlob, bagwiz::commands::kImageTopicTypes);
  // --as names the new video topic to create — a new name, not a selection
  // — so it declares no allowed_types.
  expect_slot(slot_for(encode_slots, "as"), TopicSelectorMode::kLiteral);

  auto * decode = app.get_subcommand_no_throw("decode");
  ASSERT_NE(decode, nullptr);
  const auto decode_slots = topic_slots_of(*decode);
  expect_slot(
    slot_for(decode_slots, "topics"), TopicSelectorMode::kGlob,
    bagwiz::commands::kCompressedVideoTopicTypes);
  expect_slot(slot_for(decode_slots, "as"), TopicSelectorMode::kLiteral);
}
