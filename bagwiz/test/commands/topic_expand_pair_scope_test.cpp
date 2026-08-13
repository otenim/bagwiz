// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Tests for expand_topic_selectors() (topic_expand.cpp) covering cross-slot
// behavior: TopicSlotSpec::pair_value (splitting a "<topic>=<rhs>" value at
// '=') and TopicSlotSpec::scope (a slot resolving against another slot's
// already-expanded result instead of the bag), including the optional-target
// overload's interaction with both. Independent-slot behavior — plain glob
// expansion, literal pass-through, require_present, dedupe, and the
// declaration-bug guards — lives in topic_expand_test.cpp instead; see that
// file's header comment for why this split exists. Shared bag-building
// helpers and the ExpandTopicSelectorsTest fixture live in
// topic_expand_test_util.hpp so neither half duplicates them.

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_expand.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "topic_expand_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace
{
using bagwiz::commands::add_topic_option;
using bagwiz::commands::expand_topic_selectors;
using bagwiz::commands::set_topic_input;
using bagwiz::commands::TopicSelectorMode;
using bagwiz::commands::TopicSlotSpec;
}  // namespace

TEST_F(ExpandTopicSelectorsTest, PairValueGlobsOnlyTheLeftHalf)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  std::vector<std::string> offsets;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  auto * pcd_opt = add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});
  add_topic_option(
    *sub, "--stamp-offset", offsets, "Offsets.",
    TopicSlotSpec{.pair_value = true, .scope = pcd_opt});

  app.parse(
    std::vector<std::string>{
      "/lidar/*=50ms", "--stamp-offset", "*", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(
    offsets, (std::vector<std::string>{"/lidar/left/points=50ms", "/lidar/right/points=50ms"}));
}

TEST_F(ExpandTopicSelectorsTest, PairValuesKeepTheirOwnSuffixAndLiteralsPassThrough)
{
  // Two --stamp-offset occurrences, each a glob-free literal left half with
  // a different right half: proves values from different selectors never
  // cross-contaminate suffixes, and that a literal inside a pair-value slot
  // passes through untouched (unvalidated against the scope), same as a
  // literal anywhere else.
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  std::vector<std::string> offsets;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  auto * pcd_opt = add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});
  add_topic_option(
    *sub, "--stamp-offset", offsets, "Offsets.",
    TopicSlotSpec{.pair_value = true, .scope = pcd_opt});

  app.parse(
    std::vector<std::string>{
      "/lidar/right/points=20ms", "--stamp-offset", "/lidar/left/points=10ms", "--stamp-offset",
      "/lidar/*", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/lidar/left/points", "/lidar/right/points"}));
  EXPECT_EQ(
    offsets, (std::vector<std::string>{"/lidar/left/points=10ms", "/lidar/right/points=20ms"}));
}

TEST_F(ExpandTopicSelectorsTest, ScopedSlotResolvesAgainstGoverningResultIgnoringItsOwnTypeFilter)
{
  // Two properties a naive implementation could get away with skipping: (1)
  // the scoped slot below sets allowed_types itself, so applying it to the
  // (untyped) scoped universe would wrongly filter out every entry and fail
  // the glob instead of succeeding; (2) --pcd names exactly one topic, so a
  // '*' scoped to it must resolve to that one topic, not to all three topics
  // the bag itself would offer if the scope were (wrongly) substituted with
  // the bag.
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  std::vector<std::string> offsets;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  auto * pcd_opt = add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});
  add_topic_option(
    *sub, "--stamp-offset", offsets, "Offsets.",
    TopicSlotSpec{
      .allowed_types = bagwiz::commands::kPointCloud2Type, .pair_value = true, .scope = pcd_opt});

  app.parse(
    std::vector<std::string>{
      "*=50ms", "--stamp-offset", "/lidar/left/points", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/lidar/left/points"}));
  EXPECT_EQ(offsets, (std::vector<std::string>{"/lidar/left/points=50ms"}));
}

// Task 4 shipped `scope` with no test exercising its failure mode — nothing
// used it until pcd concat (Task 7). expand_app() resolves a scoped slot by
// looking up `scope` in a map of already-processed slots, built while
// walking topic_slots_of(app) in declaration order (see resolve_context() in
// topic_expand.cpp): a scope naming an Option outside that set can never
// appear in the map, no matter when it is declared. A plain CLI option that
// was never routed through add_topic_option is exactly such an Option — it
// can never be "an earlier topic slot of this command" — so this is the
// most direct way to force the lookup to miss without needing a real
// second topic slot. The failure must be loud (expand fails) rather than a
// silent wrong-universe resolution or a crash.
TEST_F(ExpandTopicSelectorsTest, ScopeToAnOptionThatIsNotATopicSlotIsAnInternalError)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> offsets;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  std::string not_a_topic_slot;
  auto * plain_opt = sub->add_option("--not-a-slot", not_a_topic_slot, "");
  add_topic_option(
    *sub, "--stamp-offset", offsets, "Offsets.",
    TopicSlotSpec{.pair_value = true, .scope = plain_opt});

  app.parse(
    std::vector<std::string>{
      "/lidar/left/points=50ms", "--stamp-offset", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

// A pair_value literal slot's glob check has no left/right split (see the
// contains_glob() call in the kLiteral branch of expand_slot() in
// topic_expand.cpp): a '*' anywhere in <lhs>=<rhs> — even only in the rhs —
// is rejected. Mirrors `map slam --cam-info`, the mechanism's only
// pair_value + kLiteral slot in production.
TEST_F(ExpandTopicSelectorsTest, PairValueLiteralSlotChecksTheWholeUnsplitValueForAGlob)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> overrides;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--cam-info", overrides, "Overrides.",
    TopicSlotSpec{.mode = TopicSelectorMode::kLiteral, .pair_value = true});

  // The '*' sits only in the rhs; a naive split-then-check would miss it.
  app.parse(
    std::vector<std::string>{
      "/camera/image_raw=/camera/*", "--cam-info", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

// require_present combined with pair_value + kLiteral (Important 2 of the
// final review): the presence check must run against the split left half,
// never the raw "<topic>=<rhs>" value — that string is never itself a topic
// name, so checking it whole would reject every value unconditionally, no
// matter how well-formed. This is the opposite split rule from
// PairValueLiteralSlotChecksTheWholeUnsplitValueForAGlob just above: the glob
// check stays whole-value, the presence check does not.
TEST_F(ExpandTopicSelectorsTest, PairValueLiteralSlotWithRequirePresentChecksOnlyTheLeftHalf)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> overrides;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--cam-info", overrides, "Overrides.",
    TopicSlotSpec{
      .mode = TopicSelectorMode::kLiteral, .pair_value = true, .require_present = true});

  // "/lidar/left/points" is in make_bag(); the rhs is not a topic at all and
  // would fail presence if checked unsplit, the bug this test guards against.
  app.parse(
    std::vector<std::string>{
      "/lidar/left/points=/not/a/topic", "--cam-info", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  // kLiteral mode never rewrites a slot's value (see
  // OptionalTargetLiteralSlotPassesALiteralThrough below): the pair is passed
  // through exactly as typed.
  EXPECT_EQ(overrides, (std::vector<std::string>{"/lidar/left/points=/not/a/topic"}));
}

TEST_F(ExpandTopicSelectorsTest, PairValueLiteralSlotWithRequirePresentRejectsAnAbsentLeftHalf)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> overrides;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--cam-info", overrides, "Overrides.",
    TopicSlotSpec{
      .mode = TopicSelectorMode::kLiteral, .pair_value = true, .require_present = true});

  app.parse(
    std::vector<std::string>{"/not/here=/some/yaml", "--cam-info", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

TEST_F(ExpandTopicSelectorsTest, OptionalTargetLiteralSlotRejectsAGlob)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::optional<std::string> cam_info;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--cam-info", cam_info, "CameraInfo.",
    TopicSlotSpec{
      .allowed_types = bagwiz::commands::kCameraInfoType, .mode = TopicSelectorMode::kLiteral});

  app.parse(std::vector<std::string>{"/camera/*", "--cam-info", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

// Proves the optional overload's whole round trip: CLI11 parses into the
// internal proxy, the sync callback mirrors it into `cam_info`, and — since
// literal mode never writes a slot back — expansion leaves that value
// unchanged, so `cam_info` still holds exactly what was typed.
TEST_F(ExpandTopicSelectorsTest, OptionalTargetLiteralSlotPassesALiteralThrough)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::optional<std::string> cam_info;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--cam-info", cam_info, "CameraInfo.",
    TopicSlotSpec{
      .allowed_types = bagwiz::commands::kCameraInfoType, .mode = TopicSelectorMode::kLiteral});

  app.parse(std::vector<std::string>{"/not/checked", "--cam-info", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));
  ASSERT_TRUE(cam_info.has_value());
  EXPECT_EQ(*cam_info, "/not/checked");
}
