// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Tests for expand_topic_selectors() (topic_expand.cpp) covering a single
// slot's own, independent behavior: glob expansion, literal pass-through,
// require_present, glob rejection on a literal slot, skipping a non-bag
// input, dedupe within one slot, and the declaration-bug guards
// (missing set_topic_input(), a single-target slot matching more than one
// topic). Cross-slot behavior — pair_value and scope, where one slot's
// resolution depends on another's — lives in
// topic_expand_pair_scope_test.cpp instead.
//
// Split out of topic_option_test.cpp, which grew past this project's
// 800-line file guideline once Task 8 and the final-review fix wave both
// added to it; even after separating registration/store tests into
// topic_option_test.cpp, the remaining expansion-pass tests alone still
// exceeded 800 lines, hence this further split. Shared bag-building helpers
// and the ExpandTopicSelectorsTest fixture live in
// topic_expand_test_util.hpp so neither half duplicates them.

#include "bagwiz/commands/topic_expand.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "topic_expand_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace
{
using bagwiz::commands::add_topic_option;
using bagwiz::commands::expand_topic_selectors;
using bagwiz::commands::set_topic_input;
using bagwiz::commands::TopicSelectorMode;
using bagwiz::commands::TopicSlotSpec;
}  // namespace

TEST_F(ExpandTopicSelectorsTest, ExpandsAGlobFilteredByTypeAndSorted)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"*", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/lidar/left/points", "/lidar/right/points"}));
}

// A literal selector and a non-overlapping glob selector in the same
// multi-value slot: the result keeps argument order across selectors (the
// literal stays first even though it would sort after the glob's match
// lexicographically), while each glob's own matches stay sorted internally.
// Exercises what `topic drop -t <literal> -t <glob>` used to prove via its
// own in-command expansion before that moved here (Task 5).
TEST_F(ExpandTopicSelectorsTest, MixesALiteralAndANonOverlappingGlobInOneSlot)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(
    std::vector<std::string>{
      "/lidar/left/*", "--pcd", "/lidar/right/points", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/lidar/right/points", "/lidar/left/points"}));
}

// Default TopicSlotSpec::require_present is false: contrast with
// RequirePresentRejectsAnAbsentLiteral below, which sets it.
TEST_F(ExpandTopicSelectorsTest, LeavesALiteralUntouchedEvenWhenAbsentFromTheBag)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/not/here", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/not/here"}));
}

// require_present restores, for a literal, the same "matched no topic"
// rejection a non-matching glob already gets — this is what topic drop -t,
// topic keep -t, and trim --align set it for (see TopicSlotSpec's doc comment
// and RequirePresentPreventsTopicKeepFromDestroyingTheBagInPlace below).
TEST_F(ExpandTopicSelectorsTest, RequirePresentRejectsAnAbsentLiteral)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type, .require_present = true});

  app.parse(std::vector<std::string>{"/not/here", "--pcd", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

// require_present's check is presence-only: it must not reuse allowed_types,
// because that would block a wrongly-typed literal from ever reaching the
// command's own (more specific) type error. /camera/image_raw exists in
// make_bag() but is sensor_msgs/msg/Image, not PointCloud2 — it must still
// pass through untouched despite both flags being set together.
TEST_F(ExpandTopicSelectorsTest, RequirePresentIgnoresAllowedTypesForPresenceOnly)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type, .require_present = true});

  app.parse(std::vector<std::string>{"/camera/image_raw", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(pcd, (std::vector<std::string>{"/camera/image_raw"}));
}

// Regression test for the incident that prompted require_present: before this
// flag existed, `topic keep -i bag.mcap -t /typo` silently rewrote the bag in
// place with zero topics, exit 0, no warning — the presence check that used
// to live in the deleted resolve_topic_patterns() call (Task 5) had no
// replacement. This mirrors TopicCommand::configure_keep()'s wiring
// (commands/topic.cpp) closely enough that a regression here is also a
// regression in the real `topic keep` subcommand.
TEST_F(ExpandTopicSelectorsTest, RequirePresentPreventsTopicKeepFromDestroyingTheBagInPlace)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("keep", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "")->check(CLI::ExistingPath);
  add_topic_option(*sub, "-t,--topics", topics, "Topics.", TopicSlotSpec{.require_present = true});

  // Equivalent to `bagwiz topic keep -i <bag> -t /typo` — no -o/--output, the
  // in-place mode that would otherwise overwrite `bag` itself.
  app.parse(std::vector<std::string>{"/typo", "-t", bag.string(), "-i", "keep"});

  // Mirrors main.cpp's real gate: run() — and therefore any writer, in-place
  // or otherwise — is only ever reached when expansion succeeds.
  ASSERT_FALSE(expand_topic_selectors(app));

  // The bag on disk is exactly as make_bag() left it: expansion failing means
  // the command's run() is never invoked, so no writer was ever opened.
  auto reader = bagwiz::io::open_read(bag);
  std::vector<std::string> names;
  for (const auto & t : reader->topics()) {
    names.push_back(t.name);
  }
  std::sort(names.begin(), names.end());
  EXPECT_EQ(
    names,
    (std::vector<std::string>{"/camera/image_raw", "/lidar/left/points", "/lidar/right/points"}));
}

TEST_F(ExpandTopicSelectorsTest, RejectsAGlobInALiteralSlot)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::string topic;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "-t,--topic", topic, "Topic.", TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  app.parse(std::vector<std::string>{"/lidar/*", "-t", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

TEST_F(ExpandTopicSelectorsTest, FailsWhenAGlobMatchesNothing)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/nope/*", "--pcd", bag.string(), "-i", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

TEST_F(ExpandTopicSelectorsTest, SkipsExpansionWhenTheInputIsNotABag)
{
  // `cam-info recompute-p` accepts a calibration YAML here and rejects --topics
  // itself with a message about the YAML. Expansion must not pre-empt that.
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(*sub, "-t,--topics", topics, "Topics.", TopicSlotSpec{});

  app.parse(std::vector<std::string>{"/a", "-t", "/nonexistent/path.yaml", "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(topics, (std::vector<std::string>{"/a"}));
}

TEST_F(ExpandTopicSelectorsTest, ExpandsSlotsOnANestedSubcommand)
{
  // `pcd undistort`-shaped: a subcommand of a subcommand. Slots two levels
  // deep must still be reached by the recursive walk in
  // expand_topic_selectors().
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * pcd = app.add_subcommand("pcd", "");
  auto * undistort = pcd->add_subcommand("undistort", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*undistort, input);
  undistort->add_option("-i,--input", input, "");
  add_topic_option(
    *undistort, "--pcd", topics, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"*", "--pcd", bag.string(), "-i", "undistort", "pcd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(topics, (std::vector<std::string>{"/lidar/left/points", "/lidar/right/points"}));
}

TEST_F(ExpandTopicSelectorsTest, DedupeKeepsDuplicateLiterals)
{
  // Two identical literals must both survive: the command's own "topic given
  // more than once" checks (map_slam.cpp, pcd_concat.cpp) run after
  // expansion and need to see the duplicate to fire.
  const auto bag = make_dedupe_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "-t,--topics", topics, "Topics.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/a", "-t", "/a", "-t", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(topics, (std::vector<std::string>{"/a", "/a"}));
}

TEST_F(ExpandTopicSelectorsTest, DedupeSkipsGlobDuplicateOfALiteral)
{
  const auto bag = make_dedupe_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "-t,--topics", topics, "Topics.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/a*", "-t", "/a", "-t", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(topics, (std::vector<std::string>{"/a", "/a1", "/a2"}));
}

TEST_F(ExpandTopicSelectorsTest, DedupeSkipsGlobDuplicateOfAGlob)
{
  const auto bag = make_dedupe_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "-t,--topics", topics, "Topics.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/a*", "-t", "/a*", "-t", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(topics, (std::vector<std::string>{"/a", "/a1", "/a2"}));
}

// Proves a type-filtered glob on an image-typed slot (the shape `tf tree`,
// `cam-info replace`/`recompute-p`, `generate video --pcd`, `pcd undistort
// --pcd`, and `map slam --color` all now share) matches only topics of the
// declared type, ignoring the bag's other PointCloud2 topics.
TEST_F(ExpandTopicSelectorsTest, ImageSlotGlobPicksOnlyImageTopics)
{
  const auto bag = make_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> color;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--color", color, "Cameras.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kImageTopicTypes});

  app.parse(std::vector<std::string>{"*", "--color", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  EXPECT_EQ(color, (std::vector<std::string>{"/camera/image_raw"}));
}

// Important 3 of the final whole-branch review: a command that declares a
// topic slot but never calls set_topic_input() must fail loudly rather than
// silently skip expansion — miss the call and a literal-only slot stops
// rejecting '*' (the header's central promise) and a glob slot hands '*' to
// the command as a literal topic name. This already happened once on this
// branch for five real commands before the fix restored the calls; nothing
// short of this check stops a sixth.
TEST_F(ExpandTopicSelectorsTest, MissingSetTopicInputIsAnInternalError)
{
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::string topic;
  // Deliberately no set_topic_input(*sub, ...) call.
  add_topic_option(
    *sub, "-t,--topic", topic, "Topic.", TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  app.parse(std::vector<std::string>{"/anything", "-t", "cmd"});
  EXPECT_FALSE(expand_topic_selectors(app));
}

// The nullptr check above must not fire on the ordinary "no -i yet" case a
// bare TEST() can construct: set_topic_input() was called, but `input` is
// still empty because nothing parsed it. Every real slot-carrying subcommand
// makes -i ->required(), so this path is unreachable from user input — but a
// test that forgets to pass -i (many in this file's ExpandTopicSelectorsTest
// fixture do not parse -i separately from --input's own token) must keep
// working, not start failing.
TEST_F(ExpandTopicSelectorsTest, EmptyInputPathWithSetTopicInputCalledStillSkipsExpansion)
{
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::string topic;
  set_topic_input(*sub, input);
  add_topic_option(
    *sub, "-t,--topic", topic, "Topic.", TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  // No "-i" at all: `input` stays empty after parsing.
  app.parse(std::vector<std::string>{"/lidar/*", "-t", "cmd"});
  EXPECT_TRUE(expand_topic_selectors(app));
}

// pcd concat's own contract: concatenation follows --pcd's list order, and
// the FIRST topic becomes the time-matching reference (see pcd.cpp's --pcd
// help text). A glob's expansion order therefore is not incidental — it is
// what makes that reference deterministic when --pcd is a glob rather than
// an explicit literal list.
TEST_F(ExpandTopicSelectorsTest, PcdGlobExpansionOrderIsDeterministicForTheConcatReferenceTopic)
{
  const auto bag = make_dedupe_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::vector<std::string> pcd;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--pcd", pcd, "Clouds.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"*", "--pcd", bag.string(), "-i", "cmd"});
  ASSERT_TRUE(expand_topic_selectors(app));

  // Sorted lexicographically, not bag-declaration order: /a is the reference.
  EXPECT_EQ(pcd, (std::vector<std::string>{"/a", "/a1", "/a2"}));
}

// A literal-only slot's reject_reason replaces the default "does not accept
// globs" wording. `topic rename --dst` is the shape this mirrors: the operand
// names a topic that does not exist yet, so there is nothing to match a glob
// against and the message should say why, not just that globs are refused.
TEST(ExpandTopicSelectors, RejectReasonAppearsInTheError)
{
  bagwiz::core::init_logging();
  const auto dir = scratch_dir("reason");
  const auto bag = make_bag(dir);
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::string dst;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  add_topic_option(
    *sub, "--dst", dst, "New name.",
    TopicSlotSpec{
      .mode = TopicSelectorMode::kLiteral, .reject_reason = "it names the topic to create"});

  app.parse(std::vector<std::string>{"/new/*", "--dst", bag.string(), "-i", "cmd"});
  testing::internal::CaptureStderr();
  EXPECT_FALSE(expand_topic_selectors(app));
  const std::string err = testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("it names the topic to create"), std::string::npos) << err;

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
}

// Task 4 added assign_slot_result()'s "single-target slot resolved to more
// than one topic" guard defensively — every glob-capable slot declared before
// Task 8 was multi-value, so nothing ever exercised it. Task 8 is the first
// task that could: a single-target slot declared without `.mode = kLiteral`
// (the trap TopicSlotSpec::mode's kGlob default sets) silently keeps only the
// first match unless this guard fires. This proves the guard is real, not
// merely present.
TEST_F(ExpandTopicSelectorsTest, SingleTargetGlobMatchingMultipleTopicsFailsLoudly)
{
  bagwiz::core::init_logging();
  const auto bag = make_dedupe_bag(tmp_dir_ / "bag");
  CLI::App app{"bagwiz"};
  auto * sub = app.add_subcommand("cmd", "");
  std::filesystem::path input;
  std::string topic;
  set_topic_input(*sub, input);
  sub->add_option("-i,--input", input, "");
  // Deliberately omits `.mode = TopicSelectorMode::kLiteral` — the mistake
  // Task 8 must not make on any of its 18 slots.
  add_topic_option(
    *sub, "-t,--topic", topic, "Topic.",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  app.parse(std::vector<std::string>{"/a*", "-t", bag.string(), "-i", "cmd"});
  testing::internal::CaptureStderr();
  EXPECT_FALSE(expand_topic_selectors(app));
  const std::string err = testing::internal::GetCapturedStderr();
  EXPECT_NE(err.find("-t/--topic"), std::string::npos) << err;
  EXPECT_NE(err.find("/a"), std::string::npos) << err;
  EXPECT_NE(err.find("/a1"), std::string::npos) << err;
  EXPECT_NE(err.find("/a2"), std::string::npos) << err;
}
