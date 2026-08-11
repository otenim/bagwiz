// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_option.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_expand.hpp"
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <new>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::commands::add_topic_option;
using bagwiz::commands::set_topic_input;
using bagwiz::commands::topic_input_of;
using bagwiz::commands::topic_slots_of;
using bagwiz::commands::TopicSelectorMode;
using bagwiz::commands::TopicSlotSpec;
}  // namespace

TEST(TopicOption, RegistersAMultiValueSlotAndStillParses)
{
  CLI::App app{"test"};
  std::filesystem::path input;
  std::vector<std::string> topics;
  set_topic_input(app, input);
  add_topic_option(
    app, "-t,--topics", topics, "PointCloud2 topic(s).",
    TopicSlotSpec{.allowed_types = bagwiz::commands::kPointCloud2Type});

  const auto slots = topic_slots_of(app);
  ASSERT_EQ(slots.size(), 1U);
  EXPECT_EQ(slots[0].multi_target, &topics);
  EXPECT_EQ(slots[0].single_target, nullptr);
  EXPECT_EQ(slots[0].spec.mode, TopicSelectorMode::kGlob);
  ASSERT_NE(topic_input_of(app), nullptr);
  EXPECT_EQ(topic_input_of(app), &input);

  app.parse(std::vector<std::string>{"/b", "/a", "-t"});  // CLI11 takes args reversed
  EXPECT_EQ(topics, (std::vector<std::string>{"/a", "/b"}));
}

TEST(TopicOption, RegistersASingleValueLiteralSlot)
{
  CLI::App app{"test"};
  std::string topic;
  add_topic_option(
    app, "-t,--topic", topic, "Topic to inspect.",
    TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  const auto slots = topic_slots_of(app);
  ASSERT_EQ(slots.size(), 1U);
  EXPECT_EQ(slots[0].single_target, &topic);
  EXPECT_EQ(slots[0].multi_target, nullptr);
  EXPECT_EQ(slots[0].spec.mode, TopicSelectorMode::kLiteral);
}

TEST(TopicOption, ReturnsTheOptionSoChainsKeepWorking)
{
  CLI::App app{"test"};
  std::string topic;
  auto * opt = add_topic_option(app, "-t,--topic", topic, "Topic.", TopicSlotSpec{});
  ASSERT_NE(opt, nullptr);
  opt->required();
  EXPECT_TRUE(opt->get_required());
}

TEST(TopicOption, SubcommandsKeepSeparateSlotLists)
{
  CLI::App app{"test"};
  auto * a = app.add_subcommand("a", "");
  auto * b = app.add_subcommand("b", "");
  std::vector<std::string> a_topics;
  std::string b_topic;
  add_topic_option(*a, "-t,--topics", a_topics, "A.", TopicSlotSpec{});
  add_topic_option(*b, "-t,--topic", b_topic, "B.", TopicSlotSpec{});

  EXPECT_EQ(topic_slots_of(*a).size(), 1U);
  EXPECT_EQ(topic_slots_of(*b).size(), 1U);
  EXPECT_TRUE(topic_slots_of(app).empty());
}

TEST(TopicOption, ScopeRecordsTheGoverningOption)
{
  CLI::App app{"test"};
  std::vector<std::string> pcd;
  std::vector<std::string> offsets;
  auto * pcd_opt = add_topic_option(app, "--pcd", pcd, "Clouds.", TopicSlotSpec{});
  add_topic_option(
    app, "--stamp-offset", offsets, "Offsets.",
    TopicSlotSpec{.pair_value = true, .scope = pcd_opt});

  const auto slots = topic_slots_of(app);
  ASSERT_EQ(slots.size(), 2U);
  EXPECT_EQ(slots[1].spec.scope, pcd_opt);
  EXPECT_TRUE(slots[1].spec.pair_value);
}

// The other tests in this file incidentally exercise the store's cleanup
// guard: this toolchain happens to hand consecutive TEST() bodies the same
// CLI::App stack address, so without the guard a dead test's slots would
// leak into the next. That coverage is accidental — it would go quiet under
// --gtest_shuffle, a --gtest_filter selecting one test, a different
// compiler, or ASan's stack layout. This test forces the exact scenario
// instead of hoping the toolchain reproduces it: placement-new two CLI::App
// instances into the same storage, one after the other's destructor runs.
TEST(TopicOption, DestroyedAppDoesNotLeakSlotsIntoAReusedAddress)
{
  alignas(CLI::App) std::byte storage[sizeof(CLI::App)];

  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) placement-new is the
  // point of this test: it is the only way to force two CLI::App instances
  // to share an address deterministically, rather than hoping the toolchain
  // reuses a stack slot the way the other tests above incidentally rely on.
  auto * first = new (static_cast<void *>(storage)) CLI::App{"test"};
  std::vector<std::string> topics;
  add_topic_option(*first, "-t,--topics", topics, "d", TopicSlotSpec{});
  ASSERT_EQ(topic_slots_of(*first).size(), 1U);
  first->~App();

  auto * second = new (static_cast<void *>(storage)) CLI::App{"test"};
  EXPECT_TRUE(topic_slots_of(*second).empty());
  second->~App();
}

// add_topic_option() attaches a no-op validator purely to hold the store's
// cleanup guard (see arm_guard() in topic_option.cpp). CLI11 folds a
// validator's description into the option's --help type string, so that
// validator's description must stay "" — this pins that down: an app built
// through add_topic_option() must produce byte-identical --help to the same
// app built through plain CLI::App::add_option().
TEST(TopicOption, GuardValidatorLeavesHelpTextUnchanged)
{
  CLI::App via_topic_option{"test"};
  std::string topic;
  add_topic_option(via_topic_option, "-t,--topic", topic, "Topic to inspect.", TopicSlotSpec{});

  CLI::App via_plain_add_option{"test"};
  std::string plain_topic;
  via_plain_add_option.add_option("-t,--topic", plain_topic, "Topic to inspect.");

  EXPECT_EQ(via_topic_option.help(), via_plain_add_option.help());
}

namespace
{
using bagwiz::commands::expand_topic_selectors;

constexpr std::array<std::uint8_t, 4> kPayload{0x01, 0x02, 0x03, 0x04};

std::span<const std::byte> payload_view()
{
  static_assert(sizeof(std::uint8_t) == sizeof(std::byte));
  return std::span<const std::byte>(
    reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      kPayload.data()),
    kPayload.size());
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

std::filesystem::path write_bag(
  const std::filesystem::path & dir, const std::vector<bagwiz::io::TopicInfo> & topics)
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";

  auto writer = bagwiz::io::open_write(dir, opts);
  for (const auto & t : topics) {
    writer->declare_topic(t);
  }
  std::int64_t stamp = 1000;
  for (const auto & t : topics) {
    writer->write(t.name, stamp++, payload_view());
  }
  writer->close();
  return dir;
}

// Bag with two PointCloud2 topics (declared right-before-left so the test can
// prove the result is sorted, not declaration-ordered) and one Image topic.
std::filesystem::path make_bag(const std::filesystem::path & dir)
{
  return write_bag(
    dir, {
           make_topic("/lidar/right/points", "sensor_msgs/msg/PointCloud2"),
           make_topic("/lidar/left/points", "sensor_msgs/msg/PointCloud2"),
           make_topic("/camera/image_raw", "sensor_msgs/msg/Image"),
         });
}

// Bag with three PointCloud2 topics named /a, /a1, /a2 (declared out of
// sorted order, same reason as make_bag() above): a literal '/a' matches only
// itself, a glob '/a*' matches all three sorted. Mirrors the dedupe-rule
// example in topic_expand.cpp's dedupe().
std::filesystem::path make_dedupe_bag(const std::filesystem::path & dir)
{
  return write_bag(
    dir, {
           make_topic("/a2", "sensor_msgs/msg/PointCloud2"),
           make_topic("/a", "sensor_msgs/msg/PointCloud2"),
           make_topic("/a1", "sensor_msgs/msg/PointCloud2"),
         });
}
}  // namespace

class ExpandTopicSelectorsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_topic_expand_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
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
  // Two --stamp-offset occurrences, each a literal (unglobbed) left half with
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
