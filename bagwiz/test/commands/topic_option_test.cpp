// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Registration/store tests for add_topic_option() / set_topic_input() /
// topic_slots_of() / topic_input_of(): what gets recorded in topic_option.cpp's
// process-wide store, and its lifetime (the StoreGuard). Tests for the
// expansion pass built on top of this store (expand_topic_selectors(), which
// needs a real bag to expand against) live in topic_expand_test.cpp instead —
// split out because the combined file exceeded this project's 800-line file
// guideline; see that file's header comment for why the split falls here.

#include "bagwiz/commands/topic_option.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_types.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <new>
#include <optional>
#include <stdexcept>
#include <string>
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

// add_topic_option(std::optional<std::string>&) supports only
// TopicSelectorMode::kLiteral (a kGlob slot's expanded result would be
// written back through the internal proxy, never reaching `target` — see the
// declaration's doc comment). This must fail loudly at declaration time
// rather than compiling into a slot that silently strands its expansion, the
// same standard Task 8 held assign_slot_result()'s single-target guard to
// (see topic_expand_test.cpp's SingleTargetGlobMatchingMultipleTopicsFailsLoudly).
TEST(TopicOption, OptionalTargetRejectsNonLiteralModeAtDeclaration)
{
  CLI::App app{"test"};
  std::optional<std::string> topic;
  EXPECT_THROW(
    add_topic_option(app, "--cam-info", topic, "CameraInfo topic.", TopicSlotSpec{}),
    std::logic_error);
}

// add_topic_option(std::optional<std::string>&) exists so a slot whose
// absence is meaningfully different from an empty value (walk --cam-info,
// movify --cam-info) can still go through the mechanism. CLI11 must
// leave `target` untouched — nullopt — when the flag is never given.
TEST(TopicOption, OptionalTargetStaysNulloptWhenFlagOmitted)
{
  CLI::App app{"test"};
  std::optional<std::string> topic;
  add_topic_option(
    app, "--cam-info", topic, "CameraInfo topic.",
    TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  const auto slots = topic_slots_of(app);
  ASSERT_EQ(slots.size(), 1U);
  EXPECT_NE(slots[0].single_target, nullptr);
  EXPECT_EQ(slots[0].multi_target, nullptr);

  app.parse(std::vector<std::string>{});
  EXPECT_FALSE(topic.has_value());
}

TEST(TopicOption, OptionalTargetHoldsTheParsedValueWhenGiven)
{
  CLI::App app{"test"};
  std::optional<std::string> topic;
  add_topic_option(
    app, "--cam-info", topic, "CameraInfo topic.",
    TopicSlotSpec{.mode = TopicSelectorMode::kLiteral});

  app.parse(std::vector<std::string>{"/camera/camera_info", "--cam-info"});
  ASSERT_TRUE(topic.has_value());
  EXPECT_EQ(*topic, "/camera/camera_info");
}
