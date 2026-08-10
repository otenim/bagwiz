// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_option.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/topic_types.hpp"

#include <gtest/gtest.h>

#include <filesystem>
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
