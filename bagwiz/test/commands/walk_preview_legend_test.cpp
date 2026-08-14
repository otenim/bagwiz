// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview_legend.hpp"  // NOLINT(build/include_subdir) header under test

#include <gtest/gtest.h>

#include <string>

namespace
{

using bagwiz::commands::build_preview_legend;

TEST(WalkPreviewLegend, AdvertisesTheSaveKeyAsShiftS)
{
  const std::string legend = build_preview_legend(false, false);
  // The binding lives in classify_key(); this pins the on-screen hint to it,
  // so a legend that still advertises the retired bare `s` fails here.
  EXPECT_NE(legend.find("[S] save"), std::string::npos) << legend;
  EXPECT_EQ(legend.find("[s] save"), std::string::npos) << legend;
}

TEST(WalkPreviewLegend, ListsNavigationAndToggleKeysUnconditionally)
{
  const std::string legend = build_preview_legend(false, false);
  EXPECT_NE(legend.find("[g] first"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[G] last"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[u] rectify"), std::string::npos) << legend;
  // The overlay is enabled from here, so these stay visible even before a
  // PointCloud2 topic is selected.
  EXPECT_NE(legend.find("[p] project pcd"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[t] select pcd topics"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[q] back"), std::string::npos) << legend;
}

TEST(WalkPreviewLegend, PcdAdjustmentKeysAppearOnlyWithASelectedTopic)
{
  const std::string without_topic = build_preview_legend(false, false);
  EXPECT_EQ(without_topic.find("[f] property"), std::string::npos) << without_topic;
  EXPECT_EQ(without_topic.find("[c] scheme"), std::string::npos) << without_topic;
  EXPECT_EQ(without_topic.find("[r] range"), std::string::npos) << without_topic;
  EXPECT_EQ(without_topic.find("size"), std::string::npos) << without_topic;
  EXPECT_EQ(without_topic.find("alpha"), std::string::npos) << without_topic;

  const std::string with_topic = build_preview_legend(true, false);
  EXPECT_NE(with_topic.find("[f] property"), std::string::npos) << with_topic;
  EXPECT_NE(with_topic.find("[c] scheme"), std::string::npos) << with_topic;
  EXPECT_NE(with_topic.find("[r] range"), std::string::npos) << with_topic;
  EXPECT_NE(with_topic.find("[= / -] size"), std::string::npos) << with_topic;
  EXPECT_NE(with_topic.find("[ [ / ] ] alpha"), std::string::npos) << with_topic;
}

TEST(WalkPreviewLegend, QuitHintStaysLastInBothStates)
{
  // [q] back is appended after the conditional blocks, so it must remain the
  // trailing hint whatever combination of keys is present.
  for (const bool pcd_topic_selected : {false, true}) {
    for (const bool edit_active : {false, true}) {
      const std::string legend = build_preview_legend(pcd_topic_selected, edit_active);
      EXPECT_TRUE(legend.ends_with("[q] back")) << legend;
    }
  }
}

TEST(WalkPreviewLegend, AdvertisesEditEntryOnlyWithASelectedTopic)
{
  // The edit mode needs an overlay to judge the alignment against, so its
  // entry hint rides the same condition as the pcd adjustment keys.
  const std::string without_topic = build_preview_legend(false, false);
  EXPECT_EQ(without_topic.find("[e] edit extrinsic"), std::string::npos) << without_topic;

  const std::string with_topic = build_preview_legend(true, false);
  EXPECT_NE(with_topic.find("[e] edit extrinsic"), std::string::npos) << with_topic;
}

TEST(WalkPreviewLegend, EditModeListsTheNudgeKeys)
{
  const std::string legend = build_preview_legend(true, true);
  EXPECT_NE(legend.find("[x/X y/Y z/Z] translate"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[l/L n/N w/W] roll/pitch/yaw"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[m/M] step"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[0] reset"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[E] edge"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[D] export yaml"), std::string::npos) << legend;
  EXPECT_NE(legend.find("[e] done"), std::string::npos) << legend;
  // The mode-entry hint makes no sense while the mode is already on.
  EXPECT_EQ(legend.find("[e] edit extrinsic"), std::string::npos) << legend;
}

}  // namespace
