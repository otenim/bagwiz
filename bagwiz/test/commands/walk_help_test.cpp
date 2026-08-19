// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_help.hpp"  // NOLINT(build/include_subdir) header under test

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::commands::preview_footer_legend;
using bagwiz::commands::preview_help_lines;
using bagwiz::commands::yaml_footer_legend;
using bagwiz::commands::yaml_help_lines;

// The help reference is checked as one string: which line carries a hint is
// layout, not contract.
std::string flatten(const std::vector<std::string> & lines)
{
  std::string joined;
  for (const auto & line : lines) {
    joined += line;
    joined += '\n';
  }
  return joined;
}

TEST(WalkHelpFooters, YamlFooterCarriesOnlyTheWorkingSet)
{
  const std::string footer = yaml_footer_legend(true);
  EXPECT_NE(footer.find("[Space/b] next/prev"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[j/k] scroll"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[S] save"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[?] keys"), std::string::npos) << footer;
  EXPECT_TRUE(footer.ends_with("[q] quit")) << footer;
  // The reference material moved behind [?]: the time steps, the scroll
  // jumps and the array toggle must no longer ride the footer.
  EXPECT_EQ(footer.find("10s"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("Home"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("expand"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("first"), std::string::npos) << footer;
}

TEST(WalkHelpFooters, YamlFooterFitsOneRowOfAModestTerminal)
{
  // The point of the diet: the footer stays a single row on a 100-column
  // terminal instead of wrapping to three. The footer is ASCII, so byte
  // length equals display width.
  EXPECT_LE(yaml_footer_legend(true).size(), 100U);
  EXPECT_LE(yaml_footer_legend(false).size(), 100U);
}

TEST(WalkHelpFooters, YamlFooterAdvertisesPreviewOnlyWhenAvailable)
{
  EXPECT_NE(yaml_footer_legend(true).find("[i] preview"), std::string::npos);
  EXPECT_EQ(yaml_footer_legend(false).find("[i] preview"), std::string::npos);
}

TEST(WalkHelpFooters, PreviewFooterGatesOverlayEntriesOnASelectedTopic)
{
  const std::string base = preview_footer_legend(false, false);
  EXPECT_NE(base.find("[u] rectify"), std::string::npos) << base;
  EXPECT_NE(base.find("[p] pcd"), std::string::npos) << base;
  EXPECT_NE(base.find("[?] keys"), std::string::npos) << base;
  EXPECT_TRUE(base.ends_with("[q] back")) << base;
  // The edit mode and the scene pins need an overlay topic to be useful, so
  // their hints wait for one.
  EXPECT_EQ(base.find("[e] edit"), std::string::npos) << base;
  EXPECT_EQ(base.find("[P] pin"), std::string::npos) << base;

  const std::string with_topic = preview_footer_legend(true, false);
  EXPECT_NE(with_topic.find("[e] edit"), std::string::npos) << with_topic;
  EXPECT_NE(with_topic.find("[P] pin"), std::string::npos) << with_topic;
  EXPECT_TRUE(with_topic.ends_with("[q] back")) << with_topic;
}

TEST(WalkHelpFooters, PreviewFooterSwapsToTheEditWorkingSet)
{
  // Edit mode replaces the footer instead of appending to it: while nudging
  // a calibration the nudge keys are the working set, everything else is
  // reference material behind [?].
  const std::string edit = preview_footer_legend(true, true);
  EXPECT_NE(edit.find("[x/y/z] move"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[l/n/w] rotate"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[m] step"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[0] reset"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[A] apply"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[D] yaml"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[e] done"), std::string::npos) << edit;
  EXPECT_NE(edit.find("[?] keys"), std::string::npos) << edit;
  EXPECT_EQ(edit.find("[u] rectify"), std::string::npos) << edit;
  EXPECT_EQ(edit.find("next/prev"), std::string::npos) << edit;
  EXPECT_EQ(edit.find("[p] pcd"), std::string::npos) << edit;
}

TEST(WalkHelpFooters, PreviewFooterFitsOneRowOfAModestTerminal)
{
  for (const bool topic : {false, true}) {
    for (const bool edit : {false, true}) {
      EXPECT_LE(preview_footer_legend(topic, edit).size(), 100U) << topic << edit;
    }
  }
}

TEST(WalkHelpReference, YamlHelpListsEveryBindingTheFooterHides)
{
  const std::string help = flatten(yaml_help_lines());
  // Keys the footer no longer advertises must all be discoverable here.
  for (const char * hint :
       {". / ,", "> / <", "g / G", "Home / H", "End / T", "j / k", "expand long arrays",
        "image preview", "save the message as YAML", "quit"}) {
    EXPECT_NE(help.find(hint), std::string::npos) << "missing: " << hint << "\n" << help;
  }
}

TEST(WalkHelpReference, PreviewHelpListsEveryBindingTheFooterHides)
{
  const std::string help = flatten(preview_help_lines());
  for (const char * hint :
       {". / ,", "> / <", "g / G", "rectify", "PNG", "f / c / r", "= / -", "] / [", "pin / unpin",
        "x/X y/Y z/Z", "l/L n/N w/W", "m / M", "reset", "choose the edited edge", "static-TF YAML",
        "apply the edits to the input bag"}) {
    EXPECT_NE(help.find(hint), std::string::npos) << "missing: " << hint << "\n" << help;
  }
}

TEST(WalkHelpReference, HelpIsGroupedBySection)
{
  const std::string yaml_help = flatten(yaml_help_lines());
  EXPECT_NE(yaml_help.find("Navigate"), std::string::npos);
  EXPECT_NE(yaml_help.find("View"), std::string::npos);

  const std::string preview_help = flatten(preview_help_lines());
  EXPECT_NE(preview_help.find("Navigate"), std::string::npos);
  EXPECT_NE(preview_help.find("PCD overlay"), std::string::npos);
  EXPECT_NE(preview_help.find("Edit extrinsic"), std::string::npos);
}

TEST(WalkHelpReference, HelpAdvertisesItsOwnCloseKey)
{
  EXPECT_NE(flatten(yaml_help_lines()).find("close this help"), std::string::npos);
  EXPECT_NE(flatten(preview_help_lines()).find("close this help"), std::string::npos);
}

}  // namespace
