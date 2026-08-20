// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_help.hpp"  // NOLINT(build/include_subdir) header under test

#include "bagwiz/core/tui/width.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::commands::preview_footer_legend;
using bagwiz::commands::preview_help_lines;
using bagwiz::commands::yaml_footer_legend;
using bagwiz::commands::yaml_help_lines;

// Strip SGR escape sequences (\x1B[...m) so tests can match the visible
// characters of a styled entry like the rainbow [i] hint.
std::string strip_sgr(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size();) {
    if (text[i] == '\x1B' && i + 1 < text.size() && text[i + 1] == '[') {
      i += 2;
      while (i < text.size() && text[i] != 'm') {
        ++i;
      }
      i += i < text.size() ? 1 : 0;  // consume the terminating 'm'
      continue;
    }
    out.push_back(text[i]);
    ++i;
  }
  return out;
}

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
  EXPECT_NE(footer.find("[S] save"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[?] help"), std::string::npos) << footer;
  EXPECT_TRUE(footer.ends_with("[q] quit")) << footer;
  // The reference material moved behind [?]: the time steps, the scroll
  // jumps and the array toggle must no longer ride the footer — and the
  // intuitive Space/b and j/k need no label at all.
  EXPECT_EQ(footer.find("next/prev"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("scroll"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("10s"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("Home"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("expand"), std::string::npos) << footer;
  EXPECT_EQ(footer.find("first"), std::string::npos) << footer;
}

TEST(WalkHelpFooters, YamlFooterFitsOneRowOfAModestTerminal)
{
  // The point of the diet: the footer stays a single row on a 100-column
  // terminal instead of wrapping to three. Measured as display width — the
  // rainbow [i] hint carries zero-width SGR escapes.
  EXPECT_LE(bagwiz::core::tui::display_width(yaml_footer_legend(true)), 100);
  EXPECT_LE(bagwiz::core::tui::display_width(yaml_footer_legend(false)), 100);
}

TEST(WalkHelpFooters, YamlPreviewHintIsRainbowColored)
{
  // The [i] hint keeps its per-character rainbow paint; the SGR escapes are
  // zero display width, so wrapping and the one-row budget are unaffected.
  EXPECT_NE(yaml_footer_legend(true).find("\x1B[31m"), std::string::npos);
  // Without the preview there is no colored entry at all.
  EXPECT_EQ(yaml_footer_legend(false).find("\x1B["), std::string::npos);
}

TEST(WalkHelpFooters, YamlFooterAdvertisesPreviewOnlyWhenAvailable)
{
  // The rainbow paint interleaves SGR escapes with the hint's characters,
  // so match against the escape-stripped text.
  EXPECT_NE(strip_sgr(yaml_footer_legend(true)).find("[i] preview"), std::string::npos);
  EXPECT_EQ(strip_sgr(yaml_footer_legend(false)).find("preview"), std::string::npos);
}

TEST(WalkHelpFooters, PreviewFooterCarriesOnlyTheWorkingSet)
{
  const std::string footer = preview_footer_legend();
  // Space/b navigation is unlabeled here too — the working set starts at
  // the view toggles.
  EXPECT_EQ(footer.find("next/prev"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[u] rectify"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[p] pcd overlay"), std::string::npos) << footer;
  EXPECT_NE(footer.find("[?] help"), std::string::npos) << footer;
  EXPECT_TRUE(footer.ends_with("[q] back")) << footer;
}

TEST(WalkHelpFooters, PreviewFooterFitsOneRowOfAModestTerminal)
{
  EXPECT_LE(preview_footer_legend().size(), 100U);
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
       {". / ,", "> / <", "g / G", "rectify", "PNG", "f / c / r", "= / -", "] / ["}) {
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
}

TEST(WalkHelpReference, HelpAdvertisesQAsItsCloseKey)
{
  // q closes the reference ('?' only opens it), so the card must say so and
  // must not advertise '?' or Esc as additional close keys.
  for (const auto & help : {flatten(yaml_help_lines()), flatten(preview_help_lines())}) {
    EXPECT_NE(help.find("  q "), std::string::npos) << help;
    EXPECT_NE(help.find("close this help"), std::string::npos) << help;
    EXPECT_EQ(help.find("Esc"), std::string::npos) << help;
  }
}

TEST(WalkHelpFooters, BackIsQInThePreviewFooter)
{
  // Mashing q walks all the way out of the preview: the footer advertises
  // [q] back, and Esc is not a back key anywhere in walk.
  EXPECT_NE(preview_footer_legend().find("[q] back"), std::string::npos);
  EXPECT_EQ(preview_footer_legend().find("Esc"), std::string::npos);
}

TEST(WalkHelpReference, HelpListsTheLeaveKeys)
{
  // The YAML view quits walk on q outside the help overlay, so its
  // reference lists a standalone q. The preview goes back a level on q and
  // quits walk only on Ctrl-C / Ctrl-D.
  EXPECT_NE(flatten(yaml_help_lines()).find("  q "), std::string::npos);
  const std::string preview_help = flatten(preview_help_lines());
  EXPECT_NE(preview_help.find("back to the YAML view"), std::string::npos);
  EXPECT_NE(preview_help.find("Ctrl-C / Ctrl-D"), std::string::npos);
}

}  // namespace
