// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/terminal_input.hpp"

#include <gtest/gtest.h>

#include <string_view>

namespace
{

using bagwiz::core::classify_key;
using bagwiz::core::KeyEvent;

TEST(ClassifyKey, EmptyIsUnknown)
{
  EXPECT_EQ(classify_key(""), KeyEvent::kUnknown);
}

TEST(ClassifyKey, NextBindings)
{
  EXPECT_EQ(classify_key(" "), KeyEvent::kNext);
  EXPECT_EQ(classify_key(std::string_view("\x1B[C", 3)), KeyEvent::kNext);
}

TEST(ClassifyKey, PrevBindings)
{
  EXPECT_EQ(classify_key("b"), KeyEvent::kPrev);
  EXPECT_EQ(classify_key(std::string_view("\x1B[D", 3)), KeyEvent::kPrev);
}

TEST(ClassifyKey, PreviouslyRetiredKeysAreUnknown)
{
  // 'h' used to map to prev; it was dropped when the navigation key set
  // was narrowed to arrows + space + b. The extrinsic-edit mode and scene
  // pinning were later removed from walk's image preview, freeing their
  // keys ('e'/'E', the nudge letters, '0', 'D', 'A', 'P'). Pin the current
  // contract so a future re-binding is an intentional edit.
  EXPECT_EQ(classify_key("h"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("e"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("E"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("x"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("X"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("y"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("Y"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("z"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("Z"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("l"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("L"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("n"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("N"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("w"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("W"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("m"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("M"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("0"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("d"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("D"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("A"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("P"), KeyEvent::kUnknown);
}

TEST(ClassifyKey, FirstAndLast)
{
  EXPECT_EQ(classify_key("g"), KeyEvent::kFirst);
  EXPECT_EQ(classify_key("G"), KeyEvent::kLast);
}

TEST(ClassifyKey, StepForwardAndBackward1s)
{
  EXPECT_EQ(classify_key("."), KeyEvent::kStepForward1s);
  EXPECT_EQ(classify_key(","), KeyEvent::kStepBackward1s);
}

TEST(ClassifyKey, StepForwardAndBackward10s)
{
  EXPECT_EQ(classify_key(">"), KeyEvent::kStepForward10s);
  EXPECT_EQ(classify_key("<"), KeyEvent::kStepBackward10s);
}

TEST(ClassifyKey, QuitBindings)
{
  // ^C/^D terminate the session outright (kQuit) from any screen, while
  // 'q'/'Q' are a separate event (kQuitView) that only walk's YAML view
  // binds — every other screen leaves it inert. A lone ESC is kBack, not
  // quit (see BackBinding).
  EXPECT_EQ(classify_key("q"), KeyEvent::kQuitView);
  EXPECT_EQ(classify_key("Q"), KeyEvent::kQuitView);
  EXPECT_EQ(classify_key(std::string_view("\x03", 1)), KeyEvent::kQuit);  // Ctrl-C
  EXPECT_EQ(classify_key(std::string_view("\x04", 1)), KeyEvent::kQuit);  // Ctrl-D
}

TEST(ClassifyKey, BackBinding)
{
  // A lone ESC backs out one level (help -> preview -> YAML view -> exit).
  // classify_key cannot see context, so it reports kBack and each view
  // decides what one level up means.
  EXPECT_EQ(classify_key(std::string_view("\x1B", 1)), KeyEvent::kBack);
}

TEST(ClassifyKey, SaveYamlBinding)
{
  // Shift-S, not a bare 's': the save prompt is disruptive enough that a
  // single mistyped letter should not trigger it.
  EXPECT_EQ(classify_key("S"), KeyEvent::kSaveYaml);
  EXPECT_EQ(classify_key("s"), KeyEvent::kUnknown);
}

TEST(ClassifyKey, ToggleArrayExpandBinding)
{
  EXPECT_EQ(classify_key("a"), KeyEvent::kToggleArrayExpand);
}

TEST(ClassifyKey, TogglePreviewBinding)
{
  EXPECT_EQ(classify_key("i"), KeyEvent::kTogglePreview);
}

TEST(ClassifyKey, ToggleRectifyBinding)
{
  EXPECT_EQ(classify_key("u"), KeyEvent::kToggleRectify);
}

TEST(ClassifyKey, ProjectPcdBindings)
{
  EXPECT_EQ(classify_key("p"), KeyEvent::kToggleProjectPcd);
  EXPECT_EQ(classify_key("t"), KeyEvent::kSelectPcdTopic);
  EXPECT_EQ(classify_key("f"), KeyEvent::kCyclePcdProperty);
  EXPECT_EQ(classify_key("c"), KeyEvent::kCyclePcdScheme);
  EXPECT_EQ(classify_key("r"), KeyEvent::kTogglePcdRange);
  EXPECT_EQ(classify_key("="), KeyEvent::kPcdPointSizeUp);
  EXPECT_EQ(classify_key("+"), KeyEvent::kPcdPointSizeUp);
  EXPECT_EQ(classify_key("-"), KeyEvent::kPcdPointSizeDown);
  EXPECT_EQ(classify_key("]"), KeyEvent::kPcdAlphaUp);
  EXPECT_EQ(classify_key("["), KeyEvent::kPcdAlphaDown);
}

TEST(ClassifyKey, HelpBinding)
{
  // '?' opens the key-help overlay in walk's interactive views: the footers
  // advertise only the working set, so the full reference needs one
  // discoverable key shared by the YAML pager and the image preview.
  EXPECT_EQ(classify_key("?"), KeyEvent::kHelp);
}

TEST(ClassifyKey, ScrollBindings)
{
  EXPECT_EQ(classify_key("k"), KeyEvent::kScrollUp);
  EXPECT_EQ(classify_key(std::string_view("\x1B[A", 3)), KeyEvent::kScrollUp);  // Up arrow
  EXPECT_EQ(classify_key("j"), KeyEvent::kScrollDown);
  EXPECT_EQ(classify_key(std::string_view("\x1B[B", 3)), KeyEvent::kScrollDown);  // Down arrow
  EXPECT_EQ(classify_key("H"), KeyEvent::kScrollHead);
  EXPECT_EQ(classify_key(std::string_view("\x1B[H", 3)), KeyEvent::kScrollHead);  // Home
  EXPECT_EQ(classify_key("T"), KeyEvent::kScrollTail);
  EXPECT_EQ(classify_key(std::string_view("\x1B[F", 3)), KeyEvent::kScrollTail);  // End
}

TEST(ClassifyKey, UnknownSequences)
{
  EXPECT_EQ(classify_key("o"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("\t"), KeyEvent::kUnknown);
  // CSI with an unmapped final character.
  EXPECT_EQ(classify_key(std::string_view("\x1B[E", 3)), KeyEvent::kUnknown);
  // Two-byte (partial ESC [) -> unknown; callers handle this by prefetching.
  EXPECT_EQ(classify_key(std::string_view("\x1B[", 2)), KeyEvent::kUnknown);
}

TEST(ClassifyKey, ResizeIsNeverProducedByClassify)
{
  // kResize is synthesised by read_key_event() from a SIGWINCH flag,
  // never returned from byte classification. Pin this so a future
  // refactor that conflates the two paths is caught.
  for (int b = 0; b < 256; ++b) {
    const auto ch = static_cast<char>(b);
    EXPECT_NE(classify_key(std::string_view(&ch, 1)), KeyEvent::kResize);
  }
}

}  // namespace
