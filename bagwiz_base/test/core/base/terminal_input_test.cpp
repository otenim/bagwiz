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
  // was narrowed to arrows + space + b. Pin the current contract so a
  // future re-binding is an intentional edit. ('l' and 'n' were retired
  // from navigation the same way and have since been deliberately
  // re-bound to the extrinsic-edit nudges — see ExtrinsicEditRotation.)
  EXPECT_EQ(classify_key("h"), KeyEvent::kUnknown);
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
  EXPECT_EQ(classify_key("q"), KeyEvent::kQuit);
  EXPECT_EQ(classify_key("Q"), KeyEvent::kQuit);
  EXPECT_EQ(classify_key(std::string_view("\x1B", 1)), KeyEvent::kQuit);  // lone ESC
  EXPECT_EQ(classify_key(std::string_view("\x03", 1)), KeyEvent::kQuit);  // Ctrl-C
  EXPECT_EQ(classify_key(std::string_view("\x04", 1)), KeyEvent::kQuit);  // Ctrl-D
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

TEST(ClassifyKey, ExtrinsicEditModeBindings)
{
  // e/E mirror the p/t pcd pair: lowercase toggles the mode, uppercase
  // opens the picker that chooses what the mode acts on.
  EXPECT_EQ(classify_key("e"), KeyEvent::kToggleEditExtrinsic);
  EXPECT_EQ(classify_key("E"), KeyEvent::kSelectEditEdge);
}

TEST(ClassifyKey, ExtrinsicEditTranslation)
{
  // Lowercase nudges the component up by one step, uppercase down.
  EXPECT_EQ(classify_key("x"), KeyEvent::kEditTransXUp);
  EXPECT_EQ(classify_key("X"), KeyEvent::kEditTransXDown);
  EXPECT_EQ(classify_key("y"), KeyEvent::kEditTransYUp);
  EXPECT_EQ(classify_key("Y"), KeyEvent::kEditTransYDown);
  EXPECT_EQ(classify_key("z"), KeyEvent::kEditTransZUp);
  EXPECT_EQ(classify_key("Z"), KeyEvent::kEditTransZDown);
}

TEST(ClassifyKey, ExtrinsicEditRotation)
{
  // roLL / Nod (nose up-down) / yaW: r, p and Y are already taken, so the
  // mnemonic letter comes from inside the word instead.
  EXPECT_EQ(classify_key("l"), KeyEvent::kEditRollUp);
  EXPECT_EQ(classify_key("L"), KeyEvent::kEditRollDown);
  EXPECT_EQ(classify_key("n"), KeyEvent::kEditPitchUp);
  EXPECT_EQ(classify_key("N"), KeyEvent::kEditPitchDown);
  EXPECT_EQ(classify_key("w"), KeyEvent::kEditYawUp);
  EXPECT_EQ(classify_key("W"), KeyEvent::kEditYawDown);
}

TEST(ClassifyKey, ExtrinsicEditStepResetDump)
{
  EXPECT_EQ(classify_key("m"), KeyEvent::kEditStepUp);
  EXPECT_EQ(classify_key("M"), KeyEvent::kEditStepDown);
  EXPECT_EQ(classify_key("0"), KeyEvent::kEditReset);
  // Shift-D like Shift-S: the export prompt takes over the screen, so a
  // single mistyped letter should not trigger it.
  EXPECT_EQ(classify_key("D"), KeyEvent::kEditDumpYaml);
  EXPECT_EQ(classify_key("d"), KeyEvent::kUnknown);
}

TEST(ClassifyKey, ApplyEditsToBagBinding)
{
  // Shift-A, following the Shift-S / Shift-D convention: applying rewrites
  // the input bag in place, so one mistyped letter must not trigger it (a
  // confirmation prompt guards it as well). Lowercase 'a' stays the YAML
  // view's array-expand toggle.
  EXPECT_EQ(classify_key("A"), KeyEvent::kEditApplyToBag);
  EXPECT_EQ(classify_key("a"), KeyEvent::kToggleArrayExpand);
}

TEST(ClassifyKey, PinSceneBinding)
{
  // Shift-P, not a bare 'p': lowercase p toggles the pcd overlay, and the two
  // live side by side in the same preview.
  EXPECT_EQ(classify_key("P"), KeyEvent::kPinScene);
  EXPECT_EQ(classify_key("p"), KeyEvent::kToggleProjectPcd);
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
