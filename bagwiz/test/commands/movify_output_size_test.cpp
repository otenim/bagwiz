// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_output_size.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace
{
using bagwiz::commands::kOversizeWarnPixels;
using bagwiz::commands::oversized_output_warning;

constexpr std::uint32_t kUhdWidth = 3840;
constexpr std::uint32_t kUhdHeight = 2160;

// The threshold is 4K UHD expressed in pixels, which is what the message
// quotes back to the user.
TEST(OversizedOutputWarning, ThresholdIsFourKUhdInPixels)
{
  EXPECT_EQ(kOversizeWarnPixels, static_cast<std::uint64_t>(kUhdWidth) * kUhdHeight);
}

// Strictly above the threshold, so a 2x2 grid of 1080p — exactly 4K — stays
// quiet. That layout is a normal thing to ask for on purpose.
TEST(OversizedOutputWarning, ExactlyAtTheThresholdIsQuiet)
{
  EXPECT_FALSE(oversized_output_warning(kUhdWidth, kUhdHeight, "", "").has_value());
}

TEST(OversizedOutputWarning, BelowTheThresholdIsQuiet)
{
  EXPECT_FALSE(oversized_output_warning(1920, 1080, "", "").has_value());
  EXPECT_FALSE(oversized_output_warning(kUhdWidth, kUhdHeight - 2, "", "").has_value());
}

// One pixel row over the threshold already warns: the check is on the product,
// so it does not matter which dimension carries the excess.
TEST(OversizedOutputWarning, JustOverTheThresholdWarns)
{
  EXPECT_TRUE(oversized_output_warning(kUhdWidth, kUhdHeight + 2, "", "").has_value());
  EXPECT_TRUE(oversized_output_warning(kUhdWidth + 2, kUhdHeight, "", "").has_value());
}

// The product is what counts, not either dimension on its own: a tall, narrow
// output of the same pixel count is judged the same way.
TEST(OversizedOutputWarning, JudgesThePixelProductNotEitherDimension)
{
  // 1920x4320 is a 1x4 column of 1080p cells — 4K's pixel count, rearranged.
  EXPECT_FALSE(oversized_output_warning(1920, 4320, "", "").has_value());
  EXPECT_TRUE(oversized_output_warning(1920, 4322, "", "").has_value());
}

// The message names the actual size, the megapixel count, and the threshold it
// crossed, so the reader can judge how far over they are.
TEST(OversizedOutputWarning, MessageNamesTheSizeAndTheThreshold)
{
  const auto warning = oversized_output_warning(5760, 3240, "", "");
  ASSERT_TRUE(warning.has_value());
  EXPECT_NE(warning->find("5760x3240"), std::string::npos) << *warning;
  EXPECT_NE(warning->find("18.7 Mpx"), std::string::npos) << *warning;
  EXPECT_NE(warning->find("3840x2160"), std::string::npos) << *warning;
}

// `detail` explains how the size arose; `remedy` closes with the flags that
// bring it down. Both are caller-supplied because they differ per subcommand.
TEST(OversizedOutputWarning, FoldsInDetailAndRemedy)
{
  const auto warning = oversized_output_warning(
    5760, 3240, "3x3 grid of 1920x1080 cells", "Pass --width to cap the output width.");
  ASSERT_TRUE(warning.has_value());
  EXPECT_NE(warning->find("3x3 grid of 1920x1080 cells"), std::string::npos) << *warning;
  EXPECT_NE(warning->find("Pass --width to cap the output width."), std::string::npos) << *warning;
}

// An empty `detail` leaves no dangling separator behind — `movify scan` passes
// none, since its size comes straight from --width/--height.
TEST(OversizedOutputWarning, OmitsTheDetailWhenEmpty)
{
  const auto warning = oversized_output_warning(5760, 3240, "", "Lower --width/--height.");
  ASSERT_TRUE(warning.has_value());
  EXPECT_EQ(warning->find(", ,"), std::string::npos) << *warning;
  EXPECT_EQ(warning->find("(, "), std::string::npos) << *warning;
  EXPECT_NE(warning->find("(18.7 Mpx)"), std::string::npos) << *warning;
}

// A zero dimension cannot exceed the threshold; the encode paths reject such a
// frame on their own, so the check must not fire first with a nonsense size.
TEST(OversizedOutputWarning, ZeroDimensionIsQuiet)
{
  EXPECT_FALSE(oversized_output_warning(0, 4320, "", "").has_value());
  EXPECT_FALSE(oversized_output_warning(5760, 0, "", "").has_value());
}

// The product of two large uint32 dimensions must not wrap: the check widens to
// 64-bit before multiplying.
TEST(OversizedOutputWarning, DoesNotOverflowOnLargeDimensions)
{
  EXPECT_TRUE(oversized_output_warning(65536, 65536, "", "").has_value());
}

}  // namespace
