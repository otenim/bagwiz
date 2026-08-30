// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>

// Unit tests for the movify grid layout: the near-square auto grid, the
// --grid parser, and the composed canvas' cell addressing.

namespace
{

using bagwiz::commands::auto_grid_spec;
using bagwiz::commands::GridCanvas;
using bagwiz::commands::GridSpec;
using bagwiz::commands::parse_grid_spec;

// ---- auto_grid_spec ---------------------------------------------------------

TEST(AutoGridSpec, NearSquareLayouts)
{
  const auto g1 = auto_grid_spec(1);
  EXPECT_EQ(g1.cols, 1u);
  EXPECT_EQ(g1.rows, 1u);
  const auto g2 = auto_grid_spec(2);
  EXPECT_EQ(g2.cols, 2u);
  EXPECT_EQ(g2.rows, 1u);
  const auto g3 = auto_grid_spec(3);
  EXPECT_EQ(g3.cols, 2u);
  EXPECT_EQ(g3.rows, 2u);
  const auto g4 = auto_grid_spec(4);
  EXPECT_EQ(g4.cols, 2u);
  EXPECT_EQ(g4.rows, 2u);
  const auto g5 = auto_grid_spec(5);
  EXPECT_EQ(g5.cols, 3u);
  EXPECT_EQ(g5.rows, 2u);
  const auto g6 = auto_grid_spec(6);
  EXPECT_EQ(g6.cols, 3u);
  EXPECT_EQ(g6.rows, 2u);
  const auto g7 = auto_grid_spec(7);
  EXPECT_EQ(g7.cols, 3u);
  EXPECT_EQ(g7.rows, 3u);
}

// ---- parse_grid_spec --------------------------------------------------------

TEST(ParseGridSpec, EmptySelectsAuto)
{
  const auto r = parse_grid_spec("", 3);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.grid.cols, 2u);
  EXPECT_EQ(r.grid.rows, 2u);
}

TEST(ParseGridSpec, ParsesColsByRows)
{
  const auto r = parse_grid_spec("3x1", 3);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.grid.cols, 3u);
  EXPECT_EQ(r.grid.rows, 1u);
}

TEST(ParseGridSpec, ExtraCellsAllowed)
{
  const auto r = parse_grid_spec("2x2", 3);
  EXPECT_TRUE(r.ok()) << r.error;
}

TEST(ParseGridSpec, RejectsTooFewCells)
{
  const auto r = parse_grid_spec("1x1", 2);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("cell(s) for"), std::string::npos);
}

TEST(ParseGridSpec, RejectsMalformedValues)
{
  for (const char * text : {"2", "2x", "x2", "axb", "2x2x2", "0x2", "2x0", "-1x2"}) {
    EXPECT_FALSE(parse_grid_spec(text, 1).ok()) << text;
  }
}

// ---- GridCanvas -------------------------------------------------------------

TEST(GridCanvasTest, CellsTileTheComposedFrame)
{
  GridCanvas canvas(GridSpec{2, 1});
  canvas.set_cell_size(2, 1);
  EXPECT_EQ(canvas.width(), 4u);
  EXPECT_EQ(canvas.height(), 1u);
  ASSERT_EQ(canvas.pixels().size(), 12u);

  auto c0 = canvas.cell(0);
  EXPECT_EQ(c0.width, 2u);
  EXPECT_EQ(c0.height, 1u);
  EXPECT_EQ(c0.stride, 12u);
  EXPECT_EQ(c0.data, canvas.pixels().data());
  auto c1 = canvas.cell(1);
  EXPECT_EQ(c1.data, canvas.pixels().data() + 6);

  c1.data[0] = std::byte{0x2A};
  EXPECT_EQ(canvas.pixels()[6], std::byte{0x2A});
  canvas.clear();
  EXPECT_EQ(canvas.pixels()[6], std::byte{0});
}

TEST(GridCanvasTest, SecondRowStartsAfterAFullRowOfCells)
{
  GridCanvas canvas(GridSpec{2, 2});
  canvas.set_cell_size(2, 1);
  EXPECT_EQ(canvas.width(), 4u);
  EXPECT_EQ(canvas.height(), 2u);
  ASSERT_EQ(canvas.pixels().size(), 24u);
  // Cell 2 is the first cell of the second row: one composed row (12 bytes)
  // down from the top-left pixel.
  EXPECT_EQ(canvas.cell(2).data, canvas.pixels().data() + 12);
  EXPECT_EQ(canvas.cell(3).data, canvas.pixels().data() + 18);
}

}  // namespace
