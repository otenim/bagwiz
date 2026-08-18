// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_pins.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/tui/image/terminal_image_caps.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::kMaxScenePins;
using bagwiz::commands::kMinTileCols;
using bagwiz::commands::kMinTileRows;
using bagwiz::commands::kTileCaptionRows;
using bagwiz::commands::pin_number_of;
using bagwiz::commands::PinOutcome;
using bagwiz::commands::ScenePin;
using bagwiz::commands::SceneTile;
using bagwiz::commands::tile_caption;
using bagwiz::commands::tile_regions;
using bagwiz::commands::TileCaption;
using bagwiz::commands::toggle_scene_pin;
using bagwiz::core::tui::image::CellPixels;
using bagwiz::core::tui::image::CellRegion;

constexpr std::int64_t kSec = 1'000'000'000LL;

ScenePin pin_at(std::size_t index, double seconds)
{
  return ScenePin{index, static_cast<std::int64_t>(seconds * static_cast<double>(kSec))};
}

// A roomy 200x45 terminal region with the usual 2:1 cell aspect.
CellRegion wide_region()
{
  return CellRegion{.row = 3, .col = 1, .rows = 40, .cols = 200};
}

CellPixels cell()
{
  return CellPixels{.width = 10, .height = 20};
}

TEST(ToggleScenePin, PinsThenUnpinsTheSameIndex)
{
  std::vector<ScenePin> pins;
  EXPECT_EQ(toggle_scene_pin(pins, pin_at(128, 1.0)), PinOutcome::kPinned);
  ASSERT_EQ(pins.size(), 1U);
  EXPECT_EQ(pins[0].index, 128U);

  // Toggling the very same frame takes it back out, so [P] is its own undo.
  EXPECT_EQ(toggle_scene_pin(pins, pin_at(128, 1.0)), PinOutcome::kUnpinned);
  EXPECT_TRUE(pins.empty());
}

TEST(ToggleScenePin, UnpinMatchesOnIndexNotTimestamp)
{
  // The message index identifies a frame; a caller that reconstructed the
  // timestamp differently must still be able to unpin it.
  std::vector<ScenePin> pins;
  ASSERT_EQ(toggle_scene_pin(pins, pin_at(7, 1.0)), PinOutcome::kPinned);
  EXPECT_EQ(toggle_scene_pin(pins, pin_at(7, 99.0)), PinOutcome::kUnpinned);
  EXPECT_TRUE(pins.empty());
}

TEST(ToggleScenePin, RefusesBeyondTheCapAndKeepsExistingPins)
{
  std::vector<ScenePin> pins;
  for (std::size_t i = 0; i < kMaxScenePins; ++i) {
    ASSERT_EQ(toggle_scene_pin(pins, pin_at(i, static_cast<double>(i))), PinOutcome::kPinned);
  }
  EXPECT_EQ(toggle_scene_pin(pins, pin_at(99, 9.0)), PinOutcome::kFull);
  EXPECT_EQ(pins.size(), kMaxScenePins);

  // A full list still unpins: the cap only blocks new pins.
  EXPECT_EQ(toggle_scene_pin(pins, pin_at(0, 0.0)), PinOutcome::kUnpinned);
  EXPECT_EQ(pins.size(), kMaxScenePins - 1U);
}

TEST(ToggleScenePin, PinOrderIsStableAcrossAnUnpin)
{
  // Slot labels are derived from the pin order, so removing the middle pin
  // must not renumber the pins around it beyond closing the gap.
  std::vector<ScenePin> pins;
  ASSERT_EQ(toggle_scene_pin(pins, pin_at(10, 1.0)), PinOutcome::kPinned);
  ASSERT_EQ(toggle_scene_pin(pins, pin_at(20, 2.0)), PinOutcome::kPinned);
  ASSERT_EQ(toggle_scene_pin(pins, pin_at(30, 3.0)), PinOutcome::kPinned);
  ASSERT_EQ(toggle_scene_pin(pins, pin_at(20, 2.0)), PinOutcome::kUnpinned);

  ASSERT_EQ(pins.size(), 2U);
  EXPECT_EQ(pins[0].index, 10U);
  EXPECT_EQ(pins[1].index, 30U);
}

TEST(PinNumberOf, IsOneBasedAndAbsentForUnpinned)
{
  const std::vector<ScenePin> pins{pin_at(10, 1.0), pin_at(20, 2.0)};
  ASSERT_TRUE(pin_number_of(pins, 10).has_value());
  EXPECT_EQ(*pin_number_of(pins, 10), 1U);
  ASSERT_TRUE(pin_number_of(pins, 20).has_value());
  EXPECT_EQ(*pin_number_of(pins, 20), 2U);
  EXPECT_FALSE(pin_number_of(pins, 15).has_value());
}

TEST(TileRegions, SingleTileTakesTheWholeRegionMinusItsCaption)
{
  const auto tiles = tile_regions(wide_region(), 1, 1920, 1080, cell());
  ASSERT_EQ(tiles.size(), 1U);
  EXPECT_EQ(tiles[0].caption_row, 3);
  EXPECT_EQ(tiles[0].col, 1);
  EXPECT_EQ(tiles[0].cols, 200);
  EXPECT_EQ(tiles[0].image.row, 3 + kTileCaptionRows);
  EXPECT_EQ(tiles[0].image.rows, 40 - kTileCaptionRows);
  EXPECT_EQ(tiles[0].image.cols, 200);
}

TEST(TileRegions, NoTilesYieldsNoRegions)
{
  EXPECT_TRUE(tile_regions(wide_region(), 0, 1920, 1080, cell()).empty());
}

TEST(TileRegions, TwoTilesSitSideBySideOnAWideRegion)
{
  // 200x45 cells at 10x20 px is a 2000x900 px region: two 16:9 tiles show
  // more image side by side than stacked, so the grid must be 1x2.
  const auto tiles = tile_regions(wide_region(), 2, 1920, 1080, cell());
  ASSERT_EQ(tiles.size(), 2U);
  EXPECT_EQ(tiles[0].caption_row, tiles[1].caption_row);
  EXPECT_LT(tiles[0].col, tiles[1].col);
}

TEST(TileRegions, TwoTilesStackOnATallNarrowRegion)
{
  // 60x60 cells at 10x20 px is a 600x1200 px region — taller than it is
  // wide, so the same two tiles must stack instead.
  const CellRegion tall{.row = 1, .col = 1, .rows = 60, .cols = 60};
  const auto tiles = tile_regions(tall, 2, 1920, 1080, cell());
  ASSERT_EQ(tiles.size(), 2U);
  EXPECT_EQ(tiles[0].col, tiles[1].col);
  EXPECT_LT(tiles[0].caption_row, tiles[1].caption_row);
}

TEST(TileRegions, ThreeAndFourTilesUseTheSameTwoByTwoGrid)
{
  // Going from three pins to four must not reflow the tiles already on
  // screen: both counts land on 2x2, three of them leaving a hole.
  const auto three = tile_regions(wide_region(), 3, 1920, 1080, cell());
  const auto four = tile_regions(wide_region(), 4, 1920, 1080, cell());
  ASSERT_EQ(three.size(), 3U);
  ASSERT_EQ(four.size(), 4U);
  for (std::size_t i = 0; i < three.size(); ++i) {
    EXPECT_EQ(three[i].caption_row, four[i].caption_row) << "tile " << i;
    EXPECT_EQ(three[i].col, four[i].col) << "tile " << i;
    EXPECT_EQ(three[i].cols, four[i].cols) << "tile " << i;
  }
  // Row-major: the first two share a row, the third starts the next one.
  EXPECT_EQ(four[0].caption_row, four[1].caption_row);
  EXPECT_LT(four[0].caption_row, four[2].caption_row);
  EXPECT_EQ(four[2].caption_row, four[3].caption_row);
}

TEST(TileRegions, TilesStayInsideTheRegionAndNeverOverlap)
{
  const CellRegion region = wide_region();
  for (std::size_t count = 1; count <= kMaxScenePins + 1U; ++count) {
    const auto tiles = tile_regions(region, count, 1920, 1080, cell());
    ASSERT_EQ(tiles.size(), count);
    for (const auto & tile : tiles) {
      EXPECT_GE(tile.caption_row, region.row);
      EXPECT_GE(tile.col, region.col);
      EXPECT_LE(tile.image.row + tile.image.rows, region.row + region.rows);
      EXPECT_LE(tile.col + tile.cols, region.col + region.cols);
      EXPECT_GE(tile.image.rows, kMinTileRows - kTileCaptionRows);
      EXPECT_GE(tile.cols, kMinTileCols);
    }
    for (std::size_t a = 0; a + 1 < tiles.size(); ++a) {
      for (std::size_t b = a + 1; b < tiles.size(); ++b) {
        const bool rows_disjoint =
          tiles[a].caption_row + tiles[a].image.rows + kTileCaptionRows <= tiles[b].caption_row ||
          tiles[b].caption_row + tiles[b].image.rows + kTileCaptionRows <= tiles[a].caption_row;
        const bool cols_disjoint = tiles[a].col + tiles[a].cols <= tiles[b].col ||
                                   tiles[b].col + tiles[b].cols <= tiles[a].col;
        EXPECT_TRUE(rows_disjoint || cols_disjoint) << "tiles " << a << " and " << b << " overlap";
      }
    }
  }
}

TEST(TileRegions, TooSmallARegionYieldsNoGrid)
{
  // A terminal that cannot give every tile a legible box gets an empty
  // result, which the preview reads as "fall back to the single view".
  const CellRegion tiny{.row = 1, .col = 1, .rows = kMinTileRows, .cols = kMinTileCols};
  EXPECT_TRUE(tile_regions(tiny, 4, 1920, 1080, cell()).empty());
  // The same region still fits one tile.
  EXPECT_EQ(tile_regions(tiny, 1, 1920, 1080, cell()).size(), 1U);
}

TEST(TileRegions, DegenerateInputsYieldNoGrid)
{
  const CellRegion empty{.row = 1, .col = 1, .rows = 0, .cols = 0};
  EXPECT_TRUE(tile_regions(empty, 2, 1920, 1080, cell()).empty());
  // A zero-sized image or cell carries no aspect to lay out against.
  EXPECT_TRUE(tile_regions(wide_region(), 2, 0, 0, cell()).empty());
  EXPECT_TRUE(tile_regions(wide_region(), 2, 1920, 1080, CellPixels{}).empty());
}

TEST(TileCaption, LiveTileReportsZeroOffset)
{
  TileCaption tile;
  tile.pin = pin_at(128, 12.0);
  tile.live = true;
  const std::string text = tile_caption(tile, pin_at(128, 12.0).timestamp_ns);
  EXPECT_NE(text.find("#128"), std::string::npos) << text;
  EXPECT_NE(text.find("t+0.0s"), std::string::npos) << text;
  EXPECT_NE(text.find("live"), std::string::npos) << text;
}

TEST(TileCaption, PinnedTileReportsItsSignedOffsetFromTheLiveFrame)
{
  const std::int64_t live_ns = pin_at(128, 12.0).timestamp_ns;

  TileCaption ahead;
  ahead.pin = pin_at(302, 20.7);
  ahead.pin_number = 1;
  const std::string ahead_text = tile_caption(ahead, live_ns);
  EXPECT_NE(ahead_text.find("#302"), std::string::npos) << ahead_text;
  EXPECT_NE(ahead_text.find("t+8.7s"), std::string::npos) << ahead_text;
  EXPECT_NE(ahead_text.find("pin 1"), std::string::npos) << ahead_text;

  TileCaption behind;
  behind.pin = pin_at(40, 6.5);
  behind.pin_number = 2;
  const std::string behind_text = tile_caption(behind, live_ns);
  EXPECT_NE(behind_text.find("t-5.5s"), std::string::npos) << behind_text;
  EXPECT_NE(behind_text.find("pin 2"), std::string::npos) << behind_text;
}

TEST(TileCaption, LivePinnedTileCarriesBothLabels)
{
  // Pinning the frame you are looking at must not draw it twice: the one
  // tile says it is both.
  TileCaption tile;
  tile.pin = pin_at(302, 20.7);
  tile.live = true;
  tile.pin_number = 2;
  const std::string text = tile_caption(tile, tile.pin.timestamp_ns);
  EXPECT_NE(text.find("live"), std::string::npos) << text;
  EXPECT_NE(text.find("pin 2"), std::string::npos) << text;
}

TEST(TileCaption, ResidualIsShownOnlyWhenKnown)
{
  TileCaption tile;
  tile.pin = pin_at(302, 20.7);
  tile.pin_number = 1;
  EXPECT_EQ(tile_caption(tile, tile.pin.timestamp_ns).find("Δ"), std::string::npos);

  tile.residual_ns = 12'300'000LL;  // +12.3 ms
  const std::string text = tile_caption(tile, tile.pin.timestamp_ns);
  EXPECT_NE(text.find("Δ"), std::string::npos) << text;
  EXPECT_NE(text.find("+12.3ms"), std::string::npos) << text;
}

}  // namespace
