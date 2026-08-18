// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview_cache.hpp"  // NOLINT(build/include_subdir) src-local header under test

#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <utility>

namespace
{

using bagwiz::commands::PcdOverlayState;
using bagwiz::commands::tile_render_key;
using bagwiz::commands::TileRenderCache;
using bagwiz::commands::TileRenderEntry;
using bagwiz::commands::TileRenderKey;
using bagwiz::core::tui::image::CellRegion;
using bagwiz::core::tui::image::ImageBackend;
using bagwiz::core::tui::image::TerminalImageCaps;

PcdOverlayState overlay_state()
{
  PcdOverlayState pcd;
  pcd.enabled = true;
  pcd.topics = {"/points"};
  pcd.computed_min = 0.5;
  pcd.computed_max = 42.0;
  return pcd;
}

TerminalImageCaps caps()
{
  TerminalImageCaps c;
  c.backend = ImageBackend::kKitty;
  c.cell = {.width = 10, .height = 20};
  return c;
}

CellRegion region()
{
  return CellRegion{.row = 3, .col = 1, .rows = 20, .cols = 100};
}

TileRenderKey base_key()
{
  return tile_render_key(
    overlay_state(), /*rectify=*/false, /*generation=*/7, /*msg_index=*/128, region(), caps());
}

TEST(TileRenderKey, IdenticalInputsCompareEqual)
{
  EXPECT_EQ(base_key(), base_key());
}

TEST(TileRenderKey, EveryCompositionInputChangesTheKey)
{
  const TileRenderKey base = base_key();

  // Message identity: another frame is another picture.
  EXPECT_NE(base, tile_render_key(overlay_state(), false, 7, 129, region(), caps()));

  // Rectify state changes the base image the overlay is drawn on.
  EXPECT_NE(base, tile_render_key(overlay_state(), true, 7, 128, region(), caps()));

  // The generation covers what values cannot: TF edits and scan swaps.
  EXPECT_NE(base, tile_render_key(overlay_state(), false, 8, 128, region(), caps()));

  // Every value-typed overlay knob invalidates on its own.
  {
    auto pcd = overlay_state();
    pcd.enabled = false;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
  {
    auto pcd = overlay_state();
    pcd.property = bagwiz::core::pointcloud::PointCloudProperty::kIntensity;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
  {
    auto pcd = overlay_state();
    pcd.scheme = bagwiz::core::pointcloud::ColorScheme::kTurbo;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
  {
    auto pcd = overlay_state();
    pcd.point_size = 5;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
  {
    auto pcd = overlay_state();
    pcd.alpha = 0.5f;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
  // The resolved colour range: in auto mode a newly displayed cloud can
  // stretch it, and every tile must recolour to stay comparable.
  {
    auto pcd = overlay_state();
    pcd.computed_max = 50.0;
    EXPECT_NE(base, tile_render_key(pcd, false, 7, 128, region(), caps()));
  }
}

TEST(TileRenderKey, ManualRangeUsesTheManualBounds)
{
  // In manual mode the computed extent may keep drifting as clouds fold in,
  // but the picture is painted with the manual bounds — only those belong in
  // the key, or every fold would needlessly invalidate all tiles.
  auto pcd = overlay_state();
  pcd.auto_range = false;
  pcd.manual_min = 1.0;
  pcd.manual_max = 10.0;
  const TileRenderKey manual = tile_render_key(pcd, false, 7, 128, region(), caps());

  auto drifted = pcd;
  drifted.computed_max = 99.0;
  EXPECT_EQ(manual, tile_render_key(drifted, false, 7, 128, region(), caps()));

  auto retuned = pcd;
  retuned.manual_max = 20.0;
  EXPECT_NE(manual, tile_render_key(retuned, false, 7, 128, region(), caps()));
}

TEST(TileRenderKey, GeometryAndBackendChangeTheKey)
{
  const TileRenderKey base = base_key();

  // A moved or resized tile transmits different bytes at a different cursor
  // position, so the cached payload cannot be replayed.
  {
    CellRegion moved = region();
    moved.col = 51;
    EXPECT_NE(base, tile_render_key(overlay_state(), false, 7, 128, moved, caps()));
  }
  {
    CellRegion resized = region();
    resized.rows = 10;
    EXPECT_NE(base, tile_render_key(overlay_state(), false, 7, 128, resized, caps()));
  }
  {
    auto c = caps();
    c.cell = {.width = 8, .height = 16};
    EXPECT_NE(base, tile_render_key(overlay_state(), false, 7, 128, region(), c));
  }
  {
    auto c = caps();
    c.backend = ImageBackend::kSixel;
    EXPECT_NE(base, tile_render_key(overlay_state(), false, 7, 128, region(), c));
  }
}

TEST(TileRenderCache, MissesUntilStoredThenHits)
{
  TileRenderCache cache;
  const TileRenderKey key = base_key();
  EXPECT_EQ(cache.find(0, key), nullptr);

  TileRenderEntry entry;
  entry.key = key;
  entry.payload = "\x1B[3;1H<escape bytes>";
  entry.readings.residual_ns = 12'300'000LL;
  cache.store(0, std::move(entry));

  const auto * hit = cache.find(0, key);
  ASSERT_NE(hit, nullptr);
  EXPECT_EQ(hit->payload, "\x1B[3;1H<escape bytes>");
  ASSERT_TRUE(hit->readings.residual_ns.has_value());
  EXPECT_EQ(*hit->readings.residual_ns, 12'300'000LL);
}

TEST(TileRenderCache, KeyMismatchIsAMissAndStoreReplaces)
{
  TileRenderCache cache;
  TileRenderEntry entry;
  entry.key = base_key();
  entry.payload = "old";
  cache.store(1, entry);

  // A stale key must never serve: the whole point of the key is that any
  // composition change repaints the tile.
  TileRenderKey other = base_key();
  other.msg_index = 999;
  EXPECT_EQ(cache.find(1, other), nullptr);

  entry.key = other;
  entry.payload = "new";
  cache.store(1, entry);
  ASSERT_NE(cache.find(1, other), nullptr);
  EXPECT_EQ(cache.find(1, other)->payload, "new");
  EXPECT_EQ(cache.find(1, base_key()), nullptr);
}

TEST(TileRenderCache, SlotsAreIndependent)
{
  TileRenderCache cache;
  TileRenderEntry entry;
  entry.key = base_key();
  entry.payload = "slot0";
  cache.store(0, entry);

  EXPECT_NE(cache.find(0, base_key()), nullptr);
  EXPECT_EQ(cache.find(1, base_key()), nullptr);
  EXPECT_EQ(cache.find(3, base_key()), nullptr);
}

TEST(TileRenderCache, TrimDropsSlotsAtAndAboveTheCount)
{
  TileRenderCache cache;
  for (std::size_t slot = 0; slot < 4; ++slot) {
    TileRenderEntry entry;
    entry.key = base_key();
    entry.payload = "p";
    cache.store(slot, entry);
  }
  // An unpin shrinks the grid: the surviving slots keep their entries, the
  // dropped ones release theirs.
  cache.trim(2);
  EXPECT_NE(cache.find(0, base_key()), nullptr);
  EXPECT_NE(cache.find(1, base_key()), nullptr);
  EXPECT_EQ(cache.find(2, base_key()), nullptr);
  EXPECT_EQ(cache.find(3, base_key()), nullptr);
}

}  // namespace
