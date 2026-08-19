// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview_cache.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <cstdint>
#include <utility>

namespace bagwiz::commands
{

TileRenderKey tile_render_key(
  const PcdOverlayState & pcd, bool rectify_enabled, std::uint64_t overlay_generation,
  std::size_t msg_index, core::tui::image::CellRegion region,
  const core::tui::image::TerminalImageCaps & caps)
{
  TileRenderKey key;
  key.msg_index = msg_index;
  key.rectify = rectify_enabled;
  key.overlay_enabled = pcd.enabled;
  key.property = pcd.property;
  key.scheme = pcd.scheme;
  key.point_size = pcd.point_size;
  key.alpha = pcd.alpha;
  // The bounds the points are actually coloured with — the same
  // pcd_display_range the projection paints with, so in manual mode the auto
  // extent may keep drifting underneath without invalidating anything.
  const auto [range_min, range_max] = pcd_display_range(pcd);
  key.range_min = range_min;
  key.range_max = range_max;
  key.overlay_generation = overlay_generation;
  key.region_row = region.row;
  key.region_col = region.col;
  key.region_rows = region.rows;
  key.region_cols = region.cols;
  key.cell_width = caps.cell.width;
  key.cell_height = caps.cell.height;
  key.backend = caps.backend;
  return key;
}

const TileRenderEntry * TileRenderCache::find(const TileRenderKey & key) const
{
  if (!entry_.has_value() || !(entry_->key == key)) {
    return nullptr;
  }
  return &*entry_;
}

void TileRenderCache::store(TileRenderEntry entry)
{
  entry_ = std::move(entry);
}

}  // namespace bagwiz::commands
