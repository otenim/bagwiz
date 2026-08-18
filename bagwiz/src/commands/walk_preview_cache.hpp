// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_PREVIEW_CACHE_HPP_
#define COMMANDS__WALK_PREVIEW_CACHE_HPP_

#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Per-tile render cache of `bagwiz walk`'s image preview. A repaint used to
// recompose and re-encode every tile, so each pinned scene multiplied the
// cost of every keypress; but a pinned tile's bytes only change when
// something that feeds its composition changes. This module captures exactly
// those inputs as a comparable key, so the preview can replay a tile's
// escape bytes verbatim instead of re-projecting and re-encoding it —
// plain navigation then costs the same as it did before scene pinning
// existed. Pure (no terminal I/O) and unit-tested; walk_preview owns the
// capture and replay. CLI-internal: this header lives with the command
// sources and is not installed.
namespace bagwiz::commands
{

// Everything that determines the bytes a tile transmits. Two equal keys
// render pixel-identical tiles at the same cursor position; any composition
// or geometry change produces a different key and forces a repaint of that
// tile. Exact float comparison is deliberate: any range change, however
// small, recolours the points.
struct TileRenderKey
{
  // The frame shown in the tile.
  std::size_t msg_index = 0;
  // Rectification changes the base image the overlay is drawn on.
  bool rectify = false;
  // Overlay knobs that are plain values. `range_min`/`range_max` are the
  // bounds the points were actually coloured with — the manual bounds in
  // manual mode, else the running auto extent, which a newly displayed cloud
  // can stretch (and then every tile must recolour to stay comparable).
  bool overlay_enabled = false;
  core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
  core::pointcloud::ColorScheme scheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 0;
  float alpha = 0.0f;
  double range_min = 0.0;
  double range_max = 0.0;
  // Composition inputs that are not plain values — the TF buffer behind the
  // projection (extrinsic edits) and the scan behind the fetchers (topic
  // re-selection) — are covered by the controller's generation counter,
  // which bumps whenever either mutates.
  std::uint64_t overlay_generation = 0;
  // Where and how the tile is drawn: the cell region, the cell pixel size
  // the fit is computed with, and the graphics protocol emitting the bytes.
  int region_row = 0;
  int region_col = 0;
  int region_rows = 0;
  int region_cols = 0;
  int cell_width = 0;
  int cell_height = 0;
  core::tui::image::ImageBackend backend = core::tui::image::ImageBackend::kNone;

  [[nodiscard]] bool operator==(const TileRenderKey &) const = default;
};

// Assemble the key for one tile from the preview's current state.
[[nodiscard]] TileRenderKey tile_render_key(
  const PcdOverlayState & pcd, bool rectify_enabled, std::uint64_t overlay_generation,
  std::size_t msg_index, core::tui::image::CellRegion region,
  const core::tui::image::TerminalImageCaps & caps);

// One cached tile: the key its bytes were rendered under, the escape bytes
// themselves (cursor moves included, so replay is a plain write), and the
// overlay readings its caption reports.
struct TileRenderEntry
{
  TileRenderKey key;
  std::string payload;
  OverlayFrameReadings readings;
};

// Slot-indexed store, one entry per tile (slot 0 = live, 1.. = pins), the
// same numbering as PcdOverlayController's fetch slots. A slot holds only
// its latest entry: a repaint either replays it or replaces it, so there is
// nothing older worth keeping.
class TileRenderCache
{
public:
  // The slot's entry when its key matches exactly, else nullptr (miss —
  // the caller recomposes, re-encodes, and store()s the result). The pointer
  // is valid until the next store() or trim().
  [[nodiscard]] const TileRenderEntry * find(std::size_t slot, const TileRenderKey & key) const;

  void store(std::size_t slot, TileRenderEntry entry);

  // Drop the entries of slots at or above `slot_count`, releasing their
  // payloads. The preview calls this when unpinning shrinks the grid, in
  // step with PcdOverlayController::retain_slots.
  void trim(std::size_t slot_count);

private:
  std::vector<std::optional<TileRenderEntry>> slots_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_PREVIEW_CACHE_HPP_
