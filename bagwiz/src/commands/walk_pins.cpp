// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_pins.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{

using core::tui::image::CellPixels;
using core::tui::image::CellRegion;

// Size of the `slot`-th of `parts` equal shares of `total`, the first
// `total % parts` shares taking the extra cell so the grid uses every row and
// column of the region.
[[nodiscard]] int share_size(int total, int parts, int slot)
{
  const int base = total / parts;
  const int extra = total % parts;
  return base + (slot < extra ? 1 : 0);
}

// Offset of the `slot`-th share from the start of `total`.
[[nodiscard]] int share_offset(int total, int parts, int slot)
{
  const int base = total / parts;
  const int extra = total % parts;
  return slot * base + std::min(slot, extra);
}

// Pixel area of the frame a tile of this grid would actually display. Scores a
// candidate layout using the renderer's own fitting rule, so the layout choice
// and the drawing agree. The smallest tile (the one without a remainder cell)
// is measured, which is the area every tile is at least guaranteed.
[[nodiscard]] std::int64_t grid_score(
  CellRegion region, int grid_rows, int grid_cols, std::uint32_t img_width,
  std::uint32_t img_height, CellPixels cell)
{
  const int tile_rows = region.rows / grid_rows;
  const int tile_cols = region.cols / grid_cols;
  if (tile_rows < kMinTileRows || tile_cols < kMinTileCols) {
    return 0;  // this candidate cannot give every tile a legible box
  }
  CellRegion image = region;
  image.rows = tile_rows - kTileCaptionRows;
  image.cols = tile_cols;
  const auto fit = core::tui::image::fit_image(img_width, img_height, image, cell);
  return static_cast<std::int64_t>(fit.px_width) * fit.px_height;
}

}  // namespace

PinOutcome toggle_scene_pin(std::vector<ScenePin> & pins, ScenePin pin)
{
  const auto it = std::find_if(
    pins.begin(), pins.end(), [&pin](const ScenePin & p) { return p.index == pin.index; });
  if (it != pins.end()) {
    pins.erase(it);
    return PinOutcome::kUnpinned;
  }
  if (pins.size() >= kMaxScenePins) {
    return PinOutcome::kFull;
  }
  pins.push_back(pin);
  return PinOutcome::kPinned;
}

std::optional<std::size_t> pin_number_of(const std::vector<ScenePin> & pins, std::size_t index)
{
  for (std::size_t i = 0; i < pins.size(); ++i) {
    if (pins[i].index == index) {
      return i + 1;
    }
  }
  return std::nullopt;
}

std::vector<SceneTile> tile_regions(
  CellRegion region, std::size_t tile_count, std::uint32_t img_width, std::uint32_t img_height,
  CellPixels cell)
{
  if (
    tile_count == 0 || region.rows <= 0 || region.cols <= 0 || img_width == 0 || img_height == 0 ||
    cell.width <= 0 || cell.height <= 0) {
    return {};
  }

  // Minimal candidates only: for each row count the columns needed to hold
  // every tile. Anything wider merely shrinks the tiles.
  const int count = static_cast<int>(tile_count);
  int best_rows = 0;
  int best_cols = 0;
  std::int64_t best_score = 0;
  for (int rows = 1; rows <= count; ++rows) {
    const int cols = (count + rows - 1) / rows;
    const std::int64_t score = grid_score(region, rows, cols, img_width, img_height, cell);
    // Strictly greater, so a tie keeps the earlier (fewer-rows, wider) layout.
    if (score > best_score) {
      best_score = score;
      best_rows = rows;
      best_cols = cols;
    }
  }
  if (best_score == 0) {
    return {};  // no candidate fits; the caller falls back to the single view
  }

  std::vector<SceneTile> tiles;
  tiles.reserve(tile_count);
  for (std::size_t i = 0; i < tile_count; ++i) {
    const int r = static_cast<int>(i) / best_cols;
    const int c = static_cast<int>(i) % best_cols;
    SceneTile tile;
    tile.caption_row = region.row + share_offset(region.rows, best_rows, r);
    tile.col = region.col + share_offset(region.cols, best_cols, c);
    tile.cols = share_size(region.cols, best_cols, c);
    tile.image.row = tile.caption_row + kTileCaptionRows;
    tile.image.col = tile.col;
    tile.image.rows = share_size(region.rows, best_rows, r) - kTileCaptionRows;
    tile.image.cols = tile.cols;
    tiles.push_back(tile);
  }
  return tiles;
}

std::string tile_caption(const TileCaption & tile, std::int64_t live_timestamp_ns)
{
  const double offset_s =
    static_cast<double>(tile.pin.timestamp_ns - live_timestamp_ns) / 1'000'000'000.0;

  std::string label;
  if (tile.live) {
    label = "live";
  }
  if (tile.pin_number > 0) {
    if (!label.empty()) {
      label += " · ";
    }
    label += fmt::format("pin {}", tile.pin_number);
  }

  // Same emphasis the info row gives its values; the SGR wrapper is zero
  // display-width, so it does not perturb the caller's wrap/truncate.
  std::string text =
    fmt::format("  #{}  t{:+.1f}s  \x1B[1;36m{}\x1B[0m", tile.pin.index, offset_s, label);
  if (tile.residual_ns.has_value()) {
    text += fmt::format("  Δ: {:+.1f}ms", static_cast<double>(*tile.residual_ns) / 1e6);
  }
  return text;
}

}  // namespace bagwiz::commands
