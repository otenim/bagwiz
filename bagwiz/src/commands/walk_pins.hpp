// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_PINS_HPP_
#define COMMANDS__WALK_PINS_HPP_

#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Pinned scenes of `bagwiz walk`'s image preview: the pin registry and the
// tile geometry that shows several frames at once. An extrinsic that looks
// right on one frame can still be wrong elsewhere in the recording, so the
// edit mode lets you pin a handful of telling scenes and judge every nudge
// against all of them at the same time.
//
// Everything here is pure (no terminal I/O, no bag access) and unit-tested;
// walk_preview owns the compositing and the key routing. CLI-internal: this
// header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// How many scenes can be pinned at once. Plus the cursor's own frame that is
// four tiles on a 2x2 grid — enough scene diversity to catch an extrinsic
// error (near, far, left, right), while bounding both the point clouds held
// resident per selected topic and the re-projections one nudge triggers.
inline constexpr std::size_t kMaxScenePins = 3;

// A pinned frame. The message index identifies it (walk's message cache only
// ever grows, so an index stays valid for the whole session); the timestamp
// rides along so a caption can show how far the scene sits from the live
// frame without reaching back into the cache.
struct ScenePin
{
  std::size_t index = 0;
  std::int64_t timestamp_ns = 0;
};

// What toggle_scene_pin() did, so the caller can word the status row.
enum class PinOutcome {
  kPinned,    // the frame was added
  kUnpinned,  // the frame was already pinned and has been removed
  kFull,      // the cap is reached and the frame was not added
};

// Add `pin` to `pins`, or remove it when its index is already pinned ([P] is
// its own undo). Matching is by index alone: the timestamp is display data,
// not identity. Appends at the end and closes the gap on removal, so the
// slot numbers the captions show stay stable apart from the removed one.
// Returns kFull without touching `pins` when adding would exceed
// kMaxScenePins; unpinning always works, cap or not.
PinOutcome toggle_scene_pin(std::vector<ScenePin> & pins, ScenePin pin);

// 1-based position of `index` in `pins`, or nullopt when it is not pinned.
[[nodiscard]] std::optional<std::size_t> pin_number_of(
  const std::vector<ScenePin> & pins, std::size_t index);

// Rows every tile reserves above its image for the caption row.
inline constexpr int kTileCaptionRows = 1;
// Smallest tile a grid may use. Below this the images are too small to judge
// an alignment by, so tile_regions() reports "no grid" and the preview falls
// back to the single-frame view rather than drawing a useless mosaic.
inline constexpr int kMinTileRows = 4;
inline constexpr int kMinTileCols = 16;

// One tile of the scene grid: the caption row (spanning the tile's width)
// and the cell region its image is fitted into.
struct SceneTile
{
  int caption_row = 1;
  int col = 1;
  int cols = 0;
  core::tui::image::CellRegion image;
};

// Lay `tile_count` tiles out inside `region`, row-major (the live tile first).
// The grid shape is chosen by scoring every minimal rows x cols candidate
// with the renderer's own fit_image() against an `img_width` x `img_height`
// frame and keeping the one that shows the most image per tile, so a wide
// terminal puts two scenes side by side while a tall narrow one stacks them.
// Ties go to the layout with fewer rows (the wider one). Returns an empty
// vector when there is nothing to lay out, when the inputs are degenerate
// (empty region, zero-sized image or cell), or when no candidate can give
// every tile kMinTileRows x kMinTileCols.
[[nodiscard]] std::vector<SceneTile> tile_regions(
  core::tui::image::CellRegion region, std::size_t tile_count, std::uint32_t img_width,
  std::uint32_t img_height, core::tui::image::CellPixels cell);

// What a tile's caption says about the frame it shows.
struct TileCaption
{
  ScenePin pin;
  // The cursor's frame, which navigation keys move. Pinning the frame you
  // are looking at labels the one tile as both live and pinned instead of
  // drawing it twice.
  bool live = false;
  // 1-based pin slot, or 0 for a live-only tile.
  std::size_t pin_number = 0;
  // Capture-time gap between this tile's own point cloud and its frame, as
  // reported by the overlay for this tile. Absent when either side left its
  // header.stamp unset, which makes the residual undefined.
  std::optional<std::int64_t> residual_ns;
};

// Render a tile's caption: the message index, the signed time offset from the
// live frame, the live / pin label (emphasised the way the info row
// emphasises its values), and the tile's own cloud-match residual when it is
// defined.
[[nodiscard]] std::string tile_caption(const TileCaption & tile, std::int64_t live_timestamp_ns);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_PINS_HPP_
