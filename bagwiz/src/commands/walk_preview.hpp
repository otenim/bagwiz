// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_PREVIEW_HPP_
#define COMMANDS__WALK_PREVIEW_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/rectify.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "walk_cursor.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_pins.hpp"     // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

// In-terminal image preview of `bagwiz walk` (Kitty/Sixel): the decoded-frame
// LRU cache, frame composition (base decode + rectify + pcd overlay), the
// preview renderer, the PNG save, and the preview key loop. Moved out of
// walk.cpp verbatim; the loop stays TTY-coupled by design. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Bounded LRU cache of decoded preview frames keyed by message index. The base
// raster for an index is a pure function of its immutable payload, so entries
// never need invalidation. Interactive navigation revisits nearby frames, so
// evicting the least-recently-used frame keeps the working set hot far better
// than evicting by insertion order (FIFO) would. All operations are O(1).
class DecodedFrameCache
{
public:
  explicit DecodedFrameCache(std::size_t capacity) : capacity_(std::max<std::size_t>(1, capacity))
  {
  }

  // Result of a lookup: `raster` points at the cached frame (valid until the
  // next get() call) or is null when the frame failed to decode, in which case
  // `error` carries the reason. Decode failures are not cached.
  struct Lookup
  {
    const core::image::PackedRaster * raster = nullptr;
    std::string error;
  };

  Lookup get(std::size_t index, std::string_view type, std::span<const std::byte> payload)
  {
    if (const auto it = map_.find(index); it != map_.end()) {
      // Move the hit to the front so it is evicted last.
      order_.splice(order_.begin(), order_, it->second);
      return Lookup{&it->second->raster, {}};
    }
    auto decoded = core::image::to_packed_raster(type, payload);
    if (!decoded.ok()) {
      return Lookup{nullptr, std::move(decoded.error)};
    }
    order_.push_front(Entry{index, std::move(*decoded.raster)});
    map_[index] = order_.begin();
    if (map_.size() > capacity_) {
      map_.erase(order_.back().index);
      order_.pop_back();
    }
    return Lookup{&order_.front().raster, {}};
  }

private:
  struct Entry
  {
    std::size_t index;
    core::image::PackedRaster raster;
  };

  std::size_t capacity_;
  std::list<Entry> order_;  // front = most recently used, back = least
  std::unordered_map<std::size_t, std::list<Entry>::iterator> map_;
};

// Runs the image-preview sub-loop inside the pager's raw-mode + SIGWINCH
// scope. Shares the walked topic's MessageCursor with the YAML view so both
// navigate identically; the PCD overlay and the rectify state live here
// and in the referenced PcdOverlayController. Pinned scenes ([P], see
// walk_pins.hpp) turn the single image into a grid of tiles that every
// extrinsic nudge re-projects together.
class ImagePreviewSession
{
public:
  ImagePreviewSession(
    MessageCursor & cursor, PcdOverlayController & overlay, core::tui::ScrollablePager & pager,
    std::string & status, std::string topic_name, std::string type_name,
    core::tui::image::TerminalImageCaps image_caps,
    const std::optional<core::image::CameraInfo> & camera_info,
    const std::string & camera_info_error);

  // The preview key loop: navigation keys re-decode and re-render; q returns
  // to the YAML view, which the pager then repaints.
  void run();

private:
  core::image::RectifyHelper * ensure_rectify_helper(std::uint32_t w, std::uint32_t h);
  void maybe_rectify(core::image::PackedRaster * raster);

  // One composed tile: the frame to display or save, plus what the overlay
  // matched for it (the tile's caption reports its own cloud pairing).
  struct ComposedFrame
  {
    core::image::PackedRasterResult frame;
    OverlayFrameReadings readings;
  };

  // Produce the frame to display/save for message `idx` in overlay `slot`:
  // fetch the cached base raster (decoding on a miss), then apply the active
  // rectify / PCD overlay on a private copy so the cached frame stays pristine
  // and reusable. `slot` keeps each tile's point-cloud fetches independent
  // (see PcdOverlayController::maybe_overlay).
  ComposedFrame compose_frame(std::size_t idx, std::size_t slot);

  // The tiles to draw this repaint: the cursor's own frame first, then the
  // pinned scenes in pin order. A pin on the frame the cursor is already
  // showing yields one tile labelled as both, never two copies. Pin i takes
  // overlay slot i+1, so a tile keeps its cached cloud across repaints.
  [[nodiscard]] std::vector<TileCaption> tile_captions() const;

  // Draw one composed frame into `region`. Returns "" on success, or the
  // decode/render failure — reported by the caller, because a tile cannot
  // draw text over its own row without erasing its neighbour's (draw_line
  // clears the whole terminal row).
  [[nodiscard]] std::string draw_tile_image(
    std::ostream & out, const core::image::PackedRasterResult & pr,
    core::tui::image::CellRegion region);

  // [P]: pin or unpin the displayed frame, reporting the outcome on the
  // status row.
  void toggle_pin();

  // Paint one preview frame: a two-line caption, the decoded image centred in
  // the region between caption and key hint, and the key hint on the last row.
  // With scenes pinned the image region is split into a tile grid instead.
  void render(std::ostream & out, core::tui::Size term);

  // Save the frame currently shown in the preview as a PNG.
  void save_image();

  // [D]: export the edited static TF edges as the YAML `bagwiz tf static
  // update` applies to the bag (prompts for a path like the PNG save).
  void save_edit_yaml();

  MessageCursor & cursor_;
  PcdOverlayController & overlay_;
  core::tui::ScrollablePager & pager_;
  std::string & status_;
  std::string topic_name_;
  std::string type_name_;
  core::tui::image::TerminalImageCaps image_caps_;
  const std::optional<core::image::CameraInfo> & camera_info_;
  const std::string & camera_info_error_;

  bool rectify_enabled_ = false;
  std::unique_ptr<core::image::RectifyHelper> rectify_helper_;
  std::uint32_t rectify_helper_w_ = 0;
  std::uint32_t rectify_helper_h_ = 0;

  // Decoded-frame cache shared by the preview repaint and the PNG save path,
  // so navigating back to a frame (or saving the one on screen) reuses the
  // decode instead of paying for it again. Its capacity comfortably exceeds
  // the tile count, so a pinned grid never evicts a frame it still shows.
  DecodedFrameCache decoded_frames_;

  // Scenes pinned next to the cursor's own frame, in pin order. Survives
  // leaving and re-entering the preview, like the rectify state above.
  std::vector<ScenePin> pins_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_PREVIEW_HPP_
