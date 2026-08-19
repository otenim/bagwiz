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
#include "walk_cursor.hpp"         // NOLINT(build/include_subdir) src-local shared header
#include "walk_overlay.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "walk_pins.hpp"           // NOLINT(build/include_subdir) src-local shared header
#include "walk_preview_cache.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
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

// Bounded LRU cache of per-frame rasters keyed by message index, fed by a
// caller-supplied producer on a miss. Both users' rasters are pure functions
// of the frame's immutable payload — the base decode, and the rectified
// remap (CameraInfo never changes mid-session) — so entries never need
// invalidation. Interactive navigation revisits nearby frames, so evicting
// the least-recently-used frame keeps the working set hot far better than
// evicting by insertion order (FIFO) would. All operations are O(1).
class PreviewFrameCache
{
public:
  explicit PreviewFrameCache(std::size_t capacity) : capacity_(std::max<std::size_t>(1, capacity))
  {
  }

  // Result of a lookup: `raster` points at the cached frame (valid until the
  // next get() call) or is null when the producer failed, in which case
  // `error` carries the reason. Failures are not cached.
  struct Lookup
  {
    const core::image::PackedRaster * raster = nullptr;
    std::string error;
  };

  using Producer = std::function<core::image::PackedRasterResult()>;

  Lookup get(std::size_t index, const Producer & produce)
  {
    if (const auto it = map_.find(index); it != map_.end()) {
      // Move the hit to the front so it is evicted last.
      order_.splice(order_.begin(), order_, it->second);
      return Lookup{&it->second->raster, {}};
    }
    auto produced = produce();
    if (!produced.ok()) {
      return Lookup{nullptr, std::move(produced.error)};
    }
    order_.push_front(Entry{index, std::move(*produced.raster)});
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

  // How the preview key loop ended: the user backed out to the YAML view
  // (Esc), or asked to terminate the whole walk session (Ctrl-C / Ctrl-D).
  enum class Exit { kBack, kTerminate };

  // The preview key loop: navigation keys re-decode and re-render; Esc
  // returns to the YAML view, which the pager then repaints; Ctrl-C /
  // Ctrl-D exit with Exit::kTerminate so the caller can end walk. 'q' is
  // inert here — quitting walk on 'q' is the YAML view's binding alone.
  Exit run();

private:
  core::image::RectifyHelper * ensure_rectify_helper(std::uint32_t w, std::uint32_t h);

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

  // Encode one composed frame into the escape bytes that draw it in
  // `region` (cursor moves included). Returns the bytes, or an empty string
  // with `error` set on a decode/render failure — reported by the caller,
  // because a tile cannot draw text over its own row without erasing its
  // neighbour's (draw_line clears the whole terminal row).
  [[nodiscard]] std::string encode_tile(
    const core::image::PackedRasterResult & pr, core::tui::image::CellRegion region,
    std::string & error);

  // Write one tile: replay its cached escape bytes when `key` still matches
  // what is stored for `slot`, else compose (unless `composed` already is),
  // encode, emit, and cache the result. `composed` lets the live tile reuse
  // the composition the info row already needed. Fills the readings for the
  // caption and `error` when the tile could not be drawn.
  void emit_tile(
    std::ostream & out, std::size_t slot, std::size_t msg_index, const ComposedFrame * composed,
    core::tui::image::CellRegion region, OverlayFrameReadings & readings, std::string & error);

  // [P]: pin or unpin the displayed frame, reporting the outcome on the
  // status row.
  void toggle_pin();

  // Paint one preview frame: a two-line caption, the decoded image centred in
  // the region between caption and key hint, and the key hint on the last row.
  // With scenes pinned the image region is split into a tile grid instead.
  // While the '?' overlay is open this paints the key reference instead.
  void render(std::ostream & out, core::tui::Size term);

  // The '?' overlay: the preview's key reference as scrollable text, with
  // its own close hint pinned to the last row. No images are drawn, so the
  // overlay works the same on every backend.
  void render_help(std::ostream & out, core::tui::Size term);

  // Save the frame currently shown in the preview as a PNG.
  void save_image();

  // [D]: export the edited static TF edges as the YAML `bagwiz tf static
  // update` applies to the bag (prompts for a path like the PNG save).
  void save_edit_yaml();

  // [A]: overwrite the input bag's static TF in place with the edited edges
  // (`bagwiz tf static update` run in-process, so the atomic tmp+swap and
  // forest validation are the same). Prompts for an explicit "yes" first —
  // this is the one walk action that modifies the bag. On success the edits
  // are committed (commit_edits), so they read as the bag values they now
  // are.
  void apply_edits_to_bag();

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
  // '?' overlay state: while shown, the scroll keys move the reference, Esc
  // closes it, and every other key is swallowed — except Ctrl-C / Ctrl-D,
  // which terminate the session (see run()). Reset when the preview session
  // ends so re-entering starts on the image.
  bool show_help_ = false;
  std::size_t help_scroll_ = 0;
  std::unique_ptr<core::image::RectifyHelper> rectify_helper_;
  std::uint32_t rectify_helper_w_ = 0;
  std::uint32_t rectify_helper_h_ = 0;

  // Decoded-frame cache shared by the preview repaint and the PNG save path,
  // so navigating back to a frame (or saving the one on screen) reuses the
  // decode instead of paying for it again. Its capacity comfortably exceeds
  // the tile count, so a pinned grid never evicts a frame it still shows.
  PreviewFrameCache decoded_frames_;
  // Rectified counterpart: the lens-undistortion remap is the most expensive
  // per-pixel step of a composition, and its result per frame never changes
  // (CameraInfo is fixed for the session), so a nudge that must repaint
  // every tile still pays it only once per frame.
  PreviewFrameCache rectified_frames_;

  // Scenes pinned next to the cursor's own frame, in pin order. Survives
  // leaving and re-entering the preview, like the rectify state above.
  std::vector<ScenePin> pins_;

  // Rendered-tile cache: the escape bytes each tile last transmitted, keyed
  // by everything that could change them (see walk_preview_cache.hpp). With
  // it, plain navigation re-encodes only the live tile and replays the
  // pinned ones, so pins no longer multiply the cost of a keypress.
  TileRenderCache tile_cache_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_PREVIEW_HPP_
