// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/commands/tf_static_update.hpp"
#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/renderer.hpp"
#include "bagwiz/core/tui/width.hpp"
#include "walk_frame.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_help.hpp"   // NOLINT(build/include_subdir) src-local shared header
#include "walk_save.hpp"   // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

// Upper bound on decoded preview frames kept in memory at once. Decoding a
// frame (JPEG/PNG via libav, or a raw copy) dominates repaint cost, so we cache
// recently viewed rasters; the cap bounds memory (each raster is
// width * height * 3 bytes, so a handful of HD frames is a few tens of MB).
constexpr std::size_t kPreviewCacheCapacity = 16;

// Upper bound on rectified frames kept alongside. Smaller than the decode
// cache: the hot set is the pinned tiles plus the live frame, and each entry
// is another full-size raster.
constexpr std::size_t kRectifiedCacheCapacity = 8;

}  // namespace

ImagePreviewSession::ImagePreviewSession(
  MessageCursor & cursor, PcdOverlayController & overlay, core::tui::ScrollablePager & pager,
  std::string & status, std::string topic_name, std::string type_name,
  core::tui::image::TerminalImageCaps image_caps,
  const std::optional<core::image::CameraInfo> & camera_info, const std::string & camera_info_error)
: cursor_(cursor),
  overlay_(overlay),
  pager_(pager),
  status_(status),
  topic_name_(std::move(topic_name)),
  type_name_(std::move(type_name)),
  image_caps_(image_caps),
  camera_info_(camera_info),
  camera_info_error_(camera_info_error),
  decoded_frames_(kPreviewCacheCapacity),
  rectified_frames_(kRectifiedCacheCapacity)
{
}

core::image::RectifyHelper * ImagePreviewSession::ensure_rectify_helper(
  std::uint32_t w, std::uint32_t h)
{
  if (!camera_info_.has_value()) {
    return nullptr;
  }
  if (!rectify_helper_ || rectify_helper_w_ != w || rectify_helper_h_ != h) {
    rectify_helper_ = std::make_unique<core::image::RectifyHelper>(*camera_info_, w, h);
    rectify_helper_w_ = w;
    rectify_helper_h_ = h;
  }
  return rectify_helper_.get();
}

ImagePreviewSession::ComposedFrame ImagePreviewSession::compose_frame(
  std::size_t idx, std::size_t slot)
{
  ComposedFrame composed;
  auto & pr = composed.frame;
  const auto & msg = cursor_.cache()[idx];
  auto base = decoded_frames_.get(
    idx, [&] { return core::image::to_packed_raster(type_name_, msg.payload); });
  if (base.raster == nullptr) {
    pr.error = std::move(base.error);
    return composed;
  }

  // The frame the overlay is drawn on: the base decode, or its cached
  // rectified remap. Rectification per frame is deterministic (CameraInfo is
  // fixed), so the remap — the most expensive per-pixel step here — runs
  // once per frame however many repaints show it.
  const core::image::PackedRaster * source = base.raster;
  if (rectify_enabled_) {
    if (auto * helper = ensure_rectify_helper(source->width, source->height); helper != nullptr) {
      auto rectified = rectified_frames_.get(idx, [&] {
        // Only the scalars are copied from the source; its pixel buffer is
        // read in place by the remap, and the remapped bytes land directly in
        // the new raster instead of overwriting a pointless copy.
        core::image::PackedRasterResult produced;
        core::image::PackedRaster rect;
        rect.width = source->width;
        rect.height = source->height;
        rect.encoding = "bgr8";
        rect.header_stamp_ns = source->header_stamp_ns;
        const auto remapped = helper->remap(source->bgr, source->width * 3);
        rect.bgr.assign(remapped.begin(), remapped.end());
        produced.raster = std::move(rect);
        return produced;
      });
      if (rectified.raster != nullptr) {
        source = rectified.raster;
      }
    }
  }

  pr.raster = *source;  // copy the pristine frame before mutating overlays
  if (overlay_.state().enabled) {
    composed.readings = overlay_.maybe_overlay(
      slot, &*pr.raster, msg.timestamp_ns, camera_info_,
      [this](std::uint32_t w, std::uint32_t h) { return ensure_rectify_helper(w, h); },
      rectify_enabled_);
  }
  return composed;
}

std::vector<TileCaption> ImagePreviewSession::tile_captions() const
{
  const std::size_t index = cursor_.index();
  const auto & cache = cursor_.cache();

  std::vector<TileCaption> tiles;
  tiles.reserve(pins_.size() + 1);

  TileCaption live;
  live.pin = ScenePin{index, cache[index].timestamp_ns};
  live.live = true;
  // A pin on the frame the cursor is showing labels this one tile as both,
  // rather than drawing the same frame twice.
  live.pin_number = pin_number_of(pins_, index).value_or(0);
  tiles.push_back(live);

  for (std::size_t i = 0; i < pins_.size(); ++i) {
    if (pins_[i].index == index) {
      continue;  // already on screen as the live tile
    }
    TileCaption tile;
    tile.pin = pins_[i];
    tile.pin_number = i + 1;
    tiles.push_back(tile);
  }
  return tiles;
}

std::string ImagePreviewSession::encode_tile(
  const core::image::PackedRasterResult & pr, core::tui::image::CellRegion region,
  std::string & error)
{
  error.clear();
  if (!pr.ok()) {
    error = fmt::format("cannot decode this message: {}", pr.error);
    return {};
  }
  std::ostringstream buffer;
  const std::string err = core::tui::image::render_image(buffer, *pr.raster, region, image_caps_);
  if (!err.empty()) {
    error = fmt::format("preview unavailable: {}", err);
    return {};
  }
  return std::move(buffer).str();
}

void ImagePreviewSession::emit_tile(
  std::ostream & out, std::size_t slot, std::size_t msg_index, const ComposedFrame * composed,
  core::tui::image::CellRegion region, OverlayFrameReadings & readings, std::string & error)
{
  const TileRenderKey key = tile_render_key(
    overlay_.state(), rectify_enabled_, overlay_.composition_generation(), msg_index, region,
    image_caps_);
  if (const auto * hit = tile_cache_.find(slot, key); hit != nullptr) {
    // Byte-identical to what a recomposition would transmit, so replay it:
    // this is what keeps pinned tiles free on plain navigation.
    out << hit->payload;
    readings = hit->readings;
    error.clear();
    return;
  }

  ComposedFrame local;
  if (composed == nullptr) {
    local = compose_frame(msg_index, slot);
    composed = &local;
  }
  readings = composed->readings;
  std::string payload = encode_tile(composed->frame, region, error);
  if (!error.empty()) {
    return;  // failures are not cached; the next repaint retries
  }
  out << payload;
  // The composition itself can move the key: a pinned tile's first cloud can
  // stretch the auto range mid-compose, and the pixels above were painted
  // with the stretched bounds. Store under the post-compose key so the entry
  // replays next repaint instead of guaranteeing itself a miss.
  const TileRenderKey stored_key = tile_render_key(
    overlay_.state(), rectify_enabled_, overlay_.composition_generation(), msg_index, region,
    image_caps_);
  tile_cache_.store(slot, TileRenderEntry{stored_key, std::move(payload), composed->readings});
}

void ImagePreviewSession::toggle_pin()
{
  status_.clear();
  if (overlay_.state().topics.empty()) {
    // Pinning exists to judge one projection against several scenes, so it
    // waits for an overlay topic the same way the adjustment keys do.
    status_ = "pin: select a pcd topic first ([t])";
    return;
  }
  const std::size_t index = cursor_.index();
  const ScenePin pin{index, cursor_.cache()[index].timestamp_ns};
  switch (toggle_scene_pin(pins_, pin)) {
    case PinOutcome::kPinned:
      status_ = fmt::format("pinned #{} ({}/{})", index, pins_.size(), kMaxScenePins);
      break;
    case PinOutcome::kUnpinned:
      status_ = fmt::format("unpinned #{} ({}/{})", index, pins_.size(), kMaxScenePins);
      break;
    case PinOutcome::kFull:
      status_ = fmt::format("pin limit reached ({} scenes)", kMaxScenePins);
      return;  // nothing changed, so the slots still match the pins
  }
  // Unpinning shifts the pins above it down one slot, so their tiles reload
  // their clouds once; the slots the grid no longer shows are released here
  // rather than holding a cloud (and a rendered payload) each for the rest
  // of the session.
  overlay_.retain_slots(pins_.size() + 1);
  tile_cache_.trim(pins_.size() + 1);
}

void ImagePreviewSession::render(std::ostream & out, core::tui::Size term)
{
  if (show_help_) {
    render_help(out, term);
    return;
  }
  const std::size_t index = cursor_.index();
  const auto & cache = cursor_.cache();
  const bool exhausted = cursor_.exhausted();
  const auto & pcd = overlay_.state();
  const auto & status = status_;
  const auto & image_caps = image_caps_;
  const auto & topic_name = topic_name_;

  const int rows = std::max(1, term.rows);
  const int cols = std::max(1, term.cols);

  // Bracket the whole repaint in a synchronized update so the terminal keeps
  // showing the current frame until the new one is fully transmitted, then
  // swaps atomically. Without this the clear below blanks the screen for as
  // long as the terminal needs to receive and decode the next image, which
  // reads as a one-frame "blink" on every prev/next. Unsupported terminals
  // ignore the mode and behave exactly as before.
  core::tui::begin_synchronized_update(out);

  // Drop any previously transmitted graphics and wipe the screen so kitty
  // placements do not accumulate across navigation/resize.
  core::tui::image::clear_image(out, image_caps.backend);
  out << "\x1B[2J";

  const char * total_suffix = exhausted ? "" : "+";
  const std::size_t last_loaded_index = cache.size() - 1;
  // The tiles this repaint shows, and the cursor's own frame among them. The
  // live tile is composed first because the info row and the grid layout are
  // both derived from it.
  const auto tiles_meta = tile_captions();
  const auto live = compose_frame(index, kLiveOverlaySlot);
  const auto & pr = live.frame;

  std::string info;
  if (pr.ok()) {
    const auto & img = *pr.raster;
    info = fmt::format(
      "  {}x{}   [{} / {}{}]", img.width, img.height, index, last_loaded_index, total_suffix);
  } else {
    info = fmt::format("  [{} / {}{}]", index, last_loaded_index, total_suffix);
  }
  // Surface the save outcome (or any transient message) on the info row;
  // navigate() clears `status` on a cursor move, so it disappears as soon as
  // the user pages to another frame.
  if (!status.empty()) {
    info += fmt::format("   {}", status);
  }
  // Every state field reads as "field: value" with the value emphasised so
  // it stands out from the label. The SGR wrapper is zero display-width (see
  // width.cpp), so it does not perturb the wrap/truncate accounting below.
  auto hl = [](auto && value) { return fmt::format("\x1B[1;36m{}\x1B[0m", value); };

  // State badges only while a state is on; the settings themselves surface
  // transiently on the status row as their keys adjust them, so the
  // persistent row stays one compact line.
  if (rectify_enabled_) {
    info += fmt::format("   {}", hl("rect"));
  }
  if (!pcd.topics.empty() && pcd.enabled) {
    info += fmt::format("   {}", hl("pcd"));
    // The frame-match residual comes from the last maybe_overlay(), which
    // runs inside the compose_frame() above — but only while the overlay is
    // enabled, so it cannot go stale behind a switched-off overlay. With a
    // scene grid on screen every tile's caption carries its own delta, which
    // is the comparison that matters; repeating the live tile's here would
    // only be noise.
    if (pins_.empty()) {
      const std::string residual_text =
        pcd.last_residual_ns.has_value()
          ? fmt::format("{:+.1f}ms", static_cast<double>(*pcd.last_residual_ns) / 1e6)
          : "n/a";
      info += fmt::format(" Δ {}", hl(residual_text));
    }
  }
  if (!pins_.empty()) {
    info += fmt::format("   pins: {}", hl(fmt::format("{}/{}", pins_.size(), kMaxScenePins)));
  }
  const auto & edit = overlay_.edit_state();
  if (edit.editing) {
    info += fmt::format("   {}", edit_info_text(edit));
  }

  // Header: the topic/type row and the info row, each wrapped to width the
  // same way the YAML view's header and the legend below are, so a narrow
  // terminal shows the full text on continuation lines instead of truncating
  // it at the right edge. The image region starts just below the wrapped
  // header (see region_row).
  std::vector<std::string> header_lines;
  append_wrapped(header_lines, fmt::format("  {}", topic_name), cols);
  append_wrapped(header_lines, info, cols);
  for (std::size_t i = 0; i < header_lines.size(); ++i) {
    core::tui::draw_line(out, 1 + static_cast<int>(i), header_lines[i], cols);
  }

  // Wrap the key legend the way the YAML footer does, so a very narrow
  // terminal shows every key on continuation lines instead of truncating the
  // row. The wrapped legend is pinned to the bottom and the image region
  // above shrinks to make room, mirroring how the YAML view derives its body
  // height from the wrapped footer. The footer carries only the working set
  // of the current mode — everything else lives in the '?' reference.
  const std::vector<std::string> legend_lines =
    core::tui::wrap_to_width(preview_footer_legend(!pcd.topics.empty(), edit.editing), cols);
  const int legend_top = std::max(1, rows - static_cast<int>(legend_lines.size()) + 1);

  // Image region: from the row just below the wrapped header down to the row
  // above the first legend line.
  const int region_row = 1 + static_cast<int>(header_lines.size());
  const int region_rows = std::max(1, legend_top - region_row);
  core::tui::image::CellRegion region;
  region.row = region_row;
  region.col = 1;
  region.rows = region_rows;
  region.cols = cols;

  // The live frame alone, centred in the whole region: the only view when
  // nothing is pinned, and the fallback when no tile grid fits.
  const auto emit_live_full_region = [&] {
    OverlayFrameReadings readings;
    std::string err;
    emit_tile(out, kLiveOverlaySlot, index, &live, region, readings, err);
    if (!err.empty()) {
      core::tui::draw_line(out, region_row, fmt::format("  {}", err), cols);
    }
  };

  if (pins_.empty()) {
    // Nothing pinned: exactly the view the preview had before scene pinning
    // existed.
    emit_live_full_region();
  } else {
    // The grid shape is chosen against the live frame's aspect; a frame that
    // failed to decode carries none, so fall back to a 16:9 tile.
    const std::uint32_t fit_w = pr.ok() ? pr.raster->width : 1920U;
    const std::uint32_t fit_h = pr.ok() ? pr.raster->height : 1080U;
    auto tiles = tile_regions(region, tiles_meta.size(), fit_w, fit_h, image_caps.cell);
    std::string notice;
    if (tiles.empty()) {
      // No grid fits this terminal. Show the live frame alone and say so,
      // rather than a mosaic too small to judge an alignment in.
      notice = "  pinned scenes hidden: terminal too small for a grid";
      tiles = tile_regions(region, 1, fit_w, fit_h, image_caps.cell);
    }
    if (tiles.empty()) {
      // Not even one tile fits: draw the live frame across the whole region so
      // the renderer's own "too small" reason reaches the screen.
      emit_live_full_region();
    }

    // Caption rows are composed whole before being drawn: draw_line() erases
    // the row it writes, so two tiles sharing a row have to be padded into one
    // string first.
    const std::int64_t live_stamp = cache[index].timestamp_ns;
    std::string caption_text;
    int caption_row = 0;
    const auto flush_captions = [&]() {
      if (caption_row > 0) {
        core::tui::draw_line(out, caption_row, caption_text, cols);
      }
    };
    for (std::size_t i = 0; i < tiles.size(); ++i) {
      if (tiles[i].caption_row != caption_row) {
        flush_captions();
        caption_row = tiles[i].caption_row;
        caption_text.clear();
      }

      // Emit before captioning: the caption reports what this tile itself
      // paired with, which is the per-scene reading the grid exists to compare,
      // and carries the failure when the tile could not be drawn at all. A
      // pinned tile whose render key is unchanged replays its cached bytes
      // (and cached readings) without recomposing.
      TileCaption meta = tiles_meta[i];
      OverlayFrameReadings readings;
      std::string error;
      if (i == 0) {
        emit_tile(out, kLiveOverlaySlot, index, &live, tiles[i].image, readings, error);
      } else {
        emit_tile(out, meta.pin_number, meta.pin.index, nullptr, tiles[i].image, readings, error);
      }
      meta.residual_ns = readings.residual_ns;

      std::string text = (i == 0 && !notice.empty()) ? notice : tile_caption(meta, live_stamp);
      if (!error.empty()) {
        text += fmt::format("  {}", error);
      }
      const int pad = tiles[i].col - 1 - core::tui::display_width(caption_text);
      if (pad > 0) {
        caption_text.append(static_cast<std::size_t>(pad), ' ');
      }
      // Reset after every segment: truncation drops a CSI sequence whole, so a
      // caption cut short of its own reset would tint the next tile's.
      caption_text += core::tui::truncate_to_width(text, tiles[i].cols);
      caption_text += "\x1B[0m";
    }
    flush_captions();
  }

  for (std::size_t i = 0; i < legend_lines.size(); ++i) {
    core::tui::draw_line(out, legend_top + static_cast<int>(i), legend_lines[i], cols);
  }

  // Close the synchronized update: the terminal now reveals the fully
  // assembled frame in one atomic swap.
  core::tui::end_synchronized_update(out);
  out.flush();
}

void ImagePreviewSession::render_help(std::ostream & out, core::tui::Size term)
{
  const int rows = std::max(1, term.rows);
  const int cols = std::max(1, term.cols);

  core::tui::begin_synchronized_update(out);
  core::tui::image::clear_image(out, image_caps_.backend);
  out << "\x1B[2J";

  std::vector<std::string> body;
  append_wrapped(body, "  bagwiz walk keys", cols);
  body.emplace_back();
  for (const auto & line : preview_help_lines()) {
    append_wrapped(body, line, cols);
  }

  // The close hint is pinned to the bottom like the preview's own footer;
  // the reference scrolls in the region above it.
  const std::vector<std::string> footer = core::tui::wrap_to_width("  [Esc] back", cols);
  const int footer_top = std::max(1, rows - static_cast<int>(footer.size()) + 1);
  const int body_rows = std::max(1, footer_top - 1);

  const std::size_t max_scroll =
    body.size() > static_cast<std::size_t>(body_rows) ? body.size() - body_rows : 0;
  help_scroll_ = std::min(help_scroll_, max_scroll);
  for (int i = 0; i < body_rows && help_scroll_ + i < body.size(); ++i) {
    core::tui::draw_line(out, 1 + i, body[help_scroll_ + static_cast<std::size_t>(i)], cols);
  }
  for (std::size_t i = 0; i < footer.size(); ++i) {
    core::tui::draw_line(out, footer_top + static_cast<int>(i), footer[i], cols);
  }

  core::tui::end_synchronized_update(out);
  out.flush();
}

void ImagePreviewSession::save_image()
{
  const std::size_t index = cursor_.index();
  const auto & topic_name = topic_name_;
  const auto & image_caps = image_caps_;

  status_.clear();
  // The live tile, at full resolution: a pinned grid changes what is on
  // screen, not what [S] writes.
  const auto composed = compose_frame(index, kLiveOverlaySlot);
  const auto & pr = composed.frame;
  if (!pr.ok()) {
    status_ = fmt::format("cannot save: {}", pr.error);
    return;
  }
  const auto encoded = core::image::encode_png(*pr.raster);
  if (!encoded.ok()) {
    status_ = fmt::format("cannot save: {}", encoded.error);
    return;
  }

  const std::string default_base = fmt::format("{}_{}.png", topic_for_filename(topic_name), index);
  std::filesystem::path cwd;
  try {
    cwd = std::filesystem::current_path();
  } catch (const std::exception & e) {
    status_ = fmt::format("cannot resolve working directory: {}", e.what());
    return;
  }

  // Drop the on-screen graphic before switching to cooked-mode line input so
  // the prompt is not drawn over a kitty placement; run() repaints the frame
  // afterward.
  core::tui::image::clear_image(std::cout, image_caps.backend);
  std::cout << "\x1B[2J";
  std::cout.flush();

  const auto & bytes = *encoded.png;
  save_bytes_with_prompt(
    pager_, "Save image path", cwd, default_base,
    std::span<const std::byte>(bytes.data(), bytes.size()), status_);
}

void ImagePreviewSession::save_edit_yaml()
{
  status_.clear();
  const auto & input = overlay_.input_path();
  const std::string yaml = edit_yaml(
    overlay_.edit_state(), fmt::format("edited with bagwiz walk from {}", input.string()));
  if (yaml.empty()) {
    status_ = "(no extrinsic edits to export)";
    return;
  }

  std::filesystem::path cwd;
  try {
    cwd = std::filesystem::current_path();
  } catch (const std::exception & e) {
    status_ = fmt::format("cannot resolve working directory: {}", e.what());
    return;
  }
  // Name the file after the bag; a rosbag2 directory given with a trailing
  // separator has an empty stem, so fall back through its parent.
  std::string stem = input.stem().string();
  if (stem.empty()) {
    stem = input.parent_path().stem().string();
  }
  if (stem.empty()) {
    stem = "bag";
  }
  const std::string default_base = fmt::format("{}_tf_static_edit.yaml", stem);

  // Drop the on-screen graphic before switching to cooked-mode line input so
  // the prompt is not drawn over a kitty placement; run() repaints the frame
  // afterward.
  core::tui::image::clear_image(std::cout, image_caps_.backend);
  std::cout << "\x1B[2J";
  std::cout.flush();

  save_bytes_with_prompt(
    pager_, "Save TF static YAML path", cwd, default_base, std::as_bytes(std::span{yaml}), status_);
}

void ImagePreviewSession::apply_edits_to_bag()
{
  status_.clear();
  auto & edit = overlay_.edit_state();
  const auto transforms = edited_transforms(edit);
  if (transforms.empty()) {
    status_ = "(no extrinsic edits to apply)";
    return;
  }
  const auto & input = overlay_.input_path();

  // Drop the on-screen graphic before switching to cooked-mode line input so
  // the prompt is not drawn over a kitty placement; run() repaints the frame
  // afterward.
  core::tui::image::clear_image(std::cout, image_caps_.backend);
  std::cout << "\x1B[2J";
  std::cout.flush();

  bool applied = false;
  bool cancelled = false;
  pager_.with_line_input([&](std::istream & in, std::ostream & out) {
    // Show exactly what will be written, in the same rendering the exit
    // summary uses, so the confirmation is informed.
    out << edit_summary(edit);
    out << fmt::format(
      "\nOverwrite the static TF of {} IN PLACE with the {} edited edge(s) above?\n"
      "The bag is rewritten atomically (a full copy, then a swap) — this can take a\n"
      "while for a large bag and cannot be undone. [D] exports a YAML instead.\n"
      "Type \"yes\" to overwrite, anything else cancels: ",
      input.string(), transforms.size());
    out.flush();
    std::string line;
    if (!std::getline(in, line) || line != "yes") {
      cancelled = true;
      return;
    }

    // Run the update while still in cooked mode: its log lines (progress,
    // the added/updated summary, any warnings) then print sequentially
    // instead of scribbling over the repainted preview.
    const int rc = run_tf_static_update(
      input, transforms, "/tf_static", /*output_path=*/std::nullopt, /*overwrite=*/false);
    applied = rc == 0;
    out << (applied ? "\nDone." : "\nFailed; the bag is unchanged.")
        << " Press Enter to return to the preview.";
    out.flush();
    std::string pause;
    std::getline(in, pause);
  });

  if (cancelled) {
    status_ = "(apply cancelled; bag unchanged)";
    return;
  }
  if (!applied) {
    status_ = "apply failed; bag unchanged";
    return;
  }
  // The edited values ARE the bag values now: rebase the edges onto them so
  // deltas read zero, [D] and the exit summary have nothing left to report,
  // and [0] resets to the value the bag actually carries. The projection is
  // untouched — the TF buffer already holds these values.
  commit_edits(edit);
  status_ = fmt::format("updated {} ({} edge(s) applied)", input.string(), transforms.size());
}

ImagePreviewSession::Exit ImagePreviewSession::run()
{
  auto & pcd = overlay_.state();
  const auto & camera_info = camera_info_;
  const auto & camera_info_error = camera_info_error_;
  const auto & image_caps = image_caps_;

  std::ostream & out = std::cout;
  bool running = true;
  // Set when Ctrl-C / Ctrl-D asked to end the whole walk session; the caller
  // turns Exit::kTerminate into the pager's quit.
  Exit exit = Exit::kBack;
  bool needs_render = true;
  // True while an overlay initialization is (or may be) unfinished on its
  // worker thread: the loop then reads keys with a timeout so the status row
  // keeps animating the load's progress and the finished result is applied.
  bool load_in_flight = overlay_.is_loading();
  while (running) {
    if (needs_render) {
      render(out, core::tui::query_terminal_size());
      needs_render = false;
    }

    core::KeyEvent ev = core::KeyEvent::kUnknown;
    if (load_in_flight) {
      // The progress text rides the pager's normal status row instead of an
      // indicators-library bar: that library does its own cursor control on
      // std::cout, which fights the pager's alternate-screen / synchronized
      // update rendering.
      const auto maybe_ev = core::read_key_event(100);
      const auto init_state = overlay_.poll_initialize();
      if (init_state == PcdOverlayController::InitState::kRunning) {
        const std::string msg = fmt::format("loading pcd overlay ... {}%", overlay_.load_percent());
        if (msg != status_) {
          status_ = msg;
          needs_render = true;
        }
      } else {
        // kSucceeded / kFailed: poll_initialize() applied the result (or the
        // error) to the overlay state and status row.
        load_in_flight = false;
        needs_render = true;
      }
      if (!maybe_ev.has_value()) {
        continue;
      }
      ev = *maybe_ev;
    } else {
      ev = core::read_key_event();
    }

    if (show_help_) {
      // The overlay accepts only its own keys: Esc closes it ('?' only
      // opens it), the scroll keys page it, resize repaints it, and every
      // other key — q included — is swallowed so a reference lookup cannot
      // act behind the card. The one exception is Ctrl-C / Ctrl-D, which
      // terminates the session from any screen, overlays included.
      switch (ev) {
        case core::KeyEvent::kQuit:
          exit = Exit::kTerminate;
          running = false;
          break;
        case core::KeyEvent::kBack:
          show_help_ = false;
          needs_render = true;
          break;
        case core::KeyEvent::kScrollDown:
          ++help_scroll_;  // clamped against the content in render_help
          needs_render = true;
          break;
        case core::KeyEvent::kScrollUp:
          if (help_scroll_ > 0) {
            --help_scroll_;
            needs_render = true;
          }
          break;
        case core::KeyEvent::kScrollHead:
          help_scroll_ = 0;
          needs_render = true;
          break;
        case core::KeyEvent::kScrollTail:
          help_scroll_ = std::numeric_limits<std::size_t>::max();  // clamped in render_help
          needs_render = true;
          break;
        case core::KeyEvent::kResize:
          needs_render = true;
          break;
        default:
          break;  // swallowed behind the reference
      }
      continue;
    }

    switch (ev) {
      case core::KeyEvent::kNext:
        // Re-decode only when the cursor actually moved; otherwise the frame
        // is unchanged and a full decode + scale would be wasted.
        needs_render = cursor_.navigate(MsgNav::kNext);
        break;
      case core::KeyEvent::kPrev:
        needs_render = cursor_.navigate(MsgNav::kPrev);
        break;
      case core::KeyEvent::kFirst:
        needs_render = cursor_.navigate(MsgNav::kFirst);
        break;
      case core::KeyEvent::kLast:
        needs_render = cursor_.navigate(MsgNav::kLast);
        break;
      case core::KeyEvent::kStepForward1s:
        needs_render = cursor_.navigate(MsgNav::kStepForward1s);
        break;
      case core::KeyEvent::kStepForward10s:
        needs_render = cursor_.navigate(MsgNav::kStepForward10s);
        break;
      case core::KeyEvent::kStepBackward1s:
        needs_render = cursor_.navigate(MsgNav::kStepBackward1s);
        break;
      case core::KeyEvent::kStepBackward10s:
        needs_render = cursor_.navigate(MsgNav::kStepBackward10s);
        break;
      case core::KeyEvent::kResize:
        needs_render = true;  // geometry changed: re-fit and re-render
        break;
      case core::KeyEvent::kSaveYaml:
        // In the preview, [S] saves the displayed frame as a PNG (the YAML
        // view's [S] still saves YAML). Always repaint so the save status is
        // shown and the prompt's screen clear is undone.
        save_image();
        needs_render = true;
        break;
      case core::KeyEvent::kToggleRectify:
        // Toggling rectify also re-aims the pcd overlay: with rectify on
        // points project onto the rectified image, with it off they project
        // onto the raw image using the lens distortion (see maybe_overlay).
        if (!camera_info.has_value()) {
          status_ =
            camera_info_error.empty() ? "rectify: no camera_info" : "rectify: " + camera_info_error;
        } else {
          rectify_enabled_ = !rectify_enabled_;
          // The info row only badges the on state, so the toggle-off press
          // still gets visible feedback here.
          status_ = rectify_enabled_ ? "rectify: on" : "rectify: off";
        }
        needs_render = true;
        break;
      case core::KeyEvent::kToggleProjectPcd:
        if (!camera_info.has_value()) {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        } else if (load_in_flight) {
          status_ = "pcd overlay still loading ...";
        } else if (pcd.topics.empty()) {
          const auto [outcome, topics] = overlay_.prompt_for_topics(image_caps.backend);
          if (outcome == PickerOutcome::kTerminate) {
            exit = Exit::kTerminate;
            running = false;
          } else if (outcome == PickerOutcome::kConfirmed && !topics.empty()) {
            load_in_flight = overlay_.start_initialize(topics);
          }
        } else {
          pcd.enabled = !pcd.enabled;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kSelectPcdTopic:
        if (camera_info.has_value()) {
          if (load_in_flight) {
            status_ = "pcd overlay still loading ...";
          } else {
            const auto [outcome, topics] = overlay_.prompt_for_topics(image_caps.backend);
            if (outcome == PickerOutcome::kTerminate) {
              exit = Exit::kTerminate;
              running = false;
            } else if (outcome == PickerOutcome::kConfirmed) {
              if (topics.empty()) {
                pcd.enabled = false;
              } else {
                load_in_flight = overlay_.start_initialize(topics);
              }
            }
          }
        } else {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdProperty:
        overlay_.cycle_property();
        // The settings left the persistent info row in the footer diet, so
        // each adjustment reports its new value transiently instead (the
        // status clears on the next cursor move).
        status_ = fmt::format("property: {}", pcd_property_name(pcd.property));
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdScheme:
        overlay_.cycle_scheme();
        status_ = fmt::format("scheme: {}", pcd_scheme_name(pcd.scheme));
        needs_render = true;
        break;
      case core::KeyEvent::kTogglePcdRange:
        overlay_.prompt_for_range(pager_, image_caps.backend);
        // Like the other adjustment keys: the range left the persistent info
        // row, so the toggle reports where it landed.
        status_ = pcd.auto_range
                    ? "range: auto"
                    : fmt::format("range: {:.2f}-{:.2f}", pcd.manual_min, pcd.manual_max);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeUp:
        pcd.point_size = std::min(pcd.point_size + 1, 64U);
        status_ = fmt::format("point size: {}", pcd.point_size);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeDown:
        pcd.point_size = std::max(pcd.point_size - 1, 1U);
        status_ = fmt::format("point size: {}", pcd.point_size);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaUp:
        pcd.alpha = std::min(pcd.alpha + 0.1f, 1.0f);
        status_ = fmt::format("alpha: {:.1f}", pcd.alpha);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaDown:
        pcd.alpha = std::max(pcd.alpha - 0.1f, 0.0f);
        status_ = fmt::format("alpha: {:.1f}", pcd.alpha);
        needs_render = true;
        break;
      case core::KeyEvent::kToggleEditExtrinsic: {
        auto & edit = overlay_.edit_state();
        if (edit.editing) {
          edit.editing = false;
          status_ = "(edit mode off; edits stay applied)";
        } else if (!camera_info.has_value()) {
          status_ =
            camera_info_error.empty() ? "edit: no camera_info" : "edit: " + camera_info_error;
        } else if (load_in_flight) {
          status_ = "pcd overlay still loading ...";
        } else if (overlay_.tf_buffer() == nullptr || !pcd.enabled) {
          status_ = "edit: enable the pcd overlay first ([p])";
        } else {
          // Re-entering resumes the previous edge; the first entry derives
          // the candidates and, when there are several, asks which edge to
          // edit.
          const bool had_candidates = !edit.edges.empty();
          if (had_candidates || overlay_.refresh_edit_candidates(camera_info->frame_id)) {
            if (!had_candidates && edit.edges.size() > 1) {
              const PickerOutcome outcome = overlay_.prompt_for_edge(image_caps.backend);
              if (outcome == PickerOutcome::kTerminate) {
                exit = Exit::kTerminate;
                running = false;
              } else {
                edit.editing = outcome == PickerOutcome::kConfirmed;
              }
            } else {
              edit.editing = true;
              status_ = fmt::format("editing {}", edge_label(edit.edges[edit.active]));
            }
          }
        }
        needs_render = true;
        break;
      }
      case core::KeyEvent::kSelectEditEdge: {
        auto & edit = overlay_.edit_state();
        if (!camera_info.has_value()) {
          status_ =
            camera_info_error.empty() ? "edit: no camera_info" : "edit: " + camera_info_error;
        } else if (load_in_flight) {
          status_ = "pcd overlay still loading ...";
        } else if (overlay_.tf_buffer() == nullptr || !pcd.enabled) {
          status_ = "edit: enable the pcd overlay first ([p])";
        } else if (overlay_.refresh_edit_candidates(camera_info->frame_id)) {
          const PickerOutcome outcome = overlay_.prompt_for_edge(image_caps.backend);
          if (outcome == PickerOutcome::kTerminate) {
            exit = Exit::kTerminate;
            running = false;
          } else {
            edit.editing = outcome == PickerOutcome::kConfirmed;
          }
        }
        needs_render = true;
        break;
      }
      case core::KeyEvent::kEditTransXUp:
      case core::KeyEvent::kEditTransXDown:
      case core::KeyEvent::kEditTransYUp:
      case core::KeyEvent::kEditTransYDown:
      case core::KeyEvent::kEditTransZUp:
      case core::KeyEvent::kEditTransZDown:
      case core::KeyEvent::kEditRollUp:
      case core::KeyEvent::kEditRollDown:
      case core::KeyEvent::kEditPitchUp:
      case core::KeyEvent::kEditPitchDown:
      case core::KeyEvent::kEditYawUp:
      case core::KeyEvent::kEditYawDown:
      case core::KeyEvent::kEditStepUp:
      case core::KeyEvent::kEditStepDown:
      case core::KeyEvent::kEditReset: {
        // Outside the edit mode the nudge letters stay inert, so a stray
        // press cannot silently move a calibration.
        auto & edit = overlay_.edit_state();
        if (edit.editing && apply_edit_key(edit, ev)) {
          overlay_.apply_active_edit();
          needs_render = true;
        }
        break;
      }
      case core::KeyEvent::kEditDumpYaml:
        save_edit_yaml();
        needs_render = true;
        break;
      case core::KeyEvent::kEditApplyToBag:
        // Blocked while a scan worker is reading the bag: the scan opened the
        // bag before the swap, so letting it finish afterwards would install
        // a TF buffer built from the OLD content while the committed edits no
        // longer re-apply on top — the overlay would silently revert on
        // screen while the bag on disk carries the fix.
        if (load_in_flight) {
          status_ = "pcd overlay still loading ...";
        } else {
          apply_edits_to_bag();
        }
        needs_render = true;
        break;
      case core::KeyEvent::kPinScene:
        toggle_pin();
        needs_render = true;
        break;
      case core::KeyEvent::kHelp:
        show_help_ = true;
        help_scroll_ = 0;
        needs_render = true;
        break;
      case core::KeyEvent::kBack: {
        // Esc backs out one level: the edit mode first, the preview next
        // (the YAML view's Esc then quits walk).
        auto & edit = overlay_.edit_state();
        if (edit.editing) {
          edit.editing = false;
          status_ = "(edit mode off; edits stay applied)";
        } else {
          running = false;
        }
        needs_render = true;
        break;
      }
      case core::KeyEvent::kQuit:
        // Ctrl-C / Ctrl-D: leave the preview and end the whole walk session.
        exit = Exit::kTerminate;
        running = false;
        break;
      default:
        // Scroll / expand keys are inert in the preview — and so is 'q'
        // (kQuitView): quitting walk on 'q' is the YAML view's binding.
        break;
    }
  }
  // Re-entering the preview starts on the image, not on a leftover help.
  show_help_ = false;
  // Hand a clean screen back to the pager for the YAML repaint.
  core::tui::image::clear_image(out, image_caps.backend);
  out << "\x1B[2J";
  out.flush();
  return exit;
}

}  // namespace bagwiz::commands
