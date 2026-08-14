// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/renderer.hpp"
#include "bagwiz/core/tui/width.hpp"
#include "walk_frame.hpp"           // NOLINT(build/include_subdir) src-local shared header
#include "walk_preview_legend.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_save.hpp"            // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <span>
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
  decoded_frames_(kPreviewCacheCapacity)
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

void ImagePreviewSession::maybe_rectify(core::image::PackedRaster * raster)
{
  if (raster == nullptr) {
    return;
  }
  auto * helper = ensure_rectify_helper(raster->width, raster->height);
  if (helper == nullptr) {
    return;
  }
  const auto remapped = helper->remap(raster->bgr, raster->width * 3);
  raster->bgr.assign(remapped.begin(), remapped.end());
  raster->encoding = "bgr8";
}

core::image::PackedRasterResult ImagePreviewSession::compose_frame(std::size_t idx)
{
  core::image::PackedRasterResult pr;
  const auto & msg = cursor_.cache()[idx];
  auto hit = decoded_frames_.get(idx, type_name_, msg.payload);
  if (hit.raster == nullptr) {
    pr.error = std::move(hit.error);
    return pr;
  }
  pr.raster = *hit.raster;  // copy the pristine base before mutating overlays
  if (rectify_enabled_) {
    maybe_rectify(&*pr.raster);
  }
  if (overlay_.state().enabled) {
    overlay_.maybe_overlay(
      &*pr.raster, msg.timestamp_ns, camera_info_,
      [this](std::uint32_t w, std::uint32_t h) { return ensure_rectify_helper(w, h); },
      rectify_enabled_);
  }
  return pr;
}

void ImagePreviewSession::render(std::ostream & out, core::tui::Size term)
{
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
  auto pr = compose_frame(index);

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

  info += fmt::format("   rectify: {}", hl(rectify_enabled_ ? "on" : "off"));
  if (!pcd.topics.empty()) {
    const std::string range_text =
      pcd.auto_range ? "auto" : fmt::format("{:.2f}-{:.2f}", pcd.manual_min, pcd.manual_max);
    info += fmt::format("   pcd: {}", hl(pcd.enabled ? "on" : "off"));
    // The frame-match readings come from the last maybe_overlay(), which runs
    // inside the compose_frame() above — but only while the overlay is
    // enabled. Showing them only then keeps them from going stale behind a
    // switched-off overlay.
    if (pcd.enabled) {
      const std::string residual_text =
        pcd.last_residual_ns.has_value()
          ? fmt::format("{:+.1f}ms", static_cast<double>(*pcd.last_residual_ns) / 1e6)
          : "n/a";
      info += fmt::format("   match: {}   Δ: {}", hl(pcd_match_clock_name(pcd)), hl(residual_text));
    }
    info += fmt::format(
      "   property: {}   range: {}   scheme: {}   size: {}   alpha: {}",
      hl(pcd_property_name(pcd.property)), hl(range_text), hl(pcd_scheme_name(pcd.scheme)),
      hl(pcd.point_size), hl(fmt::format("{:.1f}", pcd.alpha)));
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

  // Wrap the key legend the way the YAML footer does, so a narrow terminal
  // shows every key on continuation lines instead of truncating the row. The
  // wrapped legend is pinned to the bottom and the image region above shrinks
  // to make room, mirroring how the YAML view derives its body height from
  // the wrapped footer.
  // The overlay adjustment keys are gated on the same condition the info row
  // uses to show pcd state, so the legend and the state readout agree on when
  // an overlay topic is in play.
  const std::vector<std::string> legend_lines =
    core::tui::wrap_to_width(build_preview_legend(!pcd.topics.empty(), edit.editing), cols);
  const int legend_top = std::max(1, rows - static_cast<int>(legend_lines.size()) + 1);

  // Image region: from the row just below the wrapped header down to the row
  // above the first legend line.
  const int region_row = 1 + static_cast<int>(header_lines.size());
  const int region_rows = std::max(1, legend_top - region_row);
  if (pr.ok()) {
    core::tui::image::CellRegion region;
    region.row = region_row;
    region.col = 1;
    region.rows = region_rows;
    region.cols = cols;
    const std::string err = core::tui::image::render_image(out, *pr.raster, region, image_caps);
    if (!err.empty()) {
      core::tui::draw_line(out, region_row, fmt::format("  preview unavailable: {}", err), cols);
    }
  } else {
    core::tui::draw_line(
      out, region_row, fmt::format("  cannot decode this message: {}", pr.error), cols);
  }

  for (std::size_t i = 0; i < legend_lines.size(); ++i) {
    core::tui::draw_line(out, legend_top + static_cast<int>(i), legend_lines[i], cols);
  }

  // Close the synchronized update: the terminal now reveals the fully
  // assembled frame in one atomic swap.
  core::tui::end_synchronized_update(out);
  out.flush();
}

void ImagePreviewSession::save_image()
{
  const std::size_t index = cursor_.index();
  const auto & topic_name = topic_name_;
  const auto & image_caps = image_caps_;

  status_.clear();
  auto pr = compose_frame(index);
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

void ImagePreviewSession::run()
{
  auto & pcd = overlay_.state();
  const auto & camera_info = camera_info_;
  const auto & camera_info_error = camera_info_error_;
  const auto & image_caps = image_caps_;

  std::ostream & out = std::cout;
  bool running = true;
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
        }
        needs_render = true;
        break;
      case core::KeyEvent::kToggleProjectPcd:
        if (!camera_info.has_value()) {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        } else if (load_in_flight) {
          status_ = "pcd overlay still loading ...";
        } else if (pcd.topics.empty()) {
          if (auto topics = overlay_.prompt_for_topics(image_caps.backend);
              topics.has_value() && !topics->empty()) {
            load_in_flight = overlay_.start_initialize(*topics);
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
          } else if (auto topics = overlay_.prompt_for_topics(image_caps.backend);
                     topics.has_value()) {
            if (topics->empty()) {
              pcd.enabled = false;
            } else {
              load_in_flight = overlay_.start_initialize(*topics);
            }
          }
        } else {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdProperty:
        overlay_.cycle_property();
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdScheme:
        overlay_.cycle_scheme();
        needs_render = true;
        break;
      case core::KeyEvent::kTogglePcdRange:
        overlay_.prompt_for_range(pager_, image_caps.backend);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeUp:
        pcd.point_size = std::min(pcd.point_size + 1, 64U);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeDown:
        pcd.point_size = std::max(pcd.point_size - 1, 1U);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaUp:
        pcd.alpha = std::min(pcd.alpha + 0.1f, 1.0f);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaDown:
        pcd.alpha = std::max(pcd.alpha - 0.1f, 0.0f);
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
              edit.editing = overlay_.prompt_for_edge(image_caps.backend);
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
        } else if (
          overlay_.refresh_edit_candidates(camera_info->frame_id) &&
          overlay_.prompt_for_edge(image_caps.backend)) {
          edit.editing = true;
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
      case core::KeyEvent::kQuit:
        running = false;
        break;
      default:
        break;  // scroll / expand keys are inert in the preview
    }
  }
  // Hand a clean screen back to the pager for the YAML repaint.
  core::tui::image::clear_image(out, image_caps.backend);
  out << "\x1B[2J";
  out.flush();
}

}  // namespace bagwiz::commands
