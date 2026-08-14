// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/renderer.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <istream>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

}  // namespace

std::string_view pcd_property_name(core::pointcloud::PointCloudProperty prop)
{
  switch (prop) {
    case core::pointcloud::PointCloudProperty::kX:
      return "x";
    case core::pointcloud::PointCloudProperty::kY:
      return "y";
    case core::pointcloud::PointCloudProperty::kZ:
      return "z";
    case core::pointcloud::PointCloudProperty::kDistance:
      return "distance";
    case core::pointcloud::PointCloudProperty::kIntensity:
      return "intensity";
  }
  return "?";
}

std::string_view pcd_match_clock_name(const PcdOverlayState & pcd)
{
  if (pcd.last_match_key == core::pointcloud::PointCloudMatchKey::kHeaderStamp) {
    return "header";
  }
  // Record time was used. Distinguish "capture time was never on the table"
  // from "a selected topic could not honour it", because only the latter is
  // something the user can act on (by dropping that topic from the selection).
  const bool any_topic_lacks_stamps =
    std::find(pcd.topic_header_stamps.begin(), pcd.topic_header_stamps.end(), false) !=
    pcd.topic_header_stamps.end();
  return any_topic_lacks_stamps ? "header->record" : "record";
}

std::string_view pcd_scheme_name(core::pointcloud::ColorScheme s)
{
  switch (s) {
    case core::pointcloud::ColorScheme::kViridis:
      return "viridis";
    case core::pointcloud::ColorScheme::kTurbo:
      return "turbo";
    case core::pointcloud::ColorScheme::kJet:
      return "jet";
    case core::pointcloud::ColorScheme::kPlasma:
      return "plasma";
    case core::pointcloud::ColorScheme::kInferno:
      return "inferno";
    case core::pointcloud::ColorScheme::kMagma:
      return "magma";
    case core::pointcloud::ColorScheme::kRainbow:
      return "rainbow";
  }
  return "?";
}

std::optional<std::vector<std::string>> PcdOverlayController::prompt_for_topics(
  core::tui::image::ImageBackend backend)
{
  if (pcd_topics_.empty()) {
    status_ = "no PointCloud2 topics in bag";
    return std::nullopt;
  }

  // Pre-check the topics that are currently active so the picker reflects
  // the existing selection instead of resetting every topic to unchecked.
  std::vector<bool> checked(pcd_topics_.size(), false);
  for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
    checked[i] =
      std::find(pcd_.topics.begin(), pcd_.topics.end(), pcd_topics_[i]) != pcd_.topics.end();
  }
  std::size_t cursor = 0;
  bool done = false;
  bool cancelled = false;

  while (!done) {
    core::tui::image::clear_image(std::cout, backend);
    std::cout << "\x1B[2J";
    const auto term = core::tui::query_terminal_size();
    core::tui::draw_line(
      std::cout, 1,
      "  Select PointCloud2 topics (Space toggle, Enter confirm, Esc/q cancel):", term.cols);
    for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
      const std::string marker = (i == cursor) ? ">" : " ";
      const std::string box = checked[i] ? "[x]" : "[ ]";
      core::tui::draw_line(
        std::cout, static_cast<int>(i) + 3, fmt::format("  {} {} {}", marker, box, pcd_topics_[i]),
        term.cols);
    }
    std::cout.flush();

    switch (core::read_key_event()) {
      case core::KeyEvent::kScrollUp:
        if (cursor > 0) {
          --cursor;
        }
        break;
      case core::KeyEvent::kScrollDown:
        if (cursor + 1 < pcd_topics_.size()) {
          ++cursor;
        }
        break;
      case core::KeyEvent::kFirst:
        cursor = 0;
        break;
      case core::KeyEvent::kLast:
        cursor = pcd_topics_.size() - 1;
        break;
      case core::KeyEvent::kNext:
        checked[cursor] = !checked[cursor];
        break;
      case core::KeyEvent::kConfirm:
        done = true;
        break;
      case core::KeyEvent::kQuit:
        done = true;
        cancelled = true;
        break;
      case core::KeyEvent::kResize:
      default:
        break;
    }
  }

  if (cancelled) {
    status_ = "(topic selection cancelled)";
    return std::nullopt;
  }

  std::vector<std::string> selected;
  for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
    if (checked[i]) {
      selected.push_back(pcd_topics_[i]);
    }
  }

  // Confirming a selection identical to what is already applied would kick
  // off a full (slow) bag re-scan for no visible change, so short-circuit it
  // exactly like Esc/cancel — the overlay on screen is already correct.
  auto sorted = [](std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    return v;
  };
  if (!pcd_.topics.empty() && sorted(selected) == sorted(pcd_.topics)) {
    status_ = "(topic selection unchanged)";
    return std::nullopt;
  }
  return selected;
}

PcdOverlayController::~PcdOverlayController()
{
  if (worker_.joinable()) {
    cancel_.store(true, std::memory_order_relaxed);
    worker_.join();
  }
}

bool PcdOverlayController::start_initialize(const std::vector<std::string> & topics)
{
  for (const auto & topic : topics) {
    bool valid = false;
    for (const auto & t : reader_.topics()) {
      if (t.name == topic && t.type == kPointCloudType) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      status_ = fmt::format("not a PointCloud2 topic: {}", topic);
      return false;
    }
  }

  // Cheap pre-check: the projection needs TF, so fail before launching the
  // scan when the bag has none.
  bool has_tf = false;
  for (const auto & t : reader_.topics()) {
    if (t.type == kTfMessageType) {
      has_tf = true;
      break;
    }
  }
  if (!has_tf) {
    status_ = "no tf2_msgs/msg/TFMessage topics found; cannot resolve point-cloud transform";
    return false;
  }

  // Replace any in-flight or finished-but-unreaped load.
  if (worker_.joinable()) {
    cancel_.store(true, std::memory_order_relaxed);
    worker_.join();
  }

  cancel_.store(false, std::memory_order_relaxed);
  percent_.store(0, std::memory_order_relaxed);
  scan_result_ = std::make_unique<OverlayScanResult>();
  pcd_topics_selected_ = topics;
  state_.store(InitState::kRunning, std::memory_order_release);
  worker_ = std::thread([this, topics_copy = topics] {
    scan_overlay_inputs(
      input_path_, topics_copy, cancel_,
      [this](double fraction) {
        percent_.store(static_cast<int>(fraction * 100.0), std::memory_order_relaxed);
      },
      *scan_result_);
    if (cancel_.load(std::memory_order_relaxed)) {
      // Cancelled by a newer start_initialize() or the destructor; the result
      // is discarded, so just return to idle.
      state_.store(InitState::kIdle, std::memory_order_release);
      return;
    }
    state_.store(
      scan_result_->error.empty() ? InitState::kSucceeded : InitState::kFailed,
      std::memory_order_release);
  });
  status_ = "loading pcd overlay ... 0%";
  return true;
}

PcdOverlayController::InitState PcdOverlayController::poll_initialize()
{
  const InitState s = state_.load(std::memory_order_acquire);
  if (s == InitState::kIdle || s == InitState::kRunning) {
    return s;
  }
  // Terminal state: the worker has published its result and is joinable.
  worker_.join();

  if (s == InitState::kFailed) {
    status_ = scan_result_->error;
    scan_result_.reset();
    state_.store(InitState::kIdle, std::memory_order_release);
    return s;
  }

  std::vector<core::pointcloud::PointCloudFetcher> fetchers;
  fetchers.reserve(scan_result_->entries.size());
  for (std::size_t i = 0; i < scan_result_->entries.size(); ++i) {
    fetchers.emplace_back(
      input_path_, pcd_topics_selected_[i], std::move(scan_result_->entries[i]));
  }
  pcd_fetchers_ = std::move(fetchers);
  ranged_record_ns_.assign(pcd_fetchers_.size(), std::nullopt);
  active_scan_ = std::move(scan_result_);

  pcd_.topics = pcd_topics_selected_;
  pcd_.topic_header_stamps = active_scan_->header_stamps_present;
  pcd_.has_intensity = false;
  pcd_.ranges = core::pointcloud::PropertyRanges{};
  const auto range = pcd_.ranges.resolve(pcd_.property);
  pcd_.computed_min = range.first;
  pcd_.computed_max = range.second;
  pcd_.enabled = true;

  state_.store(InitState::kIdle, std::memory_order_release);
  status_ = "pcd overlay ready";
  return s;
}

void PcdOverlayController::maybe_overlay(
  core::image::PackedRaster * raster, std::int64_t record_stamp_ns,
  const std::optional<core::image::CameraInfo> & camera_info,
  const EnsureRectifyHelper & ensure_helper, bool rectify_enabled)
{
  if (
    raster == nullptr || !pcd_.enabled || pcd_.topics.empty() || pcd_fetchers_.empty() ||
    active_scan_ == nullptr) {
    return;
  }
  // Past this point the overlay is live, so the info row will read the
  // frame-match values below. Clear them up front: every remaining early
  // return means nothing was matched this frame, and reporting a previous
  // frame's clock or residual next to a projection error would be a lie.
  pcd_.last_match_key = core::pointcloud::PointCloudMatchKey::kRecordTime;
  pcd_.last_residual_ns.reset();

  if (!camera_info.has_value()) {
    status_ = "pcd projection requires camera_info";
    return;
  }

  const auto & img = *raster;
  const auto * helper = ensure_helper(img.width, img.height);
  if (helper == nullptr) {
    status_ = "pcd projection requires camera_info";
    return;
  }
  const auto effective_ci = helper->effective_camera_info();

  std::vector<core::pointcloud::ProjectedPoint> all_points;
  std::string last_error;
  // What this frame actually matched on, recomputed from scratch so the info
  // row can never show a value carried over from an earlier frame or topic
  // selection. A single topic falling back to record time downgrades the whole
  // frame's reading, because that is what the user is seeing on screen.
  std::optional<core::pointcloud::PointCloudMatchKey> effective_key;
  std::optional<std::int64_t> worst_residual_ns;
  const auto magnitude = [](std::int64_t v) { return v < 0 ? -v : v; };

  for (std::size_t i = 0; i < pcd_fetchers_.size(); ++i) {
    // Pair the frame with the point cloud nearest in the clock both sides
    // share (see core::pointcloud::choose_frame_match): capture time when the
    // frame carries a header.stamp and every message of this topic does too,
    // else bag record time. The chosen target is also the TF-lookup time, and
    // the TF buffer is keyed by each transform's own header.stamp, so
    // capture-time matching keeps that lookup on the right clock as well.
    const bool topic_has_stamps =
      i < pcd_.topic_header_stamps.size() && pcd_.topic_header_stamps[i];
    const auto match =
      core::pointcloud::choose_frame_match(img.header_stamp_ns, record_stamp_ns, topic_has_stamps);
    const std::int64_t match_ns = match.target_ns;

    std::string error;
    const auto * cloud = pcd_fetchers_[i].fetch(match_ns, match.key, error);
    if (cloud == nullptr) {
      last_error = std::move(error);
      continue;
    }

    if (
      !effective_key.has_value() ||
      match.key == core::pointcloud::PointCloudMatchKey::kRecordTime) {
      effective_key = match.key;
    }
    // The true capture-time residual, meaningful whichever clock matched:
    // how far the chosen cloud was actually taken from this frame. Undefined
    // when either side left its stamp unset. Across topics the largest
    // magnitude wins, since that is the worst misalignment on screen.
    if (img.header_stamp_ns > 0 && cloud->timestamp_ns > 0) {
      const std::int64_t residual = cloud->timestamp_ns - img.header_stamp_ns;
      if (!worst_residual_ns.has_value() || magnitude(residual) > magnitude(*worst_residual_ns)) {
        worst_residual_ns = residual;
      }
    }

    // Fold newly displayed clouds into the running colour ranges (once per
    // cloud; the fetcher's cache address identifies it), then refresh the
    // active property's auto range. This replaces the up-front full-bag
    // min/max parse the overlay used to run at initialization.
    const std::int64_t cloud_record_ns = pcd_fetchers_[i].cached_record_ns();
    if (ranged_record_ns_[i] != cloud_record_ns) {
      if (core::pointcloud::accumulate_property_ranges(*cloud, pcd_.ranges, error)) {
        ranged_record_ns_[i] = cloud_record_ns;
        pcd_.has_intensity = pcd_.ranges.has_intensity;
        const auto range = pcd_.ranges.resolve(pcd_.property);
        pcd_.computed_min = range.first;
        pcd_.computed_max = range.second;
      } else {
        last_error = std::move(error);
      }
    }

    const auto projected = core::pointcloud::project_cloud_for_frame(
      *cloud, effective_ci, active_scan_->tf_buffer, img.width, img.height, pcd_.property,
      /*use_rectified=*/rectify_enabled, match_ns);
    if (!projected.ok()) {
      last_error = std::move(projected.error);
      continue;
    }
    all_points.insert(all_points.end(), projected.points.begin(), projected.points.end());
  }

  // No fetch succeeded -> nothing was matched on any clock; report the safe
  // reading rather than leaving the previous frame's.
  pcd_.last_match_key = effective_key.value_or(core::pointcloud::PointCloudMatchKey::kRecordTime);
  pcd_.last_residual_ns = worst_residual_ns;

  if (all_points.empty()) {
    if (!last_error.empty()) {
      status_ = std::move(last_error);
    }
    return;
  }

  const double vmin = pcd_.auto_range ? pcd_.computed_min : pcd_.manual_min;
  const double vmax = pcd_.auto_range ? pcd_.computed_max : pcd_.manual_max;
  const auto err = core::pointcloud::overlay_projected_points(
    img, all_points, vmin, vmax, pcd_.scheme, pcd_.point_size, pcd_.alpha, *raster);
  if (!err.empty()) {
    status_ = err;
  }
}

void PcdOverlayController::cycle_property()
{
  // Cycle order: distance -> intensity (only when the cloud carries it)
  //           -> x -> y -> z -> distance ...
  auto next = [&](core::pointcloud::PointCloudProperty cur) {
    using Property = core::pointcloud::PointCloudProperty;
    switch (cur) {
      case Property::kDistance:
        return pcd_.has_intensity ? Property::kIntensity : Property::kX;
      case Property::kIntensity:
        return Property::kX;
      case Property::kX:
        return Property::kY;
      case Property::kY:
        return Property::kZ;
      case Property::kZ:
        return Property::kDistance;
    }
    return Property::kDistance;
  };
  pcd_.property = next(pcd_.property);
  // Auto range reuses the running extent accumulated from displayed clouds,
  // so switching property is O(1) and never re-reads the bag (an unobserved
  // property resolves to a neutral [0, 1] until its first cloud is shown).
  if (pcd_.auto_range) {
    const auto range = pcd_.ranges.resolve(pcd_.property);
    pcd_.computed_min = range.first;
    pcd_.computed_max = range.second;
  }
}

void PcdOverlayController::cycle_scheme()
{
  switch (pcd_.scheme) {
    case core::pointcloud::ColorScheme::kJet:
      pcd_.scheme = core::pointcloud::ColorScheme::kViridis;
      break;
    case core::pointcloud::ColorScheme::kViridis:
      pcd_.scheme = core::pointcloud::ColorScheme::kTurbo;
      break;
    case core::pointcloud::ColorScheme::kTurbo:
      pcd_.scheme = core::pointcloud::ColorScheme::kPlasma;
      break;
    case core::pointcloud::ColorScheme::kPlasma:
      pcd_.scheme = core::pointcloud::ColorScheme::kInferno;
      break;
    case core::pointcloud::ColorScheme::kInferno:
      pcd_.scheme = core::pointcloud::ColorScheme::kMagma;
      break;
    case core::pointcloud::ColorScheme::kMagma:
      pcd_.scheme = core::pointcloud::ColorScheme::kRainbow;
      break;
    case core::pointcloud::ColorScheme::kRainbow:
      pcd_.scheme = core::pointcloud::ColorScheme::kJet;
      break;
  }
}

void PcdOverlayController::prompt_for_range(
  core::tui::ScrollablePager & pager, core::tui::image::ImageBackend backend)
{
  if (pcd_.auto_range) {
    pcd_.auto_range = false;
    core::tui::image::clear_image(std::cout, backend);
    std::cout << "\x1B[2J";
    pager.with_line_input([&](std::istream & in, std::ostream & out) {
      out << "Manual min: ";
      out.flush();
      std::string line;
      if (std::getline(in, line)) {
        try {
          pcd_.manual_min = std::stod(line);
        } catch (...) {
        }
      }
      out << "Manual max: ";
      out.flush();
      if (std::getline(in, line)) {
        try {
          pcd_.manual_max = std::stod(line);
        } catch (...) {
        }
      }
    });
  } else {
    pcd_.auto_range = true;
  }
}

}  // namespace bagwiz::commands
