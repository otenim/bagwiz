// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_OVERLAY_HPP_
#define COMMANDS__WALK_OVERLAY_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/rectify.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "walk_edit.hpp"          // NOLINT(build/include_subdir) src-local shared header
#include "walk_overlay_scan.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// Point-cloud projection overlay of `bagwiz walk`'s image preview: the
// overlay state, the interactive topic picker, the background initialization
// (one combined bag scan on a worker thread), and the per-frame projection.
// Moved out of walk.cpp verbatim; the interactive parts stay TTY-coupled by
// design.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// Overlay key-handling state. The default view is distance coloured with the
// viridis scheme, 2px points at full opacity over an auto-computed range —
// the same defaults `generate video --pcd` renders with, so the preview and
// the encoded video agree without touching any key.
struct PcdOverlayState
{
  bool enabled = false;
  std::vector<std::string> topics;
  core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
  core::pointcloud::ColorScheme scheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 2;
  float alpha = 1.0f;
  bool auto_range = true;
  double manual_min = 0.0;
  double manual_max = 1.0;
  double computed_min = 0.0;
  double computed_max = 1.0;
  bool has_intensity = false;
  // Running min/max of every colour property, expanded from each cloud as it
  // is first displayed. Computing these up front would require parsing every
  // cloud in the bag; the running variant converges after the first frames
  // and keeps [f] property switches free of bag re-reads.
  core::pointcloud::PropertyRanges ranges;
  // Parallel to `topics`: whether that topic's stamps form a pure capture-time
  // axis (OverlayScanResult::header_stamps_present). A false entry pins that
  // topic to record-time matching.
  std::vector<bool> topic_header_stamps;
  // What the last maybe_overlay() actually matched on, and how far the chosen
  // cloud's capture time landed from the frame's. Both are only ever read
  // while `enabled` — the preview shows them only then — so they cannot go
  // stale behind a disabled overlay. `last_residual_ns` is nullopt when either
  // side left its header.stamp unset, which makes the residual undefined.
  core::pointcloud::PointCloudMatchKey last_match_key =
    core::pointcloud::PointCloudMatchKey::kRecordTime;
  std::optional<std::int64_t> last_residual_ns;
  // Parallel to `topics`: the frame_id of the last cloud each topic fetched,
  // empty until its first successful fetch. The extrinsic edit mode derives
  // its editable edges from these frames.
  std::vector<std::string> last_cloud_frames;
  // TF-lookup time of the last successful cloud match. The edit mode
  // resolves its chains at this time, so a dynamic link on the path is
  // evaluated where the overlay evaluates it.
  std::optional<std::int64_t> last_match_ns;
};

// Display names used by the preview info row.
[[nodiscard]] std::string_view pcd_property_name(core::pointcloud::PointCloudProperty prop);
[[nodiscard]] std::string_view pcd_scheme_name(core::pointcloud::ColorScheme scheme);
// The clock the overlay paired frames and clouds on: "header" when capture
// time was used, "record" when it was never available, and "header->record"
// when a selected topic could not honour capture time and forced the fallback.
[[nodiscard]] std::string_view pcd_match_clock_name(const PcdOverlayState & pcd);

class PcdOverlayController
{
public:
  // Lazily creates (and caches) the RectifyHelper for a frame size; walk's
  // preview session owns the helper because rectification is applied to the
  // displayed frame independently of the overlay.
  using EnsureRectifyHelper =
    std::function<core::image::RectifyHelper *(std::uint32_t, std::uint32_t)>;

  // Progress of a start_initialize() worker.
  enum class InitState { kIdle, kRunning, kSucceeded, kFailed };

  // `pcd_topics` are the bag's PointCloud2 topics (the picker candidates).
  // They are not pre-filtered by message count — that count can require a
  // full bag scan — so an empty topic fails the initialization scan with a
  // "has no messages" status instead. `status` is the shared UI status row:
  // picker cancellations, initialization failures, and projection errors are
  // reported there.
  PcdOverlayController(
    std::filesystem::path input_path, const io::BagReader & reader,
    std::vector<std::string> pcd_topics, std::string & status)
  : input_path_(std::move(input_path)),
    reader_(reader),
    pcd_topics_(std::move(pcd_topics)),
    status_(status)
  {
  }

  // Cancels and joins any in-flight initialization worker.
  ~PcdOverlayController();

  PcdOverlayController(const PcdOverlayController &) = delete;
  PcdOverlayController & operator=(const PcdOverlayController &) = delete;
  PcdOverlayController(PcdOverlayController &&) = delete;
  PcdOverlayController & operator=(PcdOverlayController &&) = delete;

  [[nodiscard]] PcdOverlayState & state() noexcept { return pcd_; }
  [[nodiscard]] const PcdOverlayState & state() const noexcept { return pcd_; }

  // Interactive checkbox picker over the candidate topics. Returns the
  // selected topics, or std::nullopt when the user cancelled or confirmed an
  // unchanged selection (both leave the current overlay untouched).
  // `backend` selects the graphics-clear protocol for the prompt redraws.
  [[nodiscard]] std::optional<std::vector<std::string>> prompt_for_topics(
    core::tui::image::ImageBackend backend);

  // Start the overlay initialization on a worker thread: one combined bag
  // scan that decodes the TF topics and collects the selected topics' cloud
  // timestamps (see walk_overlay_scan). Returns false synchronously — with
  // `status` set — when a selected topic is not PointCloud2 or the bag has
  // no TF topic; otherwise returns true and the load proceeds in the
  // background. An in-flight load is cancelled and replaced.
  bool start_initialize(const std::vector<std::string> & topics);

  // True while a start_initialize() worker is in flight.
  [[nodiscard]] bool is_loading() const
  {
    return state_.load(std::memory_order_acquire) == InitState::kRunning;
  }
  // Whole-percent progress of the in-flight load (0 while idle).
  [[nodiscard]] int load_percent() const { return percent_.load(std::memory_order_relaxed); }

  // Reap a finished worker. kRunning: still in flight. kSucceeded: results
  // are applied — fetchers and TF buffer installed, the overlay enabled —
  // and the state returns to kIdle. kFailed: `status` carries the reason.
  // kIdle: no load is active.
  InitState poll_initialize();

  // Project the fetched point clouds onto `raster` when the overlay is
  // enabled and initialized. `record_stamp_ns` is the walked message's bag
  // record time; together with the raster's own header.stamp it gives
  // choose_frame_match both clocks, and the per-topic capture-time capability
  // decides which one pairs the frame with each cloud. `rectify_enabled`
  // selects projection onto the rectified vs raw image. Updates the state's
  // `last_match_key` / `last_residual_ns` readings for the preview info row.
  void maybe_overlay(
    core::image::PackedRaster * raster, std::int64_t record_stamp_ns,
    const std::optional<core::image::CameraInfo> & camera_info,
    const EnsureRectifyHelper & ensure_helper, bool rectify_enabled);

  // [f]: distance -> intensity (when present) -> x -> y -> z -> distance.
  void cycle_property();
  // [c]: jet -> viridis -> turbo -> plasma -> inferno -> magma -> rainbow.
  void cycle_scheme();
  // [r]: switch to a manually prompted range, or back to auto.
  void prompt_for_range(core::tui::ScrollablePager & pager, core::tui::image::ImageBackend backend);

  // Extrinsic edit mode (see walk_edit.hpp). The state lives here because
  // its edits are written into the overlay's TF buffer, whose lifecycle this
  // controller owns.
  [[nodiscard]] ExtrinsicEditState & edit_state() noexcept { return edit_; }
  [[nodiscard]] const ExtrinsicEditState & edit_state() const noexcept { return edit_; }

  // The TF buffer behind the active overlay, or nullptr before the first
  // successful initialization.
  [[nodiscard]] tf2::BufferCore * tf_buffer() noexcept
  {
    return active_scan_ ? &active_scan_->tf_buffer : nullptr;
  }

  // The bag the overlay (and so the edit mode) reads; the YAML export names
  // it in the file's header comment and default filename.
  [[nodiscard]] const std::filesystem::path & input_path() const noexcept { return input_path_; }

  // [e]/[E]: re-derive the editable candidates from the currently displayed
  // cloud frames and `camera_frame`, carrying edited values over
  // (walk_edit's carry_over_edits). Returns false — with `status` set — when
  // no cloud has been displayed yet or no static edge lies on the chains.
  bool refresh_edit_candidates(const std::string & camera_frame);

  // Interactive single-select list over the candidates. Returns true when a
  // row was confirmed (stored in edit_state().active); Esc/q cancels.
  bool prompt_for_edge(core::tui::image::ImageBackend backend);

  // Write the active / every edited edge into the live TF buffer, so the
  // next projection composes with the edited values. apply_all_edits() runs
  // after an initialization swap re-created the buffer from the bag.
  void apply_active_edit();
  void apply_all_edits();

private:
  std::filesystem::path input_path_;
  const io::BagReader & reader_;
  std::vector<std::string> pcd_topics_;
  std::string & status_;
  PcdOverlayState pcd_;
  ExtrinsicEditState edit_;
  std::vector<core::pointcloud::PointCloudFetcher> pcd_fetchers_;
  // Parallel to pcd_fetchers_: the bag record time of the last cloud of each
  // fetcher already folded into pcd_.ranges, so repeated display of the same
  // cloud does not re-accumulate. Record time, not the cloud pointer: the
  // fetcher's cache is an inline optional move-assigned in place, so every
  // cloud it returns has the same address (see
  // PointCloudFetcher::cached_record_ns). nullopt until the first fold.
  std::vector<std::optional<std::int64_t>> ranged_record_ns_;

  // Initialization worker state. Results are heap-allocated because
  // tf2::BufferCore is immovable: the worker writes scan_result_ while
  // active_scan_ keeps serving the currently enabled overlay; on success the
  // pointers swap roles (move-assign), on failure scan_result_ is dropped
  // and the previous overlay stays intact. The UI thread consumes results
  // only after the worker reports a terminal state (the acquire/release
  // pairing on state_ orders the handoff).
  std::thread worker_;
  std::unique_ptr<OverlayScanResult> scan_result_;
  std::unique_ptr<OverlayScanResult> active_scan_;
  std::vector<std::string> pcd_topics_selected_;
  std::atomic<InitState> state_{InitState::kIdle};
  std::atomic<bool> cancel_{false};
  std::atomic<int> percent_{0};
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_OVERLAY_HPP_
