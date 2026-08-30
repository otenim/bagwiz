// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_CAMERA_PANEL_HPP_
#define COMMANDS__MOVIFY_CAMERA_PANEL_HPP_

#include "bagwiz/commands/movify.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/rectify.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_cloud_source.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_inputs.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_layout.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"         // NOLINT(build/include_subdir) src-local shared header
#include "movify_pose_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

// The panel that shows one image topic: the decode of its messages to a
// canonical BGR24 raster, the nearest-message reader a non-clock panel
// follows the ticks with, the per-panel scale / rectification / point-cloud
// overlay renderer, and the Panel that ties them together. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Owned decode buffer that survives across BagReader::next() calls, which
// invalidate raw payload spans.
struct FrameBuffer
{
  std::int64_t timestamp_ns = 0;     // bag record time
  std::int64_t header_stamp_ns = 0;  // image header.stamp (0 if unset)
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t step = 0;
  core::video::SourcePixelFormat pixel_format = core::video::SourcePixelFormat::kBgr8;
  std::string encoding;
  std::vector<std::byte> data;
};

// The decode half of the frame pipeline: normalizes each message (raw Image
// or CompressedImage) to a canonical packed BGR24 raster. Failures come back
// as an error string; the encode loop logs them with the tick's frame index.
// Resizing is a separate step (resize_frame) because each panel picks its
// own scale.
class FrameNormalizer
{
public:
  explicit FrameNormalizer(std::string topic_type) : topic_type_(std::move(topic_type)) {}

  // Decode one message payload into an owned canonical BGR24 frame. Returns
  // nullopt with `error` set when the payload does not decode.
  [[nodiscard]] std::optional<FrameBuffer> decode(
    std::int64_t timestamp_ns, std::span<const std::byte> payload, std::string & error) const;

private:
  std::string topic_type_;
  // libav decode handles kept across frames so the codec context and its
  // buffers are not reopened per frame. mutable because decode() is logically
  // const; each panel owns its FrameNormalizer, so there is no cross-thread
  // sharing.
  mutable core::image::ImageDecoder decoder_;
};

// Resize a decoded frame in place to exactly out_w x out_h (INTER_AREA when
// shrinking, INTER_LINEAR when growing). A no-op when the frame already has
// that size. Returns "" on success, or the error when a target dimension is
// zero.
[[nodiscard]] std::string resize_frame(
  FrameBuffer & frame, std::uint32_t out_w, std::uint32_t out_h);

// Streaming reader over ONE non-clock image topic, answering "the message
// whose bag record time is nearest t" exactly, via one-message lookahead.
// Targets must be non-decreasing across fetch() calls (the encode loop's
// clock-driven ticks guarantee this).
class NearestMessageSource
{
public:
  struct Message
  {
    std::int64_t record_ns = 0;
    std::vector<std::byte> payload;
  };

  // Open the bag filtered to `topic`. Returns nullptr and fills `error` when
  // the bag does not open.
  [[nodiscard]] static std::unique_ptr<NearestMessageSource> open(
    const std::filesystem::path & input, const std::string & topic, std::string & error);

  // The message nearest `target_ns` by record time (ties prefer the earlier
  // message). The pointer stays valid until the next fetch(). Returns nullptr
  // with an empty error when the topic has no messages at all, and nullptr
  // with a filled error on a read failure.
  [[nodiscard]] const Message * fetch(std::int64_t target_ns, std::string & error);

private:
  explicit NearestMessageSource(std::unique_ptr<io::BagReader> reader) : reader_(std::move(reader))
  {
  }

  std::unique_ptr<io::BagReader> reader_;
  // The boundary pair around the last target: the latest message at or before
  // it and the first after it. Either side may be unset at the stream ends.
  std::optional<Message> before_;
  std::optional<Message> after_;
  bool eof_ = false;
};

// Point-cloud overlay styling, shared by every panel.
struct VideoOverlayParams
{
  double property_min = 0.0;
  double property_max = 0.0;
  core::pointcloud::ColorScheme colorscheme = core::pointcloud::ColorScheme::kJet;
  std::uint32_t point_size = 2;
  float alpha = 1.0f;
};

// The render-time geometry of one panel for one tick: the render size (the
// frame scaled by the panel's scale) and the CameraInfo effective at that
// size. Snapshotted at prepare() time so a tick's projection and rendering
// stay consistent even when a later tick's prepare() re-fits the panel.
struct ViewRenderGeometry
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  core::image::CameraInfo camera_info;  // effective at (width, height)
  bool has_camera_info = false;
  bool rectify = false;
};

// One camera panel's render pipeline: fixes the panel's scale from its first
// frame, scales/rectifies/overlays each selected frame, and pastes it
// centered into the panel's grid cell (with black bars when the aspect
// ratios differ).
class ViewRenderer
{
public:
  // `fixed_scale` panels (the clock) render every frame at
  // lround(w*scale) x lround(h*scale) and reject a native-size change; fit
  // panels (every other one) pick a uniform scale at their first frame that
  // fits the cell preserving aspect ratio, re-fitting if the native size
  // changes. `camera_info` is the panel's UNSCALED calibration, non-null iff
  // the panel rectifies or projects point clouds.
  ViewRenderer(
    const core::image::CameraInfo * camera_info, bool rectify, const VideoOverlayParams & params,
    std::optional<double> fixed_scale);

  // Fix the scale on first use and compute the render geometry for a native
  // w x h frame. Returns nullopt with `error` set when the result would be
  // zero-size or (fixed-scale panels) the native size changed mid-run.
  [[nodiscard]] std::optional<ViewRenderGeometry> prepare(
    std::uint32_t native_w, std::uint32_t native_h, std::uint32_t cell_w, std::uint32_t cell_h,
    std::string & error);

  // Pin the render size to exact pixel dimensions, derived from the --width
  // output-width constraint rather than a scale factor. Must be called before
  // the first prepare(); the fixed-panel native-size lock still applies.
  void set_fixed_render_size(std::uint32_t width, std::uint32_t height)
  {
    fixed_size_ = {width, height};
  }

  // Render `frame` (already at geom's size — the panel resizes right after
  // decode) into `cell`: rectify, overlay `projected` when non-null, and
  // paste centered. Returns "" on success, or the error.
  [[nodiscard]] std::string render(
    const FrameBuffer & frame, const ViewRenderGeometry & geom,
    const std::vector<core::pointcloud::ProjectedPoint> * projected, const CellView & cell);

private:
  const core::image::CameraInfo * camera_info_;
  bool rectify_;
  VideoOverlayParams params_;
  std::optional<double> fixed_scale_;
  std::optional<std::pair<std::uint32_t, std::uint32_t>> fixed_size_;
  std::uint32_t native_w_ = 0;
  std::uint32_t native_h_ = 0;
  std::unique_ptr<core::image::RectifyHelper> rectify_helper_;
  std::uint32_t helper_w_ = 0;
  std::uint32_t helper_h_ = 0;
};

// The Panel over one image topic. In the clock role it decodes each tick's
// own message and its first frame's render size becomes the grid's cell size;
// in the follower role it shows the message of its topic whose bag record
// time is nearest the tick, decoding a message once and repeating it while
// its topic is slower than the clock. Either role projects the panel's
// point-cloud topics onto the frame before pasting it into the cell.
class CameraPanel final : public Panel
{
public:
  // What both roles share: the topic (for log lines) and its message type,
  // how the panel rectifies, and what it overlays.
  struct Options
  {
    std::string topic;
    std::string topic_type;
    // The panel's UNSCALED calibration; non-null iff the panel rectifies or
    // projects point clouds. Must outlive the panel.
    const core::image::CameraInfo * camera_info = nullptr;
    bool rectify = false;
    VideoOverlayParams overlay;
    core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
    // The panel's point-cloud topics, as indexes into the CloudSources.
    std::vector<std::size_t> cloud_indexes;
    // The trajectory overlay, drawn over the frame as plates `pose_width_m`
    // wide; null when the run has none. Must outlive the panel.
    const PoseOverlay * pose = nullptr;
    double pose_width_m = 2.0;
  };

  // The clock role's render-size rule: a scale factor (--resize) applied to
  // the native frame, or the output width split across the grid columns
  // (--width), which wins when set.
  struct ClockSizing
  {
    double resize_scale = 1.0;
    std::optional<std::uint32_t> total_width;
    std::uint32_t grid_cols = 1;
  };

  // Clock role: renders each tick's own payload.
  CameraPanel(Options options, ClockSizing sizing, CloudSources * clouds);
  // Follower role: renders the message of `source`'s topic nearest each tick.
  CameraPanel(Options options, std::unique_ptr<NearestMessageSource> source, CloudSources * clouds);

  [[nodiscard]] std::string select(const TickInfo & tick, PanelSize cell) override;
  [[nodiscard]] std::optional<PanelSize> clock_cell_size() const override;
  // Clock role: start decoding `tick` on one of kDecodeAheadDepth spare
  // decoders (when one is free; otherwise the tick's select() decodes
  // inline). A follower ignores it.
  void prefetch(const TickInfo & tick) override;
  [[nodiscard]] std::string render(const CellView & cell) override;

private:
  [[nodiscard]] std::string select_clock(const TickInfo & tick);
  // Draw the --pose trajectory over the frame pasted into `cell` as plates
  // on the ground, projected through the panel's camera at the frame's time.
  [[nodiscard]] std::string draw_pose(const CellView & cell) const;

  // A decode started by prefetch(): its own decoder, a copy of the payload,
  // the tick it is for, and the frame its select() takes over.
  struct AheadResult
  {
    std::optional<FrameBuffer> frame;
    std::string error;
  };
  struct AheadSlot
  {
    explicit AheadSlot(std::string topic_type) : normalizer(std::move(topic_type)) {}
    FrameNormalizer normalizer;
    std::vector<std::byte> payload;
    std::uint64_t index = 0;
    bool busy = false;
    std::future<AheadResult> job;
  };
  // The decode prefetch() started for tick `index`, if any: waits for it and
  // frees its slot.
  [[nodiscard]] std::optional<AheadResult> take_ahead(std::uint64_t index);
  [[nodiscard]] std::string select_follower(const TickInfo & tick, PanelSize cell);
  // Project every point-cloud topic of the panel onto the selected frame.
  [[nodiscard]] std::string project(std::vector<core::pointcloud::ProjectedPoint> & out) const;

  Options options_;
  ClockSizing sizing_;
  FrameNormalizer normalizer_;
  // Clock role: the decodes in flight ahead of their ticks. The loop calls
  // prefetch() from its own thread while a select() may run on a worker, so
  // the slot table is guarded; each job touches only its own slot.
  std::vector<std::unique_ptr<AheadSlot>> ahead_;
  std::mutex ahead_mutex_;
  ViewRenderer renderer_;
  std::unique_ptr<NearestMessageSource> source_;  // follower role only
  CloudSources * clouds_;
  // The latest decoded frame at its render size, with the geometry it was
  // prepared at. A follower keeps it across ticks while its topic repeats
  // the same message (`cached_record_ns_`).
  std::shared_ptr<const FrameBuffer> cache_;
  ViewRenderGeometry cache_geom_;
  std::int64_t cached_record_ns_ = -1;
  // What the current tick shows: the cache, or nothing (a black cell).
  bool selected_ = false;
  bool clock_sized_ = false;  // clock role: the --width pin was applied
  bool warned_empty_ = false;
};

// Build `movify`'s camera panels, one per validated view in grid order: the
// view at validation.clock as the clock panel, every other view as a follower
// over its own topic. Returns nullopt after logging when a follower's reader fails to open
// or a view's point-cloud topic is missing from the scan.
[[nodiscard]] std::optional<std::vector<std::unique_ptr<Panel>>> build_camera_panels(
  const MovifyArgs & args, const VideoInputValidation & validation, const VideoInputScan & scan,
  const VideoGeometry & geometry, CloudSources & clouds);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_CAMERA_PANEL_HPP_
