// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_VIDEO_COMMON_HPP_
#define COMMANDS__MOVIFY_VIDEO_COMMON_HPP_

#include "bagwiz/commands/movify_video.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/rectify.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Internals of `movify cam`, split out of movify_video.cpp so the
// validation, pass-1 scan, tmp-file lifecycle, and frame pipeline units can be
// unit-tested without driving the full command. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// ---- grid layout + per-view bindings -------------------------------------------

// Grid dimensions in cells. Views fill the grid left to right, top to bottom;
// cells past the last view stay black.
struct GridSpec
{
  std::uint32_t cols = 0;
  std::uint32_t rows = 0;
};

// The near-square default layout for a view count: 2 views -> 2x1, 3-4 -> 2x2,
// 5-6 -> 3x2, and so on.
[[nodiscard]] GridSpec auto_grid_spec(std::size_t view_count);

struct GridParseResult
{
  GridSpec grid;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Parse the --grid value "<cols>x<rows>". Empty `text` selects
// auto_grid_spec(view_count). Errors: malformed text, a zero dimension, fewer
// cells than views.
[[nodiscard]] GridParseResult parse_grid_spec(const std::string & text, std::size_t view_count);

// --pcd entries after topic expansion: a bare value (already glob-expanded by
// the CLI layer) projects onto every view; an "<image_topic>=<pcd_topic>"
// entry projects onto that view only.
struct PcdBindings
{
  std::vector<std::string> global_topics;
  std::unordered_map<std::string, std::vector<std::string>> per_view;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Split --pcd entries into global topics and per-view bindings. Errors: an
// empty half, or an <image_topic> that is not one of `image_topics`.
[[nodiscard]] PcdBindings parse_pcd_bindings(
  std::span<const std::string> entries, std::span<const std::string> image_topics);

// --cam-info entries: a bare value applies to every view, an
// "<image_topic>=<info_topic>" entry overrides one view.
struct CamInfoEntries
{
  std::optional<std::string> global_topic;
  std::unordered_map<std::string, std::string> per_view;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Split --cam-info entries into the global topic and per-view overrides.
// Errors: an empty half, an <image_topic> outside `image_topics`, a duplicate
// override for one view, or more than one bare (global) value.
[[nodiscard]] CamInfoEntries parse_cam_info_entries(
  std::span<const std::string> entries, std::span<const std::string> image_topics);

// ---- input validation -------------------------------------------------------

// One view's resolved inputs: the image topic and its message type, the
// camera-info topic when the view needs one (nullopt when the run needs none
// for this view or none could be derived), and the point-cloud topics
// projected onto this view (global topics first, then the view's own
// bindings, duplicates removed).
struct ViewInput
{
  std::string topic;
  std::string topic_type;
  std::optional<std::string> camera_info_topic;
  std::vector<std::string> pcd_topics;
};

// Outcome of validate_video_inputs(). `views` is parallel to args.topics and
// `grid` is the resolved layout. `error` is empty on success; on failure it
// holds the message that was already logged.
struct VideoInputValidation
{
  std::vector<ViewInput> views;
  GridSpec grid;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// The command's pre-flight checks: grid parse, duplicate/empty topic
// rejection, every image topic's presence + renderable type, --pcd and
// --cam-info entry parsing, per-view cam-info resolution (explicit entry or
// derivation from the image topic name) and the cam-info requirement of
// rectification / --pcd, and every point-cloud topic's presence + type. Logs
// the command's errors and returns on the first failure.
[[nodiscard]] VideoInputValidation validate_video_inputs(const MovifyVideoArgs & args);

// Whether a validated view renders rectified: rectification must be in effect
// (the default, unless --no-rectify) and the view's camera info must have
// resolved. --no-rectify wins even with --pcd — the projection then targets
// the raw image, applying the camera's lens distortion model instead of
// assuming a rectified one.
[[nodiscard]] bool view_rectifies(bool rectify_requested, const ViewInput & view) noexcept;

// Fail-fast output checks run before the expensive encode: an existing
// `output_path` without --overwrite stops the run, and the output's parent
// directory is created when missing. Returns "" on success; on failure logs
// and returns the message.
[[nodiscard]] std::string validate_video_output_path(
  const std::filesystem::path & output_path, bool overwrite);

// ---- pass 1: frame-rate + point-cloud scan -----------------------------------

// Timestamps + count for a single topic, gathered by a payload-free scan.
struct TopicSpan
{
  std::int64_t first_ns = 0;
  std::int64_t last_ns = 0;
  std::uint64_t count = 0;
};

// Outcome of scan_video_inputs(). pcd_topics is the deduplicated union of
// every view's point-cloud topics in first-use order; pcd_spans and
// pcd_topic_has_stamps are parallel to it, and the pcd_spans entries are
// owned here (the encode loops move them out). `error` is empty on success;
// on failure it holds the message that was already logged.
struct VideoInputScan
{
  // The primary (first) view's message span, which drives the frame rate.
  TopicSpan span;
  core::video::FrameRate fps;
  std::vector<std::string> pcd_topics;
  std::vector<core::pointcloud::PointCloudIndex> pcd_spans;
  // Per pcd topic: whether it can be matched by capture time (every cloud
  // carried a header.stamp). Topics that fall back to record time are matched
  // by record time on both sides so the overlay stays in one clock.
  std::vector<bool> pcd_topic_has_stamps;
  double global_property_min = 0.0;
  double global_property_max = 0.0;
  std::string error;

  [[nodiscard]] bool ok() const { return error.empty(); }
};

// Pass 1: derive the frame rate from the primary topic's message timestamps,
// require every view's topic to carry at least one message, and, when
// point-cloud overlay topics are given, scan each for its index and the
// selected property's global min/max. Logs the command's errors and returns
// with !ok() on the first failure.
[[nodiscard]] VideoInputScan scan_video_inputs(
  const MovifyVideoArgs & args, const VideoInputValidation & validation);

// The parallel per-view pipeline is only worthwhile when there is work to
// spread across workers (several views, or point-cloud projection) and enough
// frames to hide the per-tick job-launch overhead.
[[nodiscard]] bool should_use_parallel_pipeline(
  std::size_t view_count, bool has_pointcloud_topics, bool enable_parallel,
  std::uint64_t frame_count, unsigned int hardware_concurrency);

// ---- pass-2 geometry ---------------------------------------------------------

// The camera infos (one per view, UNSCALED — each view's renderer applies its
// own scale) and TF buffer the encode loop needs for rectification / --pcd,
// loaded up front so a failure aborts before the encode. camera_infos[i] is
// set iff view i resolved a camera-info topic; the TF buffer iff any view
// projects point clouds. Filled via an out parameter because tf2::BufferCore
// is immobile (it owns a mutex), so this struct cannot be returned by value.
struct VideoGeometry
{
  std::vector<std::optional<core::image::CameraInfo>> camera_infos;
  std::optional<tf2::BufferCore> tf_buffer;
};

// Load the pass-2 geometry into `out`: camera info from each view's resolved
// topic, and the bag's TF when point-cloud overlay topics are present.
// Returns "" on success; on failure logs and returns the message.
[[nodiscard]] std::string load_video_geometry(
  const MovifyVideoArgs & args, const VideoInputValidation & validation, VideoGeometry & out);

// ---- partial tmp output -------------------------------------------------------

// The sibling temp path the video is encoded into before being moved into
// place, e.g. out.avi -> out.bagwiz-partial.avi. The real extension is kept on
// the temp file: both the encoder's codec choice and the libav muxer are
// selected from the extension, so a bare ".bagwiz-partial" suffix would be
// rejected.
[[nodiscard]] std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output);

// RAII owner of the partial tmp output's lifecycle: construction clears any
// stale tmp left by a previous aborted run; destruction removes the tmp when
// it still exists (a no-op once finalize_video_output renamed it away), so no
// error path can leave a partial file behind. Declare it BEFORE the encoder so
// the encoder is destroyed — closing the file — before the tmp is removed.
class PartialFileGuard
{
public:
  explicit PartialFileGuard(std::filesystem::path tmp_path);
  ~PartialFileGuard();
  PartialFileGuard(const PartialFileGuard &) = delete;
  PartialFileGuard & operator=(const PartialFileGuard &) = delete;
  PartialFileGuard(PartialFileGuard &&) = delete;
  PartialFileGuard & operator=(PartialFileGuard &&) = delete;

  [[nodiscard]] const std::filesystem::path & path() const noexcept { return tmp_path_; }

private:
  std::filesystem::path tmp_path_;
};

// Move the finished tmp video into place: apply the --overwrite clobber policy
// via core::prepare_output_path, then rename, falling back to copy + remove
// across filesystems. Returns "" on success; on failure logs and returns the
// message (the caller's PartialFileGuard removes the tmp).
[[nodiscard]] std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path,
  bool overwrite);

// ---- pass 2: frame pipeline ---------------------------------------------------

// Open the input bag for the encode pass, restricted to the primary image
// topic (secondary views read through their own NearestMessageSource). Logs
// "failed to open ..." and returns nullptr on failure.
[[nodiscard]] std::unique_ptr<io::BagReader> open_encode_reader(const MovifyVideoArgs & args);

// Owned decode buffer that survives across BagReader::next() calls, which
// invalidate raw payload spans. Used by every view's frame cache.
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
// or CompressedImage) to a canonical packed BGR24 raster. Errors are logged
// with the count of frames written so far. Resizing is a separate step
// (resize_frame) because each view picks its own scale.
class FrameNormalizer
{
public:
  explicit FrameNormalizer(std::string topic_type) : topic_type_(std::move(topic_type)) {}

  // Decode one message payload into an owned canonical BGR24 frame.
  // `frame_index` is the count of frames written so far (used in the log line
  // on failure). Returns nullopt after logging when the payload does not
  // decode.
  [[nodiscard]] std::optional<FrameBuffer> decode(
    std::int64_t timestamp_ns, std::span<const std::byte> payload, std::uint64_t frame_index) const;

private:
  std::string topic_type_;
  // libav decode handles kept across frames so the codec context and its
  // buffers are not reopened per frame. mutable because decode() is logically
  // const; each view owns its FrameNormalizer, so there is no cross-thread
  // sharing.
  mutable core::image::ImageDecoder decoder_;
};

// Resize a decoded frame in place to exactly out_w x out_h (INTER_AREA when
// shrinking, INTER_LINEAR when growing). A no-op when the frame already has
// that size. Returns false and logs when a target dimension is zero.
[[nodiscard]] bool resize_frame(FrameBuffer & frame, std::uint32_t out_w, std::uint32_t out_h);

// Streaming reader over ONE secondary image topic, answering "the message
// whose bag record time is nearest t" exactly, via one-message lookahead.
// Targets must be non-decreasing across fetch() calls (the encode loop's
// primary-driven ticks guarantee this).
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

// A writable view of one grid cell inside the composed output frame.
struct CellView
{
  std::byte * data = nullptr;  // the cell's top-left pixel
  std::uint32_t width = 0;     // cell dimensions
  std::uint32_t height = 0;
  std::size_t stride = 0;  // the composed frame's row stride in bytes
};

// The composed multi-view output frame: a fixed grid of equally sized cells,
// row-major. The cell size is fixed by the first primary frame; the composed
// size (cols*cell_w x rows*cell_h) then never changes, which is what the video
// encoder's fixed-geometry requirement needs.
class GridCanvas
{
public:
  explicit GridCanvas(GridSpec grid) : grid_(grid) {}

  // Fix the cell size and allocate the composed buffer. Called once, on the
  // first primary frame.
  void set_cell_size(std::uint32_t width, std::uint32_t height);

  [[nodiscard]] bool ready() const { return !pixels_.empty(); }
  [[nodiscard]] std::uint32_t cell_width() const { return cell_w_; }
  [[nodiscard]] std::uint32_t cell_height() const { return cell_h_; }
  [[nodiscard]] std::uint32_t width() const { return grid_.cols * cell_w_; }
  [[nodiscard]] std::uint32_t height() const { return grid_.rows * cell_h_; }

  // Black out the whole canvas for a new output frame.
  void clear();

  // Writable view of cell `index` (row-major; must be < cols*rows).
  [[nodiscard]] CellView cell(std::size_t index);

  [[nodiscard]] const std::vector<std::byte> & pixels() const { return pixels_; }

private:
  GridSpec grid_;
  std::uint32_t cell_w_ = 0;
  std::uint32_t cell_h_ = 0;
  std::vector<std::byte> pixels_;
};

// Point-cloud overlay styling, shared by every view.
struct VideoOverlayParams
{
  double property_min = 0.0;
  double property_max = 0.0;
  core::pointcloud::ColorScheme colorscheme = core::pointcloud::ColorScheme::kViridis;
  std::uint32_t point_size = 2;
  float alpha = 1.0f;
};

// The render-time geometry of one view for one tick: the render size (the
// frame scaled by the view's scale) and the CameraInfo effective at that
// size. Snapshotted at prepare() time so a tick's projection and rendering
// stay consistent even when a later tick's prepare() re-fits the view.
struct ViewRenderGeometry
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  core::image::CameraInfo camera_info;  // effective at (width, height)
  bool has_camera_info = false;
  bool rectify = false;
};

// One view's render pipeline: fixes the view's scale from its first frame,
// scales/rectifies/overlays each selected frame, and pastes it centered into
// the view's grid cell (with black bars when the aspect ratios differ).
class ViewRenderer
{
public:
  // `fixed_scale` views (the primary) render every frame at
  // lround(w*scale) x lround(h*scale) and reject a native-size change; fit
  // views (secondaries) pick a uniform scale at their first frame that fits
  // the cell preserving aspect ratio, re-fitting if the native size changes.
  // `camera_info` is the view's UNSCALED calibration, non-null iff the view
  // rectifies or projects point clouds.
  ViewRenderer(
    const core::image::CameraInfo * camera_info, bool rectify, const VideoOverlayParams & params,
    std::optional<double> fixed_scale);

  // Fix the scale on first use and compute the render geometry for a native
  // w x h frame. Returns nullopt after logging when the result would be
  // zero-size or (fixed-scale views) the native size changed mid-run.
  [[nodiscard]] std::optional<ViewRenderGeometry> prepare(
    std::uint32_t native_w, std::uint32_t native_h, std::uint32_t cell_w, std::uint32_t cell_h);

  // Pin the render size to exact pixel dimensions, derived from the --width
  // output-width constraint rather than a scale factor. Must be called before
  // the first prepare(); the fixed-view native-size lock still applies.
  void set_fixed_render_size(std::uint32_t width, std::uint32_t height)
  {
    fixed_size_ = {width, height};
  }

  // Render `frame` (already at geom's size — the encode loop resizes right
  // after decode) into `cell`: rectify, overlay `projected` when non-null,
  // and paste centered. Returns false after logging on failure.
  [[nodiscard]] bool render(
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

// Encode half of the frame pipeline: owns the video encoder (opened lazily on
// the first composed frame, which fixes the run's geometry). All failures are
// logged and reported as false / a non-empty string.
class VideoFrameEncoder
{
public:
  VideoFrameEncoder(const std::filesystem::path & tmp_path, core::video::FrameRate fps);

  // Encode one composed packed-BGR24 frame (row stride width*3). Returns
  // false after logging on any failure, including a mid-run size change.
  [[nodiscard]] bool encode(
    std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height);

  // Flush and close the stream. Returns "" on success; on failure logs and
  // returns the message. Either way the encoder is closed afterwards (the tmp
  // file can be renamed or removed).
  [[nodiscard]] std::string finish();

  // True once the first frame opened the encoder. A finished run still reports
  // its geometry and frame count for the summary line.
  [[nodiscard]] bool started() const { return encoder_ != nullptr; }
  [[nodiscard]] std::uint64_t written() const { return written_; }
  [[nodiscard]] std::uint32_t width() const { return enc_w_; }
  [[nodiscard]] std::uint32_t height() const { return enc_h_; }

private:
  std::filesystem::path tmp_path_;
  core::video::FrameRate fps_;

  std::unique_ptr<core::video::VideoEncoder> encoder_;
  std::uint32_t enc_w_ = 0;
  std::uint32_t enc_h_ = 0;
  std::uint64_t written_ = 0;
};

// Dispatch the encode pass: the parallel per-view pipeline when it can pay
// for itself (should_use_parallel_pipeline), otherwise the synchronous loop.
// Per tick (one primary message) each view selects its frame — the primary
// view the message itself, a secondary the one nearest by bag record time —
// projects its point clouds, renders its cell, and the composed grid is
// encoded. Moves the pcd index entries out of `scan`. Returns a process exit
// code; errors are logged inside.
[[nodiscard]] int run_encode_pass(
  io::BagReader & reader, const MovifyVideoArgs & args, const VideoInputValidation & validation,
  VideoInputScan & scan, VideoGeometry & geometry, VideoFrameEncoder & encoder);

// Close out the encode: require at least one rendered frame (pass 1 saw
// messages, so a frameless pass 2 means the bag changed between passes), flush
// + close the encoder, and move the tmp output into place. Returns "" on
// success; logs and returns the message on failure.
[[nodiscard]] std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite);

// ---- summary ------------------------------------------------------------------

// The end-of-run INFO line plus the H.264 playback guidance (with the VLC
// install hint when no vlc executable is on the host).
void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_VIDEO_COMMON_HPP_
