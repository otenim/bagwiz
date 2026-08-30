// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_camera_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

// A panel failure, attributed to the panel's topic for the loop's log line.
std::string on_topic(const std::string & topic, const std::string & error)
{
  return "topic '" + topic + "': " + error;
}
}  // namespace

std::optional<FrameBuffer> FrameNormalizer::decode(
  std::int64_t timestamp_ns, std::span<const std::byte> payload, std::string & error) const
{
  // Normalize either message type to a canonical packed BGR24 raster via the
  // shared core::image::to_packed_raster seam; rgb8 inputs are swapped so
  // every frame the encoder sees is BGR24. The member decoder carries the
  // codec context across frames instead of reopening it per frame.
  auto pr = core::image::to_packed_raster(topic_type_, payload, decoder_);
  if (!pr.ok()) {
    error = pr.error;
    return std::nullopt;
  }
  FrameBuffer frame;
  frame.timestamp_ns = timestamp_ns;
  frame.header_stamp_ns = pr.raster->header_stamp_ns;
  frame.width = pr.raster->width;
  frame.height = pr.raster->height;
  frame.step = pr.raster->width * 3U;
  frame.pixel_format = core::video::SourcePixelFormat::kBgr8;
  frame.encoding = "bgr8";
  frame.data = std::move(pr.raster->bgr);
  return frame;
}

std::string resize_frame(FrameBuffer & frame, std::uint32_t out_w, std::uint32_t out_h)
{
  if (frame.width == out_w && frame.height == out_h) {
    return "";
  }
  if (out_w == 0 || out_h == 0) {
    return fmt::format(
      "resize to {}x{} would produce a zero-size frame ({}x{})", out_w, out_h, frame.width,
      frame.height);
  }

  const cv::Mat in(
    static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3, frame.data.data(),
    frame.step);
  cv::Mat out;
  const std::uint64_t in_pixels = static_cast<std::uint64_t>(frame.width) * frame.height;
  const std::uint64_t out_pixels = static_cast<std::uint64_t>(out_w) * out_h;
  const int interpolation = (out_pixels < in_pixels) ? cv::INTER_AREA : cv::INTER_LINEAR;
  cv::resize(
    in, out, cv::Size{static_cast<int>(out_w), static_cast<int>(out_h)}, 0, 0, interpolation);

  frame.width = out_w;
  frame.height = out_h;
  frame.step = out_w * 3U;
  frame.data.assign(
    reinterpret_cast<std::byte *>(out.data),
    reinterpret_cast<std::byte *>(out.data) + out.total() * out.elemSize());
  return "";
}

std::unique_ptr<NearestMessageSource> NearestMessageSource::open(
  const std::filesystem::path & input, const std::string & topic, std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    error = "failed to open '" + input.string() + "': " + e.what();
    return nullptr;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);
  // NearestMessageSource's constructor is private (friends would couple it to
  // the factory's error string); the deleter-less new is wrapped immediately.
  return std::unique_ptr<NearestMessageSource>(new NearestMessageSource(std::move(reader)));
}

const NearestMessageSource::Message * NearestMessageSource::fetch(
  std::int64_t target_ns, std::string & error)
{
  error.clear();
  const auto read_next = [this]() -> std::optional<Message> {
    io::RawMessage raw;
    if (!reader_->next(raw)) {
      eof_ = true;
      return std::nullopt;
    }
    Message msg;
    msg.record_ns = raw.timestamp_ns;
    msg.payload.assign(raw.payload.begin(), raw.payload.end());
    return msg;
  };
  try {
    if (!after_.has_value() && !eof_) {
      after_ = read_next();
    }
    // Walk the boundary forward until after_ is the first message past the
    // target (or the stream ends); before_ is then the latest at or before it.
    while (after_.has_value() && after_->record_ns <= target_ns) {
      before_ = std::move(after_);
      after_ = eof_ ? std::nullopt : read_next();
    }
  } catch (const std::exception & e) {
    error = std::string("error reading topic: ") + e.what();
    return nullptr;
  }
  if (!before_.has_value() && !after_.has_value()) {
    return nullptr;  // the topic has no messages at all
  }
  if (!before_.has_value()) {
    return &*after_;
  }
  if (!after_.has_value()) {
    return &*before_;
  }
  const std::int64_t before_delta = target_ns - before_->record_ns;
  const std::int64_t after_delta = after_->record_ns - target_ns;
  return before_delta <= after_delta ? &*before_ : &*after_;
}

ViewRenderer::ViewRenderer(
  const core::image::CameraInfo * camera_info, bool rectify, const VideoOverlayParams & params,
  std::optional<double> fixed_scale)
: camera_info_(camera_info), rectify_(rectify), params_(params), fixed_scale_(fixed_scale)
{
}

std::optional<ViewRenderGeometry> ViewRenderer::prepare(
  std::uint32_t native_w, std::uint32_t native_h, std::uint32_t cell_w, std::uint32_t cell_h,
  std::string & error)
{
  const bool size_locked = fixed_scale_.has_value() || fixed_size_.has_value();
  if (size_locked) {
    // The clock panel's native size is locked to its first frame, exactly as
    // the single-view encoder locked its geometry.
    if (native_w_ != 0 && (native_w != native_w_ || native_h != native_h_)) {
      error = fmt::format(
        "frame changed to {}x{} from the first frame's {}x{}; aborting.", native_w, native_h,
        native_w_, native_h_);
      return std::nullopt;
    }
  }
  double scale = 1.0;
  std::uint32_t render_w = 0;
  std::uint32_t render_h = 0;
  if (fixed_size_.has_value()) {
    // --width: exact target dims derived from the output width.
    render_w = fixed_size_->first;
    render_h = fixed_size_->second;
    scale = static_cast<double>(render_w) / native_w;
  } else if (fixed_scale_.has_value()) {
    scale = *fixed_scale_;
    render_w = static_cast<std::uint32_t>(std::lround(native_w * scale));
    render_h = static_cast<std::uint32_t>(std::lround(native_h * scale));
  } else {
    scale =
      std::min(static_cast<double>(cell_w) / native_w, static_cast<double>(cell_h) / native_h);
    render_w = static_cast<std::uint32_t>(std::lround(native_w * scale));
    render_h = static_cast<std::uint32_t>(std::lround(native_h * scale));
  }
  if (render_w == 0 || render_h == 0) {
    error = fmt::format(
      "scale {:.3g} would produce a zero-size frame ({}x{})", scale, render_w, render_h);
    return std::nullopt;
  }
  native_w_ = native_w;
  native_h_ = native_h;

  ViewRenderGeometry geom;
  geom.width = render_w;
  geom.height = render_h;
  geom.rectify = rectify_;
  if (camera_info_ != nullptr) {
    // The same two-step chain the single-view path applied: pre-scale by the
    // panel's scale, then match the render size (a verbatim copy when the two
    // already agree, which they do by construction). A --width-pinned size
    // pre-scales per axis, since its height is rounded independently.
    if (fixed_size_.has_value()) {
      geom.camera_info = core::image::camera_info_for_size(
        core::image::scale_camera_info(
          *camera_info_, static_cast<double>(render_w) / native_w,
          static_cast<double>(render_h) / native_h),
        render_w, render_h);
    } else {
      geom.camera_info = core::image::camera_info_for_size(
        core::image::scale_camera_info(*camera_info_, scale), render_w, render_h);
    }
    geom.has_camera_info = true;
  }
  return geom;
}

std::string ViewRenderer::render(
  const FrameBuffer & frame, const ViewRenderGeometry & geom,
  const std::vector<core::pointcloud::ProjectedPoint> * projected, const CellView & cell)
{
  std::span<const std::byte> fdata{frame.data.data(), frame.data.size()};

  if (geom.rectify) {
    if (rectify_helper_ == nullptr || helper_w_ != geom.width || helper_h_ != geom.height) {
      rectify_helper_ =
        std::make_unique<core::image::RectifyHelper>(geom.camera_info, geom.width, geom.height);
      helper_w_ = geom.width;
      helper_h_ = geom.height;
    }
    fdata = rectify_helper_->remap(fdata, frame.step);
  }

  core::image::PackedRaster overlay_output;
  if (projected != nullptr) {
    core::image::PackedRaster src;
    src.width = geom.width;
    src.height = geom.height;
    src.encoding = frame.encoding;
    src.bgr.assign(fdata.begin(), fdata.end());

    overlay_output.width = geom.width;
    overlay_output.height = geom.height;
    overlay_output.encoding = frame.encoding;
    if (const auto err = core::pointcloud::overlay_projected_points(
          src, *projected, params_.property_min, params_.property_max, params_.colorscheme,
          params_.point_size, params_.alpha, overlay_output);
        !err.empty()) {
      return "overlay failed: " + err;
    }
    fdata = std::span<const std::byte>(overlay_output.bgr.data(), overlay_output.bgr.size());
  }

  // Paste centered; the canvas was cleared to black, so the letterbox bars
  // are already in place.
  const std::uint32_t x_off = (cell.width - geom.width) / 2U;
  const std::uint32_t y_off = (cell.height - geom.height) / 2U;
  const std::size_t row_bytes = static_cast<std::size_t>(geom.width) * 3U;
  for (std::uint32_t y = 0; y < geom.height; ++y) {
    std::copy_n(
      fdata.data() + static_cast<std::size_t>(y) * row_bytes, row_bytes,
      cell.data + static_cast<std::size_t>(y_off + y) * cell.stride +
        static_cast<std::size_t>(x_off) * 3U);
  }
  return "";
}

CameraPanel::CameraPanel(Options options, ClockSizing sizing, CloudSources * clouds)
: options_(std::move(options)),
  sizing_(sizing),
  normalizer_(options_.topic_type),
  renderer_(options_.camera_info, options_.rectify, options_.overlay, sizing.resize_scale),
  clouds_(clouds)
{
}

CameraPanel::CameraPanel(
  Options options, std::unique_ptr<NearestMessageSource> source, CloudSources * clouds)
: options_(std::move(options)),
  normalizer_(options_.topic_type),
  renderer_(options_.camera_info, options_.rectify, options_.overlay, std::nullopt),
  source_(std::move(source)),
  clouds_(clouds)
{
}

std::string CameraPanel::select(const TickInfo & tick, PanelSize cell)
{
  return source_ == nullptr ? select_clock(tick) : select_follower(tick, cell);
}

void CameraPanel::prefetch(const TickInfo & tick)
{
  if (source_ != nullptr) {
    return;  // a follower decodes what it fetches itself
  }
  AheadSlot * slot = nullptr;
  {
    const std::lock_guard<std::mutex> lock(ahead_mutex_);
    if (ahead_.empty()) {
      for (std::size_t i = 0; i < kDecodeAheadDepth; ++i) {
        ahead_.push_back(std::make_unique<AheadSlot>(options_.topic_type));
      }
    }
    for (auto & candidate : ahead_) {
      if (!candidate->busy) {
        slot = candidate.get();
        break;
      }
    }
    if (slot == nullptr) {
      return;  // every decoder is taken: the tick decodes inline
    }
    slot->busy = true;
    slot->index = tick.index;
    slot->payload.assign(tick.payload.begin(), tick.payload.end());
  }
  const std::int64_t record_ns = tick.record_ns;
  slot->job = std::async(std::launch::async, [slot, record_ns] {
    AheadResult result;
    result.frame = slot->normalizer.decode(record_ns, slot->payload, result.error);
    return result;
  });
}

std::optional<CameraPanel::AheadResult> CameraPanel::take_ahead(std::uint64_t index)
{
  AheadSlot * slot = nullptr;
  std::future<AheadResult> job;
  {
    const std::lock_guard<std::mutex> lock(ahead_mutex_);
    for (auto & candidate : ahead_) {
      if (candidate->busy && candidate->index == index) {
        slot = candidate.get();
        job = std::move(candidate->job);
        break;
      }
    }
  }
  if (slot == nullptr) {
    return std::nullopt;
  }
  AheadResult result = job.get();  // outside the lock: prefetch() must not wait on a decode
  const std::lock_guard<std::mutex> lock(ahead_mutex_);
  slot->busy = false;
  return result;
}

std::string CameraPanel::select_clock(const TickInfo & tick)
{
  selected_ = false;
  std::string error;
  std::optional<FrameBuffer> decoded;
  if (auto ahead = take_ahead(tick.index); ahead.has_value()) {
    decoded = std::move(ahead->frame);
    error = std::move(ahead->error);
  } else {
    decoded = normalizer_.decode(tick.record_ns, tick.payload, error);
  }
  if (!decoded.has_value()) {
    return on_topic(options_.topic, error);
  }
  if (!clock_sized_ && sizing_.total_width.has_value()) {
    // --width: pin the render size from the output width — the cell width is
    // the width split across the grid columns, the height follows the frame's
    // aspect ratio. Both are rounded down to even (the codecs' 4:2:0 formats
    // require even dimensions), so the output can be a few pixels narrower
    // than requested. Validation guarantees the cell width is at least 2.
    const std::uint32_t cell_w = (*sizing_.total_width / sizing_.grid_cols) & ~1U;
    const auto cell_h = static_cast<std::uint32_t>(std::lround(
                          decoded->height * (static_cast<double>(cell_w) / decoded->width))) &
                        ~1U;
    renderer_.set_fixed_render_size(cell_w, cell_h);
  }
  clock_sized_ = true;
  // The clock's scale or pinned size never reads the cell, so the zero cell
  // dimensions of the first tick are harmless.
  auto prepared = renderer_.prepare(decoded->width, decoded->height, 0, 0, error);
  if (!prepared.has_value()) {
    return on_topic(options_.topic, error);
  }
  if (const auto err = resize_frame(*decoded, prepared->width, prepared->height); !err.empty()) {
    return on_topic(options_.topic, err);
  }
  cache_geom_ = *prepared;
  cache_ = std::make_shared<const FrameBuffer>(std::move(*decoded));
  selected_ = true;
  return "";
}

std::string CameraPanel::select_follower(const TickInfo & tick, PanelSize cell)
{
  selected_ = false;
  std::string error;
  const auto * msg = source_->fetch(tick.record_ns, error);
  if (!error.empty()) {
    return error;
  }
  if (msg == nullptr) {
    // The topic has no messages at all: the cell stays black.
    if (!warned_empty_) {
      BAGWIZ_LOG_WARN(
        kLogger, "topic '%s' has no messages; its cell stays black.", options_.topic.c_str());
      warned_empty_ = true;
    }
    return "";
  }
  if (msg->record_ns != cached_record_ns_) {
    auto decoded = normalizer_.decode(msg->record_ns, msg->payload, error);
    if (!decoded.has_value()) {
      return on_topic(options_.topic, error);
    }
    auto prepared =
      renderer_.prepare(decoded->width, decoded->height, cell.width, cell.height, error);
    if (!prepared.has_value()) {
      return on_topic(options_.topic, error);
    }
    if (const auto err = resize_frame(*decoded, prepared->width, prepared->height); !err.empty()) {
      return on_topic(options_.topic, err);
    }
    cache_geom_ = *prepared;
    cache_ = std::make_shared<const FrameBuffer>(std::move(*decoded));
    cached_record_ns_ = msg->record_ns;
  }
  selected_ = true;
  return "";
}

std::optional<PanelSize> CameraPanel::clock_cell_size() const
{
  if (source_ != nullptr || !selected_) {
    return std::nullopt;
  }
  return PanelSize{cache_geom_.width, cache_geom_.height};
}

std::string CameraPanel::project(std::vector<core::pointcloud::ProjectedPoint> & out) const
{
  for (const auto idx : options_.cloud_indexes) {
    // The matched time doubles as the TF-lookup time, which keeps the
    // transform lookup on the same clock the TF messages are stamped with.
    const auto match = core::pointcloud::choose_frame_match(
      cache_->header_stamp_ns, cache_->timestamp_ns, clouds_->has_header_stamps(idx));
    std::string error;
    const auto cloud = clouds_->fetch(idx, match.target_ns, match.key, error);
    if (!cloud) {
      return error;
    }
    const auto projected = core::pointcloud::project_cloud_for_frame(
      *cloud, cache_geom_.camera_info, *clouds_->tf_buffer(), cache_geom_.width, cache_geom_.height,
      options_.property, cache_geom_.rectify, match.target_ns);
    if (!projected.ok()) {
      return projected.error;
    }
    out.insert(out.end(), projected.points.begin(), projected.points.end());
  }
  return "";
}

std::string CameraPanel::render(const CellView & cell)
{
  if (!selected_) {
    return "";  // nothing to show: the cell was cleared to black
  }
  std::vector<core::pointcloud::ProjectedPoint> points;
  const std::vector<core::pointcloud::ProjectedPoint> * points_ptr = nullptr;
  if (!options_.cloud_indexes.empty()) {
    if (const auto err = project(points); !err.empty()) {
      return err;
    }
    points_ptr = &points;
  }
  if (const auto err = renderer_.render(*cache_, cache_geom_, points_ptr, cell); !err.empty()) {
    return on_topic(options_.topic, err);
  }
  return "";
}

std::optional<std::vector<std::unique_ptr<Panel>>> build_camera_panels(
  const MovifyArgs & args, const VideoInputValidation & validation, const VideoInputScan & scan,
  const VideoGeometry & geometry, CloudSources & clouds)
{
  VideoOverlayParams overlay;
  overlay.property_min = scan.global_property_min;
  overlay.property_max = scan.global_property_max;
  overlay.colorscheme = args.colorscheme;
  overlay.point_size = args.point_size;
  overlay.alpha = args.alpha;

  std::vector<std::unique_ptr<Panel>> panels;
  panels.reserve(validation.views.size());
  for (std::size_t i = 0; i < validation.views.size(); ++i) {
    const ViewInput & view = validation.views[i];
    CameraPanel::Options options;
    options.topic = view.topic;
    options.topic_type = view.topic_type;
    options.camera_info = (i < geometry.camera_infos.size() && geometry.camera_infos[i].has_value())
                            ? &*geometry.camera_infos[i]
                            : nullptr;
    // --no-rectify wins even with --cam-pcd; the projection then targets the raw,
    // distortion-aware path (see view_rectifies).
    options.rectify = view_rectifies(args.rectify, view);
    options.overlay = overlay;
    options.property = args.property;
    for (const auto & topic : view.pcd_topics) {
      const auto it = std::find(scan.pcd_topics.begin(), scan.pcd_topics.end(), topic);
      if (it == scan.pcd_topics.end()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "internal error — pcd topic '%s' missing from the scan", topic.c_str());
        return std::nullopt;
      }
      options.cloud_indexes.push_back(static_cast<std::size_t>(it - scan.pcd_topics.begin()));
    }
    if (i == validation.clock) {
      CameraPanel::ClockSizing sizing;
      sizing.resize_scale = args.resize_scale;
      sizing.total_width = args.width;
      sizing.grid_cols = validation.grid.cols;
      panels.push_back(std::make_unique<CameraPanel>(std::move(options), sizing, &clouds));
    } else {
      std::string error;
      auto source = NearestMessageSource::open(args.input_path, view.topic, error);
      if (!source) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
        return std::nullopt;
      }
      panels.push_back(
        std::make_unique<CameraPanel>(std::move(options), std::move(source), &clouds));
    }
  }
  return panels;
}

}  // namespace bagwiz::commands
