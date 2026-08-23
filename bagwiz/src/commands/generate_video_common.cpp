// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/io/topics.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <charconv>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
// kImageType / kCompressedImageType mirror topic_types.hpp's kImageTopicTypes
// (generate video cam -t's allowed_types) via is_supported_type() below.
// kPointCloudType mirrors topic_types.hpp's kPointCloud2Type (--pcd's
// allowed_types). Keep both in sync by hand.
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

// Below this many frames the threaded projection pipeline cannot hide the
// overhead of launching a thread and opening a fresh BagReader per frame.
constexpr std::uint64_t kThreadingMinFrames = 4;

bool is_supported_type(const std::string & type)
{
  return type == kImageType || type == kCompressedImageType;
}

// Returns true for extensions that the encoder maps to H.264. Used only for
// user-facing playback guidance.
bool is_h264_extension(const std::filesystem::path & output)
{
  std::string ext = output.extension().string();
  for (auto & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext == ".mp4" || ext == ".mkv" || ext == ".mov";
}

// Best-effort check for a vlc executable on the host. Used only to decide
// whether to append an install hint to the H.264 playback guidance.
bool is_vlc_available()
{
#ifdef _WIN32
  return std::system("where vlc >nul 2>nul") == 0;
#else
  return std::system("command -v vlc >/dev/null 2>&1") == 0;
#endif
}

// Platform-specific one-line hint for installing VLC.
const char * vlc_install_hint()
{
#ifdef __linux__
  return "Install VLC with your package manager (e.g. 'sudo apt install vlc').";
#elif __APPLE__
  return "Install VLC with: brew install vlc";
#elif _WIN32
  return "Install VLC from https://www.videolan.org/vlc/";
#else
  return "Install VLC from https://www.videolan.org/vlc/";
#endif
}

// Pass 1: stream the topic's messages reading only their timestamps (no
// payload decode) to learn the count and time span for the frame-rate estimate.
int scan_topic_span(const std::filesystem::path & input, const std::string & topic, TopicSpan & out)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open '%s': %s", input.string().c_str(), e.what());
    return 1;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (out.count == 0) {
        out.first_ns = raw.timestamp_ns;
      }
      out.last_ns = raw.timestamp_ns;
      ++out.count;
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "error reading topic '%s': %s", topic.c_str(), e.what());
    return 1;
  }
  return 0;
}

// Pass 1: scan the point-cloud topic, record every timestamp, and compute the
// global min/max of the selected property unless the user supplied --min/--max.
int scan_pointcloud_span(
  const std::filesystem::path & input, const std::string & topic,
  core::pointcloud::PointCloudProperty property, const std::optional<double> & manual_min,
  const std::optional<double> & manual_max, core::pointcloud::PointCloudIndex & out)
{
  std::string error;
  auto idx = core::pointcloud::build_point_cloud_index(
    input, topic, property, manual_min, manual_max, error);
  if (!idx.has_value()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
    return 1;
  }
  out = std::move(*idx);
  return 0;
}

// One view's runtime state in the encode loops: its decoder, renderer, frame
// source (secondaries only), decoded-frame cache (resized to the render size,
// with the geometry it was prepared at), and the indexes of its point-cloud
// topics within VideoInputScan's unique topic list.
struct ViewState
{
  const ViewInput * input = nullptr;
  FrameNormalizer normalizer;
  ViewRenderer renderer;
  std::unique_ptr<NearestMessageSource> source;  // secondaries only
  std::shared_ptr<const FrameBuffer> cache;      // secondary: latest selected frame
  ViewRenderGeometry cache_geom;                 // the geometry `cache` was prepared at
  std::int64_t cached_record_ns = -1;
  std::vector<std::size_t> pcd_indexes;
  // Primary only: the --width output-width constraint and the grid's column
  // count, combined into the pinned cell size on the first tick.
  std::optional<std::uint32_t> total_width;
  std::uint32_t grid_cols = 0;
  bool warned_empty = false;
};

// The frames selected for one output tick, parallel to the view list, with
// each view's render geometry snapshot. A null frame renders as a black cell
// (a secondary whose topic has not yielded a message yet).
struct TickData
{
  std::vector<std::shared_ptr<const FrameBuffer>> frames;
  std::vector<ViewRenderGeometry> geometries;
};

// Build the per-view runtime state shared by both encode loops. Returns
// nullopt after logging when a secondary source fails to open.
std::optional<std::vector<ViewState>> build_view_states(
  const GenerateVideoArgs & args, const VideoInputValidation & validation,
  const VideoInputScan & scan, VideoGeometry & geometry)
{
  VideoOverlayParams params;
  params.property_min = scan.global_property_min;
  params.property_max = scan.global_property_max;
  params.colorscheme = args.colorscheme;
  params.point_size = args.point_size;
  params.alpha = args.alpha;

  std::vector<ViewState> states;
  states.reserve(validation.views.size());
  for (std::size_t i = 0; i < validation.views.size(); ++i) {
    const ViewInput & view = validation.views[i];
    const core::image::CameraInfo * camera_info =
      (i < geometry.camera_infos.size() && geometry.camera_infos[i].has_value())
        ? &*geometry.camera_infos[i]
        : nullptr;
    // A view rectifies when asked to, or when it projects point clouds (the
    // projection assumes a rectified image) — the single-view rule, per view.
    const bool rectify = args.rectify || !view.pcd_topics.empty();
    ViewState state{
      .input = &view,
      .normalizer = FrameNormalizer(view.topic_type),
      .renderer = ViewRenderer(
        camera_info, rectify, params,
        i == 0 ? std::optional<double>{args.resize_scale} : std::nullopt),
    };
    for (const auto & topic : view.pcd_topics) {
      const auto it = std::find(scan.pcd_topics.begin(), scan.pcd_topics.end(), topic);
      if (it == scan.pcd_topics.end()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "internal error — pcd topic '%s' missing from the scan", topic.c_str());
        return std::nullopt;
      }
      state.pcd_indexes.push_back(static_cast<std::size_t>(it - scan.pcd_topics.begin()));
    }
    if (i == 0) {
      state.total_width = args.width;
      state.grid_cols = validation.grid.cols;
    } else {
      std::string error;
      state.source = NearestMessageSource::open(args.input_path, view.topic, error);
      if (!state.source) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
        return std::nullopt;
      }
    }
    states.push_back(std::move(state));
  }
  return states;
}

// Select every view's frame for one output tick: decode the primary message,
// fix the canvas cell size on the first tick, and match each secondary to the
// message nearest the primary's bag record time. Selected frames are resized
// to their render size here, so render_tick() never rescales. Returns false
// on a logged failure.
bool prepare_tick(
  const io::RawMessage & raw, std::vector<ViewState> & states, GridCanvas & canvas,
  std::uint64_t frame_index, TickData & out)
{
  out.frames.assign(states.size(), nullptr);
  out.geometries.assign(states.size(), ViewRenderGeometry{});

  auto primary = states[0].normalizer.decode(raw.timestamp_ns, raw.payload, frame_index);
  if (!primary.has_value()) {
    return false;
  }
  {
    if (!canvas.ready() && states[0].total_width.has_value()) {
      // --width: pin the primary's render size from the output width — the
      // cell width is the width split across the grid columns, the height
      // follows the primary frame's aspect ratio. Both are rounded down to
      // even (the codecs' 4:2:0 formats require even dimensions), so the
      // output can be a few pixels narrower than requested. Validation
      // guarantees the cell width is at least 2.
      const std::uint32_t cell_w = (*states[0].total_width / states[0].grid_cols) & ~1U;
      const auto cell_h = static_cast<std::uint32_t>(std::lround(
                            primary->height * (static_cast<double>(cell_w) / primary->width))) &
                          ~1U;
      states[0].renderer.set_fixed_render_size(cell_w, cell_h);
    }
    // The cell size does not exist yet on the first tick; the primary's
    // fixed scale/size does not read it, so the zero cell dimensions are
    // harmless.
    const auto geom = states[0].renderer.prepare(
      primary->width, primary->height, canvas.cell_width(), canvas.cell_height());
    if (!geom.has_value()) {
      return false;
    }
    // The first primary frame fixes the cell size, and with it the composed
    // output size for the whole run.
    if (!canvas.ready()) {
      canvas.set_cell_size(geom->width, geom->height);
    }
    if (!resize_frame(*primary, geom->width, geom->height)) {
      return false;
    }
    out.geometries[0] = *geom;
    out.frames[0] = std::make_shared<const FrameBuffer>(std::move(*primary));
  }

  for (std::size_t i = 1; i < states.size(); ++i) {
    auto & state = states[i];
    std::string error;
    const auto * msg = state.source->fetch(raw.timestamp_ns, error);
    if (!error.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", frame_index, error.c_str());
      return false;
    }
    if (msg == nullptr) {
      if (!state.warned_empty) {
        BAGWIZ_LOG_WARN(
          kLogger, "topic '%s' has no messages; its cell stays black.", state.input->topic.c_str());
        state.warned_empty = true;
      }
      continue;
    }
    if (msg->record_ns != state.cached_record_ns) {
      auto decoded = state.normalizer.decode(msg->record_ns, msg->payload, frame_index);
      if (!decoded.has_value()) {
        return false;
      }
      const auto geom = state.renderer.prepare(
        decoded->width, decoded->height, canvas.cell_width(), canvas.cell_height());
      if (!geom.has_value()) {
        return false;
      }
      if (!resize_frame(*decoded, geom->width, geom->height)) {
        return false;
      }
      state.cache_geom = *geom;
      state.cache = std::make_shared<const FrameBuffer>(std::move(*decoded));
      state.cached_record_ns = msg->record_ns;
    }
    if (state.cache) {
      out.frames[i] = state.cache;
      out.geometries[i] = state.cache_geom;
    }
  }
  return true;
}

// Render every view's selected frame into its cell and encode the composed
// grid. points_per_view[i] (when non-null) holds view i's projected points.
// With draw_labels, each cell's top-left corner then gets its view's topic
// name — including a messageless (black) cell, so the grid always identifies
// its cameras. Returns false on a logged failure.
bool render_tick(
  std::vector<ViewState> & states, const TickData & tick, GridCanvas & canvas,
  VideoFrameEncoder & encoder,
  const std::vector<const std::vector<core::pointcloud::ProjectedPoint> *> & points_per_view,
  bool draw_labels)
{
  canvas.clear();
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (!tick.frames[i]) {
      continue;
    }
    if (!states[i].renderer.render(
          *tick.frames[i], tick.geometries[i], points_per_view[i], canvas.cell(i))) {
      return false;
    }
  }
  if (draw_labels) {
    for (std::size_t i = 0; i < states.size(); ++i) {
      draw_cell_label(canvas.cell(i), states[i].input->topic);
    }
  }
  return encoder.encode(canvas.pixels(), canvas.width(), canvas.height());
}

// Synchronous per-view projection: fetch and project every point-cloud topic
// bound to the view, at the view's render geometry. Returns false on a logged
// failure.
bool project_view_sync(
  const ViewState & state, const TickData & tick, std::size_t view_index,
  std::vector<core::pointcloud::PointCloudFetcher> & fetchers,
  const std::vector<bool> & topic_has_stamps, tf2::BufferCore & tf_buffer,
  const GenerateVideoArgs & args, std::vector<core::pointcloud::ProjectedPoint> & out,
  std::uint64_t frame_index)
{
  out.clear();
  const auto & geom = tick.geometries[view_index];
  for (const auto idx : state.pcd_indexes) {
    std::string pcd_error;
    const auto match = core::pointcloud::choose_frame_match(
      tick.frames[view_index]->header_stamp_ns, tick.frames[view_index]->timestamp_ns,
      topic_has_stamps[idx]);
    const auto * cloud = fetchers[idx].fetch(match.target_ns, match.key, pcd_error);
    if (cloud == nullptr) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", frame_index, pcd_error.c_str());
      return false;
    }
    const auto projected = core::pointcloud::project_cloud_for_frame(
      *cloud, geom.camera_info, tf_buffer, geom.width, geom.height, args.property, geom.rectify,
      match.target_ns);
    if (!projected.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", frame_index, projected.error.c_str());
      return false;
    }
    out.insert(out.end(), projected.points.begin(), projected.points.end());
  }
  return true;
}

// One view's projection job for the threaded path: which point-cloud topics
// (indexes into the shared topic list), the render geometry to project at,
// and the selected frame's two clocks.
struct ViewProjectionRequest
{
  std::size_t view_index = 0;
  ViewRenderGeometry geom;
  std::vector<std::size_t> pcd_indexes;
  std::int64_t frame_header_stamp_ns = 0;
  std::int64_t frame_record_ns = 0;
};

// Result of point-cloud transform/projection work: one point set per request,
// parallel to the requests. Kept separate from ProjectionResult so callers
// can return an error string without throwing.
struct ProjectionWorkResult
{
  std::vector<std::vector<core::pointcloud::ProjectedPoint>> per_request;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Fetch, parse, transform, and project the point clouds nearest each view's
// selected frame. Each topic is matched in its own clock (see
// core::pointcloud::choose_frame_match). Each call opens its own BagReader(s)
// so the work can safely run on a background thread; the caller supplies the
// read-only camera infos (baked into the requests) and TF buffer.
ProjectionWorkResult run_projection_work(
  const std::filesystem::path & input, const std::vector<std::string> & pointcloud_topics,
  const std::vector<std::vector<core::pointcloud::PointCloudIndexEntry>> & entries_per_topic,
  const std::vector<bool> & topic_has_stamps, const std::vector<ViewProjectionRequest> & requests,
  tf2::BufferCore & tf_buffer, core::pointcloud::PointCloudProperty property)
{
  try {
    ProjectionWorkResult combined;
    for (const auto & req : requests) {
      std::vector<core::pointcloud::ProjectedPoint> points;
      for (const auto idx : req.pcd_indexes) {
        core::pointcloud::PointCloudFetcher fetcher(
          input, pointcloud_topics[idx], entries_per_topic[idx]);
        std::string error;
        const auto match = core::pointcloud::choose_frame_match(
          req.frame_header_stamp_ns, req.frame_record_ns, topic_has_stamps[idx]);
        const auto * cloud = fetcher.fetch(match.target_ns, match.key, error);
        if (cloud == nullptr) {
          return {{}, std::move(error)};
        }
        // The matched time doubles as the TF-lookup time, exactly as walk's
        // overlay does it. The TF buffer is keyed by each transform's own
        // header.stamp, so querying at the frame's time keeps a dynamic
        // cloud->camera chain correct; passing nothing would resolve every frame
        // against the last transform in the bag.
        const auto projected = core::pointcloud::project_cloud_for_frame(
          *cloud, req.geom.camera_info, tf_buffer, req.geom.width, req.geom.height, property,
          req.geom.rectify, match.target_ns);
        if (!projected.ok()) {
          return {{}, std::move(projected.error)};
        }
        points.insert(points.end(), projected.points.begin(), projected.points.end());
      }
      combined.per_request.push_back(std::move(points));
    }
    return combined;
  } catch (const std::exception & e) {
    return {{}, std::string("point-cloud projection failed: ") + e.what()};
  }
}

int run_encode_loop_sync(
  io::BagReader & reader, const GenerateVideoArgs & args, const VideoInputValidation & validation,
  VideoInputScan & scan, VideoGeometry & geometry, VideoFrameEncoder & encoder)
{
  auto states = build_view_states(args, validation, scan, geometry);
  if (!states.has_value()) {
    return 1;
  }

  // One cached fetcher per unique topic, so small bags or single-threaded runs
  // do not pay the per-frame BagReader open/close cost.
  std::vector<core::pointcloud::PointCloudFetcher> pcd_fetchers;
  pcd_fetchers.reserve(scan.pcd_topics.size());
  for (std::size_t i = 0; i < scan.pcd_topics.size(); ++i) {
    pcd_fetchers.emplace_back(
      args.input_path, scan.pcd_topics[i], std::move(scan.pcd_spans[i].entries));
  }

  GridCanvas canvas(validation.grid);
  std::vector<std::vector<core::pointcloud::ProjectedPoint>> points_storage(states->size());
  std::vector<const std::vector<core::pointcloud::ProjectedPoint> *> points_per_view(
    states->size(), nullptr);

  io::RawMessage raw;
  while (reader.next(raw)) {
    TickData tick;
    if (!prepare_tick(raw, *states, canvas, encoder.written(), tick)) {
      return 1;
    }
    if (!scan.pcd_topics.empty()) {
      for (std::size_t i = 0; i < states->size(); ++i) {
        points_storage[i].clear();
        points_per_view[i] = nullptr;
        if (!tick.frames[i] || (*states)[i].pcd_indexes.empty()) {
          continue;
        }
        if (!project_view_sync(
              (*states)[i], tick, i, pcd_fetchers, scan.pcd_topic_has_stamps, *geometry.tf_buffer,
              args, points_storage[i], encoder.written())) {
          return 1;
        }
        points_per_view[i] = &points_storage[i];
      }
    }
    if (!render_tick(*states, tick, canvas, encoder, points_per_view, !args.no_label)) {
      return 1;
    }
  }
  return 0;
}

int run_encode_loop_async(
  io::BagReader & reader, const GenerateVideoArgs & args, const VideoInputValidation & validation,
  VideoInputScan & scan, VideoGeometry & geometry, VideoFrameEncoder & encoder)
{
  // The async path needs the per-topic index entries after moving them out of
  // the scan; collect them before the encode loop.
  std::vector<std::vector<core::pointcloud::PointCloudIndexEntry>> entries_per_topic;
  entries_per_topic.reserve(scan.pcd_spans.size());
  for (auto & pcd_span : scan.pcd_spans) {
    entries_per_topic.push_back(std::move(pcd_span.entries));
  }
  const std::vector<std::string> pcd_topics = scan.pcd_topics;
  const std::vector<bool> topic_has_stamps = scan.pcd_topic_has_stamps;

  auto states = build_view_states(args, validation, scan, geometry);
  if (!states.has_value()) {
    return 1;
  }
  tf2::BufferCore & tf_buffer = *geometry.tf_buffer;

  GridCanvas canvas(validation.grid);

  // One tick's in-flight projection: the requests (which map each result back
  // to its view) plus the future holding the point sets.
  struct PendingProjection
  {
    std::vector<ViewProjectionRequest> requests;
    std::future<ProjectionWorkResult> future;
  };

  // Keep one tick of projection work running ahead so that fetch/parse/project
  // for tick N+1 overlaps with rendering and encoding tick N. Runs right after
  // a prepare_tick(), snapshotting that tick's frames and geometries.
  auto launch_projection = [&](const TickData & tick) {
    PendingProjection pending;
    for (std::size_t i = 0; i < states->size(); ++i) {
      if (!tick.frames[i] || (*states)[i].pcd_indexes.empty()) {
        continue;
      }
      ViewProjectionRequest req;
      req.view_index = i;
      req.geom = tick.geometries[i];
      req.pcd_indexes = (*states)[i].pcd_indexes;
      req.frame_header_stamp_ns = tick.frames[i]->header_stamp_ns;
      req.frame_record_ns = tick.frames[i]->timestamp_ns;
      pending.requests.push_back(std::move(req));
    }
    pending.future = std::async(std::launch::async, [&, requests = pending.requests]() {
      return run_projection_work(
        args.input_path, pcd_topics, entries_per_topic, topic_has_stamps, requests, tf_buffer,
        args.property);
    });
    return pending;
  };

  io::RawMessage raw;
  if (!reader.next(raw)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' yielded no frames in the encode pass.",
      validation.views.front().topic.c_str());
    return 1;
  }
  TickData current;
  if (!prepare_tick(raw, *states, canvas, encoder.written(), current)) {
    return 1;
  }
  auto pending = launch_projection(current);

  while (true) {
    auto projected = pending.future.get();
    if (!projected.ok()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "frame %" PRIu64 ": %s", encoder.written(), projected.error.c_str());
      return 1;
    }

    // Prepare the next tick and launch its projection before rendering this
    // one, so the projection overlaps the render/encode below.
    io::RawMessage next_raw;
    const bool has_next = reader.next(next_raw);
    TickData next;
    std::optional<PendingProjection> next_pending;
    if (has_next) {
      if (!prepare_tick(next_raw, *states, canvas, encoder.written(), next)) {
        return 1;
      }
      next_pending = launch_projection(next);
    }

    std::vector<const std::vector<core::pointcloud::ProjectedPoint> *> points_per_view(
      states->size(), nullptr);
    for (std::size_t r = 0; r < pending.requests.size(); ++r) {
      points_per_view[pending.requests[r].view_index] = &projected.per_request[r];
    }
    if (!render_tick(*states, current, canvas, encoder, points_per_view, !args.no_label)) {
      return 1;
    }

    if (!has_next) {
      break;
    }
    current = std::move(next);
    pending = std::move(*next_pending);
  }
  return 0;
}

}  // namespace

VideoSourceCheck check_video_source(const std::filesystem::path & input, const std::string & topic)
{
  VideoSourceCheck check;

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    check.status = VideoSourceStatus::kInputUnopenable;
    check.message = "failed to open '" + input.string() + "': " + e.what();
    return check;
  }

  const io::TopicInfo * found = nullptr;
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      found = &t;
      break;
    }
  }
  if (found == nullptr) {
    check.status = VideoSourceStatus::kTopicNotFound;
    check.message = "topic '" + topic + "' not found in " + input.string();
    return check;
  }

  check.topic_type = found->type;
  if (!is_supported_type(found->type)) {
    check.status = VideoSourceStatus::kUnsupportedType;
    check.message = "topic '" + topic + "' has type '" + found->type +
                    "', which generate video cam cannot render; supported types are " + kImageType +
                    " and " + kCompressedImageType;
    return check;
  }

  check.status = VideoSourceStatus::kOk;
  return check;
}

GridSpec auto_grid_spec(std::size_t view_count)
{
  std::uint32_t cols = 1;
  while (static_cast<std::uint64_t>(cols) * cols < view_count) {
    ++cols;
  }
  const auto rows = static_cast<std::uint32_t>((view_count + cols - 1) / cols);
  return GridSpec{cols, rows};
}

GridParseResult parse_grid_spec(const std::string & text, std::size_t view_count)
{
  if (text.empty()) {
    return GridParseResult{auto_grid_spec(view_count), ""};
  }
  const auto malformed = [&text]() {
    return GridParseResult{{}, "--grid: expected <cols>x<rows> (e.g. 2x2), got '" + text + "'"};
  };
  const auto x = text.find('x');
  if (x == std::string::npos || x == 0 || x + 1 >= text.size()) {
    return malformed();
  }
  const std::string cols_text = text.substr(0, x);
  const std::string rows_text = text.substr(x + 1);
  const auto digits = [](const std::string & s) {
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
  };
  if (!digits(cols_text) || !digits(rows_text)) {
    return malformed();
  }
  std::uint32_t cols = 0;
  std::uint32_t rows = 0;
  std::from_chars(cols_text.data(), cols_text.data() + cols_text.size(), cols);
  std::from_chars(rows_text.data(), rows_text.data() + rows_text.size(), rows);
  if (cols == 0 || rows == 0) {
    return GridParseResult{{}, "--grid: both dimensions must be positive (got '" + text + "')"};
  }
  const std::uint64_t cells = static_cast<std::uint64_t>(cols) * rows;
  if (cells < view_count) {
    return GridParseResult{
      {},
      "--grid '" + text + "' provides " + std::to_string(cells) + " cell(s) for " +
        std::to_string(view_count) + " view(s)"};
  }
  return GridParseResult{GridSpec{cols, rows}, ""};
}

PcdBindings parse_pcd_bindings(
  std::span<const std::string> entries, std::span<const std::string> image_topics)
{
  PcdBindings out;
  for (const auto & entry : entries) {
    const auto eq = entry.find('=');
    if (eq == std::string::npos) {
      out.global_topics.push_back(entry);
      continue;
    }
    const std::string lhs = entry.substr(0, eq);
    const std::string rhs = entry.substr(eq + 1);
    if (lhs.empty() || rhs.empty()) {
      out.error = "malformed --pcd entry '" + entry + "': expected <image_topic>=<pcd_topic>";
      return out;
    }
    if (std::find(image_topics.begin(), image_topics.end(), lhs) == image_topics.end()) {
      out.error = "--pcd entry '" + entry + "' names image topic '" + lhs +
                  "', which is not one of the -t/--topic topics";
      return out;
    }
    out.per_view[lhs].push_back(rhs);
  }
  return out;
}

CamInfoEntries parse_cam_info_entries(
  std::span<const std::string> entries, std::span<const std::string> image_topics)
{
  CamInfoEntries out;
  for (const auto & entry : entries) {
    const auto eq = entry.find('=');
    if (eq == std::string::npos) {
      if (out.global_topic.has_value()) {
        out.error = "--cam-info takes at most one bare <info_topic> value (got a second one: '" +
                    entry + "')";
        return out;
      }
      out.global_topic = entry;
      continue;
    }
    const std::string lhs = entry.substr(0, eq);
    const std::string rhs = entry.substr(eq + 1);
    if (lhs.empty() || rhs.empty()) {
      out.error = "malformed --cam-info entry '" + entry + "': expected <image_topic>=<info_topic>";
      return out;
    }
    if (std::find(image_topics.begin(), image_topics.end(), lhs) == image_topics.end()) {
      out.error = "--cam-info entry '" + entry + "' names image topic '" + lhs +
                  "', which is not one of the -t/--topic topics";
      return out;
    }
    if (!out.per_view.emplace(lhs, rhs).second) {
      out.error = "--cam-info: duplicate override for image topic '" + lhs + "'";
      return out;
    }
  }
  return out;
}

VideoInputValidation validate_video_inputs(const GenerateVideoArgs & args)
{
  VideoInputValidation out;

  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "at least one image topic is required (-t/--topic).");
    out.error = "at least one image topic is required (-t/--topic).";
    return out;
  }

  const auto grid = parse_grid_spec(args.grid, args.topics.size());
  if (!grid.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", grid.error.c_str());
    out.error = grid.error;
    return out;
  }
  out.grid = grid.grid;

  // --width fixes the composed output width; it replaces --resize as the
  // cell-size constraint.
  if (args.width.has_value()) {
    if (args.resize_scale != 1.0f) {
      BAGWIZ_LOG_ERROR(kLogger, "--width and --resize are mutually exclusive.");
      out.error = "--width and --resize are mutually exclusive.";
      return out;
    }
    const std::uint32_t cell_w = (*args.width / out.grid.cols) & ~1U;
    if (cell_w < 2U) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--width %u is too small for %u grid column(s).", *args.width, out.grid.cols);
      out.error = "--width " + std::to_string(*args.width) + " is too small for " +
                  std::to_string(out.grid.cols) + " grid column(s).";
      return out;
    }
  }

  // A topic listed more than once is an error: grid placement is positional,
  // so a duplicate would be two cells showing the same stream.
  std::unordered_set<std::string> seen_topics;
  for (const auto & topic : args.topics) {
    if (!seen_topics.insert(topic).second) {
      BAGWIZ_LOG_ERROR(kLogger, "topic '%s' given more than once", topic.c_str());
      out.error = "topic '" + topic + "' given more than once";
      return out;
    }
  }

  // Validate every source topic and type before touching anything else.
  for (const auto & topic : args.topics) {
    const auto check = check_video_source(args.input_path, topic);
    if (!check.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", check.message.c_str());
      out.error = check.message;
      return out;
    }
    ViewInput view;
    view.topic = topic;
    view.topic_type = check.topic_type;
    out.views.push_back(std::move(view));
  }

  // Split the --pcd / --cam-info entries into global values and per-view
  // bindings, then hand each view its point-cloud topics (global topics
  // first, then the view's own bindings, duplicates removed).
  const auto bindings = parse_pcd_bindings(args.pointcloud_topics, args.topics);
  if (!bindings.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", bindings.error.c_str());
    out.error = bindings.error;
    return out;
  }
  const auto cam_info_entries = parse_cam_info_entries(args.camera_info_entries, args.topics);
  if (!cam_info_entries.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", cam_info_entries.error.c_str());
    out.error = cam_info_entries.error;
    return out;
  }
  for (auto & view : out.views) {
    view.pcd_topics = bindings.global_topics;
    if (const auto it = bindings.per_view.find(view.topic); it != bindings.per_view.end()) {
      for (const auto & topic : it->second) {
        if (
          std::find(view.pcd_topics.begin(), view.pcd_topics.end(), topic) ==
          view.pcd_topics.end()) {
          view.pcd_topics.push_back(topic);
        }
      }
    }
  }

  // Validate every explicit camera-info topic (the global one and each
  // per-view override).
  if (cam_info_entries.global_topic.has_value()) {
    if (const auto err = core::camera_info::validate_camera_info_topic(
          args.input_path, *cam_info_entries.global_topic);
        err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      out.error = *err;
      return out;
    }
  }
  for (const auto & override_entry : cam_info_entries.per_view) {
    if (const auto err =
          core::camera_info::validate_camera_info_topic(args.input_path, override_entry.second);
        err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      out.error = *err;
      return out;
    }
  }

  // Resolve each view's camera-info topic: the per-view override, else the
  // global value, else derivation from the image topic name.
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
      out.error = "failed to open '" + args.input_path.string() + "': " + e.what();
      return out;
    }
    for (auto & view : out.views) {
      if (const auto it = cam_info_entries.per_view.find(view.topic);
          it != cam_info_entries.per_view.end()) {
        view.camera_info_topic = it->second;
      } else if (cam_info_entries.global_topic.has_value()) {
        view.camera_info_topic = cam_info_entries.global_topic;
      } else {
        view.camera_info_topic =
          core::camera_info::resolve_camera_info_topic(view.topic, reader->topics()).topic;
      }

      const bool needs_camera_info = args.rectify || !view.pcd_topics.empty();
      if (needs_camera_info && !view.camera_info_topic.has_value()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "A camera-info topic is required for --rectify or --pcd, but none could be derived from "
          "'%s'. Pass it explicitly with --cam-info %s=<info_topic>.",
          view.topic.c_str(), view.topic.c_str());
        out.error =
          "A camera-info topic is required for --rectify or --pcd, but none could be derived from "
          "'" +
          view.topic + "'. Pass it explicitly with --cam-info " + view.topic + "=<info_topic>.";
        return out;
      }
    }
  }

  // Validate every unique point-cloud topic's presence and type.
  std::unordered_set<std::string> validated_pcd;
  for (const auto & view : out.views) {
    for (const auto & topic : view.pcd_topics) {
      if (!validated_pcd.insert(topic).second) {
        continue;
      }
      std::unique_ptr<io::BagReader> reader;
      try {
        reader = io::open_read(args.input_path);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
        out.error = "failed to open '" + args.input_path.string() + "': " + e.what();
        return out;
      }
      const io::TopicInfo * info = io::find_topic(*reader, topic);
      if (info == nullptr) {
        BAGWIZ_LOG_ERROR(
          kLogger, "pcd topic '%s' not found in %s", topic.c_str(),
          args.input_path.string().c_str());
        out.error = "pcd topic '" + topic + "' not found in " + args.input_path.string();
        return out;
      }
      if (info->type != kPointCloudType) {
        BAGWIZ_LOG_ERROR(
          kLogger, "pcd topic '%s' has type '%s', expected %s", topic.c_str(), info->type.c_str(),
          kPointCloudType);
        out.error =
          "pcd topic '" + topic + "' has type '" + info->type + "', expected " + kPointCloudType;
        return out;
      }
    }
  }

  return out;
}

std::string validate_video_output_path(const std::filesystem::path & output_path, bool overwrite)
{
  // Fail fast on an output collision before the expensive encode, through the
  // same check every other subcommand's -o path runs. The check is
  // non-destructive; finalize_video_output() does the removal just before the
  // rename, so an existing file is only replaced once the new video is fully
  // written.
  if (const auto r = core::check_output_path_free(output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return r.error;
  }

  // Create the output's parent directory if needed.
  if (const auto parent = output_path.parent_path(); !parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      BAGWIZ_LOG_ERROR(
        kLogger, "could not create output directory '%s': %s", parent.string().c_str(),
        ec.message().c_str());
      return "could not create output directory '" + parent.string() + "': " + ec.message();
    }
  }
  return "";
}

VideoInputScan scan_video_inputs(
  const GenerateVideoArgs & args, const VideoInputValidation & validation)
{
  VideoInputScan out;

  // Derive the frame rate from the primary topic's message timestamps.
  const std::string & primary = validation.views.front().topic;
  if (scan_topic_span(args.input_path, primary, out.span) != 0) {
    out.error = "failed to scan topic '" + primary + "'";
    return out;
  }
  if (out.span.count == 0) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' has no messages to render.", primary.c_str());
    out.error = "topic '" + primary + "' has no messages to render.";
    return out;
  }
  out.fps = core::video::derive_frame_rate(out.span.first_ns, out.span.last_ns, out.span.count);

  // Every secondary topic must carry at least one message; a view that can
  // never render would silently produce a black cell otherwise.
  for (std::size_t i = 1; i < validation.views.size(); ++i) {
    TopicSpan span;
    const auto & topic = validation.views[i].topic;
    if (scan_topic_span(args.input_path, topic, span) != 0) {
      out.error = "failed to scan topic '" + topic + "'";
      return out;
    }
    if (span.count == 0) {
      BAGWIZ_LOG_ERROR(kLogger, "topic '%s' has no messages to render.", topic.c_str());
      out.error = "topic '" + topic + "' has no messages to render.";
      return out;
    }
  }

  // Point-cloud overlay: scan timestamps and the selected property's global
  // min/max across the deduplicated union of every view's topics.
  std::unordered_set<std::string> seen;
  for (const auto & view : validation.views) {
    for (const auto & topic : view.pcd_topics) {
      if (seen.insert(topic).second) {
        out.pcd_topics.push_back(topic);
      }
    }
  }
  if (!out.pcd_topics.empty()) {
    out.pcd_spans.resize(out.pcd_topics.size());
    out.pcd_topic_has_stamps.resize(out.pcd_topics.size());
    double running_min = std::numeric_limits<double>::infinity();
    double running_max = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < out.pcd_topics.size(); ++i) {
      if (
        scan_pointcloud_span(
          args.input_path, out.pcd_topics[i], args.property, args.property_min, args.property_max,
          out.pcd_spans[i]) != 0) {
        out.error = "failed to scan point-cloud topic '" + out.pcd_topics[i] + "'";
        return out;
      }
      out.pcd_topic_has_stamps[i] = out.pcd_spans[i].header_stamps_present;
      if (!args.property_min.has_value()) {
        running_min = std::min(running_min, out.pcd_spans[i].property_min);
      }
      if (!args.property_max.has_value()) {
        running_max = std::max(running_max, out.pcd_spans[i].property_max);
      }
    }
    out.global_property_min = args.property_min.value_or(running_min);
    out.global_property_max = args.property_max.value_or(running_max);
  }
  return out;
}

bool should_use_threaded_projection(
  bool has_pointcloud_topics, bool enable_threaded, std::uint64_t frame_count,
  unsigned int hardware_concurrency)
{
  return has_pointcloud_topics && enable_threaded && frame_count >= kThreadingMinFrames &&
         hardware_concurrency > 1;
}

std::string load_video_geometry(
  const GenerateVideoArgs & args, const VideoInputValidation & validation, VideoGeometry & out)
{
  out.camera_infos.resize(validation.views.size());
  bool any_pcd = false;
  for (std::size_t i = 0; i < validation.views.size(); ++i) {
    any_pcd = any_pcd || !validation.views[i].pcd_topics.empty();
    if (!validation.views[i].camera_info_topic.has_value()) {
      continue;
    }
    auto ci =
      core::camera_info::load_camera_info(args.input_path, *validation.views[i].camera_info_topic);
    if (!ci.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", ci.error.c_str());
      return ci.error;
    }
    // Kept UNSCALED: each view's renderer applies its own scale (the primary's
    // --resize or a secondary's fit-to-cell) when it prepares a frame.
    out.camera_infos[i] = std::move(*ci.info);
  }
  if (any_pcd) {
    out.tf_buffer.emplace();
    if (const auto err = core::load_tf_buffer(args.input_path, *out.tf_buffer); err.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
      return *err;
    }
  }
  return "";
}

std::filesystem::path partial_tmp_path_for(const std::filesystem::path & output)
{
  return output.parent_path() /
         (output.stem().string() + ".bagwiz-partial" + output.extension().string());
}

PartialFileGuard::PartialFileGuard(std::filesystem::path tmp_path) : tmp_path_(std::move(tmp_path))
{
  // Clear any stale temp from a previous aborted run.
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

PartialFileGuard::~PartialFileGuard()
{
  std::error_code ec;
  std::filesystem::remove(tmp_path_, ec);
}

std::string finalize_video_output(
  const std::filesystem::path & tmp_path, const std::filesystem::path & output_path, bool overwrite)
{
  // Now that the new video is complete, replace any existing output and move
  // the temp into place.
  if (const auto r = core::prepare_output_path(output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return r.error;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, output_path, ec);
  if (ec) {
    // Fall back to copy + remove across filesystems.
    std::error_code copy_ec;
    std::filesystem::copy_file(
      tmp_path, output_path, std::filesystem::copy_options::overwrite_existing, copy_ec);
    std::error_code remove_ec;
    std::filesystem::remove(tmp_path, remove_ec);
    if (copy_ec) {
      BAGWIZ_LOG_ERROR(kLogger, "could not move output into place: %s", copy_ec.message().c_str());
      return "could not move output into place: " + copy_ec.message();
    }
  }
  return "";
}

std::unique_ptr<io::BagReader> open_encode_reader(const GenerateVideoArgs & args)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "failed to open '%s': %s", args.input_path.string().c_str(), e.what());
    return nullptr;
  }
  io::ReadFilter filter;
  filter.topics.push_back(args.topics.front());
  reader->set_filter(filter);
  return reader;
}

std::optional<FrameBuffer> FrameNormalizer::decode(
  std::int64_t timestamp_ns, std::span<const std::byte> payload, std::uint64_t frame_index) const
{
  // Normalize either message type to a canonical packed BGR24 raster via the
  // shared core::image::to_packed_raster seam; rgb8 inputs are swapped so
  // every frame the encoder sees is BGR24.
  auto pr = core::image::to_packed_raster(topic_type_, payload);
  if (!pr.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", frame_index, pr.error.c_str());
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

bool resize_frame(FrameBuffer & frame, std::uint32_t out_w, std::uint32_t out_h)
{
  if (frame.width == out_w && frame.height == out_h) {
    return true;
  }
  if (out_w == 0 || out_h == 0) {
    BAGWIZ_LOG_ERROR(
      kLogger, "resize to %ux%u would produce a zero-size frame (%ux%u)", out_w, out_h, frame.width,
      frame.height);
    return false;
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
  return true;
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

void GridCanvas::set_cell_size(std::uint32_t width, std::uint32_t height)
{
  cell_w_ = width;
  cell_h_ = height;
  pixels_.assign(
    static_cast<std::size_t>(grid_.cols) * cell_w_ * 3U * grid_.rows * cell_h_, std::byte{0});
}

void GridCanvas::clear()
{
  std::fill(pixels_.begin(), pixels_.end(), std::byte{0});
}

CellView GridCanvas::cell(std::size_t index)
{
  const std::size_t stride = static_cast<std::size_t>(width()) * 3U;
  const std::size_t col = index % grid_.cols;
  const std::size_t row = index / grid_.cols;
  return CellView{
    pixels_.data() + row * cell_h_ * stride + col * cell_w_ * 3U, cell_w_, cell_h_, stride};
}

void draw_cell_label(const CellView & cell, const std::string & label)
{
  if (label.empty() || cell.data == nullptr || cell.width == 0 || cell.height == 0) {
    return;
  }
  cv::Mat cell_mat(
    static_cast<int>(cell.height), static_cast<int>(cell.width), CV_8UC3, cell.data, cell.stride);
  // The font scale follows the cell height so the label stays readable across
  // --resize / --width, with a floor that keeps small cells legible.
  const double scale = std::max(0.4, cell.height / 240.0);
  const int thickness = std::max(1, static_cast<int>(std::lround(scale)));
  // putText's origin is the text's bottom-left: place the baseline the text
  // height plus a small margin below the cell's top edge.
  const cv::Point origin(4, static_cast<int>(std::lround(scale * 22.0)) + 2);
  // A black outline under the white fill keeps the label readable on any
  // frame, bright or dark.
  cv::putText(
    cell_mat, label, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness + 2,
    cv::LINE_8);
  cv::putText(
    cell_mat, label, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(255, 255, 255), thickness,
    cv::LINE_8);
}

ViewRenderer::ViewRenderer(
  const core::image::CameraInfo * camera_info, bool rectify, const VideoOverlayParams & params,
  std::optional<double> fixed_scale)
: camera_info_(camera_info), rectify_(rectify), params_(params), fixed_scale_(fixed_scale)
{
}

std::optional<ViewRenderGeometry> ViewRenderer::prepare(
  std::uint32_t native_w, std::uint32_t native_h, std::uint32_t cell_w, std::uint32_t cell_h)
{
  const bool size_locked = fixed_scale_.has_value() || fixed_size_.has_value();
  if (size_locked) {
    // The primary view's native size is locked to its first frame, exactly as
    // the single-view encoder locked its geometry.
    if (native_w_ != 0 && (native_w != native_w_ || native_h != native_h_)) {
      BAGWIZ_LOG_ERROR(
        kLogger, "frame changed to %ux%u from the first frame's %ux%u; aborting.", native_w,
        native_h, native_w_, native_h_);
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
    BAGWIZ_LOG_ERROR(
      kLogger, "scale %.3g would produce a zero-size frame (%ux%u)", scale, render_w, render_h);
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
    // view's scale, then match the render size (a verbatim copy when the two
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

bool ViewRenderer::render(
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
      BAGWIZ_LOG_ERROR(kLogger, "overlay failed: %s", err.c_str());
      return false;
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
  return true;
}

VideoFrameEncoder::VideoFrameEncoder(
  const std::filesystem::path & tmp_path, core::video::FrameRate fps)
: tmp_path_(tmp_path), fps_(fps)
{
}

bool VideoFrameEncoder::encode(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  if (encoder_ == nullptr) {
    // The first frame fixes the geometry and pixel encoding for the run.
    enc_w_ = width;
    enc_h_ = height;
    auto opened = core::video::open_video_encoder(tmp_path_, enc_w_, enc_h_, fps_.num, fps_.den);
    if (!opened.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", opened.error.c_str());
      return false;
    }
    encoder_ = std::move(opened.encoder);
  } else if (width != enc_w_ || height != enc_h_) {
    BAGWIZ_LOG_ERROR(
      kLogger, "frame %" PRIu64 " changed to %ux%u from the first frame's %ux%u; aborting.",
      written_, width, height, enc_w_, enc_h_);
    return false;
  }

  if (auto e = encoder_->write_frame(bgr, width * 3U, core::video::SourcePixelFormat::kBgr8);
      !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "frame %" PRIu64 ": %s", written_, e.c_str());
    return false;
  }
  ++written_;
  return true;
}

std::string VideoFrameEncoder::finish()
{
  if (auto e = encoder_->finish(); !e.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", e.c_str());
    encoder_.reset();
    return e;
  }
  encoder_.reset();  // close the temp file before the rename/clobber
  return "";
}

int run_encode_pass(
  io::BagReader & reader, const GenerateVideoArgs & args, const VideoInputValidation & validation,
  VideoInputScan & scan, VideoGeometry & geometry, VideoFrameEncoder & encoder)
{
  try {
    if (should_use_threaded_projection(
          !scan.pcd_topics.empty(), args.enable_threaded_projection, scan.span.count,
          std::thread::hardware_concurrency())) {
      return run_encode_loop_async(reader, args, validation, scan, geometry, encoder);
    }
    return run_encode_loop_sync(reader, args, validation, scan, geometry, encoder);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "error reading topic '%s': %s", validation.views.front().topic.c_str(), e.what());
    return 1;
  }
}

std::string finish_video_encode(
  VideoFrameEncoder & encoder, const std::string & topic, const std::filesystem::path & tmp_path,
  const std::filesystem::path & output_path, bool overwrite)
{
  // Pass 1 saw messages, but if pass 2 yielded none (e.g. the bag changed
  // between passes) the encoder was never created. Nothing was rendered.
  if (!encoder.started()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic '%s' yielded no frames in the encode pass.", topic.c_str());
    return "topic '" + topic + "' yielded no frames in the encode pass.";
  }
  if (const auto err = encoder.finish(); !err.empty()) {
    return err;
  }
  return finalize_video_output(tmp_path, output_path, overwrite);
}

void log_video_summary(
  const std::filesystem::path & output_path, std::uint64_t written, std::uint32_t width,
  std::uint32_t height, core::video::FrameRate fps)
{
  const double fps_value = static_cast<double>(fps.num) / static_cast<double>(fps.den);
  BAGWIZ_LOG_INFO(
    kLogger, "generate video cam: wrote %" PRIu64 " frame(s) to %s (%ux%u bgr8 @ %.3g fps).",
    written, output_path.string().c_str(), width, height, fps_value);

  if (is_h264_extension(output_path)) {
    if (is_vlc_available()) {
      BAGWIZ_LOG_INFO(
        kLogger, "H.264 output saved. If mpv fails to play, try VLC or run mpv --hwdec=no.");
    } else {
      BAGWIZ_LOG_WARN(
        kLogger,
        "H.264 output saved. If mpv fails to play, run mpv --hwdec=no, or install VLC (%s)",
        vlc_install_hint());
    }
  }
}

}  // namespace bagwiz::commands
