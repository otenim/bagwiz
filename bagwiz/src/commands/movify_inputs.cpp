// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_inputs.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";
// kImageType / kCompressedImageType mirror topic_types.hpp's kImageTopicTypes
// (movify --cam's allowed_types) via is_supported_type() below.
// kPointCloudType mirrors topic_types.hpp's kPointCloud2Type (--cam-pcd's
// allowed_types). Keep both in sync by hand.
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

bool is_supported_type(const std::string & type)
{
  return type == kImageType || type == kCompressedImageType;
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

// The first message of a point-cloud topic, parsed: the point-cloud panels
// take their view frame and BEV extent from it.
struct FirstCloud
{
  std::optional<core::pointcloud::PointCloud2> cloud;
  std::string error;
};

FirstCloud read_first_cloud(const std::filesystem::path & input, const std::string & topic)
{
  FirstCloud out;
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    out.error = "failed to open '" + input.string() + "': " + e.what();
    return out;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);
  io::RawMessage raw;
  try {
    if (!reader->next(raw)) {
      out.error = "topic '" + topic + "' has no messages to render.";
      return out;
    }
  } catch (const std::exception & e) {
    out.error = "error reading topic '" + topic + "': " + e.what();
    return out;
  }
  auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
  if (!parsed.ok()) {
    out.error = "failed to parse the first cloud of topic '" + topic + "': " + parsed.error;
    return out;
  }
  out.cloud = std::move(*parsed.cloud);
  return out;
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
                    "', which movify cannot render; supported types are " + kImageType + " and " +
                    kCompressedImageType;
    return check;
  }

  check.status = VideoSourceStatus::kOk;
  return check;
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
      out.error = "malformed --cam-pcd entry '" + entry + "': expected <image_topic>=<pcd_topic>";
      return out;
    }
    if (std::find(image_topics.begin(), image_topics.end(), lhs) == image_topics.end()) {
      out.error = "--cam-pcd entry '" + entry + "' names image topic '" + lhs +
                  "', which is not one of the --cam topics";
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
                  "', which is not one of the --cam topics";
      return out;
    }
    if (!out.per_view.emplace(lhs, rhs).second) {
      out.error = "--cam-info: duplicate override for image topic '" + lhs + "'";
      return out;
    }
  }
  return out;
}

bool view_rectifies(bool rectify_requested, const ViewInput & view) noexcept
{
  // --cam-pcd hard-requires a resolved camera-info topic (validate_video_inputs
  // fails the run otherwise), so a projecting view always has one; honoring
  // --no-rectify here is what lets the projection fall back to the raw,
  // distortion-aware path in core::pointcloud::project_pointcloud.
  return rectify_requested && view.camera_info_topic.has_value();
}

VideoInputValidation validate_video_inputs(const MovifyArgs & args)
{
  VideoInputValidation out;

  if (args.cam_topics.empty() && args.pcd_topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "nothing to render: pass at least one --cam or --pcd topic.");
    out.error = "nothing to render: pass at least one --cam or --pcd topic.";
    return out;
  }

  // The point-cloud panels: one per requested view, every --pcd topic in
  // each.
  if (!args.pcd_topics.empty()) {
    if (args.views.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "--view must name at least one projection (3d, bev).");
      out.error = "--view must name at least one projection (3d, bev).";
      return out;
    }
    for (std::size_t i = 0; i < args.views.size(); ++i) {
      for (std::size_t j = 0; j < i; ++j) {
        if (args.views[i] == args.views[j]) {
          BAGWIZ_LOG_ERROR(kLogger, "--view names the same projection more than once.");
          out.error = "--view names the same projection more than once.";
          return out;
        }
      }
    }
    out.pcd_views = args.views;
  }
  const std::size_t panel_count =
    args.cam_topics.size() + (args.pcd_topics.empty() ? 0 : args.views.size());

  const auto grid = parse_grid_spec(args.grid, panel_count);
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
  for (const auto & topic : args.cam_topics) {
    if (!seen_topics.insert(topic).second) {
      BAGWIZ_LOG_ERROR(kLogger, "topic '%s' given more than once", topic.c_str());
      out.error = "topic '" + topic + "' given more than once";
      return out;
    }
  }

  // Validate every source topic and type before touching anything else.
  for (const auto & topic : args.cam_topics) {
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

  // Every --pcd topic is drawn once.
  {
    std::unordered_set<std::string> seen_pcd;
    for (const auto & topic : args.pcd_topics) {
      if (!seen_pcd.insert(topic).second) {
        BAGWIZ_LOG_ERROR(kLogger, "topic '%s' given more than once", topic.c_str());
        out.error = "topic '" + topic + "' given more than once";
        return out;
      }
    }
    out.pcd_topics = args.pcd_topics;
  }

  // --clock names the panel whose messages define the frames; it must be one
  // of the --cam or --pcd topics. Unset picks the first camera panel, else the
  // first point-cloud panel (the panels follow the camera panels in the grid).
  if (args.clock.has_value()) {
    const auto cam_it = std::find(args.cam_topics.begin(), args.cam_topics.end(), *args.clock);
    const auto pcd_it = std::find(args.pcd_topics.begin(), args.pcd_topics.end(), *args.clock);
    if (cam_it != args.cam_topics.end()) {
      out.clock = static_cast<std::size_t>(cam_it - args.cam_topics.begin());
    } else if (pcd_it != args.pcd_topics.end()) {
      out.clock_pcd = static_cast<std::size_t>(pcd_it - args.pcd_topics.begin());
      out.clock = args.cam_topics.size();
    } else {
      BAGWIZ_LOG_ERROR(
        kLogger, "--clock '%s' is not one of the --cam or --pcd topics.", args.clock->c_str());
      out.error = "--clock '" + *args.clock + "' is not one of the --cam or --pcd topics.";
      return out;
    }
  } else if (args.cam_topics.empty()) {
    out.clock_pcd = 0;
    out.clock = 0;
  }

  // Split the --cam-pcd / --cam-info entries into global values and per-view
  // bindings, then hand each view its point-cloud topics (global topics
  // first, then the view's own bindings, duplicates removed).
  const auto bindings = parse_pcd_bindings(args.cam_pcd_entries, args.cam_topics);
  if (!bindings.ok()) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", bindings.error.c_str());
    out.error = bindings.error;
    return out;
  }
  const auto cam_info_entries = parse_cam_info_entries(args.camera_info_entries, args.cam_topics);
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

      // Rectification is on by default but degrades to a warning: a view
      // whose camera info cannot be resolved simply renders unrectified.
      // Point-cloud projection still hard-requires one.
      if (view.camera_info_topic.has_value()) {
        continue;
      }
      if (!view.pcd_topics.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "A camera-info topic is required for --cam-pcd, but none could be derived from "
          "'%s'. Pass it explicitly with --cam-info %s=<info_topic>.",
          view.topic.c_str(), view.topic.c_str());
        out.error =
          "A camera-info topic is required for --cam-pcd, but none could be derived from '" +
          view.topic + "'. Pass it explicitly with --cam-info " + view.topic + "=<info_topic>.";
        return out;
      }
      if (args.rectify) {
        BAGWIZ_LOG_WARN(
          kLogger,
          "no camera-info topic could be derived for '%s'; rendering it unrectified (pass "
          "--cam-info %s=<info_topic>, or --no-rectify to disable rectification).",
          view.topic.c_str(), view.topic.c_str());
      }
    }
  }

  // Validate every unique point-cloud topic's presence and type: the panels'
  // topics and every camera panel's overlay topics.
  std::vector<std::string> pcd_to_validate = out.pcd_topics;
  for (const auto & view : out.views) {
    pcd_to_validate.insert(pcd_to_validate.end(), view.pcd_topics.begin(), view.pcd_topics.end());
  }
  {
    std::unordered_set<std::string> validated_pcd;
    for (const auto & topic : pcd_to_validate) {
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

  // The point-cloud panels' view frame and BEV extent come from the first
  // cloud of the first --pcd topic: its frame_id unless --frame names one,
  // and, for a BEV view without --range, a percentile of its ground distances.
  if (!out.pcd_topics.empty()) {
    const auto first = read_first_cloud(args.input_path, out.pcd_topics.front());
    if (!first.cloud.has_value()) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", first.error.c_str());
      out.error = first.error;
      return out;
    }
    out.frame = args.frame.value_or(first.cloud->frame_id);
    if (out.frame.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "topic '%s' carries no frame_id; pass --frame.", out.pcd_topics.front().c_str());
      out.error = "topic '" + out.pcd_topics.front() + "' carries no frame_id; pass --frame.";
      return out;
    }
    const bool any_bev = std::any_of(
      out.pcd_views.begin(), out.pcd_views.end(), [](core::pointcloud::CloudProjection p) {
        return p == core::pointcloud::CloudProjection::kBev;
      });
    if (any_bev) {
      if (args.range_m.has_value()) {
        out.range_m = *args.range_m;
      } else {
        // The extent that keeps most of the first cloud in view: a percentile
        // of its ground distances, so a few far returns do not shrink the scene.
        std::string error;
        const auto range = core::pointcloud::bev_auto_range(
          *first.cloud, core::pointcloud::kBevAutoRangeQuantile, error);
        if (!range.has_value()) {
          BAGWIZ_LOG_ERROR(
            kLogger,
            "could not determine --range from the first cloud of topic '%s' (%s); pass "
            "--range explicitly.",
            out.pcd_topics.front().c_str(), error.c_str());
          out.error = "could not determine --range from the first cloud of topic '" +
                      out.pcd_topics.front() + "' (" + error + "); pass --range explicitly.";
          return out;
        }
        out.range_m = *range;
        BAGWIZ_LOG_INFO(
          kLogger,
          "bev: --range not given; using %.1f m, the %.0fth percentile of the first "
          "cloud's ground distances.",
          *range, 100.0 * core::pointcloud::kBevAutoRangeQuantile);
      }
    }
  }

  return out;
}

VideoInputScan scan_video_inputs(const MovifyArgs & args, const VideoInputValidation & validation)
{
  VideoInputScan out;

  // Derive the frame rate from the clock topic's message timestamps.
  const std::string & primary = clock_topic_of(validation);
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

  // Every other topic must carry at least one message; a view that can never
  // render would silently produce a black cell otherwise.
  for (std::size_t i = 0; i < validation.views.size(); ++i) {
    if (i == validation.clock) {
      continue;
    }
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

  // Point clouds: scan timestamps and the selected property's global min/max
  // across the deduplicated union of the panels' topics and every camera
  // panel's overlay topics.
  std::unordered_set<std::string> seen;
  for (const auto & topic : validation.pcd_topics) {
    if (seen.insert(topic).second) {
      out.pcd_topics.push_back(topic);
    }
  }
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
      // (A topic without messages already failed the scan above.)
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

std::string load_video_geometry(
  const MovifyArgs & args, const VideoInputValidation & validation, VideoGeometry & out)
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
    // Kept UNSCALED: each view's renderer applies its own scale (the clock's
    // --resize or another panel's fit-to-cell) when it prepares a frame.
    out.camera_infos[i] = std::move(*ci.info);
  }
  // Overlays always look their clouds up in TF; the point-cloud panels only
  // when a cloud may sit in another frame than the view frame (an explicit
  // --frame, or more than one topic). A bag without TF is fatal for the
  // former and a warning for the latter: the panel reports the missing
  // transform if it turns out to need one.
  const bool panels_may_need_tf =
    !validation.pcd_topics.empty() && (args.frame.has_value() || validation.pcd_topics.size() > 1);
  if (any_pcd || panels_may_need_tf) {
    out.tf_buffer.emplace();
    if (const auto err = core::load_tf_buffer(args.input_path, *out.tf_buffer); err.has_value()) {
      if (any_pcd) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", err->c_str());
        return *err;
      }
      BAGWIZ_LOG_WARN(
        kLogger, "%s; a point-cloud panel cloud outside the '%s' frame cannot be drawn.",
        err->c_str(), validation.frame.c_str());
      out.tf_buffer.reset();
    }
  }
  return "";
}

std::unique_ptr<io::BagReader> open_encode_reader(
  const std::filesystem::path & input, const std::string & clock_topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "failed to open '%s': %s", input.string().c_str(), e.what());
    return nullptr;
  }
  io::ReadFilter filter;
  filter.topics.push_back(clock_topic);
  reader->set_filter(filter);
  return reader;
}

}  // namespace bagwiz::commands
