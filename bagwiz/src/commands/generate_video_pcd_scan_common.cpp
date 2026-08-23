// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "generate_video_pcd_scan_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
// Mirrors topic_types.hpp's kPointCloud2Type (generate video scan -t's
// allowed_types). Keep the two in sync by hand.
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";

// Largest finite XY distance (meters) in `cloud`, reading x/y as doubles.
// Returns nullopt when no point is finite.
std::optional<double> max_xy_distance(const core::pointcloud::PointCloud2 & cloud)
{
  const auto read_f = [](const std::byte * base, core::pointcloud::PointFieldType type) -> double {
    if (type == core::pointcloud::PointFieldType::kFloat32) {
      float v = 0.0F;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<double>(v);
    }
    if (type == core::pointcloud::PointFieldType::kFloat64) {
      double v = 0.0;
      std::memcpy(&v, base, sizeof(v));
      return v;
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  const core::pointcloud::PointField * fx = nullptr;
  const core::pointcloud::PointField * fy = nullptr;
  for (const auto & f : cloud.fields) {
    if (f.name == "x") {
      fx = &f;
    } else if (f.name == "y") {
      fy = &f;
    }
  }
  if (fx == nullptr || fy == nullptr || cloud.point_step == 0) {
    return std::nullopt;
  }
  const std::uint32_t rstep = cloud.row_step != 0 ? cloud.row_step : cloud.width * cloud.point_step;
  if (cloud.data.size() < static_cast<std::size_t>(cloud.height) * rstep) {
    return std::nullopt;
  }

  std::optional<double> best;
  for (std::uint32_t r = 0; r < cloud.height; ++r) {
    for (std::uint32_t c = 0; c < cloud.width; ++c) {
      const std::byte * base = cloud.data.data() + static_cast<std::size_t>(r) * rstep +
                               static_cast<std::size_t>(c) * cloud.point_step;
      const double x = read_f(base + fx->offset, fx->datatype);
      const double y = read_f(base + fy->offset, fy->datatype);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      const double d = std::sqrt(x * x + y * y);
      if (!best.has_value() || d > *best) {
        best = d;
      }
    }
  }
  return best;
}

}  // namespace

PcdScanValidation validate_pcd_scan_inputs(const GenerateVideoPcdScanArgs & args)
{
  PcdScanValidation out;

  // Cheap argument checks first: they catch CLI-wiring and direct-call mistakes
  // before the bag is even opened.
  if (args.width < 2 || args.height < 2 || args.width % 2 != 0 || args.height % 2 != 0) {
    out.error = "output dimensions must be even and at least 2, got " + std::to_string(args.width) +
                "x" + std::to_string(args.height) + " (H.264 requires even dimensions)";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  if (args.fps < 1 || args.fps > static_cast<std::uint32_t>(core::video::kMaxFps)) {
    out.error = "--fps must be in [1, " + std::to_string(core::video::kMaxFps) + "]";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  if (!(args.speed > 0.0) || !std::isfinite(args.speed)) {
    out.error = "--speed must be a positive, finite value";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  if (args.range_m.has_value() && *args.range_m <= 0.0) {
    out.error = "--range must be positive";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  if (args.dist_m <= 0.0) {
    out.error = "--dist must be positive";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    out.error = "failed to open '" + args.input_path.string() + "': " + e.what();
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }

  const io::TopicInfo * info = io::find_topic(*reader, args.topic);
  if (info == nullptr) {
    out.error = "topic '" + args.topic + "' not found in " + args.input_path.string();
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  if (info->type != kPointCloud2Type) {
    out.error =
      "topic '" + args.topic + "' has type '" + info->type + "', expected " + kPointCloud2Type;
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }

  // Parse the topic's first message: it carries the field layout the command
  // requires (x/y/z plus a per-point time) and doubles as pass 1's auto-range
  // sample.
  io::ReadFilter filter;
  filter.topics.push_back(args.topic);
  reader->set_filter(filter);
  io::RawMessage raw;
  try {
    if (!reader->next(raw)) {
      out.error = "topic '" + args.topic + "' has no messages to render.";
      BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
      return out;
    }
  } catch (const std::exception & e) {
    out.error = "error reading topic '" + args.topic + "': " + e.what();
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }

  auto parsed = core::pointcloud::parse_pointcloud2(raw.payload);
  if (!parsed.ok()) {
    out.error = "failed to parse the first cloud of topic '" + args.topic + "': " + parsed.error;
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  out.first_cloud = std::move(*parsed.cloud);

  const auto field_offset = [&](const char * name) { return out.first_cloud.field_offset(name); };
  if (!field_offset("x") || !field_offset("y") || !field_offset("z")) {
    out.error = "topic '" + args.topic + "' is missing required x/y/z fields";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  const auto time_field = core::pointcloud::find_point_time_field(out.first_cloud);
  if (!time_field.has_value()) {
    out.error = "topic '" + args.topic +
                "' has no per-point time field ('t', 'time', 'time_stamp', or 'timestamp'; "
                "UINT32 nanoseconds or FLOAT32/FLOAT64 seconds), which generate video scan "
                "requires to order the points by firing time";
    BAGWIZ_LOG_ERROR(kLogger, "%s", out.error.c_str());
    return out;
  }
  out.time_field = *time_field;
  return out;
}

std::uint32_t scan_frames_per_sweep(
  core::video::FrameRate cloud_fps, std::uint32_t fps, double speed)
{
  const double cloud_rate =
    static_cast<double>(std::max(cloud_fps.num, 1)) / std::max(cloud_fps.den, 1);
  const double frames = static_cast<double>(fps) / (cloud_rate * speed);
  return static_cast<std::uint32_t>(std::max(1.0, std::round(frames)));
}

std::string scan_pcd_scan_span(
  const std::filesystem::path & input, const std::string & topic, TopicSpan & out)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    const std::string error = "failed to open '" + input.string() + "': " + e.what();
    BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
    return error;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  // Ask the storage layer to skip the point payloads: pass 1 only needs the
  // timestamps (see io::ReadFilter::payload_topics).
  filter.payload_topics.emplace_back("__bagwiz_no_payload__");
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
    const std::string error = "error reading topic '" + topic + "': " + e.what();
    BAGWIZ_LOG_ERROR(kLogger, "%s", error.c_str());
    return error;
  }
  return "";
}

std::optional<double> auto_range_from_cloud(const core::pointcloud::PointCloud2 & cloud)
{
  return max_xy_distance(cloud);
}

}  // namespace bagwiz::commands
