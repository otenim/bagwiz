// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_decompress.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/pointcloud/compressed_pointcloud2.hpp"
#include "bagwiz/core/pointcloud/draco_codec.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "pcd_compress_common.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "worker_threads.hpp"       // NOLINT(build/include_subdir) src-local shared header

#include <cinttypes>
#include <cstddef>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr const char * kLogger = "bagwiz.cmd.pcd";
constexpr const char * kCmd = "pcd decompress";
// Mirrors topic_types.hpp's kCompressedPointCloud2Type (pcd decompress -t's
// allowed_types). Keep both in sync by hand.
constexpr const char * kCompressedType = "point_cloud_interfaces/msg/CompressedPointCloud2";
constexpr const char * kPointCloud2TypeName = "sensor_msgs/msg/PointCloud2";
constexpr const char * kDracoSuffix = "/draco";

// The default output name: the input topic with the "/draco" suffix stripped.
// Empty when the topic does not carry the suffix (plan_pcd_transform turns
// that into an --as-naming error).
std::string default_decompress_name(const std::string & name)
{
  constexpr std::size_t suffix_len = 6;  // "/draco"
  if (
    name.size() > suffix_len &&
    name.compare(name.size() - suffix_len, suffix_len, kDracoSuffix) == 0) {
    return name.substr(0, name.size() - suffix_len);
  }
  return "";
}

// The per-message transform: CompressedPointCloud2 ("draco") -> PointCloud2.
// Messages that are not parseable as CompressedPointCloud2 or carry a
// different codec pass through under the original topic with a
// once-per-topic WARN; a Draco decode failure or a point count that
// contradicts height*width is fatal.
PcdTransformResult decompress_message(std::span<const std::byte> payload)
{
  namespace pc = core::pointcloud;
  PcdTransformResult result;

  const auto parsed = pc::parse_compressed_pointcloud2(payload);
  if (!parsed.ok()) {
    result.passthrough_reason = "not parseable as CompressedPointCloud2: " + parsed.error;
    return result;
  }
  const pc::CompressedPointCloud2 & message = *parsed.message;
  if (message.format != pc::kDracoFormatName) {
    result.passthrough_reason = "format \"" + message.format + "\" is not \"draco\"";
    return result;
  }

  const auto decoded =
    pc::decode_draco(message.compressed_data, message.fields, message.point_step);
  if (!decoded.ok()) {
    result.error = "Draco decoding failed: " + decoded.error;
    return result;
  }
  if (
    static_cast<std::uint64_t>(decoded.num_points) !=
    static_cast<std::uint64_t>(message.height) * message.width) {
    result.error = "decoded point count " + std::to_string(decoded.num_points) +
                   " does not match height*width (" + std::to_string(message.height) + "*" +
                   std::to_string(message.width) + ")";
    return result;
  }

  pc::PointCloud2 cloud;
  cloud.timestamp_ns = message.timestamp_ns;
  cloud.frame_id = message.frame_id;
  cloud.height = message.height;
  cloud.width = message.width;
  cloud.fields = message.fields;
  cloud.is_bigendian = message.is_bigendian;
  cloud.point_step = message.point_step;
  cloud.row_step = message.row_step;
  cloud.is_dense = message.is_dense;
  cloud.data = std::move(*decoded.data);
  result.payload = pc::serialize_pointcloud2(cloud);
  return result;
}

void log_decompress_summary(const PcdTransformPlan & plan, const PcdTransformStats & stats)
{
  std::uint64_t total_messages = 0;
  for (const auto & [input_info, out_name] : plan.mappings) {
    const auto it = stats.find(input_info->name);
    if (it == stats.end()) {
      continue;
    }
    const auto & s = it->second;
    total_messages += s.messages;
    if (s.passthrough > 0) {
      BAGWIZ_LOG_INFO(
        kLogger,
        "  %s -> %s: %" PRIu64 " message(s), %" PRIu64 " -> %" PRIu64 " bytes; %" PRIu64
        " passed through compressed",
        input_info->name.c_str(), out_name.c_str(), s.messages, s.in_bytes, s.out_bytes,
        s.passthrough);
    } else {
      BAGWIZ_LOG_INFO(
        kLogger, "  %s -> %s: %" PRIu64 " message(s), %" PRIu64 " -> %" PRIu64 " bytes",
        input_info->name.c_str(), out_name.c_str(), s.messages, s.in_bytes, s.out_bytes);
    }
  }
  BAGWIZ_LOG_INFO(
    kLogger, "%s: decompressed %" PRIu64 " message(s) across %zu topic(s)", kCmd, total_messages,
    plan.mappings.size());
}

}  // namespace

int run_pcd_decompress(const PcdDecompressArgs & args)
{
  // Claim -o before touching the bag (same non-destructive pre-check as the
  // other rewrite commands).
  if (args.output_path.has_value()) {
    if (const auto r = core::check_output_path_free(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }

  // ---- open reader, plan the transform --------------------------------------
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();
  const auto plan = plan_pcd_transform(
    *reader, args.topics, args.output_topic, args.force, kCompressedType, kPointCloud2TypeName,
    default_decompress_name, args.input_path, kLogger, kCmd);
  if (!plan.has_value()) {
    return 1;
  }

  // ---- transform pass (-o vs in-place shared dispatch) ----------------------
  const int num_threads =
    resolve_num_threads(args.threads.value_or(0), std::thread::hardware_concurrency());
  PcdTransformStats stats;

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "pcd decompress: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "pcd decompress: pass failed; aborting in-place swap";
  const int status = core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) {
      return run_pcd_transform_pass(
        factory, *reader, args.input_path, plan->suppress, plan->output_by_input,
        plan->declare_outputs,
        [](const std::string & /*topic*/, std::span<const std::byte> payload) {
          return decompress_message(payload);
        },
        num_threads, stats, kLogger, kCmd);
    });
  if (status != 0) {
    return status;
  }

  log_decompress_summary(*plan, stats);
  return 0;
}

}  // namespace bagwiz::commands
