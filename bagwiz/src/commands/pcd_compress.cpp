// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_compress.hpp"

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
#include <optional>
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
constexpr const char * kCmd = "pcd compress";
// Mirrors topic_types.hpp's kPointCloud2Type (pcd compress -t's
// allowed_types). Keep both in sync by hand.
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";

// Range-check one --*-bits flag. CLI11 enforces the range on the CLI; the
// runner is also a direct API entry (tests), so it validates again.
bool check_bits(const std::optional<int> & bits, const char * flag)
{
  if (bits.has_value() && (*bits < 1 || *bits > 31)) {
    BAGWIZ_LOG_ERROR(kLogger, "%s: %s must be in 1..31 (got %d)", kCmd, flag, *bits);
    return false;
  }
  return true;
}

core::pointcloud::DracoEncodeConfig make_encode_config(const PcdCompressArgs & args)
{
  core::pointcloud::DracoEncodeConfig config;
  config.lossless = args.lossless;
  if (args.position_bits.has_value()) {
    config.quantization_position = *args.position_bits;
  }
  if (args.normal_bits.has_value()) {
    config.quantization_normal = *args.normal_bits;
  }
  if (args.color_bits.has_value()) {
    config.quantization_color = *args.color_bits;
  }
  if (args.generic_bits.has_value()) {
    config.quantization_generic = *args.generic_bits;
  }
  return config;
}

// The per-message transform: PointCloud2 -> CompressedPointCloud2 ("draco").
// Messages that cannot be encoded (unparseable, big-endian, or non-dense
// under lossy quantization — quantizing NaN/Inf is undefined) pass through
// under the original topic with a once-per-topic WARN.
PcdTransformResult compress_message(
  std::span<const std::byte> payload, const core::pointcloud::DracoEncodeConfig & config)
{
  namespace pc = core::pointcloud;
  PcdTransformResult result;

  const auto parsed = pc::parse_pointcloud2(payload);
  if (!parsed.ok()) {
    result.passthrough_reason = "not parseable as PointCloud2: " + parsed.error;
    return result;
  }
  const pc::PointCloud2 & cloud = *parsed.cloud;
  if (!cloud.is_dense && !config.lossless) {
    result.passthrough_reason =
      "the cloud is not dense, and lossy quantization of non-finite points is undefined "
      "(re-run with --lossless to compress it)";
    return result;
  }
  const auto encoded = pc::encode_draco(cloud, config);
  if (!encoded.ok()) {
    result.passthrough_reason = "Draco encoding failed: " + encoded.error;
    return result;
  }

  pc::CompressedPointCloud2 message;
  message.timestamp_ns = cloud.timestamp_ns;
  message.frame_id = cloud.frame_id;
  message.height = cloud.height;
  message.width = cloud.width;
  message.fields = cloud.fields;
  message.is_bigendian = cloud.is_bigendian;
  message.point_step = cloud.point_step;
  message.row_step = cloud.row_step;
  message.is_dense = cloud.is_dense;
  message.format = pc::kDracoFormatName;
  message.compressed_data = std::move(*encoded.data);
  result.payload = pc::serialize_compressed_pointcloud2(message);
  return result;
}

void log_compress_summary(const PcdTransformPlan & plan, const PcdTransformStats & stats)
{
  std::uint64_t total_messages = 0;
  for (const auto & [input_info, out_name] : plan.mappings) {
    const auto it = stats.find(input_info->name);
    if (it == stats.end()) {
      continue;
    }
    const auto & s = it->second;
    total_messages += s.messages;
    const double ratio =
      s.out_bytes > 0 ? static_cast<double>(s.in_bytes) / static_cast<double>(s.out_bytes) : 0.0;
    if (s.passthrough > 0) {
      BAGWIZ_LOG_INFO(
        kLogger,
        "  %s -> %s: %" PRIu64 " message(s), %" PRIu64 " -> %" PRIu64 " bytes (%.2fx); %" PRIu64
        " passed through uncompressed",
        input_info->name.c_str(), out_name.c_str(), s.messages, s.in_bytes, s.out_bytes, ratio,
        s.passthrough);
    } else {
      BAGWIZ_LOG_INFO(
        kLogger, "  %s -> %s: %" PRIu64 " message(s), %" PRIu64 " -> %" PRIu64 " bytes (%.2fx)",
        input_info->name.c_str(), out_name.c_str(), s.messages, s.in_bytes, s.out_bytes, ratio);
    }
  }
  BAGWIZ_LOG_INFO(
    kLogger, "%s: compressed %" PRIu64 " message(s) across %zu topic(s)", kCmd, total_messages,
    plan.mappings.size());
}

}  // namespace

int run_pcd_compress(const PcdCompressArgs & args)
{
  // ---- validate arguments ---------------------------------------------------
  if (
    !check_bits(args.position_bits, "--position-bits") ||
    !check_bits(args.normal_bits, "--normal-bits") ||
    !check_bits(args.color_bits, "--color-bits") ||
    !check_bits(args.generic_bits, "--generic-bits")) {
    return 1;
  }
  if (
    args.lossless && (args.position_bits.has_value() || args.normal_bits.has_value() ||
                      args.color_bits.has_value() || args.generic_bits.has_value())) {
    BAGWIZ_LOG_ERROR(kLogger, "%s: --*-bits flags have no effect with --lossless", kCmd);
    return 1;
  }

  // Claim -o before touching the bag. The check is non-destructive (the
  // dispatch below removes the existing entry), so an -o collision costs one
  // stat instead of a read pass.
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
    *reader, args.topics, args.output_topic, args.force, kPointCloud2Type,
    core::pointcloud::kCompressedPointCloud2TypeName,
    [](const std::string & name) { return name + "/draco"; }, args.input_path, kLogger, kCmd);
  if (!plan.has_value()) {
    return 1;
  }

  // ---- transform pass (-o vs in-place shared dispatch) ----------------------
  const core::pointcloud::DracoEncodeConfig config = make_encode_config(args);
  const int num_threads =
    resolve_num_threads(args.threads.value_or(0), std::thread::hardware_concurrency());
  PcdTransformStats stats;

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "pcd compress: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "pcd compress: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  const int status = core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) {
      return run_pcd_transform_pass(
        factory, *reader, args.input_path, plan->suppress, plan->output_by_input,
        plan->declare_outputs,
        [&](const std::string & /*topic*/, std::span<const std::byte> payload) {
          return compress_message(payload, config);
        },
        num_threads, stats, kLogger, kCmd);
    });
  if (status != 0) {
    return status;
  }

  log_compress_summary(*plan, stats);
  return 0;
}

}  // namespace bagwiz::commands
