// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/video_encode.hpp"

#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/compressed_video.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/image_wire.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/video/frame_codec.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

namespace img = bagwiz::core::image;
namespace vid = bagwiz::core::video;

constexpr const char * kLogger = "bagwiz.cmd.video.encode";
constexpr const char * kLabel = "video encode";

// Encodes one source topic's frames. The encoder opens on the first frame,
// whose geometry and range fix the stream; every later frame must match.
class SourceEncoder
{
public:
  SourceEncoder(const VideoTopicPlan & plan, const VideoEncodeArgs & args, vid::FrameRate fps)
  : plan_(plan), args_(args), fps_(fps)
  {
  }

  // Convert one source message into a CompressedVideo payload. Returns ""
  // on success or an error that ends the run.
  std::string convert(const io::RawMessage & msg, std::vector<std::byte> & out)
  {
    counters_.messages_in += 1;
    counters_.bytes_in += msg.payload.size();
    FrameHeader header;
    vid::EncodeFrameResult encoded;
    const std::string error = plan_.source_type == img::kImageType
                                ? encode_raw_image(msg.payload, header, encoded)
                                : encode_compressed_image(msg.payload, header, encoded);
    if (!error.empty()) {
      return error;
    }
    const std::int64_t stamp = header.stamp_ns != 0 ? header.stamp_ns : msg.timestamp_ns;
    out = img::serialize_compressed_video(
      stamp, header.frame_id, vid::video_codec_format(args_.codec), encoded.frame->data);
    counters_.messages_out += 1;
    counters_.bytes_out += out.size();
    return {};
  }

  [[nodiscard]] const VideoTopicCounters & counters() const noexcept { return counters_; }
  [[nodiscard]] const std::string & backend() const noexcept { return backend_; }

private:
  struct FrameHeader
  {
    std::int64_t stamp_ns = 0;
    std::string frame_id;
  };

  std::string encode_raw_image(
    std::span<const std::byte> payload, FrameHeader & header, vid::EncodeFrameResult & encoded)
  {
    const auto extracted = img::extract_raw_image(payload);
    if (!extracted.ok()) {
      return extracted.error;
    }
    const img::RawImageView & view = *extracted.image;
    header = FrameHeader{view.header_stamp_ns, view.header_frame_id};
    vid::SourcePixelFormat format = vid::SourcePixelFormat::kBgr8;
    if (view.encoding == "rgb8") {
      format = vid::SourcePixelFormat::kRgb8;
    } else if (view.encoding != "bgr8") {
      return "image encoding '" + view.encoding + "' is not supported; only bgr8 and rgb8.";
    }
    if (auto e = ensure_open(view.width, view.height, /*full_range=*/false); !e.empty()) {
      return e;
    }
    encoded = encoder_->encode(view.data, view.step, format);
    return encoded.ok() ? std::string{} : encoded.error;
  }

  std::string encode_compressed_image(
    std::span<const std::byte> payload, FrameHeader & header, vid::EncodeFrameResult & encoded)
  {
    const auto extracted = img::extract_compressed_image(payload);
    if (!extracted.ok()) {
      return extracted.error;
    }
    const img::CompressedImageView & view = *extracted.image;
    header = FrameHeader{view.header_stamp_ns, view.header_frame_id};
    // JPEG frames usually decode to 4:2:0 planes the encoder takes as they
    // are; the stream's range follows the first such frame. Anything else
    // (PNG, other chroma layouts, a frame in the other range) goes through
    // a BGR raster and the encoder's own conversion.
    const auto yuv = decoder_.decode_to_yuv(view.data, view.format);
    if (yuv.ok() && yuv.view->chroma == img::YuvChroma::k420) {
      const img::DecodedYuvView & planes = *yuv.view;
      if (auto e = ensure_open(planes.width, planes.height, planes.full_range); !e.empty()) {
        return e;
      }
      if (planes.full_range == full_range_) {
        encoded = encoder_->encode_yuv420(
          vid::Yuv420Planes{
            planes.y, static_cast<std::size_t>(planes.y_stride), planes.u,
            static_cast<std::size_t>(planes.u_stride), planes.v,
            static_cast<std::size_t>(planes.v_stride)});
        return encoded.ok() ? std::string{} : encoded.error;
      }
    }
    const auto decoded = decoder_.decode(view.data, view.format);
    if (!decoded.ok()) {
      return decoded.error;
    }
    const img::DecodedImage & raster = *decoded.image;
    if (auto e = ensure_open(raster.width, raster.height, /*full_range=*/false); !e.empty()) {
      return e;
    }
    encoded = encoder_->encode(
      raster.bgr, static_cast<std::size_t>(raster.width) * 3U, vid::SourcePixelFormat::kBgr8);
    return encoded.ok() ? std::string{} : encoded.error;
  }

  // Open the encoder on the first frame; check later frames against it.
  std::string ensure_open(std::uint32_t width, std::uint32_t height, bool full_range)
  {
    if (encoder_ != nullptr) {
      if (width != width_ || height != height_) {
        return "frame size changed from " + std::to_string(width_) + "x" + std::to_string(height_) +
               " to " + std::to_string(width) + "x" + std::to_string(height) +
               "; a video stream needs one geometry";
      }
      return {};
    }
    vid::FrameEncoderOptions options;
    options.codec = args_.codec;
    options.backend = args_.encoder;
    options.preset = args_.preset;
    options.crf = args_.crf.value_or(-1);
    options.gop = args_.gop;
    options.threads = args_.threads.value_or(0);
    options.full_range = full_range;
    auto opened = vid::open_frame_encoder(width, height, fps_.num, fps_.den, options);
    if (!opened.ok()) {
      return opened.error;
    }
    if (!opened.fallback_note.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "%s: '%s': %s; using %s instead.", kLabel, plan_.source.c_str(),
        opened.fallback_note.c_str(), opened.backend.c_str());
    }
    BAGWIZ_LOG_INFO(
      kLogger, "%s: '%s' %" PRIu32 "x%" PRIu32 " @ %.3g fps -> '%s' as %s via %s (crf %d, gop %d).",
      kLabel, plan_.source.c_str(), width, height,
      static_cast<double>(fps_.num) / static_cast<double>(fps_.den), plan_.output.c_str(),
      std::string{vid::video_codec_format(args_.codec)}.c_str(), opened.backend.c_str(),
      options.crf < 0 ? vid::default_frame_crf(args_.codec) : options.crf, options.gop);
    encoder_ = std::move(opened.encoder);
    backend_ = std::move(opened.backend);
    width_ = width;
    height_ = height;
    full_range_ = full_range;
    return {};
  }

  const VideoTopicPlan & plan_;
  const VideoEncodeArgs & args_;
  vid::FrameRate fps_;
  std::unique_ptr<vid::FrameEncoder> encoder_;
  std::string backend_;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  bool full_range_ = false;
  img::ImageDecoder decoder_;
  VideoTopicCounters counters_;
};

// The nominal rate of a source, from the bag's per-topic counts over its
// time span: it only steers rate control, so the bag-wide span is close
// enough and costs no extra scan.
std::unordered_map<std::string, vid::FrameRate> estimate_frame_rates(
  io::BagReader & reader, std::span<const VideoTopicPlan> plan)
{
  std::unordered_map<std::string, vid::FrameRate> rates;
  const io::BagReader::Stats stats = reader.compute_stats();
  for (const auto & entry : plan) {
    std::uint64_t count = 0;
    if (const auto it = stats.per_topic.find(entry.source); it != stats.per_topic.end()) {
      count = it->second > 0 ? static_cast<std::uint64_t>(it->second) : 0;
    }
    rates[entry.source] = vid::derive_frame_rate(stats.start_ns, stats.end_ns, count);
  }
  return rates;
}

void log_summary(
  const std::vector<VideoTopicPlan> & plan,
  const std::unordered_map<std::string, std::unique_ptr<SourceEncoder>> & encoders)
{
  for (const auto & entry : plan) {
    const SourceEncoder & enc = *encoders.at(entry.source);
    const VideoTopicCounters & c = enc.counters();
    if (c.messages_in == 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "%s: topic '%s' carried no messages; '%s' was declared but is empty.", kLabel,
        entry.source.c_str(), entry.output.c_str());
      continue;
    }
    BAGWIZ_LOG_INFO(
      kLogger, "%s: '%s' -> '%s': %" PRIu64 " frame(s) via %s, %s.", kLabel, entry.source.c_str(),
      entry.output.c_str(), c.messages_out, enc.backend().c_str(),
      describe_size_change(c.bytes_in, c.bytes_out).c_str());
  }
}

// The rewrite pass: copy every surviving message through, encode each
// source frame right where it sits in the stream, and write the video
// message under the source's record time so the output stays in order.
int execute_encode_pass(
  const VideoEncodeArgs & args, const std::vector<VideoTopicPlan> & plan,
  const std::unordered_map<std::string, vid::FrameRate> & rates,
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }
  const bool declared = declare_video_pass_topics(
    *reader, *writer, plan, args.keep_inputs,
    [](const VideoTopicPlan & entry) {
      return img::make_compressed_video_topic_info(entry.output);
    },
    kLogger);
  if (!declared) {
    return 1;
  }

  std::unordered_map<std::string, std::unique_ptr<SourceEncoder>> encoders;
  std::unordered_map<std::string, std::string> output_of;
  for (const auto & entry : plan) {
    encoders.emplace(
      entry.source, std::make_unique<SourceEncoder>(entry, args, rates.at(entry.source)));
    output_of.emplace(entry.source, entry.output);
  }

  io::RawMessage raw;
  try {
    std::vector<std::byte> video_payload;  // reused across frames
    while (reader->next(raw)) {
      const std::string & name = raw.topic->name;
      const auto it = encoders.find(name);
      if (it == encoders.end() || args.keep_inputs) {
        writer->write(name, raw.timestamp_ns, raw.payload);
      }
      if (it == encoders.end()) {
        continue;
      }
      if (const auto e = it->second->convert(raw, video_payload); !e.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "%s: topic '%s' message #%" PRIu64 ": %s", kLabel, name.c_str(),
          it->second->counters().messages_in, e.c_str());
        return 1;
      }
      writer->write(output_of.at(name), raw.timestamp_ns, video_payload);
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "%s: read/write failed: %s", kLabel, e.what());
    return 1;
  }
  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }
  log_summary(plan, encoders);
  return 0;
}

}  // namespace

std::string default_video_topic(const std::string & source_topic)
{
  return source_topic + kVideoTopicSuffix;
}

int run_video_encode(const VideoEncodeArgs & args)
{
  // Claim -o before touching the bag. The check is non-destructive (the
  // dispatch below removes the existing entry), so it only moves the
  // collision verdict ahead of the read.
  if (args.output_path.has_value()) {
    if (const auto r = core::check_output_path_free(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }
  if (args.gop < 1) {
    BAGWIZ_LOG_ERROR(kLogger, "%s: --gop must be at least 1.", kLabel);
    return 1;
  }

  std::vector<VideoTopicPlan> plan;
  std::unordered_map<std::string, vid::FrameRate> rates;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    VideoTopicPlanRequest request;
    request.reader = reader.get();
    request.sources = args.topics;
    request.allowed_types = kImageTopicTypes;
    request.as_topic = args.as_topic;
    request.keep_inputs = args.keep_inputs;
    request.default_output = default_video_topic;
    request.command_label = kLabel;
    request.logger = kLogger;
    auto planned = plan_video_topics(request);
    if (!planned.has_value()) {
      return 1;
    }
    plan = std::move(*planned);
    rates = estimate_frame_rates(*reader, plan);
  }

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "video encode: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "video encode: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) {
      return execute_encode_pass(args, plan, rates, factory);
    });
}

}  // namespace bagwiz::commands
