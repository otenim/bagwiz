// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/video_decode.hpp"

#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/commands/video_encode.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/image/compressed_video.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/image/image_wire.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/video/frame_codec.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <algorithm>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <deque>
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

constexpr const char * kLogger = "bagwiz.cmd.video.decode";
constexpr const char * kLabel = "video decode";

// What a decoded frame is written as, per --format.
struct OutputShape
{
  const char * type = "";           // message type of the output topic
  const char * format_or_enc = "";  // CompressedImage.format or Image.encoding
};

OutputShape output_shape(DecodedImageFormat format)
{
  switch (format) {
    case DecodedImageFormat::kPng:
      return {"sensor_msgs/msg/CompressedImage", "png"};
    case DecodedImageFormat::kRaw:
      return {"sensor_msgs/msg/Image", "bgr8"};
    case DecodedImageFormat::kJpeg:
    default:
      return {"sensor_msgs/msg/CompressedImage", "jpeg"};
  }
}

// One message decoded into an image payload, with the source header it
// carries over.
struct DecodedMessage
{
  std::int64_t record_ns = 0;
  std::vector<std::byte> payload;
};

// Decodes one source topic's packets. The decoder opens on the first
// message, whose `format` fixes the codec. Packets and frames pair up in
// order: each source message's stamp and frame_id are queued when its
// packet goes in and dequeued when a frame comes out, so a stream that
// decodes with delay still stamps every frame with its own message's header.
class SourceDecoder
{
public:
  SourceDecoder(const VideoTopicPlan & plan, const VideoDecodeArgs & args)
  : plan_(plan), args_(args)
  {
  }

  // Feed one source message and collect every frame that comes out.
  // Returns "" on success or an error that ends the run.
  std::string convert(const io::RawMessage & msg, std::vector<DecodedMessage> & out)
  {
    counters_.messages_in += 1;
    counters_.bytes_in += msg.payload.size();
    const auto extracted = img::extract_compressed_video(msg.payload);
    if (!extracted.ok()) {
      return extracted.error;
    }
    const img::CompressedVideoView & view = *extracted.video;
    if (auto e = ensure_open(view.format); !e.empty()) {
      return e;
    }
    if (view.data.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger, "%s: '%s' message #%" PRIu64 " carries no data; skipped.", kLabel,
        plan_.source.c_str(), counters_.messages_in);
      return {};
    }
    pending_.push_back(PendingHeader{view.timestamp_ns, view.frame_id, msg.timestamp_ns});
    if (auto e = decoder_->send(view.data); !e.empty()) {
      return e;
    }
    return drain(out);
  }

  // End of stream: release frames the decoder still holds.
  std::string finish(std::vector<DecodedMessage> & out)
  {
    if (decoder_ == nullptr) {
      return {};
    }
    if (auto e = decoder_->flush(); !e.empty()) {
      return e;
    }
    if (auto e = drain(out); !e.empty()) {
      return e;
    }
    if (!pending_.empty()) {
      BAGWIZ_LOG_WARN(
        kLogger,
        "%s: '%s': %zu message(s) produced no frame (undecodable, or a stream that starts "
        "before its first keyframe).",
        kLabel, plan_.source.c_str(), pending_.size());
      counters_.messages_dropped += pending_.size();
      pending_.clear();
    }
    return {};
  }

  [[nodiscard]] const VideoTopicCounters & counters() const noexcept { return counters_; }
  [[nodiscard]] std::uint64_t dropped() const noexcept { return counters_.messages_dropped; }

private:
  struct PendingHeader
  {
    std::int64_t stamp_ns = 0;
    std::string frame_id;
    std::int64_t record_ns = 0;
  };

  std::string ensure_open(const std::string & format)
  {
    if (decoder_ != nullptr) {
      if (format != format_) {
        return "video format changed from '" + format_ + "' to '" + format +
               "'; a topic must keep one codec";
      }
      return {};
    }
    const auto codec = vid::parse_video_codec_format(format);
    if (!codec.has_value()) {
      return "video format '" + format + "' is not supported; only h264 and h265";
    }
    auto opened = vid::open_frame_decoder(*codec);
    if (!opened.ok()) {
      return opened.error;
    }
    decoder_ = std::move(opened.decoder);
    format_ = format;
    const OutputShape shape = output_shape(args_.format);
    BAGWIZ_LOG_INFO(
      kLogger, "%s: '%s' (%s) -> '%s' as %s %s.", kLabel, plan_.source.c_str(), format.c_str(),
      plan_.output.c_str(), shape.type, shape.format_or_enc);
    return {};
  }

  // Pull every ready frame out of the decoder and serialize it under the
  // oldest pending header.
  std::string drain(std::vector<DecodedMessage> & out)
  {
    while (true) {
      auto received = decoder_->receive();
      if (!received.ok()) {
        return received.error;
      }
      if (!received.frame.has_value()) {
        return {};
      }
      if (pending_.empty()) {
        return "decoder produced more frames than packets were sent";
      }
      const PendingHeader header = std::move(pending_.front());
      pending_.pop_front();
      DecodedMessage message;
      message.record_ns = header.record_ns;
      if (auto e = serialize(*received.frame, header, message.payload); !e.empty()) {
        return e;
      }
      counters_.messages_out += 1;
      counters_.bytes_out += message.payload.size();
      out.push_back(std::move(message));
    }
  }

  std::string serialize(
    const vid::DecodedFrame & frame, const PendingHeader & header, std::vector<std::byte> & payload)
  {
    const OutputShape shape = output_shape(args_.format);
    if (args_.format == DecodedImageFormat::kRaw) {
      payload = img::serialize_raw_image(
        header.stamp_ns, header.frame_id, frame.width, frame.height, shape.format_or_enc,
        frame.width * 3U, frame.bgr);
      return {};
    }
    img::PackedRaster raster;
    raster.width = frame.width;
    raster.height = frame.height;
    raster.bgr = frame.bgr;  // the encoders take the raster by reference
    raster.encoding = "bgr8";
    if (args_.format == DecodedImageFormat::kPng) {
      const auto png = img::encode_png(raster);
      if (!png.ok()) {
        return "PNG encode failed: " + png.error;
      }
      payload = img::serialize_compressed_image(
        header.stamp_ns, header.frame_id, shape.format_or_enc, *png.png);
      return {};
    }
    const auto jpeg = img::encode_jpeg(raster, args_.quality);
    if (!jpeg.ok()) {
      return "JPEG encode failed: " + jpeg.error;
    }
    payload = img::serialize_compressed_image(
      header.stamp_ns, header.frame_id, shape.format_or_enc, *jpeg.jpeg);
    return {};
  }

  const VideoTopicPlan & plan_;
  const VideoDecodeArgs & args_;
  std::unique_ptr<vid::FrameDecoder> decoder_;
  std::string format_;
  std::deque<PendingHeader> pending_;
  VideoTopicCounters counters_;
};

io::TopicInfo make_output_topic_info(const VideoTopicPlan & entry, DecodedImageFormat format)
{
  if (format == DecodedImageFormat::kRaw) {
    return img::make_image_topic_info(entry.output);
  }
  return img::make_compressed_image_topic_info(entry.output);
}

void log_summary(
  const std::vector<VideoTopicPlan> & plan,
  const std::unordered_map<std::string, std::unique_ptr<SourceDecoder>> & decoders)
{
  for (const auto & entry : plan) {
    const SourceDecoder & dec = *decoders.at(entry.source);
    const VideoTopicCounters & c = dec.counters();
    if (c.messages_in == 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "%s: topic '%s' carried no messages; '%s' was declared but is empty.", kLabel,
        entry.source.c_str(), entry.output.c_str());
      continue;
    }
    BAGWIZ_LOG_INFO(
      kLogger, "%s: '%s' -> '%s': %" PRIu64 " frame(s) from %" PRIu64 " message(s), %s.", kLabel,
      entry.source.c_str(), entry.output.c_str(), c.messages_out, c.messages_in,
      describe_size_change(c.bytes_in, c.bytes_out).c_str());
  }
}

int execute_decode_pass(
  const VideoDecodeArgs & args, const std::vector<VideoTopicPlan> & plan,
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
    [&](const VideoTopicPlan & entry) { return make_output_topic_info(entry, args.format); },
    kLogger);
  if (!declared) {
    return 1;
  }

  std::unordered_map<std::string, std::unique_ptr<SourceDecoder>> decoders;
  std::unordered_map<std::string, std::string> output_of;
  for (const auto & entry : plan) {
    decoders.emplace(entry.source, std::make_unique<SourceDecoder>(entry, args));
    output_of.emplace(entry.source, entry.output);
  }

  io::RawMessage raw;
  try {
    std::vector<DecodedMessage> decoded;  // reused across messages
    while (reader->next(raw)) {
      const std::string & name = raw.topic->name;
      const auto it = decoders.find(name);
      if (it == decoders.end() || args.keep_inputs) {
        writer->write(name, raw.timestamp_ns, raw.payload);
      }
      if (it == decoders.end()) {
        continue;
      }
      decoded.clear();
      if (const auto e = it->second->convert(raw, decoded); !e.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "%s: topic '%s' message #%" PRIu64 ": %s", kLabel, name.c_str(),
          it->second->counters().messages_in, e.c_str());
        return 1;
      }
      for (const auto & message : decoded) {
        writer->write(output_of.at(name), message.record_ns, message.payload);
      }
    }
    for (const auto & entry : plan) {
      decoded.clear();
      if (const auto e = decoders.at(entry.source)->finish(decoded); !e.empty()) {
        BAGWIZ_LOG_ERROR(kLogger, "%s: topic '%s': %s", kLabel, entry.source.c_str(), e.c_str());
        return 1;
      }
      for (const auto & message : decoded) {
        writer->write(entry.output, message.record_ns, message.payload);
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "%s: read/write failed: %s", kLabel, e.what());
    return 1;
  }
  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }
  log_summary(plan, decoders);
  return 0;
}

}  // namespace

std::string default_image_topic(const std::string & source_topic)
{
  const std::string_view suffix{kVideoTopicSuffix};
  if (
    source_topic.size() > suffix.size() &&
    std::string_view{source_topic}.substr(source_topic.size() - suffix.size()) == suffix) {
    return source_topic.substr(0, source_topic.size() - suffix.size());
  }
  return source_topic + kImageTopicSuffix;
}

int run_video_decode(const VideoDecodeArgs & args)
{
  if (args.output_path.has_value()) {
    if (const auto r = core::check_output_path_free(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
  }
  if (args.quality < img::kJpegQualityMin || args.quality > img::kJpegQualityMax) {
    BAGWIZ_LOG_ERROR(
      kLogger, "%s: --quality must be within %d..%d.", kLabel, img::kJpegQualityMin,
      img::kJpegQualityMax);
    return 1;
  }

  std::vector<VideoTopicPlan> plan;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    VideoTopicPlanRequest request;
    request.reader = reader.get();
    request.sources = args.topics;
    request.allowed_types = kCompressedVideoTopicTypes;
    request.as_topic = args.as_topic;
    request.keep_inputs = args.keep_inputs;
    request.default_output = default_image_topic;
    request.command_label = kLabel;
    request.logger = kLogger;
    auto planned = plan_video_topics(request);
    if (!planned.has_value()) {
      return 1;
    }
    plan = std::move(*planned);
  }

  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "video decode: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "video decode: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & factory) { return execute_decode_pass(args, plan, factory); });
}

}  // namespace bagwiz::commands
