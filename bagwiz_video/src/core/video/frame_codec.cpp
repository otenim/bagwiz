// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/frame_codec.hpp"

#include "bagwiz/core/video/annexb.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "libav_support.hpp"  // NOLINT(build/include_subdir) src-local shared header

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libavutil/rational.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core::video
{
namespace
{

using detail::av_err;

AVCodecID codec_id_for(VideoCodec codec)
{
  return codec == VideoCodec::kH265 ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264;
}

detail::BackendPolicy policy_for(EncoderBackend backend)
{
  switch (backend) {
    case EncoderBackend::kCpu:
      return detail::BackendPolicy::kCpuOnly;
    case EncoderBackend::kNvenc:
      return detail::BackendPolicy::kNvencOnly;
    case EncoderBackend::kAuto:
    default:
      return detail::BackendPolicy::kAuto;
  }
}

// Encoder options that make every frame come straight back out as one
// packet — no B-frames, no lookahead, no frame-threading delay — with a
// keyframe every `gop` frames and the parameter sets repeated in-band on
// each of them (no global header is requested, so the wrappers emit them
// before every IDR).
void set_zero_latency_options(
  AVCodecContext * codec, const char * encoder_name, const FrameEncoderOptions & options, int crf,
  AVDictionary ** opts)
{
  codec->max_b_frames = 0;
  codec->gop_size = options.gop;
  codec->keyint_min = options.gop;
  if (options.threads > 0) {
    codec->thread_count = options.threads;
  }
  av_dict_set(opts, "bf", "0", 0);
  const std::string crf_text = std::to_string(crf);
  if (detail::is_nvenc_encoder(encoder_name)) {
    av_dict_set(opts, "preset", detail::nvenc_preset_for(options.preset), 0);
    av_dict_set(opts, "rc", "vbr", 0);
    av_dict_set(opts, "cq", crf_text.c_str(), 0);
    av_dict_set(opts, "delay", "0", 0);
    av_dict_set(opts, "zerolatency", "1", 0);
    av_dict_set(opts, "rc-lookahead", "0", 0);
    return;
  }
  av_dict_set(opts, "preset", options.preset.c_str(), 0);
  av_dict_set(opts, "tune", "zerolatency", 0);
  av_dict_set(opts, "crf", crf_text.c_str(), 0);
  if (options.codec == VideoCodec::kH265) {
    // Closed GOPs make every keyframe an IDR a player can start from without
    // leading pictures to discard; scenecut off keeps the GOP regular.
    av_dict_set(opts, "x265-params", "open-gop=0:scenecut=0:repeat-headers=1", 0);
    return;
  }
  // Scenecut off: with B-frames off and a short GOP, libx264 otherwise tends
  // to mark every frame as an I-frame, blowing up the size for no gain.
  av_dict_set(opts, "sc_threshold", "0", 0);
}

// The plain YUV* format behind a deprecated full-range YUVJ* one, marking
// `full_range` when the mapping applied; any other format passes through.
AVPixelFormat normalize_pixel_format(AVPixelFormat fmt, bool & full_range)
{
  switch (fmt) {
    case AV_PIX_FMT_YUVJ420P:
      full_range = true;
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
      full_range = true;
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
      full_range = true;
      return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P:
      full_range = true;
      return AV_PIX_FMT_YUV440P;
    default:
      return fmt;
  }
}

bool starts_with_start_code(const AVCodecContext * codec)
{
  return codec->extradata != nullptr && codec->extradata_size >= 4 &&
         summarize_annexb(
           {reinterpret_cast<
              const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
              codec->extradata),
            static_cast<std::size_t>(codec->extradata_size)},
           VideoCodec::kH264)
           .starts_with_start_code;
}

}  // namespace

int default_frame_crf(VideoCodec codec) noexcept
{
  return codec == VideoCodec::kH265 ? 28 : 23;
}

struct FrameEncoder::Impl
{
  AVCodecContext * codec = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;
  std::unique_ptr<detail::FrameUploader> uploader;
  VideoCodec video_codec = VideoCodec::kH264;
  std::vector<std::byte> out;  // the packet handed to the caller
  std::int64_t next_pts = 0;
  bool failed = false;

  Impl() = default;
  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

  ~Impl()
  {
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
  }

  // Encode the frame currently uploaded and hand back its packet. Exactly
  // one packet must come out per frame; anything else breaks the pairing of
  // packets with source messages and is reported as an error.
  EncodeFrameResult submit()
  {
    EncodeFrameResult result;
    if (failed) {
      result.error = "encoder is unusable after an earlier error";
      return result;
    }
    frame->pts = next_pts++;
    int ret = avcodec_send_frame(codec, frame);
    if (ret < 0) {
      failed = true;
      result.error = "encoder send_frame failed: " + av_err(ret);
      return result;
    }
    out.clear();
    int packets = 0;
    bool keyframe = false;
    while (true) {
      ret = avcodec_receive_packet(codec, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      }
      if (ret < 0) {
        failed = true;
        result.error = "encoder receive_packet failed: " + av_err(ret);
        return result;
      }
      ++packets;
      keyframe = (pkt->flags & AV_PKT_FLAG_KEY) != 0;
      const auto * data =
        reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          pkt->data);
      out.insert(out.end(), data, data + static_cast<std::size_t>(pkt->size));
      av_packet_unref(pkt);
    }
    if (packets != 1) {
      failed = true;
      result.error = "encoder " + std::string(codec->codec->name) + " returned " +
                     std::to_string(packets) + " packet(s) for one frame; one is required";
      return result;
    }
    if (auto e = ensure_parameter_sets(keyframe); !e.empty()) {
      failed = true;
      result.error = std::move(e);
      return result;
    }
    result.frame = EncodedFrame{std::span<const std::byte>(out.data(), out.size()), keyframe};
    return result;
  }

  // A keyframe packet must carry the parameter sets a decoder needs to start
  // from it. The wrappers repeat them in-band when no global header was
  // requested; should one not, prepend the codec's extradata (Annex B when
  // present) and fail loudly if that still leaves them missing.
  std::string ensure_parameter_sets(bool keyframe)
  {
    if (!keyframe) {
      return {};
    }
    const auto needs = [&](const AnnexBSummary & s) {
      return !s.has_sps || !s.has_pps || (video_codec == VideoCodec::kH265 && !s.has_vps);
    };
    if (!needs(summarize_annexb(out, video_codec))) {
      return {};
    }
    if (starts_with_start_code(codec)) {
      const auto * extra =
        reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          codec->extradata);
      out.insert(out.begin(), extra, extra + static_cast<std::size_t>(codec->extradata_size));
    }
    if (needs(summarize_annexb(out, video_codec))) {
      return "encoder " + std::string(codec->codec->name) +
             " emitted a keyframe without its parameter sets (SPS/PPS" +
             (video_codec == VideoCodec::kH265 ? "/VPS" : "") +
             "); the stream would not be decodable from that frame";
    }
    return {};
  }
};

FrameEncoder::FrameEncoder(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
{
}
FrameEncoder::~FrameEncoder() = default;
FrameEncoder::FrameEncoder(FrameEncoder &&) noexcept = default;
FrameEncoder & FrameEncoder::operator=(FrameEncoder &&) noexcept = default;

EncodeFrameResult FrameEncoder::encode(
  std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format)
{
  Impl & im = *impl_;
  if (auto e = im.uploader->upload_packed(im.frame, pixels, stride, format); !e.empty()) {
    EncodeFrameResult result;
    result.error = std::move(e);
    return result;
  }
  return im.submit();
}

EncodeFrameResult FrameEncoder::encode_yuv420(const Yuv420Planes & planes)
{
  Impl & im = *impl_;
  if (auto e = im.uploader->upload_yuv420(im.frame, planes); !e.empty()) {
    EncodeFrameResult result;
    result.error = std::move(e);
    return result;
  }
  return im.submit();
}

namespace
{

std::string validate_encoder_request(
  std::uint32_t width, std::uint32_t height, int fps_num, int fps_den,
  const FrameEncoderOptions & options)
{
  if (width == 0 || height == 0) {
    return "image has zero width or height";
  }
  if (
    width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return "image dimensions exceed the supported maximum";
  }
  if ((width % 2U) != 0U || (height % 2U) != 0U) {
    return "4:2:0 video needs even dimensions, but the image is " + std::to_string(width) + "x" +
           std::to_string(height);
  }
  if (fps_num <= 0 || fps_den <= 0) {
    return "invalid frame rate";
  }
  if (std::find(kH264Presets.begin(), kH264Presets.end(), options.preset) == kH264Presets.end()) {
    return "unknown preset '" + options.preset + "'";
  }
  if (options.crf >= 0 && options.crf > kFrameCrfMax) {
    return "crf " + std::to_string(options.crf) + " is outside " + std::to_string(kFrameCrfMin) +
           ".." + std::to_string(kFrameCrfMax);
  }
  if (options.gop < 1) {
    return "gop must be at least 1 frame";
  }
  return {};
}

}  // namespace

OpenFrameEncoderResult open_frame_encoder(
  std::uint32_t width, std::uint32_t height, int fps_num, int fps_den,
  const FrameEncoderOptions & options)
{
  OpenFrameEncoderResult result;
  if (auto e = validate_encoder_request(width, height, fps_num, fps_den, options); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  const int crf = options.crf < 0 ? default_frame_crf(options.codec) : options.crf;

  auto im = std::make_unique<FrameEncoder::Impl>();
  im->video_codec = options.codec;
  const int frame_w = static_cast<int>(width);
  const int frame_h = static_cast<int>(height);
  im->uploader = std::make_unique<detail::FrameUploader>(
    frame_w, frame_h, AV_PIX_FMT_YUV420P, options.full_range);

  // Open the codec context for one encoder. Returns "" on success; on
  // failure the context is released so the next candidate can try.
  auto open_codec = [&](const AVCodec * encoder) -> std::string {
    im->codec = avcodec_alloc_context3(encoder);
    if (im->codec == nullptr) {
      return "could not allocate codec context";
    }
    im->codec->width = frame_w;
    im->codec->height = frame_h;
    im->codec->pix_fmt = AV_PIX_FMT_YUV420P;
    im->codec->time_base = AVRational{fps_den, fps_num};  // seconds per frame
    im->codec->framerate = AVRational{fps_num, fps_den};
    if (options.full_range) {
      im->codec->color_range = AVCOL_RANGE_JPEG;
    }
    AVDictionary * opts = nullptr;
    set_zero_latency_options(im->codec, encoder->name, options, crf, &opts);
    const int open_ret = avcodec_open2(im->codec, encoder, &opts);
    av_dict_free(&opts);
    if (open_ret < 0) {
      avcodec_free_context(&im->codec);
      return "could not open encoder " + std::string(encoder->name) + ": " + av_err(open_ret);
    }
    return "";
  };

  const auto candidates = detail::encoder_candidates(
    codec_id_for(options.codec), policy_for(options.backend), width, height);
  detail::EncoderAttempt attempt = detail::try_encoders(candidates, width, height, open_codec);
  if (attempt.backend.empty()) {
    result.error = std::move(attempt.error);
    return result;
  }
  result.backend = std::move(attempt.backend);
  result.fallback_note = std::move(attempt.fallback_note);

  im->frame = av_frame_alloc();
  if (im->frame == nullptr) {
    result.error = "could not allocate frame";
    return result;
  }
  im->frame->format = AV_PIX_FMT_YUV420P;
  im->frame->width = frame_w;
  im->frame->height = frame_h;
  if (options.full_range) {
    im->frame->color_range = AVCOL_RANGE_JPEG;
  }
  if (int ret = av_frame_get_buffer(im->frame, 0); ret < 0) {
    result.error = "could not allocate frame buffer: " + av_err(ret);
    return result;
  }
  im->pkt = av_packet_alloc();
  if (im->pkt == nullptr) {
    result.error = "could not allocate packet";
    return result;
  }

  result.encoder = std::make_unique<FrameEncoder>(std::move(im));
  return result;
}

// ---------------------------------------------------------------------------
// Decoder

struct FrameDecoder::Impl
{
  AVCodecContext * codec = nullptr;
  AVPacket * pkt = nullptr;
  AVFrame * frame = nullptr;
  SwsContext * sws = nullptr;
  int sws_w = 0;
  int sws_h = 0;
  AVPixelFormat sws_fmt = AV_PIX_FMT_NONE;
  bool sws_full_range = false;
  // Aligned, row-padded BGR24 destination for swscale (its SIMD paths write
  // past a tightly-packed row); the packed raster is copied out of it.
  std::array<std::uint8_t *, 4> bgr_data{};
  std::array<int, 4> bgr_linesize{};
  int bgr_w = 0;
  int bgr_h = 0;
  bool flushed = false;

  Impl() = default;
  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

  ~Impl()
  {
    av_freep(bgr_data.data());
    if (sws != nullptr) {
      sws_freeContext(sws);
    }
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
  }

  // (Re)build the conversion for the decoded frame's geometry, pixel format
  // and range; the stream's own range tag (VUI) decides the source range. A
  // full-range stream comes back in the deprecated YUVJ* formats, which
  // sws_getContext warns about on every call; map them to the plain YUV*
  // format and carry the range separately, as the image decoder does.
  std::string ensure_conversion(const AVFrame & f)
  {
    bool full_range = f.color_range == AVCOL_RANGE_JPEG;
    const AVPixelFormat fmt =
      normalize_pixel_format(static_cast<AVPixelFormat>(f.format), full_range);
    if (
      sws != nullptr && sws_w == f.width && sws_h == f.height && sws_fmt == fmt &&
      sws_full_range == full_range) {
      return {};
    }
    if (sws != nullptr) {
      sws_freeContext(sws);
      sws = nullptr;
    }
    sws = sws_getContext(
      f.width, f.height, fmt, f.width, f.height, AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr,
      nullptr);
    if (sws == nullptr) {
      return "failed to create swscale context for the decoded frame";
    }
    const int * coeffs = sws_getCoefficients(SWS_CS_DEFAULT);
    const int src_range = full_range ? 1 : 0;
    sws_setColorspaceDetails(sws, coeffs, src_range, coeffs, 1, 0, 1 << 16, 1 << 16);
    sws_w = f.width;
    sws_h = f.height;
    sws_fmt = fmt;
    sws_full_range = full_range;

    if (bgr_w != f.width || bgr_h != f.height) {
      av_freep(bgr_data.data());
      const int ret = av_image_alloc(
        bgr_data.data(), bgr_linesize.data(), f.width, f.height, AV_PIX_FMT_BGR24, 32);
      if (ret < 0) {
        bgr_w = 0;
        bgr_h = 0;
        return "could not allocate the BGR destination buffer: " + av_err(ret);
      }
      bgr_w = f.width;
      bgr_h = f.height;
    }
    return {};
  }

  std::string convert(const AVFrame & f, DecodedFrame & out)
  {
    if (auto e = ensure_conversion(f); !e.empty()) {
      return e;
    }
    sws_scale(sws, f.data, f.linesize, 0, f.height, bgr_data.data(), bgr_linesize.data());
    out.width = static_cast<std::uint32_t>(f.width);
    out.height = static_cast<std::uint32_t>(f.height);
    const std::size_t row_bytes = static_cast<std::size_t>(f.width) * 3U;
    out.bgr.resize(row_bytes * static_cast<std::size_t>(f.height));
    for (int y = 0; y < f.height; ++y) {
      std::memcpy(
        out.bgr.data() + static_cast<std::size_t>(y) * row_bytes,
        bgr_data[0] + static_cast<std::ptrdiff_t>(y) * bgr_linesize[0], row_bytes);
    }
    return {};
  }
};

FrameDecoder::FrameDecoder(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
{
}
FrameDecoder::~FrameDecoder() = default;
FrameDecoder::FrameDecoder(FrameDecoder &&) noexcept = default;
FrameDecoder & FrameDecoder::operator=(FrameDecoder &&) noexcept = default;

std::string FrameDecoder::send(std::span<const std::byte> packet)
{
  Impl & im = *impl_;
  if (im.flushed) {
    return "decoder accepts no packets after flush()";
  }
  if (packet.empty()) {
    return "empty packet";
  }
  if (packet.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return "packet of " + std::to_string(packet.size()) + " bytes exceeds the supported maximum";
  }
  if (int ret = av_new_packet(im.pkt, static_cast<int>(packet.size())); ret < 0) {
    return "could not allocate packet: " + av_err(ret);
  }
  std::memcpy(im.pkt->data, packet.data(), packet.size());
  const int ret = avcodec_send_packet(im.codec, im.pkt);
  av_packet_unref(im.pkt);
  if (ret == AVERROR(EAGAIN)) {
    return "decoder has frames pending; call receive() until it returns none before send()";
  }
  if (ret < 0) {
    return "decoder rejected the packet: " + av_err(ret);
  }
  return {};
}

ReceiveFrameResult FrameDecoder::receive()
{
  Impl & im = *impl_;
  ReceiveFrameResult result;
  const int ret = avcodec_receive_frame(im.codec, im.frame);
  if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
    return result;
  }
  if (ret < 0) {
    result.error = "decoder receive_frame failed: " + av_err(ret);
    return result;
  }
  DecodedFrame decoded;
  const std::string error = im.convert(*im.frame, decoded);
  av_frame_unref(im.frame);
  if (!error.empty()) {
    result.error = error;
    return result;
  }
  result.frame = std::move(decoded);
  return result;
}

std::string FrameDecoder::flush()
{
  Impl & im = *impl_;
  if (im.flushed) {
    return {};
  }
  im.flushed = true;
  const int ret = avcodec_send_packet(im.codec, nullptr);
  if (ret < 0 && ret != AVERROR_EOF) {
    return "decoder flush failed: " + av_err(ret);
  }
  return {};
}

OpenFrameDecoderResult open_frame_decoder(VideoCodec codec)
{
  OpenFrameDecoderResult result;
  const AVCodecID id = codec_id_for(codec);
  const AVCodec * decoder = avcodec_find_decoder(id);
  if (decoder == nullptr) {
    result.error =
      std::string("decoder not available in this FFmpeg build: ") + avcodec_get_name(id);
    return result;
  }
  auto im = std::make_unique<FrameDecoder::Impl>();
  im->codec = avcodec_alloc_context3(decoder);
  if (im->codec == nullptr) {
    result.error = "could not allocate decoder context";
    return result;
  }
  // Frames must come out as soon as their packet is in: no reorder buffering
  // (the streams carry no B-frames) and no frame-threading pipeline, which
  // would hold each frame back by the thread count. Slice threads still
  // parallelise large frames.
  im->codec->flags |= AV_CODEC_FLAG_LOW_DELAY;
  im->codec->thread_type = FF_THREAD_SLICE;
  if (int ret = avcodec_open2(im->codec, decoder, nullptr); ret < 0) {
    result.error = "could not open decoder " + std::string(decoder->name) + ": " + av_err(ret);
    return result;
  }
  im->pkt = av_packet_alloc();
  im->frame = av_frame_alloc();
  if (im->pkt == nullptr || im->frame == nullptr) {
    result.error = "could not allocate decoder buffers";
    return result;
  }
  result.decoder = std::make_unique<FrameDecoder>(std::move(im));
  return result;
}

}  // namespace bagwiz::core::video
