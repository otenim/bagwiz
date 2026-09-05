// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_encoder.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::image
{
namespace
{

std::string av_err(int errnum)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buf{};
  av_strerror(errnum, buf.data(), buf.size());
  return std::string(buf.data());
}

struct EncodeContext
{
  AVCodecContext * codec = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;

  EncodeContext() = default;
  EncodeContext(const EncodeContext &) = delete;
  EncodeContext & operator=(const EncodeContext &) = delete;
  EncodeContext(EncodeContext &&) = delete;
  EncodeContext & operator=(EncodeContext &&) = delete;

  ~EncodeContext()
  {
    if (pkt != nullptr) {
      av_packet_free(&pkt);
    }
    if (frame != nullptr) {
      av_frame_free(&frame);
    }
    if (codec != nullptr) {
      avcodec_free_context(&codec);
    }
  }
};

// Shape checks shared by every still-image encoder. Returns "" when the
// raster can be encoded.
std::string validate_raster(const PackedRaster & raster)
{
  if (raster.empty()) {
    return "cannot encode an empty raster";
  }
  // Bound the dimensions before multiplying them out, so the width * 3 * height
  // size check below cannot overflow size_t (libav's API is int-typed anyway).
  const auto int_max = static_cast<std::uint32_t>(std::numeric_limits<int>::max());
  if (raster.width > int_max || raster.height > int_max) {
    return "raster dimensions exceed the supported maximum";
  }
  const std::size_t expected = static_cast<std::size_t>(raster.width) * 3U * raster.height;
  if (raster.bgr.size() != expected) {
    return "raster pixel buffer size does not match width * 3 * height";
  }
  return {};
}

// Allocate the codec context for `id` with the raster's geometry and `pix_fmt`,
// leaving codec-specific options to the caller before avcodec_open2.
std::string alloc_codec(
  EncodeContext & ctx, AVCodecID id, const PackedRaster & raster, AVPixelFormat pix_fmt)
{
  const AVCodec * encoder = avcodec_find_encoder(id);
  if (encoder == nullptr) {
    return std::string(avcodec_get_name(id)) + " encoder not available in this FFmpeg build";
  }
  ctx.codec = avcodec_alloc_context3(encoder);
  if (ctx.codec == nullptr) {
    return "could not allocate encoder context";
  }
  ctx.codec->width = static_cast<int>(raster.width);
  ctx.codec->height = static_cast<int>(raster.height);
  ctx.codec->pix_fmt = pix_fmt;
  // A still image is a single intra frame; the time base is unused but libav
  // requires a non-zero value.
  ctx.codec->time_base = AVRational{1, 1};
  return {};
}

std::string open_codec(EncodeContext & ctx)
{
  if (int ret = avcodec_open2(ctx.codec, ctx.codec->codec, nullptr); ret < 0) {
    return "could not open encoder: " + av_err(ret);
  }
  return {};
}

// Allocate a writable frame of the codec's geometry and pixel format.
// av_frame_get_buffer aligns each row, so frame->linesize[0] may exceed the
// packed row width — fill through that stride rather than as one block.
std::string alloc_frame(EncodeContext & ctx)
{
  ctx.frame = av_frame_alloc();
  if (ctx.frame == nullptr) {
    return "could not allocate frame";
  }
  ctx.frame->format = ctx.codec->pix_fmt;
  ctx.frame->width = ctx.codec->width;
  ctx.frame->height = ctx.codec->height;
  if (int ret = av_frame_get_buffer(ctx.frame, 0); ret < 0) {
    return "could not allocate frame buffer: " + av_err(ret);
  }
  if (int ret = av_frame_make_writable(ctx.frame); ret < 0) {
    return "frame buffer is not writable: " + av_err(ret);
  }
  ctx.frame->pts = 0;
  return {};
}

// Copy the packed BGR24 source into an RGB24 frame, swapping B and R per
// pixel. A manual swap keeps the source side free of swscale's SIMD over-read
// on tightly-packed rows (the decoder pads its swscale destination for the
// same reason); a pure channel reorder needs no colorspace math.
void fill_rgb24(const AVFrame & frame, const PackedRaster & raster)
{
  const int width = static_cast<int>(raster.width);
  const int height = static_cast<int>(raster.height);
  const std::size_t src_stride = static_cast<std::size_t>(width) * 3U;
  const auto * src =
    reinterpret_cast<const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      raster.bgr.data());
  for (int y = 0; y < height; ++y) {
    const std::uint8_t * src_row = src + static_cast<std::size_t>(y) * src_stride;
    std::uint8_t * dst_row = frame.data[0] + static_cast<std::ptrdiff_t>(y) * frame.linesize[0];
    for (int x = 0; x < width; ++x) {
      const std::uint8_t * sp = src_row + static_cast<std::size_t>(x) * 3U;
      std::uint8_t * dp = dst_row + static_cast<std::size_t>(x) * 3U;
      dp[0] = sp[2];  // R <- src red
      dp[1] = sp[1];  // G <- src green
      dp[2] = sp[0];  // B <- src blue
    }
  }
}

// Owns an aligned, row-padded BGR24 copy of the raster for swscale to read
// from: its SIMD paths can read a few bytes past a tightly-packed final row.
struct PaddedSource
{
  std::array<std::uint8_t *, 4> data{};
  std::array<int, 4> linesize{};

  PaddedSource() = default;
  PaddedSource(const PaddedSource &) = delete;
  PaddedSource & operator=(const PaddedSource &) = delete;
  PaddedSource(PaddedSource &&) = delete;
  PaddedSource & operator=(PaddedSource &&) = delete;
  ~PaddedSource() { av_freep(data.data()); }

  std::string fill(const PackedRaster & raster)
  {
    const int width = static_cast<int>(raster.width);
    const int height = static_cast<int>(raster.height);
    if (const int ret =
          av_image_alloc(data.data(), linesize.data(), width, height, AV_PIX_FMT_BGR24, 32);
        ret < 0) {
      return "could not allocate the conversion source buffer: " + av_err(ret);
    }
    const std::size_t row_bytes = static_cast<std::size_t>(width) * 3U;
    const auto * src = reinterpret_cast<
      const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      raster.bgr.data());
    for (int y = 0; y < height; ++y) {
      std::memcpy(
        data[0] + static_cast<std::ptrdiff_t>(y) * linesize[0],
        src + static_cast<std::size_t>(y) * row_bytes, row_bytes);
    }
    return {};
  }
};

// Convert the BGR24 raster into the frame's full-range YUV 4:2:0 planes.
std::string fill_yuv420_full_range(AVFrame & frame, const PackedRaster & raster)
{
  PaddedSource source;
  if (auto e = source.fill(raster); !e.empty()) {
    return e;
  }
  // SWS_ACCURATE_RND matters here: without it swscale routes a same-size
  // BGR24 -> YUV420P conversion through its unscaled fast path, which is
  // wired for limited range and ignores the full-range request below (the
  // output then decodes ~8 levels too bright on a mid-grey ramp).
  SwsContext * sws = sws_getContext(
    frame.width, frame.height, AV_PIX_FMT_BGR24, frame.width, frame.height, AV_PIX_FMT_YUV420P,
    SWS_BILINEAR | SWS_ACCURATE_RND, nullptr, nullptr, nullptr);
  if (sws == nullptr) {
    return "failed to create swscale conversion context";
  }
  // Full-range (JPEG-level) YUV: swscale defaults to the limited MPEG range,
  // which a JPEG decoder would then stretch, washing the image out.
  const int * coefficients = sws_getCoefficients(SWS_CS_ITU601);
  sws_setColorspaceDetails(sws, coefficients, 1, coefficients, 1, 0, 1 << 16, 1 << 16);
  sws_scale(
    sws, source.data.data(), source.linesize.data(), 0, frame.height, frame.data, frame.linesize);
  sws_freeContext(sws);
  return {};
}

// Encode the single frame held in `ctx` and gather its packet bytes.
std::string encode_still(EncodeContext & ctx, std::vector<std::byte> & out)
{
  ctx.pkt = av_packet_alloc();
  if (ctx.pkt == nullptr) {
    return "could not allocate packet";
  }
  if (int ret = avcodec_send_frame(ctx.codec, ctx.frame); ret < 0) {
    return "encoder send_frame failed: " + av_err(ret);
  }
  // Signal end-of-stream so the single still frame is flushed out as a packet.
  // A successful flush returns 0; AVERROR_EOF would only mean the encoder was
  // already drained (impossible after a single send), so accept it defensively
  // and surface any other negative code as a real error.
  if (int ret = avcodec_send_frame(ctx.codec, nullptr); ret < 0 && ret != AVERROR_EOF) {
    return "encoder flush failed: " + av_err(ret);
  }
  while (true) {
    const int ret = avcodec_receive_packet(ctx.codec, ctx.pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    }
    if (ret < 0) {
      return "encoder receive_packet failed: " + av_err(ret);
    }
    const auto * data =
      reinterpret_cast<const std::byte *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ctx.pkt->data);
    out.insert(out.end(), data, data + static_cast<std::size_t>(ctx.pkt->size));
    av_packet_unref(ctx.pkt);
  }
  if (out.empty()) {
    return "encoder produced no output";
  }
  return {};
}

// The MJPEG encoder's quantizer scale (1 best .. 31 smallest) standing in for
// a libjpeg-style quality: 100 -> 1, 90 -> 4, 75 -> 9, 50 -> 16, 1 -> 31.
int jpeg_qscale_for(int quality)
{
  const double span = static_cast<double>(kJpegQualityMax - kJpegQualityMin);
  const double q = 31.0 - static_cast<double>(quality - kJpegQualityMin) * 30.0 / span;
  return static_cast<int>(q + 0.5);
}

}  // namespace

EncodePngResult encode_png(const PackedRaster & raster)
{
  EncodePngResult result;
  if (auto e = validate_raster(raster); !e.empty()) {
    result.error = std::move(e);
    return result;
  }

  EncodeContext ctx;
  if (auto e = alloc_codec(ctx, AV_CODEC_ID_PNG, raster, AV_PIX_FMT_RGB24); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  // libav defaults to `pred=none`, which hands deflate the raw rows and leaves
  // most of PNG's compression on the table — on photographic frames that is
  // ~1.55x versus ~2.1x, and on smooth gradients the unfiltered output is
  // larger than the input. `mixed` picks a filter per row. Level 1 rather than
  // the default 6 because the filtering does most of the work: measured on a
  // 4K camera frame, level 1 is both smaller and faster than the old default
  // (9.8 MB / 411 ms against 14.4 MB / 657 ms), while level 6 costs 3x the CPU
  // for a further 10%. Both callers — walk's `S` save and its remote preview
  // transfer — want small and fast, so neither setting is parameterized.
  ctx.codec->compression_level = 1;
  if (int ret = av_opt_set(ctx.codec->priv_data, "pred", "mixed", 0); ret < 0) {
    result.error = "could not enable PNG prediction filtering: " + av_err(ret);
    return result;
  }
  if (auto e = open_codec(ctx); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  if (auto e = alloc_frame(ctx); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  fill_rgb24(*ctx.frame, raster);

  std::vector<std::byte> png;
  if (auto e = encode_still(ctx, png); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  result.png = std::move(png);
  return result;
}

EncodeJpegResult encode_jpeg(const PackedRaster & raster, int quality)
{
  EncodeJpegResult result;
  if (quality < kJpegQualityMin || quality > kJpegQualityMax) {
    result.error = "JPEG quality " + std::to_string(quality) + " is outside " +
                   std::to_string(kJpegQualityMin) + ".." + std::to_string(kJpegQualityMax);
    return result;
  }
  if (auto e = validate_raster(raster); !e.empty()) {
    result.error = std::move(e);
    return result;
  }

  EncodeContext ctx;
  if (auto e = alloc_codec(ctx, AV_CODEC_ID_MJPEG, raster, AV_PIX_FMT_YUV420P); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  // Plain YUV420P tagged full-range rather than the deprecated YUVJ420P, which
  // makes every swscale call log a deprecation warning. The MJPEG encoder
  // accepts the pair without relaxing strictness.
  ctx.codec->color_range = AVCOL_RANGE_JPEG;
  // Constant quantizer: the still image has no rate to control.
  ctx.codec->flags |= AV_CODEC_FLAG_QSCALE;
  ctx.codec->global_quality = jpeg_qscale_for(quality) * FF_QP2LAMBDA;
  if (auto e = open_codec(ctx); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  if (auto e = alloc_frame(ctx); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  ctx.frame->color_range = AVCOL_RANGE_JPEG;
  if (auto e = fill_yuv420_full_range(*ctx.frame, raster); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  // The quantizer also rides on the frame under AV_CODEC_FLAG_QSCALE.
  ctx.frame->quality = ctx.codec->global_quality;

  std::vector<std::byte> jpeg;
  if (auto e = encode_still(ctx, jpeg); !e.empty()) {
    result.error = std::move(e);
    return result;
  }
  result.jpeg = std::move(jpeg);
  return result;
}

}  // namespace bagwiz::core::image
