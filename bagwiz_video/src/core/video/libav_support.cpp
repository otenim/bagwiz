// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "libav_support.hpp"  // NOLINT(build/include_subdir) src-local shared header

extern "C" {
#include <libavutil/error.h>
#include <libavutil/frame.h>
}

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::video::detail
{

std::string av_err(int errnum)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buf{};
  av_strerror(errnum, buf.data(), buf.size());
  return std::string(buf.data());
}

std::vector<const char *> encoder_candidates(
  AVCodecID codec, BackendPolicy policy, std::uint32_t width, std::uint32_t height)
{
  const char * cpu = codec == AV_CODEC_ID_HEVC ? kX265Name : kX264Name;
  const char * nvenc = codec == AV_CODEC_ID_HEVC ? kHevcNvencName : kH264NvencName;
  switch (policy) {
    case BackendPolicy::kCpuOnly:
      return {cpu};
    case BackendPolicy::kNvencOnly:
      return {nvenc};
    case BackendPolicy::kAuto:
    default:
      if (static_cast<std::uint64_t>(width) * height > kNvencAutoMinPixels) {
        return {nvenc, cpu};
      }
      return {cpu};
  }
}

// cppcheck-suppress passedByValue
// std::string_view is the lightweight view type and is idiomatically passed
// by value; cppcheck's heuristic flags all view-by-value uses.
bool is_nvenc_encoder(std::string_view name)
{
  return name == kH264NvencName || name == kHevcNvencName;
}

// cppcheck-suppress passedByValue
// std::string_view is the lightweight view type and is idiomatically passed
// by value; cppcheck's heuristic flags all view-by-value uses.
const char * nvenc_preset_for(std::string_view x264_preset)
{
  if (x264_preset == "ultrafast") {
    return "p1";
  }
  if (x264_preset == "superfast") {
    return "p2";
  }
  if (x264_preset == "veryfast") {
    return "p3";
  }
  if (x264_preset == "slow") {
    return "p5";
  }
  if (x264_preset == "slower") {
    return "p6";
  }
  if (x264_preset == "veryslow") {
    return "p7";
  }
  return "p4";  // faster, fast, medium
}

EncoderAttempt try_encoders(
  const std::vector<const char *> & candidates, std::uint32_t width, std::uint32_t height,
  const std::function<std::string(const AVCodec *)> & open_codec)
{
  EncoderAttempt attempt;
  std::vector<std::string> failures;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const AVCodec * encoder = avcodec_find_encoder_by_name(candidates[i]);
    std::string failure =
      encoder == nullptr
        ? std::string("encoder not available in this FFmpeg build: ") + candidates[i]
        : open_codec(encoder);
    if (failure.empty()) {
      attempt.backend = candidates[i];
      attempt.error.clear();
      return attempt;
    }
    // NVENC's H.264 encoder refuses frames past 4096x4096 on most GPUs with
    // only a generic error; say so, since a 2x2 grid of 4K cameras hits it.
    if (
      std::string_view{candidates[i]} == kH264NvencName &&
      (width > kNvencMaxH264Side || height > kNvencMaxH264Side)) {
      failure += " (NVENC's H.264 encoder tops out at " + std::to_string(kNvencMaxH264Side) + "x" +
                 std::to_string(kNvencMaxH264Side) + " on most GPUs)";
    }
    failures.push_back(failure);
    // Record why an earlier candidate was passed over, for the caller to log
    // when a later one opens.
    if (i + 1 < candidates.size()) {
      attempt.fallback_note = failure;
    }
  }
  // Every candidate failed: say why each did.
  for (const auto & failure : failures) {
    attempt.error += (attempt.error.empty() ? "" : "; ") + failure;
  }
  attempt.fallback_note.clear();
  return attempt;
}

FrameUploader::FrameUploader(int width, int height, AVPixelFormat pix_fmt, bool full_range)
: width_(width), height_(height), pix_fmt_(pix_fmt), full_range_(full_range)
{
}

FrameUploader::~FrameUploader()
{
  if (sws_ != nullptr) {
    sws_freeContext(sws_);
  }
}

std::string FrameUploader::ensure_sws(SourcePixelFormat format)
{
  if (sws_ != nullptr && sws_src_fmt_ == format) {
    return {};
  }
  if (sws_ != nullptr) {
    sws_freeContext(sws_);
    sws_ = nullptr;
  }
  const AVPixelFormat src =
    (format == SourcePixelFormat::kBgr8) ? AV_PIX_FMT_BGR24 : AV_PIX_FMT_RGB24;
  // SWS_ACCURATE_RND keeps swscale off its same-size unscaled fast path for
  // packed RGB -> YUV420P, which is wired for limited range and ignores the
  // full-range request below (a full-range stream then decodes several
  // levels too bright).
  sws_ = sws_getContext(
    width_, height_, src, width_, height_, pix_fmt_, SWS_BILINEAR | SWS_ACCURATE_RND, nullptr,
    nullptr, nullptr);
  if (sws_ == nullptr) {
    return "failed to create swscale conversion context";
  }
  if (full_range_ && pix_fmt_ == AV_PIX_FMT_YUV420P) {
    // A full-range stream converts RGB to full-range YUV (the default targets
    // limited range), so converted frames match planes handed over straight
    // from a JPEG decoder.
    const int * coefficients = sws_getCoefficients(SWS_CS_ITU601);
    sws_setColorspaceDetails(sws_, coefficients, 1, coefficients, 1, 0, 1 << 16, 1 << 16);
  }
  sws_src_fmt_ = format;
  return {};
}

std::string FrameUploader::upload_packed(
  AVFrame * frame, std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format)
{
  // sws_scale takes int strides; a stride past INT_MAX would narrow to a
  // negative value and corrupt the conversion.
  if (stride > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return "row stride " + std::to_string(stride) + " exceeds the supported maximum";
  }
  const auto need = stride * static_cast<std::size_t>(height_);
  if (pixels.size() < need) {
    return "frame buffer too small: have " + std::to_string(pixels.size()) + " bytes, need " +
           std::to_string(need);
  }
  if (auto e = ensure_sws(format); !e.empty()) {
    return e;
  }
  if (int ret = av_frame_make_writable(frame); ret < 0) {
    return "frame_make_writable failed: " + av_err(ret);
  }
  const auto * src_ptr =
    reinterpret_cast<const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      pixels.data());
  const std::array<const std::uint8_t *, 4> src_data{src_ptr, nullptr, nullptr, nullptr};
  const std::array<int, 4> src_stride{static_cast<int>(stride), 0, 0, 0};
  sws_scale(sws_, src_data.data(), src_stride.data(), 0, height_, frame->data, frame->linesize);
  return {};
}

std::string FrameUploader::upload_yuv420(AVFrame * frame, const Yuv420Planes & planes) const
{
  const auto width = static_cast<std::size_t>(width_);
  const auto height = static_cast<std::size_t>(height_);
  const std::size_t chroma_w = (width + 1) / 2;
  const std::size_t chroma_h = (height + 1) / 2;
  if (planes.y == nullptr || planes.u == nullptr || planes.v == nullptr) {
    return "yuv420 frame is missing a plane";
  }
  if (planes.y_stride < width || planes.u_stride < chroma_w || planes.v_stride < chroma_w) {
    return "yuv420 frame row stride is shorter than the frame width";
  }
  if (int ret = av_frame_make_writable(frame); ret < 0) {
    return "frame_make_writable failed: " + av_err(ret);
  }
  const auto copy_plane = [](
                            const std::uint8_t * src, std::size_t src_stride, std::uint8_t * dst,
                            std::size_t dst_stride, std::size_t row_bytes, std::size_t rows) {
    for (std::size_t r = 0; r < rows; ++r) {
      std::memcpy(dst + r * dst_stride, src + r * src_stride, row_bytes);
    }
  };
  copy_plane(
    planes.y, planes.y_stride, frame->data[0], static_cast<std::size_t>(frame->linesize[0]), width,
    height);
  copy_plane(
    planes.u, planes.u_stride, frame->data[1], static_cast<std::size_t>(frame->linesize[1]),
    chroma_w, chroma_h);
  copy_plane(
    planes.v, planes.v_stride, frame->data[2], static_cast<std::size_t>(frame->linesize[2]),
    chroma_w, chroma_h);
  return {};
}

}  // namespace bagwiz::core::video::detail
