// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__VIDEO__LIBAV_SUPPORT_HPP_
#define CORE__VIDEO__LIBAV_SUPPORT_HPP_

#include "bagwiz/core/video/pixel_source.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Private to bagwiz_video: the libav pieces the muxing file encoder
// (video_encoder.cpp) and the per-frame codec (frame_codec.cpp) share —
// error text, encoder selection, and source-frame upload. Not installed.
namespace bagwiz::core::video::detail
{

std::string av_err(int errnum);

constexpr const char * kX264Name = "libx264";
constexpr const char * kX265Name = "libx265";
constexpr const char * kH264NvencName = "h264_nvenc";
constexpr const char * kHevcNvencName = "hevc_nvenc";

// The largest frame side NVENC's H.264 encoder takes on most GPUs.
constexpr std::uint32_t kNvencMaxH264Side = 4096;
// Automatic selection reaches for NVENC only above this many pixels per frame
// (1080p): the GPU's setup and per-frame submission cost more than the CPU
// encoder saves on smaller frames, and the CPU encoder's cost grows with the
// frame.
constexpr std::uint64_t kNvencAutoMinPixels = 1920ULL * 1080ULL;

// Which encoders a caller allows for a codec.
enum class BackendPolicy { kAuto, kCpuOnly, kNvencOnly };

// The encoders to try, in order, for `codec` (AV_CODEC_ID_H264 or
// AV_CODEC_ID_HEVC) under `policy` at the given frame size.
std::vector<const char *> encoder_candidates(
  AVCodecID codec, BackendPolicy policy, std::uint32_t width, std::uint32_t height);

bool is_nvenc_encoder(std::string_view name);

// The NVENC preset (p1 fastest .. p7 slowest) standing in for a libx264 one.
const char * nvenc_preset_for(std::string_view x264_preset);

// Outcome of try_encoders(): `backend` names the encoder that opened, or is
// empty with `error` listing why every candidate failed; `fallback_note`
// records why an earlier candidate was passed over when a later one opened.
struct EncoderAttempt
{
  std::string backend;
  std::string error;
  std::string fallback_note;
};

// Try `open_codec` on each candidate in turn until one opens. `open_codec`
// returns "" on success or an error and must leave no state behind on
// failure so the next candidate can try.
EncoderAttempt try_encoders(
  const std::vector<const char *> & candidates, std::uint32_t width, std::uint32_t height,
  const std::function<std::string(const AVCodec *)> & open_codec);

// Uploads caller-provided pixels into a libav frame of fixed geometry: packed
// BGR/RGB rasters through swscale (into the frame's pixel format, in the
// requested range), or 4:2:0 planes copied as they are. Owns the swscale
// context, rebuilt only when the source layout changes.
class FrameUploader
{
public:
  FrameUploader(int width, int height, AVPixelFormat pix_fmt, bool full_range);
  ~FrameUploader();

  FrameUploader(const FrameUploader &) = delete;
  FrameUploader & operator=(const FrameUploader &) = delete;
  FrameUploader(FrameUploader &&) = delete;
  FrameUploader & operator=(FrameUploader &&) = delete;

  // Convert `pixels` (height rows of `stride` bytes) into `frame`. Returns ""
  // on success or an error.
  std::string upload_packed(
    AVFrame * frame, std::span<const std::byte> pixels, std::size_t stride,
    SourcePixelFormat format);

  // Copy 4:2:0 planes of the frame's geometry into `frame`, no conversion.
  std::string upload_yuv420(AVFrame * frame, const Yuv420Planes & planes) const;

private:
  std::string ensure_sws(SourcePixelFormat format);

  int width_;
  int height_;
  AVPixelFormat pix_fmt_;
  bool full_range_;
  SwsContext * sws_ = nullptr;
  SourcePixelFormat sws_src_fmt_ = SourcePixelFormat::kBgr8;
};

}  // namespace bagwiz::core::video::detail

#endif  // CORE__VIDEO__LIBAV_SUPPORT_HPP_
