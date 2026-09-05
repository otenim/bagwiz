// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/video_encoder.hpp"

#include "libav_support.hpp"  // NOLINT(build/include_subdir) src-local shared header

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/pixfmt.h>
#include <libavutil/rational.h>
}

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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

// Container + codec selection derived from the output extension.
struct CodecChoice
{
  AVCodecID id = AV_CODEC_ID_NONE;
  AVPixelFormat pix_fmt = AV_PIX_FMT_YUV420P;
  bool jpeg_range = false;    // tag the stream full-range (MJPEG)
  bool requires_even = true;  // 4:2:0 chroma needs even dimensions
};

std::optional<CodecChoice> codec_for_extension(
  const std::filesystem::path & output, std::string & error)
{
  std::string ext = output.extension().string();
  for (auto & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  if (ext == ".mp4" || ext == ".mkv" || ext == ".mov") {
    return CodecChoice{AV_CODEC_ID_H264, AV_PIX_FMT_YUV420P, false, true};
  }
  if (ext == ".avi") {
    return CodecChoice{AV_CODEC_ID_MJPEG, AV_PIX_FMT_YUVJ420P, true, true};
  }
  error = "unsupported output extension '" + ext + "'; use .mp4/.mkv/.mov (H.264) or .avi (MJPEG)";
  return std::nullopt;
}

detail::BackendPolicy policy_for(H264Backend backend)
{
  switch (backend) {
    case H264Backend::kX264:
      return detail::BackendPolicy::kCpuOnly;
    case H264Backend::kNvenc:
      return detail::BackendPolicy::kNvencOnly;
    case H264Backend::kAuto:
    default:
      return detail::BackendPolicy::kAuto;
  }
}

// The encoder-specific options for one H.264 encoder. Both keep B-frames off:
// the default inserts them, which leads to negative DTS in the MP4 container
// and crashes some hardware decoders (notably mpv's Vulkan hwdec path); the
// codec-context field and the option are both set so the request is
// respected regardless of which path the wrapper reads.
void set_h264_options(
  AVCodecContext * codec, const char * encoder_name, const VideoEncoderOptions & options,
  AVDictionary ** opts)
{
  codec->max_b_frames = 0;
  av_dict_set(opts, "bf", "0", 0);
  if (detail::is_nvenc_encoder(encoder_name)) {
    // Constant-quality VBR at the quality libx264's crf 23 lands near; the
    // bitrate is left unset so the quality target drives the rate.
    av_dict_set(opts, "preset", detail::nvenc_preset_for(options.preset), 0);
    av_dict_set(opts, "rc", "vbr", 0);
    av_dict_set(opts, "cq", "23", 0);
    return;
  }
  av_dict_set(opts, "preset", options.preset.c_str(), 0);
  av_dict_set(opts, "crf", "23", 0);
  // Also disable scenecut: with B-frames off and a short GOP, libx264 tends
  // to mark every frame as an I-frame, blowing up the file size for no gain.
  av_dict_set(opts, "sc_threshold", "0", 0);
}

}  // namespace

struct VideoEncoder::Impl
{
  AVFormatContext * fmt = nullptr;
  AVCodecContext * codec = nullptr;
  AVStream * stream = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;
  std::unique_ptr<detail::FrameUploader> uploader;
  bool finished = false;
  std::int64_t next_pts = 0;

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
    if (fmt != nullptr) {
      if (fmt->pb != nullptr && (fmt->oformat->flags & AVFMT_NOFILE) == 0) {
        avio_closep(&fmt->pb);
      }
      avformat_free_context(fmt);
    }
  }

  // Send one frame (or nullptr to flush) and write every packet the encoder
  // returns. Empty string on success.
  std::string drain(AVFrame * f)
  {
    int ret = avcodec_send_frame(codec, f);
    if (ret < 0) {
      av_packet_unref(pkt);  // drop any residual data from a prior partial drain
      return "encoder send_frame failed: " + av_err(ret);
    }
    while (true) {
      ret = avcodec_receive_packet(codec, pkt);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      }
      if (ret < 0) {
        return "encoder receive_packet failed: " + av_err(ret);
      }
      av_packet_rescale_ts(pkt, codec->time_base, stream->time_base);
      pkt->stream_index = stream->index;
      ret = av_interleaved_write_frame(fmt, pkt);
      av_packet_unref(pkt);
      if (ret < 0) {
        return "muxer write_frame failed: " + av_err(ret);
      }
    }
    return {};
  }
};

VideoEncoder::VideoEncoder(std::unique_ptr<Impl> impl) : impl_(std::move(impl))
{
}
VideoEncoder::~VideoEncoder() = default;
VideoEncoder::VideoEncoder(VideoEncoder &&) noexcept = default;
VideoEncoder & VideoEncoder::operator=(VideoEncoder &&) noexcept = default;

std::string VideoEncoder::write_frame(
  std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format)
{
  Impl & im = *impl_;
  if (auto e = im.uploader->upload_packed(im.frame, pixels, stride, format); !e.empty()) {
    return e;
  }
  im.frame->pts = im.next_pts++;
  return im.drain(im.frame);
}

std::string VideoEncoder::write_yuv420(const Yuv420Planes & planes)
{
  Impl & im = *impl_;
  if (auto e = im.uploader->upload_yuv420(im.frame, planes); !e.empty()) {
    return e;
  }
  im.frame->pts = im.next_pts++;
  return im.drain(im.frame);
}

std::string VideoEncoder::finish()
{
  Impl & im = *impl_;
  if (im.finished) {
    return {};
  }
  im.finished = true;
  if (auto e = im.drain(nullptr); !e.empty()) {  // flush the encoder's delay queue
    return e;
  }
  const int ret = av_write_trailer(im.fmt);
  if (ret < 0) {
    return "write_trailer failed: " + av_err(ret);
  }
  return {};
}

OpenVideoEncoderResult open_video_encoder(
  const std::filesystem::path & output, std::uint32_t width, std::uint32_t height, int fps_num,
  int fps_den, const VideoEncoderOptions & options)
{
  OpenVideoEncoderResult result;

  std::string ext_error;
  const auto choice = codec_for_extension(output, ext_error);
  if (!choice.has_value()) {
    result.error = ext_error;
    return result;
  }
  if (width == 0 || height == 0) {
    result.error = "image has zero width or height";
    return result;
  }
  if (
    width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
    height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    result.error = "image dimensions exceed the supported maximum";
    return result;
  }
  if (choice->requires_even && ((width % 2U) != 0U || (height % 2U) != 0U)) {
    result.error = "this codec needs even dimensions, but the image is " + std::to_string(width) +
                   "x" + std::to_string(height);
    return result;
  }
  if (fps_num <= 0 || fps_den <= 0) {
    result.error = "invalid frame rate";
    return result;
  }

  if (
    choice->id == AV_CODEC_ID_H264 &&
    std::find(kH264Presets.begin(), kH264Presets.end(), options.preset) == kH264Presets.end()) {
    result.error = "unknown H.264 preset '" + options.preset + "'";
    return result;
  }

  auto im = std::make_unique<VideoEncoder::Impl>();
  const int frame_w = static_cast<int>(width);
  const int frame_h = static_cast<int>(height);
  im->uploader =
    std::make_unique<detail::FrameUploader>(frame_w, frame_h, choice->pix_fmt, options.full_range);

  const std::string path_str = output.string();

  int ret = avformat_alloc_output_context2(&im->fmt, nullptr, nullptr, path_str.c_str());
  if (ret < 0 || im->fmt == nullptr) {
    result.error = "could not allocate output context: " + av_err(ret);
    return result;
  }

  im->stream = avformat_new_stream(im->fmt, nullptr);
  if (im->stream == nullptr) {
    result.error = "could not create output stream";
    return result;
  }

  // Open the codec context for one encoder. Returns "" on success; on
  // failure the context is released so the next candidate can try.
  auto open_codec = [&](const AVCodec * encoder) -> std::string {
    im->codec = avcodec_alloc_context3(encoder);
    if (im->codec == nullptr) {
      return "could not allocate codec context";
    }
    im->codec->width = frame_w;
    im->codec->height = frame_h;
    im->codec->pix_fmt = choice->pix_fmt;
    im->codec->time_base = AVRational{fps_den, fps_num};  // seconds per frame
    im->codec->framerate = AVRational{fps_num, fps_den};
    im->codec->gop_size = 12;
    if (choice->jpeg_range || options.full_range) {
      im->codec->color_range = AVCOL_RANGE_JPEG;
    }
    if ((im->fmt->oformat->flags & AVFMT_GLOBALHEADER) != 0) {
      im->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }
    AVDictionary * opts = nullptr;
    if (choice->id == AV_CODEC_ID_H264) {
      set_h264_options(im->codec, encoder->name, options, &opts);
    }
    const int open_ret = avcodec_open2(im->codec, encoder, &opts);
    av_dict_free(&opts);
    if (open_ret < 0) {
      avcodec_free_context(&im->codec);
      return "could not open encoder " + std::string(encoder->name) + ": " + av_err(open_ret);
    }
    return "";
  };

  if (choice->id == AV_CODEC_ID_H264) {
    const auto candidates =
      detail::encoder_candidates(AV_CODEC_ID_H264, policy_for(options.backend), width, height);
    detail::EncoderAttempt attempt = detail::try_encoders(candidates, width, height, open_codec);
    if (attempt.backend.empty()) {
      result.error = attempt.error + " (try an .avi output, which uses the built-in MJPEG encoder)";
      return result;
    }
    result.backend = std::move(attempt.backend);
    result.fallback_note = std::move(attempt.fallback_note);
  } else {
    const AVCodec * encoder = avcodec_find_encoder(choice->id);
    if (encoder == nullptr) {
      result.error =
        std::string("encoder not available in this FFmpeg build: ") + avcodec_get_name(choice->id);
      return result;
    }
    if (const auto err = open_codec(encoder); !err.empty()) {
      result.error = err;
      return result;
    }
    result.backend = encoder->name;
  }
  im->stream->time_base = im->codec->time_base;

  ret = avcodec_parameters_from_context(im->stream->codecpar, im->codec);
  if (ret < 0) {
    result.error = "could not copy codec parameters: " + av_err(ret);
    return result;
  }

  if ((im->fmt->oformat->flags & AVFMT_NOFILE) == 0) {
    ret = avio_open(&im->fmt->pb, path_str.c_str(), AVIO_FLAG_WRITE);
    if (ret < 0) {
      result.error = "could not open '" + path_str + "' for writing: " + av_err(ret);
      return result;
    }
  }

  ret = avformat_write_header(im->fmt, nullptr);
  if (ret < 0) {
    result.error = "could not write file header: " + av_err(ret);
    return result;
  }

  im->frame = av_frame_alloc();
  if (im->frame == nullptr) {
    result.error = "could not allocate frame";
    return result;
  }
  im->frame->format = choice->pix_fmt;
  im->frame->width = frame_w;
  im->frame->height = frame_h;
  ret = av_frame_get_buffer(im->frame, 0);
  if (ret < 0) {
    result.error = "could not allocate frame buffer: " + av_err(ret);
    return result;
  }

  im->pkt = av_packet_alloc();
  if (im->pkt == nullptr) {
    result.error = "could not allocate packet";
    return result;
  }

  result.encoder = std::make_unique<VideoEncoder>(std::move(im));
  return result;
}

VideoProbe probe_video(const std::filesystem::path & path)
{
  VideoProbe probe;
  const std::string path_str = path.string();

  AVFormatContext * fmt = nullptr;
  int ret = avformat_open_input(&fmt, path_str.c_str(), nullptr, nullptr);
  if (ret < 0) {
    probe.error = "could not open '" + path_str + "': " + av_err(ret);
    return probe;
  }
  ret = avformat_find_stream_info(fmt, nullptr);
  if (ret < 0) {
    probe.error = "could not read stream info: " + av_err(ret);
    avformat_close_input(&fmt);
    return probe;
  }
  const int vs = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (vs < 0) {
    probe.error = "no video stream found";
    avformat_close_input(&fmt);
    return probe;
  }
  AVStream * st = fmt->streams[vs];
  probe.width = static_cast<std::uint32_t>(st->codecpar->width);
  probe.height = static_cast<std::uint32_t>(st->codecpar->height);
  probe.codec = avcodec_get_name(st->codecpar->codec_id);
  probe.has_b_frames = st->codecpar->video_delay > 0;

  std::int64_t count = 0;
  AVPacket * pkt = av_packet_alloc();
  if (pkt != nullptr) {
    while (av_read_frame(fmt, pkt) >= 0) {
      if (pkt->stream_index == vs) {
        ++count;
      }
      av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
  }
  probe.frame_count = count;

  if (st->duration != AV_NOPTS_VALUE && st->duration > 0) {
    probe.duration_s = static_cast<double>(st->duration) * av_q2d(st->time_base);
  } else if (fmt->duration != AV_NOPTS_VALUE && fmt->duration > 0) {
    probe.duration_s = static_cast<double>(fmt->duration) / AV_TIME_BASE;
  }

  avformat_close_input(&fmt);
  return probe;
}

}  // namespace bagwiz::core::video
