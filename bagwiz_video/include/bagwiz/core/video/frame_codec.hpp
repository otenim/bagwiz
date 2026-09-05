// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__FRAME_CODEC_HPP_
#define BAGWIZ__CORE__VIDEO__FRAME_CODEC_HPP_

#include "bagwiz/core/video/pixel_source.hpp"
#include "bagwiz/core/video/video_codec.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

// Per-frame video encoding and decoding without a container: the encoder
// turns one source frame into one Annex B packet, the decoder turns one such
// packet back into one raster. This is the shape a bag topic of
// foxglove_msgs/msg/CompressedVideo needs (one message per frame, decodable
// on its own from the last keyframe), as opposed to the muxed file
// VideoEncoder writes. libav stays behind a pimpl, as in video_encoder.hpp.
namespace bagwiz::core::video
{

// Which encoder implementation to use. kAuto tries NVIDIA NVENC first for
// frames larger than 1080p — when this FFmpeg build has it and a GPU accepts
// the stream — and otherwise the CPU encoder (libx264 / libx265); kCpu and
// kNvenc force one.
enum class EncoderBackend { kAuto, kCpu, kNvenc };

// Bounds of FrameEncoderOptions::crf. libx264 and libx265 accept 0..51; the
// NVENC constant-quality knob spans the same range.
inline constexpr int kFrameCrfMin = 0;
inline constexpr int kFrameCrfMax = 51;

// The crf each codec uses when FrameEncoderOptions::crf is left negative:
// libx264's default 23, and 28 for libx265, whose scale sits about five
// points lower for a similar picture.
[[nodiscard]] int default_frame_crf(VideoCodec codec) noexcept;

struct FrameEncoderOptions
{
  VideoCodec codec = VideoCodec::kH264;
  EncoderBackend backend = EncoderBackend::kAuto;
  // One of kH264Presets (video_encoder.hpp); libx265 takes the same names and
  // NVENC maps them onto its p1 (fastest) .. p7 (slowest).
  std::string preset = "medium";
  // Constant-quality target; negative selects default_frame_crf(codec).
  int crf = -1;
  // Keyframe interval in frames (every gop-th frame is a keyframe, which is
  // also where a player can seek to). Must be at least 1.
  int gop = 30;
  // Encoder thread count; 0 leaves the encoder's own default.
  int threads = 0;
  // Tag the stream full-range (JPEG levels, 0-255) and convert encode() input
  // to that range, so frames handed over as decoded JPEG planes
  // (encode_yuv420) and converted frames share one range.
  bool full_range = false;
};

// One encoded frame. `data` borrows the encoder's packet buffer and is valid
// until the next encode call on that encoder.
struct EncodedFrame
{
  std::span<const std::byte> data;
  bool keyframe = false;
};

struct EncodeFrameResult
{
  std::optional<EncodedFrame> frame;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return frame.has_value() && error.empty(); }
};

// Encodes frames one at a time with no encoder delay: every call returns the
// packet for the frame it was given, so a caller can pair each packet with
// its source message. Keyframe packets carry their parameter sets in-band.
class FrameEncoder
{
public:
  struct Impl;

  explicit FrameEncoder(std::unique_ptr<Impl> impl);
  ~FrameEncoder();

  FrameEncoder(FrameEncoder &&) noexcept;
  FrameEncoder & operator=(FrameEncoder &&) noexcept;
  FrameEncoder(const FrameEncoder &) = delete;
  FrameEncoder & operator=(const FrameEncoder &) = delete;

  // Encode one packed frame. `pixels` must hold `height` rows of `stride`
  // bytes (stride may exceed width*3 for row-padded sources). After an error
  // the encoder must not be reused.
  [[nodiscard]] EncodeFrameResult encode(
    std::span<const std::byte> pixels, std::size_t stride, SourcePixelFormat format);

  // Encode one frame handed over as 4:2:0 planes of the encoder's geometry
  // (chroma planes width/2 x height/2), copied as they are — no conversion.
  [[nodiscard]] EncodeFrameResult encode_yuv420(const Yuv420Planes & planes);

private:
  std::unique_ptr<Impl> impl_;
};

// Outcome of open_frame_encoder(). On success `encoder` is non-null, `error`
// empty, `backend` the libav encoder name that opened ("libx264", "libx265",
// "h264_nvenc", "hevc_nvenc"), and `fallback_note` set when kAuto could not
// use NVENC (why); on failure `encoder` is null and `error` explains why.
struct OpenFrameEncoderResult
{
  std::unique_ptr<FrameEncoder> encoder;
  std::string error;
  std::string backend;
  std::string fallback_note;

  [[nodiscard]] bool ok() const noexcept { return encoder != nullptr && error.empty(); }
};

// Open an encoder for frames of `width` x `height` (both even: 4:2:0 chroma)
// at the nominal rate `fps_num`/`fps_den`, which only steers rate control.
// Failures (bad geometry or rate, an unknown preset, a crf or gop out of
// range, no usable encoder in this FFmpeg build) return a null encoder with
// a descriptive error.
[[nodiscard]] OpenFrameEncoderResult open_frame_encoder(
  std::uint32_t width, std::uint32_t height, int fps_num, int fps_den,
  const FrameEncoderOptions & options = {});

// One decoded frame as packed BGR24 (width * 3 * height bytes, no padding).
struct DecodedFrame
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::byte> bgr;
};

struct ReceiveFrameResult
{
  // Empty with an empty `error` when no frame is ready yet.
  std::optional<DecodedFrame> frame;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Decodes Annex B packets, one frame per packet for streams without frame
// reordering (the CompressedVideo contract). A stream that does reorder
// still decodes, only later: frames then come out of receive() on a later
// send() or after flush().
class FrameDecoder
{
public:
  struct Impl;

  explicit FrameDecoder(std::unique_ptr<Impl> impl);
  ~FrameDecoder();

  FrameDecoder(FrameDecoder &&) noexcept;
  FrameDecoder & operator=(FrameDecoder &&) noexcept;
  FrameDecoder(const FrameDecoder &) = delete;
  FrameDecoder & operator=(const FrameDecoder &) = delete;

  // Feed one packet (the `data` of one CompressedVideo message). Returns ""
  // or an error; a corrupt packet is reported here or surfaces as no frame.
  [[nodiscard]] std::string send(std::span<const std::byte> packet);

  // Take the next decoded frame, if one is ready. Call until `frame` is
  // empty after each send().
  [[nodiscard]] ReceiveFrameResult receive();

  // Signal end of stream so frames the decoder still holds come out of
  // receive(). The decoder accepts no further send() afterwards.
  [[nodiscard]] std::string flush();

private:
  std::unique_ptr<Impl> impl_;
};

struct OpenFrameDecoderResult
{
  std::unique_ptr<FrameDecoder> decoder;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return decoder != nullptr && error.empty(); }
};

// Open a decoder for `codec`. Fails when this FFmpeg build has no decoder
// for it.
[[nodiscard]] OpenFrameDecoderResult open_frame_decoder(VideoCodec codec);

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__FRAME_CODEC_HPP_
