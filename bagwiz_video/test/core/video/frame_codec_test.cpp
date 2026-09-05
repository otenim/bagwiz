// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/frame_codec.hpp"

#include "bagwiz/core/video/annexb.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>
#include <string>
#include <vector>

namespace
{

using bagwiz::core::video::DecodedFrame;
using bagwiz::core::video::EncoderBackend;
using bagwiz::core::video::FrameEncoderOptions;
using bagwiz::core::video::open_frame_decoder;
using bagwiz::core::video::open_frame_encoder;
using bagwiz::core::video::parse_video_codec_format;
using bagwiz::core::video::SourcePixelFormat;
using bagwiz::core::video::summarize_annexb;
using bagwiz::core::video::video_codec_format;
using bagwiz::core::video::VideoCodec;
using bagwiz::core::video::Yuv420Planes;

constexpr std::uint32_t kW = 64;
constexpr std::uint32_t kH = 48;

// A gentle ramp plus a per-frame offset, so consecutive frames differ (the
// encoder emits real P-frames) while chroma subsampling costs little.
std::vector<std::byte> ramp_bgr(std::uint32_t w, std::uint32_t h, int frame_index)
{
  std::vector<std::byte> px(static_cast<std::size_t>(w) * h * 3);
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      const std::size_t base = (static_cast<std::size_t>(y) * w + x) * 3;
      const unsigned shift = static_cast<unsigned>(frame_index) * 4U;
      px[base + 0] = static_cast<std::byte>((40U + (x * 80U) / w + shift) & 0xFFU);   // B
      px[base + 1] = static_cast<std::byte>((60U + (y * 60U) / h + shift) & 0xFFU);   // G
      px[base + 2] = static_cast<std::byte>((200U - (x * 60U) / w - shift) & 0xFFU);  // R
    }
  }
  return px;
}

std::vector<std::byte> solid_bgr(std::uint32_t w, std::uint32_t h, std::uint8_t level)
{
  return std::vector<std::byte>(static_cast<std::size_t>(w) * h * 3, std::byte{level});
}

struct ChannelError
{
  std::array<double, 3> mean_abs{};
  std::array<double, 3> bias{};
};

ChannelError channel_error(std::span<const std::byte> a, std::span<const std::byte> b)
{
  ChannelError err;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const int diff = static_cast<int>(a[i]) - static_cast<int>(b[i]);
    err.mean_abs[i % 3] += std::abs(diff);
    err.bias[i % 3] += diff;
  }
  const auto per_channel = static_cast<double>(a.size() / 3);
  for (std::size_t c = 0; c < 3; ++c) {
    err.mean_abs[c] /= per_channel;
    err.bias[c] /= per_channel;
  }
  return err;
}

FrameEncoderOptions cpu_options(VideoCodec codec)
{
  FrameEncoderOptions options;
  options.codec = codec;
  options.backend = EncoderBackend::kCpu;
  options.preset = "ultrafast";
  options.gop = 4;
  return options;
}

}  // namespace

TEST(FrameCodecTest, FormatNamesRoundTrip)
{
  EXPECT_EQ(video_codec_format(VideoCodec::kH264), "h264");
  EXPECT_EQ(video_codec_format(VideoCodec::kH265), "h265");
  EXPECT_EQ(parse_video_codec_format("h264"), VideoCodec::kH264);
  EXPECT_EQ(parse_video_codec_format("h265"), VideoCodec::kH265);
  EXPECT_FALSE(parse_video_codec_format("hevc").has_value());
  EXPECT_FALSE(parse_video_codec_format("vp9").has_value());
  EXPECT_FALSE(parse_video_codec_format("").has_value());
}

// The contract Foxglove's CompressedVideo relies on: one packet per frame,
// Annex B, keyframes carrying their parameter sets, the first frame a
// keyframe, and the GOP honoured.
TEST(FrameCodecTest, H264EmitsOneAnnexBPacketPerFrameWithParameterSetsOnKeyframes)
{
  auto opened = open_frame_encoder(kW, kH, 10, 1, cpu_options(VideoCodec::kH264));
  ASSERT_TRUE(opened.ok()) << opened.error;
  EXPECT_EQ(opened.backend, "libx264");

  constexpr int kFrames = 10;
  for (int i = 0; i < kFrames; ++i) {
    const auto px = ramp_bgr(kW, kH, i);
    const auto result =
      opened.encoder->encode({px.data(), px.size()}, kW * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(result.ok()) << "frame " << i << ": " << result.error;
    const auto summary = summarize_annexb(result.frame->data, VideoCodec::kH264);
    EXPECT_TRUE(summary.starts_with_start_code) << "frame " << i;
    EXPECT_GE(summary.nal_count, 1U) << "frame " << i;
    const bool expect_key = (i % 4) == 0;
    EXPECT_EQ(result.frame->keyframe, expect_key) << "frame " << i;
    EXPECT_EQ(summary.has_keyframe_slice, expect_key) << "frame " << i;
    if (expect_key) {
      EXPECT_TRUE(summary.has_sps) << "frame " << i;
      EXPECT_TRUE(summary.has_pps) << "frame " << i;
    }
  }
}

TEST(FrameCodecTest, H265KeyframesCarryVpsSpsPps)
{
  auto opened = open_frame_encoder(kW, kH, 10, 1, cpu_options(VideoCodec::kH265));
  if (!opened.ok()) {
    GTEST_SKIP() << "H.265 encoder unavailable: " << opened.error;
  }
  EXPECT_EQ(opened.backend, "libx265");
  for (int i = 0; i < 6; ++i) {
    const auto px = ramp_bgr(kW, kH, i);
    const auto result =
      opened.encoder->encode({px.data(), px.size()}, kW * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(result.ok()) << "frame " << i << ": " << result.error;
    const auto summary = summarize_annexb(result.frame->data, VideoCodec::kH265);
    EXPECT_TRUE(summary.starts_with_start_code) << "frame " << i;
    const bool expect_key = (i % 4) == 0;
    EXPECT_EQ(result.frame->keyframe, expect_key) << "frame " << i;
    if (expect_key) {
      EXPECT_TRUE(summary.has_vps) << "frame " << i;
      EXPECT_TRUE(summary.has_sps) << "frame " << i;
      EXPECT_TRUE(summary.has_pps) << "frame " << i;
      EXPECT_TRUE(summary.has_keyframe_slice) << "frame " << i;
    }
  }
}

// Each packet decodes on its own to exactly one frame (no reorder delay),
// with the picture intact: a few LSBs of codec noise and no level shift.
TEST(FrameCodecTest, PacketsDecodeOneToOneAndPreserveLevels)
{
  for (const VideoCodec codec : {VideoCodec::kH264, VideoCodec::kH265}) {
    auto options = cpu_options(codec);
    options.crf = 12;
    auto encoder = open_frame_encoder(kW, kH, 10, 1, options);
    if (!encoder.ok()) {
      GTEST_SKIP() << "encoder unavailable: " << encoder.error;
    }
    auto decoder = open_frame_decoder(codec);
    ASSERT_TRUE(decoder.ok()) << decoder.error;

    for (int i = 0; i < 6; ++i) {
      const auto px = ramp_bgr(kW, kH, i);
      const auto encoded =
        encoder.encoder->encode({px.data(), px.size()}, kW * 3, SourcePixelFormat::kBgr8);
      ASSERT_TRUE(encoded.ok()) << encoded.error;

      ASSERT_EQ(decoder.decoder->send(encoded.frame->data), "") << "frame " << i;
      auto received = decoder.decoder->receive();
      ASSERT_TRUE(received.ok()) << received.error;
      ASSERT_TRUE(received.frame.has_value()) << "frame " << i << " decoded late";
      EXPECT_EQ(received.frame->width, kW);
      EXPECT_EQ(received.frame->height, kH);
      ASSERT_EQ(received.frame->bgr.size(), px.size());
      const ChannelError err = channel_error(received.frame->bgr, px);
      for (std::size_t c = 0; c < 3; ++c) {
        EXPECT_LT(err.mean_abs[c], 5.0)
          << "codec " << video_codec_format(codec) << " frame " << i << " channel " << c;
        EXPECT_LT(std::abs(err.bias[c]), 2.5)
          << "codec " << video_codec_format(codec) << " frame " << i << " channel " << c;
      }
      // Nothing else is pending: exactly one frame came out of one packet.
      const auto extra = decoder.decoder->receive();
      EXPECT_TRUE(extra.ok()) << extra.error;
      EXPECT_FALSE(extra.frame.has_value()) << "frame " << i;
    }
    // A flush at end of stream releases nothing further.
    ASSERT_EQ(decoder.decoder->flush(), "");
    const auto tail = decoder.decoder->receive();
    EXPECT_TRUE(tail.ok()) << tail.error;
    EXPECT_FALSE(tail.frame.has_value());
  }
}

// Full-range and limited-range encodes of the same grey decode to the same
// grey: the stream's range tag and the conversion agree in both directions.
TEST(FrameCodecTest, FullRangeAndLimitedRangeDecodeToTheSameLevels)
{
  for (const bool full_range : {false, true}) {
    auto options = cpu_options(VideoCodec::kH264);
    options.crf = 10;
    options.full_range = full_range;
    auto encoder = open_frame_encoder(kW, kH, 10, 1, options);
    ASSERT_TRUE(encoder.ok()) << encoder.error;
    auto decoder = open_frame_decoder(VideoCodec::kH264);
    ASSERT_TRUE(decoder.ok()) << decoder.error;

    const auto px = solid_bgr(kW, kH, 128);
    const auto encoded =
      encoder.encoder->encode({px.data(), px.size()}, kW * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(encoded.ok()) << encoded.error;
    ASSERT_EQ(decoder.decoder->send(encoded.frame->data), "");
    const auto received = decoder.decoder->receive();
    ASSERT_TRUE(received.ok()) << received.error;
    ASSERT_TRUE(received.frame.has_value());
    const ChannelError err = channel_error(received.frame->bgr, px);
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_LT(std::abs(err.bias[c]), 3.0) << "full_range=" << full_range << " channel " << c;
    }
  }
}

TEST(FrameCodecTest, Yuv420PlanesEncodeWithoutConversion)
{
  auto options = cpu_options(VideoCodec::kH264);
  options.full_range = true;
  auto encoder = open_frame_encoder(kW, kH, 10, 1, options);
  ASSERT_TRUE(encoder.ok()) << encoder.error;

  constexpr std::size_t kYStride = kW + 8;  // padded rows
  constexpr std::size_t kCStride = kW / 2 + 8;
  const std::vector<std::uint8_t> y(kYStride * kH, 128);
  const std::vector<std::uint8_t> u(kCStride * (kH / 2), 128);
  const std::vector<std::uint8_t> v(kCStride * (kH / 2), 128);
  Yuv420Planes planes;
  planes.y = y.data();
  planes.y_stride = kYStride;
  planes.u = u.data();
  planes.u_stride = kCStride;
  planes.v = v.data();
  planes.v_stride = kCStride;

  const auto encoded = encoder.encoder->encode_yuv420(planes);
  ASSERT_TRUE(encoded.ok()) << encoded.error;
  EXPECT_TRUE(encoded.frame->keyframe);

  auto decoder = open_frame_decoder(VideoCodec::kH264);
  ASSERT_TRUE(decoder.ok()) << decoder.error;
  ASSERT_EQ(decoder.decoder->send(encoded.frame->data), "");
  const auto received = decoder.decoder->receive();
  ASSERT_TRUE(received.ok()) << received.error;
  ASSERT_TRUE(received.frame.has_value());
  // Full-range Y=U=V=128 is mid grey in RGB too.
  const auto grey = solid_bgr(kW, kH, 128);
  const ChannelError err = channel_error(received.frame->bgr, grey);
  for (std::size_t c = 0; c < 3; ++c) {
    EXPECT_LT(std::abs(err.bias[c]), 3.0) << "channel " << c;
  }
}

TEST(FrameCodecTest, OpenRejectsBadGeometryPresetAndCrf)
{
  auto options = cpu_options(VideoCodec::kH264);
  EXPECT_FALSE(open_frame_encoder(0, kH, 10, 1, options).ok());
  EXPECT_FALSE(open_frame_encoder(kW + 1, kH, 10, 1, options).ok());  // odd width
  EXPECT_FALSE(open_frame_encoder(kW, kH, 0, 1, options).ok());       // bad rate

  options.preset = "instant";
  EXPECT_FALSE(open_frame_encoder(kW, kH, 10, 1, options).ok());

  options = cpu_options(VideoCodec::kH264);
  options.crf = 99;
  EXPECT_FALSE(open_frame_encoder(kW, kH, 10, 1, options).ok());

  options = cpu_options(VideoCodec::kH264);
  options.gop = 0;
  EXPECT_FALSE(open_frame_encoder(kW, kH, 10, 1, options).ok());
}

TEST(FrameCodecTest, EncodeRejectsShortPixelBuffer)
{
  auto opened = open_frame_encoder(kW, kH, 10, 1, cpu_options(VideoCodec::kH264));
  ASSERT_TRUE(opened.ok()) << opened.error;
  const auto px = ramp_bgr(kW, kH - 1, 0);
  const auto result =
    opened.encoder->encode({px.data(), px.size()}, kW * 3, SourcePixelFormat::kBgr8);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(FrameCodecTest, DecoderSurvivesGarbageInput)
{
  auto decoder = open_frame_decoder(VideoCodec::kH264);
  ASSERT_TRUE(decoder.ok()) << decoder.error;
  std::vector<std::byte> junk(64);
  for (std::size_t i = 0; i < junk.size(); ++i) {
    junk[i] = static_cast<std::byte>((i * 37U + 11U) & 0xFFU);
  }
  // Either the send is rejected or nothing decodes; neither may crash or
  // hand back a frame.
  const auto err = decoder.decoder->send(junk);
  if (err.empty()) {
    const auto received = decoder.decoder->receive();
    EXPECT_FALSE(received.frame.has_value());
  }
}

// NVENC refuses frames below its minimum size, so this runs at a size it
// accepts; it skips (never fails) where no NVIDIA GPU or encoder is present.
TEST(FrameCodecTest, NvencBackendsEncodeOneToOneWhenAvailable)
{
  constexpr std::uint32_t kNvW = 256;
  constexpr std::uint32_t kNvH = 128;
  for (const VideoCodec codec : {VideoCodec::kH264, VideoCodec::kH265}) {
    FrameEncoderOptions options;
    options.codec = codec;
    options.backend = EncoderBackend::kNvenc;
    options.gop = 4;
    auto opened = open_frame_encoder(kNvW, kNvH, 10, 1, options);
    if (!opened.ok()) {
      GTEST_SKIP() << "NVENC unavailable: " << opened.error;
    }
    EXPECT_EQ(opened.backend, codec == VideoCodec::kH264 ? "h264_nvenc" : "hevc_nvenc");
    auto decoder = open_frame_decoder(codec);
    ASSERT_TRUE(decoder.ok()) << decoder.error;
    for (int i = 0; i < 9; ++i) {
      const auto px = ramp_bgr(kNvW, kNvH, i);
      const auto result =
        opened.encoder->encode({px.data(), px.size()}, kNvW * 3, SourcePixelFormat::kBgr8);
      ASSERT_TRUE(result.ok()) << "frame " << i << ": " << result.error;
      const auto summary = summarize_annexb(result.frame->data, codec);
      EXPECT_TRUE(summary.starts_with_start_code) << "frame " << i;
      const bool expect_key = (i % 4) == 0;
      EXPECT_EQ(result.frame->keyframe, expect_key) << "frame " << i;
      if (expect_key) {
        EXPECT_TRUE(summary.has_sps) << "frame " << i;
        EXPECT_TRUE(summary.has_pps) << "frame " << i;
        EXPECT_EQ(summary.has_vps, codec == VideoCodec::kH265) << "frame " << i;
      }
      ASSERT_EQ(decoder.decoder->send(result.frame->data), "") << "frame " << i;
      const auto received = decoder.decoder->receive();
      ASSERT_TRUE(received.ok()) << received.error;
      ASSERT_TRUE(received.frame.has_value()) << "frame " << i << " decoded late";
      const ChannelError err = channel_error(received.frame->bgr, px);
      for (std::size_t c = 0; c < 3; ++c) {
        EXPECT_LT(err.mean_abs[c], 6.0) << "frame " << i << " channel " << c;
      }
    }
  }
}
