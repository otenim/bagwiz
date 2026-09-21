// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/video_encoder.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::core::video::H264Backend;
using bagwiz::core::video::open_video_encoder;
using bagwiz::core::video::probe_video;
using bagwiz::core::video::SourcePixelFormat;
using bagwiz::core::video::VideoEncoderOptions;
using bagwiz::core::video::Yuv420Planes;

class VideoEncoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_video_encoder_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
       "_" +
       std::to_string(
         reinterpret_cast<std::uintptr_t>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
           this)));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

std::vector<std::byte> solid_bgr(std::uint32_t w, std::uint32_t h, std::uint8_t b)
{
  std::vector<std::byte> px(static_cast<std::size_t>(w) * h * 3);
  for (std::size_t i = 0; i + 2 < px.size(); i += 3) {
    px[i] = std::byte{b};         // B
    px[i + 1] = std::byte{0x40};  // G
    px[i + 2] = std::byte{0x80};  // R
  }
  return px;
}

// A diagonal gradient shifted per frame: textured enough that the quality
// target visibly changes the encoded size, unlike a flat color.
std::vector<std::byte> gradient_bgr(std::uint32_t w, std::uint32_t h, int frame_index)
{
  std::vector<std::byte> px(static_cast<std::size_t>(w) * h * 3);
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      const std::size_t base = (static_cast<std::size_t>(y) * w + x) * 3;
      const unsigned shift = static_cast<unsigned>(frame_index) * 7U;
      px[base] = static_cast<std::byte>((x * 255U / w + shift) & 0xFFU);
      px[base + 1] = static_cast<std::byte>((y * 255U / h + shift) & 0xFFU);
      px[base + 2] = static_cast<std::byte>(((x + y) * 9U + shift) & 0xFFU);
    }
  }
  return px;
}

// MJPEG/.avi is used deliberately: the MJPEG encoder is built into every FFmpeg,
// so this test does not depend on libx264 being present in the build.
TEST_F(VideoEncoderTest, EncodesMjpegAviAndProbesBack)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 8;
  const auto out = tmp_dir_ / "clip.avi";

  auto opened = open_video_encoder(out, kW, kH, 10, 1);
  ASSERT_TRUE(opened.ok()) << opened.error;

  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 30));
    const auto err = opened.encoder->write_frame(
      {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
  }
  const auto fin = opened.encoder->finish();
  ASSERT_TRUE(fin.empty()) << fin;
  opened.encoder.reset();  // close the file before reading it back

  ASSERT_TRUE(std::filesystem::exists(out));
  const auto probe = probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, kW);
  EXPECT_EQ(probe.height, kH);
  EXPECT_EQ(probe.frame_count, kFrames);
}

// H.264/.mp4 is the headline output format. libx264 ships in the conda-forge
// gpl ffmpeg these environments use, but skip gracefully if a build lacks it so
// the suite stays portable.
TEST_F(VideoEncoderTest, EncodesH264Mp4WhenLibx264Available)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 6;
  const auto out = tmp_dir_ / "clip.mp4";

  auto opened = open_video_encoder(out, kW, kH, 15, 1);
  if (!opened.ok()) {
    GTEST_SKIP() << "H.264 encoder unavailable: " << opened.error;
  }
  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 40));
    const auto err = opened.encoder->write_frame(
      {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
  }
  ASSERT_TRUE(opened.encoder->finish().empty());
  opened.encoder.reset();

  const auto probe = probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, kW);
  EXPECT_EQ(probe.height, kH);
  EXPECT_EQ(probe.frame_count, kFrames);
  EXPECT_EQ(probe.codec, "h264");
  EXPECT_FALSE(probe.has_b_frames);
}

// Every frame handed to the encoder comes back out of the container as a
// playable frame, and the file spans all of them. The mp4 muxer takes the
// last sample's length from the packet durations; a stream that carries none
// ends one frame early, with an edit list that cuts the last sample so a
// player never shows it.
TEST_F(VideoEncoderTest, KeepsTheLastFrameOfAnH264ClipWhenLibx264Available)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 8;
  constexpr int kFps = 10;
  const auto out = tmp_dir_ / "clip.mp4";

  VideoEncoderOptions options;
  options.backend = H264Backend::kX264;
  options.preset = "ultrafast";
  auto opened = open_video_encoder(out, kW, kH, kFps, 1, options);
  if (!opened.ok()) {
    GTEST_SKIP() << "libx264 unavailable: " << opened.error;
  }
  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 30));
    const auto err = opened.encoder->write_frame(
      {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
    ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
  }
  ASSERT_TRUE(opened.encoder->finish().empty());
  opened.encoder.reset();

  const auto probe = probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, kFrames);
  EXPECT_NEAR(probe.duration_s, static_cast<double>(kFrames) / kFps, 1e-3);
}

TEST_F(VideoEncoderTest, RejectsAnUnknownH264Preset)
{
  VideoEncoderOptions options;
  options.preset = "warp";
  const auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 16, 16, 10, 1, options);
  EXPECT_FALSE(opened.ok());
  EXPECT_EQ(opened.error, "unknown H.264 preset 'warp'");
}

// The H.264 quality target and keyframe interval are checked before the
// encoder opens, with the bounds `video encode` applies to the same knobs.
TEST_F(VideoEncoderTest, RejectsACrfOutsideTheEncoderRange)
{
  for (const int crf : {-1, 52}) {
    VideoEncoderOptions options;
    options.crf = crf;
    const auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 16, 16, 10, 1, options);
    EXPECT_FALSE(opened.ok()) << "crf " << crf;
    EXPECT_EQ(opened.error, "crf " + std::to_string(crf) + " is outside 0..51");
  }
}

TEST_F(VideoEncoderTest, RejectsAGopBelowOneFrame)
{
  VideoEncoderOptions options;
  options.gop = 0;
  const auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 16, 16, 10, 1, options);
  EXPECT_FALSE(opened.ok());
  EXPECT_EQ(opened.error, "gop must be at least 1 frame");
}

// With scenecut off, libx264 places a keyframe exactly every `gop` frames, so
// the probe counts them back from the container's packet flags.
TEST_F(VideoEncoderTest, GopSetsTheKeyframeIntervalWhenLibx264Available)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 8;
  for (const auto & [gop, expected_keyframes] : {std::pair{1, 8}, std::pair{4, 2}}) {
    const auto out = tmp_dir_ / ("gop" + std::to_string(gop) + ".mp4");
    VideoEncoderOptions options;
    options.backend = H264Backend::kX264;
    options.preset = "ultrafast";
    options.gop = gop;
    auto opened = open_video_encoder(out, kW, kH, 10, 1, options);
    if (!opened.ok()) {
      GTEST_SKIP() << "libx264 unavailable: " << opened.error;
    }
    for (int i = 0; i < kFrames; ++i) {
      const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 30));
      const auto err = opened.encoder->write_frame(
        {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
      ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
    }
    ASSERT_TRUE(opened.encoder->finish().empty());
    opened.encoder.reset();

    const auto probe = probe_video(out);
    ASSERT_TRUE(probe.ok()) << probe.error;
    EXPECT_EQ(probe.frame_count, kFrames) << "gop " << gop;
    EXPECT_EQ(probe.keyframe_count, expected_keyframes) << "gop " << gop;
  }
}

// crf is the constant-quality target: a lower value spends more bytes on the
// same textured frames.
TEST_F(VideoEncoderTest, LowerCrfProducesLargerOutputWhenLibx264Available)
{
  constexpr std::uint32_t kW = 64;
  constexpr std::uint32_t kH = 32;
  constexpr int kFrames = 6;
  std::uintmax_t size_at_crf10 = 0;
  std::uintmax_t size_at_crf40 = 0;
  for (const int crf : {10, 40}) {
    const auto out = tmp_dir_ / ("crf" + std::to_string(crf) + ".mp4");
    VideoEncoderOptions options;
    options.backend = H264Backend::kX264;
    options.preset = "ultrafast";
    options.crf = crf;
    auto opened = open_video_encoder(out, kW, kH, 10, 1, options);
    if (!opened.ok()) {
      GTEST_SKIP() << "libx264 unavailable: " << opened.error;
    }
    for (int i = 0; i < kFrames; ++i) {
      const auto px = gradient_bgr(kW, kH, i);
      const auto err = opened.encoder->write_frame(
        {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8);
      ASSERT_TRUE(err.empty()) << "frame " << i << ": " << err;
    }
    ASSERT_TRUE(opened.encoder->finish().empty());
    opened.encoder.reset();
    (crf == 10 ? size_at_crf10 : size_at_crf40) = std::filesystem::file_size(out);
  }
  EXPECT_GT(size_at_crf10, size_at_crf40);
}

TEST_F(VideoEncoderTest, MjpegIgnoresTheH264Options)
{
  VideoEncoderOptions options;
  options.backend = H264Backend::kNvenc;
  options.preset = "warp";
  auto opened = open_video_encoder(tmp_dir_ / "clip.avi", 16, 16, 10, 1, options);
  ASSERT_TRUE(opened.ok()) << opened.error;
  EXPECT_EQ(opened.backend, "mjpeg");
  EXPECT_TRUE(opened.fallback_note.empty());
}

TEST_F(VideoEncoderTest, ForcedX264NamesItsBackendAndTakesThePreset)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  VideoEncoderOptions options;
  options.backend = H264Backend::kX264;
  options.preset = "ultrafast";
  auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", kW, kH, 10, 1, options);
  if (!opened.ok()) {
    GTEST_SKIP() << "libx264 unavailable: " << opened.error;
  }
  EXPECT_EQ(opened.backend, "libx264");
  EXPECT_TRUE(opened.fallback_note.empty());
  // Six frames, like EncodesH264Mp4WhenLibx264Available: the probe's packet
  // count is only reliable past the container's stream-info probing window.
  constexpr int kFrames = 6;
  for (int i = 0; i < kFrames; ++i) {
    const auto px = solid_bgr(kW, kH, static_cast<std::uint8_t>(i * 40));
    ASSERT_TRUE(
      opened.encoder
        ->write_frame(
          {px.data(), px.size()}, static_cast<std::size_t>(kW) * 3, SourcePixelFormat::kBgr8)
        .empty());
  }
  ASSERT_TRUE(opened.encoder->finish().empty());
  opened.encoder.reset();
  const auto probe = probe_video(tmp_dir_ / "clip.mp4");
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.codec, "h264");
  EXPECT_EQ(probe.frame_count, kFrames);
}

// The default (auto) backend uses libx264 outright for a small frame: no
// NVENC attempt, so no fallback note.
TEST_F(VideoEncoderTest, AutoUsesX264ForSmallFrames)
{
  auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 32, 16, 10, 1);
  if (!opened.ok()) {
    GTEST_SKIP() << "no H.264 encoder: " << opened.error;
  }
  EXPECT_EQ(opened.backend, "libx264");
  EXPECT_TRUE(opened.fallback_note.empty()) << opened.fallback_note;
}

// Above 1080p, auto opens whichever H.264 encoder works here: NVENC on a
// machine with a usable GPU, else libx264 with a note saying why.
TEST_F(VideoEncoderTest, AutoTriesNvencAbove1080p)
{
  auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 2560, 1440, 10, 1);
  if (!opened.ok()) {
    GTEST_SKIP() << "no H.264 encoder: " << opened.error;
  }
  EXPECT_TRUE(opened.backend == "libx264" || opened.backend == "h264_nvenc") << opened.backend;
  EXPECT_EQ(opened.backend == "libx264", !opened.fallback_note.empty()) << opened.fallback_note;
}

// A forced backend never falls back: without a usable NVENC the open fails
// and says so.
TEST_F(VideoEncoderTest, ForcedNvencEitherOpensOrFailsLoudly)
{
  VideoEncoderOptions options;
  options.backend = H264Backend::kNvenc;
  auto opened = open_video_encoder(tmp_dir_ / "clip.mp4", 32, 16, 10, 1, options);
  if (opened.ok()) {
    EXPECT_EQ(opened.backend, "h264_nvenc");
    EXPECT_TRUE(opened.fallback_note.empty());
    return;
  }
  EXPECT_NE(opened.error.find("h264_nvenc"), std::string::npos) << opened.error;
  EXPECT_TRUE(opened.fallback_note.empty());
}

// Planes handed over as decoded: a solid 4:2:0 frame with padded strides
// encodes frame for frame, for both codecs.
TEST_F(VideoEncoderTest, WritesYuv420PlanesAsTheyAre)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 16;
  constexpr int kFrames = 6;
  constexpr std::size_t kYStride = 40;  // padded past the 32-px width
  constexpr std::size_t kCStride = 24;  // padded past the 16-px chroma width
  std::vector<std::uint8_t> y(kYStride * kH, 200);
  std::vector<std::uint8_t> u(kCStride * (kH / 2), 100);
  std::vector<std::uint8_t> v(kCStride * (kH / 2), 150);
  Yuv420Planes planes;
  planes.y = y.data();
  planes.y_stride = kYStride;
  planes.u = u.data();
  planes.u_stride = kCStride;
  planes.v = v.data();
  planes.v_stride = kCStride;
  for (const char * ext : {"clip.avi", "clip.mp4"}) {
    const auto out = tmp_dir_ / ext;
    VideoEncoderOptions options;
    options.full_range = true;
    auto opened = open_video_encoder(out, kW, kH, 10, 1, options);
    if (!opened.ok()) {
      GTEST_SKIP() << "encoder unavailable: " << opened.error;
    }
    for (int i = 0; i < kFrames; ++i) {
      ASSERT_TRUE(opened.encoder->write_yuv420(planes).empty()) << ext << " frame " << i;
    }
    ASSERT_TRUE(opened.encoder->finish().empty());
    opened.encoder.reset();
    const auto probe = probe_video(out);
    ASSERT_TRUE(probe.ok()) << probe.error;
    EXPECT_EQ(probe.width, kW);
    EXPECT_EQ(probe.height, kH);
    EXPECT_EQ(probe.frame_count, kFrames) << ext;
  }
}

TEST_F(VideoEncoderTest, RejectsYuv420PlanesThatAreMissingOrTooNarrow)
{
  auto opened = open_video_encoder(tmp_dir_ / "clip.avi", 32, 16, 10, 1);
  ASSERT_TRUE(opened.ok()) << opened.error;
  std::vector<std::uint8_t> plane(32 * 16, 0);
  Yuv420Planes planes;
  planes.y = plane.data();
  planes.y_stride = 32;
  planes.u = plane.data();
  planes.u_stride = 16;
  EXPECT_EQ(opened.encoder->write_yuv420(planes), "yuv420 frame is missing a plane");
  planes.v = plane.data();
  planes.v_stride = 8;  // narrower than the 16-px chroma width
  EXPECT_EQ(
    opened.encoder->write_yuv420(planes),
    "yuv420 frame row stride is shorter than the frame width");
}

TEST_F(VideoEncoderTest, RejectsUnsupportedExtension)
{
  const auto opened = open_video_encoder(tmp_dir_ / "clip.webm", 16, 16, 10, 1);
  EXPECT_FALSE(opened.ok());
  EXPECT_EQ(opened.encoder, nullptr);
}

TEST_F(VideoEncoderTest, RejectsOddDimensions)
{
  const auto opened = open_video_encoder(tmp_dir_ / "clip.avi", 15, 16, 10, 1);  // odd width
  EXPECT_FALSE(opened.ok());
}

}  // namespace
