// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_direct.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/commands/movify.hpp"
#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "core/image/image_fixture.hpp"
#include "movify_inputs.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_output.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::can_stream_camera_direct;
using bagwiz::commands::direct_decode_slots;
using bagwiz::commands::MovifyArgs;
using bagwiz::commands::run_direct_encode_pass;
using bagwiz::commands::VideoFrameEncoder;
using bagwiz::commands::VideoInputValidation;
using bagwiz::commands::ViewInput;
using bagwiz::test::kMovifyGarbagePayload;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::MovifyCdrBuilder;
using bagwiz::test::MovifyTmpDirTest;

constexpr const char * kTopic = "/cam/image_raw/compressed";
constexpr const char * kCompressedType = "sensor_msgs/msg/CompressedImage";
constexpr std::uint32_t kW = 32;
constexpr std::uint32_t kH = 16;

// The run every eligibility test starts from: one CompressedImage camera on
// a 1x1 grid, nothing else.
struct DirectRun
{
  MovifyArgs args;
  VideoInputValidation validation;

  DirectRun()
  {
    args.cam_topics = {kTopic};
    args.rectify = false;
    ViewInput view;
    view.topic = kTopic;
    view.topic_type = kCompressedType;
    validation.views.push_back(view);
    validation.grid.cols = 1;
    validation.grid.rows = 1;
  }
};

TEST(CanStreamCameraDirect, OneJpegCameraShownAsDecodedQualifies)
{
  const DirectRun run;
  EXPECT_TRUE(can_stream_camera_direct(run.args, run.validation));
}

TEST(CanStreamCameraDirect, RectificationInEffectComposes)
{
  DirectRun run;
  run.args.rectify = true;
  run.validation.views[0].camera_info_topic = "/cam/camera_info";
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
  // Rectification requested but no camera info to do it with: the frame is
  // shown as decoded after all.
  run.validation.views[0].camera_info_topic.reset();
  EXPECT_TRUE(can_stream_camera_direct(run.args, run.validation));
}

TEST(CanStreamCameraDirect, ResizingOrAFixedWidthComposes)
{
  DirectRun run;
  run.args.resize_scale = 0.5f;
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
  run.args.resize_scale = 1.0f;
  run.args.width = 1920;
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
}

TEST(CanStreamCameraDirect, AnyOtherPanelOrOverlayComposes)
{
  DirectRun run;
  run.validation.views[0].pcd_topics = {"/points"};
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
  run.validation.views[0].pcd_topics.clear();
  run.validation.pcd_topics = {"/points"};
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
  run.validation.pcd_topics.clear();
  run.validation.gnss_topic = "/gnss";
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
  run.validation.gnss_topic.reset();
  run.validation.grid.cols = 2;
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
}

TEST(CanStreamCameraDirect, ARawImageTopicComposes)
{
  DirectRun run;
  run.validation.views[0].topic_type = "sensor_msgs/msg/Image";
  EXPECT_FALSE(can_stream_camera_direct(run.args, run.validation));
}

TEST(DirectDecodeSlots, AQuarterOfTheCoresBetweenTwoAndFour)
{
  EXPECT_EQ(direct_decode_slots(true, 24), 4u);
  EXPECT_EQ(direct_decode_slots(true, 12), 3u);
  EXPECT_EQ(direct_decode_slots(true, 8), 2u);
  EXPECT_EQ(direct_decode_slots(true, 2), 2u);
  EXPECT_EQ(direct_decode_slots(true, 1), 1u);
  EXPECT_EQ(direct_decode_slots(false, 24), 1u);
}

// Whether this FFmpeg build encodes H.264 with libx264 (skip otherwise, so the
// suite stays portable).
bool x264_available(const std::filesystem::path & dir)
{
  bagwiz::core::video::VideoEncoderOptions options;
  options.backend = bagwiz::core::video::H264Backend::kX264;
  return bagwiz::core::video::open_video_encoder(dir / "probe.mp4", 32, 16, 10, 1, options).ok();
}

// A CompressedImage payload wrapping `bitstream` with the given format.
std::vector<std::byte> compressed_payload(
  const std::string & format, const std::vector<std::byte> & bitstream)
{
  MovifyCdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.str("cam");
  b.str(format);
  b.byte_seq(bitstream);
  return b.take();
}

class MovifyDirectTest : public MovifyTmpDirTest
{
protected:
  // A bag with `frames` messages of `format` ("jpeg" or "png") on kTopic,
  // plus a garbage payload last when `garbage_last`.
  std::filesystem::path write_bag(const std::string & format, int frames, bool garbage_last)
  {
    const auto path = tmp_dir_ / "in.mcap";
    auto w = bagwiz::io::open_write(path, movify_mcap_options());
    movify_declare_topic(*w, kTopic, kCompressedType);
    for (int i = 0; i < frames; ++i) {
      const auto bits = bagwiz::test::encode_still_image(
        format, kW, kH, static_cast<std::uint8_t>(40 * i), 80, 120);
      if (bits.empty()) {
        return {};
      }
      const auto payload = compressed_payload(format, bits);
      w->write(kTopic, 1'000'000'000LL + i * 100'000'000LL, payload);
    }
    if (garbage_last) {
      w->write(kTopic, 2'000'000'000LL, kMovifyGarbagePayload);
    }
    w->close();
    return path;
  }

  std::unique_ptr<bagwiz::io::BagReader> open_topic(const std::filesystem::path & bag)
  {
    auto reader = bagwiz::io::open_read(bag);
    bagwiz::io::ReadFilter filter;
    filter.topics.push_back(kTopic);
    reader->set_filter(filter);
    return reader;
  }
};

TEST_F(MovifyDirectTest, StreamsJpegFramesAsPlanes)
{
  const auto bag = write_bag("jpeg", 5, false);
  if (bag.empty()) {
    GTEST_SKIP() << "no JPEG encoder in this FFmpeg build";
  }
  for (const char * name : {"out.avi", "out.mp4"}) {
    const auto output = tmp_dir_ / name;
    if (output.extension() == ".mp4" && !x264_available(tmp_dir_)) {
      continue;  // the .avi pass above covered the planes; no H.264 encoder here
    }
    auto reader = open_topic(bag);
    VideoFrameEncoder encoder(output, bagwiz::core::video::FrameRate{10, 1});
    EXPECT_EQ(run_direct_encode_pass(*reader, kTopic, encoder, 3), 0) << name;
    ASSERT_TRUE(encoder.finish().empty()) << name;
    EXPECT_EQ(encoder.written(), 5u);
    const auto probe = bagwiz::core::video::probe_video(output);
    ASSERT_TRUE(probe.ok()) << probe.error;
    EXPECT_EQ(probe.width, kW);
    EXPECT_EQ(probe.height, kH);
    EXPECT_EQ(probe.frame_count, 5) << name;
  }
}

TEST_F(MovifyDirectTest, FallsBackToPackedBgrForFramesThatAreNotYuv)
{
  const auto bag = write_bag("png", 4, false);
  if (bag.empty()) {
    GTEST_SKIP() << "no PNG encoder in this FFmpeg build";
  }
  auto reader = open_topic(bag);
  const auto output = tmp_dir_ / "out.avi";
  VideoFrameEncoder encoder(output, bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(run_direct_encode_pass(*reader, kTopic, encoder, 2), 0);
  ASSERT_TRUE(encoder.finish().empty());
  const auto probe = bagwiz::core::video::probe_video(output);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 4);
}

TEST_F(MovifyDirectTest, OneDecoderStillEncodesEveryFrame)
{
  const auto bag = write_bag("jpeg", 3, false);
  if (bag.empty()) {
    GTEST_SKIP() << "no JPEG encoder in this FFmpeg build";
  }
  auto reader = open_topic(bag);
  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(run_direct_encode_pass(*reader, kTopic, encoder, 1), 0);
  ASSERT_TRUE(encoder.finish().empty());
  EXPECT_EQ(encoder.written(), 3u);
}

TEST_F(MovifyDirectTest, StopsAtAFrameThatDoesNotDecode)
{
  const auto bag = write_bag("jpeg", 2, true);
  if (bag.empty()) {
    GTEST_SKIP() << "no JPEG encoder in this FFmpeg build";
  }
  auto reader = open_topic(bag);
  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(run_direct_encode_pass(*reader, kTopic, encoder, 4), 1);
  // The two good frames before it were encoded.
  EXPECT_EQ(encoder.written(), 2u);
}

TEST_F(MovifyDirectTest, ReportsATopicWithoutFrames)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, kTopic, kCompressedType);
    w->close();
  }
  auto reader = open_topic(bag);
  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(run_direct_encode_pass(*reader, kTopic, encoder, 2), 1);
  EXPECT_FALSE(encoder.started());
}

}  // namespace
