// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_camera_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_layout.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Unit tests for the camera panel: frame decode / resize, the nearest-message
// follower reader, the per-panel renderer, and the Panel itself in both roles
// (the clock fixing the cell size, a follower fitting into it).

namespace
{

using bagwiz::commands::CameraPanel;
using bagwiz::commands::CloudSources;
using bagwiz::commands::FrameBuffer;
using bagwiz::commands::FrameNormalizer;
using bagwiz::commands::GridCanvas;
using bagwiz::commands::GridSpec;
using bagwiz::commands::NearestMessageSource;
using bagwiz::commands::PanelSize;
using bagwiz::commands::resize_frame;
using bagwiz::commands::TickInfo;
using bagwiz::commands::VideoInputScan;
using bagwiz::commands::VideoOverlayParams;
using bagwiz::commands::ViewRenderer;
using bagwiz::test::kMovifyGarbagePayload;
using bagwiz::test::kMovifyImageType;
using bagwiz::test::movify_bgr8_image_payload;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::MovifyTmpDirTest;

// ---- FrameNormalizer::decode ------------------------------------------------

TEST(FrameNormalizerDecode, Bgr8ImageBecomesCanonicalFrame)
{
  FrameNormalizer normalizer(kMovifyImageType);
  const auto payload = movify_bgr8_image_payload(2, 1, 0x2A);
  std::string error;
  const auto frame = normalizer.decode(42, payload, error);
  ASSERT_TRUE(frame.has_value()) << error;
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(frame->timestamp_ns, 42);
  EXPECT_EQ(frame->header_stamp_ns, 0);
  EXPECT_EQ(frame->width, 2u);
  EXPECT_EQ(frame->height, 1u);
  EXPECT_EQ(frame->step, 6u);
  EXPECT_EQ(frame->encoding, "bgr8");
  ASSERT_EQ(frame->data.size(), 6u);
  EXPECT_EQ(frame->data[0], std::byte{0x2A});
}

TEST(FrameNormalizerDecode, GarbagePayloadRejected)
{
  FrameNormalizer normalizer(kMovifyImageType);
  std::string error;
  EXPECT_FALSE(normalizer.decode(42, kMovifyGarbagePayload, error).has_value());
  EXPECT_FALSE(error.empty());
}

// ---- resize_frame -----------------------------------------------------------

TEST(ResizeFrame, SameSizeLeavesFrameUntouched)
{
  FrameBuffer frame;
  frame.width = 2;
  frame.height = 1;
  frame.step = 6;
  frame.data = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}};
  const auto original = frame.data;
  EXPECT_EQ(resize_frame(frame, 2, 1), "");
  EXPECT_EQ(frame.width, 2u);
  EXPECT_EQ(frame.height, 1u);
  EXPECT_EQ(frame.data, original);
}

TEST(ResizeFrame, DownscaleHalvesDimensions)
{
  FrameBuffer frame;
  frame.width = 4;
  frame.height = 2;
  frame.step = 12;
  frame.data.resize(24, std::byte{0x7F});
  EXPECT_EQ(resize_frame(frame, 2, 1), "");
  EXPECT_EQ(frame.width, 2u);
  EXPECT_EQ(frame.height, 1u);
  EXPECT_EQ(frame.step, 6u);
  EXPECT_EQ(frame.data.size(), 6u);
}

TEST(ResizeFrame, RejectsZeroSizeResult)
{
  FrameBuffer frame;
  frame.width = 1;
  frame.height = 1;
  frame.step = 3;
  frame.data.resize(3, std::byte{0x00});
  EXPECT_NE(resize_frame(frame, 0, 0), "");
}

// ---- NearestMessageSource ---------------------------------------------------

TEST_F(MovifyTmpDirTest, NearestMessageSourceMatchesByRecordTime)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/sec", kMovifyImageType);
    w->write("/sec", 100, kMovifyGarbagePayload);
    w->write("/sec", 200, kMovifyGarbagePayload);
    w->write("/sec", 400, kMovifyGarbagePayload);
    w->close();
  }
  std::string error;
  auto source = NearestMessageSource::open(bag, "/sec", error);
  ASSERT_NE(source, nullptr) << error;

  // Before the first message: the first message wins.
  const auto * m = source->fetch(90, error);
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->record_ns, 100);
  // An exact tie prefers the earlier message.
  m = source->fetch(150, error);
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->record_ns, 100);
  // Past the tie point: the later message wins.
  m = source->fetch(160, error);
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->record_ns, 200);
  m = source->fetch(350, error);
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->record_ns, 400);
  // Past the end: the last message sticks.
  m = source->fetch(999, error);
  ASSERT_NE(m, nullptr) << error;
  EXPECT_EQ(m->record_ns, 400);
}

TEST_F(MovifyTmpDirTest, NearestMessageSourceEmptyTopicYieldsNull)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/empty", kMovifyImageType);
    movify_declare_topic(*w, "/other", kMovifyImageType);
    w->write("/other", 100, kMovifyGarbagePayload);
    w->close();
  }
  std::string error;
  auto source = NearestMessageSource::open(bag, "/empty", error);
  ASSERT_NE(source, nullptr) << error;
  EXPECT_EQ(source->fetch(100, error), nullptr);
  EXPECT_TRUE(error.empty());
}

// ---- ViewRenderer -----------------------------------------------------------

TEST(ViewRendererTest, FixedScaleRejectsNativeSizeChange)
{
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, 0.5);
  std::string error;
  const auto geom = renderer.prepare(4, 2, 0, 0, error);
  ASSERT_TRUE(geom.has_value()) << error;
  EXPECT_EQ(geom->width, 2u);
  EXPECT_EQ(geom->height, 1u);
  // Same native size again: fine. A change: abort, like the single-view lock.
  EXPECT_TRUE(renderer.prepare(4, 2, 0, 0, error).has_value());
  EXPECT_FALSE(renderer.prepare(2, 2, 0, 0, error).has_value());
  EXPECT_NE(error.find("frame changed to 2x2 from the first frame's 4x2"), std::string::npos)
    << error;
}

TEST(ViewRendererTest, FitViewScalesIntoTheCellPreservingAspect)
{
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, std::nullopt);
  // A 2x2 native frame in a 4x2 cell fits at scale 1 (limited by height).
  std::string error;
  const auto geom = renderer.prepare(2, 2, 4, 2, error);
  ASSERT_TRUE(geom.has_value()) << error;
  EXPECT_EQ(geom->width, 2u);
  EXPECT_EQ(geom->height, 2u);
}

// --no-rectify on a panel that still has camera info (the --pcd case): the
// render geometry must keep the camera info, now at the render scale and with
// its distortion coefficients intact, so the projection can take the raw,
// distortion-aware path instead of assuming a rectified image.
TEST(ViewRendererTest, NoRectifyKeepsScaledCameraInfoForRawProjection)
{
  bagwiz::core::image::CameraInfo info;
  info.width = 4;
  info.height = 2;
  info.distortion_model = "plumb_bob";
  info.d = {0.1, -0.05, 0.001, -0.002, 0.0};
  info.k = {100.0, 0.0, 2.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0};
  info.p = {90.0, 0.0, 2.0, 0.0, 0.0, 90.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  ViewRenderer renderer(&info, /*rectify=*/false, VideoOverlayParams{}, 0.5);
  std::string error;
  const auto geom = renderer.prepare(4, 2, 0, 0, error);
  ASSERT_TRUE(geom.has_value()) << error;
  EXPECT_FALSE(geom->rectify);
  EXPECT_TRUE(geom->has_camera_info);
  EXPECT_EQ(geom->camera_info.width, 2u);
  EXPECT_EQ(geom->camera_info.height, 1u);
  EXPECT_DOUBLE_EQ(geom->camera_info.k[0], 50.0);
  EXPECT_DOUBLE_EQ(geom->camera_info.k[2], 1.0);
  EXPECT_EQ(geom->camera_info.distortion_model, "plumb_bob");
  EXPECT_EQ(geom->camera_info.d, info.d);
}

TEST(ViewRendererTest, RenderPastesCenteredIntoTheCell)
{
  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(4, 2);
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, std::nullopt);
  std::string error;
  const auto geom = renderer.prepare(2, 2, 4, 2, error);
  ASSERT_TRUE(geom.has_value()) << error;

  FrameBuffer frame;
  frame.width = 2;
  frame.height = 2;
  frame.step = 6;
  frame.encoding = "bgr8";
  frame.data.assign(12, std::byte{0x2A});

  canvas.clear();
  ASSERT_EQ(renderer.render(frame, *geom, nullptr, canvas.cell(0)), "");
  // x_off = (4-2)/2 = 1: columns 1 and 2 carry the frame, 0 and 3 stay black.
  for (std::uint32_t y = 0; y < 2; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * 12;
    EXPECT_EQ(canvas.pixels()[row + 0], std::byte{0});
    EXPECT_EQ(canvas.pixels()[row + 3], std::byte{0x2A});
    EXPECT_EQ(canvas.pixels()[row + 6], std::byte{0x2A});
    EXPECT_EQ(canvas.pixels()[row + 9], std::byte{0});
  }
}

// ---- CameraPanel ------------------------------------------------------------

// Cloud sources over no topics: the panels under test project nothing.
CloudSources empty_clouds(const std::filesystem::path & input)
{
  VideoInputScan scan;
  return CloudSources(input, scan, nullptr);
}

CameraPanel::Options image_options(const std::string & topic)
{
  CameraPanel::Options options;
  options.topic = topic;
  options.topic_type = kMovifyImageType;
  return options;
}

// The clock role decodes the tick's own payload; its render size (here the
// native size at scale 1) is what the first tick adopts as the cell size,
// and the frame is pasted at the cell's top-left when it fills the cell.
TEST_F(MovifyTmpDirTest, ClockPanelFixesTheCellSizeAndRendersItsPayload)
{
  auto clouds = empty_clouds(tmp_dir_ / "unused.mcap");
  CameraPanel panel(image_options("/cam/image"), CameraPanel::ClockSizing{}, &clouds);
  EXPECT_FALSE(panel.clock_cell_size().has_value());  // nothing selected yet

  const auto payload = movify_bgr8_image_payload(4, 2, 0x2A);
  const TickInfo tick{0, 1'000, payload};
  ASSERT_EQ(panel.select(tick, PanelSize{}), "");
  const auto size = panel.clock_cell_size();
  ASSERT_TRUE(size.has_value());
  EXPECT_EQ(size->width, 4u);
  EXPECT_EQ(size->height, 2u);

  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(size->width, size->height);
  canvas.clear();
  ASSERT_EQ(panel.render(canvas.cell(0)), "");
  for (const auto b : canvas.pixels()) {
    EXPECT_EQ(b, std::byte{0x2A});
  }
}

// --width pins the clock's render size from the output width: 8 px across 2
// columns gives 4-px cells, and the height follows the frame's aspect ratio.
TEST_F(MovifyTmpDirTest, ClockPanelPinsTheCellFromTheOutputWidth)
{
  auto clouds = empty_clouds(tmp_dir_ / "unused.mcap");
  CameraPanel::ClockSizing sizing;
  sizing.total_width = 8;
  sizing.grid_cols = 2;
  CameraPanel panel(image_options("/cam/image"), sizing, &clouds);

  const auto payload = movify_bgr8_image_payload(8, 4, 0x10);
  ASSERT_EQ(panel.select(TickInfo{0, 1'000, payload}, PanelSize{}), "");
  const auto size = panel.clock_cell_size();
  ASSERT_TRUE(size.has_value());
  EXPECT_EQ(size->width, 4u);
  EXPECT_EQ(size->height, 2u);
}

// A clock frame that does not decode is an error naming the topic (the loop
// adds the frame index), and the panel then has nothing to render.
TEST_F(MovifyTmpDirTest, ClockPanelReportsAnUndecodablePayload)
{
  auto clouds = empty_clouds(tmp_dir_ / "unused.mcap");
  CameraPanel panel(image_options("/cam/image"), CameraPanel::ClockSizing{}, &clouds);
  const auto error = panel.select(TickInfo{0, 1'000, kMovifyGarbagePayload}, PanelSize{});
  EXPECT_EQ(error.rfind("topic '/cam/image': ", 0), 0u) << error;
  EXPECT_FALSE(panel.clock_cell_size().has_value());
}

// A follower shows its own topic's message nearest each tick, decoding a
// message once and fitting it into the cell; a tick before its first message
// still shows that first message, and a later tick moves on to the next.
TEST_F(MovifyTmpDirTest, FollowerPanelTracksTheNearestMessage)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/sec", kMovifyImageType);
    const auto first = movify_bgr8_image_payload(2, 2, 0x11);
    const auto second = movify_bgr8_image_payload(2, 2, 0x22);
    w->write("/sec", 100, first);
    w->write("/sec", 300, second);
    w->close();
  }
  auto clouds = empty_clouds(bag);
  std::string error;
  auto source = NearestMessageSource::open(bag, "/sec", error);
  ASSERT_NE(source, nullptr) << error;
  CameraPanel panel(image_options("/sec"), std::move(source), &clouds);
  EXPECT_FALSE(panel.clock_cell_size().has_value());  // never the clock

  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(4, 2);  // the 2x2 frame fits at scale 1, centered
  const PanelSize cell{4, 2};

  canvas.clear();
  ASSERT_EQ(panel.select(TickInfo{0, 50, {}}, cell), "");
  ASSERT_EQ(panel.render(canvas.cell(0)), "");
  EXPECT_EQ(canvas.pixels()[3], std::byte{0x11});  // column 1 carries the frame
  EXPECT_EQ(canvas.pixels()[0], std::byte{0});     // column 0 is a black bar

  canvas.clear();
  ASSERT_EQ(panel.select(TickInfo{1, 250, {}}, cell), "");
  ASSERT_EQ(panel.render(canvas.cell(0)), "");
  EXPECT_EQ(canvas.pixels()[3], std::byte{0x22});
}

// A follower over a topic with no messages leaves its cell black and succeeds.
TEST_F(MovifyTmpDirTest, FollowerPanelWithNoMessagesLeavesTheCellBlack)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, movify_mcap_options());
    movify_declare_topic(*w, "/empty", kMovifyImageType);
    movify_declare_topic(*w, "/other", kMovifyImageType);
    w->write("/other", 100, kMovifyGarbagePayload);
    w->close();
  }
  auto clouds = empty_clouds(bag);
  std::string error;
  auto source = NearestMessageSource::open(bag, "/empty", error);
  ASSERT_NE(source, nullptr) << error;
  CameraPanel panel(image_options("/empty"), std::move(source), &clouds);

  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(2, 2);
  canvas.clear();
  ASSERT_EQ(panel.select(TickInfo{0, 100, {}}, PanelSize{2, 2}), "");
  ASSERT_EQ(panel.render(canvas.cell(0)), "");
  for (const auto b : canvas.pixels()) {
    EXPECT_EQ(b, std::byte{0});
  }
}

}  // namespace
