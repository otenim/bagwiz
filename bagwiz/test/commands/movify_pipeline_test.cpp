// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_pipeline.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/video/frame_rate.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_layout.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_output.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"      // NOLINT(build/include_subdir) src-local shared header
#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Unit tests for the movify encode pass: the threading decision, and the two
// encode loops driven over a stub panel — the same ticks, cell size, and
// composed frames must come out of the synchronous and the parallel loop.

namespace
{

using bagwiz::commands::CellView;
using bagwiz::commands::GridSpec;
using bagwiz::commands::Panel;
using bagwiz::commands::PanelSize;
using bagwiz::commands::run_encode_pass;
using bagwiz::commands::should_use_parallel_pipeline;
using bagwiz::commands::TickInfo;
using bagwiz::commands::VideoFrameEncoder;
using bagwiz::test::kMovifyImageType;
using bagwiz::test::movify_declare_topic;
using bagwiz::test::movify_mcap_options;
using bagwiz::test::MovifyTmpDirTest;

// ---- should_use_parallel_pipeline ---------------------------------------------

TEST(ShouldUseParallelPipeline, SinglePanelWithoutPointCloudsStaysSynchronous)
{
  // One panel and no projection work: there is nothing to spread across workers.
  EXPECT_FALSE(should_use_parallel_pipeline(1, false, true, 100, 8));
}

TEST(ShouldUseParallelPipeline, MultiplePanelsRunInParallelWithoutPointClouds)
{
  EXPECT_TRUE(should_use_parallel_pipeline(2, false, true, 100, 8));
  EXPECT_TRUE(should_use_parallel_pipeline(8, false, true, 100, 8));
}

TEST(ShouldUseParallelPipeline, PointCloudsRunInParallelWithOnePanel)
{
  EXPECT_TRUE(should_use_parallel_pipeline(1, true, true, 100, 8));
}

TEST(ShouldUseParallelPipeline, RespectsDisableFlag)
{
  EXPECT_FALSE(should_use_parallel_pipeline(2, true, false, 100, 8));
}

TEST(ShouldUseParallelPipeline, RequiresEnoughFrames)
{
  EXPECT_FALSE(should_use_parallel_pipeline(2, true, true, 3, 8));
  EXPECT_TRUE(should_use_parallel_pipeline(2, true, true, 4, 8));
}

TEST(ShouldUseParallelPipeline, RequiresParallelHardware)
{
  EXPECT_FALSE(should_use_parallel_pipeline(2, true, true, 100, 1));
  EXPECT_FALSE(should_use_parallel_pipeline(2, true, true, 100, 0));
}

// ---- run_encode_pass ------------------------------------------------------------

// A panel that paints its whole cell with one byte per tick: the clock role
// paints the first payload byte (so the encoded frame reflects the tick's
// message), a follower paints the tick's index (so every panel proves it
// was driven once per tick). Both report a 2x2 cell.
class StubPanel final : public Panel
{
public:
  StubPanel(bool clock, std::vector<std::uint64_t> * ticks_seen)
  : clock_(clock), ticks_seen_(ticks_seen)
  {
  }

  std::string select(const TickInfo & tick, PanelSize) override
  {
    ticks_seen_->push_back(tick.index);
    value_ = clock_ ? tick.payload.front() : static_cast<std::byte>(tick.index);
    return "";
  }

  std::optional<PanelSize> clock_cell_size() const override
  {
    if (!clock_) {
      return std::nullopt;
    }
    return PanelSize{2, 2};
  }

  std::string render(const CellView & cell) override
  {
    for (std::uint32_t y = 0; y < cell.height; ++y) {
      std::fill_n(cell.data + y * cell.stride, static_cast<std::size_t>(cell.width) * 3U, value_);
    }
    return "";
  }

private:
  bool clock_;
  std::vector<std::uint64_t> * ticks_seen_;
  std::byte value_{0};
};

// A bag whose clock topic carries `frames` one-byte payloads (0x10, 0x11, ...)
// at 100 ms spacing.
std::filesystem::path write_clock_bag(const std::filesystem::path & dir, int frames)
{
  const auto path = dir / "clock.mcap";
  auto w = bagwiz::io::open_write(path, movify_mcap_options());
  movify_declare_topic(*w, "/clock", kMovifyImageType);
  for (int i = 0; i < frames; ++i) {
    const std::array<std::byte, 1> payload{static_cast<std::byte>(0x10 + i)};
    w->write("/clock", 1'000'000'000LL + i * 100'000'000LL, payload);
  }
  w->close();
  return path;
}

std::unique_ptr<bagwiz::io::BagReader> open_clock_reader(const std::filesystem::path & bag)
{
  auto reader = bagwiz::io::open_read(bag);
  bagwiz::io::ReadFilter filter;
  filter.topics.push_back("/clock");
  reader->set_filter(filter);
  return reader;
}

// Encode `frames` ticks through `parallel` or the synchronous loop and return
// the output's probe plus the ticks every panel saw.
struct PassResult
{
  int exit_code = 0;
  bagwiz::core::video::VideoProbe probe;
  std::vector<std::uint64_t> clock_ticks;
  std::vector<std::uint64_t> follower_ticks;
};

// `clock` is the grid index the clock panel takes; the follower takes the
// other cell.
PassResult run_pass(
  const std::filesystem::path & dir, int frames, bool parallel, std::size_t clock = 0)
{
  PassResult result;
  const auto bag = write_clock_bag(dir, frames);
  auto reader = open_clock_reader(bag);
  std::vector<std::unique_ptr<Panel>> panels;
  for (std::size_t i = 0; i < 2; ++i) {
    const bool is_clock = i == clock;
    panels.push_back(
      std::make_unique<StubPanel>(
        is_clock, is_clock ? &result.clock_ticks : &result.follower_ticks));
  }
  const auto output = dir / (parallel ? "parallel.avi" : "sync.avi");
  VideoFrameEncoder encoder(output, bagwiz::core::video::FrameRate{10, 1});
  result.exit_code =
    run_encode_pass(*reader, panels, GridSpec{2, 1}, parallel, clock, "/clock", encoder);
  if (result.exit_code == 0 && encoder.finish().empty()) {
    result.probe = bagwiz::core::video::probe_video(output);
  }
  return result;
}

TEST_F(MovifyTmpDirTest, SynchronousLoopDrivesEveryPanelOncePerTick)
{
  const auto r = run_pass(tmp_dir_, 5, /*parallel=*/false);
  ASSERT_EQ(r.exit_code, 0);
  ASSERT_TRUE(r.probe.ok()) << r.probe.error;
  EXPECT_EQ(r.probe.frame_count, 5);
  EXPECT_EQ(r.probe.width, 4u);  // 2x1 grid of the stub's 2x2 cells
  EXPECT_EQ(r.probe.height, 2u);
  EXPECT_EQ(r.clock_ticks, std::vector<std::uint64_t>({0, 1, 2, 3, 4}));
  EXPECT_EQ(r.follower_ticks, std::vector<std::uint64_t>({0, 1, 2, 3, 4}));
}

TEST_F(MovifyTmpDirTest, ParallelLoopDrivesEveryPanelOncePerTick)
{
  const auto r = run_pass(tmp_dir_, 5, /*parallel=*/true);
  ASSERT_EQ(r.exit_code, 0);
  ASSERT_TRUE(r.probe.ok()) << r.probe.error;
  EXPECT_EQ(r.probe.frame_count, 5);
  EXPECT_EQ(r.probe.width, 4u);
  EXPECT_EQ(r.probe.height, 2u);
  EXPECT_EQ(r.clock_ticks, std::vector<std::uint64_t>({0, 1, 2, 3, 4}));
  EXPECT_EQ(r.follower_ticks, std::vector<std::uint64_t>({0, 1, 2, 3, 4}));
}

// The clock panel may sit in any grid cell: with the clock at index 1 both
// loops still drive it first on every tick and the follower in cell 0.
TEST_F(MovifyTmpDirTest, ClockPanelMayTakeAnyCell)
{
  for (const bool parallel : {false, true}) {
    const auto r = run_pass(tmp_dir_, 3, parallel, /*clock=*/1);
    ASSERT_EQ(r.exit_code, 0) << (parallel ? "parallel" : "sync");
    ASSERT_TRUE(r.probe.ok()) << r.probe.error;
    EXPECT_EQ(r.probe.frame_count, 3);
    EXPECT_EQ(r.clock_ticks, std::vector<std::uint64_t>({0, 1, 2}));
    EXPECT_EQ(r.follower_ticks, std::vector<std::uint64_t>({0, 1, 2}));
  }
}

TEST_F(MovifyTmpDirTest, ParallelLoopReportsAClockTopicWithoutFrames)
{
  const auto bag = write_clock_bag(tmp_dir_, 0);
  auto reader = open_clock_reader(bag);
  std::vector<std::uint64_t> ticks;
  std::vector<std::unique_ptr<Panel>> panels;
  panels.push_back(std::make_unique<StubPanel>(true, &ticks));
  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(
    run_encode_pass(*reader, panels, GridSpec{1, 1}, /*parallel=*/true, 0, "/clock", encoder), 1);
  EXPECT_FALSE(encoder.started());
  EXPECT_TRUE(ticks.empty());
}

// A synchronous pass over an empty clock topic composes nothing and leaves
// the "no frames" report to finish_video_encode, which sees an encoder that never
// started.
TEST_F(MovifyTmpDirTest, SynchronousLoopOverAnEmptyClockTopicStartsNoEncoder)
{
  const auto bag = write_clock_bag(tmp_dir_, 0);
  auto reader = open_clock_reader(bag);
  std::vector<std::uint64_t> ticks;
  std::vector<std::unique_ptr<Panel>> panels;
  panels.push_back(std::make_unique<StubPanel>(true, &ticks));
  VideoFrameEncoder encoder(tmp_dir_ / "out.avi", bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(
    run_encode_pass(*reader, panels, GridSpec{1, 1}, /*parallel=*/false, 0, "/clock", encoder), 0);
  EXPECT_FALSE(encoder.started());
}

}  // namespace
