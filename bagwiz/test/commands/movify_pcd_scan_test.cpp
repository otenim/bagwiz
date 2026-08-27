// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/movify_pcd_scan.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "movify_pcd_scan_common.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "topic_slot_test_util.hpp"    // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{
using bagwiz::commands::MovifyPcdScanArgs;
using bagwiz::commands::run_movify_pcd_scan;
using bagwiz::commands::scan_frames_per_sweep;
using bagwiz::commands::validate_pcd_scan_inputs;

// Little-endian CDR-1 builder, matching the wire format the production reader
// consumes (movify_video_test.cpp's CdrBuilder idiom).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }
  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

constexpr const char * kPcdTopic = "/points_raw";
constexpr const char * kPcdType = "sensor_msgs/msg/PointCloud2";

// Serialize a sensor_msgs/msg/PointCloud2 with float32 x/y/z fields plus a
// float32 "t" per-point time field (seconds). When `with_time` is false the
// "t" field is omitted and point_step shrinks accordingly.
std::vector<std::byte> make_cloud_payload(
  std::int64_t timestamp_ns, const std::vector<std::array<float, 3>> & points,
  const std::vector<float> & times, bool with_time = true)
{
  const std::uint32_t point_step = with_time ? 16 : 12;
  std::vector<std::byte> data(points.size() * point_step, std::byte{0});
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(data.data() + i * point_step, points[i].data(), 12);
    if (with_time) {
      std::memcpy(data.data() + i * point_step + 12, &times[i], sizeof(float));
    }
  }

  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL));   // sec
  b.u32(static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL));  // nanosec
  b.str("lidar");
  b.u32(1);                                          // height
  b.u32(static_cast<std::uint32_t>(points.size()));  // width
  b.u32(with_time ? 4 : 3);                          // fields length
  b.str("x");
  b.u32(0);
  b.u8(7);  // float32
  b.u32(1);
  b.str("y");
  b.u32(4);
  b.u8(7);
  b.u32(1);
  b.str("z");
  b.u32(8);
  b.u8(7);
  b.u32(1);
  if (with_time) {
    b.str("t");
    b.u32(12);
    b.u8(7);
    b.u32(1);
  }
  b.u8(0);                                         // is_bigendian
  b.u32(point_step);                               // point_step
  b.u32(static_cast<std::uint32_t>(data.size()));  // row_step
  b.byte_seq({data.data(), data.size()});
  b.u8(1);  // is_dense
  return b.take();
}

bagwiz::io::TopicInfo make_topic(std::string name, std::string type)
{
  bagwiz::io::TopicInfo t;
  t.name = std::move(name);
  t.type = std::move(type);
  t.serialization_format = "cdr";
  return t;
}

bagwiz::io::CreateOptions mcap_dir_opts()
{
  bagwiz::io::CreateOptions opts;
  opts.format = bagwiz::io::Format::Mcap;
  opts.layout = bagwiz::io::Layout::Directory;
  opts.mcap_compression = "none";
  return opts;
}

// One sweep: a quarter ring of points at 10-40 m, fired at 0/10/20/30 ms.
const std::vector<std::array<float, 3>> kPoints = {
  {10.0F, 0.0F, 0.0F}, {0.0F, 20.0F, 0.0F}, {-30.0F, 0.0F, 0.0F}, {0.0F, -40.0F, 0.0F}};
const std::vector<float> kTimes = {0.0F, 0.01F, 0.02F, 0.03F};

// Build an MCAP bag with `clouds` PointCloud2 messages at 100 ms spacing on
// kPcdTopic (10 Hz). With `with_time` false the clouds carry no per-point time.
std::filesystem::path build_bag(
  const std::filesystem::path & dir, int clouds, bool with_time = true,
  const std::string & topic_type = kPcdType)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  writer->declare_topic(make_topic(kPcdTopic, topic_type));
  for (int i = 0; i < clouds; ++i) {
    const std::int64_t ts = 1'000'000'000LL + static_cast<std::int64_t>(i) * 100'000'000LL;
    writer->write(kPcdTopic, ts, make_cloud_payload(ts, kPoints, kTimes, with_time));
  }
  writer->close();
  return path;
}

class MovifyPcdScanTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static int counter = 0;
    dir_ = std::filesystem::temp_directory_path() /
           ("bagwiz_pcd_scan_test_" + std::to_string(++counter));
    std::filesystem::create_directories(dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  std::filesystem::path dir_;
};

TEST_F(MovifyPcdScanTest, ValidateDetectsTimeField)
{
  const auto bag = build_bag(dir_, 3);
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  const auto validation = validate_pcd_scan_inputs(args);
  ASSERT_TRUE(validation.ok()) << validation.error;
  EXPECT_EQ(validation.time_field.offset, 12u);
}

TEST_F(MovifyPcdScanTest, RunTopicNotFoundFails)
{
  const auto bag = build_bag(dir_, 3);
  MovifyPcdScanArgs args{bag, "/no/such/topic", dir_ / "out.mp4", false};
  EXPECT_EQ(run_movify_pcd_scan(args), 1);
  EXPECT_FALSE(std::filesystem::exists(dir_ / "out.mp4"));
}

TEST_F(MovifyPcdScanTest, RunWrongTypeFails)
{
  const auto bag = build_bag(dir_, 3, true, "sensor_msgs/msg/Image");
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  EXPECT_EQ(run_movify_pcd_scan(args), 1);
}

TEST_F(MovifyPcdScanTest, RunMissingTimeFieldFails)
{
  const auto bag = build_bag(dir_, 3, /*with_time=*/false);
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  EXPECT_EQ(run_movify_pcd_scan(args), 1);
  EXPECT_FALSE(std::filesystem::exists(dir_ / "out.mp4"));
}

TEST_F(MovifyPcdScanTest, RunOddDimensionsFails)
{
  const auto bag = build_bag(dir_, 3);
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  args.width = 321;
  EXPECT_EQ(run_movify_pcd_scan(args), 1);
}

TEST_F(MovifyPcdScanTest, RunExistingOutputWithoutOverwriteFails)
{
  const auto bag = build_bag(dir_, 3);
  const auto out = dir_ / "out.mp4";
  {
    std::ofstream f(out);
    f << "pre-existing";
  }
  MovifyPcdScanArgs args{bag, kPcdTopic, out, false};
  EXPECT_EQ(run_movify_pcd_scan(args), 1);
}

TEST_F(MovifyPcdScanTest, RunEncodesBevVideo)
{
  // 5 clouds at 10 Hz, fps 40, speed 1.0 -> 4 frames/sweep, 20 frames at
  // 40 fps = 0.5 s.
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  MovifyPcdScanArgs args{bag, kPcdTopic, out, false};
  args.view = bagwiz::core::pointcloud::ScanPatternProjection::kBev;
  args.fps = 40;
  args.speed = 1.0;
  args.width = 320;
  args.height = 240;
  ASSERT_EQ(run_movify_pcd_scan(args), 0);
  ASSERT_TRUE(std::filesystem::exists(out));

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 320u);
  EXPECT_EQ(probe.height, 240u);
  EXPECT_EQ(probe.frame_count, 20);
  EXPECT_NEAR(probe.duration_s, 0.5, 0.5);
}

TEST_F(MovifyPcdScanTest, RunDefaultSpeedPlaysAtOneTenthRealTime)
{
  // Default speed 0.1: 5 clouds at 10 Hz, fps 4 -> 4 frames/sweep, 20 frames
  // at 4 fps = 5 s (the recording's 0.5 s stretched tenfold).
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  MovifyPcdScanArgs args{bag, kPcdTopic, out, false};
  args.fps = 4;
  args.width = 320;
  args.height = 240;
  ASSERT_EQ(run_movify_pcd_scan(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 20);
  EXPECT_NEAR(probe.duration_s, 5.0, 1.0);
}

TEST_F(MovifyPcdScanTest, RunEncodesPerspectiveVideo)
{
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  MovifyPcdScanArgs args{bag, kPcdTopic, out, false};
  args.view = bagwiz::core::pointcloud::ScanPatternProjection::kPerspective;
  args.fps = 2;  // 2 frames/sweep at 10 Hz and the default speed 0.1
  args.width = 320;
  args.height = 240;
  args.range_m = 50.0;
  args.dist_m = 100.0;
  ASSERT_EQ(run_movify_pcd_scan(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 10);
}

TEST_F(MovifyPcdScanTest, RunOverwriteReplacesExistingOutput)
{
  const auto bag = build_bag(dir_, 3);
  const auto out = dir_ / "out.mp4";
  {
    std::ofstream f(out);
    f << "pre-existing";
  }
  MovifyPcdScanArgs args{bag, kPcdTopic, out, true};
  args.width = 64;
  args.height = 64;
  ASSERT_EQ(run_movify_pcd_scan(args), 0);
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  // 3 clouds * 60 frames/sweep (default fps 60 at 10 Hz, default speed 0.1)
  EXPECT_EQ(probe.frame_count, 180);
}

// `scan` reaches an oversized canvas through explicit --width/--height, so the
// same guard reports it — before pass 1, since the size is known from the flags
// alone. One frame at 3842x2160 keeps the encode cheap.
TEST_F(MovifyPcdScanTest, RunWarnsWhenTheOutputIsOversized)
{
  const auto bag = build_bag(dir_, 2);
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  args.fps = 1;
  args.speed = 1.0;
  args.width = 3842;  // 3842x2160 = 8.30 Mpx, just past 4K's 8.29 Mpx.
  args.height = 2160;

  ::testing::internal::CaptureStderr();
  ASSERT_EQ(run_movify_pcd_scan(args), 0);
  const std::string err = ::testing::internal::GetCapturedStderr();

  EXPECT_NE(err.find("3842x2160"), std::string::npos) << err;
  EXPECT_NE(err.find("larger than 3840x2160"), std::string::npos) << err;
  // scan has no grid to explain, so the parenthetical carries only the count.
  EXPECT_NE(err.find("(8.3 Mpx)"), std::string::npos) << err;
}

TEST_F(MovifyPcdScanTest, RunStaysQuietForAnOrdinaryOutputSize)
{
  const auto bag = build_bag(dir_, 2);
  MovifyPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  args.fps = 1;
  args.speed = 1.0;
  args.width = 320;
  args.height = 240;

  ::testing::internal::CaptureStderr();
  ASSERT_EQ(run_movify_pcd_scan(args), 0);
  const std::string err = ::testing::internal::GetCapturedStderr();

  EXPECT_EQ(err.find("larger than"), std::string::npos) << err;
}

TEST(ScanFramesPerSweep, FramesPerSweepFromFpsAndSpeed)
{
  // 10 Hz clouds, 60 fps, speed 0.1 -> 60 frames per sweep.
  EXPECT_EQ(scan_frames_per_sweep({10, 1}, 60, 0.1), 60u);
}

TEST(ScanFramesPerSweep, RealTimeSpeed)
{
  // 10 Hz clouds, 30 fps, speed 1.0 -> 3 frames per sweep.
  EXPECT_EQ(scan_frames_per_sweep({10, 1}, 30, 1.0), 3u);
}

TEST(ScanFramesPerSweep, FractionalCloudRate)
{
  // 12.5 Hz clouds, 50 fps, speed 1.0 -> 4 frames per sweep.
  EXPECT_EQ(scan_frames_per_sweep({25, 2}, 50, 1.0), 4u);
}

TEST(ScanFramesPerSweep, RoundsToNearest)
{
  // 3 Hz clouds, 10 fps, speed 1.0 -> 10/3 = 3.33 -> 3 frames per sweep.
  EXPECT_EQ(scan_frames_per_sweep({3, 1}, 10, 1.0), 3u);
}

TEST(ScanFramesPerSweep, FloorsAtOneFramePerSweep)
{
  // 100 Hz clouds, 10 fps, speed 1.0 -> 0.1 -> 1 frame per sweep.
  EXPECT_EQ(scan_frames_per_sweep({100, 1}, 10, 1.0), 1u);
}

// Exercises the real MovifyCommand::configure_pcd_scan() — reached through
// the process-wide command registry that movify.cpp's BAGWIZ_REGISTER_COMMAND
// registrar populates — rather than a hand-mirrored copy of its wiring
// (MovifyVideoCliWiring idiom).
TEST(MovifyPcdScanCliWiring, TopicOptionIsDeclaredLiteralPointCloud2)
{
  bagwiz::commands::Command * movify_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "movify") {
      movify_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(movify_cmd, nullptr);

  CLI::App app{"movify"};
  movify_cmd->configure(app);

  auto * pcd_scan_sub = app.get_subcommand_no_throw("scan");
  ASSERT_NE(pcd_scan_sub, nullptr);
  const auto slots = bagwiz::commands::topic_slots_of(*pcd_scan_sub);
  ASSERT_EQ(slots.size(), 1U);  // -t/--topic only

  const auto * topic_slot = bagwiz::test::slot_for(slots, "topic");
  ASSERT_NE(topic_slot, nullptr);
  EXPECT_EQ(topic_slot->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
  ASSERT_EQ(topic_slot->spec.allowed_types.size(), 1U);
  EXPECT_EQ(topic_slot->spec.allowed_types[0], "sensor_msgs/msg/PointCloud2");
  EXPECT_TRUE(topic_slot->option->get_required());
}

}  // namespace
