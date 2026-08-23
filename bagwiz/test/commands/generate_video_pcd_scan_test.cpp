// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video_pcd_scan.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/core/video/video_encoder.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "generate_video_pcd_scan_common.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "topic_slot_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

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
using bagwiz::commands::derive_scan_frame_rate;
using bagwiz::commands::GenerateVideoPcdScanArgs;
using bagwiz::commands::run_generate_video_pcd_scan;
using bagwiz::commands::validate_pcd_scan_inputs;

// Little-endian CDR-1 builder, matching the wire format the production reader
// consumes (generate_video_test.cpp's CdrBuilder idiom).
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

class GenerateVideoPcdScanTest : public ::testing::Test
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

TEST_F(GenerateVideoPcdScanTest, ValidateDetectsTimeField)
{
  const auto bag = build_bag(dir_, 3);
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  const auto validation = validate_pcd_scan_inputs(args);
  ASSERT_TRUE(validation.ok()) << validation.error;
  EXPECT_EQ(validation.time_field.offset, 12u);
}

TEST_F(GenerateVideoPcdScanTest, RunTopicNotFoundFails)
{
  const auto bag = build_bag(dir_, 3);
  GenerateVideoPcdScanArgs args{bag, "/no/such/topic", dir_ / "out.mp4", false};
  EXPECT_EQ(run_generate_video_pcd_scan(args), 1);
  EXPECT_FALSE(std::filesystem::exists(dir_ / "out.mp4"));
}

TEST_F(GenerateVideoPcdScanTest, RunWrongTypeFails)
{
  const auto bag = build_bag(dir_, 3, true, "sensor_msgs/msg/Image");
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  EXPECT_EQ(run_generate_video_pcd_scan(args), 1);
}

TEST_F(GenerateVideoPcdScanTest, RunMissingTimeFieldFails)
{
  const auto bag = build_bag(dir_, 3, /*with_time=*/false);
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  EXPECT_EQ(run_generate_video_pcd_scan(args), 1);
  EXPECT_FALSE(std::filesystem::exists(dir_ / "out.mp4"));
}

TEST_F(GenerateVideoPcdScanTest, RunOddDimensionsFails)
{
  const auto bag = build_bag(dir_, 3);
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, dir_ / "out.mp4", false};
  args.width = 321;
  EXPECT_EQ(run_generate_video_pcd_scan(args), 1);
}

TEST_F(GenerateVideoPcdScanTest, RunExistingOutputWithoutOverwriteFails)
{
  const auto bag = build_bag(dir_, 3);
  const auto out = dir_ / "out.mp4";
  {
    std::ofstream f(out);
    f << "pre-existing";
  }
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, out, false};
  EXPECT_EQ(run_generate_video_pcd_scan(args), 1);
}

TEST_F(GenerateVideoPcdScanTest, RunEncodesBevVideo)
{
  // 5 clouds at 10 Hz, steps 4, speed 1.0 -> 20 frames at 40 fps = 0.5 s.
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, out, false};
  args.view = bagwiz::core::pointcloud::ScanPatternProjection::kBev;
  args.steps = 4;
  args.speed = 1.0;
  args.width = 320;
  args.height = 240;
  ASSERT_EQ(run_generate_video_pcd_scan(args), 0);
  ASSERT_TRUE(std::filesystem::exists(out));

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.width, 320u);
  EXPECT_EQ(probe.height, 240u);
  EXPECT_EQ(probe.frame_count, 20);
  EXPECT_NEAR(probe.duration_s, 0.5, 0.5);
}

TEST_F(GenerateVideoPcdScanTest, RunDefaultSpeedPlaysAtOneTenthRealTime)
{
  // Default speed 0.1: 5 clouds at 10 Hz, steps 4 -> 20 frames at 4 fps = 5 s
  // (the recording's 0.5 s stretched tenfold).
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, out, false};
  args.steps = 4;
  args.width = 320;
  args.height = 240;
  ASSERT_EQ(run_generate_video_pcd_scan(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 20);
  EXPECT_NEAR(probe.duration_s, 5.0, 1.0);
}

TEST_F(GenerateVideoPcdScanTest, RunEncodesPerspectiveVideo)
{
  const auto bag = build_bag(dir_, 5);
  const auto out = dir_ / "out.mp4";
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, out, false};
  args.view = bagwiz::core::pointcloud::ScanPatternProjection::kPerspective;
  args.steps = 2;
  args.width = 320;
  args.height = 240;
  args.range_m = 50.0;
  args.dist_m = 100.0;
  ASSERT_EQ(run_generate_video_pcd_scan(args), 0);

  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 10);
}

TEST_F(GenerateVideoPcdScanTest, RunOverwriteReplacesExistingOutput)
{
  const auto bag = build_bag(dir_, 3);
  const auto out = dir_ / "out.mp4";
  {
    std::ofstream f(out);
    f << "pre-existing";
  }
  GenerateVideoPcdScanArgs args{bag, kPcdTopic, out, true};
  args.width = 64;
  args.height = 64;
  ASSERT_EQ(run_generate_video_pcd_scan(args), 0);
  const auto probe = bagwiz::core::video::probe_video(out);
  ASSERT_TRUE(probe.ok()) << probe.error;
  EXPECT_EQ(probe.frame_count, 150);  // 3 clouds * default 50 steps
}

TEST(DeriveScanFrameRate, MultipliesStepsIntoTheRate)
{
  // 10 Hz clouds * 10 steps * speed 1.0 = 100 fps.
  const auto r = derive_scan_frame_rate({10, 1}, 10, 1.0);
  EXPECT_EQ(r.steps, 10u);
  EXPECT_EQ(r.fps.num, 100);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, AppliesSpeedToTheRate)
{
  // 10 Hz clouds * 10 steps * speed 0.1 = 10 fps.
  const auto r = derive_scan_frame_rate({10, 1}, 10, 0.1);
  EXPECT_EQ(r.steps, 10u);
  EXPECT_EQ(r.fps.num, 10);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, ReducesTheFraction)
{
  // 25/2 fps * 4 * 1.0 = 50 fps exactly.
  const auto r = derive_scan_frame_rate({25, 2}, 4, 1.0);
  EXPECT_EQ(r.steps, 4u);
  EXPECT_EQ(r.fps.num, 50);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, ClampsToMaxFps)
{
  // 30 Hz * 10 * 1.0 = 300 > 240 -> steps reduced to 8 (240 fps).
  const auto r = derive_scan_frame_rate({30, 1}, 10, 1.0);
  EXPECT_EQ(r.steps, 8u);
  EXPECT_EQ(r.fps.num, 240);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, SpeedFactorsIntoTheStepsClamp)
{
  // 30 Hz * 10 * 0.5 = 150 fps <= 240 -> steps kept.
  const auto r = derive_scan_frame_rate({30, 1}, 10, 0.5);
  EXPECT_EQ(r.steps, 10u);
  EXPECT_EQ(r.fps.num, 150);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, ClampsToMinFps)
{
  // 1 Hz * 1 * 0.001 = 0.001 fps < 1 -> clamped to 1 fps.
  const auto r = derive_scan_frame_rate({1, 1}, 1, 0.001);
  EXPECT_EQ(r.steps, 1u);
  EXPECT_EQ(r.fps.num, 1);
  EXPECT_EQ(r.fps.den, 1);
}

TEST(DeriveScanFrameRate, NeverDropsBelowOneStep)
{
  const auto r = derive_scan_frame_rate({240, 1}, 10, 1.0);
  EXPECT_EQ(r.steps, 1u);
  EXPECT_EQ(r.fps.num, 240);
  EXPECT_EQ(r.fps.den, 1);
}

// Exercises the real GenerateCommand::configure_pcd_scan() — reached through
// the process-wide command registry that generate.cpp's BAGWIZ_REGISTER_COMMAND
// registrar populates — rather than a hand-mirrored copy of its wiring
// (GenerateVideoCliWiring idiom).
TEST(GenerateVideoPcdScanCliWiring, TopicOptionIsDeclaredLiteralPointCloud2)
{
  bagwiz::commands::Command * generate_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "generate") {
      generate_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(generate_cmd, nullptr);

  CLI::App app{"generate"};
  generate_cmd->configure(app);

  auto * video_group = app.get_subcommand_no_throw("video");
  ASSERT_NE(video_group, nullptr);
  auto * pcd_scan_sub = video_group->get_subcommand_no_throw("scan");
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
