// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "generate_video_common.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/io/bag_io.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// Unit tests for the generate-video internals: tmp-path handling, the RAII
// partial-file guard, finalize (rename/clobber), grid + per-view binding
// parsing, input validation, the pass-1 scan, the threading decision, frame
// decode / resize, the secondary-frame nearest matcher, the grid canvas, and
// the per-view renderer. Exercises generate_video_common.hpp directly without
// driving the CLI.

namespace
{

using bagwiz::commands::auto_grid_spec;
using bagwiz::commands::finalize_video_output;
using bagwiz::commands::finish_video_encode;
using bagwiz::commands::FrameBuffer;
using bagwiz::commands::FrameNormalizer;
using bagwiz::commands::GridCanvas;
using bagwiz::commands::GridSpec;
using bagwiz::commands::load_video_geometry;
using bagwiz::commands::NearestMessageSource;
using bagwiz::commands::open_encode_reader;
using bagwiz::commands::parse_cam_info_entries;
using bagwiz::commands::parse_grid_spec;
using bagwiz::commands::parse_pcd_bindings;
using bagwiz::commands::partial_tmp_path_for;
using bagwiz::commands::PartialFileGuard;
using bagwiz::commands::resize_frame;
using bagwiz::commands::scan_video_inputs;
using bagwiz::commands::should_use_threaded_projection;
using bagwiz::commands::validate_video_inputs;
using bagwiz::commands::validate_video_output_path;
using bagwiz::commands::VideoOverlayParams;
using bagwiz::commands::ViewRenderer;

constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";
constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

// ---- tmp dir fixture --------------------------------------------------------

class GenerateVideoCommonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("bagwiz_gvc_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(tmp_dir_);
  }
  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

// ---- bag fixture helpers ----------------------------------------------------

bagwiz::io::CreateOptions mcap_options()
{
  bagwiz::io::CreateOptions o;
  o.format = bagwiz::io::Format::Mcap;
  o.layout = bagwiz::io::Layout::SingleFile;
  o.mcap_compression = "none";
  return o;
}

void declare_topic(bagwiz::io::BagWriter & w, const std::string & name, const std::string & type)
{
  bagwiz::io::TopicInfo info;
  info.name = name;
  info.type = type;
  info.serialization_format = "cdr";
  w.declare_topic(info);
}

// A bag with a single raw-image topic and `frames` garbage-payload messages at
// 100 ms spacing starting at 1 s (scan reads timestamps only, never payloads).
std::filesystem::path write_image_bag(
  const std::filesystem::path & dir, const std::string & name, int frames)
{
  const auto path = dir / name;
  auto w = bagwiz::io::open_write(path, mcap_options());
  declare_topic(*w, "/cam/image", kImageType);
  const std::array<std::byte, 4> garbage{
    std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  for (int i = 0; i < frames; ++i) {
    w->write("/cam/image", 1'000'000'000LL + i * 100'000'000LL, garbage);
  }
  w->close();
  return path;
}

// ---- minimal CDR builder for the image decode fixture -----------------------

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
  std::vector<std::byte> take() { return std::move(buf_); }

private:
  void align(std::size_t n)
  {
    while (buf_.size() % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_bgr8_image_payload(std::uint32_t w, std::uint32_t h, std::uint8_t fill)
{
  std::vector<std::byte> data(static_cast<std::size_t>(w) * h * 3, std::byte{fill});
  CdrBuilder b;
  b.i32(0);  // header.stamp.sec
  b.u32(0);  // header.stamp.nanosec
  b.str("cam");
  b.u32(h);
  b.u32(w);
  b.str("bgr8");
  b.u8(0);       // is_bigendian
  b.u32(w * 3);  // step
  b.byte_seq({data.data(), data.size()});
  return b.take();
}

void write_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out << content;
}

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// ---- partial_tmp_path_for ---------------------------------------------------

TEST(PartialTmpPath, KeepsExtensionAfterMarker)
{
  EXPECT_EQ(
    partial_tmp_path_for("/dir/out.avi"), std::filesystem::path("/dir") / "out.bagwiz-partial.avi");
}

TEST(PartialTmpPath, MultiDotNameKeepsOnlyFinalExtension)
{
  EXPECT_EQ(partial_tmp_path_for("a.b.mp4"), std::filesystem::path("a.b.bagwiz-partial.mp4"));
}

TEST(PartialTmpPath, NoExtensionAppendsMarker)
{
  EXPECT_EQ(partial_tmp_path_for("/dir/out"), std::filesystem::path("/dir") / "out.bagwiz-partial");
}

// ---- auto_grid_spec ---------------------------------------------------------

TEST(AutoGridSpec, NearSquareLayouts)
{
  const auto g1 = auto_grid_spec(1);
  EXPECT_EQ(g1.cols, 1u);
  EXPECT_EQ(g1.rows, 1u);
  const auto g2 = auto_grid_spec(2);
  EXPECT_EQ(g2.cols, 2u);
  EXPECT_EQ(g2.rows, 1u);
  const auto g3 = auto_grid_spec(3);
  EXPECT_EQ(g3.cols, 2u);
  EXPECT_EQ(g3.rows, 2u);
  const auto g4 = auto_grid_spec(4);
  EXPECT_EQ(g4.cols, 2u);
  EXPECT_EQ(g4.rows, 2u);
  const auto g5 = auto_grid_spec(5);
  EXPECT_EQ(g5.cols, 3u);
  EXPECT_EQ(g5.rows, 2u);
  const auto g6 = auto_grid_spec(6);
  EXPECT_EQ(g6.cols, 3u);
  EXPECT_EQ(g6.rows, 2u);
  const auto g7 = auto_grid_spec(7);
  EXPECT_EQ(g7.cols, 3u);
  EXPECT_EQ(g7.rows, 3u);
}

// ---- parse_grid_spec --------------------------------------------------------

TEST(ParseGridSpec, EmptySelectsAuto)
{
  const auto r = parse_grid_spec("", 3);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.grid.cols, 2u);
  EXPECT_EQ(r.grid.rows, 2u);
}

TEST(ParseGridSpec, ParsesColsByRows)
{
  const auto r = parse_grid_spec("3x1", 3);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.grid.cols, 3u);
  EXPECT_EQ(r.grid.rows, 1u);
}

TEST(ParseGridSpec, ExtraCellsAllowed)
{
  const auto r = parse_grid_spec("2x2", 3);
  EXPECT_TRUE(r.ok()) << r.error;
}

TEST(ParseGridSpec, RejectsTooFewCells)
{
  const auto r = parse_grid_spec("1x1", 2);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("cell(s) for"), std::string::npos);
}

TEST(ParseGridSpec, RejectsMalformedValues)
{
  for (const char * text : {"2", "2x", "x2", "axb", "2x2x2", "0x2", "2x0", "-1x2"}) {
    EXPECT_FALSE(parse_grid_spec(text, 1).ok()) << text;
  }
}

// ---- parse_pcd_bindings -----------------------------------------------------

TEST(ParsePcdBindings, BareValuesAreGlobal)
{
  const std::vector<std::string> entries{"/points/front", "/points/rear"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_pcd_bindings(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.global_topics, entries);
  EXPECT_TRUE(r.per_view.empty());
}

TEST(ParsePcdBindings, PairBindsToOneView)
{
  const std::vector<std::string> entries{"/cam/a=/points/left", "/cam/a=/points/right"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_pcd_bindings(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_TRUE(r.global_topics.empty());
  ASSERT_EQ(r.per_view.count("/cam/a"), 1u);
  EXPECT_EQ(r.per_view.at("/cam/a"), std::vector<std::string>({"/points/left", "/points/right"}));
}

TEST(ParsePcdBindings, RejectsUnknownImageTopic)
{
  const std::vector<std::string> entries{"/cam/nope=/points"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_pcd_bindings(entries, images);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("not one of the -t/--topic topics"), std::string::npos);
}

TEST(ParsePcdBindings, RejectsEmptyHalves)
{
  const std::vector<std::string> images{"/cam/a"};
  for (const char * entry : {"=/points", "/cam/a=", "="}) {
    const std::vector<std::string> entries{entry};
    EXPECT_FALSE(parse_pcd_bindings(entries, images).ok()) << entry;
  }
}

// ---- parse_cam_info_entries -------------------------------------------------

TEST(ParseCamInfoEntries, BareValueIsGlobal)
{
  const std::vector<std::string> entries{"/cam/camera_info"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_cam_info_entries(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_TRUE(r.global_topic.has_value());
  EXPECT_EQ(*r.global_topic, "/cam/camera_info");
  EXPECT_TRUE(r.per_view.empty());
}

TEST(ParseCamInfoEntries, PairOverridesOneView)
{
  const std::vector<std::string> entries{"/cam/a=/cam/a_info"};
  const std::vector<std::string> images{"/cam/a", "/cam/b"};
  const auto r = parse_cam_info_entries(entries, images);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_FALSE(r.global_topic.has_value());
  ASSERT_EQ(r.per_view.count("/cam/a"), 1u);
  EXPECT_EQ(r.per_view.at("/cam/a"), "/cam/a_info");
}

TEST(ParseCamInfoEntries, RejectsSecondBareValue)
{
  const std::vector<std::string> entries{"/cam/info_a", "/cam/info_b"};
  const std::vector<std::string> images{"/cam/a"};
  EXPECT_FALSE(parse_cam_info_entries(entries, images).ok());
}

TEST(ParseCamInfoEntries, RejectsDuplicateOverride)
{
  const std::vector<std::string> entries{"/cam/a=/cam/i1", "/cam/a=/cam/i2"};
  const std::vector<std::string> images{"/cam/a"};
  const auto r = parse_cam_info_entries(entries, images);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("duplicate override"), std::string::npos);
}

TEST(ParseCamInfoEntries, RejectsUnknownImageTopicAndEmptyHalves)
{
  const std::vector<std::string> images{"/cam/a"};
  for (const char * entry : {"/cam/nope=/cam/i", "=/cam/i", "/cam/a="}) {
    const std::vector<std::string> entries{entry};
    EXPECT_FALSE(parse_cam_info_entries(entries, images).ok()) << entry;
  }
}

// ---- should_use_threaded_projection -----------------------------------------

TEST(ShouldUseThreadedProjection, RequiresPointClouds)
{
  EXPECT_FALSE(should_use_threaded_projection(false, true, 100, 8));
}

TEST(ShouldUseThreadedProjection, RespectsDisableFlag)
{
  EXPECT_FALSE(should_use_threaded_projection(true, false, 100, 8));
}

TEST(ShouldUseThreadedProjection, RequiresEnoughFrames)
{
  EXPECT_FALSE(should_use_threaded_projection(true, true, 3, 8));
  EXPECT_TRUE(should_use_threaded_projection(true, true, 4, 8));
}

TEST(ShouldUseThreadedProjection, RequiresParallelHardware)
{
  EXPECT_FALSE(should_use_threaded_projection(true, true, 100, 1));
  EXPECT_FALSE(should_use_threaded_projection(true, true, 100, 0));
}

// ---- PartialFileGuard -------------------------------------------------------

TEST_F(GenerateVideoCommonTest, PartialFileGuardCtorRemovesStaleFile)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  write_file(tmp, "stale");
  {
    PartialFileGuard guard(tmp);
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(GenerateVideoCommonTest, PartialFileGuardDtorRemovesLeftover)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  {
    PartialFileGuard guard(tmp);
    write_file(tmp, "partial");  // simulate an aborted encode
  }
  EXPECT_FALSE(std::filesystem::exists(tmp));
}

TEST_F(GenerateVideoCommonTest, PartialFileGuardDtorLeavesRenamedAwayOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  {
    PartialFileGuard guard(tmp);
    write_file(tmp, "video");
    std::filesystem::rename(tmp, out);
  }
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(read_file(out), "video");
}

// ---- finalize_video_output --------------------------------------------------

TEST_F(GenerateVideoCommonTest, FinalizeRenamesTmpIntoPlace)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  EXPECT_EQ(finalize_video_output(tmp, out, false), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  ASSERT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(read_file(out), "video");
}

TEST_F(GenerateVideoCommonTest, FinalizeRejectsExistingOutputWithoutOverwrite)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  write_file(out, "old");
  const auto err = finalize_video_output(tmp, out, false);
  EXPECT_EQ(
    err, "output path '" + out.string() + "' already exists; pass -w/--overwrite to replace it");
  // The tmp is left for the caller's PartialFileGuard to remove.
  EXPECT_TRUE(std::filesystem::exists(tmp));
  EXPECT_EQ(read_file(out), "old");
}

TEST_F(GenerateVideoCommonTest, FinalizeOverwriteReplacesExistingOutput)
{
  const auto tmp = tmp_dir_ / "out.bagwiz-partial.avi";
  const auto out = tmp_dir_ / "out.avi";
  write_file(tmp, "video");
  write_file(out, "old");
  EXPECT_EQ(finalize_video_output(tmp, out, true), "");
  EXPECT_FALSE(std::filesystem::exists(tmp));
  EXPECT_EQ(read_file(out), "video");
}

// ---- validate_video_output_path ---------------------------------------------

TEST_F(GenerateVideoCommonTest, ValidateOutputPathRejectsCollisionWithoutOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  write_file(out, "old");
  // The early refusal and finalize's late one now share one message, produced
  // by core::check_output_path_free / core::prepare_output_path.
  EXPECT_EQ(
    validate_video_output_path(out, false),
    "output path '" + out.string() + "' already exists; pass -w/--overwrite to replace it");
}

TEST_F(GenerateVideoCommonTest, ValidateOutputPathAcceptsCollisionWithOverwrite)
{
  const auto out = tmp_dir_ / "out.avi";
  write_file(out, "old");
  EXPECT_EQ(validate_video_output_path(out, true), "");
}

TEST_F(GenerateVideoCommonTest, ValidateOutputPathCreatesMissingParentDirectories)
{
  const auto out = tmp_dir_ / "a" / "b" / "out.avi";
  EXPECT_EQ(validate_video_output_path(out, false), "");
  EXPECT_TRUE(std::filesystem::is_directory(tmp_dir_ / "a" / "b"));
}

// ---- validate_video_inputs --------------------------------------------------

TEST_F(GenerateVideoCommonTest, ValidateInputsUnopenableInput)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "does_not_exist.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_NE(v.error.find("failed to open"), std::string::npos);
}

TEST_F(GenerateVideoCommonTest, ValidateInputsTopicNotFound)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/nope", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "topic '/nope' not found in " + bag.string());
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPlainImageTopicOk)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 1u);
  EXPECT_EQ(v.views[0].topic_type, kImageType);
  EXPECT_FALSE(v.views[0].camera_info_topic.has_value());
  EXPECT_EQ(v.grid.cols, 1u);
  EXPECT_EQ(v.grid.rows, 1u);
}

TEST_F(GenerateVideoCommonTest, ValidateInputsDuplicateTopicFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/cam/image");
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "topic '/cam/image' given more than once");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsGridTooSmallFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/a", kImageType);
    declare_topic(*w, "/cam/b", kImageType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/cam/b");
  args.grid = "1x1";
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_NE(v.error.find("1x1"), std::string::npos);
}

TEST_F(GenerateVideoCommonTest, ValidateInputsRectifyWithoutCamInfoFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.rectify = true;
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "A camera-info topic is required for --rectify or --pcd, but none could be derived from "
    "'/cam/image'. Pass it explicitly with --cam-info /cam/image=<info_topic>.");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsDerivesCamInfoAndAcceptsPcd)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    declare_topic(*w, "/points", kPointCloudType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 1u);
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/camera_info");
  EXPECT_EQ(v.views[0].pcd_topics, std::vector<std::string>({"/points"}));
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPerViewPcdBinding)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/a/image_raw", kImageType);
    declare_topic(*w, "/cam/b/image_raw", kImageType);
    declare_topic(*w, "/cam/a/camera_info", kCameraInfoType);
    declare_topic(*w, "/cam/b/camera_info", kCameraInfoType);
    declare_topic(*w, "/points/shared", kPointCloudType);
    declare_topic(*w, "/points/a_only", kPointCloudType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/a/image_raw", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/cam/b/image_raw");
  args.pointcloud_topics = {"/points/shared", "/cam/a/image_raw=/points/a_only"};
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  ASSERT_EQ(v.views.size(), 2u);
  EXPECT_EQ(v.views[0].pcd_topics, std::vector<std::string>({"/points/shared", "/points/a_only"}));
  EXPECT_EQ(v.views[1].pcd_topics, std::vector<std::string>({"/points/shared"}));
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/a/camera_info");
  EXPECT_EQ(v.views[1].camera_info_topic, "/cam/b/camera_info");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPerViewCamInfoOverride)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/a/image_raw", kImageType);
    declare_topic(*w, "/cam/b/image_raw", kImageType);
    declare_topic(*w, "/cam/a/camera_info", kCameraInfoType);
    declare_topic(*w, "/custom/b_info", kCameraInfoType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/a/image_raw", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/cam/b/image_raw");
  args.camera_info_entries = {"/cam/b/image_raw=/custom/b_info"};
  args.rectify = true;
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  EXPECT_EQ(v.views[0].camera_info_topic, "/cam/a/camera_info");
  EXPECT_EQ(v.views[1].camera_info_topic, "/custom/b_info");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPcdTopicNotFoundFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "pcd topic '/points' not found in " + bag.string());
}

TEST_F(GenerateVideoCommonTest, ValidateInputsPcdTopicWrongTypeFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    declare_topic(*w, "/points", kImageType);  // wrong type
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.pointcloud_topics = {"/points"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(
    v.error,
    "pcd topic '/points' has type 'sensor_msgs/msg/Image', expected sensor_msgs/msg/PointCloud2");
}

TEST_F(GenerateVideoCommonTest, ValidateInputsExplicitCamInfoMissingFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.camera_info_entries = {"/cam/camera_info"};
  const auto v = validate_video_inputs(args);
  EXPECT_FALSE(v.ok());
  EXPECT_EQ(v.error, "camera_info topic '/cam/camera_info' not found in " + bag.string());
}

// ---- scan_video_inputs ------------------------------------------------------

TEST_F(GenerateVideoCommonTest, ScanEmptyTopicFails)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 0);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.error, "topic '/cam/image' has no messages to render.");
}

TEST_F(GenerateVideoCommonTest, ScanEmptySecondaryTopicFails)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/a", kImageType);
    declare_topic(*w, "/cam/b", kImageType);
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/cam/a", 1'000'000'000LL, garbage);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/a", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/cam/b");
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.error, "topic '/cam/b' has no messages to render.");
}

TEST_F(GenerateVideoCommonTest, ScanDerivesSpanAndFps)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 3);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  const auto s = scan_video_inputs(args, v);
  ASSERT_TRUE(s.ok()) << s.error;
  EXPECT_EQ(s.span.count, 3);
  EXPECT_EQ(s.span.first_ns, 1'000'000'000LL);
  EXPECT_EQ(s.span.last_ns, 1'200'000'000LL);
  EXPECT_EQ(s.fps.num, 10);
  EXPECT_EQ(s.fps.den, 1);
  EXPECT_TRUE(s.pcd_topics.empty());
  EXPECT_TRUE(s.pcd_spans.empty());
  EXPECT_TRUE(s.pcd_topic_has_stamps.empty());
  EXPECT_EQ(s.global_property_min, 0.0);
  EXPECT_EQ(s.global_property_max, 0.0);
}

// ---- load_video_geometry ----------------------------------------------------

TEST_F(GenerateVideoCommonTest, LoadVideoGeometryDefaultsToEmpty)
{
  const auto bag = write_image_bag(tmp_dir_, "in.mcap", 1);
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  bagwiz::commands::VideoGeometry g;
  EXPECT_EQ(load_video_geometry(args, v, g), "");
  ASSERT_EQ(g.camera_infos.size(), 1u);
  EXPECT_FALSE(g.camera_infos[0].has_value());
  EXPECT_FALSE(g.tf_buffer.has_value());
}

TEST_F(GenerateVideoCommonTest, LoadVideoGeometryFailsWhenCamInfoUnreadable)
{
  // The cam-info topic is declared but carries no message to load.
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image_raw", kImageType);
    declare_topic(*w, "/cam/camera_info", kCameraInfoType);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image_raw", tmp_dir_ / "out.avi", false);
  args.rectify = true;
  const auto v = validate_video_inputs(args);
  ASSERT_TRUE(v.ok()) << v.error;
  bagwiz::commands::VideoGeometry g;
  EXPECT_FALSE(load_video_geometry(args, v, g).empty());
}

// ---- open_encode_reader -----------------------------------------------------

TEST_F(GenerateVideoCommonTest, OpenEncodeReaderMissingBagReturnsNull)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "does_not_exist.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  EXPECT_EQ(open_encode_reader(args), nullptr);
}

TEST_F(GenerateVideoCommonTest, OpenEncodeReaderFiltersToThePrimaryTopic)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/cam/image", kImageType);
    declare_topic(*w, "/other", kImageType);
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/cam/image", 1'000'000'000LL, garbage);
    w->write("/other", 1'000'000'000LL, garbage);
    w->close();
  }
  bagwiz::commands::GenerateVideoArgs args(bag, "/cam/image", tmp_dir_ / "out.avi", false);
  args.topics.push_back("/other");
  auto reader = open_encode_reader(args);
  ASSERT_NE(reader, nullptr);
  bagwiz::io::RawMessage raw;
  ASSERT_TRUE(reader->next(raw));
  EXPECT_EQ(raw.topic->name, "/cam/image");
  EXPECT_FALSE(reader->next(raw));  // the secondary is filtered out of the encode reader
}

// ---- finish_video_encode ----------------------------------------------------

TEST_F(GenerateVideoCommonTest, FinishEncodeRequiresAStartedEncoder)
{
  bagwiz::commands::GenerateVideoArgs args(
    tmp_dir_ / "in.mcap", "/cam/image", tmp_dir_ / "out.avi", false);
  bagwiz::commands::VideoFrameEncoder encoder(
    partial_tmp_path_for(args.output_path), bagwiz::core::video::FrameRate{10, 1});
  EXPECT_EQ(
    finish_video_encode(
      encoder, args.topics.front(), partial_tmp_path_for(args.output_path), args.output_path,
      false),
    "topic '/cam/image' yielded no frames in the encode pass.");
  EXPECT_FALSE(std::filesystem::exists(args.output_path));
  EXPECT_FALSE(std::filesystem::exists(partial_tmp_path_for(args.output_path)));
}

// ---- FrameNormalizer::decode ------------------------------------------------

TEST(FrameNormalizerDecode, Bgr8ImageBecomesCanonicalFrame)
{
  FrameNormalizer normalizer(kImageType);
  const auto payload = make_bgr8_image_payload(2, 1, 0x2A);
  const auto frame = normalizer.decode(42, payload, 0);
  ASSERT_TRUE(frame.has_value());
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
  FrameNormalizer normalizer(kImageType);
  const std::array<std::byte, 4> garbage{
    std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
  EXPECT_FALSE(normalizer.decode(42, garbage, 0).has_value());
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
  EXPECT_TRUE(resize_frame(frame, 2, 1));
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
  EXPECT_TRUE(resize_frame(frame, 2, 1));
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
  EXPECT_FALSE(resize_frame(frame, 0, 0));
}

// ---- NearestMessageSource ---------------------------------------------------

TEST_F(GenerateVideoCommonTest, NearestMessageSourceMatchesByRecordTime)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/sec", kImageType);
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/sec", 100, garbage);
    w->write("/sec", 200, garbage);
    w->write("/sec", 400, garbage);
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

TEST_F(GenerateVideoCommonTest, NearestMessageSourceEmptyTopicYieldsNull)
{
  const auto bag = tmp_dir_ / "in.mcap";
  {
    auto w = bagwiz::io::open_write(bag, mcap_options());
    declare_topic(*w, "/empty", kImageType);
    declare_topic(*w, "/other", kImageType);
    const std::array<std::byte, 4> garbage{
      std::byte{0x00}, std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    w->write("/other", 100, garbage);
    w->close();
  }
  std::string error;
  auto source = NearestMessageSource::open(bag, "/empty", error);
  ASSERT_NE(source, nullptr) << error;
  EXPECT_EQ(source->fetch(100, error), nullptr);
  EXPECT_TRUE(error.empty());
}

// ---- GridCanvas -------------------------------------------------------------

TEST(GridCanvasTest, CellsTileTheComposedFrame)
{
  GridCanvas canvas(GridSpec{2, 1});
  canvas.set_cell_size(2, 1);
  EXPECT_EQ(canvas.width(), 4u);
  EXPECT_EQ(canvas.height(), 1u);
  ASSERT_EQ(canvas.pixels().size(), 12u);

  auto c0 = canvas.cell(0);
  EXPECT_EQ(c0.width, 2u);
  EXPECT_EQ(c0.height, 1u);
  EXPECT_EQ(c0.stride, 12u);
  EXPECT_EQ(c0.data, canvas.pixels().data());
  auto c1 = canvas.cell(1);
  EXPECT_EQ(c1.data, canvas.pixels().data() + 6);

  c1.data[0] = std::byte{0x2A};
  EXPECT_EQ(canvas.pixels()[6], std::byte{0x2A});
  canvas.clear();
  EXPECT_EQ(canvas.pixels()[6], std::byte{0});
}

// ---- ViewRenderer -----------------------------------------------------------

TEST(ViewRendererTest, FixedScaleRejectsNativeSizeChange)
{
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, 0.5);
  const auto geom = renderer.prepare(4, 2, 0, 0);
  ASSERT_TRUE(geom.has_value());
  EXPECT_EQ(geom->width, 2u);
  EXPECT_EQ(geom->height, 1u);
  // Same native size again: fine. A change: abort, like the single-view lock.
  EXPECT_TRUE(renderer.prepare(4, 2, 0, 0).has_value());
  EXPECT_FALSE(renderer.prepare(2, 2, 0, 0).has_value());
}

TEST(ViewRendererTest, FitViewScalesIntoTheCellPreservingAspect)
{
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, std::nullopt);
  // A 2x2 native frame in a 4x2 cell fits at scale 1 (limited by height).
  const auto geom = renderer.prepare(2, 2, 4, 2);
  ASSERT_TRUE(geom.has_value());
  EXPECT_EQ(geom->width, 2u);
  EXPECT_EQ(geom->height, 2u);
}

TEST(ViewRendererTest, RenderPastesCenteredIntoTheCell)
{
  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(4, 2);
  ViewRenderer renderer(nullptr, false, VideoOverlayParams{}, std::nullopt);
  const auto geom = renderer.prepare(2, 2, 4, 2);
  ASSERT_TRUE(geom.has_value());

  FrameBuffer frame;
  frame.width = 2;
  frame.height = 2;
  frame.step = 6;
  frame.encoding = "bgr8";
  frame.data.assign(12, std::byte{0x2A});

  canvas.clear();
  ASSERT_TRUE(renderer.render(frame, *geom, nullptr, canvas.cell(0)));
  // x_off = (4-2)/2 = 1: columns 1 and 2 carry the frame, 0 and 3 stay black.
  for (std::uint32_t y = 0; y < 2; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * 12;
    EXPECT_EQ(canvas.pixels()[row + 0], std::byte{0});
    EXPECT_EQ(canvas.pixels()[row + 3], std::byte{0x2A});
    EXPECT_EQ(canvas.pixels()[row + 6], std::byte{0x2A});
    EXPECT_EQ(canvas.pixels()[row + 9], std::byte{0});
  }
}

}  // namespace
