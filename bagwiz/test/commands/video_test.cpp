// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_option.hpp"
#include "bagwiz/commands/video_decode.hpp"
#include "bagwiz/commands/video_encode.hpp"
#include "bagwiz/core/image/compressed_image.hpp"
#include "bagwiz/core/image/compressed_video.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/image_wire.hpp"
#include "bagwiz/core/image/raw_image.hpp"
#include "bagwiz/core/video/annexb.hpp"
#include "bagwiz/core/video/video_codec.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "core/image/image_fixture.hpp"
#include "topic_slot_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace
{

namespace img = bagwiz::core::image;
namespace vid = bagwiz::core::video;
using bagwiz::commands::DecodedImageFormat;
using bagwiz::commands::default_image_topic;
using bagwiz::commands::default_video_topic;
using bagwiz::commands::run_video_decode;
using bagwiz::commands::run_video_encode;
using bagwiz::commands::VideoDecodeArgs;
using bagwiz::commands::VideoEncodeArgs;

constexpr std::uint32_t kW = 64;
constexpr std::uint32_t kH = 48;
constexpr const char * kImageTopic = "/cam/image_raw";
constexpr const char * kCompressedTopic = "/cam/image_raw/compressed";
constexpr const char * kOtherTopic = "/imu/data";
constexpr const char * kOtherType = "sensor_msgs/msg/Imu";
constexpr std::int64_t kStamp0 = 1'700'000'000'000'000'000LL;
constexpr std::int64_t kPeriodNs = 100'000'000LL;  // 10 fps

class VideoCommandTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    tmp_dir_ =
      std::filesystem::temp_directory_path() /
      ("bagwiz_video_cmd_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
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

// A gentle ramp shifted per frame, so frames differ but chroma subsampling
// and the codecs reproduce them closely.
std::vector<std::byte> ramp_bgr(int frame_index)
{
  std::vector<std::byte> px(static_cast<std::size_t>(kW) * kH * 3);
  for (std::uint32_t y = 0; y < kH; ++y) {
    for (std::uint32_t x = 0; x < kW; ++x) {
      const std::size_t base = (static_cast<std::size_t>(y) * kW + x) * 3;
      const unsigned shift = static_cast<unsigned>(frame_index) * 3U;
      px[base + 0] = static_cast<std::byte>((40U + (x * 80U) / kW + shift) & 0xFFU);
      px[base + 1] = static_cast<std::byte>((60U + (y * 60U) / kH + shift) & 0xFFU);
      px[base + 2] = static_cast<std::byte>((200U - (x * 60U) / kW - shift) & 0xFFU);
    }
  }
  return px;
}

std::int64_t stamp_of(int i)
{
  return kStamp0 + static_cast<std::int64_t>(i) * kPeriodNs;
}

// The record time trails header.stamp by a fixed lag, so a test can tell
// which of the two a converted message carries.
constexpr std::int64_t kRecordLagNs = 5'000'000LL;

struct BagSpec
{
  int frames = 6;
  bool raw = true;         // an Image topic (bgr8)
  bool compressed = true;  // a CompressedImage topic (jpeg)
  bool other = true;       // an unrelated topic to copy through
};

// Build an MCAP directory bag per `spec`, with `frames` messages per image
// topic at 10 fps. Returns the bag path.
std::filesystem::path build_image_bag(const std::filesystem::path & dir, const BagSpec & spec)
{
  const auto path = dir / "input";
  auto writer = bagwiz::io::open_write(path, mcap_dir_opts());
  if (spec.raw) {
    writer->declare_topic(img::make_image_topic_info(kImageTopic));
  }
  if (spec.compressed) {
    writer->declare_topic(img::make_compressed_image_topic_info(kCompressedTopic));
  }
  if (spec.other) {
    writer->declare_topic(make_topic(kOtherTopic, kOtherType));
  }
  for (int i = 0; i < spec.frames; ++i) {
    const auto stamp = stamp_of(i);
    const auto record = stamp + kRecordLagNs;
    if (spec.raw) {
      const auto px = ramp_bgr(i);
      const auto payload =
        img::serialize_raw_image(stamp, "cam_optical", kW, kH, "bgr8", kW * 3, px);
      writer->write(kImageTopic, record, payload);
    }
    if (spec.compressed) {
      const auto jpeg = bagwiz::test::encode_still_image(
        "jpeg", kW, kH, static_cast<std::uint8_t>(40 + i * 10), 100, 50);
      const auto payload = img::serialize_compressed_image(stamp, "cam_optical", "jpeg", jpeg);
      writer->write(kCompressedTopic, record, payload);
    }
    if (spec.other) {
      const std::array<std::byte, 8> blob{};
      writer->write(kOtherTopic, record, blob);
    }
  }
  writer->close();
  return path;
}

struct Message
{
  std::int64_t record_ns = 0;
  std::vector<std::byte> payload;
};

// Every message of `topic` in the bag, in record order.
std::vector<Message> read_topic(const std::filesystem::path & bag, const std::string & topic)
{
  auto reader = bagwiz::io::open_read(bag);
  bagwiz::io::RawMessage raw;
  std::vector<Message> out;
  while (reader->next(raw)) {
    if (raw.topic->name == topic) {
      out.push_back(Message{raw.timestamp_ns, {raw.payload.begin(), raw.payload.end()}});
    }
  }
  return out;
}

// Every topic of the bag by name, with embedded schemas loaded (a directory
// MCAP reader fills them in only on populate_schemas()).
std::map<std::string, bagwiz::io::TopicInfo> topics_of(const std::filesystem::path & bag)
{
  auto reader = bagwiz::io::open_read(bag);
  reader->populate_schemas();
  std::map<std::string, bagwiz::io::TopicInfo> out;
  for (const auto & t : reader->topics()) {
    out[t.name] = t;
  }
  return out;
}

VideoEncodeArgs encode_args(const std::filesystem::path & input, std::vector<std::string> topics)
{
  VideoEncodeArgs args;
  args.input_path = input;
  args.topics = std::move(topics);
  args.encoder = vid::EncoderBackend::kCpu;
  args.preset = "ultrafast";
  args.gop = 4;
  return args;
}

VideoDecodeArgs decode_args(const std::filesystem::path & input, std::vector<std::string> topics)
{
  VideoDecodeArgs args;
  args.input_path = input;
  args.topics = std::move(topics);
  return args;
}

double mean_abs_diff(std::span<const std::byte> a, std::span<const std::byte> b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    sum += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
  }
  return sum / static_cast<double>(a.size());
}

}  // namespace

TEST(VideoTopicNames, DefaultsAppendAndStripTheVideoSuffix)
{
  EXPECT_EQ(default_video_topic("/cam/image_raw"), "/cam/image_raw/video");
  EXPECT_EQ(default_image_topic("/cam/image_raw/video"), "/cam/image_raw");
  EXPECT_EQ(default_image_topic("/cam/clip"), "/cam/clip/image");
  EXPECT_EQ(default_image_topic("/video"), "/video/image");  // nothing left to strip
}

TEST_F(VideoCommandTest, EncodesRawImageTopicIntoFoxgloveVideoAndReplacesIt)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic});
  args.output_path = output;
  ASSERT_EQ(run_video_encode(args), 0);

  const auto topics = topics_of(output);
  EXPECT_EQ(topics.count(kImageTopic), 0U);  // replaced
  EXPECT_EQ(topics.count(kCompressedTopic), 1U);
  EXPECT_EQ(topics.count(kOtherTopic), 1U);
  ASSERT_EQ(topics.count("/cam/image_raw/video"), 1U);
  const auto & info = topics.at("/cam/image_raw/video");
  EXPECT_EQ(info.type, "foxglove_msgs/msg/CompressedVideo");
  EXPECT_EQ(info.schema_encoding, "ros2msg");
  EXPECT_NE(info.schema_text.find("string format\n"), std::string::npos);

  const auto messages = read_topic(output, "/cam/image_raw/video");
  ASSERT_EQ(messages.size(), 6U);
  for (std::size_t i = 0; i < messages.size(); ++i) {
    EXPECT_EQ(messages[i].record_ns, stamp_of(static_cast<int>(i)) + kRecordLagNs) << i;
    const auto video = img::extract_compressed_video(messages[i].payload);
    ASSERT_TRUE(video.ok()) << video.error;
    EXPECT_EQ(video.video->timestamp_ns, stamp_of(static_cast<int>(i))) << i;
    EXPECT_EQ(video.video->frame_id, "cam_optical") << i;
    EXPECT_EQ(video.video->format, "h264") << i;
    const auto summary = vid::summarize_annexb(video.video->data, vid::VideoCodec::kH264);
    EXPECT_TRUE(summary.starts_with_start_code) << i;
    const bool keyframe = (i % 4) == 0;
    EXPECT_EQ(summary.has_keyframe_slice, keyframe) << i;
    if (keyframe) {
      EXPECT_TRUE(summary.has_sps) << i;
      EXPECT_TRUE(summary.has_pps) << i;
    }
  }
  // Untouched topics are copied through whole.
  EXPECT_EQ(read_topic(output, kOtherTopic).size(), 6U);
  EXPECT_EQ(read_topic(output, kCompressedTopic).size(), 6U);
}

TEST_F(VideoCommandTest, KeepInputsRetainsTheSourceNextToTheVideo)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic});
  args.output_path = output;
  args.keep_inputs = true;
  ASSERT_EQ(run_video_encode(args), 0);

  const auto topics = topics_of(output);
  EXPECT_EQ(topics.count(kImageTopic), 1U);
  EXPECT_EQ(topics.count("/cam/image_raw/video"), 1U);
  EXPECT_EQ(read_topic(output, kImageTopic).size(), 6U);
  EXPECT_EQ(read_topic(output, "/cam/image_raw/video").size(), 6U);
}

TEST_F(VideoCommandTest, AsNamesTheVideoTopicAndH265IsHonoured)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kCompressedTopic});
  args.output_path = output;
  args.as_topic = "/cam/clip";
  args.codec = vid::VideoCodec::kH265;
  const int rc = run_video_encode(args);
  if (rc != 0) {
    GTEST_SKIP() << "H.265 encode unavailable in this build";
  }
  const auto topics = topics_of(output);
  EXPECT_EQ(topics.count(kCompressedTopic), 0U);
  ASSERT_EQ(topics.count("/cam/clip"), 1U);
  const auto messages = read_topic(output, "/cam/clip");
  ASSERT_EQ(messages.size(), 6U);
  const auto first = img::extract_compressed_video(messages.front().payload);
  ASSERT_TRUE(first.ok()) << first.error;
  EXPECT_EQ(first.video->format, "h265");
  const auto summary = vid::summarize_annexb(first.video->data, vid::VideoCodec::kH265);
  EXPECT_TRUE(summary.has_vps);
  EXPECT_TRUE(summary.has_sps);
  EXPECT_TRUE(summary.has_pps);
  EXPECT_TRUE(summary.has_keyframe_slice);
}

TEST_F(VideoCommandTest, EncodesSeveralTopicsWithDerivedNames)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic, kCompressedTopic});
  args.output_path = output;
  ASSERT_EQ(run_video_encode(args), 0);

  const auto topics = topics_of(output);
  EXPECT_EQ(topics.count(kImageTopic), 0U);
  EXPECT_EQ(topics.count(kCompressedTopic), 0U);
  EXPECT_EQ(topics.count("/cam/image_raw/video"), 1U);
  EXPECT_EQ(topics.count("/cam/image_raw/compressed/video"), 1U);
  EXPECT_EQ(read_topic(output, "/cam/image_raw/video").size(), 6U);
  EXPECT_EQ(read_topic(output, "/cam/image_raw/compressed/video").size(), 6U);
}

TEST_F(VideoCommandTest, AsWithSeveralSourcesIsRejectedBeforeWriting)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic, kCompressedTopic});
  args.output_path = output;
  args.as_topic = "/cam/clip";
  EXPECT_NE(run_video_encode(args), 0);
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(VideoCommandTest, MissingAndWrongTypeTopicsAreRejected)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {"/nope"});
  args.output_path = output;
  EXPECT_NE(run_video_encode(args), 0);
  args = encode_args(input, {kOtherTopic});
  args.output_path = output;
  EXPECT_NE(run_video_encode(args), 0);
  EXPECT_FALSE(std::filesystem::exists(output));
}

TEST_F(VideoCommandTest, OutputNameCollidingWithASurvivingTopicIsRejected)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic});
  args.output_path = output;
  args.as_topic = kOtherTopic;
  EXPECT_NE(run_video_encode(args), 0);
  // Reusing the source's own name is fine when the source is replaced...
  args.as_topic = kImageTopic;
  EXPECT_EQ(run_video_encode(args), 0);
  EXPECT_EQ(topics_of(output).at(kImageTopic).type, "foxglove_msgs/msg/CompressedVideo");
  // ...but not when it is kept.
  args.output_path = tmp_dir_ / "out2";
  args.keep_inputs = true;
  EXPECT_NE(run_video_encode(args), 0);
}

TEST_F(VideoCommandTest, InPlaceRewriteReplacesTheInputBag)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  auto args = encode_args(input, {kImageTopic});
  ASSERT_EQ(run_video_encode(args), 0);
  const auto topics = topics_of(input);
  EXPECT_EQ(topics.count(kImageTopic), 0U);
  EXPECT_EQ(topics.count("/cam/image_raw/video"), 1U);
  EXPECT_EQ(read_topic(input, "/cam/image_raw/video").size(), 6U);
}

TEST_F(VideoCommandTest, ExistingOutputNeedsOverwrite)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic});
  args.output_path = output;
  ASSERT_EQ(run_video_encode(args), 0);
  EXPECT_NE(run_video_encode(args), 0);
  args.overwrite = true;
  EXPECT_EQ(run_video_encode(args), 0);
}

TEST_F(VideoCommandTest, EmptySourceTopicIsDeclaredAndWarned)
{
  BagSpec spec;
  spec.frames = 0;
  const auto input = build_image_bag(tmp_dir_, spec);
  const auto output = tmp_dir_ / "out";
  auto args = encode_args(input, {kImageTopic});
  args.output_path = output;
  ASSERT_EQ(run_video_encode(args), 0);
  EXPECT_EQ(topics_of(output).count("/cam/image_raw/video"), 1U);
  EXPECT_TRUE(read_topic(output, "/cam/image_raw/video").empty());
}

// Encode then decode: every frame comes back with its header and a picture
// within codec noise of the original, in each output format.
TEST_F(VideoCommandTest, DecodeRoundTripsFramesHeadersAndPixels)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto encoded = tmp_dir_ / "encoded";
  auto enc = encode_args(input, {kImageTopic});
  enc.output_path = encoded;
  enc.crf = 12;
  ASSERT_EQ(run_video_encode(enc), 0);

  struct Case
  {
    DecodedImageFormat format;
    const char * type;
    const char * format_or_encoding;
  };
  const std::array<Case, 3> cases{
    {{DecodedImageFormat::kJpeg, "sensor_msgs/msg/CompressedImage", "jpeg"},
     {DecodedImageFormat::kPng, "sensor_msgs/msg/CompressedImage", "png"},
     {DecodedImageFormat::kRaw, "sensor_msgs/msg/Image", "bgr8"}}};
  int case_index = 0;
  for (const auto & c : cases) {
    const auto decoded = tmp_dir_ / ("decoded" + std::to_string(case_index++));
    auto dec = decode_args(encoded, {"/cam/image_raw/video"});
    dec.output_path = decoded;
    dec.format = c.format;
    ASSERT_EQ(run_video_decode(dec), 0) << c.format_or_encoding;

    const auto topics = topics_of(decoded);
    EXPECT_EQ(topics.count("/cam/image_raw/video"), 0U) << c.format_or_encoding;
    ASSERT_EQ(topics.count(kImageTopic), 1U) << c.format_or_encoding;  // /video stripped
    EXPECT_EQ(topics.at(kImageTopic).type, c.type) << c.format_or_encoding;
    EXPECT_EQ(topics.count(kOtherTopic), 1U);

    const auto messages = read_topic(decoded, kImageTopic);
    ASSERT_EQ(messages.size(), 6U) << c.format_or_encoding;
    for (std::size_t i = 0; i < messages.size(); ++i) {
      EXPECT_EQ(messages[i].record_ns, stamp_of(static_cast<int>(i)) + kRecordLagNs) << i;
      const auto expected = ramp_bgr(static_cast<int>(i));
      std::vector<std::byte> bgr;
      if (c.format == DecodedImageFormat::kRaw) {
        const auto view = img::extract_raw_image(messages[i].payload);
        ASSERT_TRUE(view.ok()) << view.error;
        EXPECT_EQ(view.image->header_stamp_ns, stamp_of(static_cast<int>(i))) << i;
        EXPECT_EQ(view.image->header_frame_id, "cam_optical") << i;
        EXPECT_EQ(view.image->encoding, "bgr8") << i;
        EXPECT_EQ(view.image->width, kW);
        EXPECT_EQ(view.image->height, kH);
        EXPECT_EQ(view.image->step, kW * 3);
        bgr.assign(view.image->data.begin(), view.image->data.end());
      } else {
        const auto view = img::extract_compressed_image(messages[i].payload);
        ASSERT_TRUE(view.ok()) << view.error;
        EXPECT_EQ(view.image->header_stamp_ns, stamp_of(static_cast<int>(i))) << i;
        EXPECT_EQ(view.image->header_frame_id, "cam_optical") << i;
        EXPECT_EQ(view.image->format, c.format_or_encoding) << i;
        const auto still = img::decode_compressed_image(view.image->data, view.image->format);
        ASSERT_TRUE(still.ok()) << still.error;
        EXPECT_EQ(still.image->width, kW);
        EXPECT_EQ(still.image->height, kH);
        bgr = still.image->bgr;
      }
      ASSERT_EQ(bgr.size(), expected.size());
      EXPECT_LT(mean_abs_diff(bgr, expected), 6.0) << c.format_or_encoding << " frame " << i;
    }
  }
}

TEST_F(VideoCommandTest, DecodeKeepInputsAndAsAreHonoured)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto encoded = tmp_dir_ / "encoded";
  auto enc = encode_args(input, {kCompressedTopic});
  enc.output_path = encoded;
  ASSERT_EQ(run_video_encode(enc), 0);

  const auto decoded = tmp_dir_ / "decoded";
  auto dec = decode_args(encoded, {"/cam/image_raw/compressed/video"});
  dec.output_path = decoded;
  dec.keep_inputs = true;
  dec.as_topic = "/cam/restored";
  ASSERT_EQ(run_video_decode(dec), 0);
  const auto topics = topics_of(decoded);
  EXPECT_EQ(topics.count("/cam/image_raw/compressed/video"), 1U);
  ASSERT_EQ(topics.count("/cam/restored"), 1U);
  EXPECT_EQ(topics.at("/cam/restored").type, "sensor_msgs/msg/CompressedImage");
  EXPECT_EQ(read_topic(decoded, "/cam/restored").size(), 6U);
}

TEST_F(VideoCommandTest, DecodeRejectsNonVideoTopicsAndUnsupportedFormats)
{
  const auto input = build_image_bag(tmp_dir_, BagSpec{});
  const auto output = tmp_dir_ / "out";
  auto dec = decode_args(input, {kImageTopic});  // an Image topic, not video
  dec.output_path = output;
  EXPECT_NE(run_video_decode(dec), 0);
  EXPECT_FALSE(std::filesystem::exists(output));

  // A CompressedVideo topic whose format bagwiz does not decode.
  const auto vp9_bag = tmp_dir_ / "vp9";
  {
    auto writer = bagwiz::io::open_write(vp9_bag, mcap_dir_opts());
    writer->declare_topic(img::make_compressed_video_topic_info("/cam/video"));
    const std::array<std::byte, 4> data{
      std::byte{0x82}, std::byte{0x49}, std::byte{0x83}, std::byte{0x42}};
    writer->write(
      "/cam/video", kStamp0, img::serialize_compressed_video(kStamp0, "cam", "vp9", data));
    writer->close();
  }
  dec = decode_args(vp9_bag, {"/cam/video"});
  dec.output_path = tmp_dir_ / "out2";
  EXPECT_NE(run_video_decode(dec), 0);
}

// The real VideoCommand::configure(), reached through the process-wide
// registry, declares the topic slots the selector expansion and the shell
// completion rely on: -t/--topics is a required multi-value glob slot typed
// to the accepted message types, --as a literal single-topic slot.
TEST(VideoCliWiring, TopicSlotsAreDeclaredOnBothSubcommands)
{
  bagwiz::commands::Command * video_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "video") {
      video_cmd = cmd.get();
      break;
    }
  }
  ASSERT_NE(video_cmd, nullptr);

  CLI::App app{"video"};
  video_cmd->configure(app);
  const auto subs = app.get_subcommands({});
  ASSERT_EQ(subs.size(), 2U);

  for (const CLI::App * sub : subs) {
    const auto slots = bagwiz::commands::topic_slots_of(*sub);
    ASSERT_EQ(slots.size(), 2U) << sub->get_name();  // --topics, --as
    const auto * topics = bagwiz::test::slot_for(slots, "topics");
    ASSERT_NE(topics, nullptr) << sub->get_name();
    EXPECT_EQ(topics->spec.mode, bagwiz::commands::TopicSelectorMode::kGlob);
    EXPECT_TRUE(topics->option->get_required());
    EXPECT_NE(topics->multi_target, nullptr);
    EXPECT_EQ(topics->spec.allowed_types.size(), sub->get_name() == "encode" ? 2U : 1U);
    const auto * as = bagwiz::test::slot_for(slots, "as");
    ASSERT_NE(as, nullptr) << sub->get_name();
    EXPECT_EQ(as->spec.mode, bagwiz::commands::TopicSelectorMode::kLiteral);
    EXPECT_FALSE(as->option->get_required());
    EXPECT_FALSE(as->spec.reject_reason.empty());
  }
}

TEST(VideoCliWiring, EncodeParsesItsOptionsIntoTheArgs)
{
  bagwiz::commands::Command * video_cmd = nullptr;
  for (const auto & cmd : bagwiz::commands::Registry::instance().all()) {
    if (cmd->name() == "video") {
      video_cmd = cmd.get();
    }
  }
  ASSERT_NE(video_cmd, nullptr);
  CLI::App app{"video"};
  video_cmd->configure(app);
  const auto tmp = std::filesystem::temp_directory_path();
  const std::vector<std::string> argv{"encode",   "-i",    tmp.string(),   "-t",    "/cam/*",
                                      "--codec",  "h265",  "--encoder",    "cpu",   "--preset",
                                      "veryfast", "--crf", "20",           "--gop", "12",
                                      "-j",       "2",     "--keep-inputs"};
  std::vector<std::string> reversed(argv.rbegin(), argv.rend());
  EXPECT_NO_THROW(app.parse(reversed));

  // Out-of-range values are refused by the parser.
  CLI::App app2{"video"};
  video_cmd->configure(app2);
  const std::vector<std::string> bad{"encode", "-i", tmp.string(), "-t", "/cam", "--crf", "99"};
  std::vector<std::string> bad_reversed(bad.rbegin(), bad.rend());
  EXPECT_THROW(app2.parse(bad_reversed), CLI::ParseError);

  CLI::App app3{"video"};
  video_cmd->configure(app3);
  const std::vector<std::string> bad_fmt{"decode", "-i",       tmp.string(), "-t",
                                         "/cam",   "--format", "bmp"};
  std::vector<std::string> bad_fmt_reversed(bad_fmt.rbegin(), bad_fmt.rend());
  EXPECT_THROW(app3.parse(bad_fmt_reversed), CLI::ParseError);
}
