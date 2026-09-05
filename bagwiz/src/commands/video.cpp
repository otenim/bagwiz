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
#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/commands/video_decode.hpp"
#include "bagwiz/commands/video_encode.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/video/frame_codec.hpp"
#include "bagwiz/core/video/video_codec.hpp"
#include "bagwiz/core/video/video_encoder.hpp"

#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.video";
}  // namespace

class VideoCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "video"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Convert image topics to video topics (encode) and back (decode)";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_encode(app);
    configure_decode(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kEncode:
        return run_video_encode(encode_args_);
      case Subcommand::kDecode:
        return run_video_decode(decode_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kEncode, kDecode };
  Subcommand selected_ = Subcommand::kNone;
  VideoEncodeArgs encode_args_;
  VideoDecodeArgs decode_args_;

  void configure_encode(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "encode",
      "Encode Image / CompressedImage topics into foxglove_msgs/CompressedVideo topics (H.264 "
      "or H.265, one message per frame, playable in Foxglove). By default each source topic "
      "is replaced by its video topic, named <source>/video.");
    sub->add_option("-i,--input", encode_args_.input_path, "Input bag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(*sub, encode_args_.input_path);
    add_topic_option(
      *sub, "-t,--topics", encode_args_.topics,
      "Image or CompressedImage (bgr8/rgb8, jpeg/png) topics to encode; each a literal name "
      "or a '*' glob.",
      TopicSlotSpec{.allowed_types = kImageTopicTypes})
      ->required()
      ->expected(-1);
    add_topic_option(
      *sub, "--as", encode_args_.as_topic,
      "Name of the video topic to create. Only with a single source topic. Default: "
      "<source>/video.",
      TopicSlotSpec{
        .mode = TopicSelectorMode::kLiteral,
        .reject_reason = "it names the new video topic to create"});
    sub->add_option(
      "-o,--output", encode_args_.output_path,
      "Output bag path. When omitted, the input bag is rewritten in place (atomic tmp swap).");
    sub->add_flag(
      "-w,--overwrite", encode_args_.overwrite, "Overwrite an existing -o/--output path.");
    sub->add_flag(
      "--keep-inputs", encode_args_.keep_inputs,
      "Keep the source image topics in the output next to the video topics (default: replace "
      "them).");
    const std::map<std::string, core::video::VideoCodec> codec_map{
      {"h264", core::video::VideoCodec::kH264}, {"h265", core::video::VideoCodec::kH265}};
    sub
      ->add_option(
        "--codec", encode_args_.codec,
        "Video codec: h264 (widest playback support) or h265 (smaller at the same quality; "
        "Foxglove needs a browser or desktop build with HEVC decoding). Default: h264.")
      ->transform(CLI::CheckedTransformer{codec_map})
      ->default_val(core::video::VideoCodec::kH264);
    const std::map<std::string, core::video::EncoderBackend> encoder_map{
      {"auto", core::video::EncoderBackend::kAuto},
      {"cpu", core::video::EncoderBackend::kCpu},
      {"nvenc", core::video::EncoderBackend::kNvenc}};
    sub
      ->add_option(
        "--encoder", encode_args_.encoder,
        "Encoder: 'auto' uses NVIDIA NVENC for frames larger than 1080p when this build and a "
        "GPU support it, else the CPU encoder (libx264 / libx265); 'cpu' and 'nvenc' force "
        "one. Default: auto.")
      ->transform(CLI::CheckedTransformer{encoder_map})
      ->default_val(core::video::EncoderBackend::kAuto);
    sub
      ->add_option(
        "--preset", encode_args_.preset,
        "Speed/quality preset, by libx264's names (ultrafast, superfast, veryfast, faster, "
        "fast, medium, slow, slower, veryslow); libx265 takes the same names and NVENC maps "
        "them onto its p1-p7. Default: medium.")
      ->check(
        CLI::IsMember(
          std::vector<std::string>(
            core::video::kH264Presets.begin(), core::video::kH264Presets.end())))
      ->default_val("medium");
    sub
      ->add_option(
        "--crf", encode_args_.crf,
        "Constant-quality target, 0 (best) to 51 (smallest). Default: 23 for h264, 28 for "
        "h265.")
      ->check(CLI::Range(core::video::kFrameCrfMin, core::video::kFrameCrfMax));
    sub
      ->add_option(
        "--gop", encode_args_.gop,
        "Keyframe interval in frames: every --gop-th frame is a keyframe a player can start "
        "or seek from. Smaller seeks faster and costs bytes. Default: 30.")
      ->check(CLI::PositiveNumber)
      ->default_val(30);
    sub
      ->add_option(
        "-j,--threads", encode_args_.threads,
        "Encoder threads (default: the encoder's own choice; 1 = single-threaded).")
      ->check(CLI::Range(1, 256));
    sub->callback([this]() { selected_ = Subcommand::kEncode; });
  }

  void configure_decode(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "decode",
      "Decode foxglove_msgs/CompressedVideo topics (h264 / h265) back into image topics: "
      "CompressedImage (jpeg or png) or Image (bgr8). By default each source topic is "
      "replaced by its image topic, named by removing a trailing /video, else <source>/image.");
    sub->add_option("-i,--input", decode_args_.input_path, "Input bag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    set_topic_input(*sub, decode_args_.input_path);
    add_topic_option(
      *sub, "-t,--topics", decode_args_.topics,
      "CompressedVideo topics to decode; each a literal name or a '*' glob.",
      TopicSlotSpec{.allowed_types = kCompressedVideoTopicTypes})
      ->required()
      ->expected(-1);
    add_topic_option(
      *sub, "--as", decode_args_.as_topic,
      "Name of the image topic to create. Only with a single source topic. Default: the "
      "source name without a trailing /video, else <source>/image.",
      TopicSlotSpec{
        .mode = TopicSelectorMode::kLiteral,
        .reject_reason = "it names the new image topic to create"});
    sub->add_option(
      "-o,--output", decode_args_.output_path,
      "Output bag path. When omitted, the input bag is rewritten in place (atomic tmp swap).");
    sub->add_flag(
      "-w,--overwrite", decode_args_.overwrite, "Overwrite an existing -o/--output path.");
    sub->add_flag(
      "--keep-inputs", decode_args_.keep_inputs,
      "Keep the source video topics in the output next to the image topics (default: replace "
      "them).");
    const std::map<std::string, DecodedImageFormat> format_map{
      {"jpeg", DecodedImageFormat::kJpeg},
      {"png", DecodedImageFormat::kPng},
      {"raw", DecodedImageFormat::kRaw}};
    sub
      ->add_option(
        "--format", decode_args_.format,
        "Image message written per frame: 'jpeg' or 'png' (sensor_msgs/CompressedImage) or "
        "'raw' (sensor_msgs/Image, bgr8). Default: jpeg.")
      ->transform(CLI::CheckedTransformer{format_map})
      ->default_val(DecodedImageFormat::kJpeg);
    sub
      ->add_option(
        "--quality", decode_args_.quality,
        "JPEG quality for --format jpeg, 1 (smallest) to 100 (best). Default: 90.")
      ->check(CLI::Range(core::image::kJpegQualityMin, core::image::kJpegQualityMax))
      ->default_val(90);
    sub->callback([this]() { selected_ = Subcommand::kDecode; });
  }
};

BAGWIZ_REGISTER_COMMAND(VideoCommand)

}  // namespace bagwiz::commands
