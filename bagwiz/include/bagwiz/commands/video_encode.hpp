// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__VIDEO_ENCODE_HPP_
#define BAGWIZ__COMMANDS__VIDEO_ENCODE_HPP_

#include "bagwiz/core/video/frame_codec.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

// The suffix appended to a source topic to name its video topic when --as
// is not given: /cam/image_raw -> /cam/image_raw/video.
inline constexpr const char * kVideoTopicSuffix = "/video";

// Every user-facing encoder effort accepted by `video encode --preset`.
inline constexpr std::array<std::string_view, 5> kVideoEncodePresets{
  {"fastest", "faster", "default", "slower", "slowest"}};

// The libx264/libx265 preset behind a user-facing effort. The choices span
// the underlying scale while keeping the encoder's current medium default.
// NVENC maps these names onto its own p1 (fastest) .. p7 (slowest) scale.
[[nodiscard]] std::optional<std::string_view> frame_encoder_preset(
  std::string_view preset) noexcept;

struct VideoEncodeArgs
{
  std::filesystem::path input_path;  // -i,--input
  // -t,--topics: the Image / CompressedImage topics to encode. Globs are
  // expanded before run_video_encode() sees them.
  std::vector<std::string> topics;
  // --as: the video topic's name; only with a single source topic. Empty
  // derives <source> + kVideoTopicSuffix.
  std::optional<std::string> as_topic;
  std::optional<std::filesystem::path> output_path;                // -o; empty => in-place
  bool overwrite = false;                                          // -w
  bool keep_inputs = false;                                        // --keep-inputs
  core::video::VideoCodec codec = core::video::VideoCodec::kH264;  // --codec
  core::video::EncoderBackend encoder = core::video::EncoderBackend::kAuto;  // --encoder
  std::string preset = "default";                                            // --preset
  std::optional<int> crf;                                                    // --crf
  int gop = 30;                                                              // --gop
  std::optional<int> threads;                                                // -j,--threads
};

int run_video_encode(const VideoEncodeArgs & args);

// The video topic a source topic maps to when --as is not given.
[[nodiscard]] std::string default_video_topic(const std::string & source_topic);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__VIDEO_ENCODE_HPP_
