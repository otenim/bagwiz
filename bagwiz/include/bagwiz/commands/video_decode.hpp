// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__VIDEO_DECODE_HPP_
#define BAGWIZ__COMMANDS__VIDEO_DECODE_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// What `video decode` writes for each decoded frame.
enum class DecodedImageFormat {
  kJpeg,  // sensor_msgs/msg/CompressedImage, format "jpeg"
  kPng,   // sensor_msgs/msg/CompressedImage, format "png"
  kRaw    // sensor_msgs/msg/Image, encoding "bgr8"
};

// The suffix appended to a source topic to name its image topic when --as
// is not given and the source does not end in the video suffix:
// /cam/clip -> /cam/clip/image.
inline constexpr const char * kImageTopicSuffix = "/image";

struct VideoDecodeArgs
{
  std::filesystem::path input_path;  // -i,--input
  // -t,--topics: the CompressedVideo topics to decode. Globs are expanded
  // before run_video_decode() sees them.
  std::vector<std::string> topics;
  // --as: the image topic's name; only with a single source topic. Empty
  // derives the name with default_image_topic().
  std::optional<std::string> as_topic;
  std::optional<std::filesystem::path> output_path;       // -o; empty => in-place
  bool overwrite = false;                                 // -w
  bool keep_inputs = false;                               // --keep-inputs
  DecodedImageFormat format = DecodedImageFormat::kJpeg;  // --format
  int quality = 90;                                       // --quality (jpeg)
};

int run_video_decode(const VideoDecodeArgs & args);

// The image topic a source video topic maps to when --as is not given: a
// trailing "/video" is removed (undoing `video encode`'s default), otherwise
// kImageTopicSuffix is appended.
[[nodiscard]] std::string default_image_topic(const std::string & source_topic);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__VIDEO_DECODE_HPP_
