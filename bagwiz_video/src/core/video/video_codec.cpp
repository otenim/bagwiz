// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/video_codec.hpp"

#include <optional>
#include <string_view>

namespace bagwiz::core::video
{

std::string_view video_codec_format(VideoCodec codec) noexcept
{
  switch (codec) {
    case VideoCodec::kH265:
      return "h265";
    case VideoCodec::kH264:
    default:
      return "h264";
  }
}

// cppcheck-suppress passedByValue
// std::string_view is the lightweight view type and is idiomatically passed
// by value; cppcheck's heuristic flags all view-by-value uses.
std::optional<VideoCodec> parse_video_codec_format(std::string_view format) noexcept
{
  if (format == "h264") {
    return VideoCodec::kH264;
  }
  if (format == "h265") {
    return VideoCodec::kH265;
  }
  return std::nullopt;
}

}  // namespace bagwiz::core::video
