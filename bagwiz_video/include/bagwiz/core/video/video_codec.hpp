// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__VIDEO_CODEC_HPP_
#define BAGWIZ__CORE__VIDEO__VIDEO_CODEC_HPP_

#include <array>
#include <optional>
#include <string_view>

namespace bagwiz::core::video
{

// The video codecs bagwiz encodes into and decodes from per-frame packets.
// Both carry their frames as Annex B byte streams (start-code delimited NAL
// units), the form foxglove_msgs/msg/CompressedVideo requires.
enum class VideoCodec { kH264, kH265 };

// The CompressedVideo `format` string for a codec ("h264" / "h265").
[[nodiscard]] std::string_view video_codec_format(VideoCodec codec) noexcept;

// The codec behind a CompressedVideo `format` string, or nullopt for a
// format bagwiz does not handle (including the message's other legal
// values, "vp9" and "av1").
[[nodiscard]] std::optional<VideoCodec> parse_video_codec_format(std::string_view format) noexcept;

// Every format string parse_video_codec_format() accepts, for CLI choices.
inline constexpr std::array<std::string_view, 2> kVideoCodecFormats{{"h264", "h265"}};

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__VIDEO_CODEC_HPP_
