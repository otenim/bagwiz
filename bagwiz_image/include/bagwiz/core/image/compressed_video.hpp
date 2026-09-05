// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__COMPRESSED_VIDEO_HPP_
#define BAGWIZ__CORE__IMAGE__COMPRESSED_VIDEO_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Wire format of foxglove_msgs/msg/CompressedVideo: one encoded video frame
// per message, the shape Foxglove's image panel plays back as video. The
// message is handled straight from its CDR bytes (no rosidl typesupport), so
// bags can carry the type without foxglove_msgs being installed.
namespace bagwiz::core::image
{

inline constexpr std::string_view kCompressedVideoType = "foxglove_msgs/msg/CompressedVideo";

// The `format` values the message definition allows.
inline constexpr std::array<std::string_view, 4> kCompressedVideoFormats{
  {"h264", "h265", "vp9", "av1"}};

struct CompressedVideoView
{
  // `timestamp` as sec * 1e9 + nanosec.
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  // One encoded frame (for h264/h265: Annex B NAL units), borrowed from the
  // payload the view was extracted from.
  std::span<const std::byte> data;
  std::string format;  // "h264", "h265", "vp9" or "av1"
};

struct CompressedVideoResult
{
  std::optional<CompressedVideoView> video;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return video.has_value() && error.empty(); }
};

// Parse one serialized CompressedVideo message. `data` in the returned view
// aliases `payload`.
[[nodiscard]] CompressedVideoResult extract_compressed_video(std::span<const std::byte> payload);

// Serialize one CompressedVideo message (little-endian CDR with the 4-byte
// encapsulation header), ready for BagWriter::write. `timestamp_ns` splits
// into builtin_interfaces/Time with nanosec kept in [0, 1e9) (floor
// semantics for negative stamps).
[[nodiscard]] std::vector<std::byte> serialize_compressed_video(
  std::int64_t timestamp_ns, std::string_view frame_id, std::string_view format,
  std::span<const std::byte> data);

// TopicInfo for a new CompressedVideo topic: "cdr" serialization and a
// ros2msg schema so MCAP outputs stay self-describing. The schema text is
// resolved from an installed foxglove_msgs package when one is on
// $AMENT_PREFIX_PATH and otherwise comes from an embedded copy of the
// definition, so the type is always declared with a schema.
[[nodiscard]] io::TopicInfo make_compressed_video_topic_info(std::string_view topic_name);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__COMPRESSED_VIDEO_HPP_
