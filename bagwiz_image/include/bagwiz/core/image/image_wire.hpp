// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__IMAGE_WIRE_HPP_
#define BAGWIZ__CORE__IMAGE__IMAGE_WIRE_HPP_

#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

// Writing side of the sensor_msgs image wire formats. The reading side
// (extract_raw_image / extract_compressed_image) lives next to each type's
// view; this header serializes the same layouts back to CDR and declares the
// topics that carry them.
namespace bagwiz::core::image
{

inline constexpr std::string_view kImageType = "sensor_msgs/msg/Image";
inline constexpr std::string_view kCompressedImageType = "sensor_msgs/msg/CompressedImage";

// Serialize one sensor_msgs/msg/Image (little-endian CDR with the
// encapsulation header). `data` must hold `step * height` bytes;
// is_bigendian is written as 0.
[[nodiscard]] std::vector<std::byte> serialize_raw_image(
  std::int64_t stamp_ns, std::string_view frame_id, std::uint32_t width, std::uint32_t height,
  std::string_view encoding, std::uint32_t step, std::span<const std::byte> data);

// Serialize one sensor_msgs/msg/CompressedImage.
[[nodiscard]] std::vector<std::byte> serialize_compressed_image(
  std::int64_t stamp_ns, std::string_view frame_id, std::string_view format,
  std::span<const std::byte> data);

// TopicInfo for a new topic of either type: "cdr" serialization and the
// ros2msg schema resolved from the sensor_msgs definitions on
// $AMENT_PREFIX_PATH. When the definition cannot be resolved both schema
// fields stay empty (the MCAP convention for "no schema known"); callers that
// have another source for the schema, such as a same-type topic in the input
// bag, may fill them in.
[[nodiscard]] io::TopicInfo make_image_topic_info(std::string_view topic_name);
[[nodiscard]] io::TopicInfo make_compressed_image_topic_info(std::string_view topic_name);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__IMAGE_WIRE_HPP_
