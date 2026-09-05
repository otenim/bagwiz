// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_wire.hpp"

#include "bagwiz/core/cdr_walker/cdr_writer.hpp"
#include "bagwiz/core/image/stamp_wire.hpp"
#include "bagwiz/core/msg_yaml/msg_definition_resolver.hpp"

#include <cstdint>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

// std_msgs/Header: builtin_interfaces/Time stamp, string frame_id.
void write_header(cdr_walker::CdrWriter & writer, std::int64_t stamp_ns, std::string_view frame_id)
{
  const WireStamp stamp = split_stamp_ns(stamp_ns);
  writer.write_i32(stamp.sec);
  writer.write_u32(stamp.nanosec);
  writer.write_string(frame_id);
}

io::TopicInfo make_topic_info(std::string_view topic_name, std::string_view type)
{
  io::TopicInfo info;
  info.name.assign(topic_name.begin(), topic_name.end());
  info.type.assign(type.begin(), type.end());
  info.serialization_format = "cdr";
  auto resolved = core::resolve_message_definition(type);
  if (!resolved.text.empty()) {
    info.schema_text = std::move(resolved.text);
    info.schema_encoding = std::move(resolved.encoding);
  }
  return info;
}

}  // namespace

std::vector<std::byte> serialize_raw_image(
  std::int64_t stamp_ns, std::string_view frame_id, std::uint32_t width, std::uint32_t height,
  std::string_view encoding, std::uint32_t step, std::span<const std::byte> data)
{
  cdr_walker::CdrWriter writer;
  write_header(writer, stamp_ns, frame_id);
  writer.write_u32(height);
  writer.write_u32(width);
  writer.write_string(encoding);
  writer.write_u8(0);  // is_bigendian: CDR payloads bagwiz writes are little-endian
  writer.write_u32(step);
  writer.write_sequence_length(static_cast<std::uint32_t>(data.size()));
  writer.write_bytes(data);
  return writer.take();
}

std::vector<std::byte> serialize_compressed_image(
  std::int64_t stamp_ns, std::string_view frame_id, std::string_view format,
  std::span<const std::byte> data)
{
  cdr_walker::CdrWriter writer;
  write_header(writer, stamp_ns, frame_id);
  writer.write_string(format);
  writer.write_sequence_length(static_cast<std::uint32_t>(data.size()));
  writer.write_bytes(data);
  return writer.take();
}

io::TopicInfo make_image_topic_info(std::string_view topic_name)
{
  return make_topic_info(topic_name, kImageType);
}

io::TopicInfo make_compressed_image_topic_info(std::string_view topic_name)
{
  return make_topic_info(topic_name, kCompressedImageType);
}

}  // namespace bagwiz::core::image
