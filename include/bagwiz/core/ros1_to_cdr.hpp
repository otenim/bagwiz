// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__ROS1_TO_CDR_HPP_
#define BAGWIZ__CORE__ROS1_TO_CDR_HPP_

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// Translate a ROS 1 type name (e.g. "std_msgs/Header") to the ROS 2 type
// name we will write into the output bag (e.g. "std_msgs/msg/Header").
//
// The mapping is auto-derived for any well-formed `pkg/Type` input by
// inserting the `/msg/` infix. A small override table handles historical
// renames that don't follow the `/msg/` rule (e.g. `tf/tfMessage` →
// `tf2_msgs/msg/TFMessage`); see `ros1_to_ros2_renames` in the .cpp.
//
// Returns nullopt only for malformed inputs (empty, missing slash,
// extra path segments, non-identifier segments). Callers must still
// validate that the resulting ROS 2 type can be loaded — auto-derive
// tells you what name to look up, not whether it exists.
std::optional<std::string> map_ros1_type(std::string_view ros1_type);

struct Ros1ToCdrResult
{
  bool ok = false;
  std::string error;           // populated when ok == false
  std::vector<std::byte> cdr;  // CDR-LE encapsulated, ready for BagWriter::write()
};

// Convert one ROS 1 raw payload into a CDR-LE encapsulated payload. The
// destination type name must be a ROS 2 form ("pkg/msg/Type"); the
// caller is expected to have looked it up via map_ros1_type().
//
// Implementation notes:
//   * Loads the destination type's introspection via the existing
//     `load_introspection` helper. Types whose package isn't installed
//     (no `lib<pkg>__rosidl_typesupport_introspection_cpp.so`) produce
//     ok=false; callers handle that as a topic-skip.
//   * Walks the destination type's field tree. ROS 1 raw and CDR share
//     primitive layouts (little-endian, same widths) so primitives are
//     a memcpy plus alignment padding.
//   * std_msgs/msg/Header has its 4-byte ROS 1 `seq` field skipped on
//     input; ROS 2 dropped this field.
Ros1ToCdrResult convert_ros1_to_cdr(
  std::string_view dest_ros2_type, std::span<const std::byte> ros1_payload);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__ROS1_TO_CDR_HPP_
