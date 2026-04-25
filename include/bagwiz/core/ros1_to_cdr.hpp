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
// Returns nullopt for any type the converter does not handle; callers
// should drop those topics with a warning.
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
//     `load_introspection` helper. Any unknown type produces ok=false.
//   * Walks the destination type's field tree. ROS 1 raw and CDR share
//     primitive layouts (little-endian, same widths) so primitives are
//     a memcpy plus alignment padding.
//   * std_msgs/msg/Header has its 4-byte ROS 1 `seq` field skipped on
//     input; ROS 2 dropped this field.
//   * Custom types that are not in the whitelist still pass through if
//     the introspection happens to be loadable and structurally
//     identical, but they are gated by map_ros1_type() upstream so the
//     code path is normally not exercised for them.
Ros1ToCdrResult convert_ros1_to_cdr(
  std::string_view dest_ros2_type, std::span<const std::byte> ros1_payload);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__ROS1_TO_CDR_HPP_
