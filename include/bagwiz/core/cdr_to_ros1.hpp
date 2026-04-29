// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CDR_TO_ROS1_HPP_
#define BAGWIZ__CORE__CDR_TO_ROS1_HPP_

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// Translate a ROS 2 type name (e.g. "std_msgs/msg/Header") to the ROS 1
// type name we will write into the output bag's connection record (e.g.
// "std_msgs/Header").
//
// The mapping is auto-derived for any well-formed `pkg/msg/Type` input
// by stripping the `/msg/` infix. A small override table (currently
// empty) handles future renames that don't follow this rule. Returns
// nullopt only for malformed inputs.
//
// Inverse of map_ros1_type(). When two ROS 1 names mapped to the same
// ROS 2 name on the forward path (e.g. legacy `tf/tfMessage` and modern
// `tf2_msgs/TFMessage` both pointing to `tf2_msgs/msg/TFMessage`), this
// reverse path picks the modern canonical form (`tf2_msgs/TFMessage`).
std::optional<std::string> map_ros2_type(std::string_view ros2_type);

struct CdrToRos1Result
{
  bool ok = false;
  std::string error;            // populated when ok == false
  std::vector<std::byte> ros1;  // raw ROS 1 wire payload, no encapsulation
};

// Convert one ROS 2 CDR-LE encapsulated payload into a ROS 1 raw wire
// payload. The source type name must be a ROS 2 form ("pkg/msg/Type");
// the caller is expected to have looked it up via map_ros2_type().
//
// Implementation notes (mirrors convert_ros1_to_cdr):
//   * Loads the source type's introspection via load_introspection().
//     Types whose package isn't installed produce ok=false; callers
//     handle that as a topic-skip.
//   * Skips the 4-byte CDR encapsulation header on input.
//   * Walks the field tree; ROS 1 raw and CDR-LE share primitive
//     layouts so primitives are a memcpy after consuming CDR alignment
//     padding.
//   * std_msgs/msg/Header has its 4-byte ROS 1 `seq` field synthesized
//     as zero on output; the original ROS 1 seq is unrecoverable from a
//     ROS 2 bag.
//   * String fields strip the trailing NUL added by CDR (and the +1 in
//     the length prefix) when emitting the ROS 1 form.
CdrToRos1Result convert_cdr_to_ros1(
  std::string_view src_ros2_type, std::span<const std::byte> cdr_payload);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__CDR_TO_ROS1_HPP_
