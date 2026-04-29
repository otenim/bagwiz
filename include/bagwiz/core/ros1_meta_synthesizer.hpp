// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__ROS1_META_SYNTHESIZER_HPP_
#define BAGWIZ__CORE__ROS1_META_SYNTHESIZER_HPP_

#include "bagwiz/core/ros1_message_definitions.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// One normalisation event surfaced to the caller. Emitted whenever a
// ROS 2-only construct had to be downgraded to a ROS 1-compatible form
// (e.g. a bounded array losing its bound). Wire-irrelevant per project
// decision 10/B, but the caller may want to log them so users can
// investigate "MD5 matched but Foxglove shows something odd" later.
struct Ros1MetaWarning
{
  std::string type;    // e.g. "geometry_msgs/PoseStamped"
  std::string field;   // dotted path, e.g. "pose.position"
  std::string kind;    // "bound_dropped" / "default_dropped"
  std::string detail;  // human-readable, e.g. "string<=64 -> string"
};

// Result of synthesising the (md5sum, message_definition) pair for a
// ROS 2 type. On success, `meta` carries the canonical ROS 1 form ready
// to drop into a ROS 1 `Connection` record; on failure, `error` carries
// the reason and the other fields are empty.
//
// `warnings` is independent of ok/error — successful synthesis can still
// surface bound/default drops the caller may want to log.
struct Ros1MetaResult
{
  bool ok = false;
  std::string error;
  Ros1TypeMeta meta;
  std::vector<Ros1MetaWarning> warnings;
};

// Synthesise the ROS 1 connection metadata (md5sum + message_definition
// concatenated form) from a ROS 2 type's `.msg` text.
//
// Inputs:
//   * `ros2_type` — the canonical ROS 2 form, e.g. "sensor_msgs/msg/Imu".
//     Either the canonical form (with `/msg/`) or the short form
//     (`pkg/Type`) is accepted.
//   * `ros2_msg_text` — the MCAP-style concatenated `.msg` text:
//     the root body, then `===` separators, then `MSG: pkg/msg/Type`
//     headers introducing each transitively-referenced dependency.
//     Both `pkg/Type` and `pkg/msg/Type` forms are accepted in
//     dependency headers and field type references.
//
// Normalisation rules (project decision 10/B):
//   * `pkg/msg/Type` references → `pkg/Type` form (.msg layer name)
//   * `builtin_interfaces/Time` field → `time` primitive
//   * `builtin_interfaces/Duration` field → `duration` primitive
//   * `std_msgs/Header` body emitted with the ROS 1-style
//     `uint32 seq` field restored at the front
//   * Default values dropped (warning)
//   * Bounded arrays `T[<=N]` → `T[]` (warning)
//   * Bounded strings `string<=N` → `string` (warning)
//   * `wstring` is wire-incompatible with ROS 1 `string` → refuse
//     synthesis (returns ok=false).
//
// MD5 is computed using the canonical ROS 1 algorithm (gentools.py
// equivalent): for each constant emit `<type> <name>=<value>`; for each
// field emit `<type><array_suffix> <name>` for primitive bases or
// `<recursive_md5> <name>` (no array suffix) for nested bases; join
// lines with `\n`, strip trailing `\n`, hash with MD5.
Ros1MetaResult synthesize_ros1_meta(std::string_view ros2_type, std::string_view ros2_msg_text);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__ROS1_META_SYNTHESIZER_HPP_
