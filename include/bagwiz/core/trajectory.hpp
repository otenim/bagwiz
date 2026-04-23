// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TRAJECTORY_HPP_
#define BAGWIZ__CORE__TRAJECTORY_HPP_

#include <cstdint>
#include <optional>
#include <ostream>
#include <span>
#include <string>

namespace rosidl_typesupport_introspection_cpp
{
struct MessageMembers_s;
using MessageMembers = MessageMembers_s;
}  // namespace rosidl_typesupport_introspection_cpp

namespace bagwiz::core
{

// One sample along a trajectory. Quaternion convention matches ROS /
// TUM: (qx, qy, qz, qw), Hamilton, normalized.
struct TrajectoryPose
{
  int64_t timestamp_ns = 0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0;
};

// Outcome of a pose extraction.
//   * `used_header_stamp` is true when the timestamp came from the
//     message's `header.stamp`; false when the message had no header
//     and the caller-supplied fallback was used. The CLI uses this to
//     emit a one-shot warning for unstamped types so users know the
//     timestamps in the TUM file come from bag log time rather than
//     from the sensor clock.
//   * `frame_id` is the reference (fixed) frame the pose is expressed
//     in (from `header.frame_id`). Empty for unstamped types.
//   * `child_frame_id` is the tracked frame -- the moving thing the
//     pose describes. Only Odometry and TransformStamped carry a
//     top-level `child_frame_id`; it is empty for the other shapes.
struct PoseExtraction
{
  TrajectoryPose pose;
  std::string frame_id;
  std::string child_frame_id;
  bool used_header_stamp = false;
};

// Extract a pose sample from a deserialized ROS 2 message by introspection.
//
// Supported shapes (no template specialization required):
//   * `header.stamp` + `pose.{position, orientation}`
//     (geometry_msgs/msg/PoseStamped, PoseWithCovarianceStamped)
//   * `header.stamp` + `pose.pose.{position, orientation}`
//     (nav_msgs/msg/Odometry and its kin)
//   * `header.stamp` + `transform.{translation, rotation}`
//     (geometry_msgs/msg/TransformStamped)
//   * `position` + `orientation` (no header)
//     (geometry_msgs/msg/Pose)
//   * `translation` + `rotation` (no header)
//     (geometry_msgs/msg/Transform)
//
// `fallback_timestamp_ns` is used when the message has no header.stamp;
// callers should pass the bag log time (recorder receive time) so the
// output trajectory still has a sensible time axis.
//
// Returns std::nullopt when the pose fields cannot be located (the
// message does not look like any supported shape).
std::optional<PoseExtraction> extract_pose(
  const rosidl_typesupport_introspection_cpp::MessageMembers & members, const void * base,
  int64_t fallback_timestamp_ns);

// Write poses in the TUM trajectory format: one sample per line,
//
//     timestamp tx ty tz qx qy qz qw
//
// with the timestamp in seconds (9 decimal places so nanosecond precision
// is preserved). No comment header is emitted so the output drops
// straight into tools like evo without post-processing.
void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TRAJECTORY_HPP_
