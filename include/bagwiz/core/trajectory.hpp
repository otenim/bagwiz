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

// Extract a pose sample from a deserialized ROS 2 message by introspection.
//
// Supported shapes (no template specialization required):
//   * `header.stamp` + `pose.position` + `pose.orientation`
//     (geometry_msgs/msg/PoseStamped, geometry_msgs/msg/PoseWithCovariance
//     + a header, etc.)
//   * `header.stamp` + `pose.pose.position` + `pose.pose.orientation`
//     (nav_msgs/msg/Odometry and its kin)
//   * `header.stamp` + `transform.translation` + `transform.rotation`
//     (geometry_msgs/msg/TransformStamped)
//
// Returns std::nullopt when no recognizable (header, pose|transform)
// pair can be located. The caller should surface that as a clear CLI
// error.
std::optional<TrajectoryPose> extract_pose(
  const rosidl_typesupport_introspection_cpp::MessageMembers & members, const void * base);

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
