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
#include <ostream>
#include <span>

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

// Write poses in the TUM trajectory format: one sample per line,
//
//     timestamp tx ty tz qx qy qz qw
//
// with the timestamp in seconds (9 decimal places so nanosecond
// precision is preserved). No comment header is emitted so the output
// drops straight into tools like evo without post-processing.
void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TRAJECTORY_HPP_
