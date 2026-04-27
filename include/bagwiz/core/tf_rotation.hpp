// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_ROTATION_HPP_
#define BAGWIZ__CORE__TF_ROTATION_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>

namespace bagwiz::core
{

struct EulerAngles
{
  double roll;
  double pitch;
  double yaw;
};

// Convention matches tf2::Matrix3x3::getRPY (and setRPY): intrinsic Z-Y-X
// (yaw-pitch-roll) Tait-Bryan angles, i.e. the same convention used
// throughout ROS for tf transforms.
inline EulerAngles quat_to_euler_rad(double x, double y, double z, double w)
{
  const tf2::Quaternion q(x, y, z, w);
  EulerAngles rpy{};
  tf2::Matrix3x3(q).getRPY(rpy.roll, rpy.pitch, rpy.yaw);
  return rpy;
}

inline std::array<double, 4> euler_rad_to_quat(const EulerAngles & rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy.roll, rpy.pitch, rpy.yaw);
  return {q.x(), q.y(), q.z(), q.w()};
}

inline EulerAngles euler_rad_to_euler_deg(const EulerAngles & rpy)
{
  constexpr double kRadToDeg = 180.0 / M_PI;
  return {rpy.roll * kRadToDeg, rpy.pitch * kRadToDeg, rpy.yaw * kRadToDeg};
}

inline EulerAngles euler_deg_to_euler_rad(const EulerAngles & rpy)
{
  constexpr double kDegToRad = M_PI / 180.0;
  return {rpy.roll * kDegToRad, rpy.pitch * kDegToRad, rpy.yaw * kDegToRad};
}

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_ROTATION_HPP_
