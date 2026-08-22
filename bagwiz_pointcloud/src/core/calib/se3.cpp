// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/se3.hpp"

#include <algorithm>
#include <cmath>

namespace bagwiz::core::calib
{

Mat4 identity_mat4()
{
  Mat4 m{};
  m[0] = m[5] = m[10] = m[15] = 1.0;
  return m;
}

Mat4 make_transform(const std::array<double, 3> & xyz, const std::array<double, 3> & rpy)
{
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  Mat4 m = identity_mat4();
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), stored column-major (m[col*4+row]).
  m[0] = cy * cp;
  m[1] = sy * cp;
  m[2] = -sp;
  m[4] = cy * sp * sr - sy * cr;
  m[5] = sy * sp * sr + cy * cr;
  m[6] = cp * sr;
  m[8] = cy * sp * cr + sy * sr;
  m[9] = sy * sp * cr - cy * sr;
  m[10] = cp * cr;
  m[12] = xyz[0];
  m[13] = xyz[1];
  m[14] = xyz[2];
  return m;
}

Mat4 mat4_multiply(const Mat4 & a, const Mat4 & b)
{
  Mat4 out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      double v = 0.0;
      for (int k = 0; k < 4; ++k) {
        v += a[k * 4 + row] * b[col * 4 + k];
      }
      out[col * 4 + row] = v;
    }
  }
  return out;
}

Mat4 rigid_inverse(const Mat4 & t)
{
  Mat4 m = identity_mat4();
  // R^T into the rotation block.
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      m[col * 4 + row] = t[row * 4 + col];
    }
  }
  // -R^T * translation.
  for (int row = 0; row < 3; ++row) {
    m[12 + row] = -(m[0 * 4 + row] * t[12] + m[1 * 4 + row] * t[13] + m[2 * 4 + row] * t[14]);
  }
  return m;
}

std::array<double, 3> translation_of(const Mat4 & t)
{
  return {t[12], t[13], t[14]};
}

std::array<double, 3> rpy_of(const Mat4 & t)
{
  // Inverse of make_transform: R = Rz(y)Ry(p)Rx(r). Gimbal lock is irrelevant
  // here — callers only extract small deltas or bag-scale mount angles.
  const double roll = std::atan2(t[6], t[10]);
  const double pitch = std::asin(-t[2]);
  const double yaw = std::atan2(t[1], t[0]);
  return {roll, pitch, yaw};
}

double rotation_angle_between(const Mat4 & a, const Mat4 & b)
{
  // trace(R_a^T * R_b): dot products of the two rotation blocks' matching
  // columns, which never needs the relative rotation materialized.
  double trace = 0.0;
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      trace += a[col * 4 + row] * b[col * 4 + row];
    }
  }
  // Clamp against roundoff pushing (trace - 1) / 2 outside acos's domain.
  const double c = std::clamp((trace - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c);
}

}  // namespace bagwiz::core::calib
