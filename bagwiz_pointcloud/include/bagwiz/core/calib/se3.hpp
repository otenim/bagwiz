// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__SE3_HPP_
#define BAGWIZ__CORE__CALIB__SE3_HPP_

#include <array>

namespace bagwiz::core::calib
{

/// Column-major 4x4 rigid transforms.
/// Matches projector.cpp's transform layout so calib and overlay code share one convention.
/// m[12..14] = translation; m[col*4+row] indexing.
using Mat4 = std::array<double, 16>;

/// Return the identity transform.
[[nodiscard]] Mat4 identity_mat4();

/// Build a rigid transform from translation and roll-pitch-yaw angles.
/// tf2 fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
[[nodiscard]] Mat4 make_transform(
  const std::array<double, 3> & xyz, const std::array<double, 3> & rpy);

/// Multiply two transforms: (a * b) applies b first, then a.
[[nodiscard]] Mat4 mat4_multiply(const Mat4 & a, const Mat4 & b);

/// Compute the inverse of a rigid transform (rotation + translation).
[[nodiscard]] Mat4 rigid_inverse(const Mat4 & t);

/// Extract the translation vector from a transform.
[[nodiscard]] std::array<double, 3> translation_of(const Mat4 & t);

/// Extract the roll-pitch-yaw angles from a transform (inverse of make_transform).
[[nodiscard]] std::array<double, 3> rpy_of(const Mat4 & t);

/// Angle in radians of the relative rotation between the two transforms'
/// rotation blocks (axis-angle magnitude of R_a^T * R_b, via the trace
/// formula). Translation does not contribute. Always in [0, pi].
[[nodiscard]] double rotation_angle_between(const Mat4 & a, const Mat4 & b);

/// The three coordinates of a rigid transform applied to a 3D point, one at a
/// time, for callers that can reject on one coordinate (a depth window, say)
/// before paying for the other two. transform_point is defined as exactly
/// these three expressions, so a coordinate computed through one of them is
/// the same double as the matching component of transform_point. Inline
/// because they sit in the hottest per-point loops.
[[nodiscard]] inline double transform_point_x(const Mat4 & t, const std::array<double, 3> & p)
{
  return t[0] * p[0] + t[4] * p[1] + t[8] * p[2] + t[12];
}
[[nodiscard]] inline double transform_point_y(const Mat4 & t, const std::array<double, 3> & p)
{
  return t[1] * p[0] + t[5] * p[1] + t[9] * p[2] + t[13];
}
[[nodiscard]] inline double transform_point_z(const Mat4 & t, const std::array<double, 3> & p)
{
  return t[2] * p[0] + t[6] * p[1] + t[10] * p[2] + t[14];
}

/// Apply a rigid transform to a 3D point.
[[nodiscard]] inline std::array<double, 3> transform_point(
  const Mat4 & t, const std::array<double, 3> & p)
{
  return {transform_point_x(t, p), transform_point_y(t, p), transform_point_z(t, p)};
}

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__SE3_HPP_
