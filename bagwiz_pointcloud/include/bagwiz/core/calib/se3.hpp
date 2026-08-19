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

/// Apply a rigid transform to a 3D point.
[[nodiscard]] std::array<double, 3> transform_point(
  const Mat4 & t, const std::array<double, 3> & p);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__SE3_HPP_
