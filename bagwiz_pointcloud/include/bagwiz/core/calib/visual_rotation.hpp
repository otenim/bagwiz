// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__VISUAL_ROTATION_HPP_
#define BAGWIZ__CORE__CALIB__VISUAL_ROTATION_HPP_

#include "bagwiz/core/calib/nid_cost.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// Frame-to-frame camera rotation from two gray images — the "visual gyro"
// that time_offset.hpp matches against a trajectory or an IMU. Corners are
// tracked between the frames (pyramidal Lucas-Kanade with a forward-backward
// check), undistorted through the camera model, and the rotation is solved
// two ways: through the essential matrix (right when the platform
// translates, where the pure-rotation model is biased by parallax) and as a
// pure rotation (right when it barely moves, where the essential matrix
// degenerates). The caller keeps whichever series fits its reference better.
// OpenCV is an implementation detail: nothing of it appears in this header.
namespace bagwiz::core::calib
{

struct VisualRotationParams
{
  int max_corners = 2000;
  double quality_level = 0.01;
  double min_distance_px = 12.0;
  double forward_backward_max_px = 0.7;  // LK consistency check
  double essential_threshold_px = 0.6;   // RANSAC reprojection threshold
  double rotation_threshold_px = 1.0;    // pure-rotation inlier threshold
  int rotation_ransac_iterations = 300;
  std::size_t min_tracks = 30;  // fewer tracked corners: no estimate
  std::uint64_t seed = 0;
};

// Rotation of the camera from `prev` to `next`, as a rotation vector in the
// optical frame of `prev` (i.e. log(R) with x_prev = R x_next for a fixed
// world direction). Each estimate is nullopt when its solver had too few
// inliers or failed outright.
struct FramePairRotation
{
  std::optional<std::array<double, 3>> essential;
  std::optional<std::array<double, 3>> pure_rotation;
  std::size_t tracks = 0;  // corners that survived the forward-backward check
  std::size_t inliers_essential = 0;
  std::size_t inliers_rotation = 0;
  double median_flow_px = 0.0;  // over the tracks
};

// `cam` must describe the images as passed (same width/height, intrinsics
// in their pixels); scale_camera_model() adapts a full-resolution model to
// a downscaled image.
[[nodiscard]] FramePairRotation frame_pair_rotation(
  const GrayImage & prev, const GrayImage & next, const CameraModel & cam,
  const VisualRotationParams & params);

// Area-averaged downscale by `scale` in (0, 1]; 1 returns a copy.
[[nodiscard]] GrayImage downscale_gray(const GrayImage & in, double scale);

// The intrinsics of `cam` for an image downscaled by `scale` (fx, fy, cx, cy,
// width and height scaled; distortion coefficients are scale-free).
[[nodiscard]] CameraModel scale_camera_model(const CameraModel & cam, double scale);

// The scale in (0, 1] that brings the longer image side down to at most
// `max_side` pixels (1 when it already is).
[[nodiscard]] double scale_for_max_side(
  std::uint32_t width, std::uint32_t height, std::uint32_t max_side);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__VISUAL_ROTATION_HPP_
