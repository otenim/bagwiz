// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__CAMERA_INFO_HPP_
#define BAGWIZ__CORE__IMAGE__CAMERA_INFO_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::image
{

// A subset of sensor_msgs/msg/CameraInfo fields needed for rectification and
// point-cloud overlay transform lookup.
struct CameraInfo
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string distortion_model;
  std::vector<double> d;       // distortion coefficients
  std::array<double, 9> k{};   // intrinsic camera matrix
  std::array<double, 9> r{};   // rectification matrix
  std::array<double, 12> p{};  // projection/camera matrix
  std::string frame_id;        // header.frame_id
};

// Outcome of extract_camera_info(). On success `info` is set and `error` is
// empty; on failure `info` is empty and `error` explains why. Never throws.
struct CameraInfoResult
{
  std::optional<CameraInfo> info;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return info.has_value() && error.empty(); }
};

// Parse a CDR-serialized sensor_msgs/msg/CameraInfo payload. Only the fields
// required for rectification are extracted; the rest are skipped. The payload
// must outlive parsing (no copy). A truncated or malformed payload yields an
// error result rather than throwing.
[[nodiscard]] CameraInfoResult extract_camera_info(std::span<const std::byte> payload);

// Return a CameraInfo whose pixel-coordinate entries are scaled by `scale` so
// that rectification/projection match an image resized by the same factor. The
// distortion coefficients, rectification matrix, homogeneous entries (k[8],
// p[10]), and the depth translation in p[11] are left unchanged.
[[nodiscard]] CameraInfo scale_camera_info(const CameraInfo & info, double scale);

// Return a CameraInfo whose pixel-coordinate entries are scaled by independent
// x and y factors. Use this when the output image has a different aspect ratio
// than the original calibration.
[[nodiscard]] CameraInfo scale_camera_info(const CameraInfo & info, double scale_x, double scale_y);

// Return the CameraInfo to use for an image that is actually `width` x `height`
// pixels: a verbatim copy when the calibration was made at that resolution, and
// an independently x/y-scaled copy when it was not (a calibration recorded at a
// different resolution than the topic's images, which is common once a pipeline
// downscales). A CameraInfo with zero dimensions carries no resolution to
// compare against and is returned unchanged.
//
// Every consumer that projects into, or rectifies, a concrete image must go
// through this rather than using the raw CameraInfo, or points land off by the
// resolution ratio. RectifyHelper applies it internally (see
// RectifyHelper::effective_camera_info), so callers that already hold a
// helper should ask it instead of calling this twice.
[[nodiscard]] CameraInfo camera_info_for_size(
  const CameraInfo & info, std::uint32_t width, std::uint32_t height);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__CAMERA_INFO_HPP_
