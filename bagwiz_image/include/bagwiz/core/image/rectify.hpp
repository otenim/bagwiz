// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__RECTIFY_HPP_
#define BAGWIZ__CORE__IMAGE__RECTIFY_HPP_

#include "bagwiz/core/image/camera_info.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace bagwiz::core::image
{

// OpenCV-based rectification helper. Initializes distortion/rectification maps
// from CameraInfo and the target image size, then remaps subsequent frames.
// Implementation is hidden with a pimpl so OpenCV headers do not leak into
// bagwiz_image's public interface.
class RectifyHelper
{
public:
  RectifyHelper(const CameraInfo & info, std::uint32_t width, std::uint32_t height);
  ~RectifyHelper();

  RectifyHelper(RectifyHelper &&) noexcept;
  RectifyHelper & operator=(RectifyHelper &&) noexcept;

  RectifyHelper(const RectifyHelper &) = delete;
  RectifyHelper & operator=(const RectifyHelper &) = delete;

  // Apply rectification to a packed 8-bit BGR24 frame. `src_step` is the row
  // stride in bytes. Returns a span over an internal packed BGR24 output buffer.
  [[nodiscard]] std::span<const std::byte> remap(
    std::span<const std::byte> src, std::uint32_t src_step);

  // Return the CameraInfo actually used for rectification. This is the input
  // CameraInfo scaled to the target image size, which is what callers need when
  // projecting other data (e.g. point clouds) onto the rectified image.
  [[nodiscard]] CameraInfo effective_camera_info() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__RECTIFY_HPP_
