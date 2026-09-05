// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__PIXEL_SOURCE_HPP_
#define BAGWIZ__CORE__VIDEO__PIXEL_SOURCE_HPP_

#include <cstddef>
#include <cstdint>

// The source-frame shapes the encoders accept: a packed 8-bit 3-channel
// raster, or borrowed 4:2:0 planes handed over without conversion.
namespace bagwiz::core::video
{

// Pixel layout of a packed source frame. Both are 8-bit, 3-channel,
// interleaved — matching ROS "bgr8" / "rgb8".
enum class SourcePixelFormat { kBgr8, kRgb8 };

// One 4:2:0 planar frame, borrowed: the luma plane and the two half-size
// chroma planes, each with its own row stride in bytes.
struct Yuv420Planes
{
  const std::uint8_t * y = nullptr;
  std::size_t y_stride = 0;
  const std::uint8_t * u = nullptr;
  std::size_t u_stride = 0;
  const std::uint8_t * v = nullptr;
  std::size_t v_stride = 0;
};

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__PIXEL_SOURCE_HPP_
