// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__IMAGE_ENCODER_HPP_
#define BAGWIZ__CORE__IMAGE__IMAGE_ENCODER_HPP_

#include "bagwiz/core/image/packed_raster.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// Encode a canonical packed BGR24 raster into a still-image bitstream. libav
// (FFmpeg) does the encode; like image_decoder.hpp this header exposes only std
// types so FFmpeg never enters bagwiz_image's export set. The inverse of
// decode_compressed_image(): a PNG produced here round-trips back to the same
// raster (PNG is lossless), which is exactly what walk's image preview needs to
// save the frame the user is looking at.
namespace bagwiz::core::image
{

// Outcome of encode_png(). On success `png` holds the PNG bitstream and `error`
// is empty; on failure `png` is empty and `error` explains why. Never throws.
// File I/O is the caller's responsibility, keeping the encoder pure and easy to
// test by round-tripping through decode_compressed_image().
struct EncodePngResult
{
  std::optional<std::vector<std::byte>> png;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return png.has_value() && error.empty(); }
};

// Encode `raster` (packed 8-bit BGR24, no row padding) into a PNG bitstream.
// Returns an error result (never throws) for an empty raster, a raster whose
// `bgr` size does not match width * 3 * height, dimensions that overflow the
// libav `int` API, a missing PNG encoder in this FFmpeg build, or any libav
// failure.
[[nodiscard]] EncodePngResult encode_png(const PackedRaster & raster);

// Outcome of encode_jpeg(), shaped like EncodePngResult.
struct EncodeJpegResult
{
  std::optional<std::vector<std::byte>> jpeg;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return jpeg.has_value() && error.empty(); }
};

// Bounds of encode_jpeg()'s quality, on the libjpeg-style 1 (smallest) to 100
// (best) scale that ROS image_transport's `jpeg_quality` parameter uses too.
inline constexpr int kJpegQualityMin = 1;
inline constexpr int kJpegQualityMax = 100;

// Encode `raster` (packed 8-bit BGR24, no row padding) into a baseline JPEG
// bitstream: 4:2:0 chroma subsampling, full (JPEG) range, so it decodes back
// the way a camera driver's own JPEG would. `quality` maps onto the MJPEG
// encoder's quantizer scale. Same failure conditions as encode_png(), plus a
// quality outside [kJpegQualityMin, kJpegQualityMax].
[[nodiscard]] EncodeJpegResult encode_jpeg(const PackedRaster & raster, int quality);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__IMAGE_ENCODER_HPP_
