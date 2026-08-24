// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/rectify.hpp"

#include "bagwiz/core/image/camera_info.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{

using bagwiz::core::image::CameraInfo;
using bagwiz::core::image::RectifyHelper;

CameraInfo identity_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info;
  info.width = w;
  info.height = h;
  info.distortion_model = "plumb_bob";
  info.k = {
    static_cast<double>(w),
    0.0,
    static_cast<double>(w) / 2.0,
    0.0,
    static_cast<double>(h),
    static_cast<double>(h) / 2.0,
    0.0,
    0.0,
    1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {info.k[0], 0.0, info.k[2], 0.0, 0.0, info.k[4], info.k[5], 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

CameraInfo distorted_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info = identity_camera_info(w, h);
  info.d = {0.2, -0.1, 0.0, 0.0, 0.0};
  return info;
}

// Some monocular publishers leave CameraInfo.r zero-filled instead of identity.
CameraInfo zero_rotation_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info = identity_camera_info(w, h);
  info.r = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return info;
}

// A fisheye (equidistant) camera: 4 coefficients, k1 strong enough that the
// fisheye and Brown-Conrady readings of the same d produce visibly different
// maps at this image size.
CameraInfo equidistant_camera_info(std::uint32_t w, std::uint32_t h)
{
  CameraInfo info = identity_camera_info(w, h);
  info.distortion_model = "equidistant";
  info.d = {-0.2, 0.05, 0.0, 0.0};
  return info;
}

// The remap cv::fisheye itself produces for `info`, as the ground truth the
// helper must match byte-for-byte on an equidistant camera. Truncates/pads d
// to the exactly-4 coefficients cv::fisheye requires, mirroring the helper's
// own convention (and camera_distortion's: missing entries are zero, extras
// are ignored).
std::vector<std::byte> fisheye_ground_truth(
  const CameraInfo & info, std::uint32_t w, std::uint32_t h, const std::vector<std::byte> & src)
{
  std::array<double, 9> k = info.k;
  std::array<double, 9> r = info.r;
  std::array<double, 12> p = info.p;
  std::array<double, 4> d{};
  for (std::size_t i = 0; i < d.size() && i < info.d.size(); ++i) {
    d[i] = info.d[i];
  }
  const cv::Size size{static_cast<int>(w), static_cast<int>(h)};
  cv::Mat map1;
  cv::Mat map2;
  cv::fisheye::initUndistortRectifyMap(
    cv::Mat(3, 3, CV_64F, k.data()), cv::Mat(4, 1, CV_64F, d.data()),
    cv::Mat(3, 3, CV_64F, r.data()), cv::Mat(3, 4, CV_64F, p.data()), size, CV_32FC1, map1, map2);
  auto src_copy = src;
  std::vector<std::byte> out(src.size());
  const cv::Mat in(
    static_cast<int>(h), static_cast<int>(w), CV_8UC3, src_copy.data(), std::size_t{w} * 3);
  cv::Mat out_mat(
    static_cast<int>(h), static_cast<int>(w), CV_8UC3, out.data(), std::size_t{w} * 3);
  cv::remap(in, out_mat, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{});
  return out;
}

std::vector<std::byte> solid_bgr(
  std::uint32_t w, std::uint32_t h, std::uint8_t b, std::uint8_t g, std::uint8_t r)
{
  std::vector<std::byte> out(static_cast<std::size_t>(w) * h * 3);
  for (std::size_t i = 0; i < out.size(); i += 3) {
    out[i] = static_cast<std::byte>(b);
    out[i + 1] = static_cast<std::byte>(g);
    out[i + 2] = static_cast<std::byte>(r);
  }
  return out;
}

std::vector<std::byte> gradient_bgr(std::uint32_t w, std::uint32_t h)
{
  std::vector<std::byte> out(static_cast<std::size_t>(w) * h * 3);
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 3;
      out[i] = static_cast<std::byte>((x * 16) % 256);
      out[i + 1] = static_cast<std::byte>((y * 16) % 256);
      out[i + 2] = static_cast<std::byte>(((x + y) * 8) % 256);
    }
  }
  return out;
}

TEST(RectifyHelper, ZeroDistortionPreservesCenterColor)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = identity_camera_info(kW, kH);
  RectifyHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

TEST(RectifyHelper, ZeroRectificationMatrixFallsBackToIdentity)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = zero_rotation_camera_info(kW, kH);
  RectifyHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  // A zero (unset) rectification matrix must be treated as identity, matching
  // tier4_perception_dataset. Without the guard, initUndistortRectifyMap emits
  // NaN maps and remap fills the whole image with the border color (black).
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

TEST(RectifyHelper, NonZeroDistortionChangesPixels)
{
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  const auto info = distorted_camera_info(kW, kH);
  RectifyHelper helper(info, kW, kH);
  const auto input = gradient_bgr(kW, kH);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  EXPECT_FALSE(std::equal(output.begin(), output.end(), input.begin()));
  // At least one interior pixel should change (not just the border).
  bool interior_changed = false;
  for (std::uint32_t y = 1; y < kH - 1 && !interior_changed; ++y) {
    for (std::uint32_t x = 1; x < kW - 1; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * kW + x) * 3;
      if (output[idx] != input[idx]) {
        interior_changed = true;
        break;
      }
    }
  }
  EXPECT_TRUE(interior_changed);
}

TEST(RectifyHelper, ScalesToDifferentSize)
{
  constexpr std::uint32_t kSrcW = 16;
  constexpr std::uint32_t kSrcH = 16;
  constexpr std::uint32_t kDstW = 8;
  constexpr std::uint32_t kDstH = 8;
  const auto info = identity_camera_info(kSrcW, kSrcH);
  RectifyHelper helper(info, kDstW, kDstH);
  const auto input = solid_bgr(kSrcW, kSrcH, 50, 100, 150);
  const auto output = helper.remap(input, kSrcW * 3);
  EXPECT_EQ(output.size(), static_cast<std::size_t>(kDstW) * kDstH * 3);
}

TEST(RectifyHelper, EffectiveCameraInfoMatchesScaledSize)
{
  constexpr std::uint32_t kSrcW = 16;
  constexpr std::uint32_t kSrcH = 16;
  constexpr std::uint32_t kDstW = 8;
  constexpr std::uint32_t kDstH = 8;
  const auto info = identity_camera_info(kSrcW, kSrcH);
  RectifyHelper helper(info, kDstW, kDstH);
  const auto effective = helper.effective_camera_info();
  EXPECT_EQ(effective.width, kDstW);
  EXPECT_EQ(effective.height, kDstH);
  // fx/fy and cx/cy should be scaled by 0.5.
  EXPECT_DOUBLE_EQ(effective.k[0], info.k[0] * 0.5);
  EXPECT_DOUBLE_EQ(effective.k[2], info.k[2] * 0.5);
  EXPECT_DOUBLE_EQ(effective.k[4], info.k[4] * 0.5);
  EXPECT_DOUBLE_EQ(effective.k[5], info.k[5] * 0.5);
}

TEST(RectifyHelper, EquidistantModelMatchesFisheyeGroundTruth)
{
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 32;
  for (const char * model : {"equidistant", "fisheye"}) {
    auto info = equidistant_camera_info(kW, kH);
    info.distortion_model = model;
    RectifyHelper helper(info, kW, kH);
    const auto input = gradient_bgr(kW, kH);
    const auto output = helper.remap(input, kW * 3);
    const auto expected = fisheye_ground_truth(info, kW, kH, input);
    ASSERT_EQ(output.size(), expected.size());
    EXPECT_TRUE(std::equal(output.begin(), output.end(), expected.begin()))
      << "model '" << model
      << "' must rectify through cv::fisheye::initUndistortRectifyMap; the Brown-Conrady map "
         "builder misreads its 4 coefficients as [k1, k2, p1, p2]";
  }
}

TEST(RectifyHelper, EquidistantIgnoresExtraCoefficients)
{
  // cv::fisheye requires exactly 4 coefficients, but tools that always emit 5
  // exist; the helper must use the first four rather than abort.
  constexpr std::uint32_t kW = 32;
  constexpr std::uint32_t kH = 32;
  auto info = equidistant_camera_info(kW, kH);
  info.d = {-0.2, 0.05, 0.0, 0.0, 0.0};
  RectifyHelper helper(info, kW, kH);
  const auto input = gradient_bgr(kW, kH);
  const auto output = helper.remap(input, kW * 3);
  const auto expected = fisheye_ground_truth(info, kW, kH, input);
  EXPECT_TRUE(std::equal(output.begin(), output.end(), expected.begin()));
}

TEST(RectifyHelper, EquidistantZeroRotationFallsBackToIdentity)
{
  // The zero-filled-r fallback must hold on the fisheye path too: the helper
  // passes an empty R, which cv::fisheye must accept as identity rather than
  // assert, and the center pixel must survive like in the plumb_bob variant.
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  auto info = equidistant_camera_info(kW, kH);
  info.r = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  RectifyHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

TEST(RectifyHelper, EquidistantEmptyDistortionPreservesCenterColor)
{
  // An empty d pads to four zeros (cv::fisheye rejects an empty D outright),
  // which is an identity distortion: the center pixel must survive.
  constexpr std::uint32_t kW = 16;
  constexpr std::uint32_t kH = 16;
  auto info = equidistant_camera_info(kW, kH);
  info.d.clear();
  RectifyHelper helper(info, kW, kH);
  const auto input = solid_bgr(kW, kH, 100, 150, 200);
  const auto output = helper.remap(input, kW * 3);
  ASSERT_EQ(output.size(), input.size());
  const std::size_t center = (kH / 2 * kW + kW / 2) * 3;
  EXPECT_EQ(output[center], input[center]);
  EXPECT_EQ(output[center + 1], input[center + 1]);
  EXPECT_EQ(output[center + 2], input[center + 2]);
}

}  // namespace
