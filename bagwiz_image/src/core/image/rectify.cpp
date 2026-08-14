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

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

// CameraInfo.r is the rectification rotation: identity for monocular cameras and
// a real rotation for stereo-rectified ones. Some publishers leave it zero-filled
// (or otherwise unset). Feeding a zero / non-finite R to initUndistortRectifyMap
// produces NaN maps and a black image, so honor R only when it is a genuine,
// finite matrix and otherwise fall back to identity — matching
// tier4_perception_dataset, which always passes identity.
[[nodiscard]] bool is_usable_rotation(const std::array<double, 9> & r)
{
  bool all_zero = true;
  for (const double v : r) {
    if (!std::isfinite(v)) {
      return false;
    }
    if (v != 0.0) {
      all_zero = false;
    }
  }
  return !all_zero;
}

}  // namespace

class RectifyHelper::Impl
{
public:
  Impl(const CameraInfo & info, std::uint32_t width, std::uint32_t height)
  : width_(width), height_(height), effective_info_(camera_info_for_size(info, width, height))
  {
    const cv::Mat k(3, 3, CV_64F, effective_info_.k.data());
    const cv::Mat p(3, 4, CV_64F, effective_info_.p.data());

    // An empty R makes initUndistortRectifyMap use identity; only pass an
    // explicit R when CameraInfo carries a usable rectification rotation.
    cv::Mat r;
    if (is_usable_rotation(effective_info_.r)) {
      r = cv::Mat(3, 3, CV_64F, effective_info_.r.data());
    }

    cv::Mat d;
    if (!effective_info_.d.empty()) {
      d = cv::Mat(static_cast<int>(effective_info_.d.size()), 1, CV_64F, effective_info_.d.data());
    }

    cv::initUndistortRectifyMap(
      k, d, r, p, cv::Size{static_cast<int>(width), static_cast<int>(height)}, CV_32FC1, map1_,
      map2_);

    output_.resize(static_cast<std::size_t>(width) * height * 3, std::byte{0});
  }

  [[nodiscard]] std::span<const std::byte> remap(
    std::span<const std::byte> src, std::uint32_t src_step)
  {
    // OpenCV's external-data cv::Mat constructor takes a non-const void* even
    // for read-only input; remap does not mutate the source, so const_cast is safe.
    const cv::Mat in(
      static_cast<int>(height_), static_cast<int>(width_), CV_8UC3,
      const_cast<std::byte *>(src.data()), src_step);
    cv::Mat out(
      static_cast<int>(height_), static_cast<int>(width_), CV_8UC3, output_.data(), width_ * 3);
    cv::remap(in, out, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar{});
    return {output_.data(), output_.size()};
  }

  [[nodiscard]] const CameraInfo & effective_camera_info() const { return effective_info_; }

private:
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  cv::Mat map1_;
  cv::Mat map2_;
  std::vector<std::byte> output_;
  CameraInfo effective_info_;
};

RectifyHelper::RectifyHelper(const CameraInfo & info, std::uint32_t width, std::uint32_t height)
: impl_(std::make_unique<Impl>(info, width, height))
{
}

RectifyHelper::~RectifyHelper() = default;

RectifyHelper::RectifyHelper(RectifyHelper &&) noexcept = default;
RectifyHelper & RectifyHelper::operator=(RectifyHelper &&) noexcept = default;

std::span<const std::byte> RectifyHelper::remap(
  std::span<const std::byte> src, std::uint32_t src_step)
{
  return impl_->remap(src, src_step);
}

CameraInfo RectifyHelper::effective_camera_info() const
{
  return impl_->effective_camera_info();
}

}  // namespace bagwiz::core::image
