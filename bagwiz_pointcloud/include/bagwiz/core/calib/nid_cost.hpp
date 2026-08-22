// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__NID_COST_HPP_
#define BAGWIZ__CORE__CALIB__NID_COST_HPP_

#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "bagwiz/core/image/camera_distortion.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace bagwiz::core::calib
{

struct GrayImage
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> gray;  // row-major, width * height bytes
};

struct CameraModel
{
  std::array<double, 9> k{};  // row-major intrinsics (CameraInfo.k)
  image::DistortionModel model = image::DistortionModel::kNone;
  std::vector<double> d;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
};

struct CalibSample
{
  GrayImage image;
  Mat4 t_world_trajframe{};                        // trajectory pose at the image stamp
  std::vector<std::array<float, 3>> points_world;  // pre-culled candidates
  std::vector<std::uint8_t> intensity_bins;        // parallel, in [0, bins)
};

struct NidParams
{
  int bins = 16;
  double min_depth = 2.0;
  double max_depth = 150.0;
  std::uint32_t cull_cell_px = 8;
  float cull_margin_m = 0.75F;
  std::size_t min_points = 1000;
};

// BGR24 (packed, stride == width*3) -> GrayImage via integer BT.601 weights.
[[nodiscard]] GrayImage gray_from_bgr24(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height);

// Histogram-equalize raw intensities into [0, bins) so skewed lidar
// intensity distributions still spread across the joint histogram. Given a
// pool, the sort and the per-value ranking run on it; the bins are the same
// either way, because a value's rank is a function of the value set alone.
[[nodiscard]] std::vector<std::uint8_t> equalize_intensity_bins(
  std::span<const float> intensities, int bins, WorkerPool * pool = nullptr);

// NID of one sample at camera pose t_cam_world (world -> camera optical).
// nullopt when fewer than params.min_points survive projection + culling.
[[nodiscard]] std::optional<double> nid_cost(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__NID_COST_HPP_
