// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nid_cost.hpp"

#include "bagwiz/core/calib/depth_cull.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace bagwiz::core::calib
{

GrayImage gray_from_bgr24(std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  GrayImage out;
  out.width = width;
  out.height = height;
  out.gray.resize(static_cast<std::size_t>(width) * height);
  for (std::size_t i = 0; i < out.gray.size(); ++i) {
    const auto b = static_cast<std::uint32_t>(bgr[i * 3 + 0]);
    const auto g = static_cast<std::uint32_t>(bgr[i * 3 + 1]);
    const auto r = static_cast<std::uint32_t>(bgr[i * 3 + 2]);
    // Integer BT.601 luma; +128 rounds.
    out.gray[i] = static_cast<std::uint8_t>((29 * b + 150 * g + 77 * r + 128) >> 8);
  }
  return out;
}

std::vector<std::uint8_t> equalize_intensity_bins(std::span<const float> intensities, int bins)
{
  // Rank-based equalization: sort a copy, then each value's bin is its rank
  // quantile. Ties share the bin of their first occurrence so identical
  // intensities always land in the same bin.
  std::vector<float> sorted(intensities.begin(), intensities.end());
  std::sort(sorted.begin(), sorted.end());
  std::vector<std::uint8_t> out(intensities.size());
  const double n = static_cast<double>(sorted.size());
  for (std::size_t i = 0; i < intensities.size(); ++i) {
    const auto rank =
      std::lower_bound(sorted.begin(), sorted.end(), intensities[i]) - sorted.begin();
    const int bin = static_cast<int>(static_cast<double>(rank) / n * bins);
    out[i] = static_cast<std::uint8_t>(std::clamp(bin, 0, bins - 1));
  }
  return out;
}

std::optional<double> nid_cost(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params)
{
  std::vector<float> us;
  std::vector<float> vs;
  std::vector<float> depths;
  std::vector<std::uint8_t> lidar_bins;
  us.reserve(sample.points_world.size());
  vs.reserve(sample.points_world.size());
  depths.reserve(sample.points_world.size());
  lidar_bins.reserve(sample.points_world.size());

  for (std::size_t i = 0; i < sample.points_world.size(); ++i) {
    const auto & p = sample.points_world[i];
    const auto pc = transform_point(t_cam_world, {p[0], p[1], p[2]});
    // Narrow first, then validate in POSITIVE form on the narrowed values.
    // Both halves matter: a NaN passes every negative-form test (`NaN < min`
    // and `NaN > max` are both false), and a double column just under `width`
    // can round UP to exactly `width` when narrowed, which would index one
    // past the last column of the image row and of depth_cull's cell grid.
    const auto fz = static_cast<float>(pc[2]);
    if (!(fz >= params.min_depth && fz <= params.max_depth)) {
      continue;
    }
    const auto nd = image::distort_normalized(pc[0] / pc[2], pc[1] / pc[2], cam.model, cam.d);
    const auto fu = static_cast<float>(cam.k[0] * nd.x + cam.k[2]);
    const auto fv = static_cast<float>(cam.k[4] * nd.y + cam.k[5]);
    if (
      !(fu >= 0.0F && fu < static_cast<float>(cam.width)) ||
      !(fv >= 0.0F && fv < static_cast<float>(cam.height))) {
      continue;
    }
    us.push_back(fu);
    vs.push_back(fv);
    depths.push_back(fz);
    lidar_bins.push_back(sample.intensity_bins[i]);
  }
  if (us.size() < params.min_points) {
    return std::nullopt;
  }

  const auto keep = depth_cull_keep(
    us, vs, depths, sample.image.width, sample.image.height, params.cull_cell_px,
    params.cull_margin_m);

  const int bins = params.bins;
  std::vector<double> joint(static_cast<std::size_t>(bins) * bins, 0.0);
  std::vector<double> h_gray(bins, 0.0);
  std::vector<double> h_lidar(bins, 0.0);
  std::size_t count = 0;
  for (std::size_t i = 0; i < us.size(); ++i) {
    if (keep[i] == 0) {
      continue;
    }
    const auto px = static_cast<std::uint32_t>(us[i]);
    const auto py = static_cast<std::uint32_t>(vs[i]);
    const int gb =
      sample.image.gray[static_cast<std::size_t>(py) * sample.image.width + px] * bins / 256;
    const int lb = lidar_bins[i];
    joint[static_cast<std::size_t>(gb) * bins + lb] += 1.0;
    h_gray[gb] += 1.0;
    h_lidar[lb] += 1.0;
    ++count;
  }
  if (count < params.min_points) {
    return std::nullopt;
  }

  const auto entropy = [count](std::span<const double> h) {
    double e = 0.0;
    for (const double c : h) {
      if (c > 0.0) {
        const double p = c / static_cast<double>(count);
        e -= p * std::log(p);
      }
    }
    return e;
  };
  const double h_g = entropy(h_gray);
  const double h_l = entropy(h_lidar);
  const double h_joint = entropy(joint);
  if (h_joint <= 0.0) {
    return std::nullopt;  // constant image or constant intensity: NID undefined
  }
  const double mutual = h_g + h_l - h_joint;
  return (h_joint - mutual) / h_joint;
}

}  // namespace bagwiz::core::calib
