// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nid_cost.hpp"

#include "bagwiz/core/base/parallel_sort.hpp"
#include "bagwiz/core/calib/depth_cull.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
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

std::vector<std::uint8_t> equalize_intensity_bins(
  std::span<const float> intensities, int bins, WorkerPool * pool)
{
  // Rank-based equalization: sort a copy, then each value's bin is its rank
  // quantile. Ties share the bin of their first occurrence so identical
  // intensities always land in the same bin.
  std::vector<float> sorted(intensities.begin(), intensities.end());
  parallel_sort(sorted, pool, std::less<float>{});
  std::vector<std::uint8_t> out(intensities.size());
  const double n = static_cast<double>(sorted.size());
  const auto rank_range = [&](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      const auto rank =
        std::lower_bound(sorted.begin(), sorted.end(), intensities[i]) - sorted.begin();
      const int bin = static_cast<int>(static_cast<double>(rank) / n * bins);
      out[i] = static_cast<std::uint8_t>(std::clamp(bin, 0, bins - 1));
    }
  };
  constexpr std::size_t kRankChunk = 1U << 16U;
  const std::size_t chunks = (intensities.size() + kRankChunk - 1) / kRankChunk;
  if (pool == nullptr || chunks <= 1) {
    rank_range(0, intensities.size());
    return out;
  }
  pool->parallel_for(chunks, [&](std::size_t c) {
    rank_range(c * kRankChunk, std::min(intensities.size(), (c + 1) * kRankChunk));
  });
  return out;
}

void project_sample_points(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params, std::size_t begin, std::size_t end,
  std::vector<DepthCullPoint> & points, std::vector<std::uint8_t> & bins)
{
  for (std::size_t i = begin; i < end; ++i) {
    const auto & p = sample.points_world[i];
    const std::array<double, 3> pw{p[0], p[1], p[2]};
    // Depth first — the window rejects most points, and the other two rows
    // of the transform are only needed once it passes. Narrow first, then
    // validate in POSITIVE form on the narrowed values. Both halves matter: a
    // NaN passes every negative-form test (`NaN < min` and `NaN > max` are
    // both false), and a double column just under `width` can round UP to
    // exactly `width` when narrowed, which would index one past the last
    // column of the image row and of depth_cull's cell grid.
    const double z = transform_point_z(t_cam_world, pw);
    const auto fz = static_cast<float>(z);
    if (!(fz >= params.min_depth && fz <= params.max_depth)) {
      continue;
    }
    const double x = transform_point_x(t_cam_world, pw);
    const double y = transform_point_y(t_cam_world, pw);
    const auto nd = image::distort_normalized(x / z, y / z, cam.model, cam.d);
    const auto fu = static_cast<float>(cam.k[0] * nd.x + cam.k[2]);
    const auto fv = static_cast<float>(cam.k[4] * nd.y + cam.k[5]);
    if (
      !(fu >= 0.0F && fu < static_cast<float>(cam.width)) ||
      !(fv >= 0.0F && fv < static_cast<float>(cam.height))) {
      continue;
    }
    points.push_back({fu, fv, fz, 0});
    bins.push_back(sample.intensity_bins[i]);
  }
}

std::optional<double> nid_of_projected(
  const CalibSample & sample, const NidParams & params, std::span<const ProjectedChunk> chunks,
  NidScratch & scratch)
{
  std::size_t projected = 0;
  for (const auto & chunk : chunks) {
    projected += chunk.points.size();
  }
  if (projected < params.min_points) {
    return std::nullopt;
  }

  scratch.grid.reset(sample.image.width, sample.image.height, params.cull_cell_px);
  for (const auto & chunk : chunks) {
    scratch.grid.observe(chunk.points);
  }

  const int bins = params.bins;
  scratch.joint.assign(static_cast<std::size_t>(bins) * bins, 0.0);
  scratch.h_gray.assign(static_cast<std::size_t>(bins), 0.0);
  scratch.h_lidar.assign(static_cast<std::size_t>(bins), 0.0);
  std::size_t count = 0;
  for (const auto & chunk : chunks) {
    for (std::size_t i = 0; i < chunk.points.size(); ++i) {
      const auto & point = chunk.points[i];
      if (!scratch.grid.keeps(point, params.cull_margin_m)) {
        continue;
      }
      const auto px = static_cast<std::uint32_t>(point.u);
      const auto py = static_cast<std::uint32_t>(point.v);
      const int gb =
        sample.image.gray[static_cast<std::size_t>(py) * sample.image.width + px] * bins / 256;
      const int lb = chunk.bins[i];
      scratch.joint[static_cast<std::size_t>(gb) * bins + lb] += 1.0;
      scratch.h_gray[gb] += 1.0;
      scratch.h_lidar[lb] += 1.0;
      ++count;
    }
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
  const double h_g = entropy(scratch.h_gray);
  const double h_l = entropy(scratch.h_lidar);
  const double h_joint = entropy(scratch.joint);
  if (h_joint <= 0.0) {
    return std::nullopt;  // constant image or constant intensity: NID undefined
  }
  const double mutual = h_g + h_l - h_joint;
  return (h_joint - mutual) / h_joint;
}

std::optional<double> nid_cost(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params)
{
  std::vector<DepthCullPoint> points;
  std::vector<std::uint8_t> bins;
  points.reserve(sample.points_world.size());
  bins.reserve(sample.points_world.size());
  project_sample_points(
    sample, cam, t_cam_world, params, 0, sample.points_world.size(), points, bins);
  NidScratch scratch;
  const std::array<ProjectedChunk, 1> chunks{ProjectedChunk{points, bins}};
  return nid_of_projected(sample, params, chunks, scratch);
}

}  // namespace bagwiz::core::calib
