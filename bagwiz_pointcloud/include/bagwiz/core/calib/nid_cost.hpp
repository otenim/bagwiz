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
#include "bagwiz/core/calib/depth_cull.hpp"
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

// Project sample.points_world[begin, end) through t_cam_world (world ->
// camera optical) and append the points that land inside the depth window
// and the image — as DepthCullPoint (pixel, depth) plus, in `bins`, the
// point's intensity bin — to the two vectors, in point order. The first half
// of nid_cost, on its own so a caller can split one sample's points into
// ranges and project them concurrently: every point's outcome depends on
// that point alone.
void project_sample_points(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params, std::size_t begin, std::size_t end,
  std::vector<DepthCullPoint> & points, std::vector<std::uint8_t> & bins);

// One chunk of projected points with their intensity bins (parallel spans).
// The points are mutable because the depth cull writes each point's cell.
struct ProjectedChunk
{
  std::span<DepthCullPoint> points;
  std::span<const std::uint8_t> bins;
};

// The histograms one NID evaluation is computed from: the bins x bins joint
// gray/lidar counts, the two marginals and the number of points counted. The
// counts are integers held in double, so histograms built over any split of
// the points add() up to exactly the histograms one pass would have built.
struct NidHistograms
{
  std::vector<double> joint;
  std::vector<double> h_gray;
  std::vector<double> h_lidar;
  std::size_t count = 0;

  // Zero everything for `bins` bins.
  void reset(int bins);
  // Elementwise sum with `other` (same bin count).
  void add(const NidHistograms & other);
};

// Count the points of `chunk` the depth cull keeps — `grid` must have
// observed every point of the sample, including this chunk's — into `into`:
// the gray bin under the point's pixel against the point's intensity bin.
void accumulate_histograms(
  const CalibSample & sample, const NidParams & params, const ProjectedChunk & chunk,
  const DepthCullGrid & grid, NidHistograms & into);

// The NID of complete histograms: nullopt when fewer than params.min_points
// were counted or the joint entropy is zero (a constant image or a constant
// intensity leaves NID undefined).
[[nodiscard]] std::optional<double> nid_from_histograms(
  const NidHistograms & histograms, const NidParams & params);

// The scratch one NID evaluation works in, owned by the caller so a pass that
// evaluates thousands of costs does not allocate the cull grid and the
// histograms afresh every time.
struct NidScratch
{
  DepthCullGrid grid;
  NidHistograms histograms;
};

// The second half of nid_cost over already-projected points, given as chunks
// in any order: observe every chunk into one cull grid, count every chunk's
// kept points into one set of histograms, and take the NID of those. The
// depth cull's per-cell nearest depth is a min-reduction and the histograms
// count integers, so every split of the projected set into chunks — and
// every split of this pass itself over grids merged and histograms added
// afterwards — yields the same cost. nullopt as nid_cost.
[[nodiscard]] std::optional<double> nid_of_projected(
  const CalibSample & sample, const NidParams & params, std::span<const ProjectedChunk> chunks,
  NidScratch & scratch);

// NID of one sample at camera pose t_cam_world (world -> camera optical):
// project_sample_points over every point, then nid_of_projected. nullopt
// when fewer than params.min_points survive projection + culling.
[[nodiscard]] std::optional<double> nid_cost(
  const CalibSample & sample, const CameraModel & cam, const Mat4 & t_cam_world,
  const NidParams & params);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__NID_COST_HPP_
