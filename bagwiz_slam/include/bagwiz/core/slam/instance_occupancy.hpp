// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__INSTANCE_OCCUPANCY_HPP_
#define BAGWIZ__CORE__SLAM__INSTANCE_OCCUPANCY_HPP_

#include "bagwiz/core/slam/euclidean_clustering.hpp"
#include "bagwiz/core/slam/ground_segmentation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

// GLIM-free and Eigen-free dynamic-point (moving-object ghost) classification
// that needs NO per-point sensor origin: a native reimplementation of the
// instance-aware pseudo-occupancy method of ERASOR2 (Lim et al., RSS 2023,
// building on ERASOR, Lim et al., RA-L 2021). Unlike the void-region ray
// casting in dynamic_removal.hpp — which is only geometrically valid when
// every point can be traced back to the LiDAR that fired it — this method
// compares the VERTICAL EXTENT of what each scan observed per 2D grid cell
// against what the accumulated map holds there, so one pose per cloud is
// enough. That makes it the method of choice for concatenated multi-LiDAR
// topics (e.g. an Autoware /sensing/lidar/concatenated/pointcloud in
// base_link), whose per-point emitter cannot be recovered.
//
// Method outline (two-pass, offline):
//   1. Per scan: ground/non-ground split (ground_segmentation.hpp) and a
//      per-cell height profile of the scan's volume of interest.
//   2. A per-cell evidence accumulator in the style of a binary Bayes filter:
//      a cell whose map profile is tall but whose scan profile is flat ground
//      was verifiably vacated by something that moved away, and repeated
//      observations of that raise the cell's dynamic posterior.
//   3. Per scan: non-ground points are grouped into instances
//      (euclidean_clustering.hpp) and each INSTANCE is kept or dropped as a
//      whole from that scan, so partially-carved ghosts and nibbled statics
//      are both avoided; an under-segmentation check splits mixed clusters and
//      a volumetric erosion sweeps stragglers around confident removals.
//
// The papers leave several constants unpublished; every such value is marked
// "implementation default" on its config field. Kept in the plain bagwiz_slam
// library (like cloud_filters.hpp) so it builds and unit-tests in every
// configuration; the caller (CloudMapper) transforms scans to world-frame xyz,
// mirroring dynamic_removal.hpp's convention.
namespace bagwiz::core::slam
{

// Tuning for the instance-aware pseudo-occupancy classification. Defaults
// follow the ERASOR/ERASOR2 papers where published, and are conservative
// implementation choices where not.
struct InstanceOccupancyConfig
{
  // Side [m] of the square x-y analysis cells. Unpublished in either paper;
  // 0.5 m keeps min_cell_points reachable at range while resolving object
  // footprints (implementation default). Must be > 0 (clamped to a tiny
  // epsilon otherwise).
  double cell_size = 0.5;

  // Voxel side [m] the exported map is merged at; the volumetric erosion
  // radius is its body diagonal (sqrt(3) * voxel_size), per ERASOR2.
  double voxel_size = 0.2;

  // Height band [m] of the volume of interest, relative to the per-scan local
  // ground level (sensor origin minus sensor_height). Points outside the band
  // are ignored by the analysis and never removed. From ERASOR (ERASOR2 keeps
  // the band but publishes no values).
  double height_min = -1.0;
  double height_max = 3.0;

  // Sensor height [m] above ground, defining the local ground level of each
  // scan. The papers measure the height band from the ground but never state
  // the sensor-to-ground offset; 1.73 m is the KITTI HDL-64 mount
  // (implementation default).
  double sensor_height = 1.73;

  // Radial bound [m] of the volume of interest around each scan's origin.
  // From ERASOR; beyond it a scan neither accumulates evidence nor has points
  // removed.
  double max_radius = 80.0;

  // Scan-ratio threshold: a cell is evidence of a vacated volume when the
  // scan's height profile is below this fraction of the map's. Both papers
  // use 0.2.
  double ratio_threshold = 0.2;

  // Minimum points on BOTH the scan and the map side of a cell before the
  // ratio test may fire (also guards its division). ERASOR2's N_min.
  int min_cell_points = 5;

  // Minimum map-side height profile [m] for the ratio test: anything the map
  // holds below this is too flat to prove a mover left. ERASOR2's
  // delta-h_min.
  double min_map_height = 0.4;

  // When a cell's scan points are at least this fraction ground, the cell was
  // observed traversable and gains mild evidence even without a height drop.
  // ERASOR2's tau_ground.
  double ground_dominance = 0.95;

  // Log-odds increment of one mild (ground-dominant) observation, and the
  // gain applied to it for one full ratio-test hit. ERASOR2 uses 0.15 and
  // 2.0.
  double base_increment = 0.15;
  double case_gain = 2.0;

  // Cells whose final posterior exceeds this propose their content for
  // instance-level scrutiny. ERASOR2's p_rp.
  double region_threshold = 0.8;

  // An instance is dropped when the average posterior under it exceeds the
  // FAR threshold (beyond near_far_distance) or the stricter NEAR threshold
  // (within it): nearby, densely-observed objects demand near-certain
  // evidence. ERASOR2's p_soft / p_hard; the near/far cutoff itself is
  // unpublished (implementation default 30 m).
  double far_threshold = 0.75;
  double near_threshold = 0.999999;
  double near_far_distance = 30.0;

  // Posterior substituted for an instance point whose cell holds no dynamic
  // evidence (posterior <= 0.5), dragging mixed instances toward keeping.
  // ERASOR2 requires < 0.5 but publishes no value (implementation default).
  double negative_posterior = 0.4;

  // Under-segmentation check: an instance that FAILED the drop test is split
  // when its footprint exceeds usc_area [m^2], more than usc_prior_ratio of
  // its footprint cells hold no evidence at all, and some cells exceed
  // usc_high_posterior — then only the points on those high cells are
  // dropped. Area and ratio from ERASOR2; the high-cell threshold is
  // unpublished (implementation default = vor_seed_posterior).
  double usc_area = 56.0;
  double usc_prior_ratio = 0.25;
  double usc_high_posterior = 0.953;

  // Volumetric erosion: instances whose average posterior exceeds this seed
  // the erosion (ERASOR2's p_v), and every static-side point within
  // sqrt(3) * voxel_size of a seed point from the surrounding
  // [-vor_window, +vor_window] scans is dropped with it (ERASOR2 uses a
  // 3-scan window, i.e. 1).
  double vor_seed_posterior = 0.953;
  int vor_window = 1;

  // Whether the volumetric erosion may eat GROUND-labeled points. ERASOR2's
  // equation exempts ground but its stated motivation (wheels mislabeled as
  // ground) requires eroding it; default true resolves the conflict in favor
  // of the motivation.
  bool vor_erode_ground = true;

  // Symmetric bound on a cell's accumulated log-odds, keeping posteriors
  // finite. Absent from the papers (implementation default; keeps
  // near_threshold = 0.999999 reachable).
  double logodds_clamp = 13.9;

  // The per-scan ground split and the instance grouping, exposed whole.
  GroundSegmentationConfig ground;
  EuclideanClusteringConfig clustering;
};

// Instance-aware pseudo-occupancy classifier over world-frame points.
// Five-phase use, driven once per stashed scan and pass:
//   1. add_scan() every scan (thread-safe; calls may run concurrently),
//   2. finalize_grid() exactly once after all add_scan() calls returned,
//   3. propose() every scan (thread-safe) with the SAME points as add_scan(),
//   4. finalize_proposals() exactly once after all propose() calls returned,
//   5. classify() each scan (const; calls may run concurrently).
// The evidence grid accumulates integer observation counts per cell (a pure
// commutative sum) and every per-scan pass is a pure function of the frozen
// grid, so the classification is byte-identical for any thread count or scan
// order. Memory holds per-cell statistics plus one verdict byte per point
// (never a copy of the clouds); the per-scan cell statistics are released by
// finalize_grid(). Violating the phase contract (wrong order, an unknown or
// repeated scan_id, mismatched point counts, an undersized keep span) throws
// std::logic_error / std::invalid_argument in every build type.
class InstanceOccupancyClassifier
{
public:
  // `scan_count` fixes the number of scans; every phase addresses scans by an
  // id in [0, scan_count).
  InstanceOccupancyClassifier(const InstanceOccupancyConfig & config, std::size_t scan_count);
  ~InstanceOccupancyClassifier();

  InstanceOccupancyClassifier(const InstanceOccupancyClassifier &) = delete;
  InstanceOccupancyClassifier & operator=(const InstanceOccupancyClassifier &) = delete;

  // Phase 1 — accumulate the scan's per-cell height profile (and the map-side
  // aggregate) for the points inside the scan's volume of interest.
  // `sensor_origin` is the scan's world-frame sensor position. Each scan_id
  // must be added exactly once; calls for different scans may run
  // concurrently.
  void add_scan(
    std::size_t scan_id, std::span<const std::array<float, 3>> world_points,
    const std::array<double, 3> & sensor_origin);

  // Phase 2 — fold every scan's profile against the completed map profile
  // into the per-cell posterior and mark the proposal region. Must be called
  // exactly once, after every add_scan() has returned; `num_threads` bounds
  // the workers (values < 1 are treated as 1) and does not affect the result.
  void finalize_grid(int num_threads);

  // Phase 3 — instance-level decisions for one scan: ground split, instance
  // grouping, per-instance keep/drop against the frozen grid, the
  // under-segmentation split, and collection of the scan's erosion seeds.
  // Must be passed the same points as add_scan() for that scan_id; calls for
  // different scans may run concurrently.
  void propose(std::size_t scan_id, std::span<const std::array<float, 3>> world_points);

  // Phase 4 — declare all propose() calls complete (the erosion in classify()
  // reads NEIGHBORING scans' seeds, so classification may only start once
  // every proposal exists). Must be called exactly once.
  void finalize_proposals();

  // Phase 5 — keep[i] = 0 iff world_points[i] is dropped (dynamic instance,
  // under-segmentation split, unclustered point on a proposed cell, or
  // volumetric erosion), else 1. Returns the number of dropped slots written.
  // Must be passed the same points as before; keep.size() must be >=
  // world_points.size(). Const and lock-free, so concurrent calls are safe.
  std::size_t classify(
    std::size_t scan_id, std::span<const std::array<float, 3>> world_points,
    std::span<std::uint8_t> keep) const;

  // Posterior of the cell containing world-frame (x, y), 0.5 for cells that
  // never accumulated evidence. Valid after finalize_grid() (diagnostic).
  [[nodiscard]] double cell_posterior(double x, double y) const;

  // Number of cells above region_threshold. Valid after finalize_grid()
  // (diagnostic).
  [[nodiscard]] std::size_t proposed_cell_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__INSTANCE_OCCUPANCY_HPP_
