// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/instance_occupancy.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
// Same guard as VoxelGrid: never divide by a non-positive cell size.
constexpr double kMinCellSize = 1e-3;

// The per-cell height statistics are accumulated as millimeter integers so
// the map-side aggregation is an exact commutative sum/min/max: the result is
// byte-identical for any thread count or scan order, which double sums (whose
// rounding depends on addition order) could not guarantee.
constexpr double kZQuant = 1000.0;

// Shard count for the map-side cell accumulation, as in dynamic_removal.cpp:
// concurrent add_scan() calls only contend when two scans touch cells hashing
// to the same shard.
constexpr std::size_t kShardCount = 64;

// Teschner et al. spatial-hash constants, as in cloud_filters.cpp.
constexpr std::size_t kHashX = 73856093U;
constexpr std::size_t kHashY = 19349663U;

// Verdict bits stored per point by propose(), read by classify().
constexpr std::uint8_t kDropBit = 1U;      // dynamic instance / split / stray
constexpr std::uint8_t kErodibleBit = 2U;  // may be swept by the volumetric erosion

struct CellKey
{
  std::int32_t x;
  std::int32_t y;
  bool operator==(const CellKey & other) const { return x == other.x && y == other.y; }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    const auto mix = [](std::int32_t index, std::size_t multiplier) {
      return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
    };
    return mix(key.x, kHashX) ^ mix(key.y, kHashY);
  }
};

// Height profile of one cell: of one scan's points (query side) or of every
// scan's points together (map side). All heights are millimeter integers.
struct CellProfile
{
  std::int64_t ground_z_sum_mm = 0;
  std::int32_t min_z_mm = std::numeric_limits<std::int32_t>::max();
  std::int32_t max_z_mm = std::numeric_limits<std::int32_t>::min();
  std::uint32_t count = 0;
  std::uint32_t ground_count = 0;

  void add(std::int32_t z_mm, bool is_ground)
  {
    min_z_mm = std::min(min_z_mm, z_mm);
    max_z_mm = std::max(max_z_mm, z_mm);
    ++count;
    if (is_ground) {
      ++ground_count;
      ground_z_sum_mm += z_mm;
    }
  }

  void merge(const CellProfile & other)
  {
    ground_z_sum_mm += other.ground_z_sum_mm;
    min_z_mm = std::min(min_z_mm, other.min_z_mm);
    max_z_mm = std::max(max_z_mm, other.max_z_mm);
    count += other.count;
    ground_count += other.ground_count;
  }
};

// One scan's profile of one cell, stashed between add_scan() and
// finalize_grid().
struct ScanCellProfile
{
  CellKey cell;
  CellProfile profile;
};

struct Shard
{
  std::mutex mutex;
  std::unordered_map<CellKey, CellProfile, CellKeyHash> cells;
};

// Read-only per-cell state assembled by finalize_grid().
struct GridCell
{
  CellProfile map_profile;
  std::uint32_t counter_index = 0;  // into the vacated/mild counter arrays
  double log_odds = 0.0;
  bool updated = false;
  bool proposed = false;
};

double sigmoid(double log_odds)
{
  return 1.0 / (1.0 + std::exp(-log_odds));
}

bool finite_point(const std::array<float, 3> & point)
{
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]);
}

// Run fn(begin, end) over [0, count) split across at most num_threads workers,
// joining them all before returning. Each worker owns a disjoint range, so fn
// needs no locking for per-item state; the split does not affect any result.
template <typename RangeFn>
void run_chunked(std::size_t count, int num_threads, const RangeFn & fn)
{
  const std::size_t workers =
    std::min(count, static_cast<std::size_t>(num_threads > 0 ? num_threads : 1));
  if (workers <= 1) {
    fn(0, count);
    return;
  }
  std::vector<std::thread> pool;
  pool.reserve(workers);
  const std::size_t chunk = (count + workers - 1) / workers;
  for (std::size_t w = 0; w < workers; ++w) {
    const std::size_t begin = w * chunk;
    const std::size_t end = std::min(begin + chunk, count);
    pool.emplace_back([&fn, begin, end]() { fn(begin, end); });
  }
  for (auto & worker : pool) {
    worker.join();
  }
}

}  // namespace

struct InstanceOccupancyClassifier::Impl
{
  const InstanceOccupancyConfig config;
  const double cell_size;
  const double inv_cell_size;
  const double erosion_radius;  // sqrt(3) * voxel_size, ERASOR2's r_v

  // Per-scan state. `stats` lives only between add_scan() and finalize_grid();
  // `verdicts` (one byte per point) and `seeds` (the confidently-removed
  // instance points) are written by propose() and read by classify().
  struct ScanData
  {
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    std::size_t point_count = 0;
    std::vector<ScanCellProfile> stats;
    std::vector<std::uint8_t> verdicts;
    std::vector<std::array<float, 3>> seeds;
    bool added = false;
    bool proposed = false;
  };
  std::vector<ScanData> scans;

  std::array<Shard, kShardCount> shards;

  std::unordered_map<CellKey, GridCell, CellKeyHash> grid;
  std::vector<std::atomic<std::uint32_t>> vacated_counts;
  std::vector<std::atomic<std::uint32_t>> mild_counts;
  std::size_t proposed_cell_total = 0;
  bool grid_ready = false;
  bool proposals_ready = false;

  Impl(const InstanceOccupancyConfig & config_in, std::size_t scan_count)
  : config(config_in),
    cell_size(config_in.cell_size > kMinCellSize ? config_in.cell_size : kMinCellSize),
    inv_cell_size(1.0 / cell_size),
    erosion_radius(std::sqrt(3.0) * std::max(config_in.voxel_size, kMinCellSize)),
    scans(scan_count)
  {
  }

  [[nodiscard]] CellKey cell_of(double x, double y) const
  {
    return {
      static_cast<std::int32_t>(std::floor(x * inv_cell_size)),
      static_cast<std::int32_t>(std::floor(y * inv_cell_size))};
  }

  [[nodiscard]] const GridCell * find_cell(const CellKey & key) const
  {
    const auto found = grid.find(key);
    return found == grid.end() ? nullptr : &found->second;
  }

  // Volume-of-interest test: finite, inside the radial bound around the
  // scan's origin, and inside the height band over the scan's local ground
  // level (origin z minus sensor_height).
  [[nodiscard]] bool in_voi(
    const std::array<float, 3> & point, const std::array<double, 3> & origin) const
  {
    if (!finite_point(point)) {
      return false;
    }
    const double dx = static_cast<double>(point[0]) - origin[0];
    const double dy = static_cast<double>(point[1]) - origin[1];
    if (dx * dx + dy * dy > config.max_radius * config.max_radius) {
      return false;
    }
    const double height = static_cast<double>(point[2]) - (origin[2] - config.sensor_height);
    return height > config.height_min && height < config.height_max;
  }

  // Log-odds substituted for a point whose cell carries no dynamic evidence
  // (ERASOR2's p_neg), clamped away from 0/1 so the logit stays finite.
  [[nodiscard]] double negative_log_odds() const
  {
    const double p = std::clamp(config.negative_posterior, 1e-6, 0.5);
    return std::log(p / (1.0 - p));
  }
};

InstanceOccupancyClassifier::InstanceOccupancyClassifier(
  const InstanceOccupancyConfig & config, std::size_t scan_count)
: impl_(std::make_unique<Impl>(config, scan_count))
{
}

InstanceOccupancyClassifier::~InstanceOccupancyClassifier() = default;

void InstanceOccupancyClassifier::add_scan(
  std::size_t scan_id, std::span<const std::array<float, 3>> world_points,
  const std::array<double, 3> & sensor_origin)
{
  assert(scan_id < impl_->scans.size());
  assert(!impl_->grid_ready);
  Impl::ScanData & scan = impl_->scans[scan_id];
  assert(!scan.added);
  scan.added = true;
  scan.origin = sensor_origin;
  scan.point_count = world_points.size();
  if (world_points.empty()) {
    return;
  }

  // Ground labels over the full scan (the fit benefits from every point);
  // only the volume of interest is profiled below. propose() recomputes the
  // same labels — both passes are pure functions of the scan, so they agree.
  std::vector<std::uint8_t> ground(world_points.size(), 0U);
  segment_ground(world_points, impl_->config.ground, ground);

  std::unordered_map<CellKey, CellProfile, CellKeyHash> local;
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    const auto & point = world_points[i];
    if (!impl_->in_voi(point, sensor_origin)) {
      continue;
    }
    const auto z_mm =
      static_cast<std::int32_t>(std::llround(static_cast<double>(point[2]) * kZQuant));
    local[impl_->cell_of(point[0], point[1])].add(z_mm, ground[i] != 0U);
  }

  scan.stats.reserve(local.size());
  for (const auto & entry : local) {
    scan.stats.push_back({entry.first, entry.second});

    Shard & shard = impl_->shards[CellKeyHash{}(entry.first) % kShardCount];
    const std::lock_guard<std::mutex> lock(shard.mutex);
    shard.cells[entry.first].merge(entry.second);
  }
}

void InstanceOccupancyClassifier::finalize_grid(int num_threads)
{
  assert(!impl_->grid_ready);
  const InstanceOccupancyConfig & config = impl_->config;

  // Assemble the read-only cell index from the shards.
  std::size_t cell_count = 0;
  for (const Shard & shard : impl_->shards) {
    cell_count += shard.cells.size();
  }
  impl_->grid.reserve(cell_count);
  std::uint32_t next_index = 0;
  for (Shard & shard : impl_->shards) {
    for (const auto & entry : shard.cells) {
      GridCell cell;
      cell.map_profile = entry.second;
      cell.counter_index = next_index++;
      impl_->grid.emplace(entry.first, cell);
    }
    shard.cells.clear();
  }
  impl_->vacated_counts = std::vector<std::atomic<std::uint32_t>>(cell_count);
  impl_->mild_counts = std::vector<std::atomic<std::uint32_t>>(cell_count);

  // Fold every scan's profile against the completed map profile. The order is
  // irrelevant: each observation only increments an integer counter.
  run_chunked(
    impl_->scans.size(), num_threads, [this, &config](std::size_t begin, std::size_t end) {
      for (std::size_t s = begin; s < end; ++s) {
        for (const ScanCellProfile & item : impl_->scans[s].stats) {
          const GridCell & cell = impl_->grid.at(item.cell);
          const CellProfile & query = item.profile;
          const CellProfile & map = cell.map_profile;

          // The lower height reference: the ground level the profiles are
          // measured from. Ground means where available (query, map, or
          // both), else the cell's lowest point.
          double lower_mm = 0.0;
          if (query.ground_count > 0 && map.ground_count > 0) {
            lower_mm = std::min(
              static_cast<double>(query.ground_z_sum_mm) / query.ground_count,
              static_cast<double>(map.ground_z_sum_mm) / map.ground_count);
          } else if (query.ground_count > 0) {
            lower_mm = static_cast<double>(query.ground_z_sum_mm) / query.ground_count;
          } else if (map.ground_count > 0) {
            lower_mm = static_cast<double>(map.ground_z_sum_mm) / map.ground_count;
          } else {
            lower_mm = static_cast<double>(std::min(query.min_z_mm, map.min_z_mm));
          }
          const double query_height = (query.max_z_mm - lower_mm) / kZQuant;
          const double map_height = (map.max_z_mm - lower_mm) / kZQuant;

          // Vacated-volume test (ERASOR2's "case A", written multiplicatively
          // so an empty-profile division can never occur): the map holds a
          // tall profile here but this scan saw next to nothing.
          const auto min_points = static_cast<std::uint32_t>(std::max(config.min_cell_points, 0));
          const bool vacated = query.count > min_points && map.count > min_points &&
                               map_height > config.min_map_height &&
                               query_height < config.ratio_threshold * map_height;
          if (vacated) {
            impl_->vacated_counts[cell.counter_index].fetch_add(1U, std::memory_order_relaxed);
          } else if (
            static_cast<double>(query.ground_count) >
            config.ground_dominance * static_cast<double>(query.count)) {
            // Ground-dominant observation: the cell was seen traversable.
            impl_->mild_counts[cell.counter_index].fetch_add(1U, std::memory_order_relaxed);
          }
        }
      }
    });

  // Fold the counters into each cell's clamped log-odds and posterior flags.
  const double mild_increment = config.base_increment;
  const double vacated_increment = config.case_gain * config.base_increment;
  const double clamp = std::abs(config.logodds_clamp);
  for (auto & entry : impl_->grid) {
    GridCell & cell = entry.second;
    const std::uint32_t vacated =
      impl_->vacated_counts[cell.counter_index].load(std::memory_order_relaxed);
    const std::uint32_t mild =
      impl_->mild_counts[cell.counter_index].load(std::memory_order_relaxed);
    cell.updated = vacated + mild > 0;
    cell.log_odds = std::clamp(
      static_cast<double>(vacated) * vacated_increment + static_cast<double>(mild) * mild_increment,
      -clamp, clamp);
    cell.proposed = sigmoid(cell.log_odds) > config.region_threshold;
    impl_->proposed_cell_total += cell.proposed ? 1U : 0U;
  }
  impl_->vacated_counts.clear();
  impl_->mild_counts.clear();

  // The per-scan profiles have served their purpose; release them.
  for (Impl::ScanData & scan : impl_->scans) {
    scan.stats = std::vector<ScanCellProfile>();
  }
  impl_->grid_ready = true;
}

void InstanceOccupancyClassifier::propose(
  std::size_t scan_id, std::span<const std::array<float, 3>> world_points)
{
  assert(impl_->grid_ready);
  assert(scan_id < impl_->scans.size());
  Impl::ScanData & scan = impl_->scans[scan_id];
  assert(scan.added && !scan.proposed);
  assert(scan.point_count == world_points.size());
  scan.proposed = true;
  if (world_points.empty()) {
    return;
  }
  const InstanceOccupancyConfig & config = impl_->config;

  std::vector<std::uint8_t> ground(world_points.size(), 0U);
  segment_ground(world_points, config.ground, ground);

  // Instances are formed from the scan's non-ground points inside the volume
  // of interest; everything else keeps cluster id 0.
  std::vector<std::uint8_t> cluster_mask(world_points.size(), 0U);
  std::vector<std::uint8_t> voi(world_points.size(), 0U);
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    voi[i] = impl_->in_voi(world_points[i], scan.origin) ? 1U : 0U;
    cluster_mask[i] = (voi[i] != 0U && ground[i] == 0U) ? 1U : 0U;
  }
  std::vector<std::uint32_t> cluster_ids(world_points.size(), 0U);
  const std::size_t cluster_count =
    cluster_points(world_points, cluster_mask, config.clustering, cluster_ids);

  std::vector<std::vector<std::uint32_t>> members(cluster_count + 1);
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    if (cluster_ids[i] != 0U) {
      members[cluster_ids[i]].push_back(static_cast<std::uint32_t>(i));
    }
  }

  scan.verdicts.assign(world_points.size(), 0U);
  const double negative = impl_->negative_log_odds();

  for (std::size_t k = 1; k <= cluster_count; ++k) {
    const std::vector<std::uint32_t> & instance = members[k];

    // Only instances touching the proposal region are scrutinized at all.
    bool in_region = false;
    for (const std::uint32_t index : instance) {
      const auto & p = world_points[index];
      const GridCell * cell = impl_->find_cell(impl_->cell_of(p[0], p[1]));
      if (cell != nullptr && cell->proposed) {
        in_region = true;
        break;
      }
    }
    if (!in_region) {
      continue;
    }

    // Average the log-odds under the instance; cells without dynamic evidence
    // contribute the negative constant, dragging mixed instances toward
    // keeping (ERASOR2 Eq. 14).
    double score_sum = 0.0;
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const std::uint32_t index : instance) {
      const auto & p = world_points[index];
      const GridCell * cell = impl_->find_cell(impl_->cell_of(p[0], p[1]));
      const double log_odds = cell != nullptr ? cell->log_odds : 0.0;
      score_sum += log_odds > 0.0 ? log_odds : negative;
      centroid_x += static_cast<double>(p[0]);
      centroid_y += static_cast<double>(p[1]);
    }
    const double score = sigmoid(score_sum / static_cast<double>(instance.size()));
    centroid_x /= static_cast<double>(instance.size());
    centroid_y /= static_cast<double>(instance.size());

    // Near instances demand near-certain evidence: they were densely observed,
    // so weaker evidence there means a mixed or static cluster.
    const double dx = centroid_x - scan.origin[0];
    const double dy = centroid_y - scan.origin[1];
    const bool far = dx * dx + dy * dy > config.near_far_distance * config.near_far_distance;
    const double threshold = far ? config.far_threshold : config.near_threshold;

    if (score > threshold) {
      for (const std::uint32_t index : instance) {
        scan.verdicts[index] |= kDropBit;
      }
      // Confident removals seed the volumetric erosion in classify().
      if (score > config.vor_seed_posterior) {
        for (const std::uint32_t index : instance) {
          scan.seeds.push_back(world_points[index]);
        }
      }
      continue;
    }

    // Under-segmentation check: a huge cluster that failed the drop test may
    // be a static structure welded to a mover (say, a wall plus a car). When
    // most of its footprint carries no evidence at all but SOME cells are
    // near-certain, only the points on those cells are carved out.
    std::unordered_set<CellKey, CellKeyHash> footprint;
    for (const std::uint32_t index : instance) {
      const auto & p = world_points[index];
      footprint.insert(impl_->cell_of(p[0], p[1]));
    }
    const double area = static_cast<double>(footprint.size()) * impl_->cell_size * impl_->cell_size;
    if (area <= config.usc_area) {
      continue;
    }
    std::size_t prior_cells = 0;
    std::unordered_set<CellKey, CellKeyHash> high_cells;
    for (const CellKey & key : footprint) {
      const GridCell * cell = impl_->find_cell(key);
      if (cell == nullptr || !cell->updated) {
        ++prior_cells;
      } else if (sigmoid(cell->log_odds) > config.usc_high_posterior) {
        high_cells.insert(key);
      }
    }
    const double prior_ratio =
      static_cast<double>(prior_cells) / static_cast<double>(footprint.size());
    if (prior_ratio > config.usc_prior_ratio && !high_cells.empty()) {
      for (const std::uint32_t index : instance) {
        const auto & p = world_points[index];
        if (high_cells.count(impl_->cell_of(p[0], p[1])) != 0U) {
          scan.verdicts[index] |= kDropBit;
        }
      }
    }
  }

  // Unclustered non-ground points on proposed cells are stray returns of
  // movers too small or ragged to cluster; drop them outright.
  std::vector<std::uint32_t> strays;
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    if (cluster_mask[i] == 0U || cluster_ids[i] != 0U) {
      continue;
    }
    const auto & p = world_points[i];
    const GridCell * cell = impl_->find_cell(impl_->cell_of(p[0], p[1]));
    if (cell != nullptr && cell->proposed) {
      scan.verdicts[i] |= kDropBit;
      strays.push_back(static_cast<std::uint32_t>(i));
    }
  }

  // Strays adjacent to a confident removal join the erosion seeds (ERASOR2's
  // U_t^Q): they are the same object's returns, so they should erode their
  // surroundings the same way.
  if (!strays.empty() && !scan.seeds.empty()) {
    const double radius_sq = impl_->erosion_radius * impl_->erosion_radius;
    std::vector<std::array<float, 3>> extra;
    for (const std::uint32_t index : strays) {
      const auto & p = world_points[index];
      for (const auto & seed : scan.seeds) {
        const double dx = static_cast<double>(p[0]) - static_cast<double>(seed[0]);
        const double dy = static_cast<double>(p[1]) - static_cast<double>(seed[1]);
        const double dz = static_cast<double>(p[2]) - static_cast<double>(seed[2]);
        if (dx * dx + dy * dy + dz * dz <= radius_sq) {
          extra.push_back(p);
          break;
        }
      }
    }
    scan.seeds.insert(scan.seeds.end(), extra.begin(), extra.end());
  }

  // Erodibility of the surviving points: non-ground always, ground only when
  // configured (ERASOR2's equation exempts ground; its wheels-mislabeled-as-
  // ground motivation needs it — the flag resolves the conflict).
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    if (voi[i] == 0U || (scan.verdicts[i] & kDropBit) != 0U) {
      continue;
    }
    if (ground[i] == 0U || config.vor_erode_ground) {
      scan.verdicts[i] |= kErodibleBit;
    }
  }
}

void InstanceOccupancyClassifier::finalize_proposals()
{
  assert(impl_->grid_ready);
#ifndef NDEBUG
  for (const Impl::ScanData & scan : impl_->scans) {
    assert(scan.proposed);
  }
#endif
  impl_->proposals_ready = true;
}

std::size_t InstanceOccupancyClassifier::classify(
  std::size_t scan_id, std::span<const std::array<float, 3>> world_points,
  std::span<std::uint8_t> keep) const
{
  assert(impl_->proposals_ready);
  assert(scan_id < impl_->scans.size());
  assert(keep.size() >= world_points.size());
  const Impl::ScanData & scan = impl_->scans[scan_id];
  assert(scan.point_count == world_points.size());

  std::size_t dropped = 0;
  for (std::size_t i = 0; i < world_points.size(); ++i) {
    const bool drop = (scan.verdicts[i] & kDropBit) != 0U;
    keep[i] = drop ? 0U : 1U;
    dropped += drop ? 1U : 0U;
  }

  // Volumetric erosion: pool the confident removals of this scan and its
  // temporal neighbors (no transform needed — everything is world-frame) and
  // sweep the surviving erodible points within the voxel diagonal of any of
  // them. Movers shed stragglers (mis-segmented wheels, clipped returns) right
  // next to their confidently-removed volume; this is what collects them.
  const int window = std::max(impl_->config.vor_window, 0);
  const std::size_t begin =
    scan_id >= static_cast<std::size_t>(window) ? scan_id - static_cast<std::size_t>(window) : 0;
  const std::size_t end =
    std::min(impl_->scans.size(), scan_id + static_cast<std::size_t>(window) + 1);

  const double radius = impl_->erosion_radius;
  const double radius_sq = radius * radius;
  const double inv_radius = 1.0 / radius;

  // 3D hash grid over the pooled seeds at the erosion radius: a point within
  // the radius of a seed is at most one grid index away on each axis.
  struct SeedKey
  {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    bool operator==(const SeedKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct SeedKeyHash
  {
    std::size_t operator()(const SeedKey & key) const
    {
      const auto mix = [](std::int32_t index, std::size_t multiplier) {
        return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
      };
      return mix(key.x, kHashX) ^ mix(key.y, kHashY) ^ mix(key.z, 83492791U);
    }
  };
  const auto seed_key_of = [inv_radius](const std::array<float, 3> & point) {
    return SeedKey{
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[0]) * inv_radius)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[1]) * inv_radius)),
      static_cast<std::int32_t>(std::floor(static_cast<double>(point[2]) * inv_radius))};
  };
  std::unordered_map<SeedKey, std::vector<std::array<float, 3>>, SeedKeyHash> pool;
  for (std::size_t s = begin; s < end; ++s) {
    for (const auto & seed : impl_->scans[s].seeds) {
      pool[seed_key_of(seed)].push_back(seed);
    }
  }
  if (pool.empty()) {
    return dropped;
  }

  for (std::size_t i = 0; i < world_points.size(); ++i) {
    if ((scan.verdicts[i] & kErodibleBit) == 0U || keep[i] == 0U) {
      continue;
    }
    const auto & point = world_points[i];
    const SeedKey base = seed_key_of(point);
    bool eroded = false;
    for (std::int32_t ox = -1; ox <= 1 && !eroded; ++ox) {
      for (std::int32_t oy = -1; oy <= 1 && !eroded; ++oy) {
        for (std::int32_t oz = -1; oz <= 1 && !eroded; ++oz) {
          const auto found = pool.find({base.x + ox, base.y + oy, base.z + oz});
          if (found == pool.end()) {
            continue;
          }
          for (const auto & seed : found->second) {
            const double dx = static_cast<double>(point[0]) - static_cast<double>(seed[0]);
            const double dy = static_cast<double>(point[1]) - static_cast<double>(seed[1]);
            const double dz = static_cast<double>(point[2]) - static_cast<double>(seed[2]);
            if (dx * dx + dy * dy + dz * dz <= radius_sq) {
              eroded = true;
              break;
            }
          }
        }
      }
    }
    if (eroded) {
      keep[i] = 0U;
      ++dropped;
    }
  }
  return dropped;
}

double InstanceOccupancyClassifier::cell_posterior(double x, double y) const
{
  assert(impl_->grid_ready);
  const GridCell * cell = impl_->find_cell(impl_->cell_of(x, y));
  return cell == nullptr ? 0.5 : sigmoid(cell->log_odds);
}

std::size_t InstanceOccupancyClassifier::proposed_cell_count() const
{
  assert(impl_->grid_ready);
  return impl_->proposed_cell_total;
}

}  // namespace bagwiz::core::slam
