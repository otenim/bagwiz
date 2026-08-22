// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/duration_parse.hpp"
#include "bagwiz/core/calib/observability.hpp"
#include "bagwiz/core/pointcloud/deskew.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr std::array<const char *, 6> kAxisNames{"x", "y", "z", "roll", "pitch", "yaw"};
constexpr double kRadToDeg = 180.0 / M_PI;

// Index of a rotation axis in the fixed x,y,z,roll,pitch,yaw order (indices
// 3..5); the translation axes 0..2 are left in meters everywhere.
bool is_rotation_axis(std::size_t axis)
{
  return axis >= 3;
}

const char * axis_observability_name(core::calib::AxisObservability observability)
{
  switch (observability) {
    case core::calib::AxisObservability::kFixed:
      return "fixed";
    case core::calib::AxisObservability::kStrong:
      return "strong";
    case core::calib::AxisObservability::kWeak:
      return "weak";
    case core::calib::AxisObservability::kDegenerate:
      return "degenerate";
  }
  return "unknown";  // unreachable; keeps -Wreturn-type / cppcheck happy
}

// Read one point field, stored at `at`, as a float regardless of its declared
// datatype. Little-endian point data only — accumulate_cloud_into_map rejects
// big-endian clouds before reading anything.
using FieldReadFn = float (*)(const std::byte * at);

template <typename T>
float read_field_as(const std::byte * at)
{
  T v;
  std::memcpy(&v, at, sizeof(v));
  return static_cast<float>(v);
}

// The reader for a field's datatype, chosen once per cloud so the per-point
// loop does not re-dispatch on the type for every field of every point.
FieldReadFn field_reader(core::pointcloud::PointFieldType type)
{
  switch (type) {
    case core::pointcloud::PointFieldType::kFloat32:
      return &read_field_as<float>;
    case core::pointcloud::PointFieldType::kFloat64:
      return &read_field_as<double>;
    case core::pointcloud::PointFieldType::kInt8:
      return &read_field_as<std::int8_t>;
    case core::pointcloud::PointFieldType::kUint8:
      return &read_field_as<std::uint8_t>;
    case core::pointcloud::PointFieldType::kInt16:
      return &read_field_as<std::int16_t>;
    case core::pointcloud::PointFieldType::kUint16:
      return &read_field_as<std::uint16_t>;
    case core::pointcloud::PointFieldType::kInt32:
      return &read_field_as<std::int32_t>;
    case core::pointcloud::PointFieldType::kUint32:
      return &read_field_as<std::uint32_t>;
  }
  return &read_field_as<float>;  // unreachable; keeps -Wreturn-type / cppcheck happy
}

// A readable field of the cloud being accumulated: its byte offset inside a
// point and the reader for its datatype.
struct FieldRef
{
  std::uint32_t offset;
  FieldReadFn read;
};

// Points per chunk below which a cloud is placed on the calling thread alone:
// spreading a few thousand points over the pool costs more in wake-ups than
// it saves.
constexpr std::size_t kMinPointsPerChunk = 4096;
// Voxels below which MapAccumulator::finish sorts on the calling thread even
// when given a pool, and the stride at which it samples voxel x indices to
// balance its sort ranges.
constexpr std::size_t kParallelFinishMinVoxels = 1U << 16U;
constexpr std::size_t kFinishSampleStride = 1024;

// The field's declared offset+size fits inside point_step (and its count is 1,
// so the readback below sees one value per point).
bool field_readable(const core::pointcloud::PointField & field, std::uint32_t point_step)
{
  return field.count == 1 &&
         field.offset + core::pointcloud::datatype_size(field.datatype) <= point_step;
}

// Minimal JSON string escaping for frame names, which come straight from CLI
// arguments (untrusted input): quotes, backslashes, and control characters
// that would otherwise break the hand-built JSON.
std::string json_escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      default:
        out += c;
    }
  }
  return out;
}

}  // namespace

std::string validate_calibrate_flags(const CalibCamLidarArgs & args)
{
  if (args.of_frame.empty() || args.ref_frame.empty()) {
    return "--of and --ref must be non-empty";
  }
  if (args.samples < 3) {
    return "--samples must be at least 3 (6-DOF needs multiple viewpoints)";
  }
  if (args.max_trans <= 0.0 || args.max_rot_deg <= 0.0) {
    return "--max-trans and --max-rot must be positive";
  }
  if (args.nid_bins < 4 || args.nid_bins > 256) {
    return "--nid-bins must be in [4, 256]";
  }
  if (args.min_depth <= 0.0 || args.max_depth <= args.min_depth) {
    return "--min-depth must be positive and below --max-depth";
  }
  if (args.voxel_size < 0.0) {
    return "--voxel must be non-negative (0 keeps every point)";
  }
  if (args.keyframe_dist < 0.0 || args.keyframe_rot_deg < 0.0) {
    return "--keyframe-dist and --keyframe-rot must be non-negative (0 disables the gate)";
  }
  if (const auto err = parse_skip_durations(args).second; !err.empty()) {
    return err;
  }
  const auto [cam_offset, cam_offset_err] = parse_cam_offset(args);
  if (!cam_offset_err.empty()) {
    return cam_offset_err;
  }
  if (!args.imu_topic.empty() && !cam_offset.auto_estimate) {
    return "--imu only applies with --cam-offset auto (nothing else reads the IMU)";
  }
  return parse_fix_spec(args.fix_axes).second;
}

std::pair<CamOffsetSpec, std::string> parse_cam_offset(const CalibCamLidarArgs & args)
{
  CamOffsetSpec spec;
  if (args.cam_offset.empty()) {
    return {spec, ""};
  }
  if (args.cam_offset == "auto") {
    spec.auto_estimate = true;
    return {spec, ""};
  }
  // The --skip-start grammar: the unit suffix is mandatory, so a bare number
  // (ms or s?) is a parse failure rather than a guess. The sign is kept.
  const auto ns = core::parse_duration_ns(args.cam_offset, core::DurationUnitPolicy::RequireUnit);
  if (!ns.has_value()) {
    return {
      spec, "--cam-offset: '" + args.cam_offset +
              "' is neither 'auto' nor a duration (expected e.g. -42ms, 1.5s; a unit suffix "
              "is required)"};
  }
  // A sensor clock offset is milliseconds to seconds; whole days only ever
  // mean a typo or a wrong unit, and the parser alone lets a value sit near
  // the int64 limit, where adding it to an epoch stamp would overflow.
  constexpr std::int64_t kMaxCamOffsetNs = 24LL * 3600LL * 1'000'000'000LL;
  if (*ns > kMaxCamOffsetNs || *ns < -kMaxCamOffsetNs) {
    return {
      spec, "--cam-offset: '" + args.cam_offset +
              "' is beyond +-24h; a sensor clock offset is "
              "milliseconds to seconds (check the unit suffix)"};
  }
  spec.offset_ns = *ns;
  return {spec, ""};
}

std::pair<std::array<std::int64_t, 2>, std::string> parse_skip_durations(
  const CalibCamLidarArgs & args)
{
  std::array<std::int64_t, 2> out{0, 0};
  const std::array<std::pair<const std::string *, const char *>, 2> flags{
    {{&args.skip_start, "--skip-start"}, {&args.skip_end, "--skip-end"}}};
  for (std::size_t i = 0; i < flags.size(); ++i) {
    const auto & [value, name] = flags[i];
    if (value->empty()) {
      continue;
    }
    // The same grammar as `trim --start/--end`: the unit suffix is mandatory,
    // so a bare number is a parse failure rather than a guess.
    const auto ns = core::parse_duration_ns(*value, core::DurationUnitPolicy::RequireUnit);
    if (!ns.has_value()) {
      return {
        out, std::string(name) + ": '" + *value +
               "' is not a duration (expected e.g. 30s, 500ms; a unit suffix is required)"};
    }
    if (*ns < 0) {
      return {out, std::string(name) + " must be non-negative"};
    }
    out[i] = *ns;
  }
  return {out, ""};
}

std::pair<FixSpec, std::string> parse_fix_spec(const std::string & csv)
{
  FixSpec spec;
  spec.auto_fix = false;
  if (csv.empty()) {
    spec.auto_fix = true;
    return {spec, ""};
  }
  bool saw_auto = false;
  bool saw_none = false;
  std::size_t axis_count = 0;
  std::size_t begin = 0;
  while (begin <= csv.size()) {
    const std::size_t comma = csv.find(',', begin);
    const std::string token =
      csv.substr(begin, comma == std::string::npos ? std::string::npos : comma - begin);
    if (token == "auto") {
      saw_auto = true;
    } else if (token == "none") {
      saw_none = true;
    } else {
      const auto it = std::find(kAxisNames.begin(), kAxisNames.end(), token);
      if (it == kAxisNames.end()) {
        return {
          spec, "--fix: unknown token '" + token + "' (expected x,y,z,roll,pitch,yaw,auto,none)"};
      }
      spec.fixed[static_cast<std::size_t>(it - kAxisNames.begin())] = true;
      ++axis_count;
    }
    if (comma == std::string::npos) {
      break;
    }
    begin = comma + 1;
  }
  if (saw_none && (saw_auto || axis_count > 0)) {
    return {spec, "--fix: 'none' cannot be combined with other tokens"};
  }
  if (axis_count == 6) {
    return {spec, "--fix: fixing all six axes leaves nothing to optimize"};
  }
  spec.auto_fix = saw_auto;
  return {spec, ""};
}

std::vector<std::size_t> eligible_sample_indices(
  std::span<const std::int64_t> image_stamps_ns, std::int64_t traj_begin_ns,
  std::int64_t traj_end_ns, std::int64_t margin_ns)
{
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < image_stamps_ns.size(); ++i) {
    if (
      image_stamps_ns[i] >= traj_begin_ns + margin_ns &&
      image_stamps_ns[i] <= traj_end_ns - margin_ns) {
      eligible.push_back(i);
    }
  }
  return eligible;
}

std::vector<std::size_t> pick_sample_indices(
  std::span<const std::int64_t> image_stamps_ns, std::int64_t traj_begin_ns,
  std::int64_t traj_end_ns, int samples, std::int64_t margin_ns)
{
  const std::vector<std::size_t> eligible =
    eligible_sample_indices(image_stamps_ns, traj_begin_ns, traj_end_ns, margin_ns);
  if (eligible.size() <= static_cast<std::size_t>(samples)) {
    return eligible;
  }
  std::vector<std::size_t> picks;
  picks.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double a = static_cast<double>(i) / (samples - 1);
    picks.push_back(
      eligible[static_cast<std::size_t>(a * static_cast<double>(eligible.size() - 1))]);
  }
  return picks;
}

std::vector<std::pair<std::size_t, std::size_t>> pose_gate_intervals(
  std::span<const core::calib::Mat4> poses, double min_dist_m, double min_rot_rad)
{
  std::vector<std::pair<std::size_t, std::size_t>> intervals;
  if (poses.empty()) {
    return intervals;
  }
  std::size_t anchor = 0;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const auto a = core::calib::translation_of(poses[anchor]);
    const auto b = core::calib::translation_of(poses[i]);
    const double dx = b[0] - a[0];
    const double dy = b[1] - a[1];
    const double dz = b[2] - a[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const bool moved = min_dist_m > 0.0 && dist >= min_dist_m;
    const bool rotated = min_rot_rad > 0.0 && core::calib::rotation_angle_between(
                                                poses[anchor], poses[i]) >= min_rot_rad;
    if (moved || rotated) {
      intervals.emplace_back(anchor, i);
      anchor = i;
    }
  }
  intervals.emplace_back(anchor, poses.size());
  return intervals;
}

double gray_sharpness(const core::calib::GrayImage & image)
{
  if (
    image.width < 3 || image.height < 3 ||
    image.gray.size() != static_cast<std::size_t>(image.width) * image.height) {
    return 0.0;
  }
  double sum = 0.0;
  for (std::uint32_t y = 1; y + 1 < image.height; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * image.width;
    for (std::uint32_t x = 1; x + 1 < image.width; ++x) {
      const int gx =
        static_cast<int>(image.gray[row + x + 1]) - static_cast<int>(image.gray[row + x - 1]);
      const int gy = static_cast<int>(image.gray[row + image.width + x]) -
                     static_cast<int>(image.gray[row - image.width + x]);
      sum += std::abs(gx) + std::abs(gy);
    }
  }
  const auto interior =
    static_cast<double>(image.width - 2) * static_cast<double>(image.height - 2);
  return sum / interior;
}

std::optional<core::calib::Mat4> mat4_from_quat(
  double tx, double ty, double tz, double qx, double qy, double qz, double qw)
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  // A zero-norm quaternion (an all-zero geometry_msgs Quaternion straight off
  // the wire is the common one) or any non-finite component would otherwise
  // divide through to NaNs and silently poison every transform downstream.
  if (
    !std::isfinite(norm) || norm <= 0.0 || !std::isfinite(tx) || !std::isfinite(ty) ||
    !std::isfinite(tz)) {
    return std::nullopt;
  }
  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  core::calib::Mat4 m = core::calib::identity_mat4();
  m[0] = 1 - 2 * (y * y + z * z);
  m[1] = 2 * (x * y + z * w);
  m[2] = 2 * (x * z - y * w);
  m[4] = 2 * (x * y - z * w);
  m[5] = 1 - 2 * (x * x + z * z);
  m[6] = 2 * (y * z + x * w);
  m[8] = 2 * (x * z + y * w);
  m[9] = 2 * (y * z - x * w);
  m[10] = 1 - 2 * (x * x + y * y);
  m[12] = tx;
  m[13] = ty;
  m[14] = tz;
  return m;
}

std::optional<core::calib::Mat4> interpolate_trajectory(
  std::span<const core::TrajectoryPose> poses, std::int64_t stamp_ns)
{
  if (
    poses.size() < 2 || stamp_ns < poses.front().timestamp_ns ||
    stamp_ns > poses.back().timestamp_ns) {
    return std::nullopt;
  }
  const auto after = std::lower_bound(
    poses.begin(), poses.end(), stamp_ns,
    [](const core::TrajectoryPose & p, std::int64_t t) { return p.timestamp_ns < t; });
  const auto & p1 = *after;
  const auto & p0 = after == poses.begin() ? *after : *(after - 1);
  const double span = static_cast<double>(p1.timestamp_ns - p0.timestamp_ns);
  const double a = span > 0.0 ? static_cast<double>(stamp_ns - p0.timestamp_ns) / span : 0.0;

  // Slerp with sign fix (shortest arc), falling back to nlerp near-parallel.
  const std::array<double, 4> q0{p0.qx, p0.qy, p0.qz, p0.qw};
  std::array<double, 4> q1{p1.qx, p1.qy, p1.qz, p1.qw};
  double dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
  if (dot < 0.0) {
    for (auto & c : q1) {
      c = -c;
    }
    dot = -dot;
  }
  std::array<double, 4> q{};
  if (dot > 0.9995) {
    for (int i = 0; i < 4; ++i) {
      q[i] = q0[i] + a * (q1[i] - q0[i]);
    }
  } else {
    const double theta = std::acos(dot);
    const double s = std::sin(theta);
    for (int i = 0; i < 4; ++i) {
      q[i] = (std::sin((1.0 - a) * theta) * q0[i] + std::sin(a * theta) * q1[i]) / s;
    }
  }
  // Quaternion -> column-major rotation (mat4_from_quat normalizes q itself),
  // translation lerped.
  return mat4_from_quat(
    p0.tx + a * (p1.tx - p0.tx), p0.ty + a * (p1.ty - p0.ty), p0.tz + a * (p1.tz - p0.tz), q[0],
    q[1], q[2], q[3]);
}

std::size_t MapAccumulator::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  // Boost-style combine of the three indices: neighbouring voxels differ by 1
  // on one axis, which the golden-ratio constant and the shifts spread across
  // buckets instead of leaving them adjacent.
  std::size_t seed = 0;
  for (const std::int32_t v : {key.x, key.y, key.z}) {
    seed ^= std::hash<std::int32_t>{}(v) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
  }
  return seed;
}

MapAccumulator::MapAccumulator(double voxel_size, int partitions)
: voxel_size_(voxel_size),
  voxels_(static_cast<std::size_t>(voxel_size > 0.0 ? std::max(partitions, 1) : 1))
{
}

std::optional<MapAccumulator::VoxelKey> MapAccumulator::voxel_of(
  const std::array<float, 3> & point) const
{
  // Floor, never truncate: truncation folds -0.5..0 onto the same index as
  // 0..0.5, merging every pair of voxels that straddles an axis.
  const double qx = std::floor(static_cast<double>(point[0]) / voxel_size_);
  const double qy = std::floor(static_cast<double>(point[1]) / voxel_size_);
  const double qz = std::floor(static_cast<double>(point[2]) / voxel_size_);
  constexpr double kMinIndex = -2147483648.0;
  constexpr double kMaxIndex = 2147483647.0;
  const auto representable = [](double q) { return q >= kMinIndex && q <= kMaxIndex; };
  if (!representable(qx) || !representable(qy) || !representable(qz)) {
    return std::nullopt;
  }
  return VoxelKey{
    static_cast<std::int32_t>(qx), static_cast<std::int32_t>(qy), static_cast<std::int32_t>(qz)};
}

int MapAccumulator::partition_of(const VoxelKey & key) const
{
  // A second mix over the map's own hash, so the keys one partition receives
  // do not all share a residue that partition's map then hashes on again.
  const std::uint64_t mixed =
    static_cast<std::uint64_t>(VoxelKeyHash{}(key)) * 0x9E3779B97F4A7C15ULL;
  return static_cast<int>((mixed >> 32U) % voxels_.size());
}

void MapAccumulator::add_to_voxel(
  int partition, const VoxelKey & key, const std::array<float, 3> & point, float intensity)
{
  assert(voxel_size_ > 0.0 && partition == partition_of(key));
  auto & accum = voxels_[static_cast<std::size_t>(partition)][key];
  accum.x += point[0];
  accum.y += point[1];
  accum.z += point[2];
  accum.intensity += intensity;
  ++accum.count;
}

bool MapAccumulator::add(const std::array<float, 3> & point, float intensity)
{
  if (voxel_size_ <= 0.0) {
    raw_.points.push_back(point);
    raw_.intensities.push_back(intensity);
    return true;
  }
  const auto key = voxel_of(point);
  if (!key.has_value()) {
    return false;
  }
  add_to_voxel(partition_of(*key), *key, point, intensity);
  return true;
}

std::size_t MapAccumulator::size() const
{
  if (voxel_size_ <= 0.0) {
    return raw_.points.size();
  }
  std::size_t total = 0;
  for (const auto & partition : voxels_) {
    total += partition.size();
  }
  return total;
}

core::pointcloud::PcdCloud MapAccumulator::finish(core::WorkerPool * pool)
{
  if (voxel_size_ <= 0.0) {
    return std::move(raw_);
  }
  using Entry = std::pair<VoxelKey, VoxelAccum>;
  // Sorted by voxel index so neither the hash containers' iteration order nor
  // the partition split — implementation details, not properties of the bag —
  // ever reaches the map. Two runs over the same clouds must produce the same
  // map.
  const auto key_less = [](const Entry & a, const Entry & b) {
    if (a.first.x != b.first.x) {
      return a.first.x < b.first.x;
    }
    if (a.first.y != b.first.y) {
      return a.first.y < b.first.y;
    }
    return a.first.z < b.first.z;
  };
  const auto emit = [](const Entry & entry, std::array<float, 3> & point, float & intensity) {
    const VoxelAccum & accum = entry.second;
    const double n = accum.count;
    point = {
      static_cast<float>(accum.x / n), static_cast<float>(accum.y / n),
      static_cast<float>(accum.z / n)};
    intensity = static_cast<float>(accum.intensity / n);
  };
  const std::size_t total = size();
  core::pointcloud::PcdCloud out;
  out.points.resize(total);
  out.intensities.resize(total);

  // A small map, or no pool: one vector, one sort, on the calling thread.
  if (pool == nullptr || pool->size() <= 1 || total < kParallelFinishMinVoxels) {
    std::vector<Entry> ordered;
    ordered.reserve(total);
    for (auto & partition : voxels_) {
      ordered.insert(ordered.end(), partition.begin(), partition.end());
      partition.clear();
    }
    std::sort(ordered.begin(), ordered.end(), key_less);
    for (std::size_t i = 0; i < total; ++i) {
      emit(ordered[i], out.points[i], out.intensities[i]);
    }
    return out;
  }

  // A large map on a pool: drain every partition on its own worker, split the
  // union into voxel-x ranges balanced on a sample of the x indices, sort the
  // ranges on the pool and write each at its offset. The ranges are disjoint
  // and emitted in x order, and every range is sorted by (x, y, z) inside, so
  // the result is exactly the single sort above.
  const std::size_t partitions = voxels_.size();
  const auto ranges = static_cast<std::size_t>(pool->size());
  std::vector<std::vector<Entry>> drained(partitions);
  std::vector<std::vector<std::int32_t>> sampled(partitions);
  pool->parallel_for(partitions, [&](std::size_t p) {
    auto & map = voxels_[p];
    drained[p].reserve(map.size());
    std::size_t i = 0;
    for (const auto & entry : map) {
      drained[p].push_back(entry);
      if (i++ % kFinishSampleStride == 0) {
        sampled[p].push_back(entry.first.x);
      }
    }
    // Release the nodes here, on the worker, rather than all of them on the
    // calling thread at the end.
    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash>().swap(map);
  });
  std::vector<std::int32_t> xs;
  for (const auto & s : sampled) {
    xs.insert(xs.end(), s.begin(), s.end());
  }
  std::sort(xs.begin(), xs.end());
  // Range r holds the entries with bounds[r-1] <= x < bounds[r] (open-ended
  // at both ends); the bounds are the sample's quantiles.
  std::vector<std::int32_t> bounds;
  for (std::size_t r = 1; r < ranges && !xs.empty(); ++r) {
    bounds.push_back(xs[(r * xs.size()) / ranges]);
  }
  const auto range_of = [&](std::int32_t x) {
    return static_cast<std::size_t>(
      std::upper_bound(bounds.begin(), bounds.end(), x) - bounds.begin());
  };
  std::vector<std::vector<std::vector<Entry>>> buckets(
    partitions, std::vector<std::vector<Entry>>(ranges));
  pool->parallel_for(partitions, [&](std::size_t p) {
    for (const auto & entry : drained[p]) {
      buckets[p][range_of(entry.first.x)].push_back(entry);
    }
    std::vector<Entry>().swap(drained[p]);
  });
  std::vector<std::size_t> offsets(ranges + 1, 0);
  for (std::size_t r = 0; r < ranges; ++r) {
    offsets[r + 1] = offsets[r];
    for (std::size_t p = 0; p < partitions; ++p) {
      offsets[r + 1] += buckets[p][r].size();
    }
  }
  pool->parallel_for(ranges, [&](std::size_t r) {
    std::vector<Entry> entries;
    entries.reserve(offsets[r + 1] - offsets[r]);
    for (std::size_t p = 0; p < partitions; ++p) {
      entries.insert(entries.end(), buckets[p][r].begin(), buckets[p][r].end());
      std::vector<Entry>().swap(buckets[p][r]);
    }
    std::sort(entries.begin(), entries.end(), key_less);
    for (std::size_t i = 0; i < entries.size(); ++i) {
      emit(entries[i], out.points[offsets[r] + i], out.intensities[offsets[r] + i]);
    }
  });
  return out;
}

bool point_in_any_view(const std::array<double, 3> & p, std::span<const SampleViewFrustum> views)
{
  for (const auto & v : views) {
    const auto pc = core::calib::transform_point(v.t_cam_ref, p);
    if (pc[2] < v.lo_depth || pc[2] > v.hi_depth) {
      continue;
    }
    const double xn = pc[0] / pc[2];
    const double yn = pc[1] / pc[2];
    if (xn >= v.lo_xn && xn <= v.hi_xn && yn >= v.lo_yn && yn <= v.hi_yn) {
      return true;
    }
  }
  return false;
}

std::vector<ViewRejectBox> view_reject_boxes(std::span<const SampleViewFrustum> views)
{
  constexpr double kInf = std::numeric_limits<double>::infinity();
  std::vector<ViewRejectBox> boxes;
  boxes.reserve(views.size());
  for (const auto & v : views) {
    ViewRejectBox box;
    if (v.lo_depth <= 0.0) {
      box.lo = {-kInf, -kInf, -kInf};
      box.hi = {kInf, kInf, kInf};
      boxes.push_back(box);
      continue;
    }
    box.lo = {kInf, kInf, kInf};
    box.hi = {-kInf, -kInf, -kInf};
    const core::calib::Mat4 t_ref_cam = core::calib::rigid_inverse(v.t_cam_ref);
    for (const double depth : {v.lo_depth, v.hi_depth}) {
      for (const double xn : {v.lo_xn, v.hi_xn}) {
        for (const double yn : {v.lo_yn, v.hi_yn}) {
          const auto corner =
            core::calib::transform_point(t_ref_cam, {xn * depth, yn * depth, depth});
          for (std::size_t k = 0; k < 3; ++k) {
            box.lo[k] = std::min(box.lo[k], corner[k] - kViewRejectBoxPadMeters);
            box.hi[k] = std::max(box.hi[k], corner[k] + kViewRejectBoxPadMeters);
          }
        }
      }
    }
    boxes.push_back(box);
  }
  return boxes;
}

bool point_in_any_view(
  const std::array<double, 3> & p, std::span<const SampleViewFrustum> views,
  std::span<const ViewRejectBox> boxes)
{
  for (std::size_t i = 0; i < views.size(); ++i) {
    const auto & box = boxes[i];
    if (
      p[0] < box.lo[0] || p[0] > box.hi[0] || p[1] < box.lo[1] || p[1] > box.hi[1] ||
      p[2] < box.lo[2] || p[2] > box.hi[2]) {
      continue;
    }
    // Depth first: the window rejects most of what the box let through, and
    // the other two coordinates are only needed once it passes. Each row is
    // the same expression transform_point evaluates, so the values are the
    // ones the plain predicate above tests.
    const auto & v = views[i];
    const double z = core::calib::transform_point_z(v.t_cam_ref, p);
    if (z < v.lo_depth || z > v.hi_depth) {
      continue;
    }
    const double xn = core::calib::transform_point_x(v.t_cam_ref, p) / z;
    const double yn = core::calib::transform_point_y(v.t_cam_ref, p) / z;
    if (xn >= v.lo_xn && xn <= v.hi_xn && yn >= v.lo_yn && yn <= v.hi_yn) {
      return true;
    }
  }
  return false;
}

MapAccumulationContext::MapAccumulationContext(
  std::span<const SampleViewFrustum> views_in, core::WorkerPool * pool_in)
: views(views_in), boxes(view_reject_boxes(views_in)), pool(pool_in)
{
}

std::optional<std::string> accumulate_cloud_into_map(
  MapAccumulator & map, core::pointcloud::PointCloud2 cloud,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & t_of_cloud, MapAccumulationStats & stats,
  std::span<const SampleViewFrustum> views)
{
  MapAccumulationContext context{views, nullptr};
  return accumulate_cloud_into_map(map, std::move(cloud), trajectory, t_of_cloud, stats, context);
}

std::optional<std::string> accumulate_cloud_into_map(
  MapAccumulator & map, core::pointcloud::PointCloud2 cloud,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & t_of_cloud, MapAccumulationStats & stats,
  MapAccumulationContext & context)
{
  ++stats.clouds_read;

  // The field layout, captured by value up front: deskew below replaces the
  // cloud (field table preserved), which would dangle pointers into the old
  // one. intensity is mandatory (NID needs it); x/y/z must be readable.
  const auto field_ref = [&cloud](const std::string & name) -> std::optional<FieldRef> {
    for (const auto & f : cloud.fields) {
      if (f.name == name && field_readable(f, cloud.point_step)) {
        return FieldRef{f.offset, field_reader(f.datatype)};
      }
    }
    return std::nullopt;
  };
  if (cloud.is_bigendian) {
    return std::string("big-endian point data is not supported");
  }
  const auto fx = field_ref("x");
  const auto fy = field_ref("y");
  const auto fz = field_ref("z");
  const auto fi = field_ref("intensity");
  if (!fx.has_value() || !fy.has_value() || !fz.has_value()) {
    return std::string("no readable x/y/z fields");
  }
  if (!fi.has_value()) {
    return std::string("no intensity field; NID needs lidar intensity");
  }
  const std::size_t n = static_cast<std::size_t>(cloud.height) * cloud.width;
  if (cloud.data.size() < n * cloud.point_step) {
    return std::string("inconsistent point layout (data smaller than height*width*point_step)");
  }

  // The cloud's placement pose (and deskew reference stamp). A cloud outside
  // the trajectory span is skipped: clamping it to an endpoint pose would
  // smear the map.
  const std::int64_t stamp_ns = cloud.timestamp_ns;
  const auto t_ref_of = interpolate_trajectory(trajectory, stamp_ns);
  if (!t_ref_of.has_value()) {
    ++stats.clouds_skipped_out_of_span;
    return std::nullopt;
  }

  // T_of_cloud as a Mat4 (identity when the cloud frame already is --of).
  core::calib::Mat4 t_of_cloud_mat = core::calib::identity_mat4();
  if (t_of_cloud.has_value()) {
    const auto m = mat4_from_quat(
      t_of_cloud->translation.x, t_of_cloud->translation.y, t_of_cloud->translation.z,
      t_of_cloud->rotation.x, t_of_cloud->rotation.y, t_of_cloud->rotation.z,
      t_of_cloud->rotation.w);
    if (!m.has_value()) {
      return std::string("the cloud frame's static extrinsic is not a usable transform");
    }
    t_of_cloud_mat = *m;
  }

  // Deskew gate: a usable per-point time field whose values are not all one
  // stamp. An all-zero relative field, or the t_ref-constant field an
  // already-undistorted cloud carries, has a zero span — no sweep motion to
  // undo, so the cloud is accumulated as-is.
  if (const auto time_field = core::pointcloud::find_point_time_field(cloud.fields);
      time_field.has_value()) {
    const auto span = core::pointcloud::absolute_point_time_span_ns(cloud, *time_field, stamp_ns);
    if (span.has_value() && span->max_ns > span->min_ns) {
      auto deskewed =
        core::pointcloud::deskew_pointcloud2(std::move(cloud), stamp_ns, trajectory, t_of_cloud);
      if (!deskewed.ok()) {
        return deskewed.error;
      }
      cloud = std::move(*deskewed.cloud);
      ++stats.clouds_deskewed;
      stats.points_clamped_out_of_span += deskewed.points_out_of_span;
    }
  }

  const core::calib::Mat4 t_ref_cloud = core::calib::mat4_multiply(*t_ref_of, t_of_cloud_mat);

  // Placement: one chunk of the point range per worker (a small cloud stays on
  // the calling thread). Each chunk reads its points' fields, drops the
  // non-finite ones, places the rest and culls them against the views,
  // keeping its survivors in point order — per-point work over immutable
  // input, so the split changes nothing about any one point.
  const std::size_t parallelism =
    context.pool != nullptr ? static_cast<std::size_t>(context.pool->size()) : 1;
  const std::size_t chunk_count = std::clamp<std::size_t>(n / kMinPointsPerChunk, 1, parallelism);
  const std::size_t chunk_size = (n + chunk_count - 1) / chunk_count;
  const auto partitions = static_cast<std::size_t>(map.partitions());
  const bool gridded = map.gridded();
  if (context.chunks.size() < chunk_count) {
    context.chunks.resize(chunk_count);
  }
  const std::span<const SampleViewFrustum> views = context.views;
  const std::span<const ViewRejectBox> boxes = context.boxes;
  const auto place_chunk = [&](std::size_t c) {
    auto & chunk = context.chunks[c];
    chunk.buckets.resize(partitions);
    for (auto & bucket : chunk.buckets) {
      bucket.keys.clear();
      bucket.points.clear();
      bucket.intensities.clear();
    }
    chunk.nonfinite = 0;
    chunk.culled = 0;
    chunk.placed = 0;
    const std::size_t begin = c * chunk_size;
    const std::size_t end = std::min(n, begin + chunk_size);
    const std::byte * const data = cloud.data.data();
    for (std::size_t i = begin; i < end; ++i) {
      const std::byte * const base = data + i * cloud.point_step;
      const float x = fx->read(base + fx->offset);
      const float y = fy->read(base + fy->offset);
      const float z = fz->read(base + fz->offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++chunk.nonfinite;
        continue;
      }
      const auto p = core::calib::transform_point(t_ref_cloud, {x, y, z});
      // A point no sample's view can ever contain is dropped here rather than
      // carried through the voxel grid and discarded at candidate assembly.
      if (!views.empty() && !point_in_any_view(p, views, boxes)) {
        ++chunk.culled;
        continue;
      }
      const std::array<float, 3> placed{
        static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
      std::size_t partition = 0;
      if (gridded) {
        // A point too far out to index a voxel is garbage the map cannot
        // place; counted with the non-finite drops rather than folded into a
        // wrong voxel. The key is kept with the point so the insertion does
        // not quantize it a second time.
        const auto key = map.voxel_of(placed);
        if (!key.has_value()) {
          ++chunk.nonfinite;
          continue;
        }
        partition = static_cast<std::size_t>(map.partition_of(*key));
        chunk.buckets[partition].keys.push_back(*key);
      }
      auto & bucket = chunk.buckets[partition];
      bucket.points.push_back(placed);
      bucket.intensities.push_back(fi->read(base + fi->offset));
      ++chunk.placed;
    }
  };
  if (context.pool != nullptr && chunk_count > 1) {
    context.pool->parallel_for(chunk_count, place_chunk);
  } else {
    for (std::size_t c = 0; c < chunk_count; ++c) {
      place_chunk(c);
    }
  }
  for (std::size_t c = 0; c < chunk_count; ++c) {
    const auto & chunk = context.chunks[c];
    stats.points_dropped_nonfinite += chunk.nonfinite;
    stats.points_culled_out_of_view += chunk.culled;
    stats.points_added += chunk.placed;
  }

  // Insertion: one worker per map partition, each walking the chunks in order
  // and its own bucket of each in point order. A voxel receives its points
  // in exactly the sequence a single loop over the points would have fed it,
  // so its running sums add the same values in the same order.
  const auto insert_partition = [&](std::size_t t) {
    const int partition = static_cast<int>(t);
    for (std::size_t c = 0; c < chunk_count; ++c) {
      const auto & bucket = context.chunks[c].buckets[t];
      for (std::size_t k = 0; k < bucket.points.size(); ++k) {
        if (gridded) {
          map.add_to_voxel(partition, bucket.keys[k], bucket.points[k], bucket.intensities[k]);
        } else {
          map.add(bucket.points[k], bucket.intensities[k]);
        }
      }
    }
  };
  if (context.pool != nullptr && partitions > 1) {
    context.pool->parallel_for(partitions, insert_partition);
  } else {
    for (std::size_t t = 0; t < partitions; ++t) {
      insert_partition(t);
    }
  }
  return std::nullopt;
}

std::string default_calib_cam_lidar_output_path(const std::filesystem::path & input)
{
  return input.stem().string() + "_calib_cam_lidar.yaml";
}

// The physical-units axis mixture of a held direction: each normalized
// component scaled by its axis's probe step and renormalized, so the report
// reads as "this combination of the edge's actual axes" (0.99y + 0.17yaw).
// Dominant component positive (the core already sign-normalizes; this is the
// same convention after the step scaling).
std::array<double, 6> held_display_components(const core::calib::HeldDirection & held)
{
  std::array<double, 6> comp;
  double norm = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    const double step = i < 3 ? core::calib::kProbeStepTrans : core::calib::kProbeStepRot;
    comp[i] = held.unit[i] * step;
    norm += comp[i] * comp[i];
  }
  norm = std::sqrt(norm);
  std::size_t dominant = 0;
  for (std::size_t i = 0; i < 6; ++i) {
    comp[i] /= norm;
    if (std::abs(comp[i]) > std::abs(comp[dominant])) {
      dominant = i;
    }
  }
  if (comp[dominant] < 0.0) {
    for (auto & c : comp) {
      c = -c;
    }
  }
  return comp;
}

// "0.99y - 0.17yaw": the axis mixture of a held direction, components below
// 0.005 dropped.
std::string held_direction_text(const core::calib::HeldDirection & held)
{
  const auto comp = held_display_components(held);
  std::string out;
  bool first = true;
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::abs(comp[i]) < 0.005) {
      continue;
    }
    if (first) {
      out += fmt::format("{:.2f}{}", comp[i], kAxisNames[i]);
      first = false;
    } else {
      out += comp[i] < 0.0 ? " - " : " + ";
      out += fmt::format("{:.2f}{}", std::abs(comp[i]), kAxisNames[i]);
    }
  }
  return out;
}

// The per-axis curvature evidence as a short string: mean/std_error, the
// ratio the significance test is decided by. "-" when the axis was not
// probed (fixed), "inf" when the estimate has no spread (a single sample
// pair — the ratio is then unbounded).
std::string curvature_ratio_text(const core::calib::CurvatureEstimate & est)
{
  if (est.pairs == 0) {
    return "-";
  }
  if (est.std_error == 0.0) {
    return "inf";
  }
  return fmt::format("{:.2f}", est.mean / est.std_error);
}

std::string render_calibrate_summary(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const std::string & yaml_path,
  const CamOffsetReport & cam_offset)
{
  std::string out = fmt::format("calib cam-lidar: {} -> {}\n", args.parent_frame, args.child_frame);
  out += fmt::format(
    "{:<6} {:>14} {:>14} {:>14} {:>8}  {}\n", "axis", "bag value", "refined value", "delta",
    "curv/se", "observability");
  const auto edge_after = core::calib::apply_edge_delta(edge_before, result.delta);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double unit_scale = is_rotation_axis(axis) ? kRadToDeg : 1.0;
    const double before = edge_before[axis] * unit_scale;
    const double delta = result.delta[axis] * unit_scale;
    const double after = edge_after[axis] * unit_scale;
    out += fmt::format(
      "{:<6} {:>14.6f} {:>14.6f} {:>14.6f} {:>8}  {}\n", kAxisNames[axis], before, after, delta,
      curvature_ratio_text(result.curvature[axis]),
      axis_observability_name(result.observability[axis]));
  }
  out += fmt::format("\nnid: {} -> {}\n", result.nid_before, result.nid_after);
  out += fmt::format("samples used: {}\n", result.samples_used);
  // Only when set: the shift is part of what the refined edge was fitted
  // under, so a report that is re-read later must carry it. Under auto the
  // measurement behind the value is spelled out too.
  if (cam_offset.applied_ns != 0 || cam_offset.estimate.has_value()) {
    out += fmt::format(
      "camera stamp offset: {:+.3f} ms", static_cast<double>(cam_offset.applied_ns) / 1e6);
    if (cam_offset.estimate.has_value()) {
      const auto & e = *cam_offset.estimate;
      out += fmt::format(
        " (estimated: +-{:.1f} ms, {} frame pair(s) vs {}, {} solver)",
        static_cast<double>(e.std_ns) / 1e6, e.pairs, e.method, e.visual_estimator);
    }
    out += "\n";
  }

  std::vector<std::array<double, 6>> held_components;
  held_components.reserve(result.auto_held.size());
  for (const auto & held : result.auto_held) {
    held_components.push_back(held_display_components(held));
    out += fmt::format("held at bag value (auto): {}\n", held_direction_text(held));
  }

  // A degenerate axis warns that its delta is still in the output and was NOT
  // held at the bag's value, which is what --fix would give. An axis dominated
  // by a held direction needs no warning: the held line above already says its
  // content is the bag value. Under --fix auto every degenerate axis is
  // covered by construction (the label is only emitted for held content), so
  // this is the --fix none / manual-axes case.
  bool warned = false;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    if (result.observability[axis] == core::calib::AxisObservability::kDegenerate) {
      const bool covered = std::any_of(
        held_components.begin(), held_components.end(),
        [axis](const std::array<double, 6> & comp) { return std::abs(comp[axis]) >= 0.5; });
      if (covered) {
        continue;
      }
      out += fmt::format(
        "warning: {} is not observable from this data; the delta shown is "
        "unconstrained — re-run with --fix {} to hold the bag value\n",
        kAxisNames[axis], kAxisNames[axis]);
      warned = true;
    }
  }
  if (warned) {
    out += "\n";
  }
  out += fmt::format(
    "apply with: bagwiz tf static update -i {} --yaml {}\n", args.input_path.string(), yaml_path);
  return out;
}

std::string render_calibrate_json(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const CamOffsetReport & cam_offset)
{
  std::string out = "{\n";
  out += fmt::format("  \"parent\": \"{}\",\n", json_escape(args.parent_frame));
  out += fmt::format("  \"child\": \"{}\",\n", json_escape(args.child_frame));
  out += fmt::format("  \"nid_before\": {},\n", result.nid_before);
  out += fmt::format("  \"nid_after\": {},\n", result.nid_after);
  out += fmt::format("  \"samples\": {},\n", result.samples_used);
  // The --cam-offset the samples were placed under (0 when omitted), so the
  // report states the shift the refined edge depends on; under auto, the
  // measurement behind it (null for a manual or omitted offset).
  out += fmt::format("  \"cam_offset_ns\": {},\n", cam_offset.applied_ns);
  if (cam_offset.estimate.has_value()) {
    const auto & e = *cam_offset.estimate;
    out += "  \"cam_offset_estimate\": {\n";
    out += fmt::format("    \"offset_ns\": {},\n", e.offset_ns);
    out += fmt::format("    \"std_ns\": {},\n", e.std_ns);
    out += fmt::format("    \"method\": \"{}\",\n", e.method);
    out += fmt::format("    \"visual_estimator\": \"{}\",\n", e.visual_estimator);
    out += fmt::format("    \"pairs\": {},\n", e.pairs);
    out += fmt::format("    \"signal_rms_mrad\": {},\n", e.signal_rms_mrad);
    out += fmt::format("    \"residual_rms_before_mrad\": {},\n", e.residual_rms_before_mrad);
    out += fmt::format("    \"residual_rms_after_mrad\": {},\n", e.residual_rms_after_mrad);
    out += fmt::format(
      "    \"camera_imu_offset_ns\": {},\n",
      e.camera_imu_offset_ns.has_value() ? std::to_string(*e.camera_imu_offset_ns) : "null");
    out += fmt::format(
      "    \"pose_imu_offset_ns\": {}\n",
      e.pose_imu_offset_ns.has_value() ? std::to_string(*e.pose_imu_offset_ns) : "null");
    out += "  },\n";
  } else {
    out += "  \"cam_offset_estimate\": null,\n";
  }
  out += "  \"axes\": {\n";
  const auto edge_after = core::calib::apply_edge_delta(edge_before, result.delta);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double before = edge_before[axis];
    const double delta = result.delta[axis];
    const double after = edge_after[axis];
    out += fmt::format("    \"{}\": {{\n", kAxisNames[axis]);
    out += fmt::format("      \"before\": {},\n", before);
    out += fmt::format("      \"after\": {},\n", after);
    out += fmt::format("      \"delta\": {},\n", delta);
    // The paired-curvature evidence behind the verdict (observability.hpp);
    // null when the axis was not probed (fixed), the ratio additionally null
    // when the estimate has no spread.
    const auto & est = result.curvature[axis];
    if (est.pairs > 0) {
      out += fmt::format("      \"curvature\": {},\n", est.mean);
      out += fmt::format("      \"std_error\": {},\n", est.std_error);
      out += fmt::format(
        "      \"curvature_ratio\": {},\n",
        est.std_error > 0.0 ? fmt::format("{}", est.mean / est.std_error) : "null");
    } else {
      out += "      \"curvature\": null,\n";
      out += "      \"std_error\": null,\n";
      out += "      \"curvature_ratio\": null,\n";
    }
    out += fmt::format(
      "      \"observability\": \"{}\"\n", axis_observability_name(result.observability[axis]));
    out += fmt::format("    }}{}\n", axis + 1 < 6 ? "," : "");
  }
  out += "  },\n";
  // The --fix auto held set: each direction as its physical-units axis
  // mixture (components under 0.005 dropped, dominant positive) plus the
  // paired-curvature measurement that judged it unobservable.
  out += "  \"held\": [";
  if (result.auto_held.empty()) {
    out += "]\n";
  } else {
    out += "\n";
    for (std::size_t i = 0; i < result.auto_held.size(); ++i) {
      const auto & held = result.auto_held[i];
      const auto comp = held_display_components(held);
      out += "    {\n";
      out += "      \"direction\": {";
      bool first = true;
      for (std::size_t axis = 0; axis < 6; ++axis) {
        if (std::abs(comp[axis]) < 0.005) {
          continue;
        }
        out += fmt::format("{}\"{}\": {}", first ? "" : ", ", kAxisNames[axis], comp[axis]);
        first = false;
      }
      out += "},\n";
      out += fmt::format("      \"curvature\": {},\n", held.curvature);
      out += fmt::format("      \"std_error\": {}\n", held.std_error);
      out += fmt::format("    }}{}\n", i + 1 < result.auto_held.size() ? "," : "");
    }
    out += "  ]\n";
  }
  out += "}";
  return out;
}

}  // namespace bagwiz::commands
