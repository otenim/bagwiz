// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/pointcloud/deskew.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
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

// Read one point's field as a float regardless of its declared datatype.
// `offset` must be in bounds for the cloud's point_step (callers check via the
// field table). Little-endian point data only — accumulate_cloud_into_map
// rejects big-endian clouds before reading anything.
float read_cloud_field(
  const core::pointcloud::PointCloud2 & cloud, std::size_t point_idx, std::uint32_t offset,
  core::pointcloud::PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case core::pointcloud::PointFieldType::kFloat32: {
      float v;
      std::memcpy(&v, base, sizeof(v));
      return v;
    }
    case core::pointcloud::PointFieldType::kFloat64: {
      double v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kInt8: {
      std::int8_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kUint8: {
      std::uint8_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kInt16: {
      std::int16_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kUint16: {
      std::uint16_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kInt32: {
      std::int32_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
    case core::pointcloud::PointFieldType::kUint32: {
      std::uint32_t v;
      std::memcpy(&v, base, sizeof(v));
      return static_cast<float>(v);
    }
  }
  return 0.0F;  // unreachable; keeps -Wreturn-type / cppcheck happy
}

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
  if (args.keyframe_dist < 0.0 || args.keyframe_rot_deg < 0.0) {
    return "--keyframe-dist and --keyframe-rot must be non-negative (0 disables the gate)";
  }
  return parse_fixed_axes(args.fix_axes).second;
}

std::pair<std::array<bool, 6>, std::string> parse_fixed_axes(const std::string & csv)
{
  std::array<bool, 6> flags{};
  if (csv.empty()) {
    return {flags, ""};
  }
  std::size_t begin = 0;
  while (begin <= csv.size()) {
    const std::size_t comma = csv.find(',', begin);
    const std::string token =
      csv.substr(begin, comma == std::string::npos ? std::string::npos : comma - begin);
    const auto it = std::find(kAxisNames.begin(), kAxisNames.end(), token);
    if (it == kAxisNames.end()) {
      return {flags, "--fix: unknown axis '" + token + "' (expected x,y,z,roll,pitch,yaw)"};
    }
    flags[static_cast<std::size_t>(it - kAxisNames.begin())] = true;
    if (comma == std::string::npos) {
      break;
    }
    begin = comma + 1;
  }
  const bool all_fixed = std::all_of(flags.begin(), flags.end(), [](bool b) { return b; });
  if (all_fixed) {
    return {flags, "--fix: fixing all six axes leaves nothing to optimize"};
  }
  return {flags, ""};
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

std::optional<std::string> accumulate_cloud_into_map(
  core::pointcloud::PcdCloud & map, core::pointcloud::PointCloud2 cloud,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & t_of_cloud, MapAccumulationStats & stats)
{
  ++stats.clouds_read;

  // The field layout, captured by value up front: deskew below replaces the
  // cloud (field table preserved), which would dangle pointers into the old
  // one. intensity is mandatory (NID needs it); x/y/z must be readable.
  struct FieldRef
  {
    std::uint32_t offset;
    core::pointcloud::PointFieldType type;
  };
  const auto field_ref = [&cloud](const std::string & name) -> std::optional<FieldRef> {
    for (const auto & f : cloud.fields) {
      if (f.name == name && field_readable(f, cloud.point_step)) {
        return FieldRef{f.offset, f.datatype};
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
  map.points.reserve(map.points.size() + n);
  map.intensities.reserve(map.intensities.size() + n);
  for (std::size_t i = 0; i < n; ++i) {
    const float x = read_cloud_field(cloud, i, fx->offset, fx->type);
    const float y = read_cloud_field(cloud, i, fy->offset, fy->type);
    const float z = read_cloud_field(cloud, i, fz->offset, fz->type);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      ++stats.points_dropped_nonfinite;
      continue;
    }
    const auto p = core::calib::transform_point(t_ref_cloud, {x, y, z});
    map.points.push_back(
      {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
    map.intensities.push_back(read_cloud_field(cloud, i, fi->offset, fi->type));
    ++stats.points_added;
  }
  return std::nullopt;
}

std::string default_calib_cam_lidar_output_path(const std::filesystem::path & input)
{
  return input.stem().string() + "_calib_cam_lidar.yaml";
}

std::string render_calibrate_summary(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const std::string & yaml_path)
{
  std::string out = fmt::format("calib cam-lidar: {} -> {}\n", args.parent_frame, args.child_frame);
  out += fmt::format(
    "{:<6} {:>14} {:>14} {:>14}  {}\n", "axis", "bag value", "refined value", "delta",
    "observability");
  const auto edge_after = core::calib::apply_edge_delta(edge_before, result.delta);
  for (std::size_t axis = 0; axis < 6; ++axis) {
    const double unit_scale = is_rotation_axis(axis) ? kRadToDeg : 1.0;
    const double before = edge_before[axis] * unit_scale;
    const double delta = result.delta[axis] * unit_scale;
    const double after = edge_after[axis] * unit_scale;
    out += fmt::format(
      "{:<6} {:>14.6f} {:>14.6f} {:>14.6f}  {}\n", kAxisNames[axis], before, after, delta,
      axis_observability_name(result.observability[axis]));
  }
  out += fmt::format("\nnid: {} -> {}\n", result.nid_before, result.nid_after);
  out += fmt::format("samples used: {}\n", result.samples_used);

  bool warned = false;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    if (result.observability[axis] == core::calib::AxisObservability::kDegenerate) {
      out += fmt::format(
        "warning: {} is not observable from this data; the delta shown is unconstrained — "
        "re-run with --fix {} to hold the bag value\n",
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
  const std::array<double, 6> & edge_before)
{
  std::string out = "{\n";
  out += fmt::format("  \"parent\": \"{}\",\n", json_escape(args.parent_frame));
  out += fmt::format("  \"child\": \"{}\",\n", json_escape(args.child_frame));
  out += fmt::format("  \"nid_before\": {},\n", result.nid_before);
  out += fmt::format("  \"nid_after\": {},\n", result.nid_after);
  out += fmt::format("  \"samples\": {},\n", result.samples_used);
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
    out += fmt::format(
      "      \"observability\": \"{}\"\n", axis_observability_name(result.observability[axis]));
    out += fmt::format("    }}{}\n", axis + 1 < 6 ? "," : "");
  }
  out += "  }\n";
  out += "}";
  return out;
}

}  // namespace bagwiz::commands
