// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "tf_static_calibrate_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
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

std::string validate_calibrate_flags(const TfStaticCalibrateArgs & args)
{
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

std::vector<std::size_t> pick_sample_indices(
  std::span<const std::int64_t> image_stamps_ns, std::int64_t traj_begin_ns,
  std::int64_t traj_end_ns, int samples, std::int64_t margin_ns)
{
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < image_stamps_ns.size(); ++i) {
    if (
      image_stamps_ns[i] >= traj_begin_ns + margin_ns &&
      image_stamps_ns[i] <= traj_end_ns - margin_ns) {
      eligible.push_back(i);
    }
  }
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

std::string default_calibrate_output_path(const std::filesystem::path & input)
{
  return input.stem().string() + "_tf_static_calib.yaml";
}

std::string render_calibrate_summary(
  const TfStaticCalibrateArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const std::string & yaml_path)
{
  std::string out =
    fmt::format("tf static calibrate: {} -> {}\n", args.parent_frame, args.child_frame);
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
    "apply with: bagwiz tf static update -i {} --yaml {}\n", args.input_path, yaml_path);
  return out;
}

std::string render_calibrate_json(
  const TfStaticCalibrateArgs & args, const core::calib::RefineResult & result,
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
