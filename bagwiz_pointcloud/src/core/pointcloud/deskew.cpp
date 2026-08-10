// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/deskew.hpp"

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

namespace bagwiz::core::pointcloud
{

namespace
{

// Mirror of core::interpolate_poses (bagwiz_tf/src/core/tf/trajectory.cpp),
// copied operation-for-operation so the per-point hot loop can inline it and
// so both produce bit-identical results. Any change to that function must be
// mirrored here.
core::TrajectoryPose interpolate_pose_inline(
  const core::TrajectoryPose & a, const core::TrajectoryPose & b, double t)
{
  core::TrajectoryPose out;
  out.timestamp_ns = a.timestamp_ns;
  const double t1 = 1.0 - t;
  out.tx = t1 * a.tx + t * b.tx;
  out.ty = t1 * a.ty + t * b.ty;
  out.tz = t1 * a.tz + t * b.tz;

  // SLERP on the quaternion.
  double dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
  double bx = b.qx;
  double by = b.qy;
  double bz = b.qz;
  double bw = b.qw;
  if (dot < 0.0) {
    dot = -dot;
    bx = -bx;
    by = -by;
    bz = -bz;
    bw = -bw;
  }
  if (dot > 0.9995) {
    out.qx = a.qx + t * (bx - a.qx);
    out.qy = a.qy + t * (by - a.qy);
    out.qz = a.qz + t * (bz - a.qz);
    out.qw = a.qw + t * (bw - a.qw);
  } else {
    const double omega = std::acos(dot);
    const double sin_omega = std::sin(omega);
    const double s0 = std::sin(t1 * omega) / sin_omega;
    const double s1 = std::sin(t * omega) / sin_omega;
    out.qx = s0 * a.qx + s1 * bx;
    out.qy = s0 * a.qy + s1 * by;
    out.qz = s0 * a.qz + s1 * bz;
    out.qw = s0 * a.qw + s1 * bw;
  }
  // Normalize to be safe.
  const double norm =
    std::sqrt(out.qx * out.qx + out.qy * out.qy + out.qz * out.qz + out.qw * out.qw);
  if (norm > 0.0) {
    out.qx /= norm;
    out.qy /= norm;
    out.qz /= norm;
    out.qw /= norm;
  }
  return out;
}

// Minimal double 3x3 rotation + translation machinery. Every function below
// reproduces the exact formula and association order of its tf2::LinearMath
// counterpart (Matrix3x3::setRotation, Matrix3x3 / Transform operator*,
// Transform::inverse), so composing these structs yields bit-identical
// results to composing tf2::Transform objects.
struct Mat3
{
  double m[3][3];
};

struct Vec3
{
  double x, y, z;
};

// tf2::Matrix3x3::setRotation: no quaternion renormalisation.
Mat3 mat_from_quat(double qx, double qy, double qz, double qw)
{
  const double d = qx * qx + qy * qy + qz * qz + qw * qw;
  const double s = 2.0 / d;
  const double xs = qx * s, ys = qy * s, zs = qz * s;
  const double wx = qw * xs, wy = qw * ys, wz = qw * zs;
  const double xx = qx * xs, xy = qx * ys, xz = qx * zs;
  const double yy = qy * ys, yz = qy * zs, zz = qz * zs;
  Mat3 r;
  r.m[0][0] = 1.0 - (yy + zz);
  r.m[0][1] = xy - wz;
  r.m[0][2] = xz + wy;
  r.m[1][0] = xy + wz;
  r.m[1][1] = 1.0 - (xx + zz);
  r.m[1][2] = yz - wx;
  r.m[2][0] = xz - wy;
  r.m[2][1] = yz + wx;
  r.m[2][2] = 1.0 - (xx + yy);
  return r;
}

Mat3 mat_transpose(const Mat3 & a)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r.m[i][j] = a.m[j][i];
    }
  }
  return r;
}

// tf2::Matrix3x3::operator*: row-of-a dotted with column-of-b, k = 0, 1, 2.
Mat3 mat_mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      r.m[i][j] = b.m[0][j] * a.m[i][0] + b.m[1][j] * a.m[i][1] + b.m[2][j] * a.m[i][2];
    }
  }
  return r;
}

// tf2::operator*(Matrix3x3, Vector3): row i dotted with v, left to right.
Vec3 mat_vec(const Mat3 & a, const Vec3 & v)
{
  return {
    a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
    a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
    a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

Vec3 vec_neg(const Vec3 & v)
{
  return {-v.x, -v.y, -v.z};
}

Vec3 vec_add(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

const PointField * find_field(std::span<const PointField> fields, const char * name)
{
  for (const auto & f : fields) {
    if (f.name == name) return &f;
  }
  return nullptr;
}

bool is_float(PointFieldType dt)
{
  return dt == PointFieldType::kFloat32 || dt == PointFieldType::kFloat64;
}

double load_xyz(const std::byte * base, PointFieldType dt)
{
  if (dt == PointFieldType::kFloat32) {
    float v;
    std::memcpy(&v, base, 4);
    return v;
  }
  double v;
  std::memcpy(&v, base, 8);
  return v;
}

void store_xyz(std::byte * base, PointFieldType dt, double val)
{
  if (dt == PointFieldType::kFloat32) {
    float v = static_cast<float>(val);
    std::memcpy(base, &v, 4);
  } else {
    std::memcpy(base, &val, 8);
  }
}

// After deskew every point shares t_ref; write a constant so downstream can't re-deskew.
void write_ref_time(std::byte * base, PointFieldType dt, bool relative, std::int64_t t_ref_ns)
{
  const double abs_sec = static_cast<double>(t_ref_ns) / 1.0e9;
  switch (dt) {
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(base, &v, 4);  // ns-relative -> 0
      break;
    }
    case PointFieldType::kFloat32: {
      float v = relative ? 0.0f : static_cast<float>(abs_sec);
      std::memcpy(base, &v, 4);
      break;
    }
    case PointFieldType::kFloat64: {
      double v = relative ? 0.0 : abs_sec;
      std::memcpy(base, &v, 8);
      break;
    }
    default:
      break;
  }
}

// Field layout resolved and validated for the deskew kernel, shared by the
// struct and CDR entry points so both accept and reject exactly the same
// clouds. `error` non-empty rejects the cloud outright; an unset time_field
// means "no usable per-point time" (including a time field declared out of
// bounds) and the cloud passes through verbatim.
struct KernelLayout
{
  PointField fx, fy, fz;  // copies: valid independently of the source field table
  std::optional<PointTimeField> time_field;
  std::uint32_t rstep = 0;
  std::string error;
};

KernelLayout resolve_kernel_layout(
  bool is_bigendian, std::span<const PointField> fields, std::uint32_t point_step,
  std::uint32_t row_step, std::uint32_t width, std::uint32_t height, std::size_t data_size)
{
  KernelLayout lay;
  if (is_bigendian) {
    lay.error = "big-endian point clouds are not supported";
    return lay;
  }
  const PointField * fx = find_field(fields, "x");
  const PointField * fy = find_field(fields, "y");
  const PointField * fz = find_field(fields, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    lay.error = "cloud is missing one of the x/y/z fields";
    return lay;
  }
  if (!is_float(fx->datatype) || fx->datatype != fy->datatype || fx->datatype != fz->datatype) {
    lay.error = "x/y/z must all be the same float type (FLOAT32 or FLOAT64)";
    return lay;
  }
  if (fx->count != 1 || fy->count != 1 || fz->count != 1) {
    lay.error = "x/y/z count must be 1";
    return lay;
  }
  if (point_step == 0) {
    lay.error = "point_step is zero";
    return lay;
  }
  const auto fits = [&](std::uint32_t offset, PointFieldType dt) {
    return static_cast<std::size_t>(offset) + datatype_size(dt) <= point_step;
  };
  if (
    !fits(fx->offset, fx->datatype) || !fits(fy->offset, fy->datatype) ||
    !fits(fz->offset, fz->datatype)) {
    lay.error = "x/y/z field exceeds point_step";
    return lay;
  }
  lay.rstep = row_step != 0 ? row_step : width * point_step;
  if (static_cast<std::size_t>(width) * point_step > lay.rstep) {
    lay.error = "row_step is smaller than width*point_step";
    return lay;
  }
  if (data_size < static_cast<std::size_t>(height) * lay.rstep) {
    lay.error = "point data buffer too small";
    return lay;
  }
  lay.fx = *fx;
  lay.fy = *fy;
  lay.fz = *fz;

  auto time_field = find_point_time_field(fields);
  if (time_field && !fits(time_field->offset, time_field->datatype)) {
    // find_point_time_field / point_time_seconds do not bounds-check the
    // field against point_step (point_time.hpp: that is the caller's job) --
    // a malformed cloud whose declared time field runs past point_step would
    // otherwise read past its own point and write past it too, corrupting
    // the next point (or, for the last point, the end of the data buffer).
    // Treat it exactly like "no usable time field".
    time_field.reset();
  }
  lay.time_field = time_field;
  return lay;
}

// The reference / extrinsic frame composition, precomputed once per cloud
// (tf2::Transform semantics; see the Mat3/Vec3 note above).
struct FrameComposition
{
  Mat3 r_ri;  // T_ref^{-1} rotation
  Vec3 t_ri;  // T_ref^{-1} translation
  Mat3 r_e;   // E rotation
  Vec3 t_e;   // E translation
  Mat3 r_ei;  // E^{-1} rotation
  Vec3 t_ei;  // E^{-1} translation
};

FrameComposition compose_frames(
  const core::TrajectoryPose & ref_pose,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic)
{
  FrameComposition fc;
  const Mat3 r_ref = mat_from_quat(ref_pose.qx, ref_pose.qy, ref_pose.qz, ref_pose.qw);
  fc.r_ri = mat_transpose(r_ref);
  fc.t_ri = mat_vec(fc.r_ri, vec_neg({ref_pose.tx, ref_pose.ty, ref_pose.tz}));

  fc.r_e = Mat3{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  fc.t_e = Vec3{0.0, 0.0, 0.0};
  if (extrinsic) {
    fc.r_e = mat_from_quat(
      extrinsic->rotation.x, extrinsic->rotation.y, extrinsic->rotation.z, extrinsic->rotation.w);
    fc.t_e = {extrinsic->translation.x, extrinsic->translation.y, extrinsic->translation.z};
  }
  fc.r_ei = mat_transpose(fc.r_e);
  fc.t_ei = mat_vec(fc.r_ei, vec_neg(fc.t_e));
  return fc;
}

// Counters the kernel accumulates; points_total, points_no_pose, and the
// reference clamp flag are the callers' to set.
struct KernelCounters
{
  std::uint64_t deskewed = 0;
  std::uint64_t no_time = 0;
  std::uint64_t nonfinite = 0;
  std::uint64_t out_of_span = 0;
};

// The per-point patch loop -- relative/absolute pre-scan, trajectory cursor,
// transform, in-place store -- shared verbatim between deskew_pointcloud2 and
// deskew_pointcloud2_cdr so both produce bit-identical points.
void run_deskew_kernel(
  std::byte * data, const KernelLayout & lay, std::uint32_t width, std::uint32_t height,
  std::uint32_t point_step, std::int64_t t_ref_ns, std::span<const core::TrajectoryPose> trajectory,
  const FrameComposition & fc, KernelCounters & out)
{
  const PointTimeField & time_field = *lay.time_field;
  const std::uint32_t rstep = lay.rstep;

  // Detect relative vs absolute times (one scan of the time field).
  double max_abs_sec = 0.0;
  for (std::uint32_t r = 0; r < height; ++r) {
    for (std::uint32_t col = 0; col < width; ++col) {
      const std::byte * b = data + static_cast<std::size_t>(r) * rstep +
                            static_cast<std::size_t>(col) * point_step + time_field.offset;
      const double s = point_time_seconds(b, time_field);
      if (std::isfinite(s)) max_abs_sec = std::max(max_abs_sec, std::abs(s));
    }
  }
  const bool relative = max_abs_sec < kRelativeTimeThresholdSec;

  // Cursor over the sorted trajectory, kept at the lower_bound position for
  // the current point time. Point times within a scan are (nearly)
  // non-decreasing, so the position usually advances a few poses per cloud
  // instead of paying a binary search per point; a backwards jump
  // (non-monotone point times) falls back to std::lower_bound. The branch
  // logic below mirrors core::lookup_pose exactly, including endpoint
  // clamping, exact-stamp hits, and interpolation.
  const std::size_t n_poses = trajectory.size();
  std::size_t lo = 0;
  std::int64_t prev_t = std::numeric_limits<std::int64_t>::min();

  for (std::uint32_t r = 0; r < height; ++r) {
    for (std::uint32_t col = 0; col < width; ++col) {
      std::byte * base =
        data + static_cast<std::size_t>(r) * rstep + static_cast<std::size_t>(col) * point_step;
      const double x = load_xyz(base + lay.fx.offset, lay.fx.datatype);
      const double y = load_xyz(base + lay.fy.offset, lay.fy.datatype);
      const double z = load_xyz(base + lay.fz.offset, lay.fz.datatype);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++out.nonfinite;
        continue;
      }
      const double sec = point_time_seconds(base + time_field.offset, time_field);
      if (!std::isfinite(sec)) {
        ++out.no_time;
        continue;
      }
      const std::int64_t t_i_ns =
        relative ? t_ref_ns + static_cast<std::int64_t>(std::llround(sec * 1.0e9))
                 : static_cast<std::int64_t>(std::llround(sec * 1.0e9));

      if (t_i_ns < prev_t) {
        const auto cmp = [](const core::TrajectoryPose & p, std::int64_t t) {
          return p.timestamp_ns < t;
        };
        lo = static_cast<std::size_t>(
          std::lower_bound(trajectory.begin(), trajectory.end(), t_i_ns, cmp) - trajectory.begin());
      } else {
        while (lo < n_poses && trajectory[lo].timestamp_ns < t_i_ns) {
          ++lo;
        }
      }
      prev_t = t_i_ns;

      core::TrajectoryPose pose_i;
      if (lo == n_poses) {
        pose_i = trajectory.back();
        ++out.out_of_span;  // t_i past the last pose: clamped to it
      } else if (trajectory[lo].timestamp_ns == t_i_ns) {
        pose_i = trajectory[lo];
      } else if (lo == 0) {
        pose_i = trajectory.front();
        ++out.out_of_span;  // t_i before the first pose: clamped to it
      } else {
        const auto & prev = trajectory[lo - 1];
        const auto & next = trajectory[lo];
        const double dt = static_cast<double>(next.timestamp_ns - prev.timestamp_ns);
        if (dt <= 0.0) {
          pose_i = prev;
        } else {
          const double t = static_cast<double>(t_i_ns - prev.timestamp_ns) / dt;
          pose_i = interpolate_pose_inline(prev, next, t);
        }
      }

      // rel = E_inv * (T_ref_inv * T_i) * E, composed in the same order as
      // the equivalent tf2::Transform chain.
      const Mat3 r_i = mat_from_quat(pose_i.qx, pose_i.qy, pose_i.qz, pose_i.qw);
      const Vec3 t_i{pose_i.tx, pose_i.ty, pose_i.tz};
      const Mat3 inner_r = mat_mul(fc.r_ri, r_i);
      const Vec3 inner_t = vec_add(mat_vec(fc.r_ri, t_i), fc.t_ri);
      const Mat3 rel2_r = mat_mul(fc.r_ei, inner_r);
      const Vec3 rel2_t = vec_add(mat_vec(fc.r_ei, inner_t), fc.t_ei);
      const Mat3 rel_r = mat_mul(rel2_r, fc.r_e);
      const Vec3 rel_t = vec_add(mat_vec(rel2_r, fc.t_e), rel2_t);
      const Vec3 p = vec_add(mat_vec(rel_r, {x, y, z}), rel_t);

      store_xyz(base + lay.fx.offset, lay.fx.datatype, p.x);
      store_xyz(base + lay.fy.offset, lay.fy.datatype, p.y);
      store_xyz(base + lay.fz.offset, lay.fz.datatype, p.z);
      write_ref_time(base + time_field.offset, time_field.datatype, relative, t_ref_ns);
      ++out.deskewed;
    }
  }
}

}  // namespace

DeskewResult deskew_pointcloud2(
  PointCloud2 input, std::int64_t t_ref_ns, std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic)
{
  DeskewResult out;
  const KernelLayout lay = resolve_kernel_layout(
    input.is_bigendian, input.fields, input.point_step, input.row_step, input.width, input.height,
    input.data.size());
  if (!lay.error.empty()) {
    out.error = lay.error;
    return out;
  }

  out.points_total = static_cast<std::uint64_t>(input.width) * input.height;

  if (!lay.time_field) {
    out.cloud = std::move(input);
    out.points_no_time = out.points_total;
    return out;
  }

  const auto ref_pose = core::lookup_pose(t_ref_ns, trajectory);
  if (!ref_pose) {
    out.cloud = std::move(input);
    out.points_no_pose = out.points_total;
    return out;
  }
  // lookup_pose clamps out-of-span stamps to the endpoint poses; report that
  // on the result so callers can warn, since a clamped reference can silently
  // turn the whole deskew into a no-op.
  out.ref_out_of_span =
    t_ref_ns < trajectory.front().timestamp_ns || t_ref_ns > trajectory.back().timestamp_ns;

  const FrameComposition fc = compose_frames(*ref_pose, extrinsic);
  KernelCounters counters;
  run_deskew_kernel(
    input.data.data(), lay, input.width, input.height, input.point_step, t_ref_ns, trajectory, fc,
    counters);
  out.points_deskewed = counters.deskewed;
  out.points_no_time = counters.no_time;
  out.points_nonfinite = counters.nonfinite;
  out.points_out_of_span = counters.out_of_span;
  out.cloud = std::move(input);
  return out;
}

DeskewCdrResult deskew_pointcloud2_cdr(
  std::span<std::byte> payload, std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic)
{
  DeskewCdrResult out;
  const auto parsed =
    parse_pointcloud2_cdr_layout(std::span<const std::byte>(payload.data(), payload.size()));
  if (!parsed.ok()) {
    out.parse_error = parsed.error;
    return out;
  }
  const PointCloud2CdrLayout & pl = *parsed.layout;
  const PointCloud2Header & h = pl.header;
  out.t_ref_ns = h.timestamp_ns;

  const KernelLayout lay = resolve_kernel_layout(
    h.is_bigendian, h.fields, h.point_step, h.row_step, h.width, h.height, pl.data_size);
  if (!lay.error.empty()) {
    out.error = lay.error;
    return out;
  }

  out.points_total = static_cast<std::uint64_t>(h.width) * h.height;

  if (!lay.time_field) {
    out.points_no_time = out.points_total;
    return out;
  }

  const auto ref_pose = core::lookup_pose(h.timestamp_ns, trajectory);
  if (!ref_pose) {
    out.points_no_pose = out.points_total;
    return out;
  }
  // Same reference-clamp report as deskew_pointcloud2 above.
  out.ref_out_of_span = h.timestamp_ns < trajectory.front().timestamp_ns ||
                        h.timestamp_ns > trajectory.back().timestamp_ns;

  const FrameComposition fc = compose_frames(*ref_pose, extrinsic);
  KernelCounters counters;
  run_deskew_kernel(
    payload.data() + pl.data_offset, lay, h.width, h.height, h.point_step, h.timestamp_ns,
    trajectory, fc, counters);
  out.points_deskewed = counters.deskewed;
  out.points_no_time = counters.no_time;
  out.points_nonfinite = counters.nonfinite;
  out.points_out_of_span = counters.out_of_span;
  return out;
}

}  // namespace bagwiz::core::pointcloud
