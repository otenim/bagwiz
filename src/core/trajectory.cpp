// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstdint>
#include <ios>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace cdr = bagwiz::core::cdr_walker;

const cdr::Object * find_object(const cdr::Value & v) noexcept
{
  return std::get_if<cdr::Object>(&v.v);
}

// cppcheck-suppress passedByValue
const cdr::Value * find_field(const cdr::Object & obj, std::string_view name) noexcept
{
  for (const auto & entry : obj.fields) {
    if (entry.first == name) {
      return &entry.second;
    }
  }
  return nullptr;
}

// Coerce a Value holding either float or double into double. Writers
// ubiquitously use float64 for pose data but the robustness of accepting
// float32 is a cheap guarantee.
bool to_double(const cdr::Value & v, double & out)
{
  if (const auto * d = std::get_if<double>(&v.v)) {
    out = *d;
    return true;
  }
  if (const auto * f = std::get_if<float>(&v.v)) {
    out = static_cast<double>(*f);
    return true;
  }
  return false;
}

bool read_xyz(const cdr::Object & obj, double & x, double & y, double & z)
{
  const auto * fx = find_field(obj, "x");
  const auto * fy = find_field(obj, "y");
  const auto * fz = find_field(obj, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    return false;
  }
  return to_double(*fx, x) && to_double(*fy, y) && to_double(*fz, z);
}

bool read_xyzw(const cdr::Object & obj, double & x, double & y, double & z, double & w)
{
  const auto * fw = find_field(obj, "w");
  if (fw == nullptr || !read_xyz(obj, x, y, z)) {
    return false;
  }
  return to_double(*fw, w);
}

// Look up `parent.<trans_name>` (an Object: Vector3) and `parent.<rot_name>`
// (an Object: Quaternion) and copy their xyz / xyzw into `out`. Returns
// false on any missing or wrong-shaped field.
bool fill_translation_rotation(
  const cdr::Object & parent, const std::string_view & trans_name,
  const std::string_view & rot_name, TrajectoryPose & out)
{
  const auto * trans_v = find_field(parent, trans_name);
  const auto * rot_v = find_field(parent, rot_name);
  if (trans_v == nullptr || rot_v == nullptr) {
    return false;
  }
  const auto * trans_obj = find_object(*trans_v);
  const auto * rot_obj = find_object(*rot_v);
  if (trans_obj == nullptr || rot_obj == nullptr) {
    return false;
  }
  if (!read_xyz(*trans_obj, out.tx, out.ty, out.tz)) {
    return false;
  }
  return read_xyzw(*rot_obj, out.qx, out.qy, out.qz, out.qw);
}

// Locate `header` + its `stamp.{sec, nanosec}` and `frame_id`. On
// success, `timestamp_ns` holds (sec * 1e9 + nanosec) and `frame_id`
// holds the contents of `header.frame_id` (empty when absent or
// non-string). Returns false when there is no scalar `header` Object
// at all — caller treats that as the "unstamped" case.
bool read_header(const cdr::Object & root, std::int64_t & timestamp_ns, std::string & frame_id)
{
  const auto * header_v = find_field(root, "header");
  if (header_v == nullptr) {
    return false;
  }
  const auto * header = find_object(*header_v);
  if (header == nullptr) {
    return false;
  }

  const auto * stamp_v = find_field(*header, "stamp");
  if (stamp_v == nullptr) {
    return false;
  }
  const auto * stamp = find_object(*stamp_v);
  if (stamp == nullptr) {
    return false;
  }

  const auto * sec_v = find_field(*stamp, "sec");
  const auto * nsec_v = find_field(*stamp, "nanosec");
  if (sec_v == nullptr || nsec_v == nullptr) {
    return false;
  }
  // builtin_interfaces/Time uses int32 sec + uint32 nanosec. Accept
  // either signedness for sec because the Python mcap-ros2-support
  // reference inadvertently emits it as uint32, and bags written by
  // tools that follow the Python contract round-trip with that wider
  // type.
  std::int64_t sec_value = 0;
  if (const auto * sec_i32 = std::get_if<std::int32_t>(&sec_v->v)) {
    sec_value = *sec_i32;
  } else if (const auto * sec_u32 = std::get_if<std::uint32_t>(&sec_v->v)) {
    sec_value = *sec_u32;
  } else {
    return false;
  }
  std::int64_t nsec_value = 0;
  if (const auto * nsec_u32 = std::get_if<std::uint32_t>(&nsec_v->v)) {
    nsec_value = *nsec_u32;
  } else if (const auto * nsec_i32 = std::get_if<std::int32_t>(&nsec_v->v)) {
    nsec_value = *nsec_i32;
  } else {
    return false;
  }
  timestamp_ns = sec_value * 1'000'000'000LL + nsec_value;

  if (const auto * fid_v = find_field(*header, "frame_id")) {
    if (const auto * fid = std::get_if<std::string>(&fid_v->v)) {
      frame_id = *fid;
    } else {
      frame_id.clear();
    }
  } else {
    frame_id.clear();
  }
  return true;
}

}  // namespace

std::optional<PoseExtraction> extract_pose(
  const cdr_walker::Value & message, std::int64_t fallback_timestamp_ns)
{
  const auto * root = find_object(message);
  if (root == nullptr) {
    return std::nullopt;
  }

  PoseExtraction out;
  if (read_header(*root, out.pose.timestamp_ns, out.frame_id)) {
    out.used_header_stamp = true;
  } else {
    out.pose.timestamp_ns = fallback_timestamp_ns;
    out.used_header_stamp = false;
    out.frame_id.clear();
  }

  // Optional top-level child_frame_id (Odometry, TransformStamped). Empty
  // for PoseStamped / Pose / Transform which do not carry one.
  if (const auto * cf_v = find_field(*root, "child_frame_id")) {
    if (const auto * cf = std::get_if<std::string>(&cf_v->v)) {
      out.child_frame_id = *cf;
    }
  }

  // TransformStamped-shaped: transform.{translation, rotation}
  if (const auto * trans_v = find_field(*root, "transform")) {
    if (const auto * trans_obj = find_object(*trans_v)) {
      if (fill_translation_rotation(*trans_obj, "translation", "rotation", out.pose)) {
        return out;
      }
    }
  }

  // Bare Transform (no header): translation + rotation at the top level.
  if (find_field(*root, "translation") != nullptr && find_field(*root, "rotation") != nullptr) {
    if (fill_translation_rotation(*root, "translation", "rotation", out.pose)) {
      return out;
    }
  }

  // PoseStamped-shaped (pose.{position, orientation}) or Odometry-shaped
  // (pose.pose.{position, orientation}).
  if (const auto * pose_v = find_field(*root, "pose")) {
    if (const auto * pose_obj = find_object(*pose_v)) {
      if (const auto * inner_v = find_field(*pose_obj, "pose")) {
        if (const auto * inner_obj = find_object(*inner_v)) {
          if (fill_translation_rotation(*inner_obj, "position", "orientation", out.pose)) {
            return out;
          }
        }
      }
      if (fill_translation_rotation(*pose_obj, "position", "orientation", out.pose)) {
        return out;
      }
    }
  }

  // Bare Pose (no header): position + orientation at the top level.
  if (find_field(*root, "position") != nullptr && find_field(*root, "orientation") != nullptr) {
    if (fill_translation_rotation(*root, "position", "orientation", out.pose)) {
      return out;
    }
  }

  return std::nullopt;
}

std::vector<PoseExtraction> extract_pose_candidates(
  const cdr_walker::Value & message, std::int64_t fallback_timestamp_ns)
{
  std::vector<PoseExtraction> out;

  const auto * root = find_object(message);
  if (root == nullptr) {
    return out;
  }

  // tf2_msgs/msg/TFMessage shape: top-level Object with a `transforms`
  // Sequence of TransformStamped. Each contained edge becomes its own
  // candidate so the caller can pick the relevant one (typically the
  // edge that matches the requested --from/--to, or any edge that
  // composes to it through the TF buffer).
  if (const auto * transforms_v = find_field(*root, "transforms")) {
    if (const auto * seq = std::get_if<cdr::Sequence>(&transforms_v->v)) {
      for (const auto & elem : seq->elements) {
        if (auto extraction = extract_pose(elem, fallback_timestamp_ns)) {
          out.push_back(std::move(*extraction));
        }
      }
      return out;
    }
  }

  if (auto extraction = extract_pose(message, fallback_timestamp_ns)) {
    out.push_back(std::move(*extraction));
  }
  return out;
}

void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses)
{
  const auto prev_flags = os.flags();
  const auto prev_prec = os.precision();
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(9);
  for (const auto & p : poses) {
    const double ts = static_cast<double>(p.timestamp_ns) / 1e9;
    os << ts << ' ' << p.tx << ' ' << p.ty << ' ' << p.tz << ' ' << p.qx << ' ' << p.qy << ' '
       << p.qz << ' ' << p.qw << '\n';
  }
  os.flags(prev_flags);
  os.precision(prev_prec);
}

}  // namespace bagwiz::core
