// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstdint>
#include <cstring>
#include <ios>
#include <string>
#include <string_view>

namespace bagwiz::core
{

namespace
{

namespace ts_types = rosidl_typesupport_introspection_cpp;

const ts_types::MessageMember * find_member(
  const ts_types::MessageMembers & members, std::string_view name)
{
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    if (name == members.members_[i].name_) {
      return &members.members_[i];
    }
  }
  return nullptr;
}

bool is_scalar_message(const ts_types::MessageMember * m)
{
  return m != nullptr && m->type_id_ == ts_types::ROS_TYPE_MESSAGE && !m->is_array_;
}

const ts_types::MessageMembers * submembers(const ts_types::MessageMember * m)
{
  return static_cast<const ts_types::MessageMembers *>(m->members_->data);
}

const std::uint8_t * byte_ptr(const void * p)
{
  return static_cast<const std::uint8_t *>(p);
}

// Pull a double out of a member that holds one of the CDR numeric
// primitives and cast. Writers ubiquitously use float64 for pose data but
// the robustness of accepting float32 is a cheap guarantee.
bool read_double(const ts_types::MessageMember & m, const void * base, double & out)
{
  const void * p = byte_ptr(base) + m.offset_;
  switch (m.type_id_) {
    case ts_types::ROS_TYPE_DOUBLE: {
      double v = 0.0;
      std::memcpy(&v, p, sizeof(v));
      out = v;
      return true;
    }
    case ts_types::ROS_TYPE_FLOAT: {
      float v = 0.0F;
      std::memcpy(&v, p, sizeof(v));
      out = static_cast<double>(v);
      return true;
    }
    default:
      return false;
  }
}

bool read_xyz(
  const ts_types::MessageMembers & m, const void * base, double & x, double & y, double & z)
{
  const auto * mx = find_member(m, "x");
  const auto * my = find_member(m, "y");
  const auto * mz = find_member(m, "z");
  if (mx == nullptr || my == nullptr || mz == nullptr) {
    return false;
  }
  return read_double(*mx, base, x) && read_double(*my, base, y) && read_double(*mz, base, z);
}

bool read_xyzw(
  const ts_types::MessageMembers & m, const void * base, double & x, double & y, double & z,
  double & w)
{
  const auto * mw = find_member(m, "w");
  if (!read_xyz(m, base, x, y, z) || mw == nullptr) {
    return false;
  }
  return read_double(*mw, base, w);
}

bool fill_translation_rotation(
  const ts_types::MessageMembers & parent, const void * base, std::string_view trans_name,
  std::string_view rot_name, TrajectoryPose & out)
{
  const auto * trans = find_member(parent, trans_name);
  const auto * rot = find_member(parent, rot_name);
  if (!is_scalar_message(trans) || !is_scalar_message(rot)) {
    return false;
  }
  const void * t_base = byte_ptr(base) + trans->offset_;
  const void * r_base = byte_ptr(base) + rot->offset_;
  if (!read_xyz(*submembers(trans), t_base, out.tx, out.ty, out.tz)) {
    return false;
  }
  if (!read_xyzw(*submembers(rot), r_base, out.qx, out.qy, out.qz, out.qw)) {
    return false;
  }
  return true;
}

// Locate `header` + its `stamp` / `frame_id` sub-members. On success
// `timestamp_ns` holds (sec * 1e9 + nanosec) and `frame_id` holds the
// contents of `header.frame_id` (empty when the field is absent or
// not a string). Returns false when the message has no scalar `header`
// member at all -- the caller treats that as the "unstamped" case.
bool read_header(
  const ts_types::MessageMembers & members, const void * base, std::int64_t & timestamp_ns,
  std::string & frame_id)
{
  const auto * header = find_member(members, "header");
  if (!is_scalar_message(header)) {
    return false;
  }
  const auto & header_mem = *submembers(header);
  const void * header_base = byte_ptr(base) + header->offset_;

  const auto * stamp = find_member(header_mem, "stamp");
  if (!is_scalar_message(stamp)) {
    return false;
  }
  const auto & stamp_mem = *submembers(stamp);
  const void * stamp_base = byte_ptr(header_base) + stamp->offset_;

  const auto * sec = find_member(stamp_mem, "sec");
  const auto * nsec = find_member(stamp_mem, "nanosec");
  if (sec == nullptr || nsec == nullptr) {
    return false;
  }
  if (sec->type_id_ != ts_types::ROS_TYPE_INT32 || nsec->type_id_ != ts_types::ROS_TYPE_UINT32) {
    return false;
  }
  std::int32_t s = 0;
  std::uint32_t n = 0;
  std::memcpy(&s, byte_ptr(stamp_base) + sec->offset_, sizeof(s));
  std::memcpy(&n, byte_ptr(stamp_base) + nsec->offset_, sizeof(n));
  timestamp_ns = static_cast<std::int64_t>(s) * 1'000'000'000LL + static_cast<std::int64_t>(n);

  const auto * fid = find_member(header_mem, "frame_id");
  if (fid != nullptr && !fid->is_array_ && fid->type_id_ == ts_types::ROS_TYPE_STRING) {
    frame_id = *reinterpret_cast<const std::string *>(byte_ptr(header_base) + fid->offset_);
  } else {
    frame_id.clear();
  }
  return true;
}

}  // namespace

std::optional<PoseExtraction> extract_pose(
  const ts_types::MessageMembers & members, const void * base, std::int64_t fallback_timestamp_ns)
{
  PoseExtraction out;
  if (read_header(members, base, out.pose.timestamp_ns, out.frame_id)) {
    out.used_header_stamp = true;
  } else {
    out.pose.timestamp_ns = fallback_timestamp_ns;
    out.used_header_stamp = false;
    out.frame_id.clear();
  }

  // Optional top-level child_frame_id (Odometry, TransformStamped). Empty
  // for PoseStamped / Pose / Transform which do not carry one.
  if (const auto * cf = find_member(members, "child_frame_id");
      cf != nullptr && !cf->is_array_ && cf->type_id_ == ts_types::ROS_TYPE_STRING) {
    out.child_frame_id = *reinterpret_cast<const std::string *>(byte_ptr(base) + cf->offset_);
  }

  // TransformStamped-shaped: transform.{translation, rotation}
  if (const auto * trans = find_member(members, "transform"); is_scalar_message(trans)) {
    const void * t_base = byte_ptr(base) + trans->offset_;
    if (fill_translation_rotation(
          *submembers(trans), t_base, "translation", "rotation", out.pose)) {
      return out;
    }
  }

  // Bare Transform (no header): translation + rotation at the top level.
  if (
    find_member(members, "translation") != nullptr && find_member(members, "rotation") != nullptr) {
    if (fill_translation_rotation(members, base, "translation", "rotation", out.pose)) {
      return out;
    }
  }

  // PoseStamped-shaped (pose.{position, orientation}) or Odometry-shaped
  // (pose.pose.{position, orientation}).
  if (const auto * pose = find_member(members, "pose"); is_scalar_message(pose)) {
    const auto & pose_mem = *submembers(pose);
    const void * pose_base = byte_ptr(base) + pose->offset_;

    if (const auto * inner = find_member(pose_mem, "pose"); is_scalar_message(inner)) {
      const void * inner_base = byte_ptr(pose_base) + inner->offset_;
      if (fill_translation_rotation(
            *submembers(inner), inner_base, "position", "orientation", out.pose)) {
        return out;
      }
    }
    if (fill_translation_rotation(pose_mem, pose_base, "position", "orientation", out.pose)) {
      return out;
    }
  }

  // Bare Pose (no header): position + orientation at the top level.
  if (
    find_member(members, "position") != nullptr && find_member(members, "orientation") != nullptr) {
    if (fill_translation_rotation(members, base, "position", "orientation", out.pose)) {
      return out;
    }
  }

  return std::nullopt;
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
