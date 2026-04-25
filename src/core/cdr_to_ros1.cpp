// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/cdr_to_ros1.hpp"

#include "bagwiz/core/introspection_loader.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace ts = rosidl_typesupport_introspection_cpp;

// Reverse of ros1_to_ros2_typemap() in ros1_to_cdr.cpp. Multi-key
// collisions on the forward path (e.g. tf/tfMessage and
// tf2_msgs/TFMessage both mapping to tf2_msgs/msg/TFMessage) are
// resolved here by picking the canonical ROS 1 form.
//
// builtin_interfaces is intentionally omitted: ROS 1 has no
// `builtin_interfaces` package, and Time/Duration are wire primitives
// rather than top-level message types in ROS 1.
const std::unordered_map<std::string, std::string> & ros2_to_ros1_typemap()
{
  static const std::unordered_map<std::string, std::string> kMap = {
    {"std_msgs/msg/Bool", "std_msgs/Bool"},
    {"std_msgs/msg/Header", "std_msgs/Header"},
    {"std_msgs/msg/String", "std_msgs/String"},
    {"std_msgs/msg/Float32", "std_msgs/Float32"},
    {"std_msgs/msg/Float64", "std_msgs/Float64"},
    {"std_msgs/msg/Int32", "std_msgs/Int32"},
    {"std_msgs/msg/Int64", "std_msgs/Int64"},
    {"std_msgs/msg/UInt32", "std_msgs/UInt32"},
    {"std_msgs/msg/UInt64", "std_msgs/UInt64"},

    {"geometry_msgs/msg/Vector3", "geometry_msgs/Vector3"},
    {"geometry_msgs/msg/Vector3Stamped", "geometry_msgs/Vector3Stamped"},
    {"geometry_msgs/msg/Point", "geometry_msgs/Point"},
    {"geometry_msgs/msg/PointStamped", "geometry_msgs/PointStamped"},
    {"geometry_msgs/msg/Quaternion", "geometry_msgs/Quaternion"},
    {"geometry_msgs/msg/QuaternionStamped", "geometry_msgs/QuaternionStamped"},
    {"geometry_msgs/msg/Pose", "geometry_msgs/Pose"},
    {"geometry_msgs/msg/PoseStamped", "geometry_msgs/PoseStamped"},
    {"geometry_msgs/msg/PoseWithCovariance", "geometry_msgs/PoseWithCovariance"},
    {"geometry_msgs/msg/PoseWithCovarianceStamped", "geometry_msgs/PoseWithCovarianceStamped"},
    {"geometry_msgs/msg/Transform", "geometry_msgs/Transform"},
    {"geometry_msgs/msg/TransformStamped", "geometry_msgs/TransformStamped"},
    {"geometry_msgs/msg/Twist", "geometry_msgs/Twist"},
    {"geometry_msgs/msg/TwistStamped", "geometry_msgs/TwistStamped"},
    {"geometry_msgs/msg/TwistWithCovariance", "geometry_msgs/TwistWithCovariance"},
    {"geometry_msgs/msg/TwistWithCovarianceStamped", "geometry_msgs/TwistWithCovarianceStamped"},
    {"geometry_msgs/msg/Accel", "geometry_msgs/Accel"},
    {"geometry_msgs/msg/AccelStamped", "geometry_msgs/AccelStamped"},

    // Forward map had two ROS 1 sources for this. The canonical ROS 1
    // form on modern systems is tf2_msgs/TFMessage; tf/tfMessage was
    // the legacy alias used by `tf` (the older package).
    {"tf2_msgs/msg/TFMessage", "tf2_msgs/TFMessage"},

    {"nav_msgs/msg/Odometry", "nav_msgs/Odometry"},
    {"nav_msgs/msg/Path", "nav_msgs/Path"},

    {"sensor_msgs/msg/Imu", "sensor_msgs/Imu"},
    {"sensor_msgs/msg/Image", "sensor_msgs/Image"},
    {"sensor_msgs/msg/CompressedImage", "sensor_msgs/CompressedImage"},
    {"sensor_msgs/msg/CameraInfo", "sensor_msgs/CameraInfo"},
    {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"},
    {"sensor_msgs/msg/PointField", "sensor_msgs/PointField"},
    {"sensor_msgs/msg/NavSatFix", "sensor_msgs/NavSatFix"},
    {"sensor_msgs/msg/NavSatStatus", "sensor_msgs/NavSatStatus"},
    {"sensor_msgs/msg/LaserScan", "sensor_msgs/LaserScan"},
    {"sensor_msgs/msg/Range", "sensor_msgs/Range"},
    {"sensor_msgs/msg/Temperature", "sensor_msgs/Temperature"},
    {"sensor_msgs/msg/FluidPressure", "sensor_msgs/FluidPressure"},
    {"sensor_msgs/msg/MagneticField", "sensor_msgs/MagneticField"},

    {"diagnostic_msgs/msg/DiagnosticArray", "diagnostic_msgs/DiagnosticArray"},
    {"diagnostic_msgs/msg/DiagnosticStatus", "diagnostic_msgs/DiagnosticStatus"},
    {"diagnostic_msgs/msg/KeyValue", "diagnostic_msgs/KeyValue"},
  };
  return kMap;
}

// ---- byte cursors -------------------------------------------------------

// Reads a CDR-LE encapsulated payload. The 4-byte encapsulation header
// (0x00 0x01 0x00 0x00) is consumed at construction; alignment is
// computed relative to the start of the body.
class CdrReader
{
public:
  explicit CdrReader(std::span<const std::byte> data) : data_(data)
  {
    if (data_.size() < 4) {
      throw std::runtime_error("cdr_to_ros1: CDR payload smaller than 4-byte encapsulation header");
    }
    // We don't strictly enforce the encapsulation byte values; rosbag2
    // writers always emit PLAIN_CDR_LE (0x00 0x01) but tolerating the
    // extras helps with hand-crafted test inputs.
    body_start_ = 4;
    pos_ = 4;
  }

  // Skip CDR alignment padding so the next read starts on a multiple of `a`
  // relative to the encapsulated body.
  void align(std::size_t a)
  {
    const std::size_t body_pos = pos_ - body_start_;
    const std::size_t rem = body_pos % a;
    if (rem == 0) {
      return;
    }
    skip(a - rem);
  }

  void skip(std::size_t n)
  {
    require(n);
    pos_ += n;
  }

  uint32_t read_u32_le()
  {
    align(4);
    require(4);
    uint32_t v;
    std::memcpy(&v, data_.data() + pos_, 4);
    pos_ += 4;
    return v;
  }

  std::span<const std::byte> read_bytes(std::size_t n)
  {
    require(n);
    auto r = data_.subspan(pos_, n);
    pos_ += n;
    return r;
  }

  bool fully_consumed() const { return pos_ == data_.size(); }
  std::size_t remaining() const { return data_.size() - pos_; }
  std::size_t position() const { return pos_; }

private:
  void require(std::size_t n) const
  {
    if (data_.size() - pos_ < n) {
      throw std::runtime_error(
        "cdr_to_ros1: CDR payload truncated at offset " + std::to_string(pos_));
    }
  }

  std::span<const std::byte> data_;
  std::size_t body_start_ = 0;
  std::size_t pos_ = 0;
};

// Builds a ROS 1 raw payload. ROS 1 wire format has no alignment
// padding and no encapsulation header; numeric primitives are written
// little-endian back-to-back.
class Ros1Writer
{
public:
  void write_bytes(std::span<const std::byte> bytes)
  {
    buf_.insert(buf_.end(), bytes.begin(), bytes.end());
  }

  void write_u32_le(uint32_t v)
  {
    std::byte tmp[4];
    std::memcpy(tmp, &v, 4);
    buf_.insert(buf_.end(), tmp, tmp + 4);
  }

  std::vector<std::byte> take() && { return std::move(buf_); }

private:
  std::vector<std::byte> buf_;
};

// ---- walker -------------------------------------------------------------

void walk_message(const ts::MessageMembers & m, CdrReader & r, Ros1Writer & w);

void walk_scalar(const ts::MessageMember & f, CdrReader & r, Ros1Writer & w)
{
  switch (f.type_id_) {
    case ts::ROS_TYPE_BOOLEAN:
    case ts::ROS_TYPE_OCTET:
    case ts::ROS_TYPE_UINT8:
    case ts::ROS_TYPE_INT8:
    case ts::ROS_TYPE_CHAR:
      // 1-byte: no alignment in either format.
      w.write_bytes(r.read_bytes(1));
      return;

    case ts::ROS_TYPE_UINT16:
    case ts::ROS_TYPE_INT16:
      r.align(2);
      w.write_bytes(r.read_bytes(2));
      return;

    case ts::ROS_TYPE_UINT32:
    case ts::ROS_TYPE_INT32:
    case ts::ROS_TYPE_FLOAT:
      r.align(4);
      w.write_bytes(r.read_bytes(4));
      return;

    case ts::ROS_TYPE_UINT64:
    case ts::ROS_TYPE_INT64:
    case ts::ROS_TYPE_DOUBLE:
      r.align(8);
      w.write_bytes(r.read_bytes(8));
      return;

    case ts::ROS_TYPE_STRING: {
      // CDR: <u32 (len+1)><len bytes><NUL>. ROS 1: <u32 len><len bytes>.
      const uint32_t cdr_len = r.read_u32_le();
      if (cdr_len == 0) {
        // Defensive: a well-formed CDR string always has at least the
        // trailing NUL counted in the length, but tolerate len==0 by
        // treating it as the empty string.
        w.write_u32_le(0);
        return;
      }
      const uint32_t ros1_len = cdr_len - 1;
      const auto bytes = r.read_bytes(cdr_len);
      // Strip the trailing NUL written by CDR.
      w.write_u32_le(ros1_len);
      w.write_bytes(bytes.subspan(0, ros1_len));
      return;
    }

    case ts::ROS_TYPE_MESSAGE: {
      const auto * sub = static_cast<const ts::MessageMembers *>(f.members_->data);
      walk_message(*sub, r, w);
      return;
    }

    default:
      throw std::runtime_error(
        "cdr_to_ros1: unsupported field type_id " + std::to_string(static_cast<int>(f.type_id_)) +
        " for field '" + std::string(f.name_ != nullptr ? f.name_ : "?") + "'");
  }
}

void walk_field(const ts::MessageMember & f, CdrReader & r, Ros1Writer & w)
{
  if (!f.is_array_) {
    walk_scalar(f, r, w);
    return;
  }

  // Sequence vs fixed-length array: identical encoding rules to the
  // forward direction. Fixed length carries no count on the wire;
  // sequences carry an explicit u32 count in both ROS 1 and CDR.
  uint32_t count = 0;
  if (!f.is_upper_bound_ && f.array_size_ > 0) {
    count = static_cast<uint32_t>(f.array_size_);
  } else {
    count = r.read_u32_le();
    w.write_u32_le(count);
  }

  for (uint32_t i = 0; i < count; ++i) {
    walk_scalar(f, r, w);
  }
}

void walk_message(const ts::MessageMembers & m, CdrReader & r, Ros1Writer & w)
{
  // Mirror the forward pre-hook in ros1_to_cdr.cpp: ROS 1's Header
  // begins with a `uint32 seq` field that ROS 2 dropped. Synthesize
  // zero on output. Per project decision (2026-04-26), we don't carry
  // a per-connection counter — most ROS 1 tooling ignores seq.
  const std::string ns = m.message_namespace_ != nullptr ? m.message_namespace_ : "";
  const std::string name = m.message_name_ != nullptr ? m.message_name_ : "";
  if (ns == "std_msgs::msg" && name == "Header") {
    w.write_u32_le(0);
  }

  for (uint32_t i = 0; i < m.member_count_; ++i) {
    walk_field(m.members_[i], r, w);
  }
}

}  // namespace

std::optional<std::string> map_ros2_type(std::string_view ros2_type)
{
  const auto & m = ros2_to_ros1_typemap();
  auto it = m.find(std::string(ros2_type));
  if (it == m.end()) {
    return std::nullopt;
  }
  return it->second;
}

CdrToRos1Result convert_cdr_to_ros1(
  std::string_view src_ros2_type, std::span<const std::byte> cdr_payload)
{
  CdrToRos1Result out;

  const auto load = load_introspection(src_ros2_type);
  if (!load.ok()) {
    out.error = "introspection load failed for '" + std::string(src_ros2_type) +
                "': " + (load.error.empty() ? "(no detail)" : load.error);
    return out;
  }

  // The CdrReader constructor validates the encapsulation header size,
  // so wrap construction along with the walk to ensure both classes of
  // failure surface as ok=false rather than as an uncaught exception.
  Ros1Writer writer;
  try {
    CdrReader reader(cdr_payload);
    walk_message(*load.members, reader, writer);
    if (!reader.fully_consumed()) {
      out.error = "cdr_to_ros1: CDR payload has " + std::to_string(reader.remaining()) +
                  " trailing byte(s) after decoding type '" + std::string(src_ros2_type) + "'";
      return out;
    }
  } catch (const std::exception & e) {
    out.error = e.what();
    return out;
  }

  out.ros1 = std::move(writer).take();
  out.ok = true;
  return out;
}

}  // namespace bagwiz::core
