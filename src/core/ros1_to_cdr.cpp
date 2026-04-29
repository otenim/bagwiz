// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_to_cdr.hpp"

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

// Whitelist of ROS 1 type names we are willing to convert, mapped to
// the ROS 2 type name that goes into the output bag's topic metadata.
// Entries are added one at a time as they are validated end-to-end;
// untested types stay out of the table on purpose so non-standard bags
// fail loud rather than producing silently broken output.
const std::unordered_map<std::string, std::string> & ros1_to_ros2_typemap()
{
  static const std::unordered_map<std::string, std::string> kMap = {
    {"std_msgs/Bool", "std_msgs/msg/Bool"},
    {"std_msgs/Header", "std_msgs/msg/Header"},
    {"std_msgs/String", "std_msgs/msg/String"},
    {"std_msgs/Float32", "std_msgs/msg/Float32"},
    {"std_msgs/Float64", "std_msgs/msg/Float64"},
    {"std_msgs/Int32", "std_msgs/msg/Int32"},
    {"std_msgs/Int64", "std_msgs/msg/Int64"},
    {"std_msgs/UInt32", "std_msgs/msg/UInt32"},
    {"std_msgs/UInt64", "std_msgs/msg/UInt64"},

    {"geometry_msgs/Vector3", "geometry_msgs/msg/Vector3"},
    {"geometry_msgs/Vector3Stamped", "geometry_msgs/msg/Vector3Stamped"},
    {"geometry_msgs/Point", "geometry_msgs/msg/Point"},
    {"geometry_msgs/PointStamped", "geometry_msgs/msg/PointStamped"},
    {"geometry_msgs/Quaternion", "geometry_msgs/msg/Quaternion"},
    {"geometry_msgs/QuaternionStamped", "geometry_msgs/msg/QuaternionStamped"},
    {"geometry_msgs/Pose", "geometry_msgs/msg/Pose"},
    {"geometry_msgs/PoseStamped", "geometry_msgs/msg/PoseStamped"},
    {"geometry_msgs/PoseWithCovariance", "geometry_msgs/msg/PoseWithCovariance"},
    {"geometry_msgs/PoseWithCovarianceStamped", "geometry_msgs/msg/PoseWithCovarianceStamped"},
    {"geometry_msgs/Transform", "geometry_msgs/msg/Transform"},
    {"geometry_msgs/TransformStamped", "geometry_msgs/msg/TransformStamped"},
    {"geometry_msgs/Twist", "geometry_msgs/msg/Twist"},
    {"geometry_msgs/TwistStamped", "geometry_msgs/msg/TwistStamped"},
    {"geometry_msgs/TwistWithCovariance", "geometry_msgs/msg/TwistWithCovariance"},
    {"geometry_msgs/TwistWithCovarianceStamped", "geometry_msgs/msg/TwistWithCovarianceStamped"},
    {"geometry_msgs/Accel", "geometry_msgs/msg/Accel"},
    {"geometry_msgs/AccelStamped", "geometry_msgs/msg/AccelStamped"},

    {"tf/tfMessage", "tf2_msgs/msg/TFMessage"},
    {"tf2_msgs/TFMessage", "tf2_msgs/msg/TFMessage"},

    {"nav_msgs/Odometry", "nav_msgs/msg/Odometry"},
    {"nav_msgs/Path", "nav_msgs/msg/Path"},

    {"sensor_msgs/Imu", "sensor_msgs/msg/Imu"},
    {"sensor_msgs/Image", "sensor_msgs/msg/Image"},
    {"sensor_msgs/CompressedImage", "sensor_msgs/msg/CompressedImage"},
    {"sensor_msgs/CameraInfo", "sensor_msgs/msg/CameraInfo"},
    {"sensor_msgs/PointCloud2", "sensor_msgs/msg/PointCloud2"},
    {"sensor_msgs/PointField", "sensor_msgs/msg/PointField"},
    {"sensor_msgs/NavSatFix", "sensor_msgs/msg/NavSatFix"},
    {"sensor_msgs/NavSatStatus", "sensor_msgs/msg/NavSatStatus"},
    {"sensor_msgs/LaserScan", "sensor_msgs/msg/LaserScan"},
    {"sensor_msgs/Range", "sensor_msgs/msg/Range"},
    {"sensor_msgs/Temperature", "sensor_msgs/msg/Temperature"},
    {"sensor_msgs/FluidPressure", "sensor_msgs/msg/FluidPressure"},
    {"sensor_msgs/MagneticField", "sensor_msgs/msg/MagneticField"},

    {"diagnostic_msgs/DiagnosticArray", "diagnostic_msgs/msg/DiagnosticArray"},
    {"diagnostic_msgs/DiagnosticStatus", "diagnostic_msgs/msg/DiagnosticStatus"},
    {"diagnostic_msgs/KeyValue", "diagnostic_msgs/msg/KeyValue"},

    // can_msgs is from ros-industrial/ros_canopen and is wire-compatible
    // between ROS 1 and ROS 2 (the only structural difference is the
    // ROS 1 Header.seq prefix, which the Header pre-hook already
    // handles for every message that embeds Header).
    {"can_msgs/Frame", "can_msgs/msg/Frame"},

    {"builtin_interfaces/Time", "builtin_interfaces/msg/Time"},
    {"builtin_interfaces/Duration", "builtin_interfaces/msg/Duration"},
  };
  return kMap;
}

// ---- byte cursors -------------------------------------------------------

class Ros1Reader
{
public:
  explicit Ros1Reader(std::span<const std::byte> data) : data_(data) {}

  void skip(std::size_t n)
  {
    require(n);
    pos_ += n;
  }

  uint32_t read_u32_le()
  {
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
  std::size_t position() const { return pos_; }
  std::size_t remaining() const { return data_.size() - pos_; }

private:
  void require(std::size_t n) const
  {
    if (data_.size() - pos_ < n) {
      throw std::runtime_error(
        "ros1_to_cdr: ROS 1 payload truncated at offset " + std::to_string(pos_));
    }
  }

  std::span<const std::byte> data_;
  std::size_t pos_ = 0;
};

class CdrWriter
{
public:
  CdrWriter()
  {
    // OMG CDR Encapsulation header. 0x00 0x01 = PLAIN_CDR_LE; the
    // remaining two bytes are representation_options, which ROS 2
    // leaves at zero.
    buf_.push_back(std::byte{0x00});
    buf_.push_back(std::byte{0x01});
    buf_.push_back(std::byte{0x00});
    buf_.push_back(std::byte{0x00});
  }

  // Pad up to the alignment relative to the start of the encapsulated
  // body (i.e. ignoring the 4-byte header).
  void align(std::size_t a)
  {
    const std::size_t pos = body_pos();
    const std::size_t rem = pos % a;
    if (rem == 0) {
      return;
    }
    const std::size_t pad = a - rem;
    for (std::size_t i = 0; i < pad; ++i) {
      buf_.push_back(std::byte{0});
    }
  }

  void write_bytes(std::span<const std::byte> bytes)
  {
    buf_.insert(buf_.end(), bytes.begin(), bytes.end());
  }

  void write_u32_le(uint32_t v)
  {
    align(4);
    std::byte tmp[4];
    std::memcpy(tmp, &v, 4);
    buf_.insert(buf_.end(), tmp, tmp + 4);
  }

  void write_byte(std::byte b) { buf_.push_back(b); }

  std::vector<std::byte> take() && { return std::move(buf_); }

private:
  std::size_t body_pos() const { return buf_.size() - 4; }

  std::vector<std::byte> buf_;
};

// ---- walker -------------------------------------------------------------

void walk_message(const ts::MessageMembers & m, Ros1Reader & r, CdrWriter & w);

void walk_scalar(const ts::MessageMember & f, Ros1Reader & r, CdrWriter & w)
{
  switch (f.type_id_) {
    case ts::ROS_TYPE_BOOLEAN:
    case ts::ROS_TYPE_OCTET:
    case ts::ROS_TYPE_UINT8:
    case ts::ROS_TYPE_INT8:
    case ts::ROS_TYPE_CHAR:
      // 1-byte: no alignment needed.
      w.write_bytes(r.read_bytes(1));
      return;

    case ts::ROS_TYPE_UINT16:
    case ts::ROS_TYPE_INT16:
      w.align(2);
      w.write_bytes(r.read_bytes(2));
      return;

    case ts::ROS_TYPE_UINT32:
    case ts::ROS_TYPE_INT32:
    case ts::ROS_TYPE_FLOAT:
      w.align(4);
      w.write_bytes(r.read_bytes(4));
      return;

    case ts::ROS_TYPE_UINT64:
    case ts::ROS_TYPE_INT64:
    case ts::ROS_TYPE_DOUBLE:
      w.align(8);
      w.write_bytes(r.read_bytes(8));
      return;

    case ts::ROS_TYPE_STRING: {
      // ROS 1: <u32 len><len bytes>. CDR: <u32 (len+1)><len bytes><NUL>.
      const uint32_t len = r.read_u32_le();
      const auto bytes = r.read_bytes(len);
      w.write_u32_le(len + 1);
      w.write_bytes(bytes);
      w.write_byte(std::byte{0});
      return;
    }

    case ts::ROS_TYPE_MESSAGE: {
      const auto * sub = static_cast<const ts::MessageMembers *>(f.members_->data);
      walk_message(*sub, r, w);
      return;
    }

    default:
      throw std::runtime_error(
        "ros1_to_cdr: unsupported field type_id " + std::to_string(static_cast<int>(f.type_id_)) +
        " for field '" + std::string(f.name_ != nullptr ? f.name_ : "?") + "'");
  }
}

void walk_field(const ts::MessageMember & f, Ros1Reader & r, CdrWriter & w)
{
  if (!f.is_array_) {
    walk_scalar(f, r, w);
    return;
  }

  // Sequence vs fixed-length array: rosidl encodes a fixed-length array
  // as (is_upper_bound_=false, array_size_>0). Anything else (bounded
  // sequence, unbounded sequence) carries an explicit u32 element count
  // in both ROS 1 raw and CDR.
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

void walk_message(const ts::MessageMembers & m, Ros1Reader & r, CdrWriter & w)
{
  // Pre-hooks for ROS 1 vs ROS 2 schema differences. The list is short
  // by design — only Header drops a wire field across the version
  // boundary; everything else is bit-for-bit identical.
  const std::string ns = m.message_namespace_ != nullptr ? m.message_namespace_ : "";
  const std::string name = m.message_name_ != nullptr ? m.message_name_ : "";
  if (ns == "std_msgs::msg" && name == "Header") {
    // ROS 1 Header begins with `uint32 seq`, dropped in ROS 2.
    r.skip(4);
  }

  for (uint32_t i = 0; i < m.member_count_; ++i) {
    walk_field(m.members_[i], r, w);
  }
}

}  // namespace

std::optional<std::string> map_ros1_type(std::string_view ros1_type)
{
  const auto & m = ros1_to_ros2_typemap();
  auto it = m.find(std::string(ros1_type));
  if (it == m.end()) {
    return std::nullopt;
  }
  return it->second;
}

Ros1ToCdrResult convert_ros1_to_cdr(
  std::string_view dest_ros2_type, std::span<const std::byte> ros1_payload)
{
  Ros1ToCdrResult out;

  const auto load = load_introspection(dest_ros2_type);
  if (!load.ok()) {
    out.error = "introspection load failed for '" + std::string(dest_ros2_type) +
                "': " + (load.error.empty() ? "(no detail)" : load.error);
    return out;
  }

  Ros1Reader reader(ros1_payload);
  CdrWriter writer;

  try {
    walk_message(*load.members, reader, writer);
  } catch (const std::exception & e) {
    out.error = e.what();
    return out;
  }

  if (!reader.fully_consumed()) {
    // Trailing bytes on the input usually means the source schema does
    // not match what we are decoding against; refuse rather than write
    // a half-decoded CDR payload.
    out.error = "ros1_to_cdr: ROS 1 payload has " + std::to_string(reader.remaining()) +
                " trailing byte(s) after decoding type '" + std::string(dest_ros2_type) + "'";
    return out;
  }

  out.cdr = std::move(writer).take();
  out.ok = true;
  return out;
}

}  // namespace bagwiz::core
