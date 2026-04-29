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

#include <cctype>
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

// Override table for ROS 2 → ROS 1 type names that cannot be derived
// mechanically from `pkg/msg/Type` → `pkg/Type`. Currently empty: every
// case in the wild auto-derives correctly, including `tf2_msgs/msg/TFMessage`
// → `tf2_msgs/TFMessage` (the legacy `tf/tfMessage` alias is *not* the
// canonical ROS 1 form chosen here — round-trip from ROS 1 `tf/tfMessage`
// will modernise to `tf2_msgs/TFMessage` by design).
//
// builtin_interfaces is intentionally not overridden: ROS 1 has no
// `builtin_interfaces` package, but `Time` / `Duration` are rarely the
// top-level type of a topic. If they appear, the auto-derive output
// `builtin_interfaces/Time` is a non-standard ROS 1 type name; the
// downstream md5 / message_definition synthesis (decision 10) will
// produce a synthetic .msg the receiver can accept.
const std::unordered_map<std::string, std::string> & ros2_to_ros1_renames()
{
  static const std::unordered_map<std::string, std::string> kMap;
  return kMap;
}

// Validate that `s` is a non-empty identifier-like token
// (alphanumerics + underscore, no leading digit). Mirrors the helper in
// ros1_to_cdr.cpp.
bool is_identifier(std::string_view s)
{
  if (s.empty()) {
    return false;
  }
  if (std::isdigit(static_cast<unsigned char>(s.front())) != 0) {
    return false;
  }
  for (char c : s) {
    const auto u = static_cast<unsigned char>(c);
    if ((std::isalnum(u) == 0) && c != '_') {
      return false;
    }
  }
  return true;
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
    // Bytes 0-1 are representation_identifier (encoding kind); bytes 2-3
    // are representation_options. Per OMG DDS-XTYPES 1.3 §7.6.3.1.2 the
    // lower two bits of the options field encode the number of padding
    // bytes (0-3) appended after the body so the total encapsulated size
    // ends on a 4-byte boundary. We trim that count from the effective
    // body so the walker's "all bytes consumed" check accepts payloads
    // produced by FastDDS / CycloneDDS (and rosbag2 storage plugins that
    // pass the field through). Legacy PLAIN_CDR_LE writers leave
    // options=0, so this is a no-op for them.
    const auto pad = static_cast<std::size_t>(static_cast<std::uint8_t>(data_[3]) & 0x03);
    // Defensive: a malformed tiny payload that happens to have the bits
    // set but no body to trim must not underflow the effective end.
    trailing_pad_ = pad <= data_.size() - 4 ? pad : 0;
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

  bool fully_consumed() const
  {
    if (pos_ + trailing_pad_ == data_.size()) {
      return true;
    }
    // Tolerance for writers that pad the encapsulated body to a 4-byte
    // boundary without setting representation_options (observed on real
    // rosbag2 + rmw_fastrtps payloads, where the header is the literal
    // `00 01 00 00` PLAIN_CDR_LE encoding but the body is followed by
    // 1-3 zero bytes). Foxglove Studio's CDR reader uses the same
    // heuristic: accept up to 3 trailing zero bytes when the total
    // encapsulated size is a multiple of 4.
    if (trailing_pad_ != 0 || data_.size() % 4 != 0) {
      return false;
    }
    const std::size_t leftover = data_.size() - pos_;
    if (leftover > 3) {
      return false;
    }
    for (std::size_t i = pos_; i < data_.size(); ++i) {
      if (static_cast<std::uint8_t>(data_[i]) != 0) {
        return false;
      }
    }
    return true;
  }
  std::size_t remaining() const
  {
    const std::size_t end = data_.size() - trailing_pad_;
    return end > pos_ ? end - pos_ : 0;
  }
  std::size_t position() const { return pos_; }

private:
  void require(std::size_t n) const
  {
    const std::size_t end = data_.size() - trailing_pad_;
    if (end < pos_ || end - pos_ < n) {
      throw std::runtime_error(
        "cdr_to_ros1: CDR payload truncated at offset " + std::to_string(pos_));
    }
  }

  std::span<const std::byte> data_;
  std::size_t body_start_ = 0;
  std::size_t pos_ = 0;
  std::size_t trailing_pad_ = 0;
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
  // 1. Check the rename override table first (currently empty; reserved
  //    for future ROS 2 → ROS 1 renames that don't fit the auto-derive).
  const auto & overrides = ros2_to_ros1_renames();
  if (auto it = overrides.find(std::string(ros2_type)); it != overrides.end()) {
    return it->second;
  }

  // 2. Auto-derive: ROS 2 type names are `pkg/msg/Type`; the equivalent
  //    ROS 1 name is `pkg/Type`. Validate the segment grammar rather
  //    than just searching for `/msg/`, so accidental ROS 1 forms
  //    (`pkg/Type`) or malformed inputs return nullopt instead of
  //    silently passing through.
  constexpr std::string_view kMsgInfix = "/msg/";
  const auto pos = ros2_type.find(kMsgInfix);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  const auto pkg = ros2_type.substr(0, pos);
  const auto type = ros2_type.substr(pos + kMsgInfix.size());
  if (!is_identifier(pkg) || !is_identifier(type)) {
    return std::nullopt;
  }
  // Refuse if there are extra path segments (e.g. `pkg/msg/Sub/Type`).
  if (type.find('/') != std::string_view::npos) {
    return std::nullopt;
  }
  std::string out;
  out.reserve(pkg.size() + 1 + type.size());
  out.append(pkg);
  out.push_back('/');
  out.append(type);
  return out;
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
