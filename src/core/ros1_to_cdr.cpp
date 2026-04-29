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

// Override table for ROS 1 → ROS 2 type names that cannot be derived
// mechanically from `pkg/Type` → `pkg/msg/Type`. Almost every ROS message
// type follows the auto-derive rule; the override list exists only for
// historical renames where the package or type name itself changed
// across the ROS 1/ROS 2 boundary.
//
// To add an entry here, the type must satisfy: (a) a different package
// name on each side, or (b) a case/spelling change in the type name, or
// both. Same-package, same-spelling types should NOT appear here — they
// are handled by the auto-derive fallback in `map_ros1_type`.
const std::unordered_map<std::string, std::string> & ros1_to_ros2_renames()
{
  static const std::unordered_map<std::string, std::string> kMap = {
    // The ROS 1 `tf` package was renamed to `tf2_msgs` and the `tfMessage`
    // type was capitalised to `TFMessage`. Both `tf/tfMessage` (legacy)
    // and `tf2_msgs/TFMessage` (modern) appear in the wild on ROS 1
    // bags; the modern form auto-derives correctly to
    // `tf2_msgs/msg/TFMessage`, but the legacy alias needs an explicit
    // override.
    {"tf/tfMessage", "tf2_msgs/msg/TFMessage"},
  };
  return kMap;
}

// Validate that `s` is a non-empty identifier-like token
// (alphanumerics + underscore, no leading digit). ROS message type
// names use this grammar for both package and type segments.
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
  // 1. Look in the rename override table first. These are special cases
  //    where the auto-derive rule would produce the wrong answer (e.g.
  //    `tf/tfMessage` → `tf2_msgs/msg/TFMessage`).
  const auto & overrides = ros1_to_ros2_renames();
  if (auto it = overrides.find(std::string(ros1_type)); it != overrides.end()) {
    return it->second;
  }

  // 2. Auto-derive: ROS 1 type names are `pkg/Type`; the equivalent ROS 2
  //    name is `pkg/msg/Type`. The actual conversion uses the ROS 2 type's
  //    introspection typesupport (`load_introspection`), so only the name
  //    is derived here — whether the type is loadable in the current
  //    environment is checked downstream.
  const auto slash = ros1_type.find('/');
  if (slash == std::string_view::npos) {
    return std::nullopt;
  }
  if (ros1_type.find('/', slash + 1) != std::string_view::npos) {
    // More than one '/' — not a valid ROS 1 type name (and almost
    // certainly an accidental ROS 2 form passed in).
    return std::nullopt;
  }
  const auto pkg = ros1_type.substr(0, slash);
  const auto type = ros1_type.substr(slash + 1);
  if (!is_identifier(pkg) || !is_identifier(type)) {
    return std::nullopt;
  }
  std::string out;
  out.reserve(pkg.size() + 5 + type.size());
  out.append(pkg);
  out.append("/msg/");
  out.append(type);
  return out;
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
