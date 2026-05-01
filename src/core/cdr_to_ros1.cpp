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
#include <cstdio>
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
    // Bytes 0-1 are representation_identifier (encoding kind). The
    // walker reads numeric primitives little-endian, so accept only
    // PLAIN_CDR_LE (`0x00 0x01`). Other identifiers (PLAIN_CDR BE,
    // PL_CDR / PL_CDR_LE parameterised, XCDR variants, ...) need
    // different decode logic; silently treating them as LE would
    // produce garbage for every multibyte field. Both Fast-DDS and
    // Cyclone DDS emit PLAIN_CDR_LE in practice, so this is the only
    // identifier we have ever seen on real rosbag2 payloads.
    const auto rid_hi = static_cast<std::uint8_t>(data_[0]);
    const auto rid_lo = static_cast<std::uint8_t>(data_[1]);
    if (rid_hi != 0x00U || rid_lo != 0x01U) {
      char hex_buf[5];
      std::snprintf(
        hex_buf, sizeof(hex_buf), "%02X%02X", static_cast<unsigned>(rid_hi),
        static_cast<unsigned>(rid_lo));
      throw std::runtime_error(
        std::string("cdr_to_ros1: unsupported CDR encapsulation identifier 0x") + hex_buf +
        " (expected 0x0001 PLAIN_CDR_LE)");
    }
    // Bytes 2-3 are representation_options. Per OMG DDS-XTYPES 1.3
    // §7.6.3.1.2 the lower two bits of the options field encode the
    // number of padding bytes (0-3) appended after the body so the
    // total encapsulated size ends on a 4-byte boundary. We trim that
    // count from the effective body so the walker's "all bytes
    // consumed" check accepts payloads produced by FastDDS / CycloneDDS
    // (and rosbag2 storage plugins that pass the field through).
    // Legacy PLAIN_CDR_LE writers leave options=0, so this is a no-op
    // for them.
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

void walk_message(
  const ts::MessageMembers & m, CdrReader & r, Ros1Writer & w,
  std::vector<TimeOverflowEvent> & overflows);

void walk_scalar(
  const ts::MessageMember & f, CdrReader & r, Ros1Writer & w,
  std::vector<TimeOverflowEvent> & overflows)
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
      walk_message(*sub, r, w, overflows);
      return;
    }

    default:
      throw std::runtime_error(
        "cdr_to_ros1: unsupported field type_id " + std::to_string(static_cast<int>(f.type_id_)) +
        " for field '" + std::string(f.name_ != nullptr ? f.name_ : "?") + "'");
  }
}

void walk_field(
  const ts::MessageMember & f, CdrReader & r, Ros1Writer & w,
  std::vector<TimeOverflowEvent> & overflows)
{
  // Empty-message placeholder: ROS 2 codegen inserts a single
  // `uint8 structure_needs_at_least_one_member` field for messages
  // whose .msg text has zero fields (std_msgs/Empty, std_srvs/Empty,
  // etc.). The CDR wire carries that placeholder byte, but the ROS 1
  // wire format has no equivalent — empty ROS 1 messages serialise to
  // 0 bytes. Skip the placeholder on the CDR side without writing
  // anything to the ROS 1 output.
  if (f.name_ != nullptr && std::strcmp(f.name_, "structure_needs_at_least_one_member") == 0) {
    (void)r.read_bytes(1);
    return;
  }

  if (!f.is_array_) {
    walk_scalar(f, r, w, overflows);
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
    walk_scalar(f, r, w, overflows);
  }
}

// Read one 32-bit field while watching for the sign-flip case described
// in TimeOverflowEvent. Symmetric to ros1_to_cdr's helper: the bytes
// pass through unchanged, we only record the event when the high bit
// is set.
void walk_signflip_u32(
  CdrReader & r, Ros1Writer & w, std::vector<TimeOverflowEvent> & overflows,
  std::string_view type_short, std::string_view field_name)
{
  const uint32_t bits = r.read_u32_le();
  if ((bits & 0x80000000U) != 0U) {
    TimeOverflowEvent ev;
    ev.type = std::string(type_short);
    ev.field = std::string(field_name);
    ev.bits = bits;
    overflows.push_back(std::move(ev));
  }
  w.write_u32_le(bits);
}

void walk_message(
  const ts::MessageMembers & m, CdrReader & r, Ros1Writer & w,
  std::vector<TimeOverflowEvent> & overflows)
{
  // Mirror the forward pre-hook in ros1_to_cdr.cpp: ROS 1's Header
  // begins with a `uint32 seq` field that ROS 2 dropped. Synthesize
  // zero on output rather than maintain a per-connection counter — the
  // original ROS 1 seq is unrecoverable from a ROS 2 bag and most ROS
  // 1 tooling ignores the field.
  const std::string ns = m.message_namespace_ != nullptr ? m.message_namespace_ : "";
  const std::string name = m.message_name_ != nullptr ? m.message_name_ : "";
  if (ns == "std_msgs::msg" && name == "Header") {
    w.write_u32_le(0);
  } else if (ns == "builtin_interfaces::msg" && name == "Time") {
    // ROS 2 `Time` (int32 sec, uint32 nanosec) ⇒ ROS 1 `time` (uint32
    // sec, uint32 nsec). Sec's sign convention differs: a negative
    // ROS 2 sec (timestamp before 1970, or default-constructed 0 with
    // a very small clock drift) reads as a huge unsigned in ROS 1.
    walk_signflip_u32(r, w, overflows, "builtin_interfaces/Time", "sec");
    // nanosec is uint32 on both sides — straight pass-through.
    const auto bytes = r.read_bytes(4);
    w.write_bytes(bytes);
    return;
  } else if (ns == "builtin_interfaces::msg" && name == "Duration") {
    // ROS 2 `Duration` (int32 sec, uint32 nanosec) ⇒ ROS 1 `duration`
    // (int32 sec, int32 nsec). Sec passes through (int32 both sides);
    // nsec switches from uint32 to int32, so flag values > INT32_MAX.
    //
    // Align before reading sec: int32 in CDR is 4-aligned, and entry
    // into a nested message does NOT pre-align. The Time branch above
    // gets the same alignment for free via walk_signflip_u32's internal
    // `r.read_u32_le()` (which calls align(4)), but Duration reads sec
    // as raw bytes, so we have to align explicitly. Without this, a
    // Duration following a 1-3 byte field in the parent message reads
    // sec from the padding instead of the real bytes.
    r.align(4);
    const auto sec_bytes = r.read_bytes(4);
    w.write_bytes(sec_bytes);
    walk_signflip_u32(r, w, overflows, "builtin_interfaces/Duration", "nanosec");
    return;
  }

  for (uint32_t i = 0; i < m.member_count_; ++i) {
    walk_field(m.members_[i], r, w, overflows);
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
    walk_message(*load.members, reader, writer, out.overflows);
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
