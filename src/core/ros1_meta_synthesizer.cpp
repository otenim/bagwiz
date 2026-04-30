// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/ros1_meta_synthesizer.hpp"

#include "bagwiz/core/msg_schema/parser.hpp"
#include "bagwiz/core/msg_schema/schema_model.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace ms = msg_schema;

// ---------------------------------------------------------------------
// MD5 (RFC 1321) — self-contained implementation. Avoids pulling in
// OpenSSL or another dependency for one algorithm. The output format
// matches `md5(text).hexdigest()` from the Python reference and the
// `md5sum` command.
// ---------------------------------------------------------------------

class Md5
{
public:
  Md5() { reset(); }

  void update(const void * data, std::size_t len)
  {
    const auto * bytes = static_cast<const std::uint8_t *>(data);
    std::size_t buf_off = static_cast<std::size_t>(count_ & 0x3F);
    count_ += static_cast<std::uint64_t>(len);

    if (buf_off != 0U) {
      const std::size_t need = 64 - buf_off;
      if (len < need) {
        std::memcpy(buffer_.data() + buf_off, bytes, len);
        return;
      }
      std::memcpy(buffer_.data() + buf_off, bytes, need);
      transform(buffer_.data());
      bytes += need;
      len -= need;
    }
    while (len >= 64U) {
      transform(bytes);
      bytes += 64;
      len -= 64;
    }
    if (len > 0U) {
      std::memcpy(buffer_.data(), bytes, len);
    }
  }

  std::string hexdigest()
  {
    // Append the standard MD5 padding: 0x80, then zeros, then the
    // 64-bit message length in bits (little-endian).
    std::array<std::uint8_t, 8> length_le{};
    const std::uint64_t bit_count = count_ * 8U;
    for (int i = 0; i < 8; ++i) {
      length_le[static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((bit_count >> (i * 8)) & 0xFFU);
    }

    // Build the full padding + length suffix in a single contiguous
    // buffer so we hand `update()` array-shaped pointers, not a
    // pointer-to-single-byte that confuses cppcheck's lifetime analysis.
    // Padding length is 1..64 bytes (0x80 + zeros) such that the total
    // message length ends on a 56-byte boundary, then 8 bytes of length.
    const std::size_t buf_off = static_cast<std::size_t>(count_ & 0x3FU);
    const std::size_t pad_len = (buf_off < 56U) ? (56U - buf_off) : (120U - buf_off);
    std::array<std::uint8_t, 72> tail{};  // worst case: 64 pad + 8 length
    tail[0] = 0x80U;                      // remaining bytes of `tail` are already zero-initialised
    std::memcpy(tail.data() + pad_len, length_le.data(), 8U);
    update(tail.data(), pad_len + 8U);

    std::array<std::uint8_t, 16> digest{};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        digest[static_cast<std::size_t>(i * 4 + j)] =
          static_cast<std::uint8_t>((state_[static_cast<std::size_t>(i)] >> (j * 8)) & 0xFFU);
      }
    }

    static constexpr char kHex[] = "0123456789abcdef";
    std::string out(32, '0');
    for (int i = 0; i < 16; ++i) {
      out[static_cast<std::size_t>(2 * i + 0)] =
        kHex[(digest[static_cast<std::size_t>(i)] >> 4) & 0x0FU];
      out[static_cast<std::size_t>(2 * i + 1)] = kHex[digest[static_cast<std::size_t>(i)] & 0x0FU];
    }
    return out;
  }

private:
  void reset()
  {
    state_[0] = 0x67452301U;
    state_[1] = 0xefcdab89U;
    state_[2] = 0x98badcfeU;
    state_[3] = 0x10325476U;
    count_ = 0U;
    buffer_.fill(0);
  }

  static std::uint32_t f(std::uint32_t x, std::uint32_t y, std::uint32_t z)
  {
    return (x & y) | ((~x) & z);
  }
  static std::uint32_t g(std::uint32_t x, std::uint32_t y, std::uint32_t z)
  {
    return (x & z) | (y & (~z));
  }
  static std::uint32_t h(std::uint32_t x, std::uint32_t y, std::uint32_t z) { return x ^ y ^ z; }
  static std::uint32_t i_fn(std::uint32_t x, std::uint32_t y, std::uint32_t z)
  {
    return y ^ (x | (~z));
  }
  static std::uint32_t rotl(std::uint32_t x, std::uint32_t n)
  {
    return (x << n) | (x >> (32U - n));
  }

  void transform(const std::uint8_t * block)
  {
    std::array<std::uint32_t, 16> m{};
    for (int j = 0; j < 16; ++j) {
      m[static_cast<std::size_t>(j)] = (static_cast<std::uint32_t>(block[j * 4 + 0]) << 0) |
                                       (static_cast<std::uint32_t>(block[j * 4 + 1]) << 8) |
                                       (static_cast<std::uint32_t>(block[j * 4 + 2]) << 16) |
                                       (static_cast<std::uint32_t>(block[j * 4 + 3]) << 24);
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];

    auto step = [](
                  std::uint32_t & av, std::uint32_t bv, std::uint32_t /*cv*/, std::uint32_t /*dv*/,
                  std::uint32_t func, std::uint32_t mv, std::uint32_t s,
                  std::uint32_t t) { av = bv + rotl(av + func + mv + t, s); };

    // Round 1
    step(a, b, c, d, f(b, c, d), m[0], 7, 0xd76aa478U);
    step(d, a, b, c, f(a, b, c), m[1], 12, 0xe8c7b756U);
    step(c, d, a, b, f(d, a, b), m[2], 17, 0x242070dbU);
    step(b, c, d, a, f(c, d, a), m[3], 22, 0xc1bdceeeU);
    step(a, b, c, d, f(b, c, d), m[4], 7, 0xf57c0fafU);
    step(d, a, b, c, f(a, b, c), m[5], 12, 0x4787c62aU);
    step(c, d, a, b, f(d, a, b), m[6], 17, 0xa8304613U);
    step(b, c, d, a, f(c, d, a), m[7], 22, 0xfd469501U);
    step(a, b, c, d, f(b, c, d), m[8], 7, 0x698098d8U);
    step(d, a, b, c, f(a, b, c), m[9], 12, 0x8b44f7afU);
    step(c, d, a, b, f(d, a, b), m[10], 17, 0xffff5bb1U);
    step(b, c, d, a, f(c, d, a), m[11], 22, 0x895cd7beU);
    step(a, b, c, d, f(b, c, d), m[12], 7, 0x6b901122U);
    step(d, a, b, c, f(a, b, c), m[13], 12, 0xfd987193U);
    step(c, d, a, b, f(d, a, b), m[14], 17, 0xa679438eU);
    step(b, c, d, a, f(c, d, a), m[15], 22, 0x49b40821U);

    // Round 2
    step(a, b, c, d, g(b, c, d), m[1], 5, 0xf61e2562U);
    step(d, a, b, c, g(a, b, c), m[6], 9, 0xc040b340U);
    step(c, d, a, b, g(d, a, b), m[11], 14, 0x265e5a51U);
    step(b, c, d, a, g(c, d, a), m[0], 20, 0xe9b6c7aaU);
    step(a, b, c, d, g(b, c, d), m[5], 5, 0xd62f105dU);
    step(d, a, b, c, g(a, b, c), m[10], 9, 0x02441453U);
    step(c, d, a, b, g(d, a, b), m[15], 14, 0xd8a1e681U);
    step(b, c, d, a, g(c, d, a), m[4], 20, 0xe7d3fbc8U);
    step(a, b, c, d, g(b, c, d), m[9], 5, 0x21e1cde6U);
    step(d, a, b, c, g(a, b, c), m[14], 9, 0xc33707d6U);
    step(c, d, a, b, g(d, a, b), m[3], 14, 0xf4d50d87U);
    step(b, c, d, a, g(c, d, a), m[8], 20, 0x455a14edU);
    step(a, b, c, d, g(b, c, d), m[13], 5, 0xa9e3e905U);
    step(d, a, b, c, g(a, b, c), m[2], 9, 0xfcefa3f8U);
    step(c, d, a, b, g(d, a, b), m[7], 14, 0x676f02d9U);
    step(b, c, d, a, g(c, d, a), m[12], 20, 0x8d2a4c8aU);

    // Round 3
    step(a, b, c, d, h(b, c, d), m[5], 4, 0xfffa3942U);
    step(d, a, b, c, h(a, b, c), m[8], 11, 0x8771f681U);
    step(c, d, a, b, h(d, a, b), m[11], 16, 0x6d9d6122U);
    step(b, c, d, a, h(c, d, a), m[14], 23, 0xfde5380cU);
    step(a, b, c, d, h(b, c, d), m[1], 4, 0xa4beea44U);
    step(d, a, b, c, h(a, b, c), m[4], 11, 0x4bdecfa9U);
    step(c, d, a, b, h(d, a, b), m[7], 16, 0xf6bb4b60U);
    step(b, c, d, a, h(c, d, a), m[10], 23, 0xbebfbc70U);
    step(a, b, c, d, h(b, c, d), m[13], 4, 0x289b7ec6U);
    step(d, a, b, c, h(a, b, c), m[0], 11, 0xeaa127faU);
    step(c, d, a, b, h(d, a, b), m[3], 16, 0xd4ef3085U);
    step(b, c, d, a, h(c, d, a), m[6], 23, 0x04881d05U);
    step(a, b, c, d, h(b, c, d), m[9], 4, 0xd9d4d039U);
    step(d, a, b, c, h(a, b, c), m[12], 11, 0xe6db99e5U);
    step(c, d, a, b, h(d, a, b), m[15], 16, 0x1fa27cf8U);
    step(b, c, d, a, h(c, d, a), m[2], 23, 0xc4ac5665U);

    // Round 4
    step(a, b, c, d, i_fn(b, c, d), m[0], 6, 0xf4292244U);
    step(d, a, b, c, i_fn(a, b, c), m[7], 10, 0x432aff97U);
    step(c, d, a, b, i_fn(d, a, b), m[14], 15, 0xab9423a7U);
    step(b, c, d, a, i_fn(c, d, a), m[5], 21, 0xfc93a039U);
    step(a, b, c, d, i_fn(b, c, d), m[12], 6, 0x655b59c3U);
    step(d, a, b, c, i_fn(a, b, c), m[3], 10, 0x8f0ccc92U);
    step(c, d, a, b, i_fn(d, a, b), m[10], 15, 0xffeff47dU);
    step(b, c, d, a, i_fn(c, d, a), m[1], 21, 0x85845dd1U);
    step(a, b, c, d, i_fn(b, c, d), m[8], 6, 0x6fa87e4fU);
    step(d, a, b, c, i_fn(a, b, c), m[15], 10, 0xfe2ce6e0U);
    step(c, d, a, b, i_fn(d, a, b), m[6], 15, 0xa3014314U);
    step(b, c, d, a, i_fn(c, d, a), m[13], 21, 0x4e0811a1U);
    step(a, b, c, d, i_fn(b, c, d), m[4], 6, 0xf7537e82U);
    step(d, a, b, c, i_fn(a, b, c), m[11], 10, 0xbd3af235U);
    step(c, d, a, b, i_fn(d, a, b), m[2], 15, 0x2ad7d2bbU);
    step(b, c, d, a, i_fn(c, d, a), m[9], 21, 0xeb86d391U);

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
  }

  std::array<std::uint32_t, 4> state_{};
  std::uint64_t count_{};
  std::array<std::uint8_t, 64> buffer_{};
};

std::string md5_hex(std::string_view text)
{
  Md5 m;
  m.update(text.data(), text.size());
  return m.hexdigest();
}

// ---------------------------------------------------------------------
// ROS 1 type-name helpers
// ---------------------------------------------------------------------

std::string_view primitive_to_ros1_name(ms::PrimitiveKind k)
{
  switch (k) {
    case ms::PrimitiveKind::Bool:
      return "bool";
    case ms::PrimitiveKind::Byte:
      return "byte";
    case ms::PrimitiveKind::Char:
      return "char";
    case ms::PrimitiveKind::Float32:
      return "float32";
    case ms::PrimitiveKind::Float64:
      return "float64";
    case ms::PrimitiveKind::LongDouble:
      return "float128";  // unofficial; ROS 1 has no official long-double
    case ms::PrimitiveKind::Int8:
      return "int8";
    case ms::PrimitiveKind::Uint8:
      return "uint8";
    case ms::PrimitiveKind::Int16:
      return "int16";
    case ms::PrimitiveKind::Uint16:
      return "uint16";
    case ms::PrimitiveKind::Int32:
      return "int32";
    case ms::PrimitiveKind::Uint32:
      return "uint32";
    case ms::PrimitiveKind::Int64:
      return "int64";
    case ms::PrimitiveKind::Uint64:
      return "uint64";
    case ms::PrimitiveKind::String:
      return "string";
    case ms::PrimitiveKind::Wstring:
      return "wstring";
  }
  return "";
}

std::string array_suffix(const ms::ArraySpec & a)
{
  switch (a.kind) {
    case ms::ArrayKind::Scalar:
      return "";
    case ms::ArrayKind::FixedArray:
      return "[" + std::to_string(a.size.value_or(0)) + "]";
    case ms::ArrayKind::BoundedSequence:
    case ms::ArrayKind::UnboundedSequence:
      // ROS 1 has no bounded sequences, so the upper bound is dropped
      // (the wire format is identical to an unbounded sequence in CDR).
      // Caller is expected to log the drop as a warning.
      return "[]";
  }
  return "";
}

// cppcheck-suppress passedByValue ; std::string_view is a view, pass by value
bool is_header_short(std::string_view short_name)
{
  return short_name == "std_msgs/Header";
}

// cppcheck-suppress passedByValue ; std::string_view is a view, pass by value
bool is_builtin_interfaces_alias(std::string_view short_name)
{
  return short_name == "builtin_interfaces/Time" || short_name == "builtin_interfaces/Duration";
}

// cppcheck-suppress passedByValue ; std::string_view is a view, pass by value
std::string_view builtin_alias_to_ros1(std::string_view short_name)
{
  if (short_name == "builtin_interfaces/Time") {
    return "time";
  }
  if (short_name == "builtin_interfaces/Duration") {
    return "duration";
  }
  return "";
}

// Always returns the short ("pkg/Type") form — strips "/msg/" if present.
std::string to_short_name(std::string_view canonical_or_short)
{
  constexpr std::string_view kInfix = "/msg/";
  const auto pos = canonical_or_short.find(kInfix);
  if (pos == std::string_view::npos) {
    return std::string(canonical_or_short);
  }
  std::string out;
  out.reserve(canonical_or_short.size() - kInfix.size() + 1);
  out.append(canonical_or_short.substr(0, pos));
  out.push_back('/');
  out.append(canonical_or_short.substr(pos + kInfix.size()));
  return out;
}

// ---------------------------------------------------------------------
// Synthesizer driver
// ---------------------------------------------------------------------

constexpr std::string_view kSeparator =
  "================================================================================";

class Synthesizer
{
public:
  // cppcheck-suppress passedByValue ; std::string_view is a view, pass by value
  Synthesizer(const ms::SchemaModel & schema, std::string_view root_short)
  : schema_(schema), root_short_(root_short)
  {
  }

  Ros1MetaResult run() &&
  {
    Ros1MetaResult result;

    // Prime the MD5 cache for the root, which transitively forces every
    // referenced type to be hashed and added to dep_order_.
    try {
      const std::string root_md5 = compute_md5(root_short_);
      result.meta.md5sum = root_md5;
    } catch (const std::exception & e) {
      result.error = e.what();
      return result;
    }

    // Build message_definition: root body, then each dep in DFS order.
    std::string body = emit_normalized_body(root_short_);
    for (const auto & dep : dep_order_) {
      body.append("\n");
      body.append(kSeparator);
      body.append("\nMSG: ");
      body.append(dep);
      body.append("\n");
      body.append(emit_normalized_body(dep));
    }
    result.meta.message_definition = std::move(body);
    result.warnings = std::move(warnings_);
    result.ok = true;
    return result;
  }

private:
  // Recursive MD5: caches by short name, throws on missing type or
  // wstring (the only field type the synthesizer refuses outright;
  // every other ROS 2-only construct degrades to its closest ROS 1
  // form).
  std::string compute_md5(std::string_view short_name)
  {
    if (auto it = md5_cache_.find(std::string(short_name)); it != md5_cache_.end()) {
      return it->second;
    }

    const ms::MessageDef * def = schema_.find(short_name);
    if (def == nullptr) {
      throw std::runtime_error(
        "ros1_meta: nested type not found in schema: " + std::string(short_name));
    }

    std::string text = build_md5_text(*def, short_name);
    std::string hex = md5_hex(text);
    md5_cache_.emplace(std::string(short_name), hex);

    // Track DFS order for the dep listing in message_definition. Skip
    // root (callers emit it separately) and skip builtin_interfaces
    // (ROS 1 has no such package).
    if (
      short_name != root_short_ && !is_builtin_interfaces_alias(short_name) &&
      dep_seen_.insert(std::string(short_name)).second) {
      dep_order_.emplace_back(short_name);
    }
    return hex;
  }

  // cppcheck-suppress passedByValue ; std::string_view is a view, pass by value
  std::string build_md5_text(const ms::MessageDef & def, std::string_view current_short)
  {
    std::ostringstream ss;

    // Constants first, in declaration order.
    for (const auto & c : def.constants) {
      ss << primitive_to_ros1_name(c.type) << " " << c.name << "=" << c.raw_value << "\n";
    }

    // std_msgs/Header gets a synthetic seq prefix that ROS 2 dropped.
    // We inject it before iterating the parsed fields so its
    // contribution lands in field order.
    if (is_header_short(current_short)) {
      ss << "uint32 seq\n";
    }

    for (const auto & f : def.fields) {
      emit_md5_field_line(ss, f, current_short);
    }

    std::string text = ss.str();
    while (!text.empty() && text.back() == '\n') {
      text.pop_back();
    }
    return text;
  }

  void emit_md5_field_line(
    std::ostringstream & ss, const ms::FieldDef & f, std::string_view current_short)
  {
    if (f.type.is_primitive()) {
      const auto kind = std::get<ms::PrimitiveKind>(f.type.base);
      if (kind == ms::PrimitiveKind::Wstring) {
        throw std::runtime_error(
          "ros1_meta: wstring is not representable in ROS 1 (field '" + std::string(current_short) +
          "." + f.name + "')");
      }
      record_warnings(f, current_short);
      ss << primitive_to_ros1_name(kind) << array_suffix(f.type.array) << " " << f.name << "\n";
      return;
    }

    // Nested base.
    const std::string base_short = to_short_name(std::get<std::string>(f.type.base));
    if (is_builtin_interfaces_alias(base_short)) {
      // Treated as primitive `time`/`duration` for both md5 and body.
      record_warnings(f, current_short);
      ss << builtin_alias_to_ros1(base_short) << array_suffix(f.type.array) << " " << f.name
         << "\n";
      return;
    }

    // Real nested: recursive MD5, no array suffix (ROS 1 quirk —
    // `Pose[]` and `Pose` produce the same line in the md5 text).
    const std::string sub_md5 = compute_md5(base_short);
    record_warnings(f, current_short);
    ss << sub_md5 << " " << f.name << "\n";
  }

  // Same iteration as build_md5_text but emits a ROS 1-style normalised
  // .msg body (no md5 substitution, type names in their textual form,
  // bounds dropped, defaults dropped).
  std::string emit_normalized_body(std::string_view short_name)
  {
    const ms::MessageDef * def = schema_.find(short_name);
    if (def == nullptr) {
      // Should not happen — compute_md5 already validated existence.
      return "";
    }

    std::ostringstream ss;
    for (const auto & c : def->constants) {
      ss << primitive_to_ros1_name(c.type) << " " << c.name << "=" << c.raw_value << "\n";
    }
    if (is_header_short(short_name)) {
      ss << "uint32 seq\n";
    }
    for (const auto & f : def->fields) {
      emit_body_field_line(ss, f);
    }
    return ss.str();
  }

  void emit_body_field_line(std::ostringstream & ss, const ms::FieldDef & f)
  {
    if (f.type.is_primitive()) {
      const auto kind = std::get<ms::PrimitiveKind>(f.type.base);
      // emit_md5_field_line already raised for wstring before we reach
      // the body emit; defensively check anyway.
      if (kind == ms::PrimitiveKind::Wstring) {
        return;
      }
      ss << primitive_to_ros1_name(kind) << array_suffix(f.type.array) << " " << f.name << "\n";
      return;
    }

    const std::string base_short = to_short_name(std::get<std::string>(f.type.base));
    if (is_builtin_interfaces_alias(base_short)) {
      ss << builtin_alias_to_ros1(base_short) << array_suffix(f.type.array) << " " << f.name
         << "\n";
      return;
    }

    // Nested type: emit short ("pkg/Type") form. Same-package shortening
    // (just "Type" instead of "pkg/Type" when pkg matches the current
    // message's package) is what the upstream ROS 1 .msg files do, but
    // we always emit the qualified form here — both forms are accepted
    // by ROS 1 readers and the qualified form is unambiguous.
    ss << base_short << array_suffix(f.type.array) << " " << f.name << "\n";
  }

  // Surface bound / default drops as warnings. Wire-irrelevant — both
  // bounds and ROS 2 default values are absent from the serialised
  // form — but useful for users tracking down "MD5 matched but
  // something looks off downstream".
  void record_warnings(const ms::FieldDef & f, std::string_view current_short)
  {
    if (f.type.is_primitive() && f.type.string_upper_bound.has_value()) {
      Ros1MetaWarning w;
      w.type = std::string(current_short);
      w.field = f.name;
      w.kind = "bound_dropped";
      w.detail =
        std::string("string<=") + std::to_string(*f.type.string_upper_bound) + " -> string";
      warnings_.push_back(std::move(w));
    }
    if (f.type.array.is_bounded_sequence()) {
      Ros1MetaWarning w;
      w.type = std::string(current_short);
      w.field = f.name;
      w.kind = "bound_dropped";
      w.detail = std::string("[<=") + std::to_string(f.type.array.size.value_or(0)) + "] -> []";
      warnings_.push_back(std::move(w));
    }
    if (f.default_value.has_value()) {
      Ros1MetaWarning w;
      w.type = std::string(current_short);
      w.field = f.name;
      w.kind = "default_dropped";
      w.detail = "default value '" + f.default_value->raw + "' dropped";
      warnings_.push_back(std::move(w));
    }
  }

  const ms::SchemaModel & schema_;
  std::string root_short_;
  std::unordered_map<std::string, std::string> md5_cache_;
  std::unordered_set<std::string> dep_seen_;
  std::vector<std::string> dep_order_;
  std::vector<Ros1MetaWarning> warnings_;
};

}  // namespace

Ros1MetaResult synthesize_ros1_meta(std::string_view ros2_type, std::string_view ros2_msg_text)
{
  Ros1MetaResult result;

  // The msg_schema parser accepts both short and canonical forms in
  // the root name argument; pass through verbatim.
  auto parsed = ms::parse_schema(ros2_type, ros2_msg_text);
  if (!parsed.ok()) {
    result.error = "ros1_meta: failed to parse ros2 .msg text: " +
                   (parsed.error.empty() ? "(no detail)" : parsed.error);
    return result;
  }

  const auto & schema = *parsed.schema;
  const std::string root_short = to_short_name(ros2_type);
  if (schema.find(root_short) == nullptr) {
    result.error = "ros1_meta: parsed schema does not contain root type: " + root_short;
    return result;
  }

  return Synthesizer(schema, root_short).run();
}

}  // namespace bagwiz::core
