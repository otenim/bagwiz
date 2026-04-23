// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/message_formatter.hpp"

#include "bagwiz/core/introspection_loader.hpp"

#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>

namespace bagwiz::core
{

namespace
{

namespace ts_types = rosidl_typesupport_introspection_cpp;

// --- helpers --------------------------------------------------------------

std::string float_to_string(float value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.7g", static_cast<double>(value));
  return std::string(buf);
}

std::string double_to_string(double value)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", value);
  return std::string(buf);
}

std::string hex_uint16_to_string(std::uint16_t value)
{
  char buf[16];
  std::snprintf(buf, sizeof(buf), "u+%04x", value);
  return std::string(buf);
}

std::string escape_for_yaml(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out += '\'';
  for (const char c : s) {
    if (c == '\'') {
      out += "''";
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
  out += '\'';
  return out;
}

const ts_types::MessageMembers & nested_members_of(const ts_types::MessageMember & m)
{
  return *static_cast<const ts_types::MessageMembers *>(m.members_->data);
}

// Convert a pointer-to-field into its textual YAML value for any primitive
// type id. `ptr` must point at an object whose C++ type matches `type_id`
// (this is the layout guarantee provided by rosidl_generator_cpp).
std::string primitive_at(const void * ptr, std::uint8_t type_id)
{
  using ts_types::ROS_TYPE_BOOLEAN;
  using ts_types::ROS_TYPE_CHAR;
  using ts_types::ROS_TYPE_DOUBLE;
  using ts_types::ROS_TYPE_FLOAT;
  using ts_types::ROS_TYPE_INT16;
  using ts_types::ROS_TYPE_INT32;
  using ts_types::ROS_TYPE_INT64;
  using ts_types::ROS_TYPE_INT8;
  using ts_types::ROS_TYPE_LONG_DOUBLE;
  using ts_types::ROS_TYPE_OCTET;
  using ts_types::ROS_TYPE_STRING;
  using ts_types::ROS_TYPE_UINT16;
  using ts_types::ROS_TYPE_UINT32;
  using ts_types::ROS_TYPE_UINT64;
  using ts_types::ROS_TYPE_UINT8;
  using ts_types::ROS_TYPE_WCHAR;
  using ts_types::ROS_TYPE_WSTRING;
  switch (type_id) {
    case ROS_TYPE_BOOLEAN:
      return *static_cast<const bool *>(ptr) ? "true" : "false";
    case ROS_TYPE_OCTET:
    case ROS_TYPE_UINT8:
      return std::to_string(static_cast<unsigned>(*static_cast<const std::uint8_t *>(ptr)));
    case ROS_TYPE_CHAR:
    case ROS_TYPE_INT8:
      return std::to_string(static_cast<int>(*static_cast<const std::int8_t *>(ptr)));
    case ROS_TYPE_UINT16:
      return std::to_string(*static_cast<const std::uint16_t *>(ptr));
    case ROS_TYPE_INT16:
      return std::to_string(*static_cast<const std::int16_t *>(ptr));
    case ROS_TYPE_UINT32:
      return std::to_string(*static_cast<const std::uint32_t *>(ptr));
    case ROS_TYPE_INT32:
      return std::to_string(*static_cast<const std::int32_t *>(ptr));
    case ROS_TYPE_UINT64:
      return std::to_string(*static_cast<const std::uint64_t *>(ptr));
    case ROS_TYPE_INT64:
      return std::to_string(*static_cast<const std::int64_t *>(ptr));
    case ROS_TYPE_FLOAT:
      return float_to_string(*static_cast<const float *>(ptr));
    case ROS_TYPE_DOUBLE:
      return double_to_string(*static_cast<const double *>(ptr));
    case ROS_TYPE_LONG_DOUBLE:
      return "<long double>";
    case ROS_TYPE_STRING:
      return escape_for_yaml(*static_cast<const std::string *>(ptr));
    case ROS_TYPE_WCHAR:
      return hex_uint16_to_string(*static_cast<const std::uint16_t *>(ptr));
    case ROS_TYPE_WSTRING:
      return "<wstring>";
    default:
      return "<unknown type " + std::to_string(static_cast<int>(type_id)) + ">";
  }
}

bool is_primitive(std::uint8_t type_id)
{
  return type_id != ts_types::ROS_TYPE_MESSAGE;
}

// Count of elements for an array/sequence member. Falls back to
// `array_size_` for callers that did not set size_function (shouldn't
// happen for rosidl-generated code, but the null-check keeps us safe).
std::size_t member_count(const ts_types::MessageMember & m, const void * field_ptr)
{
  if (m.size_function != nullptr) {
    return m.size_function(field_ptr);
  }
  return m.array_size_;
}

const void * member_element(
  const ts_types::MessageMember & m, const void * field_ptr, std::size_t index)
{
  // get_const_function is the universal accessor for both std::array and
  // std::vector (and BoundedVector) across introspection-cpp. It handles
  // the indirection details we do not want to replicate.
  return m.get_const_function(field_ptr, index);
}

// --- emitter --------------------------------------------------------------

class Emitter
{
public:
  Emitter(std::string & out, const FormatOptions & opts) : out_(out), opts_(opts) {}

  void emit_members(
    const ts_types::MessageMembers & members, const void * base, const std::string & indent,
    std::size_t depth)
  {
    if (depth > opts_.max_depth) {
      out_ += indent;
      out_ += "<max depth reached>\n";
      return;
    }
    const auto * base_bytes = static_cast<const std::uint8_t *>(base);
    for (std::uint32_t i = 0; i < members.member_count_; ++i) {
      const ts_types::MessageMember & m = members.members_[i];
      const void * field = base_bytes + m.offset_;
      emit_member(m, field, indent, depth);
    }
  }

private:
  void emit_member(
    const ts_types::MessageMember & m, const void * field, const std::string & indent,
    std::size_t depth)
  {
    out_ += indent;
    out_ += m.name_;
    out_ += ':';

    if (!m.is_array_) {
      if (is_primitive(m.type_id_)) {
        out_ += ' ';
        out_ += primitive_at(field, m.type_id_);
        out_ += '\n';
      } else {
        out_ += '\n';
        emit_members(nested_members_of(m), field, indent + "  ", depth + 1);
      }
      return;
    }

    const std::size_t count = member_count(m, field);
    if (count == 0) {
      out_ += " []\n";
      return;
    }

    if (is_primitive(m.type_id_)) {
      emit_primitive_array(m, count, field);
      return;
    }

    // Array / sequence of messages: block style with "- " markers.
    out_ += '\n';
    const ts_types::MessageMembers & sub = nested_members_of(m);
    const std::string item_indent = indent + "  ";
    for (std::size_t i = 0; i < count; ++i) {
      const void * elem = member_element(m, field, i);
      emit_message_list_item(sub, elem, indent, item_indent, depth + 1);
    }
  }

  void emit_primitive_array(
    const ts_types::MessageMember & m, std::size_t count, const void * field)
  {
    if (count > opts_.max_inline_array) {
      out_ += " [<";
      out_ += std::to_string(count);
      out_ += " items>]\n";
      return;
    }
    out_ += " [";
    for (std::size_t i = 0; i < count; ++i) {
      if (i != 0) {
        out_ += ", ";
      }
      const void * elem = member_element(m, field, i);
      out_ += primitive_at(elem, m.type_id_);
    }
    out_ += "]\n";
  }

  // One element of a list of messages. The first child field uses the
  // "- " dash; subsequent ones align under it.
  void emit_message_list_item(
    const ts_types::MessageMembers & sub, const void * base, const std::string & list_indent,
    const std::string & item_indent, std::size_t depth)
  {
    if (depth > opts_.max_depth) {
      out_ += list_indent;
      out_ += "- <max depth reached>\n";
      return;
    }
    if (sub.member_count_ == 0) {
      out_ += list_indent;
      out_ += "- {}\n";
      return;
    }
    const auto * base_bytes = static_cast<const std::uint8_t *>(base);
    for (std::uint32_t i = 0; i < sub.member_count_; ++i) {
      const ts_types::MessageMember & child = sub.members_[i];
      const void * child_field = base_bytes + child.offset_;
      out_ += (i == 0) ? list_indent : item_indent;
      out_ += (i == 0) ? "- " : "  ";
      out_ += child.name_;
      out_ += ':';
      emit_list_item_child_value(child, child_field, item_indent, depth);
    }
  }

  void emit_list_item_child_value(
    const ts_types::MessageMember & m, const void * field, const std::string & item_indent,
    std::size_t depth)
  {
    if (!m.is_array_) {
      if (is_primitive(m.type_id_)) {
        out_ += ' ';
        out_ += primitive_at(field, m.type_id_);
        out_ += '\n';
      } else {
        out_ += '\n';
        emit_members(nested_members_of(m), field, item_indent + "    ", depth + 1);
      }
      return;
    }
    const std::size_t count = member_count(m, field);
    if (count == 0) {
      out_ += " []\n";
      return;
    }
    if (is_primitive(m.type_id_)) {
      emit_primitive_array(m, count, field);
      return;
    }
    out_ += '\n';
    const ts_types::MessageMembers & sub = nested_members_of(m);
    const std::string inner_list_indent = item_indent + "  ";
    const std::string inner_item_indent = inner_list_indent + "  ";
    for (std::size_t i = 0; i < count; ++i) {
      const void * elem = member_element(m, field, i);
      emit_message_list_item(sub, elem, inner_list_indent, inner_item_indent, depth + 1);
    }
  }

  std::string & out_;
  const FormatOptions & opts_;
};

// --- RMW deserialize + lifecycle -----------------------------------------

// RAII wrapper for an aligned buffer big enough to hold one default-
// constructed instance of the type described by `members`. The buffer is
// init_function'd on construction and fini_function'd on destruction.
class MessageBuffer
{
public:
  explicit MessageBuffer(const ts_types::MessageMembers & members) : members_(&members)
  {
    void * p = nullptr;
    // posix_memalign requires size to be non-zero and >= alignment. Use
    // max_align_t which covers std::string/vector alignment on Linux.
    const std::size_t size = members.size_of_ == 0 ? 1 : members.size_of_;
    if (::posix_memalign(&p, alignof(std::max_align_t), size) != 0 || p == nullptr) {
      throw std::bad_alloc();
    }
    buffer_ = p;
    members.init_function(buffer_, rosidl_runtime_cpp::MessageInitialization::ALL);
    initialized_ = true;
  }

  ~MessageBuffer()
  {
    if (initialized_ && members_ != nullptr && buffer_ != nullptr) {
      members_->fini_function(buffer_);
    }
    std::free(buffer_);
  }

  MessageBuffer(const MessageBuffer &) = delete;
  MessageBuffer & operator=(const MessageBuffer &) = delete;

  void * data() { return buffer_; }
  const void * data() const { return buffer_; }

private:
  const ts_types::MessageMembers * members_;
  void * buffer_ = nullptr;
  bool initialized_ = false;
};

std::string hex_preview(std::span<const std::byte> payload, std::size_t max_bytes = 16)
{
  const std::size_t n = std::min(payload.size(), max_bytes);
  std::string out;
  out.reserve(n * 3 + 16);
  for (std::size_t i = 0; i < n; ++i) {
    char buf[4];
    std::snprintf(buf, sizeof(buf), "%02x ", static_cast<unsigned>(payload[i]));
    out += buf;
  }
  if (payload.size() > n) {
    out += "...";
  }
  return out;
}

// Copy the bag payload into a freshly-initialized rmw_serialized_message_t
// and hand it off to rmw_deserialize. The copy is intentional: keeps
// ownership of the buffer inside rmw/rcutils so we never have to reason
// about whether the implementation mutates or frees the input span.
void rmw_decode(
  std::span<const std::byte> payload, const rosidl_message_type_support_t & ts, void * out)
{
  rcutils_allocator_t alloc = rcutils_get_default_allocator();
  rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  const rmw_ret_t init_ret = rmw_serialized_message_init(&serialized, payload.size(), &alloc);
  if (init_ret != RMW_RET_OK) {
    throw std::runtime_error("rmw_serialized_message_init failed");
  }
  std::memcpy(serialized.buffer, payload.data(), payload.size());
  serialized.buffer_length = payload.size();

  const rmw_ret_t rc = rmw_deserialize(&serialized, &ts, out);
  std::string err;
  if (rc != RMW_RET_OK) {
    const rcutils_error_state_t * s = rcutils_get_error_state();
    err = "rmw_deserialize failed (size=" + std::to_string(payload.size()) +
          ", first bytes: " + hex_preview(payload) + "): ";
    err += s != nullptr ? s->message : "(no error message)";
    rcutils_reset_error();
  }
  rmw_serialized_message_fini(&serialized);
  if (!err.empty()) {
    throw std::runtime_error(err);
  }
}

}  // namespace

FormatResult format_message(
  const IntrospectionLoad & introspection, std::span<const std::byte> cdr_payload,
  const FormatOptions & options)
{
  FormatResult result;
  if (!introspection.ok()) {
    result.error = "introspection not loaded";
    return result;
  }
  try {
    MessageBuffer buffer(*introspection.members);
    // Pass the introspection typesupport handle directly. Both cyclonedds
    // and fastrtps's rmw_deserialize paths accept it without going through
    // the rosidl_typesupport_cpp wrapper (which on Humble is not linked
    // against the per-package RMW-specific libraries and therefore cannot
    // dispatch).
    rmw_decode(cdr_payload, *introspection.typesupport, buffer.data());
    Emitter emitter(result.text, options);
    emitter.emit_members(*introspection.members, buffer.data(), "", 0);
  } catch (const std::exception & e) {
    result.text.clear();
    result.error = e.what();
  }
  return result;
}

}  // namespace bagwiz::core
