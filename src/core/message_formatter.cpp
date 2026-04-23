// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/message_formatter.hpp"

#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/message_deserializer.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstdint>
#include <cstdio>
#include <exception>
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
    DeserializedMessage decoded(introspection, cdr_payload);
    Emitter emitter(result.text, options);
    emitter.emit_members(decoded.members(), decoded.data(), "", 0);
  } catch (const std::exception & e) {
    result.text.clear();
    result.error = e.what();
  }
  return result;
}

}  // namespace bagwiz::core
