// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/schema_resolver.hpp"

#include "bagwiz/core/introspection_loader.hpp"
#include "bagwiz/core/msg_definition_resolver.hpp"

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

namespace ts_types = rosidl_typesupport_introspection_cpp;

constexpr std::string_view kSeparator =
  "================================================================================";

// ---------------------------------------------------------------------
// Bag-embedded path
// ---------------------------------------------------------------------
//
// The bag's self-describing schema record (rosbag2 mcap, Iron+) carries
// the concatenated .msg text directly; no transformation is needed
// beyond validating that the encoding is the one we expect.

ResolvedSchemaCandidate try_bag_embedded(const ResolveSchemaInput & input)
{
  ResolvedSchemaCandidate c;
  c.source = SchemaSource::BagEmbedded;

  if (input.bag_embedded_text.empty()) {
    c.error = "bag-embedded: no schema text provided";
    return c;
  }
  if (input.bag_embedded_encoding != "ros2msg") {
    c.error =
      "bag-embedded: encoding '" +
      (input.bag_embedded_encoding.empty() ? std::string("(empty)") : input.bag_embedded_encoding) +
      "' is not 'ros2msg'";
    return c;
  }

  c.text = input.bag_embedded_text;
  c.encoding = "ros2msg";
  return c;
}

// ---------------------------------------------------------------------
// AMENT install path
// ---------------------------------------------------------------------

ResolvedSchemaCandidate try_ament(const ResolveSchemaInput & input)
{
  ResolvedSchemaCandidate c;
  c.source = SchemaSource::AmentInstall;

  const auto resolved = resolve_message_definition(input.ros2_type);
  if (resolved.text.empty() || resolved.encoding.empty()) {
    c.error = "ament: package or .msg not found for '" + input.ros2_type + "'";
    return c;
  }
  c.text = resolved.text;
  c.encoding = resolved.encoding;
  return c;
}

// ---------------------------------------------------------------------
// Introspection path
// ---------------------------------------------------------------------
//
// Walk the MessageMembers tree and emit ROS 2 .msg-shaped body lines.
// Constants and default values are not preserved (the introspection
// metadata does not expose them); the synthesised body is wire-faithful
// but lacks those decorations. Bound information is preserved so the
// downstream synthesizer can record the same warnings that the AMENT
// path would.

std::string_view introspection_primitive_name(std::uint8_t type_id)
{
  // Note: ROS_TYPE_BOOL / _BYTE / _FLOAT32 / _FLOAT64 are aliases of
  // ROS_TYPE_BOOLEAN / _OCTET / _FLOAT / _DOUBLE in
  // <rosidl_typesupport_introspection_cpp/field_types.hpp>, so they
  // cannot share switch labels. We list the canonical-spelling label
  // (the one that has the IDL keyword's name) once.
  switch (type_id) {
    case ts_types::ROS_TYPE_BOOLEAN:
      return "bool";
    case ts_types::ROS_TYPE_OCTET:
      return "byte";
    case ts_types::ROS_TYPE_CHAR:
      return "char";
    case ts_types::ROS_TYPE_FLOAT:
      return "float32";
    case ts_types::ROS_TYPE_DOUBLE:
      return "float64";
    case ts_types::ROS_TYPE_INT8:
      return "int8";
    case ts_types::ROS_TYPE_UINT8:
      return "uint8";
    case ts_types::ROS_TYPE_INT16:
      return "int16";
    case ts_types::ROS_TYPE_UINT16:
      return "uint16";
    case ts_types::ROS_TYPE_INT32:
      return "int32";
    case ts_types::ROS_TYPE_UINT32:
      return "uint32";
    case ts_types::ROS_TYPE_INT64:
      return "int64";
    case ts_types::ROS_TYPE_UINT64:
      return "uint64";
    case ts_types::ROS_TYPE_STRING:
      return "string";
    case ts_types::ROS_TYPE_WSTRING:
      return "wstring";
    case ts_types::ROS_TYPE_LONG_DOUBLE:
      // No official ROS 2 .msg keyword; emit the rosidl-internal name so
      // downstream synthesisers can detect and refuse it.
      return "float128";
    default:
      return "";
  }
}

// Convert "pkg::msg" / "pkg::msg::sub" / "pkg" namespace strings into the
// short package name. Introspection always uses C++ namespace separators
// (`::`), so the rule is "take the leading segment".
std::string namespace_to_short_package(const char * ns)
{
  if (ns == nullptr) {
    return "";
  }
  std::string s(ns);
  const auto sep = s.find("::");
  if (sep == std::string::npos) {
    return s;
  }
  return s.substr(0, sep);
}

// "pkg/Type" form for a nested introspection type, used as both the
// field reference in the parent body and the dependency lookup key.
std::string nested_short_name(const ts_types::MessageMembers & members)
{
  return namespace_to_short_package(members.message_namespace_) + "/" +
         (members.message_name_ != nullptr ? members.message_name_ : "");
}

// Render the array suffix for one introspection field.
//   is_array_=false                             -> ""        (scalar)
//   is_array_=true,  array_size_=0              -> "[]"      (unbounded)
//   is_array_=true,  is_upper_bound_=true,N>0   -> "[<=N]"   (bounded)
//   is_array_=true,  array_size_=N>0            -> "[N]"     (fixed)
std::string introspection_array_suffix(const ts_types::MessageMember & m)
{
  if (!m.is_array_) {
    return "";
  }
  if (m.array_size_ == 0U) {
    return "[]";
  }
  if (m.is_upper_bound_) {
    return "[<=" + std::to_string(m.array_size_) + "]";
  }
  return "[" + std::to_string(m.array_size_) + "]";
}

// Bounded-string suffix on the type token: `string<=N` instead of plain
// `string` when string_upper_bound_ is set.
std::string introspection_string_bound_suffix(const ts_types::MessageMember & m)
{
  if (m.string_upper_bound_ == 0U) {
    return "";
  }
  return "<=" + std::to_string(m.string_upper_bound_);
}

// One field line. Returns the empty string for unsupported type ids,
// which we surface as a synthesis failure to the caller (rather than
// emit a malformed body).
std::string introspection_field_line(const ts_types::MessageMember & m)
{
  if (m.name_ == nullptr) {
    return "";
  }

  std::string type_token;
  if (m.type_id_ == ts_types::ROS_TYPE_MESSAGE) {
    if (m.members_ == nullptr || m.members_->data == nullptr) {
      return "";
    }
    const auto & sub = *static_cast<const ts_types::MessageMembers *>(m.members_->data);
    type_token = nested_short_name(sub);
  } else {
    const std::string_view prim = introspection_primitive_name(m.type_id_);
    if (prim.empty()) {
      return "";
    }
    type_token.assign(prim);
    if (m.type_id_ == ts_types::ROS_TYPE_STRING || m.type_id_ == ts_types::ROS_TYPE_WSTRING) {
      type_token.append(introspection_string_bound_suffix(m));
    }
  }

  std::string line = type_token + introspection_array_suffix(m) + " " + m.name_ + "\n";
  return line;
}

// Recursively collect the ordered set of dependency types referenced by
// `members` (excluding the root). Order is DFS-by-first-mention, matching
// the convention used by AMENT-resolved text.
struct IntrospectionWalk
{
  bool ok = true;
  std::string error;
  // For each unique short ("pkg/Type") name we visit, the corresponding
  // MessageMembers pointer used to render its body.
  std::unordered_map<std::string, const ts_types::MessageMembers *> bodies;
  // Order in which we first saw each non-root type — used to emit
  // dependencies deterministically.
  std::vector<std::string> dep_order;
  // Set of every key we've seen (root + deps) to short-circuit cycles
  // and duplicates without affecting `dep_order`.
  std::unordered_set<std::string> seen;
};

void walk_introspection(
  IntrospectionWalk & w, const ts_types::MessageMembers & members, const std::string & this_short,
  bool is_root)
{
  if (!w.ok) {
    return;
  }
  if (!w.seen.insert(this_short).second) {
    return;  // already visited (cycle or shared dep)
  }
  w.bodies.emplace(this_short, &members);
  if (!is_root) {
    w.dep_order.push_back(this_short);
  }

  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const auto & m = members.members_[i];
    if (m.type_id_ != ts_types::ROS_TYPE_MESSAGE) {
      continue;
    }
    if (m.members_ == nullptr || m.members_->data == nullptr) {
      w.ok = false;
      w.error = "introspection: nested type for field '" +
                std::string(m.name_ != nullptr ? m.name_ : "(unnamed)") + "' is null";
      return;
    }
    const auto & sub = *static_cast<const ts_types::MessageMembers *>(m.members_->data);
    walk_introspection(w, sub, nested_short_name(sub), /*is_root=*/false);
    if (!w.ok) {
      return;
    }
  }
}

// Render one type's field block (no `MSG:` header). Returns empty
// string and sets `error` on unsupported field shapes.
std::string render_introspection_body(const ts_types::MessageMembers & members, std::string & error)
{
  std::string out;
  for (std::uint32_t i = 0; i < members.member_count_; ++i) {
    const auto & m = members.members_[i];
    const std::string line = introspection_field_line(m);
    if (line.empty()) {
      error = std::string("introspection: cannot render field '") +
              (m.name_ != nullptr ? m.name_ : "(unnamed)") +
              "' (type_id=" + std::to_string(static_cast<int>(m.type_id_)) + ")";
      return "";
    }
    out.append(line);
  }
  return out;
}

ResolvedSchemaCandidate try_introspection(const ResolveSchemaInput & input)
{
  ResolvedSchemaCandidate c;
  c.source = SchemaSource::Introspection;

  const auto load = load_introspection(input.ros2_type);
  if (!load.ok()) {
    c.error =
      "introspection: " + (load.error.empty() ? std::string("typesupport not loaded") : load.error);
    return c;
  }

  const auto & root = *load.members;
  IntrospectionWalk walk;
  const std::string root_short = nested_short_name(root);
  if (root_short.empty() || root_short == "/") {
    c.error = "introspection: root type has no namespace/name";
    return c;
  }
  walk_introspection(walk, root, root_short, /*is_root=*/true);
  if (!walk.ok) {
    c.error = walk.error;
    return c;
  }

  std::string body_error;
  std::string body = render_introspection_body(root, body_error);
  if (!body_error.empty()) {
    c.error = body_error;
    return c;
  }

  // Concatenated form: root body, then a separator + `MSG:` header for
  // each transitively referenced dependency, in DFS-first-seen order.
  std::ostringstream ss;
  ss << body;
  for (const auto & dep : walk.dep_order) {
    auto it = walk.bodies.find(dep);
    if (it == walk.bodies.end() || it->second == nullptr) {
      c.error = "introspection: missing body for dep '" + dep + "'";
      return c;
    }
    std::string dep_body = render_introspection_body(*it->second, body_error);
    if (!body_error.empty()) {
      c.error = body_error;
      return c;
    }
    ss << kSeparator << "\n"
       << "MSG: " << dep << "\n"
       << dep_body;
  }

  c.text = ss.str();
  c.encoding = "ros2msg";
  return c;
}

}  // namespace

std::string_view to_string(SchemaSource source) noexcept
{
  switch (source) {
    case SchemaSource::BagEmbedded:
      return "bag-embedded";
    case SchemaSource::AmentInstall:
      return "ament";
    case SchemaSource::Introspection:
      return "introspection";
  }
  return "unknown";
}

ResolveSchemaResult resolve_schema(const ResolveSchemaInput & input)
{
  ResolveSchemaResult result;

  // Empty / malformed type names short-circuit before any source is
  // attempted; otherwise both AMENT and introspection would each
  // produce their own confused error message.
  if (input.ros2_type.empty()) {
    return result;
  }

  // Always attempt every source so the caller can crosscheck. Priority
  // order is bag-embedded → AMENT → introspection, so the producer-
  // shipped schema text wins outright when present.
  result.candidates.reserve(3);
  result.candidates.push_back(try_bag_embedded(input));
  result.candidates.push_back(try_ament(input));
  result.candidates.push_back(try_introspection(input));

  for (const auto & c : result.candidates) {
    if (c.ok()) {
      result.ok = true;
      result.text = c.text;
      result.encoding = c.encoding;
      result.source = c.source;
      break;
    }
  }
  return result;
}

}  // namespace bagwiz::core
