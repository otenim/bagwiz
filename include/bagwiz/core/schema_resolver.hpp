// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SCHEMA_RESOLVER_HPP_
#define BAGWIZ__CORE__SCHEMA_RESOLVER_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// Provenance of a resolved ROS 2 schema. The resolver tries sources in
// the priority order BagEmbedded → AmentInstall → Introspection so that
// a self-described bag wins over local install metadata, and
// introspection typesupport serves as a last-ditch fallback. This enum
// lets callers report which source was used and lets crosscheck
// reporting compare candidates from different sources.
enum class SchemaSource : int {
  // Schema text shipped inside the bag itself (rosbag2 mcap's
  // self-describing schema, available in Iron+). Highest priority — it
  // is the only source guaranteed to match what the producer actually
  // serialised, even if the local machine has a stale or missing
  // package install.
  BagEmbedded = 0,

  // Schema text loaded from `$AMENT_PREFIX_PATH/share/<pkg>/msg/<Type>.msg`
  // and recursively expanded via msg_definition_resolver. Second
  // priority — present when the user has the matching ROS 2 distro
  // sourced.
  AmentInstall = 1,

  // Schema text synthesised from the introspection typesupport library
  // (`lib<pkg>__rosidl_typesupport_introspection_cpp.so`) by walking the
  // MessageMembers tree. Third priority — emits a wire-equivalent .msg
  // body but cannot recover constants or default values, so MD5
  // comparisons against a constants-bearing producer may diverge. Used
  // as the final fallback so that "introspection-only" packages
  // (binary-installed with no .msg files on disk) still convert.
  Introspection = 2,
};

// Human-readable label for a SchemaSource. Used in CLI summaries and
// crosscheck output so users can tell where a md5 came from.
std::string_view to_string(SchemaSource source) noexcept;

// One attempt at a particular source. The resolver records every
// attempted source (success or failure) so callers can drive multi-
// source MD5 crosscheck reporting without re-running the resolution
// pipeline. Each candidate's `text` (when ok) can be passed through
// `synthesize_ros1_meta` to derive a per-source md5, and any divergence
// between sources is a meaningful signal — for example, an AMENT-vs-
// introspection mismatch means the local install drifted from the
// runtime typesupport.
struct ResolvedSchemaCandidate
{
  SchemaSource source;

  // Concatenated rosbag2-style .msg text:
  //   <root body>
  //   ================================================================================
  //   MSG: dep_pkg/dep_Type
  //   <dep body>
  //   ...
  //
  // Empty when this source could not produce a schema (see `error`).
  std::string text;

  // "ros2msg" on success; empty on failure.
  std::string encoding;

  // Empty on success; carries the human-readable failure reason
  // otherwise (e.g. "AMENT: package not found", "introspection: dlopen
  // failed: <dlerror>").
  std::string error;

  bool ok() const noexcept { return error.empty() && !text.empty(); }
};

// Inputs to the resolver. `bag_embedded_text` is optional: empty or
// non-"ros2msg" encodings disable the bag-embedded path. The other two
// sources are always tried.
//
// The resolver does not consume the bag itself; the caller (convert
// pipeline) is responsible for plumbing whatever schema text the bag
// reader surfaced for this connection. This keeps the resolver
// independently testable without rosbag2 dependencies.
struct ResolveSchemaInput
{
  // ROS 2 type name, accepted in canonical "pkg/msg/Type" form
  // (preferred — what rosbag2 emits) or short "pkg/Type" form (legacy).
  std::string ros2_type;

  // Schema text from the bag's self-describing record. Empty when the
  // bag predates Iron or the connection has no embedded schema.
  std::string bag_embedded_text;

  // Encoding string from the bag's schema record (e.g. "ros2msg"). The
  // resolver only honours bag-embedded text whose encoding is "ros2msg".
  std::string bag_embedded_encoding;
};

// Final resolution outcome. `text` / `encoding` / `source` describe the
// winning candidate (the first source whose attempt succeeded, in
// priority order). `candidates` records every attempt for downstream
// crosscheck reporting.
//
// `ok == false` means no source produced a usable schema; `text` and
// `encoding` are empty in that case but `candidates` still describes
// what was tried and why each failed.
struct ResolveSchemaResult
{
  bool ok = false;
  std::string text;
  std::string encoding;
  SchemaSource source = SchemaSource::BagEmbedded;
  std::vector<ResolvedSchemaCandidate> candidates;
};

// Resolve the canonical .msg text for `input.ros2_type`, attempting each
// source in priority order (bag-embedded → AMENT → introspection). The
// first successful source wins; remaining sources are still attempted so
// the caller can crosscheck multiple md5s for divergence.
//
// Thread-safe: the per-process AMENT cache from msg_definition_resolver
// is mutex-guarded; introspection's dlopen is process-global and once
// loaded a typesupport handle remains valid for the lifetime of the
// process.
ResolveSchemaResult resolve_schema(const ResolveSchemaInput & input);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__SCHEMA_RESOLVER_HPP_
