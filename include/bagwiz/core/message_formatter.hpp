// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_
#define BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_

#include <cstddef>
#include <span>
#include <string>

namespace bagwiz::core
{

struct IntrospectionLoad;

// Options controlling how format_message renders large values. Defaults
// keep single-message output reasonable for a terminal screen: primitive
// arrays wider than `max_inline_array` are summarized, deeper nested
// messages beyond `max_depth` become "<truncated>".
struct FormatOptions
{
  std::size_t max_inline_array = 32;
  std::size_t max_depth = 16;
};

// Outcome of a format_message() call. On success `text` holds the rendered
// YAML-ish string; on failure `error` explains what went wrong (RMW
// deserialize failure or walker error).
struct FormatResult
{
  std::string text;
  std::string error;
  bool ok() const { return error.empty(); }
};

// Decode the CDR payload via the active RMW (rmw_deserialize), then walk
// the resulting in-memory struct using introspection metadata to emit a
// YAML-ish rendering that mirrors `ros2 topic echo`.
//
// The caller passes an IntrospectionLoad that resolved both the cpp and
// introspection typesupports. Allocation + construction + destruction of
// the intermediate struct are managed internally (RAII).
FormatResult format_message(
  const IntrospectionLoad & introspection, std::span<const std::byte> cdr_payload,
  const FormatOptions & options = {});

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_
