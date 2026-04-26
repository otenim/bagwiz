// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_
#define BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"

#include <cstddef>
#include <string>

namespace bagwiz::core
{

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
// YAML-ish string; on failure `error` explains what went wrong (always a
// shape mismatch — the bytes-to-Value step is the decoder's job).
struct FormatResult
{
  std::string text;
  std::string error;
  bool ok() const { return error.empty(); }
};

// Render a decoded message to a YAML-ish string mirroring `ros2 topic
// echo`. Input is the Value produced by the Phase D decoder (either
// schema-driven or introspection-based — both yield the same shape).
//
// The Value MUST wrap a top-level Object (a struct). Primitive or
// sequence Values at the root are rejected; the shape contract for
// "decoded message" is always an Object.
FormatResult format_message(const cdr_walker::Value & root, const FormatOptions & options = {});

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__MESSAGE_FORMATTER_HPP_
