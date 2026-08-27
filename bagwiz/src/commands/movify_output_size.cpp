// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_output_size.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr double kPixelsPerMegapixel = 1'000'000.0;
}  // namespace

std::optional<std::string> oversized_output_warning(
  std::uint32_t width, std::uint32_t height, std::string_view detail, std::string_view remedy)
{
  // Widen before multiplying: two uint32 dimensions overflow a uint32 product
  // long before they reach a size worth reporting.
  const std::uint64_t pixels = static_cast<std::uint64_t>(width) * height;
  if (pixels <= kOversizeWarnPixels) {
    return std::nullopt;
  }

  std::string message = fmt::format(
    "output is {}x{} ({}{:.1f} Mpx), larger than {}x{}; encoding will be slow and the output "
    "file large.",
    width, height, detail.empty() ? std::string{} : fmt::format("{}, ", detail),
    static_cast<double>(pixels) / kPixelsPerMegapixel, kOversizeWarnWidth, kOversizeWarnHeight);
  if (!remedy.empty()) {
    message += " ";
    message += remedy;
  }
  return message;
}

}  // namespace bagwiz::commands
