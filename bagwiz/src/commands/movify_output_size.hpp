// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_OUTPUT_SIZE_HPP_
#define COMMANDS__MOVIFY_OUTPUT_SIZE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

// The oversize guard for `movify`'s composed output. The grid canvas composes
// its cell size from the clock panel's first frame, so the resolution is a
// product the user never typed directly: three columns of 1080p cameras is a
// 5760-pixel-wide output nobody asked for by name. This is not an error — a
// large render is sometimes exactly what is wanted — so the size is reported
// rather than rejected.
//
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// The size above which a composed output is worth reporting. 4K UHD is the
// largest resolution a run is likely to have asked for on purpose (a 2x2 grid
// of 1080p views lands exactly here), so the check fires strictly above it.
inline constexpr std::uint32_t kOversizeWarnWidth = 3840;
inline constexpr std::uint32_t kOversizeWarnHeight = 2160;
inline constexpr std::uint64_t kOversizeWarnPixels =
  static_cast<std::uint64_t>(kOversizeWarnWidth) * kOversizeWarnHeight;

// The warning for an output of `width` x `height`, or std::nullopt when it fits
// within kOversizeWarnPixels. The test is on the pixel product, so a tall,
// narrow output is judged by what it actually costs to encode rather than by
// either dimension alone.
//
// `detail` says how the size arose and is folded into the parenthetical beside
// the megapixel count; pass an empty view to omit it (a 1x1 grid has nothing
// to add — "1x1 grid of WxH cells" would only restate the size beside it).
// `remedy` closes the message with the flags that bring the size down. Both
// are caller-supplied because only the caller knows them.
[[nodiscard]] std::optional<std::string> oversized_output_warning(
  std::uint32_t width, std::uint32_t height, std::string_view detail, std::string_view remedy);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_OUTPUT_SIZE_HPP_
