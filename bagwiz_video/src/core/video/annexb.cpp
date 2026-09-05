// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/annexb.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace bagwiz::core::video
{

namespace
{

// Length of the start code at `pos` (3 or 4), or 0 when there is none.
std::size_t start_code_length_at(std::span<const std::byte> data, std::size_t pos)
{
  const auto at = [&](std::size_t i) { return static_cast<std::uint8_t>(data[pos + i]); };
  if (pos + 3 <= data.size() && at(0) == 0 && at(1) == 0 && at(2) == 1) {
    return 3;
  }
  if (pos + 4 <= data.size() && at(0) == 0 && at(1) == 0 && at(2) == 0 && at(3) == 1) {
    return 4;
  }
  return 0;
}

// Fold one NAL unit's header byte into the summary.
void classify_nal(AnnexBSummary & summary, std::uint8_t header, VideoCodec codec)
{
  ++summary.nal_count;
  if (codec == VideoCodec::kH264) {
    // H.264 nal_unit_type: 5 IDR slice, 7 SPS, 8 PPS.
    const std::uint8_t type = header & 0x1FU;
    summary.has_sps = summary.has_sps || type == 7;
    summary.has_pps = summary.has_pps || type == 8;
    summary.has_keyframe_slice = summary.has_keyframe_slice || type == 5;
    return;
  }
  // H.265 nal_unit_type sits in bits 1..6 of the first header byte:
  // 16..23 IRAP slices (BLA/IDR/CRA), 32 VPS, 33 SPS, 34 PPS.
  const std::uint8_t type = (header >> 1U) & 0x3FU;
  summary.has_vps = summary.has_vps || type == 32;
  summary.has_sps = summary.has_sps || type == 33;
  summary.has_pps = summary.has_pps || type == 34;
  summary.has_keyframe_slice = summary.has_keyframe_slice || (type >= 16 && type <= 23);
}

}  // namespace

AnnexBSummary summarize_annexb(std::span<const std::byte> data, VideoCodec codec)
{
  AnnexBSummary summary;
  const std::size_t first = start_code_length_at(data, 0);
  if (first == 0) {
    return summary;
  }
  summary.starts_with_start_code = true;

  // Each NAL unit begins right after a start code; its first byte is the
  // NAL header. A start code that ends the buffer has no NAL behind it.
  std::size_t pos = 0;
  while (pos < data.size()) {
    const std::size_t sc = start_code_length_at(data, pos);
    if (sc == 0) {
      ++pos;
      continue;
    }
    pos += sc;
    if (pos >= data.size()) {
      break;
    }
    classify_nal(summary, static_cast<std::uint8_t>(data[pos]), codec);
    ++pos;
  }
  return summary;
}

}  // namespace bagwiz::core::video
