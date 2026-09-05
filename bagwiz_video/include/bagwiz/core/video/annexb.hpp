// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__VIDEO__ANNEXB_HPP_
#define BAGWIZ__CORE__VIDEO__ANNEXB_HPP_

#include "bagwiz/core/video/video_codec.hpp"

#include <cstddef>
#include <span>

namespace bagwiz::core::video
{

// What one Annex B access unit (the `data` of a CompressedVideo message)
// carries, as far as Foxglove's playback requirements go: the stream must
// be start-code delimited, and a keyframe message must also hold the
// parameter sets a decoder needs to start from it (SPS/PPS for H.264,
// VPS/SPS/PPS for H.265).
struct AnnexBSummary
{
  bool starts_with_start_code = false;  // a 3- or 4-byte 00 00 (00) 01 prefix
  std::size_t nal_count = 0;            // NAL units found behind start codes
  bool has_vps = false;                 // H.265 only; always false for H.264
  bool has_sps = false;
  bool has_pps = false;
  // An IDR slice (H.264) or an IRAP slice (H.265: IDR/CRA/BLA).
  bool has_keyframe_slice = false;
};

// Walk the NAL units of `data` and classify them by type. Data that does not
// begin with a start code (e.g. AVCC length-prefixed samples) yields a
// summary with `nal_count == 0` and every flag false.
[[nodiscard]] AnnexBSummary summarize_annexb(std::span<const std::byte> data, VideoCodec codec);

}  // namespace bagwiz::core::video

#endif  // BAGWIZ__CORE__VIDEO__ANNEXB_HPP_
