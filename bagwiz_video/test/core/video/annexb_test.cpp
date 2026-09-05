// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/video/annexb.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace
{

using bagwiz::core::video::summarize_annexb;
using bagwiz::core::video::VideoCodec;

std::vector<std::byte> bytes(std::initializer_list<int> values)
{
  std::vector<std::byte> out;
  for (int v : values) {
    out.push_back(static_cast<std::byte>(v));
  }
  return out;
}

}  // namespace

TEST(AnnexBTest, CountsH264NalUnitsBehindBothStartCodeLengths)
{
  // 4-byte start code + SPS (type 7), 3-byte start code + PPS (type 8),
  // 4-byte start code + IDR slice (type 5).
  const auto data = bytes({0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x1E, 0x00, 0x00, 0x01, 0x68,
                           0xCE, 0x38, 0x80, 0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00});
  const auto s = summarize_annexb(data, VideoCodec::kH264);
  EXPECT_TRUE(s.starts_with_start_code);
  EXPECT_EQ(s.nal_count, 3U);
  EXPECT_TRUE(s.has_sps);
  EXPECT_TRUE(s.has_pps);
  EXPECT_FALSE(s.has_vps);  // H.264 has no VPS
  EXPECT_TRUE(s.has_keyframe_slice);
}

TEST(AnnexBTest, H264NonIdrSliceIsNotAKeyframe)
{
  const auto data = bytes({0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x00});  // type 1
  const auto s = summarize_annexb(data, VideoCodec::kH264);
  EXPECT_EQ(s.nal_count, 1U);
  EXPECT_FALSE(s.has_sps);
  EXPECT_FALSE(s.has_pps);
  EXPECT_FALSE(s.has_keyframe_slice);
}

TEST(AnnexBTest, ClassifiesH265ParameterSetsAndIrap)
{
  // HEVC NAL header is two bytes; the type sits in bits 1..6 of the first.
  // VPS 32 -> 0x40, SPS 33 -> 0x42, PPS 34 -> 0x44, IDR_W_RADL 19 -> 0x26,
  // TRAIL_R 1 -> 0x02.
  const auto data =
    bytes({0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x42, 0x01, 0x01,
           0x00, 0x00, 0x00, 0x01, 0x44, 0x01, 0xC0, 0x00, 0x00, 0x01, 0x26, 0x01, 0xAF});
  const auto s = summarize_annexb(data, VideoCodec::kH265);
  EXPECT_EQ(s.nal_count, 4U);
  EXPECT_TRUE(s.has_vps);
  EXPECT_TRUE(s.has_sps);
  EXPECT_TRUE(s.has_pps);
  EXPECT_TRUE(s.has_keyframe_slice);

  const auto trail = bytes({0x00, 0x00, 0x01, 0x02, 0x01, 0xD0});
  const auto t = summarize_annexb(trail, VideoCodec::kH265);
  EXPECT_EQ(t.nal_count, 1U);
  EXPECT_FALSE(t.has_keyframe_slice);
  EXPECT_FALSE(t.has_sps);
}

TEST(AnnexBTest, DataWithoutStartCodeIsFlagged)
{
  // An AVCC-style length prefix instead of a start code.
  const auto data = bytes({0x00, 0x00, 0x00, 0x05, 0x65, 0x88, 0x84, 0x00, 0x11});
  const auto s = summarize_annexb(data, VideoCodec::kH264);
  EXPECT_FALSE(s.starts_with_start_code);
  EXPECT_EQ(s.nal_count, 0U);
  EXPECT_FALSE(s.has_keyframe_slice);

  const auto empty = summarize_annexb({}, VideoCodec::kH264);
  EXPECT_FALSE(empty.starts_with_start_code);
  EXPECT_EQ(empty.nal_count, 0U);
}

TEST(AnnexBTest, TrailingStartCodeWithoutPayloadIsIgnored)
{
  const auto data = bytes({0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x00, 0x00, 0x01});
  const auto s = summarize_annexb(data, VideoCodec::kH264);
  EXPECT_EQ(s.nal_count, 1U);
  EXPECT_TRUE(s.has_sps);
}
