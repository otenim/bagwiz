// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::absolute_point_time_span_ns;
using bagwiz::core::pointcloud::find_point_time_field;
using bagwiz::core::pointcloud::point_time_seconds;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;

PointCloud2 cloud_with(std::vector<PointField> fields)
{
  PointCloud2 c;
  c.fields = std::move(fields);
  return c;
}

}  // namespace

TEST(PointTime, FindsRecognisedNameAndType)
{
  const auto c =
    cloud_with({{"x", 0, PointFieldType::kFloat32, 1}, {"time", 12, PointFieldType::kUint32, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 12u);
  EXPECT_EQ(tf->datatype, PointFieldType::kUint32);
}

TEST(PointTime, NamePrecedenceTBeatsTime)
{
  // Both "time" and "t" qualify; name precedence picks "t".
  const auto c =
    cloud_with({{"time", 4, PointFieldType::kFloat32, 1}, {"t", 8, PointFieldType::kFloat64, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 8u);  // "t" wins over "time"
  EXPECT_EQ(tf->datatype, PointFieldType::kFloat64);
}

TEST(PointTime, RejectsCountNotOne)
{
  const auto c = cloud_with({{"time", 0, PointFieldType::kUint32, 2}});
  EXPECT_FALSE(find_point_time_field(c).has_value());
}

TEST(PointTime, RejectsUnsupportedDatatypeAndFallsThrough)
{
  // "t" is INT32 (unsupported) -> skipped; "time" is UINT32 -> used.
  const auto c =
    cloud_with({{"t", 0, PointFieldType::kInt32, 1}, {"time", 4, PointFieldType::kUint32, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 4u);
  EXPECT_EQ(tf->datatype, PointFieldType::kUint32);
}

TEST(PointTime, NoTimeFieldIsNullopt)
{
  const auto c =
    cloud_with({{"x", 0, PointFieldType::kFloat32, 1}, {"ring", 4, PointFieldType::kUint16, 1}});
  EXPECT_FALSE(find_point_time_field(c).has_value());
}

TEST(PointTime, SecondsConversion)
{
  std::array<std::byte, 8> buf{};

  const std::uint32_t ns = 20'000'000;  // 20 ms
  std::memcpy(buf.data(), &ns, sizeof(ns));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kUint32}), 0.02, 1e-12);

  const float fs = 0.03F;
  std::memcpy(buf.data(), &fs, sizeof(fs));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kFloat32}), 0.03, 1e-6);

  const double ds = 1700.5;
  std::memcpy(buf.data(), &ds, sizeof(ds));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kFloat64}), 1700.5, 1e-12);
}

// [x y z time_stamp] as 4x float32 (point_step 16) with the given relative
// times in seconds.
namespace
{

PointCloud2 cloud_rel_times_f32(const std::vector<float> & rel_sec)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(rel_sec.size());
  c.point_step = 16;
  c.row_step = c.point_step * c.width;
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"time_stamp", 12, PointFieldType::kFloat32, 1},
  };
  c.data.resize(static_cast<std::size_t>(c.point_step) * c.width);
  for (std::size_t i = 0; i < rel_sec.size(); ++i) {
    std::memcpy(c.data.data() + i * c.point_step + 12, &rel_sec[i], sizeof(float));
  }
  return c;
}

}  // namespace

TEST(PointTime, AbsoluteSpanResolvesRelativeTimesAgainstTRef)
{
  auto c = cloud_rel_times_f32({0.01F, 0.09F});
  const auto field = find_point_time_field(c);
  ASSERT_TRUE(field.has_value());
  const auto span = absolute_point_time_span_ns(c, *field, 1'000'000'000);
  ASSERT_TRUE(span.has_value());
  EXPECT_NEAR(span->min_ns, 1'010'000'000, 10);  // float32 rounding of 0.01/0.09
  EXPECT_NEAR(span->max_ns, 1'090'000'000, 10);
}

TEST(PointTime, AbsoluteSpanKeepsEpochTimes)
{
  // FLOAT64 epoch-seconds field: classified absolute, t_ref is irrelevant.
  PointCloud2 c;
  c.height = 1;
  c.width = 2;
  c.point_step = 32;
  c.row_step = 64;
  c.fields = {
    {"x", 0, PointFieldType::kFloat64, 1},
    {"y", 8, PointFieldType::kFloat64, 1},
    {"z", 16, PointFieldType::kFloat64, 1},
    {"timestamp", 24, PointFieldType::kFloat64, 1},
  };
  c.data.resize(64);
  const double t0 = 1.7706783709e9;
  const double t1 = 1.7706783710e9;
  std::memcpy(c.data.data() + 24, &t0, sizeof(t0));
  std::memcpy(c.data.data() + 32 + 24, &t1, sizeof(t1));
  const auto field = find_point_time_field(c);
  ASSERT_TRUE(field.has_value());
  const auto span = absolute_point_time_span_ns(c, *field, /*t_ref_ns=*/0);
  ASSERT_TRUE(span.has_value());
  EXPECT_NEAR(span->min_ns, 1'770'678'370'900'000'000, 1000);
  EXPECT_NEAR(span->max_ns, 1'770'678'371'000'000'000, 1000);
}

TEST(PointTime, AbsoluteSpanSkipsNonFiniteTimes)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto c = cloud_rel_times_f32({nan, 0.05F});
  const auto field = find_point_time_field(c);
  ASSERT_TRUE(field.has_value());
  const auto span = absolute_point_time_span_ns(c, *field, 1'000'000'000);
  ASSERT_TRUE(span.has_value());
  EXPECT_NEAR(span->min_ns, 1'050'000'000, 10);
  EXPECT_NEAR(span->max_ns, 1'050'000'000, 10);

  auto all_nan = cloud_rel_times_f32({nan, nan});
  EXPECT_FALSE(absolute_point_time_span_ns(all_nan, *field, 1'000'000'000).has_value());
}

TEST(PointTime, AbsoluteSpanRejectsOutOfBoundsField)
{
  auto c = cloud_rel_times_f32({0.01F});
  // A field whose declared bytes run past point_step cannot be scanned.
  const bagwiz::core::pointcloud::PointTimeField bad{20, PointFieldType::kFloat32};
  EXPECT_FALSE(absolute_point_time_span_ns(c, bad, 1'000'000'000).has_value());
}
