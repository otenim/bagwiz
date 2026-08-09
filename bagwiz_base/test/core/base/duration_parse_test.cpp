// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/duration_parse.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace
{
using bagwiz::core::DurationUnitPolicy;
using bagwiz::core::parse_duration_ns;
using bagwiz::core::parse_period_ns;
}  // namespace

TEST(ParseDurationNs, UnitSuffixes)
{
  EXPECT_EQ(parse_duration_ns("50ms"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("500ns"), 500);
  EXPECT_EQ(parse_duration_ns("2us"), 2'000);
  EXPECT_EQ(parse_duration_ns("µs"), std::nullopt);  // unit alone, no number
  EXPECT_EQ(parse_duration_ns("3µs"), 3'000);        // micro sign
  EXPECT_EQ(parse_duration_ns("1s"), 1'000'000'000);
  EXPECT_EQ(parse_duration_ns("0.05s"), 50'000'000);
}

TEST(ParseDurationNs, NoUnitDefaultsToMilliseconds)
{
  EXPECT_EQ(parse_duration_ns("50"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("1.5"), 1'500'000);
}

TEST(ParseDurationNs, RequireUnitRejectsBareNumbers)
{
  EXPECT_EQ(parse_duration_ns("50", DurationUnitPolicy::RequireUnit), std::nullopt);
  EXPECT_EQ(parse_duration_ns("1.5", DurationUnitPolicy::RequireUnit), std::nullopt);
  EXPECT_EQ(parse_duration_ns(" 5 ", DurationUnitPolicy::RequireUnit), std::nullopt);
  EXPECT_EQ(parse_duration_ns("-5", DurationUnitPolicy::RequireUnit), std::nullopt);
}

TEST(ParseDurationNs, RequireUnitAcceptsExplicitUnits)
{
  EXPECT_EQ(parse_duration_ns("50ms", DurationUnitPolicy::RequireUnit), 50'000'000);
  EXPECT_EQ(parse_duration_ns("5s", DurationUnitPolicy::RequireUnit), 5'000'000'000);
  EXPECT_EQ(parse_duration_ns("3µs", DurationUnitPolicy::RequireUnit), 3'000);
  EXPECT_EQ(parse_duration_ns("500ns", DurationUnitPolicy::RequireUnit), 500);
  EXPECT_EQ(parse_duration_ns("0.05s", DurationUnitPolicy::RequireUnit), 50'000'000);
}

TEST(ParseDurationNs, RequireUnitStillRejectsGarbage)
{
  EXPECT_EQ(parse_duration_ns("", DurationUnitPolicy::RequireUnit), std::nullopt);
  EXPECT_EQ(parse_duration_ns("abc", DurationUnitPolicy::RequireUnit), std::nullopt);
  EXPECT_EQ(parse_duration_ns("50min", DurationUnitPolicy::RequireUnit), std::nullopt);
}

TEST(ParseDurationNs, SignedAndFractional)
{
  EXPECT_EQ(parse_duration_ns("-50ms"), -50'000'000);
  EXPECT_EQ(parse_duration_ns("+50ms"), 50'000'000);
  EXPECT_EQ(parse_duration_ns("1.5ms"), 1'500'000);
  EXPECT_EQ(parse_duration_ns("-500ns"), -500);
}

TEST(ParseDurationNs, WhitespaceTolerated)
{
  EXPECT_EQ(parse_duration_ns("  50ms  "), 50'000'000);
  EXPECT_EQ(parse_duration_ns("50 ms"), 50'000'000);
}

TEST(ParseDurationNs, RejectsGarbage)
{
  EXPECT_EQ(parse_duration_ns(""), std::nullopt);
  EXPECT_EQ(parse_duration_ns("abc"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("50min"), std::nullopt);  // unknown unit
  EXPECT_EQ(parse_duration_ns("5x"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("ms"), std::nullopt);  // no number
}

TEST(ParseDurationNs, RejectsNonFinite)
{
  EXPECT_EQ(parse_duration_ns("nan"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("inf"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("infinity"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("-inf"), std::nullopt);
  EXPECT_EQ(parse_duration_ns("1e400ms"), std::nullopt);  // strtod overflows to +inf
  EXPECT_EQ(parse_duration_ns("1e30s"), std::nullopt);    // finite but out of int64 ns range
}

TEST(ParsePeriodNs, TimeUnitsAreTakenAsThePeriod)
{
  EXPECT_EQ(parse_period_ns("10ms"), 10'000'000);
  EXPECT_EQ(parse_period_ns("0.01s"), 10'000'000);
  EXPECT_EQ(parse_period_ns("10000us"), 10'000'000);
  EXPECT_EQ(parse_period_ns("10000µs"), 10'000'000);
  EXPECT_EQ(parse_period_ns("10000000ns"), 10'000'000);
}

TEST(ParsePeriodNs, HzIsInvertedIntoAPeriod)
{
  EXPECT_EQ(parse_period_ns("100hz"), 10'000'000);
  EXPECT_EQ(parse_period_ns("10.0hz"), 100'000'000);
  EXPECT_EQ(parse_period_ns("1hz"), 1'000'000'000);
  EXPECT_EQ(parse_period_ns("0.5hz"), 2'000'000'000);
  // 3 Hz is not exactly representable in integer nanoseconds; it rounds.
  EXPECT_EQ(parse_period_ns("3hz"), 333'333'333);
}

TEST(ParsePeriodNs, PeriodAndFrequencySpellingsAgree)
{
  EXPECT_EQ(parse_period_ns("100hz"), parse_period_ns("10ms"));
  EXPECT_EQ(parse_period_ns("200hz"), parse_period_ns("5ms"));
}

TEST(ParsePeriodNs, WhitespaceTolerated)
{
  EXPECT_EQ(parse_period_ns("  100hz  "), 10'000'000);
  EXPECT_EQ(parse_period_ns("100 hz"), 10'000'000);
  EXPECT_EQ(parse_period_ns("  10 ms "), 10'000'000);
}

TEST(ParsePeriodNs, UnitIsMandatory)
{
  EXPECT_EQ(parse_period_ns("10"), std::nullopt);
  EXPECT_EQ(parse_period_ns("100"), std::nullopt);
  EXPECT_EQ(parse_period_ns(" 10 "), std::nullopt);
}

TEST(ParsePeriodNs, UnitsAreCaseSensitive)
{
  EXPECT_EQ(parse_period_ns("100HZ"), std::nullopt);
  EXPECT_EQ(parse_period_ns("100Hz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("10MS"), std::nullopt);
  EXPECT_EQ(parse_period_ns("10Ms"), std::nullopt);
  EXPECT_EQ(parse_period_ns("1S"), std::nullopt);
}

TEST(ParsePeriodNs, RejectsNonPositive)
{
  EXPECT_EQ(parse_period_ns("0ms"), std::nullopt);
  EXPECT_EQ(parse_period_ns("0hz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("-10ms"), std::nullopt);
  EXPECT_EQ(parse_period_ns("-100hz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("+10ms"), 10'000'000);  // an explicit plus stays valid
}

TEST(ParsePeriodNs, RejectsPeriodsThatRoundToZero)
{
  EXPECT_EQ(parse_period_ns("1e12hz"), std::nullopt);  // 0.001 ns
  EXPECT_EQ(parse_period_ns("0.4ns"), std::nullopt);   // rounds to 0 ns
}

TEST(ParsePeriodNs, RejectsGarbage)
{
  EXPECT_EQ(parse_period_ns(""), std::nullopt);
  EXPECT_EQ(parse_period_ns("abc"), std::nullopt);
  EXPECT_EQ(parse_period_ns("hz"), std::nullopt);     // no number
  EXPECT_EQ(parse_period_ns("10khz"), std::nullopt);  // unsupported unit
  EXPECT_EQ(parse_period_ns("10ms5"), std::nullopt);  // trailing garbage
  EXPECT_EQ(parse_period_ns("10hz5"), std::nullopt);
  EXPECT_EQ(parse_period_ns("50min"), std::nullopt);
}

TEST(ParsePeriodNs, RejectsNonFinite)
{
  EXPECT_EQ(parse_period_ns("nanhz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("infhz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("1e400hz"), std::nullopt);
  EXPECT_EQ(parse_period_ns("1e30s"), std::nullopt);  // out of the int64 ns range
}
