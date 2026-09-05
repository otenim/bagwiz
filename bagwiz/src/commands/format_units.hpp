// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__FORMAT_UNITS_HPP_
#define COMMANDS__FORMAT_UNITS_HPP_

#include <fmt/core.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

// Human-readable renderings of byte counts and durations, shared by the
// commands that print them (`du`, `info`) so a size or a span of time reads
// the same everywhere on the CLI.
namespace bagwiz::commands
{

// Raw byte count, or a 1024-based human-readable rendering in the style of
// `du -h`: values below 1 KiB stay raw bytes, everything above prints one
// decimal and a K/M/G/T/P/E suffix ("4.0K", "1.2M").
inline std::string format_size(std::uint64_t bytes, bool human)
{
  if (!human) {
    return fmt::format("{}", bytes);
  }
  constexpr std::array<char, 6> kSuffixes{'K', 'M', 'G', 'T', 'P', 'E'};
  double value = static_cast<double>(bytes);
  std::size_t idx = 0;
  while (value >= 1024.0 && idx < kSuffixes.size()) {
    value /= 1024.0;
    ++idx;
  }
  if (idx == 0) {
    return fmt::format("{}", bytes);
  }
  return fmt::format("{:.1f}{}", value, kSuffixes[idx - 1]);
}

// A span of time in nanoseconds as hours, minutes and seconds with
// millisecond precision, dropping the leading units that are zero: "2.500s",
// "1m 02.500s", "1h 02m 03.456s". A negative span (an end before its start)
// renders as "0.000s".
inline std::string format_duration(std::int64_t ns)
{
  constexpr std::int64_t kNanosPerSecond = 1'000'000'000;
  constexpr std::int64_t kSecondsPerMinute = 60;
  constexpr std::int64_t kSecondsPerHour = 3600;
  if (ns < 0) {
    ns = 0;
  }
  const std::int64_t whole_seconds = ns / kNanosPerSecond;
  const double seconds =
    static_cast<double>(whole_seconds % kSecondsPerMinute) +
    static_cast<double>(ns % kNanosPerSecond) / static_cast<double>(kNanosPerSecond);
  const std::int64_t minutes = (whole_seconds / kSecondsPerMinute) % kSecondsPerMinute;
  const std::int64_t hours = whole_seconds / kSecondsPerHour;
  if (hours > 0) {
    return fmt::format("{}h {:02}m {:06.3f}s", hours, minutes, seconds);
  }
  if (minutes > 0) {
    return fmt::format("{}m {:06.3f}s", minutes, seconds);
  }
  return fmt::format("{:.3f}s", seconds);
}

}  // namespace bagwiz::commands

#endif  // COMMANDS__FORMAT_UNITS_HPP_
