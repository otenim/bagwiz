// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__STAMP_WIRE_HPP_
#define BAGWIZ__CORE__IMAGE__STAMP_WIRE_HPP_

#include <cstdint>

// Conversion between the nanosecond stamps bagwiz carries around and the
// builtin_interfaces/Time pair (int32 sec, uint32 nanosec) the image and
// video messages serialize.
namespace bagwiz::core::image
{

inline constexpr std::int64_t kNanosPerSecond = 1'000'000'000LL;

struct WireStamp
{
  std::int32_t sec = 0;
  std::uint32_t nanosec = 0;
};

// Split a nanosecond stamp with floor semantics, so nanosec stays in
// [0, 1e9) for negative stamps too: -1.7 s -> {sec: -2, nanosec: 3e8}, as
// builtin_interfaces/Time specifies.
[[nodiscard]] constexpr WireStamp split_stamp_ns(std::int64_t stamp_ns) noexcept
{
  std::int64_t sec = stamp_ns / kNanosPerSecond;
  std::int64_t nanosec = stamp_ns % kNanosPerSecond;
  if (nanosec < 0) {
    nanosec += kNanosPerSecond;
    sec -= 1;
  }
  return WireStamp{static_cast<std::int32_t>(sec), static_cast<std::uint32_t>(nanosec)};
}

[[nodiscard]] constexpr std::int64_t join_stamp_ns(std::int32_t sec, std::uint32_t nanosec) noexcept
{
  return static_cast<std::int64_t>(sec) * kNanosPerSecond + static_cast<std::int64_t>(nanosec);
}

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__STAMP_WIRE_HPP_
