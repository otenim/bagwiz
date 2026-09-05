// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IO__PRORATE_BYTES_HPP_
#define IO__PRORATE_BYTES_HPP_

#include <cmath>
#include <cstdint>

// Src-local building block of `bagwiz du`: the one rounding rule behind every
// "share of a compressed container" figure it reports.
namespace bagwiz::io::detail
{

// A topic's share of a compressed container: its `bytes` inside the container
// scaled by the container's compression ratio (`compressed` over
// `uncompressed`), to the nearest byte. A container that did not shrink
// (equal sizes), or whose uncompressed size is unknown (zero), charges the
// bytes exactly. Per-container rounding keeps every share an integer and the
// result independent of the order containers are summed in.
inline std::uint64_t prorate_bytes(
  std::uint64_t bytes, std::uint64_t compressed, std::uint64_t uncompressed)
{
  if (compressed == uncompressed || uncompressed == 0) {
    return bytes;
  }
  return static_cast<std::uint64_t>(std::llround(
    static_cast<long double>(bytes) * static_cast<long double>(compressed) /
    static_cast<long double>(uncompressed)));
}

}  // namespace bagwiz::io::detail

#endif  // IO__PRORATE_BYTES_HPP_
