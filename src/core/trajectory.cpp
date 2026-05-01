// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/trajectory.hpp"

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <ios>
#include <ostream>
#include <span>

namespace bagwiz::core
{

void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses)
{
  const auto prev_flags = os.flags();
  const auto prev_prec = os.precision();
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(9);
  for (const auto & p : poses) {
    // Format the timestamp from the integer ns value directly. Going
    // through double would round to the nearest representable value;
    // around year-2026 magnitudes (~1.77e18 ns) the double ULP is ~256,
    // so 9-digit output silently drifts from the source header.stamp.
    const std::int64_t ns = p.timestamp_ns;
    const std::int64_t sec = ns / 1'000'000'000LL;
    const std::int64_t nsec = ns % 1'000'000'000LL;
    char ts_buf[32];
    std::snprintf(ts_buf, sizeof(ts_buf), "%" PRId64 ".%09" PRId64, sec, nsec);
    os << ts_buf << ' ' << p.tx << ' ' << p.ty << ' ' << p.tz << ' ' << p.qx << ' ' << p.qy << ' '
       << p.qz << ' ' << p.qw << '\n';
  }
  os.flags(prev_flags);
  os.precision(prev_prec);
}

}  // namespace bagwiz::core
