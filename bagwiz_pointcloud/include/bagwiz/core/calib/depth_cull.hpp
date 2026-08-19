// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_
#define BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_

#include <cstdint>
#include <span>
#include <vector>

namespace bagwiz::core::calib
{

// Keep flags (1 = keep) parallel to the inputs. u/v are pixel coordinates
// already verified in [0,width) x [0,height); depth is meters.
[[nodiscard]] std::vector<std::uint8_t> depth_cull_keep(
  std::span<const float> u, std::span<const float> v, std::span<const float> depth,
  std::uint32_t width, std::uint32_t height, std::uint32_t cell_px, float margin_m);

}  // namespace bagwiz::core::calib

#endif  // BAGWIZ__CORE__CALIB__DEPTH_CULL_HPP_
