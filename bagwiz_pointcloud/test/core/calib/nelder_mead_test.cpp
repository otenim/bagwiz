// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/nelder_mead.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace calib = bagwiz::core::calib;

TEST(NelderMeadTest, MinimizesShiftedQuadraticBowl)
{
  const auto f = [](std::span<const double> x) {
    const double a = x[0] - 1.0, b = x[1] + 2.0;
    return a * a + 3.0 * b * b;
  };
  calib::NelderMeadParams params;
  params.init_step = {0.5, 0.5};
  const std::array<double, 2> x0{0.0, 0.0};
  const auto result = calib::nelder_mead_minimize(f, x0, params);
  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.x[0], 1.0, 1e-3);
  EXPECT_NEAR(result.x[1], -2.0, 1e-3);
}

TEST(NelderMeadTest, RespectsInfiniteWall)
{
  // Minimum of (x-1)^2 subject to x <= 0.5 modeled as +inf beyond the wall:
  // the simplex must settle at the wall, not cross it.
  const auto f = [](std::span<const double> x) {
    if (x[0] > 0.5) {
      return std::numeric_limits<double>::infinity();
    }
    const double a = x[0] - 1.0;
    return a * a;
  };
  calib::NelderMeadParams params;
  params.init_step = {0.2};
  const std::array<double, 1> x0{0.0};
  const auto result = calib::nelder_mead_minimize(f, x0, params);
  EXPECT_LE(result.x[0], 0.5 + 1e-9);
  EXPECT_NEAR(result.x[0], 0.5, 1e-2);
}

TEST(NelderMeadTest, StopsAtIterationCap)
{
  const auto f = [](std::span<const double> x) { return x[0]; };  // unbounded below
  calib::NelderMeadParams params;
  params.init_step = {1.0};
  params.max_iterations = 20;
  const std::array<double, 1> x0{0.0};
  const auto result = calib::nelder_mead_minimize(f, x0, params);
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.iterations, 20);
}
