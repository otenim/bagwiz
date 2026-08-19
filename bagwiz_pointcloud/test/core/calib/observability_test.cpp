// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/observability.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace calib = bagwiz::core::calib;

namespace
{

// The 2-norm of the difference of two vectors, for orthonormality checks.
double diff_norm(
  const std::vector<std::vector<double>> & a, std::size_t col_a,
  const std::vector<std::vector<double>> & b, std::size_t col_b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    sum += (a[i][col_a] - b[i][col_b]) * (a[i][col_a] - b[i][col_b]);
  }
  return std::sqrt(sum);
}

double dot(const std::vector<std::vector<double>> & m, std::size_t ca, std::size_t cb)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < m.size(); ++i) {
    sum += m[i][ca] * m[i][cb];
  }
  return sum;
}

}  // namespace

TEST(ObservabilityTest, JacobiEigenOfDiagonalMatrix)
{
  const std::vector<std::vector<double>> m{{3.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 2.0}};
  const auto es = calib::jacobi_eigen(m);
  ASSERT_EQ(es.values.size(), 3U);
  // Eigenvalues come back ascending.
  EXPECT_NEAR(es.values[0], 1.0, 1e-12);
  EXPECT_NEAR(es.values[1], 2.0, 1e-12);
  EXPECT_NEAR(es.values[2], 3.0, 1e-12);
  // Eigenvectors are orthonormal (columns).
  for (std::size_t j = 0; j < 3; ++j) {
    EXPECT_NEAR(dot(es.vectors, j, j), 1.0, 1e-9);
    for (std::size_t k = j + 1; k < 3; ++k) {
      EXPECT_NEAR(dot(es.vectors, j, k), 0.0, 1e-9);
    }
  }
}

TEST(ObservabilityTest, JacobiEigenRecoversRotatedBasis)
{
  // A 2x2 with eigen-directions at 45 deg: [[2, 1], [1, 2]] has eigenvalues
  // 1 (along (1,-1)/sqrt2) and 3 (along (1,1)/sqrt2).
  const std::vector<std::vector<double>> m{{2.0, 1.0}, {1.0, 2.0}};
  const auto es = calib::jacobi_eigen(m);
  ASSERT_EQ(es.values.size(), 2U);
  EXPECT_NEAR(es.values[0], 1.0, 1e-12);
  EXPECT_NEAR(es.values[1], 3.0, 1e-12);
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  EXPECT_NEAR(std::abs(es.vectors[0][0]), inv_sqrt2, 1e-9);
  EXPECT_NEAR(std::abs(es.vectors[1][0]), inv_sqrt2, 1e-9);
  // The two eigenvectors are orthogonal and span opposite diagonals.
  EXPECT_NEAR(es.vectors[0][0] * es.vectors[1][0], -inv_sqrt2 * inv_sqrt2, 1e-9);
  EXPECT_NEAR(es.vectors[0][1] * es.vectors[1][1], inv_sqrt2 * inv_sqrt2, 1e-9);
}

TEST(ObservabilityTest, JacobiEigenReconstructsMatrix)
{
  // V * diag(w) * V^T reproduces the input for an arbitrary symmetric 4x4.
  const std::vector<std::vector<double>> m{
    {4.0, 1.0, 0.5, 0.0}, {1.0, 3.0, 0.25, 0.75}, {0.5, 0.25, 2.0, 1.0}, {0.0, 0.75, 1.0, 1.0}};
  const auto es = calib::jacobi_eigen(m);
  for (std::size_t i = 0; i < m.size(); ++i) {
    for (std::size_t j = 0; j < m.size(); ++j) {
      double v = 0.0;
      for (std::size_t k = 0; k < es.values.size(); ++k) {
        v += es.vectors[i][k] * es.values[k] * es.vectors[j][k];
      }
      EXPECT_NEAR(v, m[i][j], 1e-9) << i << "," << j;
    }
  }
}

TEST(ObservabilityTest, PairedCurvatureComputesMeanAndStdError)
{
  // Three samples, center 1.0 each; plus/minus vary so the per-sample
  // curvatures are 0.2, 0.4, 0.6: mean 0.4, sample stddev sqrt(0.04)=0.2,
  // standard error 0.2/sqrt(3).
  const std::vector<std::optional<double>> center{1.0, 1.0, 1.0};
  const std::vector<std::optional<double>> plus{1.1, 1.3, 1.5};
  const std::vector<std::optional<double>> minus{1.1, 1.1, 1.1};
  const auto est = calib::paired_curvature(center, plus, minus);
  EXPECT_EQ(est.pairs, 3);
  EXPECT_NEAR(est.mean, 0.4, 1e-12);
  EXPECT_NEAR(est.std_error, 0.2 / std::sqrt(3.0), 1e-12);
}

TEST(ObservabilityTest, PairedCurvatureSkipsSamplesInvalidAtAnyPoint)
{
  const std::vector<std::optional<double>> center{1.0, std::nullopt, 1.0};
  const std::vector<std::optional<double>> plus{1.5, 1.5, std::nullopt};
  const std::vector<std::optional<double>> minus{1.5, 1.5, 1.5};
  const auto est = calib::paired_curvature(center, plus, minus);
  // Only the first sample is valid at all three evaluation points.
  EXPECT_EQ(est.pairs, 1);
  EXPECT_NEAR(est.mean, 1.0, 1e-12);
  EXPECT_EQ(est.std_error, 0.0);  // a single pair carries no spread
}

TEST(ObservabilityTest, CurvatureSignificanceUsesNoiseAndFloor)
{
  // Above the absolute floor with no spread information: significant.
  const auto loud = calib::paired_curvature(
    std::vector<std::optional<double>>{1.0}, std::vector<std::optional<double>>{1.1},
    std::vector<std::optional<double>>{1.1});
  EXPECT_TRUE(calib::curvature_significant(loud));
  // A tiny curvature below the absolute floor is not significant even with
  // zero measured spread.
  const auto faint = calib::paired_curvature(
    std::vector<std::optional<double>>{1.0}, std::vector<std::optional<double>>{1.0 + 1e-7},
    std::vector<std::optional<double>>{1.0 + 1e-7});
  EXPECT_FALSE(calib::curvature_significant(faint));
  // Above the floor but inside the noise band: not significant.
  calib::CurvatureEstimate noisy;
  noisy.mean = 1e-3;
  noisy.std_error = 1e-3;
  noisy.pairs = 8;
  EXPECT_FALSE(calib::curvature_significant(noisy));
  noisy.mean = 1e-2;  // 10x the standard error
  EXPECT_TRUE(calib::curvature_significant(noisy));
}
