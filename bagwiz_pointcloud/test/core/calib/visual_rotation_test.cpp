// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/visual_rotation.hpp"

#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/image/camera_distortion.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace calib = bagwiz::core::calib;
namespace image = bagwiz::core::image;

namespace
{

constexpr std::uint32_t kW = 640;
constexpr std::uint32_t kH = 480;
constexpr double kF = 500.0;

calib::CameraModel pinhole()
{
  calib::CameraModel cam;
  cam.k = {kF, 0.0, 320.0, 0.0, kF, 240.0, 0.0, 0.0, 1.0};
  cam.model = image::DistortionModel::kNone;
  cam.width = kW;
  cam.height = kH;
  return cam;
}

calib::CameraModel plumb_bob()
{
  calib::CameraModel cam = pinhole();
  cam.model = image::DistortionModel::kPlumbBob;
  cam.d = {-0.12, 0.03, 0.0, 0.0, 0.0};
  return cam;
}

// A textured scene: a few hundred soft blobs of random size and brightness
// on a mid-gray ground, plus a little noise. Trackable everywhere, no
// repeated structure.
calib::GrayImage make_texture(std::uint64_t seed)
{
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> ux(0.0, kW);
  std::uniform_real_distribution<double> uy(0.0, kH);
  std::uniform_real_distribution<double> usig(1.5, 6.0);
  std::uniform_real_distribution<double> amplitude_dist(-90.0, 90.0);
  std::vector<double> acc(static_cast<std::size_t>(kW) * kH, 128.0);
  for (int b = 0; b < 700; ++b) {
    const double cx = ux(rng);
    const double cy = uy(rng);
    const double sig = usig(rng);
    const double amp = amplitude_dist(rng);
    const int r = static_cast<int>(3.0 * sig) + 1;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int x = static_cast<int>(cx) + dx;
        const int y = static_cast<int>(cy) + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(kW) || y >= static_cast<int>(kH)) {
          continue;
        }
        const double ddx = x - cx;
        const double ddy = y - cy;
        acc[static_cast<std::size_t>(y) * kW + static_cast<std::size_t>(x)] +=
          amp * std::exp(-(ddx * ddx + ddy * ddy) / (2.0 * sig * sig));
      }
    }
  }
  std::normal_distribution<double> noise(0.0, 1.5);
  calib::GrayImage img;
  img.width = kW;
  img.height = kH;
  img.gray.resize(acc.size());
  for (std::size_t i = 0; i < acc.size(); ++i) {
    img.gray[i] = static_cast<std::uint8_t>(std::clamp(acc[i] + noise(rng), 0.0, 255.0));
  }
  return img;
}

using Mat3 = std::array<std::array<double, 3>, 3>;

Mat3 rotation_from_rotvec(const std::array<double, 3> & r)
{
  const double a = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  Mat3 R{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  if (a < 1e-15) {
    return R;
  }
  const double kx = r[0] / a;
  const double ky = r[1] / a;
  const double kz = r[2] / a;
  const double c = std::cos(a);
  const double s = std::sin(a);
  const double v = 1.0 - c;
  R = {
    {{c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s},
     {ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s},
     {kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v}}};
  return R;
}

std::array<double, 3> rotate_vec(const Mat3 & m, const std::array<double, 3> & v)
{
  return {
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

double sample_bilinear(const calib::GrayImage & img, double x, double y)
{
  if (x < 0.0 || y < 0.0 || x >= img.width - 1.0 || y >= img.height - 1.0) {
    return 128.0;
  }
  const auto x0 = static_cast<std::size_t>(x);
  const auto y0 = static_cast<std::size_t>(y);
  const double fx = x - static_cast<double>(x0);
  const double fy = y - static_cast<double>(y0);
  const auto at = [&](std::size_t xx, std::size_t yy) {
    return static_cast<double>(img.gray[yy * img.width + xx]);
  };
  return at(x0, y0) * (1 - fx) * (1 - fy) + at(x0 + 1, y0) * fx * (1 - fy) +
         at(x0, y0 + 1) * (1 - fx) * fy + at(x0 + 1, y0 + 1) * fx * fy;
}

// Render the view after the camera rotated by `rotvec` (optical frame of the
// source view) and, for a planar scene `plane_n . x = plane_d` in the source
// camera (plane_d = 0 means a far scene: pure rotation, translation ignored),
// translated by `t`. For each destination pixel the ray is lifted
// (undistorted), expressed in the source camera, intersected with the plane
// when one is given, projected and distorted into the source image, and
// sampled there. Exact for a planar/far scene, so the recovered rotation has
// a known truth.
struct Plane
{
  std::array<double, 3> n;  // unit normal, source camera frame
  double d;                 // n . x = d
};

calib::GrayImage render_view(
  const calib::GrayImage & src, const calib::CameraModel & cam,
  const std::array<double, 3> & rotvec, const std::array<double, 3> & t,
  const std::vector<Plane> & planes)
{
  const Mat3 R = rotation_from_rotvec(rotvec);  // x_src = R x_dst
  calib::GrayImage out;
  out.width = src.width;
  out.height = src.height;
  out.gray.resize(src.gray.size());
  const double fx = cam.k[0];
  const double fy = cam.k[4];
  const double cx = cam.k[2];
  const double cy = cam.k[5];
  for (std::uint32_t v = 0; v < out.height; ++v) {
    for (std::uint32_t u = 0; u < out.width; ++u) {
      const auto n = image::invert_distortion_normalized(
        (static_cast<double>(u) - cx) / fx, (static_cast<double>(v) - cy) / fy, cam.model, cam.d);
      std::array<double, 3> ray{n.x, n.y, 1.0};  // in the destination camera
      std::array<double, 3> p;
      if (!planes.empty()) {
        // Destination camera at pose (R, t) in the source frame: the point
        // along the ray is t + s * R * ray; the scene is the nearest plane hit
        // in front of the camera (a creased surface, so translation has real,
        // non-uniform parallax).
        const auto dir = rotate_vec(R, ray);
        double best_s = std::numeric_limits<double>::infinity();
        for (const auto & pl : planes) {
          const double n_dot_dir = pl.n[0] * dir[0] + pl.n[1] * dir[1] + pl.n[2] * dir[2];
          const double n_dot_t = pl.n[0] * t[0] + pl.n[1] * t[1] + pl.n[2] * t[2];
          if (std::abs(n_dot_dir) < 1e-12) {
            continue;
          }
          const double s = (pl.d - n_dot_t) / n_dot_dir;
          if (s > 0.0 && s < best_s) {
            best_s = s;
          }
        }
        const double s = std::isfinite(best_s) ? best_s : 1.0;
        p = {t[0] + s * dir[0], t[1] + s * dir[1], t[2] + s * dir[2]};
      } else {
        p = rotate_vec(R, ray);
      }
      const auto d = image::distort_normalized(p[0] / p[2], p[1] / p[2], cam.model, cam.d);
      const double sx = d.x * fx + cx;
      const double sy = d.y * fy + cy;
      out.gray[static_cast<std::size_t>(v) * out.width + u] =
        static_cast<std::uint8_t>(std::clamp(sample_bilinear(src, sx, sy), 0.0, 255.0));
    }
  }
  return out;
}

double deg(double rad)
{
  return rad * 180.0 / M_PI;
}

void expect_rotvec_near(
  const std::optional<std::array<double, 3>> & got, const std::array<double, 3> & want,
  double tol_deg)
{
  ASSERT_TRUE(got.has_value());
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_NEAR(deg((*got)[k]), deg(want[k]), tol_deg) << "axis " << k;
  }
}

}  // namespace

TEST(VisualRotationTest, PureRotationRecoveredByBothSolversOnFarScene)
{
  const auto cam = pinhole();
  const auto prev = make_texture(1);
  const std::array<double, 3> rot{0.3 * M_PI / 180.0, -0.5 * M_PI / 180.0, 0.2 * M_PI / 180.0};
  const auto next = render_view(prev, cam, rot, {0, 0, 0}, {});
  const auto r = calib::frame_pair_rotation(prev, next, cam, calib::VisualRotationParams{});
  EXPECT_GT(r.tracks, 300U);
  EXPECT_GT(r.median_flow_px, 2.0);
  expect_rotvec_near(r.pure_rotation, rot, 0.03);
  EXPECT_GT(r.inliers_rotation, 200U);
}

TEST(VisualRotationTest, EssentialMatrixRecoversRotationWithTranslation)
{
  // A creased scene — a wall 1 m ahead meeting a slanted floor — seen while
  // the camera moves 4 cm sideways and 6 cm forward and turns. The depth
  // structure gives the translation real, non-uniform parallax, so the
  // essential matrix separates it from the rotation and returns the
  // rotation; the pure-rotation model is biased by it (not asserted). A
  // single fronto-parallel plane would NOT do: its translational flow is
  // uniform and reads as a rotation to any solver, and a single plane of any
  // tilt leaves the essential matrix its two-fold planar ambiguity.
  const auto cam = pinhole();
  const auto prev = make_texture(2);
  const std::array<double, 3> rot{-0.2 * M_PI / 180.0, 0.6 * M_PI / 180.0, 0.1 * M_PI / 180.0};
  const auto next = render_view(
    prev, cam, rot, {0.04, 0.0, 0.06}, {Plane{{0.0, 0.0, 1.0}, 1.0}, Plane{{0.0, -0.8, 0.6}, 0.5}});
  const auto r = calib::frame_pair_rotation(prev, next, cam, calib::VisualRotationParams{});
  EXPECT_GT(r.tracks, 300U);
  expect_rotvec_near(r.essential, rot, 0.08);
  EXPECT_GT(r.inliers_essential, 150U);
}

TEST(VisualRotationTest, IdentityPairReadsZero)
{
  const auto cam = pinhole();
  const auto prev = make_texture(3);
  const auto r = calib::frame_pair_rotation(prev, prev, cam, calib::VisualRotationParams{});
  EXPECT_GT(r.tracks, 300U);
  expect_rotvec_near(r.pure_rotation, {0, 0, 0}, 0.01);
  EXPECT_LT(r.median_flow_px, 0.1);
}

TEST(VisualRotationTest, DistortedCameraIsUndistortedBeforeSolving)
{
  const auto cam = plumb_bob();
  const auto prev = make_texture(4);
  const std::array<double, 3> rot{0.4 * M_PI / 180.0, 0.3 * M_PI / 180.0, -0.3 * M_PI / 180.0};
  const auto next = render_view(prev, cam, rot, {0, 0, 0}, {});
  const auto r = calib::frame_pair_rotation(prev, next, cam, calib::VisualRotationParams{});
  EXPECT_GT(r.tracks, 300U);
  expect_rotvec_near(r.pure_rotation, rot, 0.03);
}

TEST(VisualRotationTest, FeaturelessImageYieldsNoEstimate)
{
  const auto cam = pinhole();
  calib::GrayImage flat;
  flat.width = kW;
  flat.height = kH;
  flat.gray.assign(static_cast<std::size_t>(kW) * kH, 100);
  const auto r = calib::frame_pair_rotation(flat, flat, cam, calib::VisualRotationParams{});
  EXPECT_FALSE(r.essential.has_value());
  EXPECT_FALSE(r.pure_rotation.has_value());
  EXPECT_LT(r.tracks, 30U);
}

TEST(VisualRotationTest, DownscaleKeepsGeometryWithScaledModel)
{
  const auto cam = pinhole();
  const auto prev = make_texture(5);
  const std::array<double, 3> rot{0.0, 0.7 * M_PI / 180.0, 0.0};
  const auto next = render_view(prev, cam, rot, {0, 0, 0}, {});
  const double scale = calib::scale_for_max_side(kW, kH, 320);
  EXPECT_DOUBLE_EQ(scale, 0.5);
  EXPECT_DOUBLE_EQ(calib::scale_for_max_side(kW, kH, 2000), 1.0);
  const auto prev_s = calib::downscale_gray(prev, scale);
  const auto next_s = calib::downscale_gray(next, scale);
  EXPECT_EQ(prev_s.width, kW / 2);
  EXPECT_EQ(prev_s.height, kH / 2);
  const auto cam_s = calib::scale_camera_model(cam, scale);
  EXPECT_DOUBLE_EQ(cam_s.k[0], kF * 0.5);
  EXPECT_EQ(cam_s.width, kW / 2);
  const auto r = calib::frame_pair_rotation(prev_s, next_s, cam_s, calib::VisualRotationParams{});
  EXPECT_GT(r.tracks, 100U);
  expect_rotvec_near(r.pure_rotation, rot, 0.05);
}
