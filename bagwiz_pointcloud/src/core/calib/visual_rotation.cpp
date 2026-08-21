// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/calib/visual_rotation.hpp"

#include "bagwiz/core/image/camera_distortion.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace bagwiz::core::calib
{
namespace
{

// Non-owning 8-bit view of a GrayImage for the OpenCV calls (all read-only).
cv::Mat as_mat(const GrayImage & g)
{
  return cv::Mat(
    static_cast<int>(g.height), static_cast<int>(g.width), CV_8UC1,
    const_cast<std::uint8_t *>(g.gray.data()));  // NOLINT(cppcoreguidelines-pro-type-const-cast)
}

std::array<double, 3> rotvec_of(const cv::Matx33d & r)
{
  cv::Vec3d v;
  cv::Rodrigues(r, v);
  return {v[0], v[1], v[2]};
}

// Kabsch: the rotation R with b_i ~ R a_i over the listed indices (unit
// bearing vectors).
cv::Matx33d kabsch(
  const std::vector<cv::Vec3d> & a, const std::vector<cv::Vec3d> & b,
  const std::vector<std::size_t> & idx)
{
  cv::Matx33d h = cv::Matx33d::zeros();
  for (const std::size_t i : idx) {
    h += a[i] * b[i].t();  // a b^T
  }
  cv::Matx33d u;
  cv::Matx33d vt;
  cv::Vec3d s;
  cv::SVD::compute(h, s, u, vt);
  // R = V diag(1, 1, det(V U^T)) U^T
  const cv::Matx33d v = vt.t();
  const double det = cv::determinant(v * u.t());
  const cv::Matx33d fix = cv::Matx33d::diag(cv::Vec3d(1.0, 1.0, det < 0.0 ? -1.0 : 1.0));
  return v * fix * u.t();
}

// sin of the angle between R a and b (small-angle residual).
double bearing_residual(const cv::Matx33d & r, const cv::Vec3d & a, const cv::Vec3d & b)
{
  const cv::Vec3d ra = r * a;
  return cv::norm(ra.cross(b));
}

}  // namespace

double scale_for_max_side(std::uint32_t width, std::uint32_t height, std::uint32_t max_side)
{
  const std::uint32_t side = std::max(width, height);
  if (side == 0 || side <= max_side) {
    return 1.0;
  }
  return static_cast<double>(max_side) / static_cast<double>(side);
}

GrayImage downscale_gray(const GrayImage & in, double scale)
{
  if (scale >= 1.0 || in.width == 0 || in.height == 0) {
    return in;
  }
  cv::Mat out;
  cv::resize(as_mat(in), out, cv::Size(), scale, scale, cv::INTER_AREA);
  GrayImage g;
  g.width = static_cast<std::uint32_t>(out.cols);
  g.height = static_cast<std::uint32_t>(out.rows);
  g.gray.resize(static_cast<std::size_t>(out.cols) * static_cast<std::size_t>(out.rows));
  for (int y = 0; y < out.rows; ++y) {
    std::copy_n(
      out.ptr<std::uint8_t>(y), out.cols,
      g.gray.begin() + static_cast<std::ptrdiff_t>(y) * out.cols);
  }
  return g;
}

CameraModel scale_camera_model(const CameraModel & cam, double scale)
{
  CameraModel out = cam;
  out.k[0] *= scale;
  out.k[4] *= scale;
  out.k[2] *= scale;
  out.k[5] *= scale;
  // cv::resize rounds the scaled size; match it so the model and the image agree.
  out.width = static_cast<std::uint32_t>(std::lround(static_cast<double>(cam.width) * scale));
  out.height = static_cast<std::uint32_t>(std::lround(static_cast<double>(cam.height) * scale));
  return out;
}

FramePairRotation frame_pair_rotation(
  const GrayImage & prev, const GrayImage & next, const CameraModel & cam,
  const VisualRotationParams & params)
{
  FramePairRotation out;
  if (
    prev.width == 0 || prev.height == 0 || prev.width != next.width || prev.height != next.height ||
    prev.gray.size() != next.gray.size()) {
    return out;
  }
  const cv::Mat img0 = as_mat(prev);
  const cv::Mat img1 = as_mat(next);

  std::vector<cv::Point2f> p0;
  cv::goodFeaturesToTrack(
    img0, p0, params.max_corners, params.quality_level, params.min_distance_px, cv::noArray(), 7);
  if (p0.size() < params.min_tracks) {
    out.tracks = p0.size();
    return out;
  }
  std::vector<cv::Point2f> p1;
  std::vector<cv::Point2f> p0_back;
  std::vector<std::uint8_t> st;
  std::vector<std::uint8_t> st_back;
  std::vector<float> err;
  const cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01);
  cv::calcOpticalFlowPyrLK(img0, img1, p0, p1, st, err, cv::Size(21, 21), 4, crit);
  cv::calcOpticalFlowPyrLK(img1, img0, p1, p0_back, st_back, err, cv::Size(21, 21), 4, crit);

  const double fx = cam.k[0];
  const double fy = cam.k[4];
  const double cx = cam.k[2];
  const double cy = cam.k[5];
  const double f_mean = 0.5 * (fx + fy);
  std::vector<cv::Point2d> n0;  // undistorted normalized coordinates
  std::vector<cv::Point2d> n1;
  std::vector<double> flow;
  for (std::size_t i = 0; i < p0.size(); ++i) {
    if (st[i] == 0 || st_back[i] == 0) {
      continue;
    }
    const double fb = cv::norm(p0[i] - p0_back[i]);
    if (fb > params.forward_backward_max_px) {
      continue;
    }
    const auto u0 = image::invert_distortion_normalized(
      (p0[i].x - cx) / fx, (p0[i].y - cy) / fy, cam.model, cam.d);
    const auto u1 = image::invert_distortion_normalized(
      (p1[i].x - cx) / fx, (p1[i].y - cy) / fy, cam.model, cam.d);
    n0.emplace_back(u0.x, u0.y);
    n1.emplace_back(u1.x, u1.y);
    flow.push_back(cv::norm(p1[i] - p0[i]));
  }
  out.tracks = n0.size();
  if (out.tracks < params.min_tracks) {
    return out;
  }
  {
    std::vector<double> sorted = flow;
    std::nth_element(
      sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2),
      sorted.end());
    out.median_flow_px = sorted[sorted.size() / 2];
  }

  // (1) Essential matrix. recoverPose's R satisfies x_next = R x_prev (+ t),
  // so the camera's rotation prev -> next in the prev frame is R^T.
  {
    cv::Mat mask;
    cv::Mat e = cv::findEssentialMat(
      n0, n1, 1.0, cv::Point2d(0.0, 0.0), cv::RANSAC, 0.999, params.essential_threshold_px / f_mean,
      mask);
    if (!e.empty() && e.cols == 3 && e.rows >= 3) {
      if (e.rows > 3) {
        e = e.rowRange(0, 3).clone();  // several candidates: the first one
      }
      cv::Mat r;
      cv::Mat t;
      const int inliers = cv::recoverPose(e, n0, n1, r, t, 1.0, cv::Point2d(0.0, 0.0), mask);
      if (inliers >= 20 && r.rows == 3 && r.cols == 3) {
        out.inliers_essential = static_cast<std::size_t>(inliers);
        cv::Matx33d rm;
        r.convertTo(rm, CV_64F);
        out.essential = rotvec_of(rm.t());
      }
    }
  }

  // (2) Pure rotation: RANSAC over 3-point Kabsch on bearing vectors, then a
  // refit on the inliers. Kabsch gives b ~ R a (prev -> next coordinates),
  // so the camera's rotation in the prev frame is again R^T.
  {
    std::vector<cv::Vec3d> a(n0.size());
    std::vector<cv::Vec3d> b(n0.size());
    for (std::size_t i = 0; i < n0.size(); ++i) {
      a[i] = cv::normalize(cv::Vec3d(n0[i].x, n0[i].y, 1.0));
      b[i] = cv::normalize(cv::Vec3d(n1[i].x, n1[i].y, 1.0));
    }
    const double thr = params.rotation_threshold_px / f_mean;
    std::mt19937_64 rng(params.seed);
    std::uniform_int_distribution<std::size_t> pick(0, a.size() - 1);
    std::vector<std::size_t> best_inliers;
    for (int it = 0; it < params.rotation_ransac_iterations; ++it) {
      std::vector<std::size_t> sample;
      while (sample.size() < 3) {
        const std::size_t k = pick(rng);
        if (std::find(sample.begin(), sample.end(), k) == sample.end()) {
          sample.push_back(k);
        }
      }
      const cv::Matx33d r = kabsch(a, b, sample);
      std::vector<std::size_t> inl;
      for (std::size_t i = 0; i < a.size(); ++i) {
        if (bearing_residual(r, a[i], b[i]) < thr) {
          inl.push_back(i);
        }
      }
      if (inl.size() > best_inliers.size()) {
        best_inliers = std::move(inl);
      }
    }
    if (best_inliers.size() >= 20) {
      cv::Matx33d r = kabsch(a, b, best_inliers);
      std::vector<std::size_t> refined;
      for (std::size_t i = 0; i < a.size(); ++i) {
        if (bearing_residual(r, a[i], b[i]) < thr) {
          refined.push_back(i);
        }
      }
      if (refined.size() >= 20) {
        r = kabsch(a, b, refined);
        best_inliers = std::move(refined);
      }
      out.inliers_rotation = best_inliers.size();
      out.pure_rotation = rotvec_of(r.t());
    }
  }
  return out;
}

}  // namespace bagwiz::core::calib
