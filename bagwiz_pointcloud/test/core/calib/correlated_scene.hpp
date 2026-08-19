// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__CALIB__CORRELATED_SCENE_HPP_
#define CORE__CALIB__CORRELATED_SCENE_HPP_

#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace
{

// A frontal wall of points at z=8 m, and an image painted as a horizontal
// intensity ramp (gray = px * 255 / (width - 1)), constant along v. Each
// point's lidar bin is derived from its own ground-truth projected pixel, so
// gray-bin and lidar-bin are the same monotone function of u: the joint
// histogram is diagonal at the true pose, and a sub-pixel misalignment moves
// a proportional fraction of points across a gray-bin boundary. That gives
// the NID cost a smooth basin instead of the quantization plateau a
// splatted, piecewise-constant image would produce (see
// 2026-08-19-tf-static-calibrate/task-6-report.md).
bagwiz::core::calib::CalibSample make_correlated_sample(
  const bagwiz::core::calib::CameraModel & cam, int bins)
{
  bagwiz::core::calib::CalibSample sample;
  sample.t_world_trajframe = bagwiz::core::calib::identity_mat4();
  sample.image.width = cam.width;
  sample.image.height = cam.height;
  sample.image.gray.resize(static_cast<std::size_t>(cam.width) * cam.height);
  for (std::uint32_t py = 0; py < cam.height; ++py) {
    for (std::uint32_t px = 0; px < cam.width; ++px) {
      sample.image.gray[static_cast<std::size_t>(py) * cam.width + px] =
        static_cast<std::uint8_t>(px * 255 / (cam.width - 1));
    }
  }
  for (int iy = -30; iy < 30; ++iy) {
    for (int ix = -50; ix < 50; ++ix) {
      const float x = 0.1F * static_cast<float>(ix);
      const float y = 0.1F * static_cast<float>(iy);
      const double u = cam.k[0] * (x / 8.0) + cam.k[2];
      const int bin = std::clamp(static_cast<int>(u * bins / cam.width), 0, bins - 1);
      sample.points_world.push_back({x, y, 8.0F});
      sample.intensity_bins.push_back(static_cast<std::uint8_t>(bin));
    }
  }
  return sample;
}

bagwiz::core::calib::CameraModel test_camera()
{
  bagwiz::core::calib::CameraModel cam;
  cam.k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  cam.width = 640;
  cam.height = 480;
  return cam;
}

}  // namespace

#endif  // CORE__CALIB__CORRELATED_SCENE_HPP_
