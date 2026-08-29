// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__CALIB_CAM_LIDAR_OFFSET_HPP_
#define COMMANDS__CALIB_CAM_LIDAR_OFFSET_HPP_

#include "bagwiz/core/calib/se3.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "calib_cam_lidar_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <cstdint>
#include <optional>
#include <span>

// `calib cam-lidar --cam-offset auto`: measure the camera-vs-pose clock offset
// from the bag the command is already reading. Split out of calib_cam_lidar.cpp
// so the image block reading, the IMU reading and the call into the core
// estimators (bagwiz_pointcloud's visual_rotation + time_offset) stay in one
// place. CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// An aggregate of references: every member is bound by the brace
// initializer at the single construction site, which is the only way to build
// it. Recent cppcheck releases still report the reference members as
// uninitialized (uninitMemberVarNoCtor); a reference cannot be left unbound,
// so the report is suppressed per member.
struct CamOffsetEstimateInput
{
  const CalibCamLidarArgs & args;
  // Every image stamp on --cam, unshifted, in bag order (what
  // scan_image_stamps read), and the trimmed --pose trajectory.
  std::span<const std::int64_t> image_stamps_ns;
  std::span<const core::TrajectoryPose> poses;
  // The image topic's CameraInfo (full resolution) and the pose of the
  // camera's optical frame in --of at the bag's edge value, whose rotation
  // block carries the visual rotations into the --of frame.
  // cppcheck-suppress uninitMemberVarNoCtor
  const core::image::CameraInfo & cam_info;
  // cppcheck-suppress uninitMemberVarNoCtor
  const core::calib::Mat4 & t_trajframe_cam0;
  // Static-only TF buffer, for the --imu frame's chain from --of.
  // cppcheck-suppress uninitMemberVarNoCtor
  tf2::BufferCore & static_buffer;
};

// Estimate the offset: frame-to-frame camera rotations over at most
// kCamOffsetMaxFrames frames (evenly spread contiguous blocks of
// kCamOffsetBlockFrames so a long bag stays bounded) are matched against the
// --pose trajectory's rotation, or — when --imu names a topic — against the
// gyro, with the trajectory matched against the same gyro and the two
// offsets differenced so the IMU's own latency cancels. Logs the estimate
// and every failure; nullopt on failure (too little rotation, too few frame
// pairs, a spread too wide to apply, an unreadable IMU topic or frame chain).
[[nodiscard]] std::optional<CamOffsetEstimateReport> estimate_cam_offset(
  const CamOffsetEstimateInput & in);

inline constexpr std::size_t kCamOffsetMaxFrames = 1200;
inline constexpr std::size_t kCamOffsetBlockFrames = 100;
// The longer image side the frames are downscaled to before tracking: plenty
// of corners at a fraction of the decode-and-track cost of a 4K frame.
inline constexpr std::uint32_t kCamOffsetMaxImageSide = 1440;

}  // namespace bagwiz::commands

#endif  // COMMANDS__CALIB_CAM_LIDAR_OFFSET_HPP_
