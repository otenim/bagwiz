// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TOPIC_TYPES_HPP_
#define BAGWIZ__COMMANDS__TOPIC_TYPES_HPP_

#include "bagwiz/core/tf/tf_topics.hpp"

#include <array>
#include <string_view>

// The message types each topic-valued CLI option accepts. One definition per
// set, shared by two consumers that used to keep hand-synced copies: the
// option declaration (add_topic_option()'s allowed_types, which filters glob
// expansion) and shell completion (which only offers topics of an accepted
// type).
//
// Each command's own validator is deliberately NOT a reader of these arrays —
// it type-checks one already-resolved topic name against a private copy
// rather than filtering a candidate list, and moving it here would mean the
// validator's rejection wording (and its exception cases, e.g. cam-info dump
// accepting a calibration YAML that isn't a bag at all) leaks into a header
// meant only for slot declaration and completion. That copy is noted below,
// per constant, as "must mirror" — keep the two in sync by hand. If a
// validator gains a type this array lacks, the failure is silent narrowing,
// not an error: glob expansion (and completion) simply stops offering that
// topic while the validator would still accept it as a literal.
namespace bagwiz::commands
{

// No command validator mirrors this one: `tf tree` tests membership against
// the bag's already type-filtered TfTopic list (bagwiz_tf), never a literal
// string comparison, so there is no private copy to drift.
inline constexpr std::array<std::string_view, 1> kTfMessageTypes{{core::kTfMessageTypeName}};

// Types `traj dump` can sample a trajectory from. Must mirror the
// kTfMessageType / kPoseStampedType / kPoseWithCovarianceStampedType /
// kOdometryType constants in bagwiz/src/commands/traj.cpp, which
// TrajCommand::run_dump() dispatches on.
inline constexpr std::array<std::string_view, 4> kTrajDumpSupportedTypes{{
  core::kTfMessageTypeName,
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "nav_msgs/msg/Odometry",
}};

// Self-position types `pcd undistort --pose` accepts. Must mirror
// is_supported_pose_topic_type() in
// bagwiz/src/commands/pcd_undistort_common.cpp. Happens to equal
// kTrajDumpSupportedTypes today, but the two commands gate on separate
// validators and can drift independently.
inline constexpr std::array<std::string_view, 4> kUndistortPoseTopicTypes{{
  core::kTfMessageTypeName,
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "nav_msgs/msg/Odometry",
}};

// Vehicle-velocity types `pcd undistort --twist` accepts. Must mirror
// is_supported_twist_topic_type() in
// bagwiz/src/commands/pcd_undistort_common.cpp.
inline constexpr std::array<std::string_view, 3> kUndistortTwistTopicTypes{{
  "geometry_msgs/msg/Twist",
  "geometry_msgs/msg/TwistStamped",
  "geometry_msgs/msg/TwistWithCovarianceStamped",
}};

// Image types the shared to_packed_raster() decoder handles — `movify`
// rendering, `walk`'s image preview, `map slam --color`, and `calib cam-lidar
// --cam` all gate on it. Must mirror is_supported_type() in
// bagwiz/src/commands/movify_inputs.cpp AND is_supported_image_type()
// in bagwiz_image/src/core/image/packed_raster.cpp AND the
// kImageMsgType/kCompressedImageMsgType pair in
// bagwiz/src/commands/calib_cam_lidar.cpp — three independent private copies,
// not one.
inline constexpr std::array<std::string_view, 2> kImageTopicTypes{{
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
}};

// Must mirror the private kCameraInfoType constant in each of: (1)
// bagwiz/src/commands/cam_info_common.hpp, used by `cam-info replace` and
// `cam-info recompute-p`; (2) bagwiz/src/commands/cam_info_dump.cpp, used by
// `cam-info dump` — a separate copy from (1), not a reuse of it; (3)
// bagwiz_image/src/core/image/camera_info_resolver.cpp, the resolver
// `movify --cam-info` and `walk --cam-info` both go through.
inline constexpr std::array<std::string_view, 1> kCameraInfoType{{
  "sensor_msgs/msg/CameraInfo",
}};

// Must mirror the private kPointCloud2Type constant in each of:
// bagwiz/src/commands/pcd_undistort_common.cpp (`pcd undistort --pcd`),
// bagwiz/src/commands/pcd_concat.cpp (`pcd concat --pcd`), and
// bagwiz/src/commands/map_slam.cpp (`map slam --pcd`) — plus the
// differently-named kPointCloudType in
// bagwiz/src/commands/movify_inputs.cpp (`movify --cam-pcd`) and
// kPointCloud2MsgType in bagwiz/src/commands/calib_cam_lidar.cpp
// (`calib cam-lidar --pcd`).
inline constexpr std::array<std::string_view, 1> kPointCloud2Type{{
  "sensor_msgs/msg/PointCloud2",
}};

// The topics `movify --clock` may name: any camera panel topic, any
// point-cloud panel topic, or the map panel's NavSatFix topic. Must mirror
// the check in validate_video_inputs() (bagwiz/src/commands/movify_inputs.cpp),
// which resolves --clock against the --cam, --pcd and --gnss topics already
// given.
inline constexpr std::array<std::string_view, 4> kMovifyClockTopicTypes{{
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
  "sensor_msgs/msg/PointCloud2",
  "sensor_msgs/msg/NavSatFix",
}};

// The pose topics `movify --pose` draws as a trajectory over the camera and
// point-cloud panels: the body-posing types of kUndistortPoseTopicTypes, not
// TFMessage (a set of edges, not a body's pose). Must mirror the type check in
// load_pose_overlay() (bagwiz/src/commands/movify_pose_overlay.cpp).
inline constexpr std::array<std::string_view, 3> kMovifyPoseTopicTypes{{
  "nav_msgs/msg/Odometry",
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
}};

// The topics `video decode` reads; `video encode` writes this type.
inline constexpr std::array<std::string_view, 1> kCompressedVideoTopicTypes{{
  "foxglove_msgs/msg/CompressedVideo",
}};

// Must mirror the private kImuType constant in
// bagwiz/src/commands/map_slam.cpp (`map slam --imu`).
inline constexpr std::array<std::string_view, 1> kImuType{{"sensor_msgs/msg/Imu"}};

// Must mirror the private kNavSatFixType constant in
// bagwiz/src/commands/map_slam.cpp (`map slam --gnss`).
inline constexpr std::array<std::string_view, 1> kNavSatFixType{{"sensor_msgs/msg/NavSatFix"}};

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_TYPES_HPP_
