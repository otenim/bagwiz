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
// set, shared by three consumers that used to keep hand-synced copies: the
// option declaration (which filters glob expansion), the command's own
// validator (which rejects a wrongly typed literal), and shell completion
// (which only offers topics of an accepted type).
namespace bagwiz::commands
{

inline constexpr std::array<std::string_view, 1> kTfMessageTypes{{core::kTfMessageTypeName}};

// Types `traj dump` can sample a trajectory from.
inline constexpr std::array<std::string_view, 4> kTrajDumpSupportedTypes{{
  core::kTfMessageTypeName,
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "nav_msgs/msg/Odometry",
}};

// Self-position types `pcd undistort --pose` accepts.
inline constexpr std::array<std::string_view, 4> kUndistortPoseTopicTypes{{
  core::kTfMessageTypeName,
  "geometry_msgs/msg/PoseStamped",
  "geometry_msgs/msg/PoseWithCovarianceStamped",
  "nav_msgs/msg/Odometry",
}};

// Vehicle-velocity types `pcd undistort --twist` accepts.
inline constexpr std::array<std::string_view, 3> kUndistortTwistTopicTypes{{
  "geometry_msgs/msg/Twist",
  "geometry_msgs/msg/TwistStamped",
  "geometry_msgs/msg/TwistWithCovarianceStamped",
}};

// Image types the shared to_packed_raster() decoder handles — `generate video`
// rendering, `walk`'s image preview, and `map slam --color` all gate on it.
inline constexpr std::array<std::string_view, 2> kImageTopicTypes{{
  "sensor_msgs/msg/Image",
  "sensor_msgs/msg/CompressedImage",
}};

inline constexpr std::array<std::string_view, 1> kCameraInfoType{{
  "sensor_msgs/msg/CameraInfo",
}};

inline constexpr std::array<std::string_view, 1> kPointCloud2Type{{
  "sensor_msgs/msg/PointCloud2",
}};

inline constexpr std::array<std::string_view, 1> kImuType{{"sensor_msgs/msg/Imu"}};

inline constexpr std::array<std::string_view, 1> kNavSatFixType{{"sensor_msgs/msg/NavSatFix"}};

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_TYPES_HPP_
