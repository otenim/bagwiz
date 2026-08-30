// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector_helpers.hpp"

#include "bagwiz/core/pointcloud/projector.hpp"

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace bagwiz::core::pointcloud
{

namespace
{

// tf2::BufferCore is not guaranteed to be thread-safe for concurrent
// lookups, and every caller (including movify's panel workers) shares the
// same buffer, so every lookup here takes this one lock.
std::mutex & tf_lookup_mutex()
{
  static std::mutex mutex;
  return mutex;
}

}  // namespace

std::optional<RigidTransform> lookup_rigid_transform(
  tf2::BufferCore & tf_buffer, const std::string & target_frame, const std::string & source_frame,
  std::int64_t stamp_ns, std::string & error)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    const std::lock_guard<std::mutex> lock(tf_lookup_mutex());
    tf = tf_buffer.lookupTransform(
      target_frame, source_frame, tf2::TimePoint{std::chrono::nanoseconds(stamp_ns)});
  } catch (const tf2::TransformException & e) {
    error = "cannot transform " + source_frame + " -> " + target_frame + ": " + e.what();
    return std::nullopt;
  }
  RigidTransform out;
  out.rotation = quat_to_rotation_matrix(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);
  out.translation = {
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z};
  return out;
}

ProjectionResult project_cloud_for_frame(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info, tf2::BufferCore & tf_buffer,
  std::uint32_t image_width, std::uint32_t image_height, PointCloudProperty property,
  bool use_rectified, std::optional<std::int64_t> timestamp_ns)
{
  const std::string & image_frame = camera_info.frame_id;
  const std::string & cloud_frame = cloud.frame_id;

  geometry_msgs::msg::TransformStamped tf;
  try {
    const std::lock_guard<std::mutex> lock(tf_lookup_mutex());
    if (timestamp_ns.has_value()) {
      const tf2::TimePoint tp{std::chrono::nanoseconds(*timestamp_ns)};
      tf = tf_buffer.lookupTransform(image_frame, cloud_frame, tp);
    } else {
      tf = tf_buffer.lookupTransform(image_frame, cloud_frame, tf2::TimePointZero);
    }
  } catch (const tf2::TransformException & e) {
    return {
      {}, std::string("cannot transform ") + cloud_frame + " -> " + image_frame + ": " + e.what()};
  }

  std::array<double, 16> transform{};
  tf2::Quaternion q(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);
  tf2::Matrix3x3(q).getOpenGLSubMatrix(transform.data());
  transform[12] = tf.transform.translation.x;
  transform[13] = tf.transform.translation.y;
  transform[14] = tf.transform.translation.z;

  return project_pointcloud(
    cloud, camera_info, transform, image_width, image_height, property, use_rectified);
}

}  // namespace bagwiz::core::pointcloud
