// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/static_extrinsic.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_trajectory_sample.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";
constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
constexpr const char * kPoseWithCovarianceStampedType =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
constexpr const char * kTwistType = "geometry_msgs/msg/Twist";
constexpr const char * kTwistStampedType = "geometry_msgs/msg/TwistStamped";
constexpr const char * kTwistWithCovarianceStampedType =
  "geometry_msgs/msg/TwistWithCovarianceStamped";

// TFMessage pose topic: delegate to the shared bagwiz_tf sampler
// (core::sample_tf_message_trajectory), which replays the topic's transforms
// into `buffer` as dynamic edges (tf_static is already loaded there),
// resolves the --ref -> --of chain, and samples it at every stamp the chain's
// edges are actually published on `pose_topic`. The switch below only maps
// the sampler's structured failure onto this command's wording, so log texts
// and exit behavior stay unchanged.
TrajectoryBuildResult build_trajectory_from_tf_message(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, const std::string & ref,
  const std::string & of, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const auto sampled = core::sample_tf_message_trajectory(input_path, pose_ti, ref, of, buffer);
  using Failure = core::TfMessageTrajectoryResult::Failure;
  switch (sampled.failure) {
    case Failure::kNone:
      break;
    case Failure::kOpenBag:
      out.error = "failed to reopen bag for pose topic: " + sampled.failure_detail;
      return out;
    case Failure::kOpenDecoder:
      out.error =
        "could not open decoder for pose topic '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kDecode:
      out.error = "failed to decode message on '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kRead:
      out.error = "error reading pose topic '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kNoTransforms:
      out.error = "pose topic '" + pose_ti.name + "' carried no TransformStamped entries";
      return out;
    case Failure::kNoPath:
      out.error = "no TF path from --of '" + of + "' to --ref '" + ref + "' (checked '" +
                  pose_ti.name + "' + the bag's static TF)";
      return out;
    case Failure::kNoPathStamps:
      out.error = "--of '" + of + "' -> --ref '" + ref +
                  "' resolves via static TF, but none of the edges on that path are published on "
                  "pose topic '" +
                  pose_ti.name + "'";
      return out;
  }

  if (sampled.poses.empty()) {
    out.error = "all " + std::to_string(sampled.sample_stamps) +
                " sample stamp(s) failed to resolve via lookupTransform; last reason: " +
                (sampled.last_skip_reason.empty() ? "(none)" : sampled.last_skip_reason);
    return out;
  }
  out.trajectory = std::move(sampled.poses);
  return out;
}

// Odometry / PoseStamped / PoseWithCovarianceStamped pose topic: for each
// message, compose T_ref_of = T_ref_header * T_header_body * T_body_of,
// bridging into --ref / --of via the bag's static TF when the message's own
// frames do not already match. Mirrors the traj dump pose-topic composition
// (core::compose_tf_bridged_sample in bagwiz_tf), except an unresolvable
// bridge is fatal here rather than a per-sample skip: pcd undistort's TF is
// static-only, so a failure is a configuration problem, not transient sensor
// noise.
TrajectoryBuildResult build_trajectory_from_pose_topic(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, PoseComposeKind kind,
  const std::string & ref, const std::string & of, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const bool is_odom = (kind == PoseComposeKind::kOdometry);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for pose topic: ") + e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {pose_ti.name};
  reader->set_filter(filter);

  auto open = core::decoder::open_decoder(pose_ti);
  if (!open.ok()) {
    out.error = "could not open decoder for pose topic '" + pose_ti.name + "': " + open.error;
    return out;
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != pose_ti.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode message on '" + pose_ti.name + "': " + decoded.error;
        return out;
      }
      PoseSample sample;
      if (!decode_pose_sample(kind, *decoded.value, sample)) {
        continue;  // unparsable sample; tolerated like traj dump's skip
      }
      if (sample.pose.header.frame_id.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': message has empty header.frame_id";
        return out;
      }
      if (is_odom && sample.child_frame.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': Odometry message has empty child_frame_id";
        return out;
      }

      const std::string & header_frame = sample.pose.header.frame_id;
      const std::int64_t ns =
        static_cast<std::int64_t>(sample.pose.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(sample.pose.header.stamp.nanosec);

      std::optional<geometry_msgs::msg::Transform> from_header;
      if (ref != header_frame) {
        const auto resolved = core::pointcloud::resolve_static_extrinsic(buffer, ref, header_frame);
        if (!resolved.missing.empty()) {
          out.error = "--ref '" + ref + "' has no static TF chain to pose topic '" + pose_ti.name +
                      "'s frame '" + header_frame + "'";
          return out;
        }
        if (!resolved.ok()) {
          out.error = "--ref '" + ref + "' -> '" + header_frame +
                      "' TF lookup failed: " + resolved.lookup_error;
          return out;
        }
        from_header = resolved.transform.transform;
      }
      std::optional<geometry_msgs::msg::Transform> body_to;
      if (is_odom && of != sample.child_frame) {
        const auto resolved =
          core::pointcloud::resolve_static_extrinsic(buffer, sample.child_frame, of);
        if (!resolved.missing.empty()) {
          out.error = "--of '" + of + "' has no static TF chain from Odometry child frame '" +
                      sample.child_frame + "'";
          return out;
        }
        if (!resolved.ok()) {
          out.error = "'" + sample.child_frame + "' -> --of '" + of +
                      "' TF lookup failed: " + resolved.lookup_error;
          return out;
        }
        body_to = resolved.transform.transform;
      }

      const auto composed = core::compose_trajectory_pose(from_header, sample.pose.pose, body_to);
      core::TrajectoryPose p;
      p.timestamp_ns = ns;
      p.tx = composed.position.x;
      p.ty = composed.position.y;
      p.tz = composed.position.z;
      p.qx = composed.orientation.x;
      p.qy = composed.orientation.y;
      p.qz = composed.orientation.z;
      p.qw = composed.orientation.w;
      out.trajectory.push_back(p);
    }
  } catch (const std::exception & e) {
    out.error = "error reading pose topic '" + pose_ti.name + "': " + e.what();
    return out;
  }

  if (out.trajectory.empty()) {
    out.error = "no poses decoded from pose topic '" + pose_ti.name + "'";
  }
  return out;
}

// ---- twist-source trajectory building ----------------------------------------

// Minimal quaternion (x, y, z, w; ROS / tf2 Hamilton convention) used by the
// twist dead reckoning below. Plain doubles keep this independent of tf2's
// transform math; the trajectory it feeds (TrajectoryPose) uses the same
// convention.
struct Quat
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

Quat quat_mul(const Quat & a, const Quat & b)
{
  return {
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Quat quat_normalized(const Quat & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}

// Rotate vector (vx, vy, vz) by q via q * (v,0) * q^-1 (q must be normalized).
void quat_rotate(
  const Quat & q, double vx, double vy, double vz, double & ox, double & oy, double & oz)
{
  // t = 2 * cross(q.xyz, v)
  const double tx = 2.0 * (q.y * vz - q.z * vy);
  const double ty = 2.0 * (q.z * vx - q.x * vz);
  const double tz = 2.0 * (q.x * vy - q.y * vx);
  // v' = v + q.w * t + cross(q.xyz, t)
  ox = vx + q.w * tx + (q.y * tz - q.z * ty);
  oy = vy + q.w * ty + (q.z * tx - q.x * tz);
  oz = vz + q.w * tz + (q.x * ty - q.y * tx);
}

// One decoded twist sample, already expressed in the --of frame.
struct TwistSample
{
  std::int64_t stamp_ns = 0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
};

// Advance (p, q) by one zero-order-hold constant-twist interval: the twist
// (v, w) is treated as constant in the body frame over [0, dt]. The rotation
// increment is the axis-angle exponential of w*dt; the translation increment
// is the exact integral of the rotating velocity, dt * J(w*dt) * v with the
// SO(3) left Jacobian J. Below the small-angle threshold, 1-cos and
// theta-sin lose all double precision, so the first-order forms (dp = v*dt,
// dq from phi/2) are used — the dropped terms are O(theta) relative and
// irrelevant at that magnitude.
void integrate_constant_twist(
  const TwistSample & s, double dt, double & px, double & py, double & pz, Quat & q)
{
  constexpr double kSmallAngle = 1e-8;
  const double phix = s.wx * dt;
  const double phiy = s.wy * dt;
  const double phiz = s.wz * dt;
  const double theta = std::sqrt(phix * phix + phiy * phiy + phiz * phiz);

  Quat dq;
  double dpx, dpy, dpz;
  if (theta < kSmallAngle) {
    dq = quat_normalized({phix * 0.5, phiy * 0.5, phiz * 0.5, 1.0});
    dpx = s.vx * dt;
    dpy = s.vy * dt;
    dpz = s.vz * dt;
  } else {
    const double half = 0.5 * theta;
    const double k = std::sin(half) / theta;
    dq = {phix * k, phiy * k, phiz * k, std::cos(half)};
    const double a = (1.0 - std::cos(theta)) / (theta * theta);
    const double b = (theta - std::sin(theta)) / (theta * theta * theta);
    // J(phi) * v = v + a * (phi x v) + b * (phi x (phi x v))
    const double c1x = phiy * s.vz - phiz * s.vy;
    const double c1y = phiz * s.vx - phix * s.vz;
    const double c1z = phix * s.vy - phiy * s.vx;
    const double c2x = phiy * c1z - phiz * c1y;
    const double c2y = phiz * c1x - phix * c1z;
    const double c2z = phix * c1y - phiy * c1x;
    dpx = dt * (s.vx + a * c1x + b * c2x);
    dpy = dt * (s.vy + a * c1y + b * c2y);
    dpz = dt * (s.vz + a * c1z + b * c2z);
  }

  double wx, wy, wz;
  quat_rotate(q, dpx, dpy, dpz, wx, wy, wz);
  px += wx;
  py += wy;
  pz += wz;
  q = quat_normalized(quat_mul(q, dq));
}

// Twist motion source (Twist / TwistStamped / TwistWithCovarianceStamped):
// dead-reckon the velocity samples into a relative trajectory, identity at the
// first sample. The deskew kernel only consumes T(t_ref)^-1 * T(t_i), which is
// invariant to the fixed frame a trajectory is expressed in, so the arbitrary
// integration origin is fine and --ref plays no role here. Stamped types carry
// the frame the twist is expressed in; a frame that is neither empty nor --of
// is rotated into --of via the bag's static TF, with an unresolvable chain
// fatal (mirrors the pose-topic bridge policy: static-only TF means a failure
// is a configuration problem, not transient noise). A bare Twist has no
// header: samples are stamped with the bag's log time and assumed to already
// be in the --of frame.
TrajectoryBuildResult build_trajectory_from_twist_topic(
  const std::filesystem::path & input_path, const io::TopicInfo & twist_ti, const std::string & of,
  tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const bool is_bare = (twist_ti.type == kTwistType);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for twist topic: ") + e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {twist_ti.name};
  reader->set_filter(filter);

  auto open = core::decoder::open_decoder(twist_ti);
  if (!open.ok()) {
    out.error = "could not open decoder for twist topic '" + twist_ti.name + "': " + open.error;
    return out;
  }

  // Lazily resolved static-TF rotation per twist frame (twist frame -> --of).
  std::unordered_map<std::string, Quat> frame_rotations;
  std::vector<TwistSample> samples;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != twist_ti.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode message on '" + twist_ti.name + "': " + decoded.error;
        return out;
      }

      TwistSample sample;
      std::string frame;  // empty: bare Twist, or assumed --of
      if (is_bare) {
        const auto twist = core::extract_twist_message(*decoded.value);
        if (!twist.has_value()) {
          continue;  // unparsable sample; tolerated like the pose path's skip
        }
        sample.stamp_ns = raw.timestamp_ns;
        sample.vx = twist->linear.x;
        sample.vy = twist->linear.y;
        sample.vz = twist->linear.z;
        sample.wx = twist->angular.x;
        sample.wy = twist->angular.y;
        sample.wz = twist->angular.z;
      } else {
        const auto ts = (twist_ti.type == kTwistStampedType)
                          ? core::extract_twist_stamped_message(*decoded.value)
                          : core::extract_twist_with_covariance_stamped_message(*decoded.value);
        if (!ts.has_value()) {
          continue;
        }
        sample.stamp_ns = static_cast<std::int64_t>(ts->header.stamp.sec) * 1'000'000'000LL +
                          static_cast<std::int64_t>(ts->header.stamp.nanosec);
        sample.vx = ts->twist.linear.x;
        sample.vy = ts->twist.linear.y;
        sample.vz = ts->twist.linear.z;
        sample.wx = ts->twist.angular.x;
        sample.wy = ts->twist.angular.y;
        sample.wz = ts->twist.angular.z;
        frame = ts->header.frame_id;
      }

      if (!frame.empty() && frame != of) {
        const auto [it, inserted] = frame_rotations.try_emplace(frame);
        if (inserted) {
          const auto resolved = core::pointcloud::resolve_static_extrinsic(buffer, of, frame);
          if (!resolved.missing.empty()) {
            out.error = "--of '" + of + "' has no static TF chain to twist topic '" +
                        twist_ti.name + "'s frame '" + frame + "'";
            return out;
          }
          if (!resolved.ok()) {
            out.error = "--of '" + of + "' -> twist frame '" + frame +
                        "' TF lookup failed: " + resolved.lookup_error;
            return out;
          }
          const auto & r = resolved.transform.transform.rotation;
          it->second = quat_normalized({r.x, r.y, r.z, r.w});
        }
        double rx, ry, rz;
        quat_rotate(it->second, sample.vx, sample.vy, sample.vz, rx, ry, rz);
        sample.vx = rx;
        sample.vy = ry;
        sample.vz = rz;
        quat_rotate(it->second, sample.wx, sample.wy, sample.wz, rx, ry, rz);
        sample.wx = rx;
        sample.wy = ry;
        sample.wz = rz;
      }
      samples.push_back(sample);
    }
  } catch (const std::exception & e) {
    out.error = "error reading twist topic '" + twist_ti.name + "': " + e.what();
    return out;
  }

  if (samples.empty()) {
    out.error = "no twist samples decoded from twist topic '" + twist_ti.name + "'";
    return out;
  }
  std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
    return a.stamp_ns < b.stamp_ns;
  });

  out.trajectory.reserve(samples.size());
  core::TrajectoryPose pose;
  pose.timestamp_ns = samples.front().stamp_ns;
  pose.qw = 1.0;
  out.trajectory.push_back(pose);
  double px = 0.0, py = 0.0, pz = 0.0;
  Quat q;
  for (std::size_t k = 0; k + 1 < samples.size(); ++k) {
    const double dt = static_cast<double>(samples[k + 1].stamp_ns - samples[k].stamp_ns) * 1e-9;
    integrate_constant_twist(samples[k], dt, px, py, pz, q);
    pose.timestamp_ns = samples[k + 1].stamp_ns;
    pose.tx = px;
    pose.ty = py;
    pose.tz = pz;
    pose.qx = q.x;
    pose.qy = q.y;
    pose.qz = q.z;
    pose.qw = q.w;
    out.trajectory.push_back(pose);
  }
  return out;
}

}  // namespace

bool is_supported_pose_topic_type(const std::string & type)
{
  return type == kTfMessageType || type == kOdometryType || type == kPoseStampedType ||
         type == kPoseWithCovarianceStampedType;
}

bool is_supported_twist_topic_type(const std::string & type)
{
  return type == kTwistType || type == kTwistStampedType || type == kTwistWithCovarianceStampedType;
}

PoseComposeKind pose_compose_kind(const std::string & type)
{
  if (type == kOdometryType) {
    return PoseComposeKind::kOdometry;
  }
  if (type == kPoseStampedType) {
    return PoseComposeKind::kPoseStamped;
  }
  return PoseComposeKind::kPoseWithCovarianceStamped;  // caller validated the type
}

bool decode_pose_sample(
  PoseComposeKind kind, const core::cdr_walker::Value & value, PoseSample & out)
{
  switch (kind) {
    case PoseComposeKind::kPoseStamped: {
      const auto ps = core::extract_pose_stamped_message(value);
      if (!ps.has_value()) {
        return false;
      }
      out.pose = *ps;
      return true;
    }
    case PoseComposeKind::kPoseWithCovarianceStamped: {
      const auto pwc = core::extract_pose_with_covariance_stamped_message(value);
      if (!pwc.has_value()) {
        return false;
      }
      out.pose.header = pwc->header;
      out.pose.pose = pwc->pose.pose;
      return true;
    }
    case PoseComposeKind::kOdometry: {
      const auto odom = core::extract_odometry_message(value);
      if (!odom.has_value()) {
        return false;
      }
      out.pose.header = odom->header;
      out.pose.pose = odom->pose.pose;
      out.child_frame = odom->child_frame_id;
      return true;
    }
  }
  return false;
}

const io::TopicInfo * validate_undistort_topics(
  const io::BagReader & reader, const std::string & motion_topic, bool motion_is_twist,
  const std::vector<std::string> & pcd_topics, const std::filesystem::path & bag_path,
  const char * logger)
{
  const io::TopicInfo * motion_ti = io::find_topic_or_log(reader, motion_topic, bag_path, logger);
  if (motion_ti == nullptr) {
    return nullptr;
  }
  if (motion_is_twist) {
    if (!is_supported_twist_topic_type(motion_ti->type)) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' has unsupported type '%s'. Supported: %s, %s, %s.",
        motion_topic.c_str(), motion_ti->type.c_str(), kTwistType, kTwistStampedType,
        kTwistWithCovarianceStampedType);
      return nullptr;
    }
  } else if (!is_supported_pose_topic_type(motion_ti->type)) {
    BAGWIZ_LOG_ERROR(
      logger, "Topic '%s' has unsupported type '%s'. Supported: %s, %s, %s, %s.",
      motion_topic.c_str(), motion_ti->type.c_str(), kTfMessageType, kOdometryType,
      kPoseStampedType, kPoseWithCovarianceStampedType);
    return nullptr;
  }
  for (const auto & topic : pcd_topics) {
    const io::TopicInfo * info = io::find_topic_or_log(reader, topic, bag_path, logger);
    if (info == nullptr) {
      return nullptr;
    }
    if (info->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' is %s, expected %s", topic.c_str(), info->type.c_str(),
        kPointCloud2Type);
      return nullptr;
    }
  }
  return motion_ti;
}

TrajectoryBuildResult build_sorted_of_ref_trajectory(
  const std::filesystem::path & input_path, const io::TopicInfo & motion_ti,
  const std::string & ref, const std::string & of, bool motion_is_twist, tf2::BufferCore & buffer,
  const char * logger)
{
  TrajectoryBuildResult out;
  if (const auto error = core::load_static_tf_buffer(input_path, buffer); error.has_value()) {
    // load_static_tf_buffer is a shared, caller-neutral helper (it names no
    // command's flags), so its detail is always safe to forward here.
    BAGWIZ_LOG_ERROR(
      logger,
      "pcd undistort: could not load the bag's static TF (needed to resolve --ref '%s' / --of "
      "'%s' and any --pcd topic's sensor extrinsic); detail: %s",
      ref.c_str(), of.c_str(), error->c_str());
    out.error = *error;
    return out;
  }

  if (motion_is_twist) {
    out = build_trajectory_from_twist_topic(input_path, motion_ti, of, buffer);
    if (!out.ok()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd undistort: could not build the motion trajectory from twist topic '%s': %s",
        motion_ti.name.c_str(), out.error.c_str());
      return out;
    }
  } else {
    out = (motion_ti.type == kTfMessageType)
            ? build_trajectory_from_tf_message(input_path, motion_ti, ref, of, buffer)
            : build_trajectory_from_pose_topic(
                input_path, motion_ti, pose_compose_kind(motion_ti.type), ref, of, buffer);
    if (!out.ok()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd undistort: could not resolve --of '%s' -> --ref '%s' from pose topic '%s': %s",
        of.c_str(), ref.c_str(), motion_ti.name.c_str(), out.error.c_str());
      return out;
    }
  }
  std::sort(out.trajectory.begin(), out.trajectory.end(), [](const auto & a, const auto & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return out;
}

bool cloud_has_usable_point_time(
  const std::vector<core::pointcloud::PointField> & fields, std::uint32_t point_step)
{
  core::pointcloud::PointCloud2 shim;
  shim.fields = fields;
  const auto field = core::pointcloud::find_point_time_field(shim);
  if (!field.has_value()) {
    return false;
  }
  return static_cast<std::size_t>(field->offset) +
           core::pointcloud::datatype_size(field->datatype) <=
         point_step;
}

std::optional<std::unordered_map<std::string, PcdTopicState>> peek_pcd_topic_states(
  const std::filesystem::path & input_path, const std::vector<std::string> & pcd_topics,
  const char * logger)
{
  std::unordered_map<std::string, PcdTopicState> states;
  std::unique_ptr<io::BagReader> preader;
  try {
    preader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = pcd_topics;
  preader->set_filter(filter);
  std::unordered_set<std::string> pending(pcd_topics.begin(), pcd_topics.end());
  io::RawMessage raw;
  try {
    while (!pending.empty() && preader->next(raw)) {
      const auto it = pending.find(raw.topic->name);
      if (it == pending.end()) {
        continue;  // already peeked this topic's first message
      }
      const auto header = core::pointcloud::parse_pointcloud2_header(raw.payload);
      if (!header.ok()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: could not parse the first message on --pcd topic '%s': %s",
          raw.topic->name.c_str(), header.error.c_str());
        return std::nullopt;
      }
      PcdTopicState st;
      st.frame_id = header.header->frame_id;
      st.has_time = cloud_has_usable_point_time(header.header->fields, header.header->point_step);
      states.emplace(raw.topic->name, st);
      pending.erase(it);
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "read error peeking --pcd topics: %s", e.what());
    return std::nullopt;
  }
  return states;
}

bool validate_pcd_topic_states(
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger)
{
  for (const auto & topic : pcd_topics) {
    const auto it = states.find(topic);
    if (it == states.end()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd undistort: --pcd topic '%s' has no decodable PointCloud2 message",
        topic.c_str());
      return false;
    }
    if (!it->second.has_time) {
      BAGWIZ_LOG_ERROR(
        logger,
        "pcd undistort: --pcd topic '%s' has no per-point time field (checked t / time / "
        "time_stamp / timestamp); pcd undistort requires per-point time to deskew",
        topic.c_str());
      return false;
    }
  }
  return true;
}

std::optional<ExtrinsicMap> resolve_pcd_extrinsics(
  const tf2::BufferCore & buffer, const std::string & of,
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger)
{
  ExtrinsicMap extrinsics;
  for (const auto & topic : pcd_topics) {
    const std::string & frame_id = states.at(topic).frame_id;
    std::optional<geometry_msgs::msg::Transform> extrinsic;
    if (frame_id != of) {
      const auto resolved = core::pointcloud::resolve_static_extrinsic(buffer, of, frame_id);
      if (!resolved.missing.empty()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: --of '%s' has no static TF chain to --pcd topic '%s' frame '%s'",
          of.c_str(), topic.c_str(), frame_id.c_str());
        return std::nullopt;
      }
      if (!resolved.ok()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: --of '%s' -> --pcd topic '%s' frame '%s' TF lookup failed: %s",
          of.c_str(), topic.c_str(), frame_id.c_str(), resolved.lookup_error.c_str());
        return std::nullopt;
      }
      extrinsic = resolved.transform.transform;
    }
    extrinsics.emplace(topic, extrinsic);
  }
  return extrinsics;
}

}  // namespace bagwiz::commands
