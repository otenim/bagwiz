// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "calib_cam_lidar_offset.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/time_offset.hpp"
#include "bagwiz/core/calib/visual_rotation.hpp"
#include "bagwiz/core/image/camera_distortion.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/slam/imu_sample.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr const char * kLogger = "bagwiz.cmd.calib.cam_lidar";
constexpr std::string_view kImuMsgType = "sensor_msgs/msg/Imu";
// A frame pair contributes to a series only when its solver rested on at
// least this many inliers — below it the rotation is too noisy to time
// against, and a pair with no solution at all is skipped anyway.
constexpr std::size_t kMinPairInliers = 50;
// Trajectory intervals for the gyro leg: consecutive poses no further apart
// than this (a pose topic gap is not a rotation measurement).
constexpr std::int64_t kMaxTrajectoryIntervalNs = 500'000'000;

// The frames to track: every eligible frame when there are at most
// kCamOffsetMaxFrames of them, else kCamOffsetMaxFrames / kCamOffsetBlockFrames
// contiguous blocks spread evenly over the eligible range. Returns, per image
// index, the block id it belongs to (nullopt = not used).
std::vector<std::optional<std::size_t>> select_frames(
  std::span<const std::int64_t> stamps, std::int64_t traj_begin_ns, std::int64_t traj_end_ns)
{
  std::vector<std::size_t> eligible;
  for (std::size_t i = 0; i < stamps.size(); ++i) {
    if (stamps[i] >= traj_begin_ns && stamps[i] <= traj_end_ns) {
      eligible.push_back(i);
    }
  }
  std::vector<std::optional<std::size_t>> block_of(stamps.size());
  if (eligible.size() <= kCamOffsetMaxFrames) {
    for (const std::size_t i : eligible) {
      block_of[i] = 0;
    }
    return block_of;
  }
  const std::size_t blocks = kCamOffsetMaxFrames / kCamOffsetBlockFrames;
  for (std::size_t b = 0; b < blocks; ++b) {
    const std::size_t start =
      blocks > 1 ? (b * (eligible.size() - kCamOffsetBlockFrames)) / (blocks - 1) : 0;
    for (std::size_t k = 0; k < kCamOffsetBlockFrames; ++k) {
      block_of[eligible[start + k]] = b;
    }
  }
  return block_of;
}

// Rotate a vector by the rotation block of a column-major Mat4.
std::array<double, 3> rotate(const core::calib::Mat4 & t, const std::array<double, 3> & v)
{
  return {
    t[0] * v[0] + t[4] * v[1] + t[8] * v[2],
    t[1] * v[0] + t[5] * v[1] + t[9] * v[2],
    t[2] * v[0] + t[6] * v[1] + t[10] * v[2],
  };
}

struct VisualSeries
{
  std::vector<core::calib::RotationInterval> essential;
  std::vector<core::calib::RotationInterval> rotation;
  std::size_t frames_tracked = 0;
  std::size_t pairs_attempted = 0;
};

// Decode the selected frames in bag order and turn every consecutive pair
// inside a block into rotation intervals (both solvers), rotated into the
// --of frame.
std::optional<VisualSeries> track_frames(
  const CamOffsetEstimateInput & in, std::span<const std::optional<std::size_t>> block_of)
{
  auto reader = io::open_read_or_log(in.args.input_path, kLogger);
  if (!reader) {
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = {in.args.cam_topic};
  reader->set_filter(filter);

  VisualSeries series;
  core::image::ImageDecoder decoder;
  std::optional<core::calib::GrayImage> prev;
  std::optional<std::size_t> prev_block;
  std::int64_t prev_stamp_ns = 0;
  // The tracking camera model and the downscale it was built for, from the
  // first decoded frame.
  struct ScaledCamera
  {
    core::calib::CameraModel cam;
    double scale = 1.0;
  };
  std::optional<ScaledCamera> cam;
  const core::calib::VisualRotationParams params;
  io::RawMessage raw;
  try {
    std::size_t idx = 0;
    while (idx < block_of.size() && reader->next(raw)) {
      const std::size_t i = idx++;
      if (!block_of[i].has_value()) {
        continue;
      }
      const auto packed = core::image::to_packed_raster(raw.topic->type, raw.payload, decoder);
      if (!packed.ok()) {
        BAGWIZ_LOG_WARN(
          kLogger, "cam-offset: skipping frame at %" PRId64 " ns: failed to decode: %s",
          in.image_stamps_ns[i], packed.error.c_str());
        prev.reset();
        continue;
      }
      const auto full = core::calib::gray_from_bgr24(
        packed.raster->bgr, packed.raster->width, packed.raster->height);
      if (!cam.has_value()) {
        const auto ci = core::image::camera_info_for_size(in.cam_info, full.width, full.height);
        core::calib::CameraModel model;
        model.k = ci.k;
        model.model = core::image::select_distortion_model(ci.distortion_model);
        model.d = ci.d;
        model.width = full.width;
        model.height = full.height;
        const double scale =
          core::calib::scale_for_max_side(full.width, full.height, kCamOffsetMaxImageSide);
        cam = ScaledCamera{core::calib::scale_camera_model(model, scale), scale};
      }
      auto gray = core::calib::downscale_gray(full, cam->scale);
      if (gray.width != cam->cam.width || gray.height != cam->cam.height) {
        BAGWIZ_LOG_WARN(
          kLogger, "cam-offset: skipping frame at %" PRId64 " ns: size %ux%u differs from %ux%u.",
          in.image_stamps_ns[i], gray.width, gray.height, cam->cam.width, cam->cam.height);
        prev.reset();
        continue;
      }
      ++series.frames_tracked;
      if (prev.has_value() && prev_block == block_of[i] && in.image_stamps_ns[i] > prev_stamp_ns) {
        ++series.pairs_attempted;
        const auto r = core::calib::frame_pair_rotation(*prev, gray, cam->cam, params);
        if (r.essential.has_value() && r.inliers_essential >= kMinPairInliers) {
          series.essential.push_back(
            {prev_stamp_ns, in.image_stamps_ns[i], rotate(in.t_trajframe_cam0, *r.essential)});
        }
        if (r.pure_rotation.has_value() && r.inliers_rotation >= kMinPairInliers) {
          series.rotation.push_back(
            {prev_stamp_ns, in.image_stamps_ns[i], rotate(in.t_trajframe_cam0, *r.pure_rotation)});
        }
      }
      prev = std::move(gray);
      prev_block = block_of[i];
      prev_stamp_ns = in.image_stamps_ns[i];
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Error reading topic '%s': %s", in.args.cam_topic.c_str(), e.what());
    return std::nullopt;
  }
  return series;
}

// The --imu topic as gyro samples in the --of frame (rotated through the
// static chain from --of to the IMU's header.frame_id). nullopt after logging
// when the topic is missing or wrong-typed, no sample parses, or the chain is
// absent.
std::optional<std::vector<core::calib::GyroSample>> read_gyro(const CamOffsetEstimateInput & in)
{
  auto reader = io::open_read_or_log(in.args.input_path, kLogger);
  if (!reader) {
    return std::nullopt;
  }
  const io::TopicInfo * ti =
    io::find_topic_or_log(*reader, in.args.imu_topic, in.args.input_path, kLogger);
  if (ti == nullptr) {
    return std::nullopt;
  }
  if (ti->type != kImuMsgType) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Topic '%s' has type '%s', which `--imu` cannot read; expected %s.",
      in.args.imu_topic.c_str(), ti->type.c_str(), std::string(kImuMsgType).c_str());
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = {in.args.imu_topic};
  reader->set_filter(filter);

  std::vector<core::slam::ImuSample> samples;
  std::size_t failed = 0;
  std::string frame;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto parsed = core::slam::parse_imu(raw.payload);
      if (!parsed.ok()) {
        ++failed;
        continue;
      }
      if (frame.empty()) {
        frame = parsed.sample->frame_id;
      }
      samples.push_back(std::move(*parsed.sample));
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Error reading topic '%s': %s", in.args.imu_topic.c_str(), e.what());
    return std::nullopt;
  }
  if (samples.size() < 2) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Topic '%s' yielded %zu usable Imu message(s) (%zu failed to parse); at least 2 "
      "are needed to integrate the gyro.",
      in.args.imu_topic.c_str(), samples.size(), failed);
    return std::nullopt;
  }
  if (failed > 0) {
    BAGWIZ_LOG_WARN(
      kLogger, "%zu of %zu Imu message(s) on '%s' failed to parse; using the rest.", failed,
      samples.size() + failed, in.args.imu_topic.c_str());
  }
  if (frame.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Imu topic '%s' has an empty header.frame_id; cannot resolve its chain from --of.",
      in.args.imu_topic.c_str());
    return std::nullopt;
  }
  core::calib::Mat4 t_of_imu;
  try {
    const auto ts = in.static_buffer.lookupTransform(in.args.of_frame, frame, tf2::TimePointZero);
    const auto m = mat4_from_quat(
      ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z,
      ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
      ts.transform.rotation.w);
    if (!m.has_value()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "The static transform %s -> %s is not a usable rigid transform.",
        in.args.of_frame.c_str(), frame.c_str());
      return std::nullopt;
    }
    t_of_imu = *m;
  } catch (const tf2::TransformException & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "No static TF path from --of '%s' to the Imu frame '%s': %s",
      in.args.of_frame.c_str(), frame.c_str(), e.what());
    return std::nullopt;
  }
  std::sort(samples.begin(), samples.end(), [](const auto & a, const auto & b) {
    return a.stamp_ns < b.stamp_ns;
  });
  std::vector<core::calib::GyroSample> gyro;
  gyro.reserve(samples.size());
  for (const auto & s : samples) {
    gyro.push_back({s.stamp_ns, rotate(t_of_imu, s.angular_velocity)});
  }
  BAGWIZ_LOG_INFO(
    kLogger, "cam-offset: %zu Imu sample(s) on '%s' (frame '%s') span %.3f s.", gyro.size(),
    in.args.imu_topic.c_str(), frame.c_str(),
    static_cast<double>(gyro.back().stamp_ns - gyro.front().stamp_ns) / 1e9);
  return gyro;
}

struct SeriesFit
{
  const char * name = "";
  core::calib::TimeOffsetResult result;
  std::optional<core::calib::GyroBridgedOffset> bridged;
};

// The series that explains more of its own signal wins (residual after the
// fit relative to the signal rms); on a tie the smaller spread.
bool better_than(const SeriesFit & a, const SeriesFit & b)
{
  if (!a.result.ok) {
    return false;
  }
  if (!b.result.ok) {
    return true;
  }
  const auto ratio = [](const SeriesFit & f) {
    return f.result.fit.signal_rms_rad > 0.0
             ? f.result.fit.residual_rms_after_rad / f.result.fit.signal_rms_rad
             : 1.0;
  };
  const double ra = ratio(a);
  const double rb = ratio(b);
  if (ra != rb) {
    return ra < rb;
  }
  return a.result.fit.std_ns < b.result.fit.std_ns;
}

}  // namespace

std::optional<CamOffsetEstimateReport> estimate_cam_offset(const CamOffsetEstimateInput & in)
{
  if (in.poses.size() < 2) {
    BAGWIZ_LOG_ERROR(kLogger, "--cam-offset auto needs at least 2 trajectory poses.");
    return std::nullopt;
  }
  const auto block_of =
    select_frames(in.image_stamps_ns, in.poses.front().timestamp_ns, in.poses.back().timestamp_ns);
  const std::size_t selected = static_cast<std::size_t>(
    std::count_if(block_of.begin(), block_of.end(), [](const auto & b) { return b.has_value(); }));
  BAGWIZ_LOG_INFO(
    kLogger, "--cam-offset auto: tracking %zu of %zu image(s) on '%s' against %s.", selected,
    in.image_stamps_ns.size(), in.args.cam_topic.c_str(),
    in.args.imu_topic.empty() ? "the --pose trajectory" : "the --imu gyro");

  const auto series = track_frames(in, block_of);
  if (!series.has_value()) {
    return std::nullopt;
  }
  BAGWIZ_LOG_INFO(
    kLogger,
    "--cam-offset auto: %zu frame(s) decoded, %zu pair(s) attempted: %zu essential-matrix and %zu "
    "pure-rotation estimate(s).",
    series->frames_tracked, series->pairs_attempted, series->essential.size(),
    series->rotation.size());

  const core::calib::TimeOffsetParams params;
  std::optional<core::calib::GyroIntegral> gyro;
  std::vector<core::calib::RotationInterval> trajectory_intervals;
  if (!in.args.imu_topic.empty()) {
    const auto samples = read_gyro(in);
    if (!samples.has_value()) {
      return std::nullopt;
    }
    gyro.emplace(*samples);
    trajectory_intervals =
      core::calib::trajectory_rotation_intervals(in.poses, kMaxTrajectoryIntervalNs);
  }
  const auto reference = core::calib::trajectory_rotation_provider(in.poses);

  std::array<SeriesFit, 2> fits;
  fits[0].name = "essential";
  fits[1].name = "rotation";
  const std::array<const std::vector<core::calib::RotationInterval> *, 2> inputs{
    &series->essential, &series->rotation};
  for (std::size_t k = 0; k < 2; ++k) {
    if (gyro.has_value()) {
      fits[k].bridged =
        core::calib::fit_time_offset_via_gyro(*inputs[k], trajectory_intervals, *gyro, params);
      fits[k].result = fits[k].bridged->combined;
    } else {
      fits[k].result = core::calib::fit_time_offset(*inputs[k], reference, params);
    }
    if (fits[k].result.ok) {
      BAGWIZ_LOG_INFO(
        kLogger,
        "--cam-offset auto: %s solver: %+.2f ms (+-%.2f ms, %zu pair(s); residual %.3f -> %.3f "
        "mrad of %.3f mrad signal).",
        fits[k].name, static_cast<double>(fits[k].result.fit.offset_ns) / 1e6,
        static_cast<double>(fits[k].result.fit.std_ns) / 1e6, fits[k].result.fit.intervals,
        fits[k].result.fit.residual_rms_before_rad * 1e3,
        fits[k].result.fit.residual_rms_after_rad * 1e3, fits[k].result.fit.signal_rms_rad * 1e3);
    } else {
      BAGWIZ_LOG_INFO(
        kLogger, "--cam-offset auto: %s solver: no estimate (%s).", fits[k].name,
        fits[k].result.error.c_str());
    }
  }
  const SeriesFit & best = better_than(fits[1], fits[0]) ? fits[1] : fits[0];
  if (!best.result.ok) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "--cam-offset auto: could not estimate the camera stamp offset (essential-matrix: %s; "
      "pure-rotation: %s). Pass --cam-offset <dur> with a known value instead.",
      fits[0].result.error.c_str(), fits[1].result.error.c_str());
    return std::nullopt;
  }

  CamOffsetEstimateReport report;
  report.offset_ns = best.result.fit.offset_ns;
  report.std_ns = best.result.fit.std_ns;
  report.method = gyro.has_value() ? "imu" : "trajectory";
  report.visual_estimator = best.name;
  report.pairs = best.result.fit.intervals;
  report.signal_rms_mrad = best.result.fit.signal_rms_rad * 1e3;
  report.residual_rms_before_mrad = best.result.fit.residual_rms_before_rad * 1e3;
  report.residual_rms_after_mrad = best.result.fit.residual_rms_after_rad * 1e3;
  if (best.bridged.has_value()) {
    report.camera_imu_offset_ns = best.bridged->sensor_gyro.fit.offset_ns;
    report.pose_imu_offset_ns = best.bridged->trajectory_gyro.fit.offset_ns;
    BAGWIZ_LOG_INFO(
      kLogger, "--cam-offset auto: camera vs gyro %+.2f ms, pose vs gyro %+.2f ms.",
      static_cast<double>(*report.camera_imu_offset_ns) / 1e6,
      static_cast<double>(*report.pose_imu_offset_ns) / 1e6);
  }
  BAGWIZ_LOG_INFO(
    kLogger, "Estimated --cam-offset %+.2f ms (+-%.2f ms, %zu frame pair(s), vs %s; %s solver).",
    static_cast<double>(report.offset_ns) / 1e6, static_cast<double>(report.std_ns) / 1e6,
    report.pairs, report.method.c_str(), report.visual_estimator.c_str());
  return report;
}

}  // namespace bagwiz::commands
