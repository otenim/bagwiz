// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_pose_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/commands/topic_types.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/pointcloud/static_extrinsic.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/topics.hpp"
#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/imgproc.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";
constexpr const char * kDefaultBodyFrame = "base_link";
constexpr std::int64_t kNsPerSecond = 1'000'000'000LL;

// Colors, BGR.
const cv::Scalar kFutureColor(0, 170, 255);  // orange
const cv::Scalar kPastColor(210, 210, 210);

// The first message's frames: the world frame every message poses in and,
// for Odometry, the body frame it poses. nullopt with `error` set when the
// topic has no message or the first does not parse.
struct PoseFrames
{
  std::string world;
  std::string body;  // empty unless Odometry
};

std::optional<PoseFrames> peek_pose_frames(
  const std::filesystem::path & input, const io::TopicInfo & topic, std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    error = "failed to open '" + input.string() + "': " + e.what();
    return std::nullopt;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {topic.name};
  reader->set_filter(filter);
  auto open = core::decoder::open_decoder(topic);
  if (!open.ok()) {
    error = "could not open a decoder for pose topic '" + topic.name + "': " + open.error;
    return std::nullopt;
  }
  io::RawMessage raw;
  try {
    if (!reader->next(raw)) {
      error = "topic '" + topic.name + "' has no messages to render.";
      return std::nullopt;
    }
  } catch (const std::exception & e) {
    error = "error reading topic '" + topic.name + "': " + e.what();
    return std::nullopt;
  }
  const auto decoded = open.decoder->decode(raw.payload);
  if (!decoded.ok()) {
    error =
      "failed to decode the first message of pose topic '" + topic.name + "': " + decoded.error;
    return std::nullopt;
  }
  PoseSample sample;
  if (!decode_pose_sample(pose_compose_kind(topic.type), *decoded.value, sample)) {
    error = "the first message of pose topic '" + topic.name + "' is not a " + topic.type;
    return std::nullopt;
  }
  PoseFrames frames;
  frames.world = sample.pose.header.frame_id;
  frames.body = sample.child_frame;
  if (frames.world.empty()) {
    error = "pose topic '" + topic.name + "': the first message has an empty header.frame_id";
    return std::nullopt;
  }
  return frames;
}

tf2::Transform as_transform(const core::TrajectoryPose & pose)
{
  return tf2::Transform(
    tf2::Quaternion(pose.qx, pose.qy, pose.qz, pose.qw), tf2::Vector3(pose.tx, pose.ty, pose.tz));
}

std::array<double, 3> apply(const tf2::Transform & tf, double x, double y, double z)
{
  const tf2::Vector3 p = tf * tf2::Vector3(x, y, z);
  return {p.x(), p.y(), p.z()};
}
}  // namespace

PoseOverlayResult load_pose_overlay(
  const std::filesystem::path & input, const std::string & topic,
  const std::optional<std::string> & body_frame, double window_s)
{
  PoseOverlayResult out;
  const auto fail = [&](std::string message) -> PoseOverlayResult {
    BAGWIZ_LOG_ERROR(kLogger, "%s", message.c_str());
    out.error = std::move(message);
    out.overlay.reset();
    return std::move(out);
  };

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return fail("failed to open '" + input.string() + "': " + e.what());
  }
  const io::TopicInfo * info = io::find_topic(*reader, topic);
  if (info == nullptr) {
    return fail("pose topic '" + topic + "' not found in " + input.string());
  }
  const bool supported = std::any_of(
    kMovifyPoseTopicTypes.begin(), kMovifyPoseTopicTypes.end(),
    [&](std::string_view type) { return info->type == type; });
  if (!supported) {
    return fail(
      "pose topic '" + topic + "' has type '" + info->type +
      "', expected nav_msgs/msg/Odometry, geometry_msgs/msg/PoseStamped or "
      "geometry_msgs/msg/PoseWithCovarianceStamped");
  }
  // The reader's topic list is what the builder and the peek index into;
  // copy the entry, since the builders reopen the bag themselves.
  const io::TopicInfo topic_info = *info;
  const bool has_static_tf = std::any_of(
    reader->topics().begin(), reader->topics().end(),
    [](const io::TopicInfo & t) { return core::is_static_tf_topic(t); });
  reader.reset();

  std::string error;
  const auto frames = peek_pose_frames(input, topic_info, error);
  if (!frames.has_value()) {
    return fail(error);
  }

  auto overlay = std::make_unique<PoseOverlay>();
  overlay->topic = topic;
  overlay->world_frame = frames->world;
  overlay->body_frame =
    body_frame.value_or(frames->body.empty() ? std::string{kDefaultBodyFrame} : frames->body);
  overlay->window_s = window_s;

  // The bag's static TF, kept in the overlay's buffer for the builder's
  // bridges and the panels' body -> panel-frame lookups. A bag without it can
  // still draw in the body's own frame.
  if (has_static_tf) {
    if (const auto err = core::load_static_tf_buffer(input, overlay->buffer); err.has_value()) {
      return fail("pose topic '" + topic + "': " + *err);
    }
  } else {
    BAGWIZ_LOG_INFO(
      kLogger,
      "the bag has no static TF: the --pose trajectory is drawn only in its body "
      "frame '%s'.",
      overlay->body_frame.c_str());
  }
  // The panels place the trajectory through the static TF from the body's
  // frame, so a frame the static TF does not know cannot be drawn anywhere:
  // an Odometry's child frame (an INS's own link that was never published,
  // say) or the body a pose topic is taken as (--pose-of, else base_link).
  // Stop here rather than at the first frame.
  const auto require_frame = [&](const std::string & frame, const char * role) {
    if (overlay->buffer._frameExists(frame)) {
      return std::string{};
    }
    return "pose topic '" + topic + "': its " + role + " '" + frame +
           "' is not in the bag's static TF, so the trajectory cannot be placed in the panels' "
           "frames; add the frame's static transform to the bag (bagwiz tf static update) first.";
  };
  if (!frames->body.empty()) {
    if (const auto err = require_frame(frames->body, "Odometry child frame"); !err.empty()) {
      return fail(err);
    }
  }
  if (const auto err = require_frame(
        overlay->body_frame, frames->body.empty()
                               ? "body frame (--pose-of, or base_link when unset)"
                               : "--pose-of frame");
      !err.empty()) {
    return fail(err);
  }
  auto built = build_sorted_of_ref_trajectory(
    input, topic_info, overlay->world_frame, overlay->body_frame, /*motion_is_twist=*/false,
    overlay->buffer, kLogger, StaticTfInBuffer::kPreloaded);
  if (!built.ok()) {
    return fail(built.error.empty() ? "pose topic '" + topic + "' yielded no poses" : built.error);
  }
  overlay->poses = std::move(built.trajectory);
  BAGWIZ_LOG_INFO(
    kLogger, "pose overlay: %zu poses of '%s' in '%s' from '%s', +-%.0f s drawn.",
    overlay->poses.size(), overlay->body_frame.c_str(), overlay->world_frame.c_str(), topic.c_str(),
    window_s);
  out.overlay = std::move(overlay);
  return out;
}

namespace
{
// The trajectory window around one tick in the world frame, and the
// transform that takes the world into the panel's frame at that tick.
struct WorldWindow
{
  std::vector<std::array<double, 3>> past;    // up to and including the body
  std::vector<std::array<double, 3>> future;  // from the body on
  tf2::Transform frame_from_world;
};

std::optional<WorldWindow> world_window(
  const PoseOverlay & overlay, const std::string & frame, std::int64_t stamp_ns,
  std::string & error)
{
  const auto current = core::lookup_pose(stamp_ns, overlay.poses);
  if (!current.has_value()) {
    error = "the pose overlay has no poses";
    return std::nullopt;
  }
  // frame <- world = (frame <- body) * (body <- world at the tick).
  tf2::Transform frame_from_body;
  frame_from_body.setIdentity();
  if (frame != overlay.body_frame) {
    const auto resolved =
      core::pointcloud::resolve_static_extrinsic(overlay.buffer, frame, overlay.body_frame);
    if (!resolved.missing.empty() || !resolved.ok()) {
      error = "no static TF chain from the pose body frame '" + overlay.body_frame + "' to '" +
              frame + "'" + (resolved.lookup_error.empty() ? "" : ": " + resolved.lookup_error);
      return std::nullopt;
    }
    const auto & t = resolved.transform.transform;
    frame_from_body = tf2::Transform(
      tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
      tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
  }
  WorldWindow out;
  out.frame_from_world = frame_from_body * as_transform(*current).inverse();

  const auto window_ns = static_cast<std::int64_t>(overlay.window_s * kNsPerSecond);
  const auto by_stamp = [](const core::TrajectoryPose & pose, std::int64_t ns) {
    return pose.timestamp_ns < ns;
  };
  const auto & poses = overlay.poses;
  const auto first = std::lower_bound(poses.begin(), poses.end(), stamp_ns - window_ns, by_stamp);
  const auto split = std::lower_bound(poses.begin(), poses.end(), stamp_ns, by_stamp);
  const auto last = std::upper_bound(
    poses.begin(), poses.end(), stamp_ns + window_ns,
    [](std::int64_t ns, const core::TrajectoryPose & pose) { return ns < pose.timestamp_ns; });
  for (auto it = first; it != split; ++it) {
    out.past.push_back({it->tx, it->ty, it->tz});
  }
  // The pose at the tick joins the two halves.
  const std::array<double, 3> here{current->tx, current->ty, current->tz};
  out.past.push_back(here);
  out.future.push_back(here);
  for (auto it = split; it != last; ++it) {
    if (it->timestamp_ns == stamp_ns) {
      continue;  // already the junction
    }
    out.future.push_back({it->tx, it->ty, it->tz});
  }
  return out;
}

std::array<double, 3> apply(const tf2::Transform & tf, const std::array<double, 3> & p)
{
  return apply(tf, p[0], p[1], p[2]);
}

// The plates along one stretch of the path (world frame), walking it by arc
// length from its first point: a plate every spacing, its ends' lateral
// axes from the path's direction there and the world's up. `total_m` is
// the stretch's length, for the fade.
void lay_tiles(
  const std::vector<std::array<double, 3>> & path, double width_m, bool ahead,
  const tf2::Transform & frame_from_world, std::vector<PoseTile> & out)
{
  if (path.size() < 2) {
    return;
  }
  // Cumulative arc length at each point.
  std::vector<double> at(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    at[i] = at[i - 1] + std::hypot(
                          path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1],
                          path[i][2] - path[i - 1][2]);
  }
  const double total = at.back();
  // The point and unit direction at arc length `s`.
  // `s` only grows over the walk below, so the segment search resumes from
  // the last segment instead of the start.
  std::size_t cursor = 1;
  const auto sample = [&](double s, std::array<double, 3> & point, std::array<double, 3> & dir) {
    std::size_t & i = cursor;
    while (i + 1 < path.size() && at[i] < s) {
      ++i;
    }
    const double seg = at[i] - at[i - 1];
    const double t = seg > 0.0 ? std::clamp((s - at[i - 1]) / seg, 0.0, 1.0) : 0.0;
    for (int k = 0; k < 3; ++k) {
      point[k] = path[i - 1][k] + t * (path[i][k] - path[i - 1][k]);
      dir[k] = path[i][k] - path[i - 1][k];
    }
    const double len = std::hypot(dir[0], dir[1], dir[2]);
    if (len > 0.0) {
      for (auto & c : dir) {
        c /= len;
      }
    }
  };
  // The lateral axis: the world's up crossed with the direction, i.e. the
  // path's left, flattened to the ground plane. A direction with no ground
  // component (a path going straight up) keeps the previous axis.
  std::array<double, 3> last_left{0.0, 1.0, 0.0};
  const auto left_of = [&last_left](const std::array<double, 3> & dir) {
    std::array<double, 3> left{-dir[1], dir[0], 0.0};
    const double len = std::hypot(left[0], left[1]);
    if (len > 0.0) {
      left[0] /= len;
      left[1] /= len;
      last_left = left;
    }
    return last_left;
  };
  const double tile_len = kPoseTileSpacingM * kPoseTileLengthRatio;
  for (double s = 0.0; s + tile_len <= total; s += kPoseTileSpacingM) {
    std::array<double, 3> c0{};
    std::array<double, 3> d0{};
    std::array<double, 3> c1{};
    std::array<double, 3> d1{};
    sample(s, c0, d0);
    sample(s + tile_len, c1, d1);
    const auto l0 = left_of(d0);
    const auto l1 = left_of(d1);
    const double h = width_m / 2.0;
    PoseTile tile;
    tile.ahead = ahead;
    tile.fade = total > 0.0 ? std::clamp(1.0 - s / total, 0.0, 1.0) : 1.0;
    const std::array<std::array<double, 3>, 4> world{{
      {c0[0] + h * l0[0], c0[1] + h * l0[1], c0[2]},
      {c1[0] + h * l1[0], c1[1] + h * l1[1], c1[2]},
      {c1[0] - h * l1[0], c1[1] - h * l1[1], c1[2]},
      {c0[0] - h * l0[0], c0[1] - h * l0[1], c0[2]},
    }};
    for (std::size_t k = 0; k < 4; ++k) {
      tile.corners[k] = apply(frame_from_world, world[k]);
    }
    out.push_back(tile);
  }
}
}  // namespace

std::optional<std::vector<PoseTile>> pose_tiles_in_frame(
  const PoseOverlay & overlay, const std::string & frame, std::int64_t stamp_ns, double width_m,
  std::string & error)
{
  const auto window = world_window(overlay, frame, stamp_ns, error);
  if (!window.has_value()) {
    return std::nullopt;
  }
  std::vector<PoseTile> tiles;
  lay_tiles(window->future, width_m, true, window->frame_from_world, tiles);
  // The stretch behind is walked away from the body too, so its plates fade
  // with the distance behind.
  std::vector<std::array<double, 3>> behind(window->past.rbegin(), window->past.rend());
  lay_tiles(behind, width_m, false, window->frame_from_world, tiles);
  return tiles;
}

std::vector<ProjectedPoseTile> project_pose_tiles(
  const std::vector<PoseTile> & tiles, const PoseCornerProjector & project,
  const PoseTilePlacement & placement)
{
  const double max_u = kPoseTileMaxOverhang * placement.width;
  const double max_v = kPoseTileMaxOverhang * placement.height;
  std::vector<ProjectedPoseTile> out;
  out.reserve(tiles.size());
  for (const auto & tile : tiles) {
    ProjectedPoseTile projected;
    projected.ahead = tile.ahead;
    projected.fade = tile.fade;
    bool visible = true;
    double depth = 0.0;
    for (std::size_t k = 0; k < 4; ++k) {
      const auto corner = project(tile.corners[k]);
      if (
        !corner.has_value() || !std::isfinite(corner->u) || !std::isfinite(corner->v) ||
        std::abs(corner->u) > max_u || std::abs(corner->v) > max_v) {
        visible = false;
        break;
      }
      projected.corners[k] = cv::Point(
        static_cast<int>(std::lround(corner->u)) + static_cast<int>(placement.x_off),
        static_cast<int>(std::lround(corner->v)) + static_cast<int>(placement.y_off));
      depth += corner->depth / 4.0;
    }
    if (visible) {
      projected.depth = depth;
      out.push_back(projected);
    }
  }
  return out;
}

void draw_pose_tiles(
  cv::Mat & canvas, const std::vector<ProjectedPoseTile> & tiles, double ui_scale)
{
  constexpr double kNearAlpha = 0.55;
  constexpr double kFarAlpha = 0.12;
  // Far plates shrink to slivers: below this projected height a plate is
  // not drawn at all (it would only darken the road), and below the outline
  // threshold only its fill is, so the far end does not clot into a solid
  // strip of outlines.
  constexpr int kMinPlateHeightPx = 2;
  constexpr int kOutlineMinHeightPx = 6;
  const int outline = std::max(1, static_cast<int>(std::lround(1.5 * ui_scale)));
  const cv::Rect bounds(0, 0, canvas.cols, canvas.rows);
  std::vector<const ProjectedPoseTile *> far_first;
  far_first.reserve(tiles.size());
  for (const auto & tile : tiles) {
    far_first.push_back(&tile);
  }
  std::stable_sort(
    far_first.begin(), far_first.end(),
    [](const ProjectedPoseTile * a, const ProjectedPoseTile * b) { return a->depth > b->depth; });
  for (const auto * tile_ptr : far_first) {
    const auto & tile = *tile_ptr;
    const cv::Scalar & color = tile.ahead ? kFutureColor : kPastColor;
    const std::vector<cv::Point> quad(tile.corners.begin(), tile.corners.end());
    // Blend only the plate's own bounding box: a translucent fill of the
    // plate, then its outline.
    const cv::Rect plate = cv::boundingRect(quad);
    const cv::Rect roi = plate & bounds;
    if (roi.width <= 0 || roi.height <= 0 || plate.height < kMinPlateHeightPx) {
      continue;
    }
    const double alpha = kFarAlpha + (kNearAlpha - kFarAlpha) * std::clamp(tile.fade, 0.0, 1.0);
    cv::Mat region = canvas(roi);
    cv::Mat filled = region.clone();
    std::vector<cv::Point> local;
    local.reserve(4);
    for (const auto & c : quad) {
      local.emplace_back(c.x - roi.x, c.y - roi.y);
    }
    cv::fillConvexPoly(filled, local, color, cv::LINE_AA);
    cv::addWeighted(region, 1.0 - alpha, filled, alpha, 0.0, region);
    if (plate.height >= kOutlineMinHeightPx) {
      cv::polylines(canvas, quad, true, color, outline, cv::LINE_AA);
    }
  }
}

double pose_ui_scale(std::uint32_t cell_height) noexcept
{
  constexpr double kReferenceHeight = 720.0;
  return std::clamp(cell_height / kReferenceHeight, 0.5, 4.0);
}

}  // namespace bagwiz::commands
