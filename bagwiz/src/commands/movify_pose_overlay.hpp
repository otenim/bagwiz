// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_POSE_OVERLAY_HPP_
#define COMMANDS__MOVIFY_POSE_OVERLAY_HPP_

#include "bagwiz/core/tf/trajectory.hpp"
#include "movify_layout.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>
#include <tf2/buffer_core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// The trajectory overlay: a --pose topic (Odometry, PoseStamped or
// PoseWithCovarianceStamped) read whole into the body's trajectory in its
// world frame, and, per tick, the stretch of it around the tick expressed
// in a panel's frame — the bag's static TF bridging body and panel frames —
// which the camera and point-cloud panels draw over their pictures as
// plates laid on the ground along the path.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// The loaded overlay. Immobile: it owns the static-TF buffer its lookups
// go through (tf2::BufferCore holds a mutex).
struct PoseOverlay
{
  std::string topic;
  std::string world_frame;                          // the pose messages' header.frame_id
  std::string body_frame;                           // the frame the trajectory is of
  std::vector<core::TrajectoryPose> poses;          // sorted by stamp
  double window_s = 10.0;                           // drawn on each side of the tick
  tf2::BufferCore buffer{std::chrono::seconds{1}};  // static TF only

  PoseOverlay() = default;
  PoseOverlay(const PoseOverlay &) = delete;
  PoseOverlay & operator=(const PoseOverlay &) = delete;
};

// Outcome of load_pose_overlay(): the overlay, or the error (already logged).
struct PoseOverlayResult
{
  std::unique_ptr<PoseOverlay> overlay;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return overlay != nullptr && error.empty(); }
};

// Read the --pose topic whole: the body's trajectory in the messages' own
// frame, bridged through the bag's static TF when `body_frame` (--pose-of)
// is not the frame the messages pose (for Odometry, its child_frame_id;
// for the pose topics, which do not name a body, `body_frame` is asserted).
// Errors: the topic is absent or of another type, has no messages, a message
// lacks its frames, the body frame (or an Odometry's child frame) is not in
// the bag's static TF, or a bridge does not resolve.
[[nodiscard]] PoseOverlayResult load_pose_overlay(
  const std::filesystem::path & input, const std::string & topic,
  const std::optional<std::string> & body_frame, double window_s);

// The panels' rendering of the trajectory: plates laid on the ground
// along the path, every kPoseTileSpacingM meters of it, each
// kPoseTileLengthRatio of that spacing long and `width_m` wide across the
// path (the vehicle's width, say), the way end-to-end driving demos show a
// planned path. The four corners are in the panel's frame, in order around
// the plate; `ahead` tells the stretch ahead of the body from the one
// behind; `fade` runs from 1 at the body to 0 at the far end of the window,
// for the plates to thin out with distance.
inline constexpr double kPoseTileSpacingM = 2.0;
inline constexpr double kPoseTileLengthRatio = 0.6;

struct PoseTile
{
  std::array<std::array<double, 3>, 4> corners;
  bool ahead = true;
  double fade = 1.0;
};

// The plates in `frame` at `stamp_ns`: every pose within the window is
// moved from the world frame into the body's frame at the tick (the inverse
// of the interpolated pose) and on into `frame` through the static TF; the
// plates are laid along the trajectory resampled by arc length, across the
// ground plane of the world frame (its +z up). A window shorter than one
// spacing yields none. nullopt with `error` set when the static chain
// body -> frame does not resolve.
[[nodiscard]] std::optional<std::vector<PoseTile>> pose_tiles_in_frame(
  const PoseOverlay & overlay, const std::string & frame, std::int64_t stamp_ns, double width_m,
  std::string & error);

// A plate projected into the cell: its corners in pixels, in order.
struct ProjectedPoseTile
{
  std::array<cv::Point, 4> corners;
  bool ahead = true;
  double fade = 1.0;
  double depth = 0.0;  // the plate's mean distance along the optical axis
};

// How far outside the picture a plate's corner may project (as a multiple
// of the picture size) before the plate is dropped: a corner grazing the
// camera's own plane projects to absurd coordinates that would overflow the
// drawing.
inline constexpr double kPoseTileMaxOverhang = 8.0;

// One corner of a plate projected by a panel's view: its pixel in the
// panel's picture (not yet clipped) and its distance along the view's axis.
struct ProjectedPoseCorner
{
  double u = 0.0;
  double v = 0.0;
  double depth = 0.0;
};

// A corner in the panel's frame to its projection, or nullopt when the
// view cannot place it (behind a perspective camera, say).
using PoseCornerProjector =
  std::function<std::optional<ProjectedPoseCorner>(const std::array<double, 3> &)>;

// Where a panel's picture sits in its cell: the picture's size, which the
// overhang guard is relative to, and its offset within the cell (a camera
// frame pasted centered between black bars; zero for a picture that fills
// the cell).
struct PoseTilePlacement
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t x_off = 0;
  std::uint32_t y_off = 0;
};

// Project plates into a cell through `project`: a plate whose four corners
// all project to finite pixels within kPoseTileMaxOverhang of the picture
// becomes a ProjectedPoseTile (its corners offset into the cell, its depth
// the mean of theirs); one with a corner that does not — behind the camera,
// or grazing its plane and shooting off to absurd coordinates — cannot be
// drawn as a quadrilateral and is left out.
[[nodiscard]] std::vector<ProjectedPoseTile> project_pose_tiles(
  const std::vector<PoseTile> & tiles, const PoseCornerProjector & project,
  const PoseTilePlacement & placement);

// Draw the plates over `canvas`, farthest first so nearer plates lie on
// top: a translucent fill (fading with `fade`) and, on plates tall enough to
// show one, a thin outline; the stretch ahead in the trajectory's color, the
// one behind dimmer. Plates partly off the canvas are clipped; slivers a
// pixel high are skipped.
void draw_pose_tiles(
  cv::Mat & canvas, const std::vector<ProjectedPoseTile> & tiles, double ui_scale);

// The outline widths are sized for a 720-px-tall cell and scale with the
// cell height, within bounds, like the map panel's elements.
[[nodiscard]] double pose_ui_scale(std::uint32_t cell_height) noexcept;

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_POSE_OVERLAY_HPP_
