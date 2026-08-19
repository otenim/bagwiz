// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_
#define COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_

#include "bagwiz/core/calib/extrinsic_refine.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

// Internals of `bagwiz calib cam-lidar`, split out so the flag validation,
// sample picking, trajectory interpolation, and report rendering can be
// unit-tested without a bag or a real refinement run. Pure over the args, no
// bag access. CLI-internal: this header lives with the command sources and is
// not installed.
namespace bagwiz::commands
{

// Parsed arguments for `bagwiz calib cam-lidar`. Refines one static-TF edge on
// a camera's chain by registering the bag's LiDAR map (from a prior `bagwiz map
// slam` run) against the bag's images via NID, and writes a YAML that `bagwiz
// tf static update` applies.
struct CalibCamLidarArgs
{
  // -i,--input: bag path (file or directory). A std::filesystem::path (not a
  // string) because set_topic_input() binds the completion registry's topic
  // slots to it — the same shape every other slot-declaring command uses.
  std::filesystem::path input_path;
  std::string map_path;        // --map: dense map PCD from `map slam`
  std::string traj_path;       // --traj: TUM trajectory from `map slam`
  std::string traj_frame;      // --traj-frame: frame the trajectory poses express
  std::string topic;           // -t,--topic: image topic to calibrate against
  std::string parent_frame;    // --parent: parent frame of the edited static edge
  std::string child_frame;     // --child: child frame of the edited static edge
  std::string cam_info_topic;  // --cam-info; empty = auto-resolve from the image topic
  // Whether --cam-info was actually passed (CLI11 ->count() > 0), so an
  // explicit empty string can be told apart from "omitted" — mirrors `walk`'s
  // empty-vs-omitted rule for optional topic overrides.
  bool cam_info_given = false;
  std::string
    output_path;    // -o,--output; empty = default name (see default_calib_cam_lidar_output_path)
  int samples = 8;  // --samples; image samples to use (min 3)
  std::string fix_axes;      // --fix; raw csv of axes to hold at the bag value
  double max_trans = 0.2;    // --max-trans; trust region, meters
  double max_rot_deg = 2.0;  // --max-rot; trust region, degrees
  int nid_bins = 16;         // --nid-bins; NID histogram bins
  double min_depth = 2.0;    // --min-depth; nearest projected point depth, meters
  double max_depth = 150.0;  // --max-depth; farthest projected point depth, meters
  // --keyframe-dist / --keyframe-rot: pose-gated keyframe sampling. When
  // either is > 0, eligible images are partitioned into gate intervals (a new
  // interval opens once the interpolated pose moved >= keyframe_dist meters
  // or rotated >= keyframe_rot_deg degrees from the interval's first frame),
  // sample intervals are picked evenly, and each picked interval contributes
  // its SHARPEST member (gray_sharpness) instead of an arbitrary one. Both 0
  // (the default) keeps the plain even-time-spacing behavior.
  double keyframe_dist = 0.0;
  double keyframe_rot_deg = 0.0;
  bool json = false;       // --json; emit the stdout summary as JSON
  bool overwrite = false;  // -w,--overwrite; replace an existing -o/--output path
};

// Validate the cross-field/range constraints the per-option CLI checks
// cannot express. Returns the first violation found as a human-readable
// message, or an empty string when the combination is valid. Pure over
// CalibCamLidarArgs (no bag access), so run_calib_cam_lidar calls it before
// any bag work.
[[nodiscard]] std::string validate_calibrate_flags(const CalibCamLidarArgs & args);

// Parse --fix's comma-separated axis list (x, y, z, roll, pitch, yaw) into a
// per-axis "hold at the bag value" flag array in that fixed order. An empty
// csv fixes nothing (all false). An unknown token, or a csv that fixes all
// six axes (nothing left to optimize), is reported as an error message in the
// second member; the first member is only meaningful when that message is
// empty.
[[nodiscard]] std::pair<std::array<bool, 6>, std::string> parse_fixed_axes(const std::string & csv);

// Pick up to `samples` image-stamp indices into `image_stamps_ns` (sorted
// ascending), evenly spread inside the trajectory span shrunk by `margin_ns`
// on each side ([traj_begin_ns + margin_ns, traj_end_ns - margin_ns]) so every
// pick has bracketing poses on both sides for interpolate_trajectory. When at
// most `samples` stamps fall inside that window, every one of them is
// returned (fewer than requested). Returned indices are strictly increasing.
[[nodiscard]] std::vector<std::size_t> pick_sample_indices(
  std::span<const std::int64_t> image_stamps_ns, std::int64_t traj_begin_ns,
  std::int64_t traj_end_ns, int samples, std::int64_t margin_ns);

// The eligibility half of pick_sample_indices on its own: every index whose
// stamp falls inside [traj_begin_ns + margin_ns, traj_end_ns - margin_ns], in
// order. The keyframe-gated sampling path needs the full eligible list (to
// interpolate a pose per frame) before deciding which frames to keep.
[[nodiscard]] std::vector<std::size_t> eligible_sample_indices(
  std::span<const std::int64_t> image_stamps_ns, std::int64_t traj_begin_ns,
  std::int64_t traj_end_ns, std::int64_t margin_ns);

// Partition time-ordered poses into keyframe gate intervals [begin, end).
// A new interval opens at the first pose that moved >= min_dist_m meters or
// rotated >= min_rot_rad radians from the current interval's ANCHOR (its
// first pose) — the same gate `map slam --color-min-dist` applies before
// colorizing. A threshold <= 0 disables that half of the gate; with both
// disabled (or a stationary platform) every pose lands in one interval.
// Empty input yields no intervals.
[[nodiscard]] std::vector<std::pair<std::size_t, std::size_t>> pose_gate_intervals(
  std::span<const core::calib::Mat4> poses, double min_dist_m, double min_rot_rad);

// Whole-image sharpness: mean |gx| + |gy| central-difference gradient over
// the interior pixels — the GrayImage counterpart of the colorizer's
// image_sharpness_score convention (bagwiz_slam's colorize_keyframe). Higher
// = sharper; a uniform image, or one without interior pixels, scores 0.
[[nodiscard]] double gray_sharpness(const core::calib::GrayImage & image);

// Build a rigid transform from a translation and a quaternion (x, y, z, w;
// ROS / tf2 Hamilton convention). The quaternion is normalized internally, so
// a caller need not pre-normalize it (e.g. a raw geometry_msgs Quaternion
// straight off the wire). Shared by interpolate_trajectory (TUM trajectory
// poses) and the run path (tf2::BufferCore::lookupTransform results,
// TransformStamped records from the bag's static TF) so both go through one
// quaternion-to-rotation-matrix implementation.
//
// nullopt when the input cannot produce a usable transform: a quaternion whose
// norm is zero (an all-zero geometry_msgs Quaternion is the common case) or
// non-finite, or a non-finite translation component. Callers treat that as a
// skipped sample (interpolate_trajectory) or a hard error (the run path)
// rather than propagating NaNs into the chain.
[[nodiscard]] std::optional<core::calib::Mat4> mat4_from_quat(
  double tx, double ty, double tz, double qx, double qy, double qz, double qw);

// Interpolate the trajectory frame's pose at `stamp_ns`: translation lerp,
// orientation shortest-path SLERP (nlerp near-parallel) between the two poses
// bracketing the stamp, returned as a rigid transform. `poses` must be sorted
// ascending by timestamp_ns. nullopt when `poses` has fewer than two entries,
// `stamp_ns` falls outside [poses.front().timestamp_ns,
// poses.back().timestamp_ns] (no extrapolation), or the interpolated pose is
// not a usable rigid transform (see mat4_from_quat).
[[nodiscard]] std::optional<core::calib::Mat4> interpolate_trajectory(
  std::span<const core::TrajectoryPose> poses, std::int64_t stamp_ns);

// Default `-o/--output` path when omitted: "<input stem>_calib_cam_lidar.yaml"
// in the current working directory.
[[nodiscard]] std::string default_calib_cam_lidar_output_path(const std::filesystem::path & input);

// Render the human-readable stdout summary of a refine result: a per-axis
// table (bag value / refined value / delta / observability, rotations shown
// in degrees), the NID before/after, the sample count used, one warning line
// per degenerate axis (the delta there is unconstrained, so the warning points
// at `--fix <axis>`), and the `tf static update` apply hint. `edge_before` is
// the edited edge's x,y,z,roll,pitch,yaw (meters/radians) as recorded in the
// bag, in the same axis order as RefineResult::delta; the "refined value"
// column is core::calib::apply_edge_delta of the two, the same composition the
// emitted YAML uses.
[[nodiscard]] std::string render_calibrate_summary(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const std::string & yaml_path);

// Render the machine-readable `--json` summary of a refine result, mirroring
// `tf static calc --json`'s nesting/key-naming style (hand-built with fmt
// instead of a JSON library — this command's only JSON output, so pulling one
// in isn't worth it). Rotation axes are left in radians (the edge's native
// unit, same as `edge_before` / RefineResult::delta) rather than converted to
// degrees, unlike render_calibrate_summary's human table.
//
// Deviates from the brief's `render_calibrate_json(result, edge_before)`
// signature: producing the required "parent"/"child" fields needs the edge's
// frame names, which only `args` carries, so `args` is threaded through here
// too (matching render_calibrate_summary's first parameter). Flagged for
// Task 8/9 — see the Task 7 report.
[[nodiscard]] std::string render_calibrate_json(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before);

// Entry point for `bagwiz calib cam-lidar`'s run path: loads the map PCD and
// TUM trajectory, resolves the image's CameraInfo and the static-TF chain from
// --traj-frame to the camera's optical frame, samples images spread across the
// trajectory span, refines the --parent/--child edge via
// core::calib::refine_extrinsic, and writes the result as a static-TF-tree
// YAML that `bagwiz tf static update` applies. Returns 0 on success, 1 on any
// error (bad flag combination, unreadable map/trajectory, missing/wrong-typed
// image or CameraInfo topic, an unresolvable or off-chain edge, too few usable
// image samples, or a refinement failure), with messages through the same
// logging pattern run_tf_static_dump uses. Defined in calib_cam_lidar.cpp.
int run_calib_cam_lidar(const CalibCamLidarArgs & args);

}  // namespace bagwiz::commands

#endif  // COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_
