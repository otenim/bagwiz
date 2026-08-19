// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TF_STATIC_CALIBRATE_COMMON_HPP_
#define COMMANDS__TF_STATIC_CALIBRATE_COMMON_HPP_

#include "bagwiz/core/calib/extrinsic_refine.hpp"
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

// Internals of `tf static calibrate`, split out so the flag validation,
// sample picking, trajectory interpolation, and report rendering can be
// unit-tested without a bag or a real refinement run. Pure over the args, no
// bag access. CLI-internal: this header lives with the command sources and is
// not installed.
namespace bagwiz::commands
{

// Parsed arguments for `bagwiz tf static calibrate`. Refines one static-TF
// edge on a camera's chain by registering the bag's LiDAR map (from a prior
// `bagwiz map slam` run) against the bag's images via NID, and writes a YAML
// that `bagwiz tf static update` applies.
struct TfStaticCalibrateArgs
{
  std::string input_path;      // -i,--input: bag path (file or directory)
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
  std::string output_path;  // -o,--output; empty = default name (see default_calibrate_output_path)
  int samples = 8;          // --samples; image samples to use (min 3)
  std::string fix_axes;     // --fix; raw csv of axes to hold at the bag value
  double max_trans = 0.2;   // --max-trans; trust region, meters
  double max_rot_deg = 2.0;  // --max-rot; trust region, degrees
  int nid_bins = 16;         // --nid-bins; NID histogram bins
  double min_depth = 2.0;    // --min-depth; nearest projected point depth, meters
  double max_depth = 150.0;  // --max-depth; farthest projected point depth, meters
  bool json = false;         // --json; emit the stdout summary as JSON
  bool overwrite = false;    // -w,--overwrite; replace an existing -o/--output path
};

// Validate the cross-field/range constraints the per-option CLI checks
// cannot express. Returns the first violation found as a human-readable
// message, or an empty string when the combination is valid. Pure over
// TfStaticCalibrateArgs (no bag access), so run_tf_static_calibrate calls it
// before any bag work.
[[nodiscard]] std::string validate_calibrate_flags(const TfStaticCalibrateArgs & args);

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

// Build a rigid transform from a translation and a quaternion (x, y, z, w;
// ROS / tf2 Hamilton convention). The quaternion is normalized internally, so
// a caller need not pre-normalize it (e.g. a raw geometry_msgs Quaternion
// straight off the wire). Shared by interpolate_trajectory (TUM trajectory
// poses) and the run path (tf2::BufferCore::lookupTransform results,
// TransformStamped records from the bag's static TF) so both go through one
// quaternion-to-rotation-matrix implementation.
[[nodiscard]] core::calib::Mat4 mat4_from_quat(
  double tx, double ty, double tz, double qx, double qy, double qz, double qw);

// Interpolate the trajectory frame's pose at `stamp_ns`: translation lerp,
// orientation shortest-path SLERP (nlerp near-parallel) between the two poses
// bracketing the stamp, returned as a rigid transform. `poses` must be sorted
// ascending by timestamp_ns. nullopt when `poses` has fewer than two entries
// or `stamp_ns` falls outside [poses.front().timestamp_ns,
// poses.back().timestamp_ns] (no extrapolation).
[[nodiscard]] std::optional<core::calib::Mat4> interpolate_trajectory(
  std::span<const core::TrajectoryPose> poses, std::int64_t stamp_ns);

// Default `-o/--output` path when omitted: "<input stem>_tf_static_calib.yaml"
// in the current working directory.
[[nodiscard]] std::string default_calibrate_output_path(const std::filesystem::path & input);

// Render the human-readable stdout summary of a refine result: a per-axis
// table (bag value / refined value / delta / observability, rotations shown
// in degrees), the NID before/after, the sample count used, one warning line
// per degenerate axis ("its value is the bag's, consider --fix <axis>"), and
// the `tf static update` apply hint. `edge_before` is the edited edge's
// x,y,z,roll,pitch,yaw (meters/radians) as recorded in the bag, in the same
// axis order as RefineResult::delta.
[[nodiscard]] std::string render_calibrate_summary(
  const TfStaticCalibrateArgs & args, const core::calib::RefineResult & result,
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
  const TfStaticCalibrateArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before);

// Entry point for `bagwiz tf static calibrate`'s run path: loads the map PCD
// and TUM trajectory, resolves the image's CameraInfo and the static-TF chain
// from --traj-frame to the camera's optical frame, samples images spread
// across the trajectory span, refines the --parent/--child edge via
// core::calib::refine_extrinsic, and writes the result as a static-TF-tree
// YAML that `bagwiz tf static update` applies. Returns 0 on success, 1 on any
// error (bad flag combination, unreadable map/trajectory, missing/wrong-typed
// image or CameraInfo topic, an unresolvable or off-chain edge, too few usable
// image samples, or a refinement failure), with messages through the same
// logging pattern run_tf_static_dump uses. Defined in tf_static_calibrate.cpp.
int run_tf_static_calibrate(const TfStaticCalibrateArgs & args);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TF_STATIC_CALIBRATE_COMMON_HPP_
