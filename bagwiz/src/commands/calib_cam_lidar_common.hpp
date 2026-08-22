// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_
#define COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_

#include "bagwiz/core/base/worker_pool.hpp"
#include "bagwiz/core/calib/extrinsic_refine.hpp"
#include "bagwiz/core/calib/nid_cost.hpp"
#include "bagwiz/core/calib/se3.hpp"
#include "bagwiz/core/pointcloud/point_cloud_io.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Internals of `bagwiz calib cam-lidar`, split out so the flag validation,
// sample picking, trajectory interpolation, map accumulation, and report
// rendering can be unit-tested without a bag or a real refinement run. Pure
// over the args, no bag access. CLI-internal: this header lives with the
// command sources and is not installed.
namespace bagwiz::commands
{

// Parsed arguments for `bagwiz calib cam-lidar`. Refines one static-TF edge on
// a camera's chain by registering the bag's LiDAR clouds (accumulated into a
// map through the bag's own pose topic) against the bag's images via NID, and
// writes a YAML that `bagwiz tf static update` applies.
struct CalibCamLidarArgs
{
  // -i,--input: bag path (file or directory). A std::filesystem::path (not a
  // string) because set_topic_input() binds the completion registry's topic
  // slots to it — the same shape every other slot-declaring command uses.
  std::filesystem::path input_path;
  std::string pcd_topic;   // --pcd: PointCloud2 topic accumulated into the map
  std::string pose_topic;  // --pose: self-position topic (same types as `pcd undistort --pose`)
  std::string cam_topic;   // --cam: image topic to calibrate against
  // --of / --ref: the trajectory expresses the pose of --of in the --ref frame
  // (the same pair and defaults as `pcd undistort`). --of anchors the static
  // TF chain to the camera's optical frame and the per-cloud extrinsic.
  std::string of_frame = "base_link";
  std::string ref_frame = "map";
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
  // --fix; raw csv of axes to hold at the bag value, plus `auto` (default:
  // also hold every direction the data cannot constrain) and `none`.
  std::string fix_axes = "auto";
  double max_trans = 0.2;    // --max-trans; trust region, meters
  double max_rot_deg = 2.0;  // --max-rot; trust region, degrees
  int nid_bins = 16;         // --nid-bins; NID histogram bins
  double min_depth = 2.0;    // --min-depth; nearest projected point depth, meters
  double max_depth = 150.0;  // --max-depth; farthest projected point depth, meters
  // --voxel; edge length of the grid the accumulated map is collapsed onto,
  // meters. 0 keeps every point of every cloud (see MapAccumulator).
  double voxel_size = 0.1;
  // --keyframe-dist / --keyframe-rot: pose-gated keyframe sampling. When
  // either is > 0, eligible images are partitioned into gate intervals (a new
  // interval opens once the interpolated pose moved >= keyframe_dist meters
  // or rotated >= keyframe_rot_deg degrees from the interval's first frame),
  // sample intervals are picked evenly, and each picked interval contributes
  // its SHARPEST member (gray_sharpness) instead of an arbitrary one. Both 0
  // (the default) keeps the plain even-time-spacing behavior.
  double keyframe_dist = 0.0;
  double keyframe_rot_deg = 0.0;
  // --skip-start / --skip-end: raw duration strings (the same grammar as
  // `trim --start/--end` — a unit suffix is mandatory, e.g. "30s", "1.5s").
  // Empty = disabled. Each excludes that duration, measured from the bag's
  // time extent (its start / its end), from the estimation; see
  // parse_skip_durations.
  std::string skip_start;
  std::string skip_end;
  // --cam-offset: raw signed duration string (the same grammar as --skip-start:
  // a unit suffix is mandatory, e.g. "-42ms", "+1.5s"), or "auto". Empty =
  // none. Added to every image's stamp before anything reads it, so the image
  // stamped t is placed at pose(t + offset): a camera clock that runs late
  // relative to the --pose clock is corrected with a negative value. "auto"
  // measures the offset from the bag's own images against the --pose
  // trajectory (or, with --imu, bridged through the gyro) and applies the
  // estimate. See parse_cam_offset and estimate_cam_offset.
  std::string cam_offset;
  // --imu: Imu topic used as the timing bridge for `--cam-offset auto`; empty
  // = none. Only meaningful with auto (validate_calibrate_flags rejects it
  // otherwise). The IMU frame must be reachable from --of through the bag's
  // static TF; the gyro is rotated into --of through that chain.
  std::string imu_topic;
  bool json = false;       // --json; emit the stdout summary as JSON
  bool overwrite = false;  // -w,--overwrite; replace an existing -o/--output path
  // -j,--threads: total parallelism of the map accumulation, the refinement
  // and the sample decoding (the same knob `pcd undistort` has). 0 = the
  // hardware concurrency, 1 = everything on the calling thread; resolved by
  // resolve_num_threads. The value never changes the result: the map is
  // filled in the same order and the NID histograms count the same integers
  // whatever the split, so the YAML and the report are identical for every
  // thread count.
  int threads = 0;
};

// Validate the cross-field/range constraints the per-option CLI checks
// cannot express. Returns the first violation found as a human-readable
// message, or an empty string when the combination is valid. Pure over
// CalibCamLidarArgs (no bag access), so run_calib_cam_lidar calls it before
// any bag work.
[[nodiscard]] std::string validate_calibrate_flags(const CalibCamLidarArgs & args);

// The parsed --fix value: the axes held at the bag value no matter what,
// plus whether directions the data cannot constrain are held automatically
// (`auto`, the default). See parse_fix_spec.
struct FixSpec
{
  std::array<bool, 6> fixed{};
  bool auto_fix = true;
};

// Parse --fix's comma-separated list. Tokens: the six axis names (x, y, z,
// roll, pitch, yaw) plus `auto` and `none`. An empty csv means `auto` (the
// CLI default). A manual axis list alone switches auto off; `auto` composes
// with manual axes; `none` must stand alone (nothing held — the pre-auto
// behavior). Errors (returned in the second member; the first member is only
// meaningful when it is empty): an unknown token, `none` combined with other
// tokens, or a csv fixing all six axes (nothing left to optimize).
[[nodiscard]] std::pair<FixSpec, std::string> parse_fix_spec(const std::string & csv);

// The parsed --skip-start / --skip-end values, in nanoseconds, as
// {skip_start_ns, skip_end_ns} (0 for a flag that was omitted). Each is a
// duration measured from the bag's time extent: the run path excludes
// [bag_start, bag_start + skip_start_ns) and (bag_end - skip_end_ns, bag_end]
// from the estimation by trimming the --pose trajectory to what lies between
// them, which every downstream consumer (image-sample eligibility, the cloud
// span check, deskew clamping) already keys off. Errors (returned in the
// second member; the first member is only meaningful when it is empty): a
// value that fails the `trim --start/--end` duration grammar (a unit suffix
// is mandatory) or a negative duration.
[[nodiscard]] std::pair<std::array<std::int64_t, 2>, std::string> parse_skip_durations(
  const CalibCamLidarArgs & args);

// The parsed --cam-offset: either a fixed value in nanoseconds (0 for an
// omitted flag) or the request to estimate it (`auto`). The run path adds the
// applied value to every image stamp the moment the stamps are read, so
// sample eligibility and picking, the keyframe gate, the map's frustum cull,
// the pre-cull and each sample's trajectory pose all see the same shifted
// time: the image stamped t is placed at pose(t + offset). The sign is
// literal — a camera clock that stamps late relative to the --pose clock is
// corrected with a negative offset.
struct CamOffsetSpec
{
  bool auto_estimate = false;  // `auto`: measure it from the bag (estimate_cam_offset)
  std::int64_t offset_ns = 0;  // the fixed value; 0 under auto or when omitted
};

// Errors (returned in the second member; the first member is only meaningful
// when it is empty): a value that is neither `auto` nor a duration in the
// --skip-start grammar (a unit suffix is mandatory), or a magnitude beyond
// 24 h — a sensor clock offset is milliseconds to seconds, and an unbounded
// value added to an epoch stamp could overflow.
[[nodiscard]] std::pair<CamOffsetSpec, std::string> parse_cam_offset(
  const CalibCamLidarArgs & args);

// What `--cam-offset auto` measured, for the report. Offsets follow the
// --cam-offset convention (the value added to image stamps).
struct CamOffsetEstimateReport
{
  std::int64_t offset_ns = 0;
  std::int64_t std_ns = 0;       // bootstrap spread
  std::string method;            // "trajectory" (images vs --pose) or "imu" (gyro bridge)
  std::string visual_estimator;  // "essential" or "rotation": the frame-pair solver kept
  std::size_t pairs = 0;         // frame pairs the fit rests on
  double signal_rms_mrad = 0.0;
  double residual_rms_before_mrad = 0.0;
  double residual_rms_after_mrad = 0.0;
  // imu method only: the two legs of the bridge (camera vs gyro, pose vs gyro).
  std::optional<std::int64_t> camera_imu_offset_ns;
  std::optional<std::int64_t> pose_imu_offset_ns;
};

// The offset a run applied, for the report: the manual value, or the estimate
// under auto (then `estimate` is set). Default: nothing applied.
struct CamOffsetReport
{
  std::int64_t applied_ns = 0;
  std::optional<CamOffsetEstimateReport> estimate;
};

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

// Running tallies of the --pcd accumulation pass (one call per cloud on the
// topic), surfaced as the run path's end-of-pass info/warning lines.
struct MapAccumulationStats
{
  std::uint64_t clouds_read = 0;                 // clouds offered to the pass
  std::uint64_t clouds_skipped_out_of_span = 0;  // header.stamp outside the trajectory span
  std::uint64_t clouds_deskewed = 0;             // clouds deskewed before accumulation
  std::uint64_t points_added = 0;
  std::uint64_t points_dropped_nonfinite = 0;
  // Points dropped because no image sample's view can ever contain them (only
  // counted when the caller passes a view set to cull against).
  std::uint64_t points_culled_out_of_view = 0;
  // Points the deskew clamped to a trajectory endpoint (their own stamp fell
  // outside the trajectory span) — deskewed against a pose at a different
  // time than their own, so never silent.
  std::uint64_t points_clamped_out_of_span = 0;
};

// One sample's camera view in the form the map accumulation culls against:
// the camera optical frame's pose in the --ref frame, the depth window, and
// the normalized-image (x/z, y/z) bounds of the pixel rectangle, padded like
// the per-sample pre-cull and widened so distortion cannot push a kept point
// outside them. A point outside every sample's view can never be projected
// by any NID evaluation, so dropping it before voxelization is safe — but
// note this is a superset filter: the exact per-sample predicate (with the
// real distortion model) still runs later, at candidate assembly.
struct SampleViewFrustum
{
  core::calib::Mat4 t_cam_ref{};  // --ref frame -> camera optical frame
  double lo_xn = 0.0;             // normalized x/z bounds of the padded image rect
  double hi_xn = 0.0;
  double lo_yn = 0.0;
  double hi_yn = 0.0;
  double lo_depth = 0.0;  // the --min-depth/--max-depth window, in meters
  double hi_depth = 0.0;
};

// True when the --ref-frame point `p` falls inside any of the given sample
// views. With an empty view set, everything is inside (the cull is off).
[[nodiscard]] bool point_in_any_view(
  const std::array<double, 3> & p, std::span<const SampleViewFrustum> views);

// The axis-aligned box, in the --ref frame, around one sample view's frustum:
// the eight corners of the padded normalized rectangle at lo_depth and
// hi_depth carried through the inverse of t_cam_ref, widened by
// kViewRejectBoxPadMeters on every side. Every point the exact predicate
// accepts lies inside it — the frustum is convex for lo_depth > 0 and the pad
// covers the roundoff of both transforms — so a point outside the box can be
// rejected on six compares before any transform is paid for. A view with
// lo_depth <= 0 is not a convex frustum (its normalized bounds flip sign
// behind the camera) and gets an unbounded box, i.e. no pre-rejection.
struct ViewRejectBox
{
  std::array<double, 3> lo{};
  std::array<double, 3> hi{};
};
inline constexpr double kViewRejectBoxPadMeters = 1e-3;
[[nodiscard]] std::vector<ViewRejectBox> view_reject_boxes(
  std::span<const SampleViewFrustum> views);

// point_in_any_view with each view's reject box (from view_reject_boxes, so
// `boxes` is parallel to `views`) tried first, and the camera-frame depth
// evaluated before the other two coordinates: the same truth value, at a
// fraction of the cost for the points every view rejects — which, for a
// narrow camera against a 360-degree lidar, is most of them.
[[nodiscard]] bool point_in_any_view(
  const std::array<double, 3> & p, std::span<const SampleViewFrustum> views,
  std::span<const ViewRejectBox> boxes);

// The calibration map under construction: points are collapsed onto a voxel
// grid as they arrive, one running centroid and mean intensity per occupied
// voxel, so the map costs memory in proportion to the SURFACE it covers
// rather than to the number of points recorded. That matters because a
// driving platform re-measures the same surface once per sweep — hundreds of
// times over a bag — and NID reads the map as a statistical sample of
// (intensity, gray) pairs, which those duplicates do not enrich. A voxel size
// of 0 turns the grid off and keeps every point verbatim, in arrival order.
//
// The grid can be split into `partitions` independent hash maps, each owning
// the voxels whose key hashes to it, so an insertion pass can feed every
// partition from its own thread (add_to_partition). A voxel's running sums
// still add the same points in the same order as they would in one map, and
// finish() orders the union by voxel index, so the map does not depend on
// the partition count.
class MapAccumulator
{
public:
  explicit MapAccumulator(double voxel_size, int partitions = 1);

  // Adds one point already expressed in the --ref frame. False when the
  // coordinates cannot be quantized — a magnitude no voxel index can hold is
  // as unusable as a NaN, and the caller counts it the same way — which
  // cannot happen with the grid off.
  bool add(const std::array<float, 3> & point, float intensity);

  // Whether points are collapsed onto the grid at all (voxel size > 0).
  [[nodiscard]] bool gridded() const { return voxel_size_ > 0.0; }

  // The independently fed slices of the grid (1 with the grid off).
  [[nodiscard]] int partitions() const { return static_cast<int>(voxels_.size()); }

  // The grid cell a --ref point falls in, for the split insertion below.
  struct VoxelKey
  {
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;
    bool operator==(const VoxelKey & other) const = default;
  };

  // The voxel of a --ref point, or nullopt when the point cannot be quantized
  // (what add would refuse). Only meaningful with the grid on.
  [[nodiscard]] std::optional<VoxelKey> voxel_of(const std::array<float, 3> & point) const;

  // The partition `key`'s voxel lives in.
  [[nodiscard]] int partition_of(const VoxelKey & key) const;

  // add() for a point whose voxel (from voxel_of) and partition (from
  // partition_of) the caller already holds: safe to call concurrently for
  // different partitions, because each partition's voxels live in their own
  // map, and never refused, because the point already quantized. The caller
  // keeps the per-voxel order it wants. Grid on only.
  void add_to_voxel(
    int partition, const VoxelKey & key, const std::array<float, 3> & point, float intensity);

  // Occupied voxels so far (points so far, with the grid off).
  [[nodiscard]] std::size_t size() const;
  [[nodiscard]] bool empty() const { return size() == 0; }

  // Materializes the map and empties the accumulator (one-shot): one point per
  // occupied voxel, at its centroid and carrying its mean intensity, ordered
  // by voxel index so the same points give the same map whatever order the
  // bag delivered them in, and whatever the partition count. With the grid
  // off, the points as they were added. Given a pool, a large map is drained,
  // split into voxel-index ranges and sorted on it — the same order, since
  // the ranges are disjoint and emitted in index order.
  [[nodiscard]] core::pointcloud::PcdCloud finish(core::WorkerPool * pool = nullptr);

private:
  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const;
  };
  // Running sums, in double because a voxel far from the origin accumulates
  // hundreds of large coordinates and a float sum would lose centimeters.
  struct VoxelAccum
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double intensity = 0.0;
    std::uint32_t count = 0;
  };

  double voxel_size_;
  core::pointcloud::PcdCloud raw_;  // grid off
  // One map per partition (exactly one with the grid off).
  std::vector<std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash>> voxels_;
};

// Append one --pcd cloud to the accumulated map, expressed in the
// trajectory's --ref frame: each point is placed by
// T_ref_of(header.stamp) * T_of_cloud, where `trajectory` carries T_ref_of
// (sorted ascending) and `t_of_cloud` is the cloud frame's static extrinsic
// into the --of frame (nullopt = the cloud frame already is --of). A cloud
// whose per-point time field is usable AND non-uniform is deskewed to its
// header stamp first (deskew_pointcloud2 with the same trajectory and
// extrinsic); a uniform field — all zeros, or the t_ref-constant field `pcd
// undistort` leaves behind — means no sweep motion and the cloud is
// accumulated as-is. A cloud whose header.stamp falls outside the trajectory
// span is skipped (counted, not an error: clamping it to an endpoint pose
// would smear the map), and non-finite points are dropped (counted).
//
// Returns an error message only for an unusable cloud: no intensity field
// (NID needs it), an unreadable field layout, big-endian point data, an
// unusable static extrinsic, or a deskew failure. The caller reports it
// against the topic and aborts.
//
// `views` is the frustum union of the image samples the map is built for: a
// placed point outside every view is dropped (counted in
// MapAccumulationStats::points_culled_out_of_view), since no NID evaluation
// could ever project it. The default empty span disables the cull. This
// overload runs everything on the calling thread; the run path uses the
// MapAccumulationContext overload below.
[[nodiscard]] std::optional<std::string> accumulate_cloud_into_map(
  MapAccumulator & map, core::pointcloud::PointCloud2 cloud,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & t_of_cloud, MapAccumulationStats & stats,
  std::span<const SampleViewFrustum> views = {});

// Per-run state of the --pcd accumulation pass, shared by every cloud: the
// sample views with their reject boxes, the worker pool the per-point work is
// spread over (nullptr = everything on the calling thread), and the per-chunk
// survivor buffers the pass reuses from cloud to cloud. `views` must outlive
// the context.
struct MapAccumulationContext
{
  explicit MapAccumulationContext(
    std::span<const SampleViewFrustum> views = {}, core::WorkerPool * pool = nullptr);

  std::span<const SampleViewFrustum> views;
  std::vector<ViewRejectBox> boxes;  // parallel to views
  core::WorkerPool * pool = nullptr;

  // One chunk's share of a cloud's placement pass: the placed points that
  // survived the finiteness check, the cull and quantization, bucketed by the
  // map partition their voxel belongs to and kept in point order inside each
  // bucket, plus the drop counts. Kept across clouds so the buffers keep
  // their capacity.
  struct Bucket
  {
    std::vector<MapAccumulator::VoxelKey> keys;  // grid on: parallel to points
    std::vector<std::array<float, 3>> points;
    std::vector<float> intensities;  // parallel to points
  };
  struct Chunk
  {
    std::vector<Bucket> buckets;  // one per map partition
    std::uint64_t nonfinite = 0;  // non-finite or unquantizable coordinates
    std::uint64_t culled = 0;
    std::uint64_t placed = 0;  // points that reached a bucket
  };
  std::vector<Chunk> chunks;
};

// accumulate_cloud_into_map over a shared context: the per-point work (field
// reads, the finiteness check, the placement transform, the cull, the voxel
// lookup) is split into one chunk per worker and run on the context's pool,
// each chunk bucketing its survivors by map partition in point order; the
// partitions are then each fed on their own worker, chunk by chunk, so every
// voxel's running sums add its points in the order a single loop over the
// points would have — the map is the same whatever the pool size and the
// partition count.
[[nodiscard]] std::optional<std::string> accumulate_cloud_into_map(
  MapAccumulator & map, core::pointcloud::PointCloud2 cloud,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & t_of_cloud, MapAccumulationStats & stats,
  MapAccumulationContext & context);

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
  const std::array<double, 6> & edge_before, const std::string & yaml_path,
  const CamOffsetReport & cam_offset = {});

// Render the machine-readable `--json` summary of a refine result, mirroring
// `tf static calc --json`'s nesting/key-naming style (hand-built with fmt
// instead of a JSON library — this command's only JSON output, so pulling one
// in isn't worth it). Rotation axes are left in radians (the edge's native
// unit, same as `edge_before` / RefineResult::delta) rather than converted to
// degrees, unlike render_calibrate_summary's human table. The signature takes
// `args` (not just the result) because the "parent"/"child" fields name the
// edited edge's frames, which only `args` carries — the same threading
// render_calibrate_summary uses.
[[nodiscard]] std::string render_calibrate_json(
  const CalibCamLidarArgs & args, const core::calib::RefineResult & result,
  const std::array<double, 6> & edge_before, const CamOffsetReport & cam_offset = {});

// Entry point for `bagwiz calib cam-lidar`'s run path: builds the --of -> --ref
// trajectory from the --pose topic, accumulates the --pcd topic's clouds into
// a map in the --ref frame, resolves the image's CameraInfo and the static-TF
// chain from --of to the camera's optical frame, samples images spread across
// the trajectory span, refines the --parent/--child edge via
// core::calib::refine_extrinsic, and writes the result as a static-TF-tree
// YAML that `bagwiz tf static update` applies. Returns 0 on success, 1 on any
// error (bad flag combination, missing/wrong-typed --pcd/--pose/--cam or
// CameraInfo topic, a --pcd topic without an intensity field, an unresolvable
// or off-chain edge, --skip-start/--skip-end covering the whole bag or leaving
// fewer than two trajectory poses, too few usable image samples, or a
// refinement failure),
// with messages through the same logging pattern run_tf_static_dump uses.
// Defined in calib_cam_lidar.cpp.
int run_calib_cam_lidar(const CalibCamLidarArgs & args);

}  // namespace bagwiz::commands

#endif  // COMMANDS__CALIB_CAM_LIDAR_COMMON_HPP_
