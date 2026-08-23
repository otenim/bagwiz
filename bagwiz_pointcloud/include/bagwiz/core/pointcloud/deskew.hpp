// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_
#define BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::pointcloud
{

// Outcome of deskew_pointcloud2(). On success `cloud` holds the deskewed
// points and `error` is empty; on failure `cloud` is reset and `error` carries
// the reason. The counters classify every point of the input (they sum to
// `points_total` except when the whole cloud is rejected up front):
//   - points_deskewed:  moved to the reference pose.
//   - points_no_time:   no usable per-point time (cloud-wide, or a single
//                        point whose time value is non-finite).
//   - points_no_pose:   no trajectory pose could be resolved for the point's
//                        (or the reference's) timestamp.
//   - points_nonfinite: xyz itself is NaN/Inf; passed through unchanged.
// Two further members report endpoint clamping, which is not an error but
// means the affected points were deskewed against a pose at a different time
// than their own (up to no correction at all):
//   - points_out_of_span: a subset of points_deskewed whose timestamp fell
//     outside the trajectory's time span and was clamped to the nearest
//     endpoint pose.
//   - ref_out_of_span: `t_ref_ns` itself fell outside the trajectory's span
//     (the reference pose was likewise clamped).
struct DeskewResult
{
  std::optional<PointCloud2> cloud;
  std::string error;
  std::uint64_t points_total = 0;
  std::uint64_t points_deskewed = 0;
  std::uint64_t points_no_time = 0;
  std::uint64_t points_no_pose = 0;
  std::uint64_t points_nonfinite = 0;
  std::uint64_t points_out_of_span = 0;
  bool ref_out_of_span = false;

  [[nodiscard]] bool ok() const noexcept { return cloud.has_value() && error.empty(); }
};

// Deskew `input`, moving each point to the reference timestamp `t_ref_ns` using
// the world trajectory (T_world_sensor over time). `extrinsic` E maps a point
// from the cloud frame into the trajectory (`--of`) frame; nullopt = identity.
//   p' = E^{-1} * (T(t_ref)^{-1} * T(t_i)) * E * p
// `input` is taken by value and rewritten in place (callers can std::move the
// parsed cloud in to avoid a full data copy). Non-target fields/bytes are
// preserved; only xyz + one per-point time field are
// rewritten (time -> t_ref-equivalent to block downstream double-deskew, unless
// `keep_point_time` is true). Time field detected per point_time.hpp. Big-endian
// input is rejected. A cloud with no usable time field is returned verbatim with
// points_no_time set (the command treats that as fatal upfront).
DeskewResult deskew_pointcloud2(
  PointCloud2 input, std::int64_t t_ref_ns, std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic = std::nullopt,
  bool keep_point_time = false);

// Outcome of deskew_pointcloud2_cdr(): the counters and clamp flags of
// DeskewResult (same meanings), without a materialised cloud -- the points
// were patched directly inside the caller's payload buffer. `parse_error`
// and `error` are distinct so callers can keep parse_pointcloud2's
// pass-through-with-warning semantics for undecodable messages while
// treating a rejected cloud layout as a hard failure.
struct DeskewCdrResult
{
  std::string parse_error;    // payload is not a decodable PointCloud2; buffer untouched
  std::string error;          // cloud layout rejected by validation; buffer untouched
  std::int64_t t_ref_ns = 0;  // the header stamp the points were deskewed to
  std::uint64_t points_total = 0;
  std::uint64_t points_deskewed = 0;
  std::uint64_t points_no_time = 0;
  std::uint64_t points_no_pose = 0;
  std::uint64_t points_nonfinite = 0;
  std::uint64_t points_out_of_span = 0;
  bool ref_out_of_span = false;

  [[nodiscard]] bool ok() const noexcept { return parse_error.empty() && error.empty(); }
};

// Deskew a serialized PointCloud2 in place: parse the CDR header, then patch
// only the x/y/z and per-point time bytes of the point-data section inside
// `payload`. Every other byte -- header, field table, alignment padding, any
// trailing bytes -- is left untouched, so this produces the same message as
// parse_pointcloud2 + deskew_pointcloud2 + serialize_pointcloud2 without the
// parse-side data copy or the full re-serialize. The reference timestamp is
// the message's own header.stamp (reported back via t_ref_ns); semantics
// otherwise match deskew_pointcloud2, including the pass-through cases (no
// usable time field, no resolvable reference pose), which leave the payload
// bytes verbatim with the corresponding counter set.
DeskewCdrResult deskew_pointcloud2_cdr(
  std::span<std::byte> payload, std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic = std::nullopt,
  bool keep_point_time = false);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_
