// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_MAP_TRACK_HPP_
#define COMMANDS__MOVIFY_MAP_TRACK_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

// The map panel's data: a NavSatFix topic read whole and projected into a
// local East-North-Up plane, so the panel can draw the vehicle's track and
// place the current fix on it. CLI-internal: this header lives with the
// command sources and is not installed.
namespace bagwiz::commands
{

// One fix with a position: where it sits in the bag (the record time the
// panel matches ticks against), its own stamp, its ENU position relative to
// the track's first fix, and the geographic coordinates it came from (for
// the panel's readout).
struct MapFix
{
  std::int64_t record_ns = 0;
  std::int64_t stamp_ns = 0;
  double east = 0.0;
  double north = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
};

// The WGS84 fix the ENU plane is anchored at: the first positioned fix in
// bag order, where east = north = 0.
struct MapOrigin
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
};

// The whole topic: the fixes in record-time order, the plane's origin, and
// the fixes' ENU bounding box.
struct MapTrack
{
  std::vector<MapFix> fixes;
  MapOrigin origin;
  double min_east = 0.0;
  double max_east = 0.0;
  double min_north = 0.0;
  double max_north = 0.0;
};

// Outcome of load_map_track(): the track, or the error that was already
// logged.
struct MapTrackResult
{
  std::optional<MapTrack> track;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return track.has_value() && error.empty(); }
};

// Read every message of the NavSatFix `topic`, drop the fixes without a
// position (NavSatStatus STATUS_NO_FIX, or a non-finite latitude/longitude),
// and project the rest into an ENU plane anchored at the first of them (the
// track's `origin`; a missing altitude anchors at 0 m).
// Errors (logged): the bag cannot be opened or read, a message fails to
// parse, the topic has no messages, or no message carries a position.
[[nodiscard]] MapTrackResult load_map_track(
  const std::filesystem::path & input, const std::string & topic);

// The index of the fix whose record time is nearest `record_ns` (the earlier
// one on a tie). Requires a non-empty track.
[[nodiscard]] std::size_t nearest_map_fix(const MapTrack & track, std::int64_t record_ns);

// The least distance the vehicle must have moved for a heading to be read
// off the track: below it, GNSS noise would spin the marker at rest.
inline constexpr double kMapHeadingMinMoveM = 0.5;

// The direction of travel into fix `index`, in radians counter-clockwise from
// east: from the latest earlier fix at least kMapHeadingMinMoveM away. nullopt
// when no earlier fix is that far (the first fix, or a vehicle at rest since
// the start).
[[nodiscard]] std::optional<double> map_heading_at(const MapTrack & track, std::size_t index);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_MAP_TRACK_HPP_
