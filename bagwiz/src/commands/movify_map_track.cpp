// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_track.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/slam/gnss_projector.hpp"
#include "bagwiz/core/slam/gnss_sample.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <memory>
#include <string>
#include <utility>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

MapTrackResult fail(std::string message)
{
  BAGWIZ_LOG_ERROR(kLogger, "%s", message.c_str());
  MapTrackResult out;
  out.error = std::move(message);
  return out;
}

// Whether the sample carries a position the panel can place.
bool has_position(const core::slam::GnssSample & sample)
{
  return sample.status != core::slam::kNavSatStatusNoFix && std::isfinite(sample.latitude) &&
         std::isfinite(sample.longitude);
}

MapTrack with_bounds(MapTrack track)
{
  track.min_east = track.fixes.front().east;
  track.max_east = track.min_east;
  track.min_north = track.fixes.front().north;
  track.max_north = track.min_north;
  for (const auto & fix : track.fixes) {
    track.min_east = std::min(track.min_east, fix.east);
    track.max_east = std::max(track.max_east, fix.east);
    track.min_north = std::min(track.min_north, fix.north);
    track.max_north = std::max(track.max_north, fix.north);
  }
  return track;
}
}  // namespace

MapTrackResult load_map_track(const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return fail("failed to open '" + input.string() + "': " + e.what());
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  MapTrack track;
  core::slam::GnssProjector projector;
  std::uint64_t messages = 0;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      ++messages;
      const auto parsed = core::slam::parse_navsatfix(raw.payload);
      if (!parsed.ok()) {
        return fail(
          "topic '" + topic + "': failed to parse the NavSatFix message at record time " +
          std::to_string(raw.timestamp_ns) + " ns: " + parsed.error);
      }
      const auto & sample = *parsed.sample;
      if (!has_position(sample)) {
        continue;
      }
      // The altitude only tilts the tangent plane by a hair; a missing one
      // must not poison the projection.
      const double altitude = std::isfinite(sample.altitude) ? sample.altitude : 0.0;
      const auto enu = projector.project(sample.latitude, sample.longitude, altitude);
      MapFix fix;
      fix.record_ns = raw.timestamp_ns;
      fix.stamp_ns = sample.stamp_ns;
      fix.east = enu[0];
      fix.north = enu[1];
      fix.latitude = sample.latitude;
      fix.longitude = sample.longitude;
      fix.altitude = sample.altitude;
      track.fixes.push_back(fix);
    }
  } catch (const std::exception & e) {
    return fail("error reading topic '" + topic + "': " + e.what());
  }
  if (messages == 0) {
    return fail("topic '" + topic + "' has no messages to render.");
  }
  if (track.fixes.empty()) {
    return fail("topic '" + topic + "' carries no fix with a position (every message is NO_FIX).");
  }
  // The lookups binary-search by record time; a bag that interleaves its
  // record order (a merge of several recordings) must not break them.
  std::stable_sort(track.fixes.begin(), track.fixes.end(), [](const MapFix & a, const MapFix & b) {
    return a.record_ns < b.record_ns;
  });

  const auto origin = projector.origin();
  track.origin.latitude = (*origin)[0];
  track.origin.longitude = (*origin)[1];
  track.origin.altitude = (*origin)[2];

  MapTrackResult out;
  out.track = with_bounds(std::move(track));
  return out;
}

std::size_t nearest_map_fix(const MapTrack & track, std::int64_t record_ns)
{
  const auto & fixes = track.fixes;
  const auto after = std::lower_bound(
    fixes.begin(), fixes.end(), record_ns,
    [](const MapFix & fix, std::int64_t ns) { return fix.record_ns < ns; });
  if (after == fixes.begin()) {
    return 0;
  }
  if (after == fixes.end()) {
    return fixes.size() - 1;
  }
  const auto before = std::prev(after);
  const auto gap_before = record_ns - before->record_ns;
  const auto gap_after = after->record_ns - record_ns;
  const auto nearest = gap_after < gap_before ? after : before;
  return static_cast<std::size_t>(nearest - fixes.begin());
}

std::optional<double> map_heading_at(const MapTrack & track, std::size_t index)
{
  const auto & fixes = track.fixes;
  if (index >= fixes.size()) {
    return std::nullopt;
  }
  const MapFix & to = fixes[index];
  for (std::size_t j = index; j > 0; --j) {
    const MapFix & from = fixes[j - 1];
    const double de = to.east - from.east;
    const double dn = to.north - from.north;
    if (std::hypot(de, dn) >= kMapHeadingMinMoveM) {
      return std::atan2(dn, de);
    }
  }
  return std::nullopt;
}

}  // namespace bagwiz::commands
