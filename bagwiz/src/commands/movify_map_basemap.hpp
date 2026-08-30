// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_MAP_BASEMAP_HPP_
#define COMMANDS__MOVIFY_MAP_BASEMAP_HPP_

#include "bagwiz/core/slam/gnss_projector.hpp"
#include "movify_map_tiles.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_track.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_viewport.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>

#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <string>

// The map under the map panel's track: the Web Mercator tiles covering a
// viewport, warped into the panel's ENU plane. The ENU plane is the track's
// (anchored at its origin fix), so the track drawn over the warped tiles
// lands on the roads it was driven on. CLI-internal: this header lives with
// the command sources and is not installed.
namespace bagwiz::commands
{

// What a missing tile shows: a dark neutral the track stays legible on.
inline const cv::Scalar kMapNoDataColor(40, 40, 40);

// The tiles a viewport draws from: a zoom level and an inclusive tile-index
// box (empty when x1 < x0 or y1 < y0). x may run past the world's edge — it
// wraps at the antimeridian when looked up; y is clipped to the world.
struct MapTileRange
{
  int zoom = 0;
  int x0 = 0;
  int x1 = -1;
  int y0 = 0;
  int y1 = -1;

  [[nodiscard]] int width() const noexcept { return x1 - x0 + 1; }
  [[nodiscard]] int height() const noexcept { return y1 - y0 + 1; }
  [[nodiscard]] bool empty() const noexcept { return width() <= 0 || height() <= 0; }

  friend bool operator==(const MapTileRange &, const MapTileRange &) = default;
};

// The most tiles one viewport is drawn from: the zoom choice bounds a
// viewport to a few dozen tiles, so a range beyond this signals a broken
// viewport (a NaN scale) rather than a big panel, and draws nothing.
inline constexpr int kMapMaxMosaicTiles = 4096;

class MapBasemap
{
public:
  struct Options
  {
    // The ENU plane's anchor: the fix the track's coordinates are relative to.
    MapOrigin origin;
    std::shared_ptr<MapTileSource> source;
    int max_zoom = kMapTileMaxZoom;
  };

  explicit MapBasemap(Options options);

  // What was fetched by prefetch(): the tiles the views needed, and those
  // among them the source could not provide.
  struct PrefetchReport
  {
    std::size_t needed = 0;
    std::size_t missing = 0;
  };

  // The tiles `view` is drawn from: at the zoom map_tile_zoom() picks for
  // the view's latitude and scale, the box covering the cell's four corners.
  [[nodiscard]] MapTileRange tile_range_of(const MapViewport & view) const;

  // Fetch and keep every tile the views need that is not held yet, on
  // `threads` workers (each tile once). A tile the source cannot provide is
  // remembered as missing and never asked for again.
  PrefetchReport prefetch(std::span<const MapViewport> views, unsigned threads);

  // Draw the map under `view` into `canvas` (the cell, CV_8UC3): its tiles
  // warped from Web Mercator into the ENU plane, kMapNoDataColor where a
  // tile is missing. A tile prefetch() did not cover is fetched on the spot.
  void draw(cv::Mat & canvas, const MapViewport & view);

  // Why the source's latest failed tile failed ("" when none has).
  [[nodiscard]] std::string source_error() const;

private:
  // The Mercator pixel under cell pixel (x, y), its longitude unwrapped to
  // within 180 deg of `center_longitude` so a view across the antimeridian
  // still maps to one contiguous run of columns (wrapped at lookup).
  [[nodiscard]] MercatorPixel mercator_of(
    const MapViewport & view, double x, double y, int zoom, double center_longitude) const;
  [[nodiscard]] double center_longitude_of(const MapViewport & view) const;
  // The tile, fetched on first use; empty when the source cannot provide it.
  [[nodiscard]] cv::Mat tile_of(const MapTileKey & key);
  void build_mosaic(const MapTileRange & range);

  Options options_;
  // Latched at options_.origin. Not thread-safe: only the thread driving
  // the basemap (tile_range_of / draw) touches it; the prefetch workers
  // fetch tiles by key and never project.
  core::slam::GnssProjector projector_;
  std::mutex mutex_;  // guards tiles_
  std::map<MapTileKey, cv::Mat> tiles_;
  // The last-drawn range's tiles laid side by side, reused while the range
  // holds (a fitted track draws the same range every frame).
  MapTileRange mosaic_range_;
  cv::Mat mosaic_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_MAP_BASEMAP_HPP_
