// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_basemap.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
// The tile column `x` names once wrapped into the `zoom` level's world.
int wrapped_column(int x, int zoom)
{
  const int columns = 1 << zoom;
  return ((x % columns) + columns) % columns;
}

// `longitude` shifted by whole turns to within 180 deg of `reference`.
double unwrapped_longitude(double longitude, double reference)
{
  double out = longitude;
  while (out - reference > 180.0) {
    out -= 360.0;
  }
  while (out - reference < -180.0) {
    out += 360.0;
  }
  return out;
}

// A Mercator pixel coordinate bounded so its tile index fits an int (the
// zoom cap keeps it far below this; the clamp is what makes the cast below
// safe by construction, whatever a caller sets max_zoom to).
constexpr double kMaxTilePixel =
  static_cast<double>(std::numeric_limits<int>::max() / 2) * static_cast<double>(kMapTileSizePx);

int tile_index_of(double pixel)
{
  return static_cast<int>(
    std::floor(std::clamp(pixel, -kMaxTilePixel, kMaxTilePixel) / kMapTileSizePx));
}
}  // namespace

MapBasemap::MapBasemap(Options options) : options_(std::move(options))
{
  (void)projector_.project(
    options_.origin.latitude, options_.origin.longitude, options_.origin.altitude);
}

MercatorPixel MapBasemap::mercator_of(
  const MapViewport & view, double x, double y, int zoom, double center_longitude) const
{
  const auto fix = projector_.reverse(view.east_of(x), view.north_of(y), 0.0);
  return mercator_pixel(fix[0], unwrapped_longitude(fix[1], center_longitude), zoom);
}

double MapBasemap::center_longitude_of(const MapViewport & view) const
{
  return projector_.reverse(view.center_east, view.center_north, 0.0)[1];
}

MapTileRange MapBasemap::tile_range_of(const MapViewport & view) const
{
  const double width = view.cell.width;
  const double height = view.cell.height;
  const auto center = projector_.reverse(view.center_east, view.center_north, 0.0);
  MapTileRange range;
  range.zoom = map_tile_zoom(center[0], view.px_per_m, options_.max_zoom);
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  bool first = true;
  for (const auto & [x, y] : std::array<std::array<double, 2>, 4>{
         {{0.0, 0.0}, {width, 0.0}, {0.0, height}, {width, height}}}) {
    const auto px = mercator_of(view, x, y, range.zoom, center[1]);
    if (!std::isfinite(px.x) || !std::isfinite(px.y)) {
      return range;  // off the projection (beyond ~85 deg): nothing to draw
    }
    min_x = first ? px.x : std::min(min_x, px.x);
    max_x = first ? px.x : std::max(max_x, px.x);
    min_y = first ? px.y : std::min(min_y, px.y);
    max_y = first ? px.y : std::max(max_y, px.y);
    first = false;
  }
  const int last_row = (1 << range.zoom) - 1;
  range.x0 = tile_index_of(min_x);
  range.x1 = tile_index_of(max_x);
  range.y0 = std::max(tile_index_of(min_y), 0);
  range.y1 = std::min(tile_index_of(max_y), last_row);
  return range;
}

cv::Mat MapBasemap::tile_of(const MapTileKey & key)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (const auto it = tiles_.find(key); it != tiles_.end()) {
      return it->second;
    }
  }
  cv::Mat tile;
  if (options_.source) {
    tile = options_.source->tile(key).value_or(cv::Mat{});
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  return tiles_.emplace(key, tile).first->second;
}

MapBasemap::PrefetchReport MapBasemap::prefetch(
  std::span<const MapViewport> views, unsigned threads)
{
  std::set<MapTileKey> wanted;
  for (const auto & view : views) {
    const auto range = tile_range_of(view);
    if (range.empty() || range.width() * range.height() > kMapMaxMosaicTiles) {
      continue;
    }
    for (int y = range.y0; y <= range.y1; ++y) {
      for (int x = range.x0; x <= range.x1; ++x) {
        wanted.insert(MapTileKey{range.zoom, wrapped_column(x, range.zoom), y});
      }
    }
  }
  std::vector<MapTileKey> pending;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & key : wanted) {
      if (!tiles_.contains(key)) {
        pending.push_back(key);
      }
    }
  }
  PrefetchReport report;
  report.needed = wanted.size();
  std::atomic<std::size_t> next{0};
  const auto worker = [&]() {
    for (auto i = next.fetch_add(1); i < pending.size(); i = next.fetch_add(1)) {
      (void)tile_of(pending[i]);
    }
  };
  std::vector<std::thread> workers;
  const std::size_t worker_count =
    std::min<std::size_t>(std::max(threads, 1U), std::max<std::size_t>(pending.size(), 1));
  for (std::size_t i = 1; i < worker_count; ++i) {
    workers.emplace_back(worker);
  }
  worker();
  for (auto & thread : workers) {
    thread.join();
  }
  // Counted over everything wanted, so tiles found missing by an earlier
  // prefetch are reported again.
  const std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & key : wanted) {
    if (const auto it = tiles_.find(key); it != tiles_.end() && it->second.empty()) {
      ++report.missing;
    }
  }
  return report;
}

std::string MapBasemap::source_error() const
{
  return options_.source ? options_.source->last_error() : std::string{"no tile source"};
}

void MapBasemap::build_mosaic(const MapTileRange & range)
{
  mosaic_ = cv::Mat(
    range.height() * kMapTileSizePx, range.width() * kMapTileSizePx, CV_8UC3, kMapNoDataColor);
  for (int y = range.y0; y <= range.y1; ++y) {
    for (int x = range.x0; x <= range.x1; ++x) {
      const cv::Mat tile = tile_of(MapTileKey{range.zoom, wrapped_column(x, range.zoom), y});
      if (tile.empty()) {
        continue;
      }
      const cv::Rect at(
        (x - range.x0) * kMapTileSizePx, (y - range.y0) * kMapTileSizePx, kMapTileSizePx,
        kMapTileSizePx);
      tile.copyTo(mosaic_(at));
    }
  }
  mosaic_range_ = range;
}

void MapBasemap::draw(cv::Mat & canvas, const MapViewport & view)
{
  const auto range = tile_range_of(view);
  if (range.empty() || range.width() * range.height() > kMapMaxMosaicTiles) {
    canvas.setTo(kMapNoDataColor);
    return;
  }
  if (range != mosaic_range_ || mosaic_.empty()) {
    build_mosaic(range);
  }
  // The ENU plane is not quite the Mercator plane, but over one panel the
  // difference is below a pixel (the plane's curvature and the meridians'
  // convergence are second-order in the panel's extent), so an affine map
  // fitted at three corners places every pixel: cell (x, y) -> mosaic pixel.
  const double width = view.cell.width;
  const double height = view.cell.height;
  const double center_longitude = center_longitude_of(view);
  const auto mosaic_px = [&](double x, double y) {
    const auto px = mercator_of(view, x, y, range.zoom, center_longitude);
    return cv::Point2f(
      static_cast<float>(px.x - static_cast<double>(range.x0) * kMapTileSizePx),
      static_cast<float>(px.y - static_cast<double>(range.y0) * kMapTileSizePx));
  };
  const std::array<cv::Point2f, 3> cell{
    cv::Point2f(0.0f, 0.0f), cv::Point2f(static_cast<float>(width), 0.0f),
    cv::Point2f(0.0f, static_cast<float>(height))};
  const std::array<cv::Point2f, 3> mosaic{
    mosaic_px(0.0, 0.0), mosaic_px(width, 0.0), mosaic_px(0.0, height)};
  const cv::Mat cell_to_mosaic = cv::getAffineTransform(cell.data(), mosaic.data());
  cv::warpAffine(
    mosaic_, canvas, cell_to_mosaic, canvas.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
    cv::BORDER_CONSTANT, kMapNoDataColor);
}

}  // namespace bagwiz::commands
