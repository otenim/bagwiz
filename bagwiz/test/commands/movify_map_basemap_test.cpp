// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_basemap.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "movify_map_tiles.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_track.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_viewport.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"         // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::kMapNoDataColor;
using bagwiz::commands::kMapTileSizePx;
using bagwiz::commands::MapBasemap;
using bagwiz::commands::MapOrigin;
using bagwiz::commands::MapTileKey;
using bagwiz::commands::MapTileRange;
using bagwiz::commands::MapTileSource;
using bagwiz::commands::MapViewport;
using bagwiz::commands::mercator_pixel;
using bagwiz::commands::PanelSize;

constexpr PanelSize kCell{320, 180};
const MapOrigin kOrigin{35.0, 139.0, 40.0};

// Every tile one color, counting the requests and remembering their keys.
class SolidTileSource final : public MapTileSource
{
public:
  explicit SolidTileSource(cv::Scalar color) : color_(color) {}
  std::optional<cv::Mat> tile(const MapTileKey & key) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++requests_;
    keys_.insert(key);
    return cv::Mat(kMapTileSizePx, kMapTileSizePx, CV_8UC3, color_);
  }
  [[nodiscard]] std::size_t requests() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }
  [[nodiscard]] std::set<MapTileKey> keys() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return keys_;
  }

private:
  cv::Scalar color_;
  mutable std::mutex mutex_;
  std::size_t requests_ = 0;
  std::set<MapTileKey> keys_;
};

// Gray tiles with a red dot where the origin fix lies, so a drawn basemap
// reveals where the ENU origin landed on the panel.
class OriginMarkerTileSource final : public MapTileSource
{
public:
  std::optional<cv::Mat> tile(const MapTileKey & key) override
  {
    cv::Mat tile(kMapTileSizePx, kMapTileSizePx, CV_8UC3, cv::Scalar(128, 128, 128));
    const auto origin = mercator_pixel(kOrigin.latitude, kOrigin.longitude, key.zoom);
    const double local_x = origin.x - static_cast<double>(key.x) * kMapTileSizePx;
    const double local_y = origin.y - static_cast<double>(key.y) * kMapTileSizePx;
    if (local_x >= 0.0 && local_x < kMapTileSizePx && local_y >= 0.0 && local_y < kMapTileSizePx) {
      cv::circle(
        tile, cv::Point(static_cast<int>(local_x), static_cast<int>(local_y)), 3,
        cv::Scalar(0, 0, 255), cv::FILLED);
    }
    return tile;
  }
};

class MissingTileSource final : public MapTileSource
{
public:
  std::optional<cv::Mat> tile(const MapTileKey &) override { return std::nullopt; }
};

MapViewport view_at(double east, double north, double px_per_m = 1.0)
{
  MapViewport view;
  view.center_east = east;
  view.center_north = north;
  view.px_per_m = px_per_m;
  view.cell = kCell;
  return view;
}

// Whether a red dot lies within `radius` pixels of (x, y) in `canvas`.
bool red_near(const cv::Mat & canvas, int x, int y, int radius = 2)
{
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int px = x + dx;
      const int py = y + dy;
      if (px < 0 || py < 0 || px >= canvas.cols || py >= canvas.rows) {
        continue;
      }
      const auto & p = canvas.at<cv::Vec3b>(py, px);
      if (p[2] > 150 && p[0] < 90 && p[1] < 90) {
        return true;
      }
    }
  }
  return false;
}

MapBasemap basemap_with(std::shared_ptr<MapTileSource> source, MapOrigin origin = kOrigin)
{
  MapBasemap::Options options;
  options.origin = origin;
  options.source = std::move(source);
  return MapBasemap(std::move(options));
}

TEST(MapTileRange, CoversTheCellAroundTheOriginAtAFittingZoom)
{
  MapBasemap basemap = basemap_with(std::make_shared<SolidTileSource>(cv::Scalar(1, 1, 1)));
  const auto range = basemap.tile_range_of(view_at(0.0, 0.0));
  // 1 px/m at 35 deg: zoom 17 (0.98 m/px) is the coarsest fine enough.
  EXPECT_EQ(range.zoom, 17);
  const auto origin = mercator_pixel(kOrigin.latitude, kOrigin.longitude, range.zoom);
  const int origin_x = static_cast<int>(std::floor(origin.x / kMapTileSizePx));
  const int origin_y = static_cast<int>(std::floor(origin.y / kMapTileSizePx));
  EXPECT_LE(range.x0, origin_x);
  EXPECT_GE(range.x1, origin_x);
  EXPECT_LE(range.y0, origin_y);
  EXPECT_GE(range.y1, origin_y);
  // A 320x180 cell at ~1 px per tile pixel spans at most 3x2 tiles.
  EXPECT_GE(range.width(), 1);
  EXPECT_LE(range.width(), 3);
  EXPECT_GE(range.height(), 1);
  EXPECT_LE(range.height(), 2);
}

// A view straddling the antimeridian: its corners' longitudes jump from
// +180 to -180, yet the range is the few adjacent columns across the seam
// (the ones past the world's edge wrap when looked up), not the whole world.
TEST(MapTileRange, StaysContiguousAcrossTheAntimeridian)
{
  const MapOrigin seam{0.0, 179.9995, 0.0};
  auto source = std::make_shared<SolidTileSource>(cv::Scalar(1, 1, 1));
  MapBasemap basemap = basemap_with(source, seam);
  const auto range = basemap.tile_range_of(view_at(0.0, 0.0));
  EXPECT_FALSE(range.empty());
  EXPECT_LE(range.width(), 3);
  EXPECT_LE(range.height(), 2);
  // The columns straddle the world's last one: x0 on its west side, x1 past it.
  const int columns = 1 << range.zoom;
  EXPECT_LT(range.x0, columns);
  EXPECT_GE(range.x1, columns);

  // And the draw fetches only those, wrapped into the world.
  cv::Mat canvas(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(canvas, view_at(0.0, 0.0));
  EXPECT_EQ(source->requests(), static_cast<std::size_t>(range.width() * range.height()));
  for (const auto & key : source->keys()) {
    EXPECT_GE(key.x, 0);
    EXPECT_LT(key.x, columns);
  }
  EXPECT_EQ(canvas.at<cv::Vec3b>(90, 160), cv::Vec3b(1, 1, 1));
}

TEST(MapBasemap, DrawsTheTilesAcrossTheWholeCell)
{
  MapBasemap basemap = basemap_with(std::make_shared<SolidTileSource>(cv::Scalar(70, 80, 90)));
  cv::Mat canvas(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(canvas, view_at(0.0, 0.0));
  for (const auto & [x, y] :
       {std::pair{0, 0}, std::pair{319, 0}, std::pair{160, 90}, std::pair{0, 179},
        std::pair{319, 179}}) {
    EXPECT_EQ(canvas.at<cv::Vec3b>(y, x), cv::Vec3b(70, 80, 90)) << x << "," << y;
  }
}

TEST(MapBasemap, PlacesTheOriginWhereTheViewportPutsIt)
{
  MapBasemap basemap = basemap_with(std::make_shared<OriginMarkerTileSource>());
  // Centered on the origin: the dot is at the cell's center.
  cv::Mat centered(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(centered, view_at(0.0, 0.0));
  EXPECT_TRUE(red_near(centered, 160, 90));
  EXPECT_FALSE(red_near(centered, 110, 90));

  // Centered 50 m east of it: the origin moves 50 px left.
  cv::Mat east(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(east, view_at(50.0, 0.0));
  EXPECT_TRUE(red_near(east, 110, 90));
  EXPECT_FALSE(red_near(east, 160, 90));

  // Centered 40 m north of it: the origin moves 40 px down.
  cv::Mat north(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(north, view_at(0.0, 40.0));
  EXPECT_TRUE(red_near(north, 160, 130));

  // Twice the scale: 50 m east is 100 px.
  cv::Mat zoomed(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(zoomed, view_at(50.0, 0.0, 2.0));
  EXPECT_TRUE(red_near(zoomed, 60, 90));
}

TEST(MapBasemap, MissingTilesLeaveTheNoDataFill)
{
  MapBasemap basemap = basemap_with(std::make_shared<MissingTileSource>());
  const std::vector<MapViewport> views{view_at(0.0, 0.0)};
  const auto report = basemap.prefetch(views, 2);
  EXPECT_GT(report.needed, 0U);
  EXPECT_EQ(report.missing, report.needed);

  cv::Mat canvas(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(canvas, views.front());
  const cv::Vec3b no_data(
    static_cast<unsigned char>(kMapNoDataColor[0]), static_cast<unsigned char>(kMapNoDataColor[1]),
    static_cast<unsigned char>(kMapNoDataColor[2]));
  EXPECT_EQ(canvas.at<cv::Vec3b>(90, 160), no_data);
  EXPECT_EQ(canvas.at<cv::Vec3b>(0, 0), no_data);
}

TEST(MapBasemap, PrefetchAsksForEachTileOnce)
{
  auto source = std::make_shared<SolidTileSource>(cv::Scalar(5, 5, 5));
  MapBasemap basemap = basemap_with(source);
  // Two overlapping views share most of their tiles.
  const std::vector<MapViewport> views{view_at(0.0, 0.0), view_at(10.0, 0.0)};
  const auto report = basemap.prefetch(views, 2);
  EXPECT_EQ(report.missing, 0U);
  EXPECT_EQ(report.needed, source->keys().size());
  EXPECT_EQ(source->requests(), source->keys().size());

  // A second prefetch and the draws are served from what is held.
  const auto again = basemap.prefetch(views, 2);
  EXPECT_EQ(again.needed, report.needed);
  cv::Mat canvas(kCell.height, kCell.width, CV_8UC3, cv::Scalar(0, 0, 0));
  basemap.draw(canvas, views.front());
  basemap.draw(canvas, views.back());
  EXPECT_EQ(source->requests(), report.needed);
}

TEST(MapBasemap, WithoutASourceEverythingIsMissing)
{
  MapBasemap basemap = basemap_with(nullptr);
  const std::vector<MapViewport> views{view_at(0.0, 0.0)};
  const auto report = basemap.prefetch(views, 1);
  EXPECT_EQ(report.missing, report.needed);
}

}  // namespace
