// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "movify_layout.hpp"       // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_basemap.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_tiles.hpp"    // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_track.hpp"    // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"        // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

using bagwiz::commands::CellView;
using bagwiz::commands::fit_map_viewport;
using bagwiz::commands::follow_map_viewport;
using bagwiz::commands::kMapFitMarginPx;
using bagwiz::commands::kMapMinExtentM;
using bagwiz::commands::kMapTileSizePx;
using bagwiz::commands::map_scale_bar_meters;
using bagwiz::commands::map_ui_scale;
using bagwiz::commands::MapBasemap;
using bagwiz::commands::MapFix;
using bagwiz::commands::MapOrigin;
using bagwiz::commands::MapPanel;
using bagwiz::commands::MapTileKey;
using bagwiz::commands::MapTileSource;
using bagwiz::commands::MapTrack;
using bagwiz::commands::MapViewport;
using bagwiz::commands::PanelSize;
using bagwiz::commands::SyntheticSizing;
using bagwiz::commands::TickInfo;

constexpr PanelSize kHd{1280, 720};

MapFix at(double east, double north, std::int64_t record_ns = 0)
{
  MapFix fix;
  fix.east = east;
  fix.north = north;
  fix.record_ns = record_ns;
  return fix;
}

// A track through the given fixes, with its bounds.
MapTrack track_of(std::vector<MapFix> fixes)
{
  MapTrack track;
  track.fixes = std::move(fixes);
  track.min_east = track.max_east = track.fixes.front().east;
  track.min_north = track.max_north = track.fixes.front().north;
  for (const auto & fix : track.fixes) {
    track.min_east = std::min(track.min_east, fix.east);
    track.max_east = std::max(track.max_east, fix.east);
    track.min_north = std::min(track.min_north, fix.north);
    track.max_north = std::max(track.max_north, fix.north);
  }
  return track;
}

// A packed BGR buffer the size of one cell, and a view onto it.
struct CellBuffer
{
  explicit CellBuffer(PanelSize size)
  : width(size.width), height(size.height), pixels(static_cast<std::size_t>(width) * height * 3U)
  {
  }
  [[nodiscard]] CellView view()
  {
    return CellView{pixels.data(), width, height, static_cast<std::size_t>(width) * 3U};
  }
  [[nodiscard]] bool is_black_at(std::uint32_t x, std::uint32_t y) const
  {
    const std::size_t idx = (static_cast<std::size_t>(y) * width + x) * 3U;
    return pixels[idx] == std::byte{0} && pixels[idx + 1] == std::byte{0} &&
           pixels[idx + 2] == std::byte{0};
  }
  [[nodiscard]] bool is_white_at(std::uint32_t x, std::uint32_t y) const
  {
    const std::size_t idx = (static_cast<std::size_t>(y) * width + x) * 3U;
    return pixels[idx] == std::byte{255} && pixels[idx + 1] == std::byte{255} &&
           pixels[idx + 2] == std::byte{255};
  }
  [[nodiscard]] bool is_color_at(std::uint32_t x, std::uint32_t y, int b, int g, int r) const
  {
    const std::size_t idx = (static_cast<std::size_t>(y) * width + x) * 3U;
    return std::to_integer<int>(pixels[idx]) == b && std::to_integer<int>(pixels[idx + 1]) == g &&
           std::to_integer<int>(pixels[idx + 2]) == r;
  }
  std::uint32_t width;
  std::uint32_t height;
  std::vector<std::byte> pixels;
};

TEST(MapUiScale, FollowsTheCellHeightWithinBounds)
{
  EXPECT_DOUBLE_EQ(map_ui_scale(kHd), 1.0);
  EXPECT_DOUBLE_EQ(map_ui_scale(PanelSize{3840, 2160}), 3.0);
  EXPECT_DOUBLE_EQ(map_ui_scale(PanelSize{320, 180}), 0.5);    // floor
  EXPECT_DOUBLE_EQ(map_ui_scale(PanelSize{7680, 4320}), 4.0);  // ceiling
}

TEST(MapViewport, FitsTheTrackBoundingBoxCentered)
{
  const auto track = track_of({at(0, 0), at(100, 50)});
  const MapViewport view = fit_map_viewport(track, kHd);
  // The 100x50 m box, inside the 48 px margin of a 1280x720 cell: the width
  // binds (1184/100 = 11.84 px/m against 624/50 = 12.48).
  EXPECT_DOUBLE_EQ(view.center_east, 50.0);
  EXPECT_DOUBLE_EQ(view.center_north, 25.0);
  EXPECT_NEAR(view.px_per_m, (1280.0 - 2.0 * kMapFitMarginPx) / 100.0, 1e-9);
  EXPECT_NEAR(view.x_of(50.0), 640.0, 1e-9);
  EXPECT_NEAR(view.y_of(25.0), 360.0, 1e-9);
  EXPECT_NEAR(view.x_of(0.0), kMapFitMarginPx, 1e-9);
  EXPECT_NEAR(view.x_of(100.0), 1280.0 - kMapFitMarginPx, 1e-9);
  // North is up: a higher north lands at a smaller y.
  EXPECT_NEAR(view.y_of(50.0), 360.0 - 25.0 * view.px_per_m, 1e-9);
  EXPECT_LT(view.y_of(50.0), view.y_of(0.0));
}

TEST(MapViewport, FitsAStationaryTrackToTheMinimumExtent)
{
  const auto track = track_of({at(3, 4), at(3.1, 4.1)});
  const MapViewport view = fit_map_viewport(track, kHd);
  EXPECT_NEAR(view.center_east, 3.05, 1e-9);
  EXPECT_NEAR(view.center_north, 4.05, 1e-9);
  // The height binds against the 20 m floor: 624/20 = 31.2 px/m.
  EXPECT_NEAR(view.px_per_m, (720.0 - 2.0 * kMapFitMarginPx) / kMapMinExtentM, 1e-9);
}

TEST(MapViewport, FollowsTheFixAtTheRange)
{
  const MapViewport view = follow_map_viewport(at(10, -5), 50.0, kHd);
  EXPECT_DOUBLE_EQ(view.center_east, 10.0);
  EXPECT_DOUBLE_EQ(view.center_north, -5.0);
  // The shorter axis (720 px) spans +-50 m: 7.2 px/m.
  EXPECT_DOUBLE_EQ(view.px_per_m, 7.2);
  EXPECT_DOUBLE_EQ(view.x_of(10.0), 640.0);
  EXPECT_DOUBLE_EQ(view.y_of(-5.0), 360.0);
  EXPECT_DOUBLE_EQ(view.y_of(45.0), 0.0);
  EXPECT_DOUBLE_EQ(view.x_of(60.0), 1000.0);
}

TEST(MapScaleBar, PicksANiceLengthUnderAQuarterOfTheWidth)
{
  MapViewport view;
  view.cell = kHd;
  view.px_per_m = 11.84;  // a quarter of the width is 27.0 m -> 20 m
  EXPECT_DOUBLE_EQ(map_scale_bar_meters(view), 20.0);
  view.px_per_m = 1.0;  // 320 m -> 200 m
  EXPECT_DOUBLE_EQ(map_scale_bar_meters(view), 200.0);
  view.px_per_m = 3.2;  // exactly 100 m
  EXPECT_DOUBLE_EQ(map_scale_bar_meters(view), 100.0);
  view.px_per_m = 0.001;  // 320 km -> 200 km
  EXPECT_DOUBLE_EQ(map_scale_bar_meters(view), 200'000.0);
  view.px_per_m = 500.0;  // 0.64 m: never below 1 m
  EXPECT_DOUBLE_EQ(map_scale_bar_meters(view), 1.0);
}

TEST(MapPanel, ClockRoleSizesTheDefaultCell)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100)});
  MapPanel panel(std::move(options), SyntheticSizing{});
  EXPECT_FALSE(panel.clock_cell_size().has_value());  // nothing selected yet
  EXPECT_EQ(panel.select(TickInfo{}, PanelSize{}), "");
  ASSERT_TRUE(panel.clock_cell_size().has_value());
  EXPECT_EQ(panel.clock_cell_size()->width, 1280u);
  EXPECT_EQ(panel.clock_cell_size()->height, 720u);
}

TEST(MapPanel, ClockRoleSplitsAFixedWidthAcrossTheColumns)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100)});
  SyntheticSizing sizing;
  sizing.total_width = 1920;
  sizing.grid_cols = 2;
  MapPanel panel(std::move(options), sizing);
  EXPECT_EQ(panel.select(TickInfo{}, PanelSize{}), "");
  ASSERT_TRUE(panel.clock_cell_size().has_value());
  EXPECT_EQ(panel.clock_cell_size()->width, 960u);
  EXPECT_EQ(panel.clock_cell_size()->height, 540u);
}

TEST(MapPanel, FollowerRendersTheMarkerAtTheFixNearestTheTick)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100), at(100, 0, 200)});
  MapPanel panel(std::move(options));
  EXPECT_FALSE(panel.clock_cell_size().has_value());  // never the clock

  // A 320x180 cell draws at the 0.5 floor of the UI scale: a 24 px margin
  // leaves 272 px for the 100 m track (2.72 px/m) and the 20 m floor of the
  // north extent fits easily, so the fixes land at x = 24 and x = 296, on
  // the middle row.
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 200;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  EXPECT_TRUE(buffer.is_white_at(296, 90));   // the marker's filled center
  EXPECT_FALSE(buffer.is_black_at(24, 90));   // the traveled track at the first fix
  EXPECT_FALSE(buffer.is_black_at(160, 90));  // and between the two
  EXPECT_FALSE(buffer.is_black_at(306, 25));  // the north arrow's shaft (top-right)

  // The earlier tick moves the marker back to the first fix.
  CellBuffer earlier(cell);
  tick.record_ns = 100;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(earlier.view()), "");
  EXPECT_TRUE(earlier.is_white_at(24, 90));
  EXPECT_FALSE(earlier.is_white_at(296, 90));
}

TEST(MapPanel, FollowModeCentersTheCurrentFix)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100), at(1000, 0, 200)});
  options.follow_range_m = 50.0;
  MapPanel panel(std::move(options));
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 200;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  // The fix sits at the cell's center whatever the track's extent.
  EXPECT_TRUE(buffer.is_white_at(160, 90));
}

TEST(MapPanel, RendersASingleFixTrack)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100)});
  MapPanel panel(std::move(options));
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 100;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  // A one-point track has no line to draw; the fix sits at the center of the
  // minimum-extent fit, with no heading.
  EXPECT_TRUE(buffer.is_white_at(160, 90));
}

TEST(MapPanel, HeadingArrowPointsAlongTheTravel)
{
  // Ten meters due north, followed at +-20 m in a 320x180 cell: 4.5 px/m, the
  // current fix at the center, the arrow (26 px at the 0.5 UI scale) rising
  // from it to y = 77.
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100), at(0, 10, 200)});
  options.follow_range_m = 20.0;
  MapPanel panel(std::move(options));
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 200;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  EXPECT_TRUE(buffer.is_white_at(160, 90));   // the marker
  EXPECT_FALSE(buffer.is_black_at(160, 80));  // the arrow's shaft, above it
  EXPECT_TRUE(buffer.is_black_at(150, 80));   // nothing beside the shaft
  EXPECT_TRUE(buffer.is_black_at(170, 80));
  EXPECT_FALSE(buffer.is_black_at(160, 120));  // the traveled track, below
}

// A tile source of one solid color, and one that has no tiles at all.
class SolidTileSource final : public MapTileSource
{
public:
  explicit SolidTileSource(cv::Scalar color) : color_(color) {}
  std::optional<cv::Mat> tile(const MapTileKey &) override
  {
    return cv::Mat(kMapTileSizePx, kMapTileSizePx, CV_8UC3, color_);
  }

private:
  cv::Scalar color_;
};

class MissingTileSource final : public MapTileSource
{
public:
  std::optional<cv::Mat> tile(const MapTileKey &) override { return std::nullopt; }
  [[nodiscard]] std::string last_error() const override { return "no tiles here"; }
};

MapPanel::Options options_over(std::shared_ptr<MapTileSource> source)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100), at(100, 0, 200)});
  options.track.origin = MapOrigin{35.0, 139.0, 0.0};
  MapBasemap::Options basemap;
  basemap.origin = options.track.origin;
  basemap.source = std::move(source);
  options.basemap = std::make_shared<MapBasemap>(std::move(basemap));
  options.attribution = "(c) test tiles";
  return options;
}

// The same 320x180 cell as above: the track on the middle row from x = 24
// to 296, a 20 m grid (lines at y = 35.6, 90, 144.4). (160, 60) is off the
// track, the grid and every overlay.
TEST(MapPanel, DrawsTheBasemapUnderTheTrack)
{
  MapPanel panel(options_over(std::make_shared<SolidTileSource>(cv::Scalar(70, 80, 90))));
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 200;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  EXPECT_TRUE(buffer.is_color_at(160, 60, 70, 80, 90));   // the tiles show through
  EXPECT_TRUE(buffer.is_white_at(296, 90));               // the marker sits on top
  EXPECT_FALSE(buffer.is_color_at(160, 90, 70, 80, 90));  // as does the track
}

TEST(MapPanel, FallsBackToTheGridWhenNoTileCanBeFetched)
{
  MapPanel panel(options_over(std::make_shared<MissingTileSource>()));
  const PanelSize cell{320, 180};
  CellBuffer buffer(cell);
  TickInfo tick;
  tick.record_ns = 200;
  EXPECT_EQ(panel.select(tick, cell), "");
  EXPECT_EQ(panel.render(buffer.view()), "");
  EXPECT_TRUE(buffer.is_black_at(160, 60));  // no no-data fill: the plain plan view
  EXPECT_TRUE(buffer.is_white_at(296, 90));
}

TEST(MapPanel, RendersNothingBeforeASelection)
{
  MapPanel::Options options;
  options.track = track_of({at(0, 0, 100)});
  MapPanel panel(std::move(options));
  CellBuffer buffer(PanelSize{64, 32});
  EXPECT_EQ(panel.render(buffer.view()), "");
  for (std::uint32_t y = 0; y < 32; ++y) {
    for (std::uint32_t x = 0; x < 64; ++x) {
      ASSERT_TRUE(buffer.is_black_at(x, y)) << x << "," << y;
    }
  }
}

TEST(MapPanel, RefusesAnEmptyCell)
{
  MapPanel::Options options;
  options.topic = "/gnss";
  options.track = track_of({at(0, 0, 100)});
  MapPanel panel(std::move(options));
  EXPECT_EQ(
    panel.select(TickInfo{}, PanelSize{}),
    "topic '/gnss': the map panel has no cell to render into");
}

}  // namespace
