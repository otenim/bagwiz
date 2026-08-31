// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_tiles.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "movify_test_util.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <utility>

namespace
{

using bagwiz::commands::default_map_tile_cache_dir;
using bagwiz::commands::expand_map_tile_template;
using bagwiz::commands::HttpMapTileSource;
using bagwiz::commands::kMapTileMaxZoom;
using bagwiz::commands::kMapTileSizePx;
using bagwiz::commands::kMapTilesNone;
using bagwiz::commands::kMercatorEquatorMetersPerPixel;
using bagwiz::commands::map_tile_attribution;
using bagwiz::commands::map_tile_zoom;
using bagwiz::commands::MapTileKey;
using bagwiz::commands::mercator_meters_per_pixel;
using bagwiz::commands::mercator_pixel;
using bagwiz::commands::validate_map_tile_template;
using bagwiz::test::MovifyTmpDirTest;

// Sets (or unsets) one environment variable for a scope.
class EnvVarGuard
{
public:
  EnvVarGuard(std::string name, const std::optional<std::string> & value) : name_(std::move(name))
  {
    const char * const previous = std::getenv(name_.c_str());
    had_previous_ = previous != nullptr;
    if (had_previous_) {
      previous_value_ = previous;
    }
    if (value.has_value()) {
      setenv(name_.c_str(), value->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }
  EnvVarGuard(const EnvVarGuard &) = delete;
  EnvVarGuard & operator=(const EnvVarGuard &) = delete;
  EnvVarGuard(EnvVarGuard &&) = delete;
  EnvVarGuard & operator=(EnvVarGuard &&) = delete;
  ~EnvVarGuard()
  {
    if (had_previous_) {
      setenv(name_.c_str(), previous_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::string previous_value_;
  bool had_previous_ = false;
};

// Writes a solid-colored `size`-square PNG tile at <dir>/<z>/<x>/<y>.png.
void write_tile(
  const std::filesystem::path & dir, const MapTileKey & key, const cv::Scalar & color,
  int size = kMapTileSizePx)
{
  const auto path =
    dir / std::to_string(key.zoom) / std::to_string(key.x) / (std::to_string(key.y) + ".png");
  std::filesystem::create_directories(path.parent_path());
  const cv::Mat tile(size, size, CV_8UC3, color);
  ASSERT_TRUE(cv::imwrite(path.string(), tile));
}

TEST(MercatorPixel, TheWorldIsOneTileAtZoomZero)
{
  const auto center = mercator_pixel(0.0, 0.0, 0);
  EXPECT_NEAR(center.x, 128.0, 1e-9);
  EXPECT_NEAR(center.y, 128.0, 1e-9);
  // The projection's northern and southern edges are the tile's top and bottom.
  EXPECT_NEAR(mercator_pixel(85.0511287798, 0.0, 0).y, 0.0, 1e-6);
  EXPECT_NEAR(mercator_pixel(-85.0511287798, 0.0, 0).y, 256.0, 1e-6);
  // West is x = 0, east is x = 256.
  EXPECT_NEAR(mercator_pixel(0.0, -180.0, 0).x, 0.0, 1e-9);
  EXPECT_NEAR(mercator_pixel(0.0, 180.0, 0).x, 256.0, 1e-9);
}

TEST(MercatorPixel, MatchesTheReferenceFormulaAtDeeperZooms)
{
  // Reference values from the slippy-map formulas evaluated independently.
  const auto tokyo = mercator_pixel(35.0, 139.0, 1);
  EXPECT_NEAR(tokyo.x, 453.68888888888887, 1e-9);
  EXPECT_NEAR(tokyo.y, 202.80208962887065, 1e-9);
  const auto louisiana = mercator_pixel(30.314781, -91.834406, 16);
  EXPECT_NEAR(louisiana.x, 4108814.4841841776, 1e-6);
  EXPECT_NEAR(louisiana.y, 6904897.1450983165, 1e-6);
}

TEST(MercatorMetersPerPixel, ShrinksWithLatitudeAndZoom)
{
  EXPECT_NEAR(mercator_meters_per_pixel(0.0, 0), kMercatorEquatorMetersPerPixel, 1e-9);
  EXPECT_NEAR(mercator_meters_per_pixel(35.0, 16), 1.956673374248794, 1e-9);
}

TEST(MapTileZoom, PicksTheCoarsestZoomAtLeastAsFineAsThePanel)
{
  // 2.5 m/px at the equator: zoom 16 (2.39 m/px) is fine enough, 15 (4.77) is not.
  EXPECT_EQ(map_tile_zoom(0.0, 1.0 / 2.5), 16);
  // A whole-world panel wants zoom 0; a panel finer than any tile clamps to the deepest.
  EXPECT_EQ(map_tile_zoom(0.0, 1.0 / kMercatorEquatorMetersPerPixel), 0);
  EXPECT_EQ(map_tile_zoom(0.0, 1e-9), 0);
  EXPECT_EQ(map_tile_zoom(35.0, 18.7), kMapTileMaxZoom);
  EXPECT_EQ(map_tile_zoom(35.0, 18.7, 17), 17);
  // A degenerate scale is zoom 0, not a NaN turned into an int.
  EXPECT_EQ(map_tile_zoom(0.0, 0.0), 0);
}

TEST(MapTileTemplate, ExpandsThePlaceholders)
{
  EXPECT_EQ(
    expand_map_tile_template("https://tiles.example/{z}/{x}/{y}.png", MapTileKey{16, 4108, 6904}),
    "https://tiles.example/16/4108/6904.png");
  // Every occurrence is filled, whatever their order.
  EXPECT_EQ(
    expand_map_tile_template("file:///t/{y}_{x}_{z}/{z}.png", MapTileKey{1, 2, 3}),
    "file:///t/3_2_1/1.png");
}

TEST(MapTileTemplate, ValidatesTheSchemeAndThePlaceholders)
{
  EXPECT_EQ(validate_map_tile_template(kMapTilesNone), "");
  EXPECT_EQ(validate_map_tile_template("https://tile.openstreetmap.org/{z}/{x}/{y}.png"), "");
  EXPECT_EQ(validate_map_tile_template("http://localhost:8080/{z}/{x}/{y}.png"), "");
  EXPECT_EQ(validate_map_tile_template("file:///tmp/tiles/{z}/{x}/{y}.png"), "");
  EXPECT_NE(validate_map_tile_template("https://tiles.example/{z}/{x}.png"), "");
  EXPECT_NE(validate_map_tile_template("ftp://tiles.example/{z}/{x}/{y}.png"), "");
  EXPECT_NE(validate_map_tile_template("tiles.example/{z}/{x}/{y}.png"), "");
  EXPECT_NE(validate_map_tile_template(""), "");
}

TEST(MapTileTemplate, AttributesOpenStreetMapOnly)
{
  EXPECT_EQ(
    map_tile_attribution("https://tile.openstreetmap.org/{z}/{x}/{y}.png"),
    "(c) OpenStreetMap contributors");
  EXPECT_EQ(
    map_tile_attribution("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png"),
    "(c) OpenStreetMap contributors");
  EXPECT_EQ(map_tile_attribution("https://tiles.example/{z}/{x}/{y}.png"), "");
  EXPECT_EQ(map_tile_attribution("file:///tmp/openstreetmap.org/{z}/{x}/{y}.png"), "");
}

TEST(MapTileCacheDir, FollowsXdgThenHome)
{
  {
    const EnvVarGuard xdg("XDG_CACHE_HOME", std::string{"/tmp/xdg-cache"});
    EXPECT_EQ(default_map_tile_cache_dir(), std::filesystem::path("/tmp/xdg-cache/bagwiz/tiles"));
  }
  {
    const EnvVarGuard xdg("XDG_CACHE_HOME", std::nullopt);
    const EnvVarGuard home("HOME", std::string{"/home/someone"});
    EXPECT_EQ(
      default_map_tile_cache_dir(), std::filesystem::path("/home/someone/.cache/bagwiz/tiles"));
  }
  {
    const EnvVarGuard xdg("XDG_CACHE_HOME", std::nullopt);
    const EnvVarGuard home("HOME", std::nullopt);
    EXPECT_FALSE(default_map_tile_cache_dir().has_value());
  }
}

TEST_F(MovifyTmpDirTest, FileSourceReadsTilesFromTheDirectory)
{
  const auto tiles = tmp_dir_ / "tiles";
  write_tile(tiles, MapTileKey{3, 1, 2}, cv::Scalar(10, 20, 30));
  HttpMapTileSource::Options options;
  options.url_template = "file://" + (tiles / "{z}/{x}/{y}.png").string();
  HttpMapTileSource source(std::move(options));

  const auto tile = source.tile(MapTileKey{3, 1, 2});
  ASSERT_TRUE(tile.has_value());
  EXPECT_EQ(tile->rows, kMapTileSizePx);
  EXPECT_EQ(tile->cols, kMapTileSizePx);
  EXPECT_EQ(tile->type(), CV_8UC3);
  EXPECT_EQ(tile->at<cv::Vec3b>(100, 100), cv::Vec3b(10, 20, 30));
  EXPECT_EQ(source.last_error(), "");

  // A tile the directory lacks is nullopt, with the reason kept.
  EXPECT_FALSE(source.tile(MapTileKey{3, 1, 3}).has_value());
  EXPECT_NE(source.last_error(), "");
}

TEST_F(MovifyTmpDirTest, FileSourceResizesARetinaTile)
{
  const auto tiles = tmp_dir_ / "tiles";
  write_tile(tiles, MapTileKey{0, 0, 0}, cv::Scalar(40, 50, 60), 512);
  HttpMapTileSource::Options options;
  options.url_template = "file://" + (tiles / "{z}/{x}/{y}.png").string();
  HttpMapTileSource source(std::move(options));
  const auto tile = source.tile(MapTileKey{0, 0, 0});
  ASSERT_TRUE(tile.has_value());
  EXPECT_EQ(tile->rows, kMapTileSizePx);
  EXPECT_EQ(tile->cols, kMapTileSizePx);
  EXPECT_EQ(tile->at<cv::Vec3b>(128, 128), cv::Vec3b(40, 50, 60));
}

TEST_F(MovifyTmpDirTest, FileSourceRejectsATileThatIsNotAnImage)
{
  const auto tiles = tmp_dir_ / "tiles";
  std::filesystem::create_directories(tiles / "0/0");
  std::ofstream(tiles / "0/0/0.png") << "not a png";
  HttpMapTileSource::Options options;
  options.url_template = "file://" + (tiles / "{z}/{x}/{y}.png").string();
  HttpMapTileSource source(std::move(options));
  EXPECT_FALSE(source.tile(MapTileKey{0, 0, 0}).has_value());
  EXPECT_NE(source.last_error(), "");
}

TEST_F(MovifyTmpDirTest, HttpSourceKeepsFetchedTilesInTheCacheDirectory)
{
  // An unreachable server with a warm cache: the cached tile is served, the
  // uncached one fails with the transport's reason.
  const auto cache = tmp_dir_ / "cache";
  HttpMapTileSource::Options options;
  options.url_template = "http://127.0.0.1:9/{z}/{x}/{y}.png";  // port 9: discard, refuses
  options.cache_dir = cache;
  options.timeout_s = 2;
  write_tile(cache / "127.0.0.1_9", MapTileKey{2, 1, 1}, cv::Scalar(1, 2, 3));
  HttpMapTileSource source(std::move(options));

  const auto cached = source.tile(MapTileKey{2, 1, 1});
  ASSERT_TRUE(cached.has_value());
  EXPECT_EQ(cached->at<cv::Vec3b>(0, 0), cv::Vec3b(1, 2, 3));
  EXPECT_EQ(source.last_error(), "");

  EXPECT_FALSE(source.tile(MapTileKey{2, 1, 2}).has_value());
  EXPECT_NE(source.last_error(), "");
}

TEST_F(MovifyTmpDirTest, HttpSourceDropsACachedTileThatDoesNotDecode)
{
  // A corrupt cached tile is removed and fetched again (which fails here,
  // the server being unreachable) instead of being served — or reported
  // missing — forever.
  const auto cache = tmp_dir_ / "cache";
  HttpMapTileSource::Options options;
  options.url_template = "http://127.0.0.1:9/{z}/{x}/{y}.png";
  options.cache_dir = cache;
  options.timeout_s = 2;
  const auto corrupt = cache / "127.0.0.1_9" / "2" / "1" / "1.png";
  std::filesystem::create_directories(corrupt.parent_path());
  std::ofstream(corrupt) << "not a png";
  HttpMapTileSource source(std::move(options));

  EXPECT_FALSE(source.tile(MapTileKey{2, 1, 1}).has_value());
  EXPECT_FALSE(std::filesystem::exists(corrupt));
  EXPECT_NE(source.last_error(), "");
}

}  // namespace
