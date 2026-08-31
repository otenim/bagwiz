// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_MAP_TILES_HPP_
#define COMMANDS__MOVIFY_MAP_TILES_HPP_

#include <opencv2/core.hpp>

#include <compare>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>

// The map panel's tiles: the Web Mercator "slippy map" tile scheme every
// public tile server speaks (a square world of 256 * 2^zoom pixels, tile
// x/y counted from the north-west corner), and a source that fetches the
// tiles of a URL template over HTTP(S) — or from a local directory through
// a file: template — through an on-disk cache. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

inline constexpr int kMapTileSizePx = 256;
// OpenStreetMap's deepest zoom; a finer panel gets upscaled tiles.
inline constexpr int kMapTileMaxZoom = 19;
// The tile server the map panel draws from unless --map-tiles says
// otherwise, and the --map-tiles value that keeps the panel an offline plan
// view.
inline constexpr const char * kDefaultMapTileTemplate =
  "https://tile.openstreetmap.org/{z}/{x}/{y}.png";
inline constexpr const char * kMapTilesNone = "none";
// OpenStreetMap's tile usage policy asks for a User-Agent that identifies
// the application, at most two download threads, and local caching; every
// source is fetched the same way.
inline constexpr const char * kMapTileUserAgent =
  "bagwiz-movify (+https://github.com/otenim/bagwiz)";
inline constexpr unsigned kMapTileFetchThreads = 2;
inline constexpr long kMapTileTimeoutS = 20;  // NOLINT(runtime/int) libcurl takes a long
// Web Mercator's ground resolution at the equator at zoom 0: the WGS84
// equator (2 * pi * 6378137 m) across one 256-px tile.
inline constexpr double kMercatorEquatorMetersPerPixel = 156543.03392804097;

// A position in Web Mercator pixels at a zoom level: x grows east from the
// antimeridian, y grows south from ~85.05 deg north (the projection's edge).
struct MercatorPixel
{
  double x = 0.0;
  double y = 0.0;
};

[[nodiscard]] MercatorPixel mercator_pixel(double latitude_deg, double longitude_deg, int zoom);

// The ground distance one tile pixel spans at that latitude and zoom.
[[nodiscard]] double mercator_meters_per_pixel(double latitude_deg, int zoom);

// The coarsest zoom whose tiles are at least as fine as a panel drawn at
// `px_per_m`, clamped to [0, max_zoom].
[[nodiscard]] int map_tile_zoom(
  double latitude_deg, double px_per_m, int max_zoom = kMapTileMaxZoom);

struct MapTileKey
{
  int zoom = 0;
  int x = 0;
  int y = 0;

  friend auto operator<=>(const MapTileKey &, const MapTileKey &) = default;
};

// `url_template` with its {z}, {x} and {y} placeholders filled in.
[[nodiscard]] std::string expand_map_tile_template(
  const std::string & url_template, const MapTileKey & key);

// "" when `url_template` is kMapTilesNone or a http, https or file URL
// carrying all three placeholders; else the error to report.
[[nodiscard]] std::string validate_map_tile_template(const std::string & url_template);

// The attribution the panel owes the tiles' provider: OpenStreetMap's for
// its servers, "" for any other template (whose terms the caller knows).
[[nodiscard]] std::string map_tile_attribution(const std::string & url_template);

// Where fetched tiles are kept between runs: $XDG_CACHE_HOME/bagwiz/tiles,
// else ~/.cache/bagwiz/tiles; nullopt when neither HOME nor XDG_CACHE_HOME
// is set (tiles are then fetched every run).
[[nodiscard]] std::optional<std::filesystem::path> default_map_tile_cache_dir();

// A source of tile images: the kMapTileSizePx-square BGR image of a tile,
// or nullopt when the tile cannot be had (the source has said why).
class MapTileSource
{
public:
  MapTileSource() = default;
  virtual ~MapTileSource() = default;
  MapTileSource(const MapTileSource &) = delete;
  MapTileSource & operator=(const MapTileSource &) = delete;
  MapTileSource(MapTileSource &&) = delete;
  MapTileSource & operator=(MapTileSource &&) = delete;

  [[nodiscard]] virtual std::optional<cv::Mat> tile(const MapTileKey & key) = 0;

  // Why the latest failed tile() failed, for the caller's warning; "" when
  // every tile so far succeeded.
  [[nodiscard]] virtual std::string last_error() const { return ""; }
};

// Tiles from a URL template: a file: template reads the tile straight from
// disk; http(s) templates are fetched with libcurl, through `cache_dir`
// (<cache_dir>/<host>/<z>/<x>/<y>.png, kept indefinitely) when it is set.
// A tile that is not kMapTileSizePx square (a "retina" server) is resized.
// Safe to call from several threads at once.
class HttpMapTileSource final : public MapTileSource
{
public:
  struct Options
  {
    std::string url_template;
    std::optional<std::filesystem::path> cache_dir;
    std::string user_agent = kMapTileUserAgent;
    long timeout_s = kMapTileTimeoutS;  // NOLINT(runtime/int) libcurl takes a long
  };

  explicit HttpMapTileSource(Options options);

  [[nodiscard]] std::optional<cv::Mat> tile(const MapTileKey & key) override;
  [[nodiscard]] std::string last_error() const override;

private:
  [[nodiscard]] std::optional<std::filesystem::path> cache_path_of(const MapTileKey & key) const;
  void note_error(const std::string & error);

  Options options_;
  mutable std::mutex mutex_;
  std::string last_error_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_MAP_TILES_HPP_
