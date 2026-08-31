// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_MAP_PANEL_HPP_
#define COMMANDS__MOVIFY_MAP_PANEL_HPP_

#include "bagwiz/commands/movify.hpp"
#include "movify_inputs.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_layout.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_track.hpp"     // NOLINT(build/include_subdir) src-local shared header
#include "movify_map_viewport.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"         // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

// The panel that draws the GNSS track: a plan view of the ENU plane — over
// the map tiles under it, or a bare grid — with the whole track, the part
// driven so far, the current fix with its heading, a north arrow, a scale
// bar, and the fix's coordinates. CLI-internal: this header lives with the
// command sources and is not installed.
namespace bagwiz::commands
{

class MapBasemap;

// The margin a fitted track keeps from the cell's edges, in pixels of a
// 720-px-tall cell (it scales with the cell), and the least extent a track is
// fitted to on each axis: a vehicle at rest would otherwise have GNSS noise
// scaled up to fill the panel.
inline constexpr double kMapFitMarginPx = 48.0;
inline constexpr double kMapMinExtentM = 20.0;

// The drawn elements are sized for a 720-px-tall cell and scale with the cell
// height, within these bounds, so a 4K camera cell keeps legible markers and
// text.
[[nodiscard]] double map_ui_scale(PanelSize cell) noexcept;

// Fit the whole track into `cell`: its bounding box, at least kMapMinExtentM
// on each axis, centered and uniformly scaled to fit inside the margin.
[[nodiscard]] MapViewport fit_map_viewport(const MapTrack & track, PanelSize cell);

// Follow the fix: it sits at the center, and the shorter cell axis spans
// +-range_m around it.
[[nodiscard]] MapViewport follow_map_viewport(const MapFix & fix, double range_m, PanelSize cell);

// The scale bar's length in meters: the largest 1, 2 or 5 x 10^k that spans
// at most a quarter of the cell width, and at least 1 m.
[[nodiscard]] double map_scale_bar_meters(const MapViewport & viewport);

class MapPanel final : public Panel
{
public:
  struct Options
  {
    MapTrack track;
    // The panel's topic, for log lines.
    std::string topic;
    // Follow the current fix at +-range; unset fits the whole track.
    std::optional<double> follow_range_m;
    // The map drawn under the track, and its provider's attribution line;
    // null draws the bare grid.
    std::shared_ptr<MapBasemap> basemap;
    std::string attribution;
  };

  // Clock role: the ticks are the topic's own messages; the panel sizes its
  // cell by `sizing`.
  MapPanel(Options options, SyntheticSizing sizing);
  // Follower role: the fix nearest each tick by record time.
  explicit MapPanel(Options options);

  [[nodiscard]] std::string select(const TickInfo & tick, PanelSize cell) override;
  [[nodiscard]] std::optional<PanelSize> clock_cell_size() const override;
  [[nodiscard]] std::string render(const CellView & cell) override;

private:
  [[nodiscard]] MapViewport viewport_of(std::size_t fix, PanelSize cell) const;
  // Fetch the tiles of every viewport the run will draw, once the cell size
  // is known; a source that yields nothing drops the basemap for the run.
  void prepare_basemap();

  Options options_;
  std::optional<SyntheticSizing> sizing_;  // clock role only
  // The current tick: the cell size and the fix shown.
  PanelSize size_;
  std::size_t fix_ = 0;
  bool selected_ = false;
  bool basemap_ready_ = false;
};

// Build `movify`'s map panel around the --gnss track the scan loaded, in the
// clock role when the clock is the --gnss topic (validation.clock_gnss),
// over the tiles of args.map_tiles unless that is kMapTilesNone.
[[nodiscard]] std::unique_ptr<Panel> build_map_panel(
  const MovifyArgs & args, const VideoInputValidation & validation, MapTrack track);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_MAP_PANEL_HPP_
