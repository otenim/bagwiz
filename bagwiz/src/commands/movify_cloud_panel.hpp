// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_CLOUD_PANEL_HPP_
#define COMMANDS__MOVIFY_CLOUD_PANEL_HPP_

#include "bagwiz/commands/movify.hpp"
#include "bagwiz/core/pointcloud/cloud_view.hpp"
#include "bagwiz/core/pointcloud/color_mapper.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/point_raster.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "movify_cloud_source.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_inputs.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_layout.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"         // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// The panel that draws point clouds: every one of its topics is fetched at
// each tick, moved into one common frame, projected by a CloudView (a
// bird's-eye or a perspective view) and rasterized with a depth test into
// the panel's cell. CLI-internal: this header lives with the command sources
// and is not installed.
namespace bagwiz::commands
{

// The cell a point-cloud clock panel renders into when no camera fixes the
// size: 16:9 at 1280x720, or the output width split across the grid columns.
inline constexpr std::uint32_t kCloudPanelDefaultWidth = 1280;
inline constexpr std::uint32_t kCloudPanelDefaultHeight = 720;

class CloudPanel final : public Panel
{
public:
  struct Options
  {
    // The projection and its camera / extent; the canvas size is set per
    // tick from the cell.
    core::pointcloud::CloudView view;
    // The frame every cloud is transformed into before projection (the view
    // frame). A cloud already in this frame is drawn as is.
    std::string frame;
    core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
    core::pointcloud::ColorScheme scheme = core::pointcloud::ColorScheme::kViridis;
    double value_min = 0.0;
    double value_max = 0.0;
    std::uint32_t point_size = 2;
    // The panel's topics as indexes into the CloudSources, with their names
    // for log lines (parallel vectors).
    std::vector<std::size_t> cloud_indexes;
    std::vector<std::string> topics;
  };

  // The clock role's cell-size rule: the default cell, or the output width
  // split across the grid columns at the default aspect ratio.
  struct ClockSizing
  {
    std::optional<std::uint32_t> total_width;
    std::uint32_t grid_cols = 1;
  };

  // Clock role: each tick's payload is a message of the panel's topic at
  // position `clock_topic` in `options.topics`; the other topics are fetched.
  CloudPanel(Options options, ClockSizing sizing, std::size_t clock_topic, CloudSources * clouds);
  // Follower role: every topic is fetched nearest the tick by record time.
  CloudPanel(Options options, CloudSources * clouds);

  [[nodiscard]] std::string select(const TickInfo & tick, PanelSize cell) override;
  [[nodiscard]] std::optional<PanelSize> clock_cell_size() const override;
  [[nodiscard]] std::string render(const CellView & cell) override;

private:
  [[nodiscard]] PanelSize clock_size() const;

  Options options_;
  std::optional<ClockSizing> sizing_;       // clock role only
  std::optional<std::size_t> clock_topic_;  // clock role only
  CloudSources * clouds_;
  core::pointcloud::ColorMapper mapper_;
  core::pointcloud::PointRaster raster_;
  // The current tick: the canvas size and every topic's projected points.
  PanelSize size_;
  std::vector<core::pointcloud::ProjectedPoint> points_;
  bool selected_ = false;
};

// Build `movify`'s point-cloud panels, one per requested view in `--view`
// order, each drawing every `--pcd` topic. The first of them takes the clock
// role when the clock is a point-cloud topic (validation.clock_pcd). Returns
// nullopt after logging when a topic is missing from the scan.
[[nodiscard]] std::optional<std::vector<std::unique_ptr<Panel>>> build_cloud_panels(
  const MovifyArgs & args, const VideoInputValidation & validation, const VideoInputScan & scan,
  CloudSources & clouds);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_CLOUD_PANEL_HPP_
