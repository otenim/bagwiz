// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_cloud_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.movify";

// A panel failure, attributed to the topic for the loop's log line.
std::string on_topic(const std::string & topic, const std::string & error)
{
  return "topic '" + topic + "': " + error;
}
}  // namespace

CloudPanel::CloudPanel(
  Options options, ClockSizing sizing, std::size_t clock_topic, CloudSources * clouds)
: options_(std::move(options)),
  sizing_(sizing),
  clock_topic_(clock_topic),
  clouds_(clouds),
  mapper_(options_.scheme)
{
}

CloudPanel::CloudPanel(Options options, CloudSources * clouds)
: options_(std::move(options)), clouds_(clouds), mapper_(options_.scheme)
{
}

PanelSize CloudPanel::clock_size() const
{
  if (sizing_.has_value() && sizing_->total_width.has_value()) {
    // --width: the cell width is the width split across the grid columns,
    // the height keeps the default aspect ratio; both rounded down to even
    // (the codecs' 4:2:0 formats require even dimensions).
    const std::uint32_t cell_w = (*sizing_->total_width / sizing_->grid_cols) & ~1U;
    const auto cell_h =
      static_cast<std::uint32_t>(std::lround(
        cell_w * (static_cast<double>(kCloudPanelDefaultHeight) / kCloudPanelDefaultWidth))) &
      ~1U;
    return PanelSize{cell_w, cell_h};
  }
  return PanelSize{kCloudPanelDefaultWidth, kCloudPanelDefaultHeight};
}

std::string CloudPanel::select(const TickInfo & tick, PanelSize cell)
{
  selected_ = false;
  points_.clear();
  size_ = clock_topic_.has_value() ? clock_size() : cell;
  if (size_.width == 0 || size_.height == 0) {
    return "the point-cloud panel has no cell to render into";
  }
  core::pointcloud::CloudView view = options_.view;
  view.width = size_.width;
  view.height = size_.height;

  for (std::size_t k = 0; k < options_.cloud_indexes.size(); ++k) {
    const std::string & topic = options_.topics[k];
    std::shared_ptr<const core::pointcloud::PointCloud2> cloud;
    std::string error;
    if (clock_topic_.has_value() && *clock_topic_ == k) {
      // The clock's own message arrives with the tick.
      auto parsed = core::pointcloud::parse_pointcloud2(tick.payload);
      if (!parsed.ok()) {
        return on_topic(topic, parsed.error);
      }
      cloud = std::make_shared<const core::pointcloud::PointCloud2>(std::move(*parsed.cloud));
    } else {
      // Every other topic follows the tick by bag record time, like a camera
      // panel that is not the clock.
      cloud = clouds_->fetch(
        options_.cloud_indexes[k], tick.record_ns,
        core::pointcloud::PointCloudMatchKey::kRecordTime, error);
      if (!cloud) {
        return on_topic(topic, error);
      }
    }

    // Move the cloud into the view frame at its own capture time, so a cloud
    // published in a moving frame lands where that frame was when the sweep
    // was taken.
    core::pointcloud::RigidTransform transform;
    if (!options_.frame.empty() && cloud->frame_id != options_.frame) {
      if (clouds_->tf_buffer() == nullptr) {
        return on_topic(
          topic, "cannot transform " + cloud->frame_id + " -> " + options_.frame +
                   ": the bag carries no TF");
      }
      const auto looked_up = core::pointcloud::lookup_rigid_transform(
        *clouds_->tf_buffer(), options_.frame, cloud->frame_id, cloud->timestamp_ns, error);
      if (!looked_up.has_value()) {
        return on_topic(topic, error);
      }
      transform = *looked_up;
    }

    auto projected =
      core::pointcloud::project_cloud_to_view(*cloud, transform, view, options_.property);
    if (!projected.ok()) {
      return on_topic(topic, projected.error);
    }
    points_.insert(points_.end(), projected.points.begin(), projected.points.end());
  }
  selected_ = true;
  return "";
}

std::optional<PanelSize> CloudPanel::clock_cell_size() const
{
  if (!clock_topic_.has_value() || !selected_) {
    return std::nullopt;
  }
  return size_;
}

std::string CloudPanel::render(const CellView & cell)
{
  if (!selected_) {
    return "";  // nothing to show: the cell was cleared to black
  }
  raster_.reset(cell.width, cell.height);
  raster_.draw(points_, mapper_, options_.value_min, options_.value_max, options_.point_size);
  // The raster is exactly the cell; copy it row by row into the composed frame.
  const std::size_t row_bytes = static_cast<std::size_t>(cell.width) * 3U;
  for (std::uint32_t y = 0; y < cell.height; ++y) {
    std::copy_n(
      raster_.bgr().data() + static_cast<std::size_t>(y) * row_bytes, row_bytes,
      cell.data + static_cast<std::size_t>(y) * cell.stride);
  }
  return "";
}

std::optional<std::vector<std::unique_ptr<Panel>>> build_cloud_panels(
  const MovifyArgs & args, const VideoInputValidation & validation, const VideoInputScan & scan,
  CloudSources & clouds)
{
  std::vector<std::unique_ptr<Panel>> panels;
  if (validation.pcd_topics.empty()) {
    return panels;
  }

  CloudPanel::Options options;
  options.view.elev_deg = args.elev_deg;
  options.view.azim_deg = args.azim_deg;
  options.view.dist_m = args.dist_m;
  options.view.range_m = validation.range_m;
  options.frame = validation.frame;
  options.property = args.property;
  options.scheme = args.colorscheme;
  options.value_min = scan.global_property_min;
  options.value_max = scan.global_property_max;
  options.point_size = args.point_size;
  for (const auto & topic : validation.pcd_topics) {
    const auto it = std::find(scan.pcd_topics.begin(), scan.pcd_topics.end(), topic);
    if (it == scan.pcd_topics.end()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "internal error — pcd topic '%s' missing from the scan", topic.c_str());
      return std::nullopt;
    }
    options.cloud_indexes.push_back(static_cast<std::size_t>(it - scan.pcd_topics.begin()));
    options.topics.push_back(topic);
  }

  panels.reserve(validation.pcd_views.size());
  for (std::size_t i = 0; i < validation.pcd_views.size(); ++i) {
    CloudPanel::Options panel_options = options;
    panel_options.view.projection = validation.pcd_views[i];
    if (i == 0 && validation.clock_pcd.has_value()) {
      CloudPanel::ClockSizing sizing;
      sizing.total_width = args.width;
      sizing.grid_cols = validation.grid.cols;
      panels.push_back(
        std::make_unique<CloudPanel>(
          std::move(panel_options), sizing, *validation.clock_pcd, &clouds));
    } else {
      panels.push_back(std::make_unique<CloudPanel>(std::move(panel_options), &clouds));
    }
  }
  return panels;
}

}  // namespace bagwiz::commands
