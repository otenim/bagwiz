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

#include <opencv2/core.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <optional>
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
  Options options, SyntheticSizing sizing, std::size_t clock_topic, CloudSources * clouds)
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

// One topic's points for a tick, or the error the panel reports.
struct CloudPanel::TopicProjection
{
  std::vector<core::pointcloud::ProjectedPoint> points;
  std::string error;
};

CloudPanel::TopicProjection CloudPanel::project_topic(
  std::size_t k, const TickInfo & tick, const core::pointcloud::CloudView & view) const
{
  const std::string & topic = options_.topics[k];
  try {
    return project_topic_unguarded(k, tick, view);
  } catch (const std::exception & e) {
    TopicProjection out;
    out.error = on_topic(topic, std::string("projection failed: ") + e.what());
    return out;
  }
}

CloudPanel::TopicProjection CloudPanel::project_topic_unguarded(
  std::size_t k, const TickInfo & tick, const core::pointcloud::CloudView & view) const
{
  TopicProjection out;
  const std::string & topic = options_.topics[k];
  std::shared_ptr<const core::pointcloud::PointCloud2> cloud;
  std::string error;
  if (clock_topic_.has_value() && *clock_topic_ == k) {
    // The clock's own message arrives with the tick.
    auto parsed = core::pointcloud::parse_pointcloud2(tick.payload);
    if (!parsed.ok()) {
      out.error = on_topic(topic, parsed.error);
      return out;
    }
    cloud = std::make_shared<const core::pointcloud::PointCloud2>(std::move(*parsed.cloud));
  } else {
    // Every other topic follows the tick by bag record time, like a camera
    // panel that is not the clock.
    cloud = clouds_->fetch(
      options_.cloud_indexes[k], tick.record_ns, core::pointcloud::PointCloudMatchKey::kRecordTime,
      error);
    if (!cloud) {
      out.error = on_topic(topic, error);
      return out;
    }
  }

  if (k == 0 && cloud->timestamp_ns != 0) {
    stamp_ns_ = cloud->timestamp_ns;  // the tick's capture time, for the overlay
  }
  // Move the cloud into the view frame at its own capture time, so a cloud
  // published in a moving frame lands where that frame was when the sweep
  // was taken.
  core::pointcloud::RigidTransform transform;
  if (!options_.frame.empty() && cloud->frame_id != options_.frame) {
    if (clouds_->tf_buffer() == nullptr) {
      out.error = on_topic(
        topic, "cannot transform " + cloud->frame_id + " -> " + options_.frame +
                 ": the bag carries no TF");
      return out;
    }
    const auto looked_up = core::pointcloud::lookup_rigid_transform(
      *clouds_->tf_buffer(), options_.frame, cloud->frame_id, cloud->timestamp_ns, error);
    if (!looked_up.has_value()) {
      out.error = on_topic(topic, error);
      return out;
    }
    transform = *looked_up;
  }

  auto projected =
    core::pointcloud::project_cloud_to_view(*cloud, transform, view, options_.property);
  if (!projected.ok()) {
    out.error = on_topic(topic, projected.error);
    return out;
  }
  out.points = std::move(projected.points);
  return out;
}

std::string CloudPanel::select(const TickInfo & tick, PanelSize cell)
{
  selected_ = false;
  points_.clear();
  size_ = sizing_.has_value() ? synthetic_clock_cell(*sizing_) : cell;
  if (size_.width == 0 || size_.height == 0) {
    return "the point-cloud panel has no cell to render into";
  }
  core::pointcloud::CloudView view = options_.view;
  view.width = size_.width;
  view.height = size_.height;
  view_ = view;
  stamp_ns_ = tick.record_ns;

  // Each topic's fetch, transform and projection is independent of the
  // others' (the sources serialize per topic, TF lookups behind their own
  // mutex), so the topics past the first run on their own threads while this
  // one does the first; the points merge in topic order, though the depth
  // test makes the raster the same either way.
  const std::size_t topics = options_.cloud_indexes.size();
  std::vector<std::future<TopicProjection>> pending;
  pending.reserve(topics > 0 ? topics - 1 : 0);
  for (std::size_t k = 1; k < topics; ++k) {
    pending.push_back(std::async(std::launch::async, [this, k, &tick, &view] {
      return project_topic(k, tick, view);
    }));
  }
  std::string first_error;
  if (topics > 0) {
    TopicProjection first = project_topic(0, tick, view);
    if (!first.error.empty()) {
      first_error = std::move(first.error);
    } else {
      points_ = std::move(first.points);
    }
  }
  for (auto & job : pending) {
    TopicProjection projection = job.get();  // every job is collected, error or not
    if (!projection.error.empty()) {
      if (first_error.empty()) {
        first_error = std::move(projection.error);
      }
      continue;
    }
    points_.insert(points_.end(), projection.points.begin(), projection.points.end());
  }
  if (!first_error.empty()) {
    points_.clear();
    return first_error;
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
  if (options_.pose != nullptr) {
    return draw_pose(cell);
  }
  return "";
}

std::string CloudPanel::draw_pose(const CellView & cell) const
{
  std::string error;
  const auto tiles =
    pose_tiles_in_frame(*options_.pose, options_.frame, stamp_ns_, options_.pose_width_m, error);
  if (!tiles.has_value()) {
    return on_topic(
      options_.topics.empty() ? std::string{"point-cloud panel"} : options_.topics.front(), error);
  }
  const PoseTilePlacement placement{cell.width, cell.height, 0, 0};
  const auto projected = project_pose_tiles(*tiles, corner_projector(), placement);
  cv::Mat canvas(
    static_cast<int>(cell.height), static_cast<int>(cell.width), CV_8UC3,
    static_cast<void *>(cell.data), cell.stride);
  draw_pose_tiles(canvas, projected, pose_ui_scale(cell.height));  // sorts far to near itself
  return "";
}

PoseCornerProjector CloudPanel::corner_projector() const
{
  if (view_.projection == core::pointcloud::CloudProjection::kPerspective) {
    // The virtual camera's depth orders the plates; a corner at or behind
    // the camera does not project.
    const core::pointcloud::PerspectiveCamera camera =
      core::pointcloud::make_perspective_camera(view_);
    return [camera,
            view = view_](const std::array<double, 3> & p) -> std::optional<ProjectedPoseCorner> {
      const auto projected = core::pointcloud::project_perspective(p[0], p[1], p[2], camera, view);
      if (!projected.has_value()) {
        return std::nullopt;
      }
      return ProjectedPoseCorner{
        static_cast<double>(projected->u), static_cast<double>(projected->v), projected->depth};
    };
  }
  // The BEV places every corner (one off the canvas just clips) and has no
  // depth of its own: the corner's ground distance from the view's origin
  // stands in, so where the path crosses itself the plate nearer the body
  // lies on top.
  return [view = view_](const std::array<double, 3> & p) -> std::optional<ProjectedPoseCorner> {
    const auto projected = core::pointcloud::project_bev(p[0], p[1], view);
    return ProjectedPoseCorner{
      static_cast<double>(projected.u), static_cast<double>(projected.v), std::hypot(p[0], p[1])};
  };
}

std::optional<std::vector<std::unique_ptr<Panel>>> build_cloud_panels(
  const MovifyArgs & args, const VideoInputValidation & validation, const VideoInputScan & scan,
  CloudSources & clouds, const PoseOverlay * pose)
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
  options.pose = pose;
  options.pose_width_m = args.pose_width_m;
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
      SyntheticSizing sizing;
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
