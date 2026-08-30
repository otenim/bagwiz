// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_map_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{
// Colors, BGR.
const cv::Scalar kGridColor(40, 40, 40);
const cv::Scalar kTrackColor(96, 96, 96);
const cv::Scalar kTraveledColor(0, 170, 255);  // orange
const cv::Scalar kMarkerColor(255, 255, 255);
const cv::Scalar kTextColor(230, 230, 230);
constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;

// The bounds map_ui_scale() keeps the element sizes within.
constexpr double kMinUiScale = 0.5;
constexpr double kMaxUiScale = 4.0;

// Pixel coordinates handed to OpenCV are clamped to this magnitude: a fix far
// outside a follow view must clip, not overflow the int conversion.
constexpr double kMaxPixelCoord = 1.0e7;

int px(double v)
{
  return static_cast<int>(std::lround(std::clamp(v, -kMaxPixelCoord, kMaxPixelCoord)));
}

int thickness(double v)
{
  return std::max(1, px(v));
}

cv::Point pixel_of(const MapViewport & view, const MapFix & fix)
{
  return {px(view.x_of(fix.east)), px(view.y_of(fix.north))};
}

std::string format_meters(double meters)
{
  return meters >= 1000.0 ? fmt::format("{:g} km", meters / 1000.0) : fmt::format("{:g} m", meters);
}

// Grid lines at every scale-bar length, on world-coordinate multiples of it,
// so the grid keeps still while a follow view pans.
void draw_grid(cv::Mat & canvas, const MapViewport & view, double spacing_m)
{
  const double half_w = view.cell.width / (2.0 * view.px_per_m);
  const double half_h = view.cell.height / (2.0 * view.px_per_m);
  const double first_east = std::floor((view.center_east - half_w) / spacing_m) * spacing_m;
  const double first_north = std::floor((view.center_north - half_h) / spacing_m) * spacing_m;
  for (double east = first_east; east <= view.center_east + half_w; east += spacing_m) {
    const int x = px(view.x_of(east));
    cv::line(canvas, {x, 0}, {x, static_cast<int>(view.cell.height)}, kGridColor, 1);
  }
  for (double north = first_north; north <= view.center_north + half_h; north += spacing_m) {
    const int y = px(view.y_of(north));
    cv::line(canvas, {0, y}, {static_cast<int>(view.cell.width), y}, kGridColor, 1);
  }
}

// The whole track dimmed, then the part driven up to the current fix on top.
void draw_track(
  cv::Mat & canvas, const MapViewport & view, const MapTrack & track, std::size_t current,
  double scale)
{
  std::vector<cv::Point> points;
  points.reserve(track.fixes.size());
  for (const auto & fix : track.fixes) {
    points.push_back(pixel_of(view, fix));
  }
  cv::polylines(canvas, points, false, kTrackColor, thickness(2.0 * scale), cv::LINE_AA);
  const std::vector<cv::Point> traveled(points.begin(), points.begin() + current + 1);
  cv::polylines(canvas, traveled, false, kTraveledColor, thickness(3.0 * scale), cv::LINE_AA);
}

void draw_marker(
  cv::Mat & canvas, const cv::Point & at, std::optional<double> heading, double scale)
{
  if (heading.has_value()) {
    const double length = 26.0 * scale;
    const cv::Point tip(
      at.x + px(length * std::cos(*heading)), at.y - px(length * std::sin(*heading)));
    cv::arrowedLine(canvas, at, tip, kTraveledColor, thickness(3.0 * scale), cv::LINE_AA, 0, 0.35);
  }
  const int radius = std::max(3, px(7.0 * scale));
  cv::circle(canvas, at, radius, kMarkerColor, cv::FILLED, cv::LINE_AA);
  cv::circle(canvas, at, radius, kTraveledColor, thickness(2.0 * scale), cv::LINE_AA);
}

void draw_north_arrow(cv::Mat & canvas, PanelSize cell, double scale)
{
  const int margin = px(28.0 * scale);
  const int length = px(44.0 * scale);
  const cv::Point base(static_cast<int>(cell.width) - margin, margin + length);
  const cv::Point tip(base.x, margin);
  cv::arrowedLine(canvas, base, tip, kMarkerColor, thickness(2.0 * scale), cv::LINE_AA, 0, 0.3);
  cv::putText(
    canvas, "N", {base.x - px(7.0 * scale), base.y + px(20.0 * scale)}, kFont, 0.6 * scale,
    kMarkerColor, thickness(1.5 * scale), cv::LINE_AA);
}

void draw_scale_bar(cv::Mat & canvas, const MapViewport & view, double meters, double scale)
{
  const int margin = px(24.0 * scale);
  const int y = static_cast<int>(view.cell.height) - margin;
  const int x0 = margin;
  const int x1 = x0 + px(meters * view.px_per_m);
  const int tick = px(6.0 * scale);
  cv::line(canvas, {x0, y}, {x1, y}, kMarkerColor, thickness(3.0 * scale), cv::LINE_AA);
  cv::line(canvas, {x0, y - tick}, {x0, y + tick}, kMarkerColor, thickness(2.0 * scale));
  cv::line(canvas, {x1, y - tick}, {x1, y + tick}, kMarkerColor, thickness(2.0 * scale));
  cv::putText(
    canvas, format_meters(meters), {x0, y - px(10.0 * scale)}, kFont, 0.5 * scale, kTextColor,
    thickness(1.2 * scale), cv::LINE_AA);
}

void draw_readout(cv::Mat & canvas, const MapFix & fix, double scale)
{
  const auto position = fmt::format("lat {:.6f}  lon {:.6f}", fix.latitude, fix.longitude);
  const auto altitude =
    std::isfinite(fix.altitude) ? fmt::format("alt {:.1f} m", fix.altitude) : std::string{"alt -"};
  cv::putText(
    canvas, position, {px(16.0 * scale), px(30.0 * scale)}, kFont, 0.55 * scale, kTextColor,
    thickness(1.2 * scale), cv::LINE_AA);
  cv::putText(
    canvas, altitude, {px(16.0 * scale), px(54.0 * scale)}, kFont, 0.55 * scale, kTextColor,
    thickness(1.2 * scale), cv::LINE_AA);
}
}  // namespace

double map_ui_scale(PanelSize cell) noexcept
{
  return std::clamp(
    cell.height / static_cast<double>(kSyntheticCellHeight), kMinUiScale, kMaxUiScale);
}

MapViewport fit_map_viewport(const MapTrack & track, PanelSize cell)
{
  MapViewport view;
  view.cell = cell;
  view.center_east = (track.min_east + track.max_east) / 2.0;
  view.center_north = (track.min_north + track.max_north) / 2.0;
  const double extent_east = std::max(track.max_east - track.min_east, kMapMinExtentM);
  const double extent_north = std::max(track.max_north - track.min_north, kMapMinExtentM);
  const double margin = kMapFitMarginPx * map_ui_scale(cell);
  const double usable_w = std::max(cell.width - 2.0 * margin, 1.0);
  const double usable_h = std::max(cell.height - 2.0 * margin, 1.0);
  view.px_per_m = std::min(usable_w / extent_east, usable_h / extent_north);
  return view;
}

MapViewport follow_map_viewport(const MapFix & fix, double range_m, PanelSize cell)
{
  MapViewport view;
  view.cell = cell;
  view.center_east = fix.east;
  view.center_north = fix.north;
  view.px_per_m = std::min(cell.width, cell.height) / (2.0 * range_m);
  return view;
}

double map_scale_bar_meters(const MapViewport & viewport)
{
  const double longest_m = viewport.cell.width / (4.0 * viewport.px_per_m);
  if (!(longest_m >= 1.0)) {
    return 1.0;
  }
  const double magnitude = std::pow(10.0, std::floor(std::log10(longest_m)));
  for (const double factor : {5.0, 2.0}) {
    if (factor * magnitude <= longest_m) {
      return factor * magnitude;
    }
  }
  return magnitude;
}

MapPanel::MapPanel(Options options, SyntheticSizing sizing)
: options_(std::move(options)), sizing_(sizing)
{
}

MapPanel::MapPanel(Options options) : options_(std::move(options))
{
}

std::string MapPanel::select(const TickInfo & tick, PanelSize cell)
{
  selected_ = false;
  size_ = sizing_.has_value() ? synthetic_clock_cell(*sizing_) : cell;
  if (size_.width == 0 || size_.height == 0) {
    return "topic '" + options_.topic + "': the map panel has no cell to render into";
  }
  fix_ = nearest_map_fix(options_.track, tick.record_ns);
  selected_ = true;
  return "";
}

std::optional<PanelSize> MapPanel::clock_cell_size() const
{
  if (!sizing_.has_value() || !selected_) {
    return std::nullopt;
  }
  return size_;
}

std::string MapPanel::render(const CellView & cell)
{
  if (!selected_) {
    return "";  // nothing to show: the cell was cleared to black
  }
  cv::Mat canvas(
    static_cast<int>(cell.height), static_cast<int>(cell.width), CV_8UC3,
    static_cast<void *>(cell.data), cell.stride);
  const PanelSize size{cell.width, cell.height};
  const double scale = map_ui_scale(size);
  const MapFix & fix = options_.track.fixes[fix_];
  const MapViewport view = options_.follow_range_m.has_value()
                             ? follow_map_viewport(fix, *options_.follow_range_m, size)
                             : fit_map_viewport(options_.track, size);
  const double bar_m = map_scale_bar_meters(view);
  draw_grid(canvas, view, bar_m);
  draw_track(canvas, view, options_.track, fix_, scale);
  draw_marker(canvas, pixel_of(view, fix), map_heading_at(options_.track, fix_), scale);
  draw_north_arrow(canvas, size, scale);
  draw_scale_bar(canvas, view, bar_m, scale);
  draw_readout(canvas, fix, scale);
  return "";
}

std::unique_ptr<Panel> build_map_panel(
  const MovifyArgs & args, const VideoInputValidation & validation, MapTrack track)
{
  MapPanel::Options options;
  options.track = std::move(track);
  options.topic = validation.gnss_topic.value_or("");
  options.follow_range_m = args.map_range_m;
  if (validation.clock_gnss) {
    SyntheticSizing sizing;
    sizing.total_width = args.width;
    sizing.grid_cols = validation.grid.cols;
    return std::make_unique<MapPanel>(std::move(options), sizing);
  }
  return std::make_unique<MapPanel>(std::move(options));
}

}  // namespace bagwiz::commands
