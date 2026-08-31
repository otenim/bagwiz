// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MOVIFY_MAP_VIEWPORT_HPP_
#define COMMANDS__MOVIFY_MAP_VIEWPORT_HPP_

#include "movify_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header

// The map panel's window onto the ENU plane, shared by the panel (which
// places the track in it) and the basemap (which warps the map tiles into
// it). CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// The point at the cell's center and the scale, mapping east to +x (right)
// and north to -y (up).
struct MapViewport
{
  double center_east = 0.0;
  double center_north = 0.0;
  double px_per_m = 1.0;
  PanelSize cell;

  [[nodiscard]] double x_of(double east) const noexcept
  {
    return cell.width / 2.0 + (east - center_east) * px_per_m;
  }
  [[nodiscard]] double y_of(double north) const noexcept
  {
    return cell.height / 2.0 - (north - center_north) * px_per_m;
  }
  // The inverse: the ENU point under a pixel.
  [[nodiscard]] double east_of(double x) const noexcept
  {
    return center_east + (x - cell.width / 2.0) / px_per_m;
  }
  [[nodiscard]] double north_of(double y) const noexcept
  {
    return center_north - (y - cell.height / 2.0) / px_per_m;
  }
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__MOVIFY_MAP_VIEWPORT_HPP_
