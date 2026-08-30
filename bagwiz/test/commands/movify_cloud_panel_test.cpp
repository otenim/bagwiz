// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_cloud_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/pointcloud/cloud_view.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "movify_cloud_source.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_inputs.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_layout.hpp"        // NOLINT(build/include_subdir) src-local shared header
#include "movify_panel.hpp"         // NOLINT(build/include_subdir) src-local shared header
#include "movify_pose_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "movify_test_util.hpp"     // NOLINT(build/include_subdir) src-local shared header

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace
{

using bagwiz::commands::CloudPanel;
using bagwiz::commands::CloudSources;
using bagwiz::commands::GridCanvas;
using bagwiz::commands::GridSpec;
using bagwiz::commands::PanelSize;
using bagwiz::commands::PoseOverlay;
using bagwiz::commands::TickInfo;
using bagwiz::commands::VideoInputScan;
using bagwiz::test::MovifyTmpDirTest;

// A straight drive along +x at 10 m/s, 1 s apart, with no static TF: the
// overlay draws in the body's own frame.
void fill_straight_overlay(PoseOverlay & overlay)
{
  overlay.topic = "/odom";
  overlay.world_frame = "map";
  overlay.body_frame = "base_link";
  overlay.window_s = 5.0;
  for (int i = 0; i <= 5; ++i) {
    bagwiz::core::TrajectoryPose pose;
    pose.timestamp_ns = 1'000'000'000LL + i * 1'000'000'000LL;
    pose.tx = 10.0 * i;
    pose.qw = 1.0;
    overlay.poses.push_back(pose);
  }
}

// A --pose overlay through a bird's-eye panel with no topics of its own: the
// body drives along +x (up in the BEV), so the plates ahead run up the center
// column from the cell's middle — every 2 m, 1.2 m long, 2 m wide — with the
// ground between them left black; and a body frame the static TF cannot
// reach is the panel's error.
TEST_F(MovifyTmpDirTest, PanelDrawsThePoseTrajectoryAsTilesOverTheBev)
{
  PoseOverlay overlay;
  fill_straight_overlay(overlay);
  VideoInputScan scan;
  CloudSources clouds(tmp_dir_ / "unused.mcap", scan, nullptr);
  CloudPanel::Options options;
  options.view.projection = bagwiz::core::pointcloud::CloudProjection::kBev;
  options.view.range_m = 20.0;
  options.frame = "base_link";
  options.pose = &overlay;
  options.pose_width_m = 2.0;
  CloudPanel panel(std::move(options), &clouds);

  constexpr std::uint32_t kW = 100;
  constexpr std::uint32_t kH = 100;
  TickInfo tick;
  tick.record_ns = 1'000'000'000LL;  // the body at x = 0
  ASSERT_EQ(panel.select(tick, PanelSize{kW, kH}), "");
  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(kW, kH);
  canvas.clear();
  ASSERT_EQ(panel.render(canvas.cell(0)), "");
  // +-20 m across 100 px is 2.5 px/m. The plate at 2..3.2 m ahead spans rows
  // 42..45 on the center column: orange at row 43 (x = 2.8 m).
  const auto at = [&](int row, int col) { return canvas.pixels().data() + (row * kW + col) * 3; };
  const auto * plate = at(43, 50);
  EXPECT_GT(static_cast<int>(plate[2]), 100) << "red";  // orange
  EXPECT_EQ(plate[0], std::byte{0}) << "blue";
  // The gap before the next plate (4..5.2 m, rows 37..40): x = 3.6 m is bare.
  const auto * gap = at(41, 50);
  EXPECT_EQ(gap[2], std::byte{0}) << "gap between plates";
  // Across the path the plate is 2 m wide: 1.6 m to the side is bare, as is
  // far off to the side.
  EXPECT_EQ(at(43, 46)[2], std::byte{0}) << "beside the plate";
  EXPECT_EQ(at(43, 20)[2], std::byte{0}) << "off the path";
  // Nothing behind the body at the start of the drive.
  EXPECT_EQ(at(60, 50)[2], std::byte{0}) << "behind the body";

  overlay.body_frame = "elsewhere";
  ASSERT_EQ(panel.select(tick, PanelSize{kW, kH}), "");
  const auto error = panel.render(canvas.cell(0));
  EXPECT_NE(error.find("no static TF chain"), std::string::npos) << error;
}

// The same drive through the perspective (3d) view, mid-way: the plates
// ahead recede into the picture in orange and the ones behind in grey, each
// where the view projects its centre; the ground beside the path stays
// black.
TEST_F(MovifyTmpDirTest, PanelDrawsThePoseTrajectoryAsTilesInThePerspectiveView)
{
  PoseOverlay overlay;
  fill_straight_overlay(overlay);
  VideoInputScan scan;
  CloudSources clouds(tmp_dir_ / "unused.mcap", scan, nullptr);
  CloudPanel::Options options;
  options.view.projection = bagwiz::core::pointcloud::CloudProjection::kPerspective;
  options.view.elev_deg = 20.0;
  options.view.azim_deg = 180.0;
  options.view.dist_m = 30.0;
  options.frame = "base_link";
  options.pose = &overlay;
  options.pose_width_m = 2.0;
  const bagwiz::core::pointcloud::CloudView view_spec = options.view;
  CloudPanel panel(std::move(options), &clouds);

  constexpr std::uint32_t kW = 200;
  constexpr std::uint32_t kH = 200;
  TickInfo tick;
  tick.record_ns = 3'000'000'000LL;  // the body at x = 20, mid-drive
  ASSERT_EQ(panel.select(tick, PanelSize{kW, kH}), "");
  GridCanvas canvas(GridSpec{1, 1});
  canvas.set_cell_size(kW, kH);
  canvas.clear();
  ASSERT_EQ(panel.render(canvas.cell(0)), "");

  bagwiz::core::pointcloud::CloudView view = view_spec;
  view.width = kW;
  view.height = kH;
  const auto pixel_of = [&](double x, double y) {
    const auto projected = bagwiz::core::pointcloud::project_perspective(x, y, 0.0, view);
    EXPECT_TRUE(projected.has_value());
    return canvas.pixels().data() + (projected->v * static_cast<int>(kW) + projected->u) * 3;
  };
  // The centre of the plate 2..3.2 m ahead (body frame) is orange.
  const auto * ahead = pixel_of(2.6, 0.0);
  EXPECT_GT(static_cast<int>(ahead[2]), 100) << "red";
  EXPECT_EQ(ahead[0], std::byte{0}) << "blue";
  // The centre of the plate 2..3.2 m behind is grey: equal channels, lit.
  const auto * behind = pixel_of(-2.6, 0.0);
  EXPECT_GT(static_cast<int>(behind[2]), 60) << "lit";
  EXPECT_EQ(behind[0], behind[1]);
  EXPECT_EQ(behind[1], behind[2]);
  // 5 m beside the path, level with the plate ahead, nothing is drawn.
  const auto * aside = pixel_of(2.6, 5.0);
  EXPECT_EQ(aside[2], std::byte{0});
}

}  // namespace
