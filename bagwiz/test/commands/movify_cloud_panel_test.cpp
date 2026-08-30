// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "movify_cloud_panel.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "bagwiz/core/pointcloud/cloud_view.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
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

// A --pose overlay through a bird's-eye panel with no topics of its own: the
// body drives along +x (up in the BEV), so the path ahead runs up the center
// column from the cell's middle, and a body frame the static TF cannot reach
// is the panel's error.
TEST_F(MovifyTmpDirTest, PanelDrawsThePoseTrajectoryOverTheBev)
{
  PoseOverlay overlay;
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

  VideoInputScan scan;
  CloudSources clouds(tmp_dir_ / "unused.mcap", scan, nullptr);
  CloudPanel::Options options;
  options.view.projection = bagwiz::core::pointcloud::CloudProjection::kBev;
  options.view.range_m = 20.0;
  options.frame = "base_link";
  options.pose = &overlay;
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
  // +-20 m across 100 px: 12.5 m ahead is 31 px above the center.
  const auto * ahead = canvas.pixels().data() + (19 * kW + 50) * 3;
  EXPECT_GT(static_cast<int>(ahead[2]), 100);  // orange
  const auto * aside = canvas.pixels().data() + (19 * kW + 20) * 3;
  EXPECT_EQ(aside[2], std::byte{0});  // nothing beside the path

  overlay.body_frame = "elsewhere";
  ASSERT_EQ(panel.select(tick, PanelSize{kW, kH}), "");
  const auto error = panel.render(canvas.cell(0));
  EXPECT_NE(error.find("no static TF chain"), std::string::npos) << error;
}

// The panel's styling defaults mirror the CLI's: jet is the colour scheme every
// bagwiz visualization starts from.
TEST(CloudPanelOptionsDefaults, SchemeIsJet)
{
  const CloudPanel::Options options;
  EXPECT_EQ(options.scheme, bagwiz::core::pointcloud::ColorScheme::kJet);
}

}  // namespace
