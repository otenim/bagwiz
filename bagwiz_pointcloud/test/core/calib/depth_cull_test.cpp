#include "bagwiz/core/calib/depth_cull.hpp"

#include <gtest/gtest.h>

namespace calib = bagwiz::core::calib;

TEST(DepthCullTest, FarPointBehindNearPointInSameCellIsCulled)
{
  std::vector<calib::DepthCullPoint> points{{10.0F, 10.0F, 5.0F}, {12.0F, 12.0F, 20.0F}};
  std::vector<std::uint8_t> keep(points.size());
  calib::depth_cull_keep(points, 640, 480, 8, 0.75F, keep);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 0);
}

TEST(DepthCullTest, CoplanarNeighborWithinMarginSurvives)
{
  std::vector<calib::DepthCullPoint> points{{10.0F, 10.0F, 5.0F}, {12.0F, 12.0F, 5.5F}};
  std::vector<std::uint8_t> keep(points.size());
  calib::depth_cull_keep(points, 640, 480, 8, 0.75F, keep);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 1);
}

TEST(DepthCullTest, DifferentCellsDoNotInteract)
{
  std::vector<calib::DepthCullPoint> points{{10.0F, 10.0F, 5.0F}, {100.0F, 100.0F, 50.0F}};
  std::vector<std::uint8_t> keep(points.size());
  calib::depth_cull_keep(points, 640, 480, 8, 0.75F, keep);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 1);
}

TEST(DepthCullTest, CellScratchIsFilledWithThePointsCellIndex)
{
  // 640px wide image with 8px cells -> 80 cells per row; pixel (10, 10) lands
  // in cell (1, 1) = 81. The scratch field is how pass 2 avoids recomputing
  // the cell, so pin the value the function promises to leave behind.
  std::vector<calib::DepthCullPoint> points{{10.0F, 10.0F, 5.0F}};
  std::vector<std::uint8_t> keep(points.size());
  calib::depth_cull_keep(points, 640, 480, 8, 0.75F, keep);
  EXPECT_EQ(points[0].cell, 81U);
}
