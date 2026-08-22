#include "bagwiz/core/calib/depth_cull.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

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

TEST(DepthCullTest, GridObservesBatchesInAnyOrder)
{
  // The nearest depth per cell is a min over the points that land in it, so
  // observing the points as two batches, in either order, keeps exactly what
  // one batch keeps.
  std::vector<calib::DepthCullPoint> pts{{4.0F, 4.0F, 10.0F, 0},  {5.0F, 5.0F, 2.0F, 0},
                                         {20.0F, 4.0F, 5.0F, 0},  {21.0F, 5.0F, 5.5F, 0},
                                         {40.0F, 40.0F, 1.0F, 0}, {41.0F, 41.0F, 30.0F, 0}};
  constexpr std::uint32_t kW = 64;
  constexpr std::uint32_t kH = 64;
  constexpr std::uint32_t kCell = 8;
  constexpr float kMargin = 0.75F;
  std::vector<std::uint8_t> reference(pts.size());
  calib::depth_cull_keep(pts, kW, kH, kCell, kMargin, reference);

  for (const bool reversed : {false, true}) {
    calib::DepthCullGrid grid;
    grid.reset(kW, kH, kCell);
    const std::span<calib::DepthCullPoint> first(pts.data(), 3);
    const std::span<calib::DepthCullPoint> second(pts.data() + 3, 3);
    grid.observe(reversed ? second : first);
    grid.observe(reversed ? first : second);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      EXPECT_EQ(grid.keeps(pts[i], kMargin) ? 1 : 0, reference[i]) << i;
    }
  }
}

TEST(DepthCullTest, MergedRangeGridsKeepWhatOneGridKeeps)
{
  // Two halves observed into two grids and merged give every point the
  // verdict one grid over both halves gives: the per-cell nearest depth is a
  // min, so it does not matter which grid saw which point.
  std::vector<calib::DepthCullPoint> pts{{4.0F, 4.0F, 10.0F, 0},  {5.0F, 5.0F, 2.0F, 0},
                                         {20.0F, 4.0F, 5.0F, 0},  {21.0F, 5.0F, 5.5F, 0},
                                         {40.0F, 40.0F, 1.0F, 0}, {41.0F, 41.0F, 30.0F, 0}};
  constexpr std::uint32_t kW = 64;
  constexpr std::uint32_t kH = 64;
  constexpr std::uint32_t kCell = 8;
  constexpr float kMargin = 0.75F;
  std::vector<std::uint8_t> reference(pts.size());
  calib::depth_cull_keep(pts, kW, kH, kCell, kMargin, reference);

  calib::DepthCullGrid first;
  calib::DepthCullGrid second;
  first.reset(kW, kH, kCell);
  second.reset(kW, kH, kCell);
  first.observe(std::span<calib::DepthCullPoint>(pts.data(), 3));
  second.observe(std::span<calib::DepthCullPoint>(pts.data() + 3, 3));
  calib::DepthCullGrid merged;
  merged.reset(kW, kH, kCell);
  merged.merge(second);
  merged.merge(first);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_EQ(merged.keeps(pts[i], kMargin) ? 1 : 0, reference[i]) << i;
  }
}
