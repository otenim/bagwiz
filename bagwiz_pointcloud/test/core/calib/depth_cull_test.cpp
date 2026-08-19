#include "bagwiz/core/calib/depth_cull.hpp"

#include <gtest/gtest.h>

namespace calib = bagwiz::core::calib;

TEST(DepthCullTest, FarPointBehindNearPointInSameCellIsCulled)
{
  const std::vector<float> u{10.0F, 12.0F};
  const std::vector<float> v{10.0F, 12.0F};
  const std::vector<float> depth{5.0F, 20.0F};
  const auto keep = calib::depth_cull_keep(u, v, depth, 640, 480, 8, 0.75F);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 0);
}

TEST(DepthCullTest, CoplanarNeighborWithinMarginSurvives)
{
  const std::vector<float> u{10.0F, 12.0F};
  const std::vector<float> v{10.0F, 12.0F};
  const std::vector<float> depth{5.0F, 5.5F};
  const auto keep = calib::depth_cull_keep(u, v, depth, 640, 480, 8, 0.75F);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 1);
}

TEST(DepthCullTest, DifferentCellsDoNotInteract)
{
  const std::vector<float> u{10.0F, 100.0F};
  const std::vector<float> v{10.0F, 100.0F};
  const std::vector<float> depth{5.0F, 50.0F};
  const auto keep = calib::depth_cull_keep(u, v, depth, 640, 480, 8, 0.75F);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(keep[1], 1);
}
