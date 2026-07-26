#include <cstdint>

#include "gtest/gtest.h"

#include "lunar_rover_navigation/terrain_cost.hpp"

namespace lunar_rover_navigation
{
namespace
{

TEST(TerrainCost, PreservesFreeSpace)
{
  EXPECT_EQ(terrainValueToCost(0, TerrainCostParameters{}), kFreeSpace);
}

TEST(TerrainCost, ScalesCautionCost)
{
  const auto cost = terrainValueToCost(70, TerrainCostParameters{});
  EXPECT_GT(cost, kFreeSpace);
  EXPECT_LT(cost, kInscribedInflatedObstacle);
}

TEST(TerrainCost, MarksThresholdAndAboveLethal)
{
  TerrainCostParameters parameters;
  parameters.lethal_threshold = 100U;
  EXPECT_EQ(terrainValueToCost(99, parameters) < kLethalObstacle, true);
  EXPECT_EQ(terrainValueToCost(100, parameters), kLethalObstacle);
}

TEST(TerrainCost, UnknownPolicyIsExplicit)
{
  TerrainCostParameters conservative;
  conservative.unknown_is_lethal = true;
  EXPECT_EQ(terrainValueToCost(-1, conservative), kLethalObstacle);

  TerrainCostParameters permissive;
  permissive.unknown_is_lethal = false;
  EXPECT_EQ(terrainValueToCost(-1, permissive), kNoInformation);
}

}  // namespace
}  // namespace lunar_rover_navigation
