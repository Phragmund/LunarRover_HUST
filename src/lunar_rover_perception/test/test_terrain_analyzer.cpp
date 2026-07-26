#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "lunar_rover_perception/terrain_analyzer.hpp"

namespace lunar_rover_perception
{
namespace
{

TerrainParameters testParameters()
{
  TerrainParameters parameters;
  parameters.resolution = 0.1;
  parameters.length_x = 0.4;
  parameters.length_y = 0.4;
  parameters.max_point_age = 1.0;
  parameters.required_point_weight = 3.0;
  return parameters;
}

void addSurface(
  TerrainAnalyzer & analyzer,
  const std::function<double(double, double)> & height)
{
  std::vector<TerrainPoint> points;
  for (std::size_t y = 0; y < 4U; ++y) {
    for (std::size_t x = 0; x < 4U; ++x) {
      const double px = (static_cast<double>(x) + 0.5) * 0.1;
      const double py = (static_cast<double>(y) + 0.5) * 0.1;
      for (int sample = 0; sample < 4; ++sample) {
        points.push_back(TerrainPoint{
            px, py, height(px, py), 1.0, PointSource::kLidar});
      }
    }
  }
  analyzer.addPoints(points, 0.1);
}

TEST(TerrainAnalyzer, FlatSurfaceIsTraversable)
{
  TerrainAnalyzer analyzer(testParameters());
  addSurface(analyzer, [](double, double) {return 0.0;});

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 0.1);
  const auto & cell = snapshot.at(1U, 1U);
  EXPECT_TRUE(cell.observed);
  EXPECT_NEAR(cell.slope, 0.0, 1.0e-9);
  EXPECT_NEAR(cell.roughness, 0.0, 1.0e-9);
  EXPECT_EQ(cell.traversability, 0);
  EXPECT_FALSE(cell.lethal);
}

TEST(TerrainAnalyzer, StepAboveSevenCentimetersIsLethal)
{
  TerrainAnalyzer analyzer(testParameters());
  addSurface(
    analyzer, [](double x, double) {
      return x < 0.2 ? 0.0 : 0.08;
    });

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 0.1);
  const auto & cell = snapshot.at(1U, 1U);
  EXPECT_GT(cell.step, 0.07);
  EXPECT_EQ(cell.traversability, 100);
  EXPECT_TRUE(cell.lethal);
}

TEST(TerrainAnalyzer, FiveCentimeterStepProducesSlowdownCost)
{
  TerrainAnalyzer analyzer(testParameters());
  addSurface(
    analyzer, [](double x, double) {
      return x < 0.2 ? 0.0 : 0.05;
    });

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 0.1);
  const auto & cell = snapshot.at(1U, 1U);
  EXPECT_GE(cell.traversability, 60);
  EXPECT_LT(cell.traversability, 100);
}

TEST(TerrainAnalyzer, SlopeAboveTwentyTwoDegreesIsLethal)
{
  TerrainAnalyzer analyzer(testParameters());
  constexpr double slope_radians = 25.0 * 3.14159265358979323846 / 180.0;
  addSurface(
    analyzer, [](double x, double) {
      return x * std::tan(slope_radians);
    });

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 0.1);
  const auto & cell = snapshot.at(1U, 1U);
  EXPECT_GT(cell.slope, 22.0 * 3.14159265358979323846 / 180.0);
  EXPECT_EQ(cell.traversability, 100);
}

TEST(TerrainAnalyzer, OldObservationsAreRemoved)
{
  TerrainAnalyzer analyzer(testParameters());
  addSurface(analyzer, [](double, double) {return 0.0;});
  ASSERT_GT(analyzer.storedCellCount(), 0U);

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 1.2);
  EXPECT_EQ(analyzer.storedCellCount(), 0U);
  EXPECT_EQ(snapshot.at(1U, 1U).traversability, -1);
}

TEST(TerrainAnalyzer, SimulationTimeResetClearsAccumulatedMap)
{
  TerrainAnalyzer analyzer(testParameters());
  addSurface(analyzer, [](double, double) {return 0.0;});
  ASSERT_GT(analyzer.storedCellCount(), 0U);
  (void)analyzer.makeSnapshot(0.2, 0.2, 0.1);

  const auto snapshot = analyzer.makeSnapshot(0.2, 0.2, 0.0);
  EXPECT_EQ(analyzer.storedCellCount(), 0U);
  EXPECT_EQ(snapshot.at(1U, 1U).traversability, -1);
}

TEST(TerrainAnalyzer, RejectsInvalidThresholdOrdering)
{
  auto parameters = testParameters();
  parameters.caution_step_height = 0.08;
  parameters.lethal_step_height = 0.07;
  EXPECT_THROW(TerrainAnalyzer analyzer(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace lunar_rover_perception
