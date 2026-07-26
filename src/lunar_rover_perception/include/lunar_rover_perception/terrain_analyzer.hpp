#ifndef LUNAR_ROVER_PERCEPTION__TERRAIN_ANALYZER_HPP_
#define LUNAR_ROVER_PERCEPTION__TERRAIN_ANALYZER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace lunar_rover_perception
{

enum class PointSource : std::uint8_t
{
  kLidar,
  kDepth
};

struct TerrainPoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double weight{1.0};
  PointSource source{PointSource::kLidar};
};

struct TerrainParameters
{
  double resolution{0.10};
  double length_x{10.0};
  double length_y{10.0};
  double max_point_age{1.5};
  double caution_step_height{0.05};
  double lethal_step_height{0.07};
  double caution_slope_radians{15.0 * 3.14159265358979323846 / 180.0};
  double lethal_slope_radians{22.0 * 3.14159265358979323846 / 180.0};
  double caution_roughness{0.025};
  double lethal_roughness{0.060};
  double minimum_confidence{0.35};
  double required_point_weight{3.0};
  std::size_t maximum_stored_cells{200000U};
};

struct TerrainCell
{
  double x{0.0};
  double y{0.0};
  double elevation{std::numeric_limits<double>::quiet_NaN()};
  double variance{std::numeric_limits<double>::quiet_NaN()};
  double slope{std::numeric_limits<double>::quiet_NaN()};
  double roughness{std::numeric_limits<double>::quiet_NaN()};
  double step{std::numeric_limits<double>::quiet_NaN()};
  double confidence{0.0};
  std::int8_t traversability{-1};
  bool observed{false};
  bool lethal{false};
};

struct TerrainSnapshot
{
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::size_t width{0U};
  std::size_t height{0U};
  std::vector<TerrainCell> cells;

  const TerrainCell & at(std::size_t x, std::size_t y) const;
};

class TerrainAnalyzer
{
public:
  explicit TerrainAnalyzer(TerrainParameters parameters = TerrainParameters{});

  void addPoints(const std::vector<TerrainPoint> & points, double stamp_seconds);
  TerrainSnapshot makeSnapshot(
    double center_x, double center_y, double stamp_seconds);
  void clear();
  std::size_t storedCellCount() const;
  const TerrainParameters & parameters() const;

private:
  struct CellKey
  {
    std::int64_t x{0};
    std::int64_t y{0};

    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct CellKeyHash
  {
    std::size_t operator()(const CellKey & key) const;
  };

  struct CellStatistics
  {
    double weight{0.0};
    double mean{0.0};
    double m2{0.0};
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};
    double last_seen{0.0};
    std::uint32_t lidar_points{0U};
    std::uint32_t depth_points{0U};
  };

  CellKey keyFor(double x, double y) const;
  void prune(double center_x, double center_y, double stamp_seconds);
  std::int8_t costFor(
    double slope, double roughness, double step, double confidence) const;

  TerrainParameters parameters_;
  std::unordered_map<CellKey, CellStatistics, CellKeyHash> cells_;
  double last_snapshot_stamp_{-std::numeric_limits<double>::infinity()};
};

}  // namespace lunar_rover_perception

#endif  // LUNAR_ROVER_PERCEPTION__TERRAIN_ANALYZER_HPP_
