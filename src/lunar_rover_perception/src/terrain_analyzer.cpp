#include "lunar_rover_perception/terrain_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lunar_rover_perception
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

double normalizedBand(double value, double lower, double upper)
{
  if (value <= lower) {
    return 0.0;
  }
  if (value >= upper) {
    return 1.0;
  }
  return (value - lower) / (upper - lower);
}

}  // namespace

const TerrainCell & TerrainSnapshot::at(std::size_t x, std::size_t y) const
{
  if (x >= width || y >= height) {
    throw std::out_of_range("terrain snapshot index is outside the map");
  }
  return cells[y * width + x];
}

TerrainAnalyzer::TerrainAnalyzer(TerrainParameters parameters)
: parameters_(std::move(parameters))
{
  if (
    parameters_.resolution <= 0.0 || parameters_.length_x <= 0.0 ||
    parameters_.length_y <= 0.0 || parameters_.max_point_age <= 0.0)
  {
    throw std::invalid_argument("terrain map dimensions, resolution and age must be positive");
  }
  if (
    parameters_.required_point_weight <= 0.0 ||
    parameters_.minimum_confidence < 0.0 || parameters_.minimum_confidence > 1.0 ||
    parameters_.maximum_stored_cells == 0U)
  {
    throw std::invalid_argument("terrain confidence parameters are invalid");
  }
  if (
    parameters_.caution_step_height >= parameters_.lethal_step_height ||
    parameters_.caution_slope_radians >= parameters_.lethal_slope_radians ||
    parameters_.caution_roughness >= parameters_.lethal_roughness)
  {
    throw std::invalid_argument("each caution threshold must be below its lethal threshold");
  }
}

std::size_t TerrainAnalyzer::CellKeyHash::operator()(const CellKey & key) const
{
  const auto hx = std::hash<std::int64_t>{}(key.x);
  const auto hy = std::hash<std::int64_t>{}(key.y);
  return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6U) + (hx >> 2U));
}

TerrainAnalyzer::CellKey TerrainAnalyzer::keyFor(double x, double y) const
{
  return CellKey{
    static_cast<std::int64_t>(std::floor(x / parameters_.resolution)),
    static_cast<std::int64_t>(std::floor(y / parameters_.resolution))};
}

void TerrainAnalyzer::addPoints(
  const std::vector<TerrainPoint> & points, double stamp_seconds)
{
  for (const auto & point : points) {
    if (
      !std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.weight) ||
      point.weight <= 0.0)
    {
      continue;
    }

    auto & statistics = cells_[keyFor(point.x, point.y)];
    const double previous_weight = statistics.weight;
    const double combined_weight = previous_weight + point.weight;
    const double delta = point.z - statistics.mean;
    statistics.mean += point.weight * delta / combined_weight;
    statistics.m2 += point.weight * delta * (point.z - statistics.mean);
    statistics.weight = combined_weight;
    statistics.minimum = std::min(statistics.minimum, point.z);
    statistics.maximum = std::max(statistics.maximum, point.z);
    statistics.last_seen = std::max(statistics.last_seen, stamp_seconds);
    if (point.source == PointSource::kLidar) {
      ++statistics.lidar_points;
    } else {
      ++statistics.depth_points;
    }
  }
}

void TerrainAnalyzer::prune(
  double center_x, double center_y, double stamp_seconds)
{
  const double half_x = parameters_.length_x * 0.5 + parameters_.resolution;
  const double half_y = parameters_.length_y * 0.5 + parameters_.resolution;
  for (auto iterator = cells_.begin(); iterator != cells_.end();) {
    const double x = (static_cast<double>(iterator->first.x) + 0.5) *
      parameters_.resolution;
    const double y = (static_cast<double>(iterator->first.y) + 0.5) *
      parameters_.resolution;
    const double age = stamp_seconds - iterator->second.last_seen;
    if (
      age > parameters_.max_point_age ||
      std::abs(x - center_x) > half_x || std::abs(y - center_y) > half_y)
    {
      iterator = cells_.erase(iterator);
    } else {
      ++iterator;
    }
  }

  if (cells_.size() <= parameters_.maximum_stored_cells) {
    return;
  }

  std::vector<std::pair<CellKey, double>> ages;
  ages.reserve(cells_.size());
  for (const auto & entry : cells_) {
    ages.emplace_back(entry.first, entry.second.last_seen);
  }
  const auto keep = parameters_.maximum_stored_cells;
  std::nth_element(
    ages.begin(), ages.begin() + static_cast<std::ptrdiff_t>(keep), ages.end(),
    [](const auto & left, const auto & right) {
      return left.second > right.second;
    });
  for (auto iterator = ages.begin() + static_cast<std::ptrdiff_t>(keep);
    iterator != ages.end(); ++iterator)
  {
    cells_.erase(iterator->first);
  }
}

std::int8_t TerrainAnalyzer::costFor(
  double slope, double roughness, double step, double confidence) const
{
  if (
    !std::isfinite(slope) || !std::isfinite(roughness) ||
    !std::isfinite(step) || confidence < parameters_.minimum_confidence)
  {
    return -1;
  }
  if (
    step > parameters_.lethal_step_height ||
    slope > parameters_.lethal_slope_radians ||
    roughness > parameters_.lethal_roughness)
  {
    return 100;
  }

  const double step_score = normalizedBand(
    step, parameters_.caution_step_height, parameters_.lethal_step_height);
  const double slope_score = normalizedBand(
    slope, parameters_.caution_slope_radians, parameters_.lethal_slope_radians);
  const double roughness_score = normalizedBand(
    roughness, parameters_.caution_roughness, parameters_.lethal_roughness);
  const double caution_score = std::max({step_score, slope_score, roughness_score});
  const bool caution =
    step >= parameters_.caution_step_height ||
    slope >= parameters_.caution_slope_radians ||
    roughness >= parameters_.caution_roughness;

  const double nominal_score = std::max({
      step / parameters_.caution_step_height,
      slope / parameters_.caution_slope_radians,
      roughness / parameters_.caution_roughness});
  if (caution) {
    return static_cast<std::int8_t>(
      std::clamp(std::lround(60.0 + 39.0 * caution_score), 0L, 99L));
  }
  return static_cast<std::int8_t>(
    std::clamp(std::lround(40.0 * nominal_score), 0L, 40L));
}

TerrainSnapshot TerrainAnalyzer::makeSnapshot(
  double center_x, double center_y, double stamp_seconds)
{
  if (stamp_seconds < last_snapshot_stamp_ - kEpsilon) {
    cells_.clear();
  }
  last_snapshot_stamp_ = stamp_seconds;
  prune(center_x, center_y, stamp_seconds);

  TerrainSnapshot result;
  result.resolution = parameters_.resolution;
  result.width = static_cast<std::size_t>(
    std::ceil(parameters_.length_x / parameters_.resolution));
  result.height = static_cast<std::size_t>(
    std::ceil(parameters_.length_y / parameters_.resolution));
  result.origin_x = std::floor(
    (center_x - 0.5 * static_cast<double>(result.width) * result.resolution) /
    result.resolution) * result.resolution;
  result.origin_y = std::floor(
    (center_y - 0.5 * static_cast<double>(result.height) * result.resolution) /
    result.resolution) * result.resolution;
  result.cells.resize(result.width * result.height);

  for (std::size_t y = 0; y < result.height; ++y) {
    for (std::size_t x = 0; x < result.width; ++x) {
      auto & output = result.cells[y * result.width + x];
      output.x = result.origin_x + (static_cast<double>(x) + 0.5) * result.resolution;
      output.y = result.origin_y + (static_cast<double>(y) + 0.5) * result.resolution;
      const auto found = cells_.find(keyFor(output.x, output.y));
      if (
        found == cells_.end() ||
        found->second.weight < parameters_.required_point_weight)
      {
        continue;
      }

      output.observed = true;
      output.elevation = found->second.mean;
      output.variance = std::max(0.0, found->second.m2 / found->second.weight);
      const double age = std::max(0.0, stamp_seconds - found->second.last_seen);
      const double observation_confidence = std::min(
        1.0, found->second.weight / parameters_.required_point_weight);
      const double sensor_factor =
        (found->second.lidar_points > 0U && found->second.depth_points > 0U) ? 1.0 : 0.85;
      output.confidence = observation_confidence * sensor_factor *
        std::exp(-age / parameters_.max_point_age);
    }
  }

  for (std::size_t y = 0; y < result.height; ++y) {
    for (std::size_t x = 0; x < result.width; ++x) {
      auto & output = result.cells[y * result.width + x];
      if (!output.observed) {
        continue;
      }

      struct Neighbor
      {
        double dx;
        double dy;
        double dz;
      };
      std::vector<Neighbor> neighbors;
      double maximum_step = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const auto nx = static_cast<std::int64_t>(x) + dx;
          const auto ny = static_cast<std::int64_t>(y) + dy;
          if (
            nx < 0 || ny < 0 ||
            nx >= static_cast<std::int64_t>(result.width) ||
            ny >= static_cast<std::int64_t>(result.height))
          {
            continue;
          }
          const auto & neighbor =
            result.cells[static_cast<std::size_t>(ny) * result.width +
            static_cast<std::size_t>(nx)];
          if (!neighbor.observed) {
            continue;
          }
          const double dz = neighbor.elevation - output.elevation;
          maximum_step = std::max(maximum_step, std::abs(dz));
          neighbors.push_back(Neighbor{
              static_cast<double>(dx) * result.resolution,
              static_cast<double>(dy) * result.resolution,
              dz});
        }
      }

      double gradient_x = 0.0;
      double gradient_y = 0.0;
      if (neighbors.size() >= 2U) {
        double xx = 0.0;
        double xy = 0.0;
        double yy = 0.0;
        double xz = 0.0;
        double yz = 0.0;
        for (const auto & neighbor : neighbors) {
          xx += neighbor.dx * neighbor.dx;
          xy += neighbor.dx * neighbor.dy;
          yy += neighbor.dy * neighbor.dy;
          xz += neighbor.dx * neighbor.dz;
          yz += neighbor.dy * neighbor.dz;
        }
        const double determinant = xx * yy - xy * xy;
        if (std::abs(determinant) > kEpsilon) {
          gradient_x = (xz * yy - yz * xy) / determinant;
          gradient_y = (yz * xx - xz * xy) / determinant;
        }
      }

      double residual_sum = 0.0;
      for (const auto & neighbor : neighbors) {
        const double residual =
          neighbor.dz - gradient_x * neighbor.dx - gradient_y * neighbor.dy;
        residual_sum += residual * residual;
      }
      const double neighborhood_variance = neighbors.empty() ? 0.0 :
        residual_sum / static_cast<double>(neighbors.size());
      output.slope = std::atan(std::hypot(gradient_x, gradient_y));
      output.step = maximum_step;
      output.roughness = std::sqrt(std::max(0.0, output.variance + neighborhood_variance));
      if (neighbors.size() < 2U) {
        output.confidence *= 0.65;
      }
      output.traversability = costFor(
        output.slope, output.roughness, output.step, output.confidence);
      output.lethal = output.traversability == 100;
    }
  }

  return result;
}

void TerrainAnalyzer::clear()
{
  cells_.clear();
  last_snapshot_stamp_ = -std::numeric_limits<double>::infinity();
}

std::size_t TerrainAnalyzer::storedCellCount() const
{
  return cells_.size();
}

const TerrainParameters & TerrainAnalyzer::parameters() const
{
  return parameters_;
}

}  // namespace lunar_rover_perception
