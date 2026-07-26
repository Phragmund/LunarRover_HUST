#ifndef LUNAR_ROVER_NAVIGATION__TERRAIN_COST_HPP_
#define LUNAR_ROVER_NAVIGATION__TERRAIN_COST_HPP_

#include <algorithm>
#include <cstdint>

namespace lunar_rover_navigation
{

struct TerrainCostParameters
{
  std::uint8_t lethal_threshold{100U};
  bool unknown_is_lethal{true};
};

constexpr std::uint8_t kFreeSpace = 0U;
constexpr std::uint8_t kInscribedInflatedObstacle = 253U;
constexpr std::uint8_t kLethalObstacle = 254U;
constexpr std::uint8_t kNoInformation = 255U;

inline std::uint8_t terrainValueToCost(
  std::int8_t value, const TerrainCostParameters & parameters)
{
  if (value < 0) {
    return parameters.unknown_is_lethal ? kLethalObstacle : kNoInformation;
  }
  const auto numeric_value = static_cast<std::uint8_t>(value);
  if (numeric_value >= parameters.lethal_threshold) {
    return kLethalObstacle;
  }
  const auto denominator = std::max<std::uint8_t>(1U, parameters.lethal_threshold);
  const auto scaled =
    static_cast<unsigned int>(numeric_value) *
    static_cast<unsigned int>(kInscribedInflatedObstacle - 1U) /
    static_cast<unsigned int>(denominator);
  return static_cast<std::uint8_t>(
    std::min<unsigned int>(scaled, kInscribedInflatedObstacle - 1U));
}

}  // namespace lunar_rover_navigation

#endif  // LUNAR_ROVER_NAVIGATION__TERRAIN_COST_HPP_
