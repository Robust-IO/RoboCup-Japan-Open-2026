#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace handyman_ros2 {

struct PlacementCandidate {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

std::string resolveDestinationRoom(
  const std::string &instruction,
  const std::vector<std::string> &rooms,
  const std::string &pickup_room);

std::vector<PlacementCandidate> placementCandidates(
  const std::string &environment,
  const std::string &destination,
  const std::string &destination_room);

std::size_t nextPlacementCandidateIndex(
  std::size_t current_index,
  std::size_t candidate_count);

}  // namespace handyman_ros2
