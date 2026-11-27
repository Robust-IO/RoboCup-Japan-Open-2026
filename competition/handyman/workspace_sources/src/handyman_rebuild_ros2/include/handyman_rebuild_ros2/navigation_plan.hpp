#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "handyman_rebuild_ros2/environment_config.hpp"

namespace handyman_rebuild_ros2
{

struct NavigationCandidate
{
  Pose2D pose;
  std::string room;
  std::size_t source_index{0};
  std::vector<Pose2D> route_waypoints;
};

class NavigationPlan
{
public:
  static NavigationPlan forSearchPoint(
    const EnvironmentConfig & environment, const std::string & room,
    std::size_t index, std::size_t maximum_attempts,
    const std::optional<Pose2D> & robot_pose = std::nullopt);
  static NavigationPlan forRoom(
    const EnvironmentConfig & environment,
    const std::string & room,
    std::size_t maximum_attempts,
    const std::optional<Pose2D> & robot_pose = std::nullopt);

  static NavigationPlan forDestination(
    const EnvironmentConfig & environment,
    const std::string & destination,
    const std::string & destination_room,
    std::size_t maximum_attempts,
    const std::optional<Pose2D> & robot_pose = std::nullopt);

  bool valid() const noexcept;
  bool exhausted() const noexcept;
  const NavigationCandidate * current() const noexcept;
  bool advance() noexcept;
  std::size_t attemptNumber() const noexcept;
  std::size_t maximumAttempts() const noexcept;
  const std::string & error() const noexcept;

private:
  NavigationPlan(
    std::vector<NavigationCandidate> candidates,
    std::size_t maximum_attempts,
    std::string error);

  std::vector<NavigationCandidate> candidates_;
  std::size_t maximum_attempts_{0};
  std::size_t attempted_{0};
  std::string error_;
};

bool isPoseInsideRoom(
  const RoomConfig & room,
  const Pose2D & pose,
  double boundary_tolerance_m = 0.0);

}  // namespace handyman_rebuild_ros2
