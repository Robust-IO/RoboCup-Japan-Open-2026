#include "handyman_ros2/room_navigation.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace handyman_ros2 {
namespace {

std::string toLower(const std::string &value)
{
  std::string lowered = value;
  std::transform(
    lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lowered;
}

std::vector<PatrolWaypoint> fallbackPatrolWaypoints()
{
  return {{0.5, 2.0, 1.57}};
}

}  // namespace

std::vector<PatrolWaypoint> roomPatrolWaypoints(
  const std::string &environment,
  const std::string &room)
{
  const std::string env_lower = toLower(environment);

  if (env_lower == "layout2019hm01") {
    if (room == "living") {
      return {{2.0, 1.0, 1.57}, {2.5, 4.08, 3.14}};
    }
    if (room == "bedroom") {
      return {{2.57, -4.31, 0.0}, {8.64, -5.7, 0.0}};
    }
    if (room == "lobby") {
      return {{1.2, -6.16, -1.57}, {1.0, -3.6, 0.0}};
    }
    if (room == "kitchen") {
      return {{8.5, 2.8, 3.14}};
    }
  } else if (env_lower == "layout2019hm02") {
    if (room == "living") {
      return {{3.5, 9.6, 2.4}, {1.84, 10.2, 0.0}};
    }
    if (room == "lobby") {
      return {{1.0, 0.0, 0.0}, {2.5, 2.0, 0.0}};
    }
    if (room == "kitchen") {
      return {{5.5, -1.13, 0.0}, {8.42, -1.13, 0.0}};
    }
  } else if (env_lower == "layout2020hm01") {
    if (room == "living") {
      return {
        {0.5, 2.0, 1.57},
        {0.42, 3.48, 2.355},
        {4.5, 3.48, 0.0},
        {4.5, -0.65, 0.0}
      };
    }
    if (room == "bedroom") {
      return {{0.1, 6.9, 0.0}, {3.0, 8.0, 0.0}};
    }
    if (room == "kitchen") {
      return {{6.5, -1.2, 0.0}, {7.8, 1.2, 0.0}, {6.5, 3.9, 0.0}};
    }
  } else if (env_lower == "layout2021hm01") {
    if (room == "living") {
      return {{1.0, 0.0, 0.0}, {3.5, 0.0, 0.0}};
    }
    if (room == "bedroom") {
      return {{4.0, -8.5, 0.0}, {1.69, -8.0, 0.0}};
    }
    if (room == "lobby") {
      return {{-1.86, -8.38, 0.0}, {-4.86, -8.7, 0.0}};
    }
  }

  return fallbackPatrolWaypoints();
}

bool isRobotWithinRoomPatrolArea(
  double robot_x,
  double robot_y,
  const std::vector<PatrolWaypoint> &patrol_waypoints,
  double threshold_meters)
{
  return std::any_of(
    patrol_waypoints.begin(), patrol_waypoints.end(),
    [robot_x, robot_y, threshold_meters](const PatrolWaypoint &waypoint) {
      return std::hypot(robot_x - waypoint.x, robot_y - waypoint.y) < threshold_meters;
    });
}

RoomEntryDecision decideRoomEntryAction(
  bool room_reached,
  bool nav_goal_sent,
  double robot_x,
  double robot_y,
  const std::vector<PatrolWaypoint> &patrol_waypoints,
  double threshold_meters)
{
  if (room_reached || nav_goal_sent) {
    return {false, false, false};
  }

  if (isRobotWithinRoomPatrolArea(robot_x, robot_y, patrol_waypoints, threshold_meters)) {
    return {true, true, false};
  }

  return {false, false, true};
}

}  // namespace handyman_ros2
