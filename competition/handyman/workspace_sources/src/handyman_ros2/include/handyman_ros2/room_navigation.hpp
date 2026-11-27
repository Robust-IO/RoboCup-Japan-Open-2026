#pragma once

#include <string>
#include <vector>

namespace handyman_ros2 {

struct PatrolWaypoint {
  double x;
  double y;
  double yaw;
};

struct RoomEntryDecision {
  bool mark_room_reached;
  bool send_room_reached_message;
  bool should_send_navigation_goal;
};

std::vector<PatrolWaypoint> roomPatrolWaypoints(
  const std::string &environment,
  const std::string &room);

bool isRobotWithinRoomPatrolArea(
  double robot_x,
  double robot_y,
  const std::vector<PatrolWaypoint> &patrol_waypoints,
  double threshold_meters);

RoomEntryDecision decideRoomEntryAction(
  bool room_reached,
  bool nav_goal_sent,
  double robot_x,
  double robot_y,
  const std::vector<PatrolWaypoint> &patrol_waypoints,
  double threshold_meters);

}  // namespace handyman_ros2
