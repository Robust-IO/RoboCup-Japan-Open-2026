#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace interactive_cleanup
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PlanarGoal
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ApproachCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
  bool reached{false};
};

double normalizeAngle(double angle);

double bearingToTarget(const PlanarPose & robot_pose, double tx, double ty);

PlanarGoal buildApproachGoal(
  const PlanarPose & robot_pose,
  const geometry_msgs::msg::Point & target_point,
  double standoff);

ApproachCommand computeFinalApproachCommand(
  double distance,
  double heading_error,
  double stop_distance,
  double turn_threshold,
  double max_linear_speed,
  double max_angular_speed);

ApproachCommand computeApproachCommandToTarget(
  const PlanarPose & robot_pose,
  double tx,
  double ty,
  double stop_distance,
  double turn_threshold,
  double max_linear_speed,
  double max_angular_speed);

double computeApproachTimeout(
  double initial_distance,
  double stop_distance,
  double nominal_linear_speed,
  double min_timeout,
  double max_timeout,
  double reaction_time);

bool isWithinApproachTolerance(
  double distance,
  double stop_distance,
  double tolerance);

PlanarGoal transformGoalToMap(
  const geometry_msgs::msg::Point & source_point,
  double source_yaw,
  const geometry_msgs::msg::TransformStamped & map_from_source);

}  // namespace interactive_cleanup
