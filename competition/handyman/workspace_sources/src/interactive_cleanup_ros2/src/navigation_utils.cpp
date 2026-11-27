#include "interactive_cleanup/navigation_utils.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace interactive_cleanup
{

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double bearingToTarget(const PlanarPose & robot_pose, double tx, double ty)
{
  return normalizeAngle(
    std::atan2(ty - robot_pose.y, tx - robot_pose.x) - robot_pose.yaw);
}

PlanarGoal buildApproachGoal(
  const PlanarPose & robot_pose,
  const geometry_msgs::msg::Point & target_point,
  double standoff)
{
  const double dx = target_point.x - robot_pose.x;
  const double dy = target_point.y - robot_pose.y;
  const double distance = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);

  PlanarGoal goal;
  goal.yaw = normalizeAngle(yaw);

  if (distance <= standoff) {
    goal.x = robot_pose.x;
    goal.y = robot_pose.y;
    return goal;
  }

  goal.x = target_point.x - std::cos(yaw) * standoff;
  goal.y = target_point.y - std::sin(yaw) * standoff;
  return goal;
}

ApproachCommand computeFinalApproachCommand(
  double distance,
  double heading_error,
  double stop_distance,
  double turn_threshold,
  double max_linear_speed,
  double max_angular_speed)
{
  ApproachCommand cmd;
  if (distance <= stop_distance) {
    cmd.reached = true;
    return cmd;
  }

  if (std::abs(heading_error) > turn_threshold) {
    cmd.angular_z = std::clamp(
      heading_error * 1.2, -max_angular_speed, max_angular_speed);
    return cmd;
  }

  cmd.linear_x = std::clamp(
    (distance - stop_distance) * 0.6, 0.0, max_linear_speed);
  cmd.angular_z = std::clamp(
    heading_error * 0.8, -max_angular_speed, max_angular_speed);
  return cmd;
}

ApproachCommand computeApproachCommandToTarget(
  const PlanarPose & robot_pose,
  double tx,
  double ty,
  double stop_distance,
  double turn_threshold,
  double max_linear_speed,
  double max_angular_speed)
{
  const double distance = std::hypot(tx - robot_pose.x, ty - robot_pose.y);
  const double heading_error = bearingToTarget(robot_pose, tx, ty);
  return computeFinalApproachCommand(
    distance, heading_error,
    stop_distance, turn_threshold,
    max_linear_speed, max_angular_speed);
}

double computeApproachTimeout(
  double initial_distance,
  double stop_distance,
  double nominal_linear_speed,
  double min_timeout,
  double max_timeout,
  double reaction_time)
{
  const double remaining_distance = std::max(0.0, initial_distance - stop_distance);
  const double safe_speed = std::max(nominal_linear_speed, 1e-3);
  const double travel_time = remaining_distance / safe_speed;
  const double timeout = travel_time * 1.5 + reaction_time;
  return std::clamp(timeout, min_timeout, max_timeout);
}

bool isWithinApproachTolerance(
  double distance,
  double stop_distance,
  double tolerance)
{
  return distance <= stop_distance + std::max(0.0, tolerance);
}

PlanarGoal transformGoalToMap(
  const geometry_msgs::msg::Point & source_point,
  double source_yaw,
  const geometry_msgs::msg::TransformStamped & map_from_source)
{
  tf2::Quaternion q;
  tf2::fromMsg(map_from_source.transform.rotation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  PlanarGoal goal;
  goal.x = map_from_source.transform.translation.x +
    c * source_point.x - s * source_point.y;
  goal.y = map_from_source.transform.translation.y +
    s * source_point.x + c * source_point.y;
  goal.yaw = normalizeAngle(source_yaw + yaw);
  return goal;
}

}  // namespace interactive_cleanup
