#include "interactive_cleanup/grasp_utils.hpp"

#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

BaseCorrection computePreGraspBaseCorrection(
  double current_forward,
  double current_lateral,
  double target_forward,
  double target_lateral,
  double forward_tolerance,
  double lateral_tolerance,
  double max_linear_x,
  double max_linear_y)
{
  BaseCorrection correction;

  const double forward_error = current_forward - target_forward;
  const double lateral_error = current_lateral - target_lateral;

  if (std::abs(forward_error) <= std::max(0.0, forward_tolerance) &&
      std::abs(lateral_error) <= std::max(0.0, lateral_tolerance)) {
    correction.aligned = true;
    return correction;
  }

  correction.linear_x = std::clamp(forward_error * 0.6, -max_linear_x, max_linear_x);
  correction.linear_y = std::clamp(lateral_error * 0.8, -max_linear_y, max_linear_y);
  return correction;
}

std::array<double, 5> computeParameterizedGraspArmPose(double target_height)
{
  const double clamped_height = std::clamp(target_height, 0.0, 0.20);
  const double arm_lift = std::clamp(0.02 + clamped_height * 0.55, 0.02, 0.14);

  return {arm_lift, -0.8, 0.0, -0.7, 0.0};
}

bool gripperLikelyHoldingObject(
  double hand_motor_joint_position,
  double fully_closed_position,
  double holding_margin)
{
  return hand_motor_joint_position > fully_closed_position + std::max(0.0, holding_margin);
}

bool isGraspVerificationSuccessful(
  bool gripper_holding_object,
  bool target_visible_near_pickup,
  bool hand_camera_confirms_grasp)
{
  return gripper_holding_object || hand_camera_confirms_grasp || !target_visible_near_pickup;
}

}  // namespace interactive_cleanup
