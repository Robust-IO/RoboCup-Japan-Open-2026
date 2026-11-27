#pragma once

#include <array>

namespace interactive_cleanup
{

struct BaseCorrection
{
  double linear_x{0.0};
  double linear_y{0.0};
  bool aligned{false};
};

BaseCorrection computePreGraspBaseCorrection(
  double current_forward,
  double current_lateral,
  double target_forward,
  double target_lateral,
  double forward_tolerance,
  double lateral_tolerance,
  double max_linear_x,
  double max_linear_y);

std::array<double, 5> computeParameterizedGraspArmPose(double target_height);

bool gripperLikelyHoldingObject(
  double hand_motor_joint_position,
  double fully_closed_position,
  double holding_margin);

bool isGraspVerificationSuccessful(
  bool gripper_holding_object,
  bool target_visible_near_pickup,
  bool hand_camera_confirms_grasp = false);

}  // namespace interactive_cleanup
