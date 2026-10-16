#pragma once

#include <geometry_msgs/msg/point.hpp>

namespace interactive_cleanup
{

struct HsrArmPose
{
  double arm_lift{0.0};
  double arm_flex{0.0};
  double arm_roll{0.0};
  double wrist_flex{0.0};
  double wrist_roll{0.0};
};

struct HsrManipulatorPose
{
  geometry_msgs::msg::Point palm;
  geometry_msgs::msg::Point hand_camera;
};

HsrArmPose clampArmPoseToLimits(const HsrArmPose &pose);

geometry_msgs::msg::Point handCameraOffsetFromWristRoll();

HsrManipulatorPose estimateManipulatorPose(const HsrArmPose &pose);

double estimateHandPitch(const HsrArmPose &pose);

}  // namespace interactive_cleanup
