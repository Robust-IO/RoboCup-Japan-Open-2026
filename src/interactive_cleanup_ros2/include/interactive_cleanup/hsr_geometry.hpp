#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

// SIGVerse HSRCommon.cs JointRangeMap / GetMaxJointSpeed (radians, rad/s).
// These are actuator limits, not task-specific perception thresholds.
inline constexpr double HSR_HEAD_PAN_MIN = -3.839;
inline constexpr double HSR_HEAD_PAN_MAX = 1.745;
inline constexpr double HSR_HEAD_TILT_MIN = -1.570;
inline constexpr double HSR_HEAD_TILT_MAX = 0.523;
inline constexpr double HSR_HEAD_MAX_SPEED = 1.0;
struct HsrHeadMotion { double pan, tilt, duration; };
inline HsrHeadMotion planHeadMotion(double current_pan, double current_tilt,
                                    double pan, double tilt, double duration)
{
  pan = std::clamp(pan, HSR_HEAD_PAN_MIN, HSR_HEAD_PAN_MAX);
  tilt = std::clamp(tilt, HSR_HEAD_TILT_MIN, HSR_HEAD_TILT_MAX);
  duration = std::max({duration, std::abs(pan-current_pan)/HSR_HEAD_MAX_SPEED,
                      std::abs(tilt-current_tilt)/HSR_HEAD_MAX_SPEED});
  return {pan, tilt, duration};
}

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
