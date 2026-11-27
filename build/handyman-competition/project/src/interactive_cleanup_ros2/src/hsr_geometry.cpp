#include "interactive_cleanup/hsr_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

namespace
{

constexpr double kArmLiftMin = 0.0;
constexpr double kArmLiftMax = 0.69;
constexpr double kArmFlexMin = -2.62;
constexpr double kArmFlexMax = 0.0;
constexpr double kArmRollMin = -2.09;
constexpr double kArmRollMax = 3.84;
constexpr double kWristFlexMin = -1.92;
constexpr double kWristFlexMax = 1.22;
constexpr double kWristRollMin = -1.92;
constexpr double kWristRollMax = 3.67;

constexpr double kBaseToArmLiftZ = 0.340;
constexpr double kArmShoulderX = 0.141;
constexpr double kArmShoulderY = 0.078;
constexpr double kUpperArmOffsetX = 0.005;
constexpr double kUpperArmOffsetZ = 0.345;
constexpr double kPalmOffsetX = 0.012;
constexpr double kPalmOffsetZ = 0.1405;

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::msg::Point rotateAroundY(
  const geometry_msgs::msg::Point &point,
  double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return makePoint(
    c * point.x + s * point.z,
    point.y,
    -s * point.x + c * point.z);
}

geometry_msgs::msg::Point addPoints(
  const geometry_msgs::msg::Point &lhs,
  const geometry_msgs::msg::Point &rhs)
{
  return makePoint(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

}  // namespace

HsrArmPose clampArmPoseToLimits(const HsrArmPose &pose)
{
  HsrArmPose clamped = pose;
  clamped.arm_lift = std::clamp(clamped.arm_lift, kArmLiftMin, kArmLiftMax);
  clamped.arm_flex = std::clamp(clamped.arm_flex, kArmFlexMin, kArmFlexMax);
  clamped.arm_roll = std::clamp(clamped.arm_roll, kArmRollMin, kArmRollMax);
  clamped.wrist_flex = std::clamp(clamped.wrist_flex, kWristFlexMin, kWristFlexMax);
  clamped.wrist_roll = std::clamp(clamped.wrist_roll, kWristRollMin, kWristRollMax);
  return clamped;
}

geometry_msgs::msg::Point handCameraOffsetFromWristRoll()
{
  return makePoint(-0.027, 0.0, 0.136);
}

HsrManipulatorPose estimateManipulatorPose(const HsrArmPose &pose)
{
  const auto clamped = clampArmPoseToLimits(pose);
  const double wrist_pitch = estimateHandPitch(clamped);

  const auto shoulder = makePoint(
    kArmShoulderX,
    kArmShoulderY,
    kBaseToArmLiftZ + clamped.arm_lift);
  const double arm_pitch = -clamped.arm_flex;
  const auto upper_arm = rotateAroundY(
    makePoint(kUpperArmOffsetX, 0.0, kUpperArmOffsetZ),
    arm_pitch);
  const auto wrist_origin = addPoints(shoulder, upper_arm);

  HsrManipulatorPose estimate;
  estimate.palm = addPoints(
    wrist_origin,
    rotateAroundY(makePoint(kPalmOffsetX, 0.0, kPalmOffsetZ), wrist_pitch));
  estimate.hand_camera = addPoints(
    wrist_origin,
    rotateAroundY(handCameraOffsetFromWristRoll(), wrist_pitch));
  return estimate;
}

double estimateHandPitch(const HsrArmPose &pose)
{
  const auto clamped = clampArmPoseToLimits(pose);
  return -clamped.arm_flex - clamped.wrist_flex;
}

}  // namespace interactive_cleanup
