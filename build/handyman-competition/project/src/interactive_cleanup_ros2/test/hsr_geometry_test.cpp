#include <gtest/gtest.h>

#include "interactive_cleanup/hsr_geometry.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(HsrGeometryTest, ClampsArmPoseToOfficialJointLimits)
{
  HsrArmPose pose;
  pose.arm_lift = 1.0;
  pose.arm_flex = -3.5;
  pose.arm_roll = 5.0;
  pose.wrist_flex = -3.0;
  pose.wrist_roll = -3.0;

  const auto clamped = clampArmPoseToLimits(pose);

  EXPECT_NEAR(clamped.arm_lift, 0.69, 1e-6);
  EXPECT_NEAR(clamped.arm_flex, -2.62, 1e-6);
  EXPECT_NEAR(clamped.arm_roll, 3.84, 1e-6);
  EXPECT_NEAR(clamped.wrist_flex, -1.92, 1e-6);
  EXPECT_NEAR(clamped.wrist_roll, -1.92, 1e-6);
}

TEST(HsrGeometryTest, ComputesHandCameraOffsetFromOfficialDescription)
{
  const auto offset = handCameraOffsetFromWristRoll();

  EXPECT_NEAR(offset.x, -0.027, 1e-6);
  EXPECT_NEAR(offset.y, 0.0, 1e-6);
  EXPECT_NEAR(offset.z, 0.136, 1e-6);
}

TEST(HsrGeometryTest, RaisesHandCameraWhenArmLiftIncreases)
{
  HsrArmPose low_pose;
  low_pose.arm_lift = 0.05;
  low_pose.arm_flex = -1.8;
  low_pose.wrist_flex = -1.0;

  HsrArmPose high_pose = low_pose;
  high_pose.arm_lift = 0.30;

  const auto low_estimate = estimateManipulatorPose(low_pose);
  const auto high_estimate = estimateManipulatorPose(high_pose);

  EXPECT_LT(low_estimate.hand_camera.z, high_estimate.hand_camera.z);
}

}  // namespace
}  // namespace interactive_cleanup
