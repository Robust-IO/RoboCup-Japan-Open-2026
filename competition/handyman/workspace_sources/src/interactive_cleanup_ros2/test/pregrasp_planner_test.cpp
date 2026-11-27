#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>

#include "interactive_cleanup/hsr_geometry.hpp"
#include "interactive_cleanup/pregrasp_planner.hpp"

namespace interactive_cleanup
{
namespace
{

TEST(PregraspPlannerTest, PlansFloorPickWithLowerCameraAndLongerStandoff)
{
  geometry_msgs::msg::Point floor_target;
  floor_target.x = 1.1;
  floor_target.y = 0.0;
  floor_target.z = 0.03;

  geometry_msgs::msg::Point table_target = floor_target;
  table_target.z = 0.15;

  const auto floor_plan = planPregrasp(floor_target, "floor_pick");
  const auto table_plan = planPregrasp(table_target, "table_low_pick");

  ASSERT_TRUE(floor_plan.valid);
  ASSERT_TRUE(table_plan.valid);
  EXPECT_GT(floor_plan.nav_standoff, table_plan.nav_standoff);
  EXPECT_LT(floor_plan.arm_pose.arm_flex, table_plan.arm_pose.arm_flex);

  const auto floor_pose = estimateManipulatorPose(floor_plan.arm_pose);
  const auto table_pose = estimateManipulatorPose(table_plan.arm_pose);
  EXPECT_LT(floor_pose.hand_camera.z, table_pose.hand_camera.z);
}

TEST(PregraspPlannerTest, PlansFloorPickWithHandApproximatelyLevel)
{
  geometry_msgs::msg::Point floor_target;
  floor_target.x = 1.1;
  floor_target.y = 0.0;
  floor_target.z = 0.03;

  const auto floor_plan = planPregrasp(floor_target, "floor_pick");

  ASSERT_TRUE(floor_plan.valid);
  EXPECT_NEAR(estimateHandPitch(floor_plan.arm_pose), 1.57, 0.25);
}

TEST(PregraspPlannerTest, PlansMidTablePickHigherThanLowPick)
{
  geometry_msgs::msg::Point low_target;
  low_target.x = 1.0;
  low_target.y = 0.0;
  low_target.z = 0.18;

  geometry_msgs::msg::Point mid_target = low_target;
  mid_target.z = 0.30;

  const auto low_plan = planPregrasp(low_target, "table_low_pick");
  const auto mid_plan = planPregrasp(mid_target, "table_mid_pick");

  ASSERT_TRUE(low_plan.valid);
  ASSERT_TRUE(mid_plan.valid);
  EXPECT_LT(low_plan.arm_pose.arm_lift, mid_plan.arm_pose.arm_lift);
  EXPECT_LT(low_plan.desired_hand_camera_height, mid_plan.desired_hand_camera_height);
}

TEST(PregraspPlannerTest, RejectsUnknownGraspMode)
{
  geometry_msgs::msg::Point target;
  target.x = 1.0;
  target.z = 0.10;

  const auto plan = planPregrasp(target, "unsupported_mode");
  EXPECT_FALSE(plan.valid);
}

}  // namespace
}  // namespace interactive_cleanup
