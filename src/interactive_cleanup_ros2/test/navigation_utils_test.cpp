#include <gtest/gtest.h>

#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "interactive_cleanup/navigation_utils.hpp"

namespace
{

geometry_msgs::msg::TransformStamped makeTransform(
  double x, double y, double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "odom";
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  tf.transform.rotation = tf2::toMsg(q);
  return tf;
}

}  // namespace

TEST(NavigationUtilsTest, TransformsOdomGoalIntoMapFrame)
{
  geometry_msgs::msg::Point odom_target;
  odom_target.x = 1.0;
  odom_target.y = 0.0;
  odom_target.z = 0.2;

  auto goal = interactive_cleanup::transformGoalToMap(
    odom_target, 0.0, makeTransform(10.0, -3.0, M_PI_2));

  EXPECT_NEAR(goal.x, 10.0, 1e-6);
  EXPECT_NEAR(goal.y, -2.0, 1e-6);
  EXPECT_NEAR(goal.yaw, M_PI_2, 1e-6);
}

TEST(NavigationUtilsTest, ComputesBearingInRequestedFrame)
{
  interactive_cleanup::PlanarPose robot_pose;
  robot_pose.x = -2.0;
  robot_pose.y = -3.5;
  robot_pose.yaw = M_PI_2;

  const double yaw = interactive_cleanup::bearingToTarget(
    robot_pose, -0.74, 1.42);

  EXPECT_NEAR(yaw, -0.2507092594, 1e-6);
}

TEST(NavigationUtilsTest, BuildsObjectApproachGoalWithStandoff)
{
  interactive_cleanup::PlanarPose robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;

  geometry_msgs::msg::Point target;
  target.x = 1.0;
  target.y = 0.0;

  auto goal = interactive_cleanup::buildApproachGoal(robot_pose, target, 0.75);

  EXPECT_NEAR(goal.x, 0.25, 1e-6);
  EXPECT_NEAR(goal.y, 0.0, 1e-6);
  EXPECT_NEAR(goal.yaw, 0.0, 1e-6);
}

TEST(NavigationUtilsTest, StopsFinalApproachWhenWithinSafeDistance)
{
  auto cmd = interactive_cleanup::computeFinalApproachCommand(
    0.40, 0.02, 0.45, 0.15, 0.08, 0.35);

  EXPECT_TRUE(cmd.reached);
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST(NavigationUtilsTest, TurnsInPlaceWhenFinalApproachIsMisaligned)
{
  auto cmd = interactive_cleanup::computeFinalApproachCommand(
    0.80, 0.30, 0.45, 0.15, 0.08, 0.35);

  EXPECT_FALSE(cmd.reached);
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_GT(cmd.angular_z, 0.0);
}

TEST(NavigationUtilsTest, ComputesForwardFallbackApproachToStoredTarget)
{
  interactive_cleanup::PlanarPose robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;

  auto cmd = interactive_cleanup::computeApproachCommandToTarget(
    robot_pose, 1.0, 0.0, 0.45, 0.15, 0.06, 0.25);

  EXPECT_FALSE(cmd.reached);
  EXPECT_GT(cmd.linear_x, 0.0);
  EXPECT_NEAR(cmd.angular_z, 0.0, 1e-6);
}

TEST(NavigationUtilsTest, MarksFallbackApproachReachedNearStoredTarget)
{
  interactive_cleanup::PlanarPose robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;

  auto cmd = interactive_cleanup::computeApproachCommandToTarget(
    robot_pose, 0.40, 0.0, 0.45, 0.15, 0.06, 0.25);

  EXPECT_TRUE(cmd.reached);
  EXPECT_DOUBLE_EQ(cmd.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.angular_z, 0.0);
}

TEST(NavigationUtilsTest, ExtendsApproachTimeoutForLongFallbackDistances)
{
  const double timeout = interactive_cleanup::computeApproachTimeout(
    0.95, 0.45, 0.06, 8.0, 20.0, 1.0);

  EXPECT_GT(timeout, 12.0);
  EXPECT_LT(timeout, 20.0);
}

TEST(NavigationUtilsTest, KeepsMinimumApproachTimeoutWhenAlreadyNearGoal)
{
  const double timeout = interactive_cleanup::computeApproachTimeout(
    0.50, 0.45, 0.06, 8.0, 20.0, 1.0);

  EXPECT_DOUBLE_EQ(timeout, 8.0);
}

TEST(NavigationUtilsTest, AcceptsCloseEnoughFallbackDistance)
{
  EXPECT_TRUE(interactive_cleanup::isWithinApproachTolerance(
    0.52, 0.45, 0.08));
}

TEST(NavigationUtilsTest, RejectsDistanceOutsideFallbackTolerance)
{
  EXPECT_FALSE(interactive_cleanup::isWithinApproachTolerance(
    0.56, 0.45, 0.08));
}
