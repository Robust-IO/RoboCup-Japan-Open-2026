#include <gtest/gtest.h>

#include "handyman_ros2/room_navigation.hpp"

TEST(RoomNavigationTest, RobotAlreadyInRoomSkipsNavigationAndRequestsRoomReached)
{
  const auto patrol_points = handyman_ros2::roomPatrolWaypoints("Layout2020HM01", "living");
  const auto decision = handyman_ros2::decideRoomEntryAction(
    false, false, 0.5, 2.0, patrol_points, 3.5);

  EXPECT_TRUE(decision.mark_room_reached);
  EXPECT_TRUE(decision.send_room_reached_message);
  EXPECT_FALSE(decision.should_send_navigation_goal);
}

TEST(RoomNavigationTest, RobotOutsideRoomRequestsNavigation)
{
  const auto patrol_points = handyman_ros2::roomPatrolWaypoints("Layout2020HM01", "living");
  const auto decision = handyman_ros2::decideRoomEntryAction(
    false, false, 20.0, 20.0, patrol_points, 3.5);

  EXPECT_FALSE(decision.mark_room_reached);
  EXPECT_FALSE(decision.send_room_reached_message);
  EXPECT_TRUE(decision.should_send_navigation_goal);
}

TEST(RoomNavigationTest, ExistingRoomReachedBlocksFurtherNavigationDispatch)
{
  const auto patrol_points = handyman_ros2::roomPatrolWaypoints("Layout2020HM01", "living");
  const auto decision = handyman_ros2::decideRoomEntryAction(
    true, false, 20.0, 20.0, patrol_points, 3.5);

  EXPECT_FALSE(decision.mark_room_reached);
  EXPECT_FALSE(decision.send_room_reached_message);
  EXPECT_FALSE(decision.should_send_navigation_goal);
}
