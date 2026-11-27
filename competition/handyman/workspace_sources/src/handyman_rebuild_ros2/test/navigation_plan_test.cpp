#include <gtest/gtest.h>

#include "handyman_rebuild_ros2/navigation_plan.hpp"

namespace handyman_rebuild_ros2
{
namespace
{

EnvironmentConfig sampleEnvironment()
{
  EnvironmentConfig environment;
  environment.name = "LayoutA";
  RoomConfig kitchen;
  kitchen.region = {{0.0, 0.0}, {0.0, 5.0}, {5.0, 5.0}, {5.0, 0.0}};
  kitchen.search_points = {{1.0, 1.0, 0.0}, {4.0, 4.0, 1.0}};
  environment.rooms.emplace("kitchen", kitchen);
  environment.destinations["dining_table"] = {
    {"kitchen", {1.5, 2.0, 0.0}},
    {"kitchen", {4.0, 3.5, 3.14}},
  };
  return environment;
}

TEST(NavigationPlan, RoomCandidatesStartAtNearestPoseAndRetryAnotherCandidate)
{
  const auto environment = sampleEnvironment();
  auto plan = NavigationPlan::forRoom(environment, "kitchen", 3, Pose2D{4.2, 4.1, 0.0});
  ASSERT_TRUE(plan.valid());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->source_index, 1U);
  EXPECT_EQ(plan.attemptNumber(), 1U);
  ASSERT_TRUE(plan.advance());
  EXPECT_EQ(plan.current()->source_index, 0U);
  EXPECT_EQ(plan.attemptNumber(), 2U);
  ASSERT_TRUE(plan.advance());
  EXPECT_EQ(plan.current()->source_index, 1U);
  EXPECT_FALSE(plan.advance());
  EXPECT_TRUE(plan.exhausted());
}

TEST(NavigationPlan, SearchPointIsFixedAcrossRetries)
{
  auto environment = sampleEnvironment();
  auto plan = NavigationPlan::forSearchPoint(environment, "kitchen", 1, 3);
  ASSERT_TRUE(plan.valid());
  do {
    ASSERT_NE(plan.current(), nullptr);
    EXPECT_EQ(plan.current()->source_index, 1U);
    EXPECT_DOUBLE_EQ(plan.current()->pose.x, 4.0);
  } while (plan.advance());
  EXPECT_FALSE(NavigationPlan::forSearchPoint(environment, "kitchen", 2, 3).valid());
  EXPECT_FALSE(NavigationPlan::forSearchPoint(environment, "missing", 0, 3).valid());
  environment.rooms.at("kitchen").search_points[0].x = 100;
  EXPECT_FALSE(NavigationPlan::forSearchPoint(environment, "kitchen", 0, 3).valid());
}

TEST(NavigationPlan, DestinationCandidatesAreFilteredByRoom)
{
  auto environment = sampleEnvironment();
  environment.rooms["lobby"] = environment.rooms.at("kitchen");
  environment.destinations["dining_table"].push_back({"lobby", {2.0, 2.0, 0.0}});
  auto plan = NavigationPlan::forDestination(
    environment, "dining_table", "lobby", 2);
  ASSERT_TRUE(plan.valid());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->room, "lobby");
  EXPECT_EQ(plan.current()->source_index, 2U);
}

TEST(NavigationPlan, CrossRoomPlanUsesConfiguredWaypointsAndKeepsSafeCandidateOrder)
{
  auto environment = sampleEnvironment();
  RoomConfig bedroom;
  bedroom.region = {{6.0, 0.0}, {6.0, 5.0}, {11.0, 5.0}, {11.0, 0.0}};
  bedroom.search_points = {{9.0, 2.5, 0.0}, {6.5, 1.0, 0.0}};
  environment.rooms.emplace("bedroom", bedroom);
  environment.routes.push_back({"kitchen", "bedroom", {{5.5, 2.0, 0.0}, {6.5, 2.0, 0.0}}});

  auto plan = NavigationPlan::forRoom(
    environment, "bedroom", 2, Pose2D{4.5, 1.0, 0.0});
  ASSERT_TRUE(plan.valid());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->source_index, 0U);
  ASSERT_EQ(plan.current()->route_waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(plan.current()->route_waypoints[0].x, 5.5);
}

TEST(NavigationPlan, FailedAttemptAdvancesToAlternateDoorwayBeforeNextTarget)
{
  auto environment = sampleEnvironment();
  RoomConfig bedroom;
  bedroom.region = {{6.0, 0.0}, {6.0, 5.0}, {11.0, 5.0}, {11.0, 0.0}};
  bedroom.search_points = {{9.0, 2.5, 0.0}, {6.5, 1.0, 0.0}};
  environment.rooms.emplace("bedroom", bedroom);
  environment.routes.push_back(
    {"kitchen", "bedroom", {{5.5, 2.0, 0.0}, {6.5, 2.0, 0.0}}});
  environment.routes.push_back(
    {"kitchen", "bedroom", {{5.5, 4.0, 0.0}, {6.5, 4.0, 0.0}}});

  auto plan = NavigationPlan::forRoom(
    environment, "bedroom", 3, Pose2D{4.5, 1.0, 0.0});
  ASSERT_TRUE(plan.valid());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->source_index, 0U);
  EXPECT_DOUBLE_EQ(plan.current()->route_waypoints[0].y, 2.0);
  ASSERT_TRUE(plan.advance());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->source_index, 0U);
  EXPECT_DOUBLE_EQ(plan.current()->route_waypoints[0].y, 4.0);
  ASSERT_TRUE(plan.advance());
  ASSERT_NE(plan.current(), nullptr);
  EXPECT_EQ(plan.current()->source_index, 1U);
  EXPECT_DOUBLE_EQ(plan.current()->route_waypoints[0].y, 2.0);
}

TEST(NavigationPlan, RejectsMissingRoomAndDestinationCandidate)
{
  const auto environment = sampleEnvironment();
  EXPECT_FALSE(NavigationPlan::forRoom(environment, "bedroom", 3).valid());
  EXPECT_FALSE(
    NavigationPlan::forDestination(environment, "dining_table", "bedroom", 3).valid());
  EXPECT_FALSE(NavigationPlan::forRoom(environment, "kitchen", 0).valid());
}

TEST(NavigationPlan, RoomVerificationUsesPolygonAndSmallBoundaryTolerance)
{
  const auto room = sampleEnvironment().rooms.at("kitchen");
  EXPECT_TRUE(isPoseInsideRoom(room, {2.0, 2.0, 0.0}));
  EXPECT_TRUE(isPoseInsideRoom(room, {0.0, 2.0, 0.0}));
  EXPECT_FALSE(isPoseInsideRoom(room, {-0.25, 2.0, 0.0}, 0.20));
  EXPECT_TRUE(isPoseInsideRoom(room, {-0.10, 2.0, 0.0}, 0.20));
}

}  // namespace
}  // namespace handyman_rebuild_ros2
