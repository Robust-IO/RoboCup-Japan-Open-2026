#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "handyman_ros2/placement_navigation.hpp"

TEST(PlacementNavigationTest, UsesLastMentionFromInstruction)
{
  const std::string instruction =
    "First visit the bedroom, then the lobby, and finally head to the living room.";
  const auto rooms = std::vector<std::string>{"bedroom"};

  EXPECT_EQ(
    handyman_ros2::resolveDestinationRoom(instruction, rooms, "bedroom"),
    "living");
}

TEST(PlacementNavigationTest, HandlesLivingRoomAliasFromRooms)
{
  const std::string instruction = "Bring it to the living room, please.";
  const auto rooms = std::vector<std::string>{"living room"};

  EXPECT_EQ(
    handyman_ros2::resolveDestinationRoom(instruction, rooms, "bedroom"),
    "living");
}

TEST(PlacementNavigationTest, NormalizesPunctuationAndCasing)
{
  const std::string instruction = "Deliver to the LIVing-RooM!";
  const auto rooms = std::vector<std::string>{"living"};

  EXPECT_EQ(
    handyman_ros2::resolveDestinationRoom(instruction, rooms, "bedroom"),
    "living");
}

TEST(PlacementNavigationTest, FallsBackToRoomsVector)
{
  const std::string instruction = "Drop it somewhere."
    " There is no clear room mention here.";
  const auto rooms = std::vector<std::string>{"bedroom", "living"};

  EXPECT_EQ(
    handyman_ros2::resolveDestinationRoom(instruction, rooms, "bedroom"),
    "living");
}

TEST(PlacementNavigationTest, FallsBackToPickupRoomWhenNoRooms)
{
  const std::string instruction = "Leave it on the floor.";
  const std::vector<std::string> rooms{};

  EXPECT_EQ(
    handyman_ros2::resolveDestinationRoom(instruction, rooms, "kitchen"),
    "kitchen");
}

TEST(PlacementNavigationTest, SquareLowTableCandidatesIncludeOrderedYaw)
{
  const auto candidates = handyman_ros2::placementCandidates(
    "Layout2019HM01", "square_low_table", "living");

  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_DOUBLE_EQ(candidates[0].x, 0.774);
  EXPECT_DOUBLE_EQ(candidates[0].y, 2.79);
  EXPECT_DOUBLE_EQ(candidates[0].yaw, 0.8);
  EXPECT_DOUBLE_EQ(candidates[1].x, 1.10);
  EXPECT_DOUBLE_EQ(candidates[1].y, 2.35);
  EXPECT_DOUBLE_EQ(candidates[1].yaw, 1.20);
  EXPECT_DOUBLE_EQ(candidates[2].x, 0.30);
  EXPECT_DOUBLE_EQ(candidates[2].y, 2.35);
  EXPECT_DOUBLE_EQ(candidates[2].yaw, 0.20);
}

TEST(PlacementNavigationTest, SquareLowTableCandidatesAcceptLivingRoomAlias)
{
  const auto candidates = handyman_ros2::placementCandidates(
    "layout2019hm01", "square_low_table", "Living Room");

  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_DOUBLE_EQ(candidates[0].x, 0.774);
}

TEST(PlacementNavigationTest, TrashBoxCandidatesOrdered)
{
  const auto candidates = handyman_ros2::placementCandidates(
    "Layout2019HM01", "trash_box_for_bottle_can", "living");

  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_DOUBLE_EQ(candidates[0].x, -0.8);
  EXPECT_DOUBLE_EQ(candidates[0].y, -2.0);
  EXPECT_DOUBLE_EQ(candidates[0].yaw, 3.14);
  EXPECT_DOUBLE_EQ(candidates[1].x, -0.2);
  EXPECT_DOUBLE_EQ(candidates[1].y, -2.0);
  EXPECT_DOUBLE_EQ(candidates[2].x, -0.8);
  EXPECT_DOUBLE_EQ(candidates[2].y, -1.4);
}

TEST(PlacementNavigationTest, UnknownDestinationReturnsEmptyCandidates)
{
  const auto candidates = handyman_ros2::placementCandidates(
    "Layout2019HM01", "unknown_destination", "living");

  EXPECT_TRUE(candidates.empty());
}

TEST(PlacementNavigationTest, NextPlacementCandidateIndexHandlesEdges)
{
  EXPECT_EQ(handyman_ros2::nextPlacementCandidateIndex(0, 0), 0u);
  EXPECT_EQ(handyman_ros2::nextPlacementCandidateIndex(2, 1), 0u);
  EXPECT_EQ(handyman_ros2::nextPlacementCandidateIndex(0, 3), 1u);
  EXPECT_EQ(handyman_ros2::nextPlacementCandidateIndex(2, 3), 0u);
}
