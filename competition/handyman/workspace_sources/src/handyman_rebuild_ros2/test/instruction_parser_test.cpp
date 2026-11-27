#include <gtest/gtest.h>

#include <string>

#include "handyman_rebuild_ros2/instruction_parser.hpp"

using handyman_rebuild_ros2::HandymanTask;
using handyman_rebuild_ros2::NameAliases;
using handyman_rebuild_ros2::RuleBasedInstructionParser;

namespace
{

RuleBasedInstructionParser makeParser()
{
  NameAliases aliases;
  std::string error;
  EXPECT_TRUE(aliases.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/name_aliases.yaml", error)) << error;
  return RuleBasedInstructionParser(std::move(aliases));
}

}  // namespace

TEST(InstructionParserTest, ParsesOfficialStyleInstruction)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "Go to the kitchen, grasp the apple and bring it to the dining table.", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.pickup_room, "kitchen");
  EXPECT_EQ(task.target_object, "apple");
  EXPECT_TRUE(task.destination_room.empty());
  EXPECT_EQ(task.destination, "dining_table");
  EXPECT_FALSE(task.destination_is_avatar);
  EXPECT_EQ(task.raw_instruction, "Go to the kitchen, grasp the apple and bring it to the dining table.");
}

TEST(InstructionParserTest, UsesFirstRoomForPickupAndLastForDestination)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "From the bedroom take the toy penguin to the white side table in the living room.", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.pickup_room, "bedroom");
  EXPECT_EQ(task.destination_room, "living_room");
  EXPECT_EQ(task.target_object, "toy_penguin");
  EXPECT_EQ(task.destination, "white_side_table");
}

TEST(InstructionParserTest, RecognizesAvatarHandover)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "Go to the lobby, grasp the camera and bring it here.", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.destination, "avatar");
  EXPECT_TRUE(task.destination_is_avatar);
  EXPECT_TRUE(task.destination_room.empty());
}

TEST(InstructionParserTest, NormalizesCaseHyphensAndPunctuation)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "In the LIVING-ROOM, pick up the PINK-CUP; put it on the SQUARE-LOW-TABLE!", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.pickup_room, "living_room");
  EXPECT_EQ(task.target_object, "pink_cup");
  EXPECT_EQ(task.destination, "square_low_table");
}

TEST(InstructionParserTest, DoesNotReusePickupRoomAsImplicitDestinationRoom)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "Go to the kitchen, grasp the apple and bring it to the dining table.", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.pickup_room, "kitchen");
  EXPECT_TRUE(task.destination_room.empty());
}

TEST(InstructionParserTest, PrefersSpecificObjectOverContainedAlias)
{
  auto parser = makeParser();
  HandymanTask task;
  const auto result = parser.parse(
    "Go to the kitchen and take the empty ketchup to the wagon.", task);
  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(task.target_object, "empty_ketchup");
}

TEST(InstructionParserTest, RejectsIncompleteAndUnknownInstructions)
{
  auto parser = makeParser();
  HandymanTask task;
  EXPECT_FALSE(parser.parse("", task).success);
  EXPECT_FALSE(parser.parse("Bring the apple to the table.", task).success);
  EXPECT_FALSE(parser.parse("Go to the kitchen and bring the moon rock here.", task).success);
  EXPECT_FALSE(parser.parse("Go to the kitchen and bring the apple somewhere.", task).success);
}
