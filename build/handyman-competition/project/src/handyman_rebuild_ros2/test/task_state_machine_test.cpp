#include <gtest/gtest.h>

#include "handyman_rebuild_ros2/task_state_machine.hpp"

using handyman_rebuild_ros2::TaskState;
using handyman_rebuild_ros2::TaskStateMachine;

TEST(TaskStateMachineTest, RequiresEnvironmentBeforeReady)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  EXPECT_FALSE(machine.acceptReady());
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);

  machine.setEnvironment("LayoutA");
  EXPECT_TRUE(machine.acceptReady());
  EXPECT_EQ(machine.state(), TaskState::kWaitingInstruction);
}

TEST(TaskStateMachineTest, AcceptsNonEmptyInstructionOnlyInExpectedState)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("LayoutA");
  ASSERT_TRUE(machine.acceptReady());

  EXPECT_FALSE(machine.acceptInstruction(""));
  EXPECT_TRUE(machine.acceptInstruction("Go to the kitchen and grasp the apple."));
  EXPECT_EQ(machine.state(), TaskState::kParsingInstruction);
  EXPECT_EQ(machine.task().raw_instruction, "Go to the kitchen and grasp the apple.");
}

TEST(TaskStateMachineTest, StoresParsedTaskAndRecoversFromParseFailure)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("LayoutA");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the apple here."));

  handyman_rebuild_ros2::HandymanTask parsed = machine.task();
  parsed.pickup_room = "kitchen";
  parsed.target_object = "apple";
  parsed.destination = "avatar";
  parsed.destination_is_avatar = true;
  ASSERT_TRUE(machine.parsingSucceeded(parsed));
  EXPECT_EQ(machine.task().pickup_room, "kitchen");
  EXPECT_EQ(machine.task().target_object, "apple");

  machine.moderatorFailed();
  machine.setEnvironment("LayoutA");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("unknown"));
  EXPECT_TRUE(machine.parsingFailed());
  EXPECT_EQ(machine.state(), TaskState::kRecovering);
}

TEST(TaskStateMachineTest, CompletesHappyPathWithoutSkippingStates)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("Environment_01");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the apple to the table."));

  EXPECT_FALSE(machine.roomNavigationSucceeded());
  ASSERT_TRUE(machine.parsingSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kNavigatingToRoom);
  ASSERT_TRUE(machine.roomNavigationSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kVerifyingRoom);
  ASSERT_TRUE(machine.roomVerified());
  EXPECT_EQ(machine.state(), TaskState::kSearchingObject);
  ASSERT_TRUE(machine.objectLocated());
  EXPECT_EQ(machine.state(), TaskState::kApproachingObject);
  ASSERT_TRUE(machine.approachSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kGrasping);
  ASSERT_TRUE(machine.graspSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kVerifyingGrasp);
  ASSERT_TRUE(machine.graspVerified());
  EXPECT_EQ(machine.state(), TaskState::kNavigatingToDestination);
  ASSERT_TRUE(machine.destinationReached());
  EXPECT_EQ(machine.state(), TaskState::kPlacing);
  ASSERT_TRUE(machine.placementSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kVerifyingPlacement);
  ASSERT_TRUE(machine.placementVerified());
  EXPECT_EQ(machine.state(), TaskState::kWaitingModeratorResult);
  ASSERT_TRUE(machine.moderatorSucceeded());
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);
  EXPECT_TRUE(machine.task().raw_instruction.empty());
}

TEST(TaskStateMachineTest, ObjectMissingAllowsCorrectedInstruction)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("Environment_02");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Find the apple."));
  ASSERT_TRUE(machine.parsingSucceeded());
  ASSERT_TRUE(machine.roomNavigationSucceeded());
  ASSERT_TRUE(machine.roomVerified());

  ASSERT_TRUE(machine.objectNotFound());
  EXPECT_EQ(machine.state(), TaskState::kRecovering);
  EXPECT_FALSE(machine.acceptInstruction("Find the cup."));
  EXPECT_TRUE(machine.acceptInstruction("Find the cup.", true));
  EXPECT_EQ(machine.state(), TaskState::kParsingInstruction);
  EXPECT_EQ(machine.task().raw_instruction, "Find the cup.");
}

TEST(TaskStateMachineTest, GiveUpWaitsForModeratorResult)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("Environment_03");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the cup."));
  ASSERT_TRUE(machine.parsingSucceeded());

  ASSERT_TRUE(machine.giveUp());
  EXPECT_EQ(machine.state(), TaskState::kWaitingModeratorResult);
  EXPECT_FALSE(machine.giveUp());
  EXPECT_TRUE(machine.moderatorFailed());
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);
}

TEST(TaskStateMachineTest, ModeratorFailureCanInterruptAnActiveCheckpoint)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("Environment_04");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the bottle."));
  ASSERT_TRUE(machine.parsingSucceeded());
  ASSERT_TRUE(machine.roomNavigationSucceeded());
  ASSERT_TRUE(machine.roomVerified());

  EXPECT_TRUE(machine.moderatorFailed());
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);
  EXPECT_TRUE(machine.task().environment.empty());
}

TEST(TaskStateMachineTest, FailureResetsTaskSafely)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("LayoutB");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the cup here."));

  machine.moderatorFailed();
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);
  EXPECT_TRUE(machine.task().raw_instruction.empty());
  EXPECT_TRUE(machine.task().environment.empty());
}

TEST(TaskStateMachineTest, MissionCompleteIsTerminal)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.missionCompleted();
  EXPECT_EQ(machine.state(), TaskState::kFinished);
}
