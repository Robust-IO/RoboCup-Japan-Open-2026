#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"

namespace protocol = handyman_rebuild_ros2::protocol;

namespace
{

handyman_msgs::msg::HandymanMsg makeMessage(
  const std::string & message, const std::string & detail = "")
{
  handyman_msgs::msg::HandymanMsg result;
  result.message = message;
  result.detail = detail;
  return result;
}

}  // namespace

TEST(CompetitionProtocolTest, ParsesEveryOfficialIncomingEvent)
{
  using Event = protocol::CompetitionEvent;
  const std::vector<std::pair<handyman_msgs::msg::HandymanMsg, Event>> cases{
    {makeMessage("Environment", "LayoutA"), Event::kEnvironment},
    {makeMessage("Are_you_ready?"), Event::kAreYouReady},
    {makeMessage("Instruction", "Go to the kitchen."), Event::kInstruction},
    {makeMessage("Corrected_instruction", "Find the apple."), Event::kCorrectedInstruction},
    {makeMessage("Task_succeeded"), Event::kTaskSucceeded},
    {makeMessage("Task_failed", "Time_is_up"), Event::kTaskFailed},
    {makeMessage("Mission_complete"), Event::kMissionComplete},
  };

  for (const auto & test_case : cases) {
    const auto parsed = protocol::parseIncomingMessage(test_case.first);
    ASSERT_TRUE(parsed.valid) << parsed.reason;
    EXPECT_EQ(parsed.message.event, test_case.second);
    EXPECT_EQ(parsed.message.direction, protocol::MessageDirection::kToRobot);
    EXPECT_EQ(parsed.message.detail, test_case.first.detail);
  }
}

TEST(CompetitionProtocolTest, BuildsEveryOfficialOutgoingEvent)
{
  using Event = protocol::CompetitionEvent;
  const std::vector<std::pair<Event, std::string>> cases{
    {Event::kIAmReady, "I_am_ready"},
    {Event::kRoomReached, "Room_reached"},
    {Event::kDoesNotExist, "Does_not_exist"},
    {Event::kObjectGrasped, "Object_grasped"},
    {Event::kTaskFinished, "Task_finished"},
    {Event::kGiveUp, "Give_up"},
  };

  for (const auto & test_case : cases) {
    const auto built = protocol::makeOutgoingMessage(test_case.first);
    ASSERT_TRUE(built.valid) << built.reason;
    EXPECT_EQ(built.message.message, test_case.second);
    EXPECT_TRUE(built.message.detail.empty());
  }
}

TEST(CompetitionProtocolTest, RequiresDetailWhereOfficialProtocolRequiresPayload)
{
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Environment")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Instruction")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Corrected_instruction")).valid);
  EXPECT_TRUE(protocol::parseIncomingMessage(makeMessage("Are_you_ready?")).valid);
}

TEST(CompetitionProtocolTest, RejectsUnknownAndIncorrectlyCasedEvents)
{
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("instruction", "task")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Task-Succeeded")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Unknown_event")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("")).valid);
}

TEST(CompetitionProtocolTest, EnforcesMessageDirection)
{
  using Event = protocol::CompetitionEvent;
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("I_am_ready")).valid);
  EXPECT_FALSE(protocol::parseIncomingMessage(makeMessage("Room_reached")).valid);
  EXPECT_FALSE(protocol::makeOutgoingMessage(Event::kInstruction).valid);
  EXPECT_FALSE(protocol::makeOutgoingMessage(Event::kTaskSucceeded).valid);
  EXPECT_FALSE(protocol::makeOutgoingMessage(Event::kUnknown).valid);
}

TEST(CompetitionProtocolTest, RejectsUnexpectedOutgoingDetail)
{
  const auto built = protocol::makeOutgoingMessage(
    protocol::CompetitionEvent::kGiveUp, "navigation unavailable");
  EXPECT_FALSE(built.valid);
}
