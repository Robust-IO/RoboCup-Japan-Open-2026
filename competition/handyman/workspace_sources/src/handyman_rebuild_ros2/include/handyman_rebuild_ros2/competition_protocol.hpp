#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include <handyman_msgs/msg/handyman_msg.hpp>

namespace handyman_rebuild_ros2::protocol
{

inline constexpr std::string_view kToRobotTopic = "/handyman/message/to_robot";
inline constexpr std::string_view kToModeratorTopic = "/handyman/message/to_moderator";

enum class MessageDirection : std::uint8_t
{
  kToRobot,
  kToModerator,
};

enum class CompetitionEvent : std::uint8_t
{
  kEnvironment,
  kAreYouReady,
  kInstruction,
  kCorrectedInstruction,
  kTaskSucceeded,
  kTaskFailed,
  kMissionComplete,
  kIAmReady,
  kRoomReached,
  kDoesNotExist,
  kObjectGrasped,
  kTaskFinished,
  kGiveUp,
  kUnknown,
};

struct CompetitionMessage
{
  CompetitionEvent event{CompetitionEvent::kUnknown};
  MessageDirection direction{MessageDirection::kToRobot};
  std::string detail;
};

struct ParseResult
{
  bool valid{false};
  CompetitionMessage message{};
  std::string reason;
};

struct BuildResult
{
  bool valid{false};
  handyman_msgs::msg::HandymanMsg message{};
  std::string reason;
};

ParseResult parseIncomingMessage(const handyman_msgs::msg::HandymanMsg & ros_message);
BuildResult makeOutgoingMessage(CompetitionEvent event, const std::string & detail = "");
std::string_view eventName(CompetitionEvent event) noexcept;

}  // namespace handyman_rebuild_ros2::protocol
