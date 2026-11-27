#include "handyman_rebuild_ros2/competition_protocol.hpp"

#include <array>
#include <utility>

namespace handyman_rebuild_ros2::protocol
{
namespace
{

struct ProtocolEntry
{
  enum class DetailPolicy : std::uint8_t {kBlank, kRequired, kOptional};
  CompetitionEvent event;
  std::string_view wire_name;
  MessageDirection direction;
  DetailPolicy detail_policy;
};

constexpr std::array<ProtocolEntry, 13> kProtocolEntries{{
  {CompetitionEvent::kEnvironment, "Environment", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kRequired},
  {CompetitionEvent::kAreYouReady, "Are_you_ready?", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kInstruction, "Instruction", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kRequired},
  {CompetitionEvent::kCorrectedInstruction, "Corrected_instruction", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kRequired},
  {CompetitionEvent::kTaskSucceeded, "Task_succeeded", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kTaskFailed, "Task_failed", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kOptional},
  {CompetitionEvent::kMissionComplete, "Mission_complete", MessageDirection::kToRobot, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kIAmReady, "I_am_ready", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kRoomReached, "Room_reached", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kDoesNotExist, "Does_not_exist", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kObjectGrasped, "Object_grasped", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kTaskFinished, "Task_finished", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
  {CompetitionEvent::kGiveUp, "Give_up", MessageDirection::kToModerator, ProtocolEntry::DetailPolicy::kBlank},
}};

const ProtocolEntry * findByWireName(const std::string & wire_name) noexcept
{
  for (const auto & entry : kProtocolEntries) {
    if (wire_name == entry.wire_name) {
      return &entry;
    }
  }
  return nullptr;
}

const ProtocolEntry * findByEvent(CompetitionEvent event) noexcept
{
  for (const auto & entry : kProtocolEntries) {
    if (entry.event == event) {
      return &entry;
    }
  }
  return nullptr;
}

BuildResult invalidBuildResult(const std::string & reason)
{
  BuildResult result;
  result.reason = reason;
  return result;
}

}  // namespace

ParseResult parseIncomingMessage(const handyman_msgs::msg::HandymanMsg & ros_message)
{
  const ProtocolEntry * entry = findByWireName(ros_message.message);
  if (entry == nullptr) {
    return {false, {}, "Unknown competition event: " + ros_message.message};
  }
  if (entry->direction != MessageDirection::kToRobot) {
    return {false, {}, "Event is not valid on the to_robot topic: " + ros_message.message};
  }
  if (entry->detail_policy == ProtocolEntry::DetailPolicy::kRequired && ros_message.detail.empty()) {
    return {false, {}, "Event requires non-empty detail: " + ros_message.message};
  }
  if (entry->detail_policy == ProtocolEntry::DetailPolicy::kBlank && !ros_message.detail.empty()) {
    return {false, {}, "Event requires blank detail: " + ros_message.message};
  }
  return {true, {entry->event, entry->direction, ros_message.detail}, ""};
}

BuildResult makeOutgoingMessage(CompetitionEvent event, const std::string & detail)
{
  const ProtocolEntry * entry = findByEvent(event);
  if (entry == nullptr || event == CompetitionEvent::kUnknown) {
    return invalidBuildResult("Unknown competition event cannot be sent");
  }
  if (entry->direction != MessageDirection::kToModerator) {
    return invalidBuildResult("Incoming-only event cannot be sent to the moderator");
  }
  if (entry->detail_policy == ProtocolEntry::DetailPolicy::kBlank && !detail.empty()) {
    return invalidBuildResult("Official outgoing event requires blank detail");
  }

  handyman_msgs::msg::HandymanMsg ros_message;
  ros_message.message = std::string(entry->wire_name);
  ros_message.detail = detail;
  return {true, std::move(ros_message), ""};
}

std::string_view eventName(CompetitionEvent event) noexcept
{
  const ProtocolEntry * entry = findByEvent(event);
  return entry == nullptr ? std::string_view{"Unknown"} : entry->wire_name;
}

}  // namespace handyman_rebuild_ros2::protocol
