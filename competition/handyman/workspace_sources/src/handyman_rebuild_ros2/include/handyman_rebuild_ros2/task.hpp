#pragma once

#include <cstdint>
#include <string>

namespace handyman_rebuild_ros2
{

enum class TaskState : std::uint8_t
{
  kBooting,
  kWaitingReady,
  kWaitingInstruction,
  kParsingInstruction,
  kNavigatingToRoom,
  kVerifyingRoom,
  kSearchingObject,
  kApproachingObject,
  kGrasping,
  kVerifyingGrasp,
  kNavigatingToDestination,
  kPlacing,
  kVerifyingPlacement,
  kWaitingModeratorResult,
  kRecovering,
  kFinished,
};

struct HandymanTask
{
  std::string task_id;
  std::string raw_instruction;
  std::string environment;
  std::string pickup_room;
  std::string target_object;
  std::string destination_room;
  std::string destination;
  bool destination_is_avatar{false};
  int navigation_attempts{0};
  int search_attempts{0};
  int grasp_attempts{0};
  int placement_attempts{0};
};

}  // namespace handyman_rebuild_ros2
