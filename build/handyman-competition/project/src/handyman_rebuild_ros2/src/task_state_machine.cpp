#include "handyman_rebuild_ros2/task_state_machine.hpp"

namespace handyman_rebuild_ros2
{

TaskState TaskStateMachine::state() const noexcept { return state_; }
const HandymanTask & TaskStateMachine::task() const noexcept { return task_; }

void TaskStateMachine::bootCompleted()
{
  if (state_ == TaskState::kBooting) {
    state_ = TaskState::kWaitingReady;
  }
}

void TaskStateMachine::setEnvironment(const std::string & environment)
{
  task_.environment = environment;
}

bool TaskStateMachine::acceptReady()
{
  if (state_ != TaskState::kWaitingReady || task_.environment.empty()) {
    return false;
  }
  state_ = TaskState::kWaitingInstruction;
  return true;
}

bool TaskStateMachine::acceptInstruction(const std::string & instruction, bool corrected)
{
  const bool initial = state_ == TaskState::kWaitingInstruction;
  const bool correction = corrected &&
    (state_ == TaskState::kSearchingObject || state_ == TaskState::kRecovering);
  if (instruction.empty() || (!initial && !correction)) {
    return false;
  }
  task_.raw_instruction = instruction;
  task_.pickup_room.clear();
  task_.target_object.clear();
  task_.destination_room.clear();
  task_.destination.clear();
  state_ = TaskState::kParsingInstruction;
  return true;
}

bool TaskStateMachine::parsingSucceeded()
{
  return parsingSucceeded(task_);
}

bool TaskStateMachine::parsingSucceeded(const HandymanTask & parsed_task)
{
  if (state_ != TaskState::kParsingInstruction) {
    return false;
  }
  task_ = parsed_task;
  state_ = TaskState::kNavigatingToRoom;
  return true;
}

bool TaskStateMachine::parsingFailed()
{
  if (state_ != TaskState::kParsingInstruction) {
    return false;
  }
  state_ = TaskState::kRecovering;
  return true;
}

bool TaskStateMachine::roomNavigationSucceeded()
{
  if (state_ != TaskState::kNavigatingToRoom) {
    return false;
  }
  state_ = TaskState::kVerifyingRoom;
  return true;
}

bool TaskStateMachine::roomVerified()
{
  if (state_ != TaskState::kVerifyingRoom) {
    return false;
  }
  state_ = TaskState::kSearchingObject;
  return true;
}

bool TaskStateMachine::objectLocated()
{
  if (state_ != TaskState::kSearchingObject) {
    return false;
  }
  state_ = TaskState::kApproachingObject;
  return true;
}

bool TaskStateMachine::approachSucceeded()
{
  if (state_ != TaskState::kApproachingObject) {
    return false;
  }
  state_ = TaskState::kGrasping;
  return true;
}

bool TaskStateMachine::graspSucceeded()
{
  if (state_ != TaskState::kGrasping) {
    return false;
  }
  state_ = TaskState::kVerifyingGrasp;
  return true;
}

bool TaskStateMachine::graspVerified()
{
  if (state_ != TaskState::kVerifyingGrasp) {
    return false;
  }
  state_ = TaskState::kNavigatingToDestination;
  return true;
}

bool TaskStateMachine::destinationReached()
{
  if (state_ != TaskState::kNavigatingToDestination) {
    return false;
  }
  state_ = TaskState::kPlacing;
  return true;
}

bool TaskStateMachine::placementSucceeded()
{
  if (state_ != TaskState::kPlacing) {
    return false;
  }
  state_ = TaskState::kVerifyingPlacement;
  return true;
}

bool TaskStateMachine::placementVerified()
{
  if (state_ != TaskState::kVerifyingPlacement) {
    return false;
  }
  state_ = TaskState::kWaitingModeratorResult;
  return true;
}

bool TaskStateMachine::objectNotFound()
{
  if (state_ != TaskState::kSearchingObject) {
    return false;
  }
  state_ = TaskState::kRecovering;
  return true;
}

bool TaskStateMachine::giveUp()
{
  switch (state_) {
    case TaskState::kParsingInstruction:
    case TaskState::kNavigatingToRoom:
    case TaskState::kVerifyingRoom:
    case TaskState::kSearchingObject:
    case TaskState::kApproachingObject:
    case TaskState::kGrasping:
    case TaskState::kVerifyingGrasp:
    case TaskState::kNavigatingToDestination:
    case TaskState::kPlacing:
    case TaskState::kVerifyingPlacement:
    case TaskState::kRecovering:
      state_ = TaskState::kWaitingModeratorResult;
      return true;
    default:
      return false;
  }
}

bool TaskStateMachine::moderatorSucceeded()
{
  if (state_ != TaskState::kWaitingModeratorResult) {
    return false;
  }
  state_ = TaskState::kWaitingReady;
  task_ = HandymanTask{};
  return true;
}

bool TaskStateMachine::moderatorFailed()
{
  if (
    state_ == TaskState::kBooting || state_ == TaskState::kWaitingReady ||
    state_ == TaskState::kFinished)
  {
    return false;
  }
  state_ = TaskState::kWaitingReady;
  task_ = HandymanTask{};
  return true;
}

bool TaskStateMachine::missionCompleted()
{
  if (state_ == TaskState::kFinished) {
    return false;
  }
  state_ = TaskState::kFinished;
  return true;
}

}  // namespace handyman_rebuild_ros2
