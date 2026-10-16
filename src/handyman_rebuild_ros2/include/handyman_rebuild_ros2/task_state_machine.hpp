#pragma once

#include <string>

#include "handyman_rebuild_ros2/task.hpp"

namespace handyman_rebuild_ros2
{

class TaskStateMachine
{
public:
  TaskState state() const noexcept;
  const HandymanTask & task() const noexcept;

  void bootCompleted();
  void setEnvironment(const std::string & environment);
  bool acceptReady();
  bool acceptInstruction(const std::string & instruction, bool corrected = false);
  bool parsingSucceeded();
  bool parsingSucceeded(const HandymanTask & parsed_task);
  bool parsingFailed();
  bool roomNavigationSucceeded();
  bool roomVerified();
  bool objectLocated();
  bool approachSucceeded();
  bool graspSucceeded();
  bool graspVerified();
  bool destinationReached();
  bool placementSucceeded();
  bool placementVerified();
  bool objectNotFound();
  bool giveUp();
  bool moderatorSucceeded();
  bool moderatorFailed();
  bool missionCompleted();

private:
  TaskState state_{TaskState::kBooting};
  HandymanTask task_{};
};

}  // namespace handyman_rebuild_ros2
