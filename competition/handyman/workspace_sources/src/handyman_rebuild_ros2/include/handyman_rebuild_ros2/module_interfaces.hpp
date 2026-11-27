#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "handyman_rebuild_ros2/task.hpp"

namespace handyman_rebuild_ros2
{

struct OperationResult
{
  bool success{false};
  bool retryable{false};
  std::string reason;
};

class InstructionParser
{
public:
  virtual ~InstructionParser() = default;
  virtual OperationResult parse(const std::string & instruction, HandymanTask & task) = 0;
};

class NavigationManager
{
public:
  virtual ~NavigationManager() = default;
  virtual OperationResult loadEnvironment(const std::string & environment) = 0;
  virtual OperationResult navigateToRoom(const std::string & room) = 0;
  virtual OperationResult navigateToDestination(const HandymanTask & task) = 0;
};

class ObjectPerception
{
public:
  virtual ~ObjectPerception() = default;
  virtual OperationResult findTarget(
    const HandymanTask & task, geometry_msgs::msg::PoseStamped & target_pose) = 0;
};

class ManipulationManager
{
public:
  virtual ~ManipulationManager() = default;
  virtual OperationResult grasp(
    const HandymanTask & task, const geometry_msgs::msg::PoseStamped & target_pose) = 0;
  virtual OperationResult verifyGrasp(const HandymanTask & task) = 0;
  virtual OperationResult place(const HandymanTask & task) = 0;
  virtual OperationResult verifyPlacement(const HandymanTask & task) = 0;
};

}  // namespace handyman_rebuild_ros2
