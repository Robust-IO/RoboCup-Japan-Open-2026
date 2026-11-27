#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"
#include "handyman_rebuild_ros2/environment_config.hpp"
#include "handyman_rebuild_ros2/instruction_parser.hpp"
#include "handyman_rebuild_ros2/navigation_executor.hpp"
#include "handyman_rebuild_ros2/task_state_machine.hpp"
#include "handyman_rebuild_ros2/cancellation_barrier.hpp"
#include "handyman_rebuild_ros2/search_cancellation_barrier.hpp"

namespace handyman_rebuild_ros2
{

class CoordinatorNode final : public rclcpp::Node
{
public:
  explicit CoordinatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void stopNavigation();
  CancellationBarrier cancellation_barrier_;
  SearchCancellationBarrier search_cancellation_barrier_;
  bool dispatchBlocked();
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr search_status_subscription_;
  void publishSearchRequest();
  void invalidateSearchRequest(const std::string & reason);
  bool search_requests_enabled_{false};
  std::string search_request_id_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr search_request_publisher_;
  void stopSimulation();
  void startSimulation();
  void runSimulationStep();
  bool parseCurrentInstruction(const std::string & instruction);
  void startRoomNavigation();
  void handleNavigationOutcome(const NavigationOutcome & outcome);
  void publishEvent(protocol::CompetitionEvent event);
  void handleEvent(const protocol::CompetitionMessage & message);
  void onMessage(const handyman_msgs::msg::HandymanMsg & ros_message);

  TaskStateMachine state_machine_;
  EnvironmentCatalog environment_catalog_;
  std::unique_ptr<RuleBasedInstructionParser> instruction_parser_;
  std::unique_ptr<NavigationExecutor> navigation_executor_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  bool simulate_modules_{false};
  int simulation_step_ms_{100};
  int simulation_step_{0};
};

}  // namespace handyman_rebuild_ros2
