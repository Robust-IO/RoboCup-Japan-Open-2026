#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"

namespace handyman_rebuild_ros2
{

class SimulatedModeratorNode final : public rclcpp::Node
{
public:
  explicit SimulatedModeratorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  bool succeeded() const noexcept;
  bool finished() const noexcept;

private:
  enum class Step
  {
    kWaitingConnections,
    kWaitingIAmReady,
    kWaitingRoomReached,
    kWaitingObjectGrasped,
    kWaitingTaskFinished,
    kSendMissionComplete,
    kFinish,
  };

  void publishIncoming(protocol::CompetitionEvent event, const std::string & detail = "");
  void fail(const std::string & reason);
  void onTimer();
  void expectAndAdvance(
    const handyman_msgs::msg::HandymanMsg & message,
    protocol::CompetitionEvent expected,
    Step next_step);
  void onRobotMessage(const handyman_msgs::msg::HandymanMsg & message);

  Step step_{Step::kWaitingConnections};
  std::chrono::steady_clock::time_point started_at_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool succeeded_{false};
  bool finished_{false};
  int handshake_ticks_{0};
};

}  // namespace handyman_rebuild_ros2
