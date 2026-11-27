#include "handyman_rebuild_ros2/simulated_moderator_node.hpp"

#include <chrono>
#include <string>

namespace handyman_rebuild_ros2
{

SimulatedModeratorNode::SimulatedModeratorNode(const rclcpp::NodeOptions & options)
: Node("handyman_simulated_moderator", options),
  started_at_(std::chrono::steady_clock::now())
{
    using handyman_msgs::msg::HandymanMsg;
    publisher_ = create_publisher<HandymanMsg>(std::string(protocol::kToRobotTopic), 10);
    subscription_ = create_subscription<HandymanMsg>(
      std::string(protocol::kToModeratorTopic), 10,
      [this](const HandymanMsg::ConstSharedPtr message) { onRobotMessage(*message); });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "Simulated Moderator is waiting for the Coordinator");
  }

bool SimulatedModeratorNode::succeeded() const noexcept { return succeeded_; }
bool SimulatedModeratorNode::finished() const noexcept { return finished_; }

void SimulatedModeratorNode::publishIncoming(
  protocol::CompetitionEvent event, const std::string & detail)
{
    handyman_msgs::msg::HandymanMsg message;
    message.message = std::string(protocol::eventName(event));
    message.detail = detail;
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Moderator sent: %s", message.message.c_str());
  }

void SimulatedModeratorNode::fail(const std::string & reason)
{
    if (finished_) {
      return;
    }
    finished_ = true;
    timer_->cancel();
    RCLCPP_ERROR(get_logger(), "PHASE 1 SIMULATION FAILED: %s", reason.c_str());
  }

void SimulatedModeratorNode::onTimer()
{
    if (finished_) {
      return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    if (elapsed > std::chrono::seconds(15)) {
      fail("protocol round timed out");
      return;
    }

    switch (step_) {
      case Step::kWaitingConnections:
        publishIncoming(protocol::CompetitionEvent::kEnvironment, "LayoutA");
        publishIncoming(protocol::CompetitionEvent::kAreYouReady);
        step_ = Step::kWaitingIAmReady;
        break;
      case Step::kWaitingIAmReady:
        ++handshake_ticks_;
        if (handshake_ticks_ % 5 == 0) {
          // DDS discovery can be delayed in WSL. Repeating the idempotent handshake
          // makes the simulator robust without relying on graph discovery counters.
          publishIncoming(protocol::CompetitionEvent::kEnvironment, "LayoutA");
          publishIncoming(protocol::CompetitionEvent::kAreYouReady);
        }
        break;
      case Step::kSendMissionComplete:
        publishIncoming(protocol::CompetitionEvent::kMissionComplete);
        step_ = Step::kFinish;
        succeeded_ = true;
        finished_ = true;
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "PHASE 1 SIMULATION PASSED");
        break;
      case Step::kFinish:
        break;
      default:
        break;
    }
  }

void SimulatedModeratorNode::expectAndAdvance(
    const handyman_msgs::msg::HandymanMsg & message,
    protocol::CompetitionEvent expected,
    Step next_step)
{
    const std::string expected_name(protocol::eventName(expected));
    if (message.message != expected_name) {
      fail("expected " + expected_name + ", received " + message.message);
      return;
    }
    if (!message.detail.empty()) {
      fail(message.message + " must have blank detail");
      return;
    }
    RCLCPP_INFO(get_logger(), "Moderator received expected event: %s", message.message.c_str());
    step_ = next_step;
  }

void SimulatedModeratorNode::onRobotMessage(const handyman_msgs::msg::HandymanMsg & message)
{
    switch (step_) {
      case Step::kWaitingIAmReady:
        expectAndAdvance(
          message, protocol::CompetitionEvent::kIAmReady, Step::kWaitingRoomReached);
        if (step_ == Step::kWaitingRoomReached) {
          publishIncoming(
            protocol::CompetitionEvent::kInstruction,
            "Go to the kitchen, grasp the apple and bring it to the dining table.");
        }
        break;
      case Step::kWaitingRoomReached:
        expectAndAdvance(
          message, protocol::CompetitionEvent::kRoomReached,
          Step::kWaitingObjectGrasped);
        break;
      case Step::kWaitingObjectGrasped:
        expectAndAdvance(
          message, protocol::CompetitionEvent::kObjectGrasped,
          Step::kWaitingTaskFinished);
        break;
      case Step::kWaitingTaskFinished:
        expectAndAdvance(
          message, protocol::CompetitionEvent::kTaskFinished,
          Step::kSendMissionComplete);
        if (step_ == Step::kSendMissionComplete) {
          publishIncoming(protocol::CompetitionEvent::kTaskSucceeded);
        }
        break;
      default:
        fail("received an event at an unexpected point: " + message.message);
        break;
    }
  }

}  // namespace handyman_rebuild_ros2
