#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "handyman_rebuild_ros2/simulated_moderator_node.hpp"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<handyman_rebuild_ros2::SimulatedModeratorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->finished()) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  const auto flush_until = std::chrono::steady_clock::now() + 300ms;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < flush_until) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return node->succeeded() ? 0 : 1;
}
