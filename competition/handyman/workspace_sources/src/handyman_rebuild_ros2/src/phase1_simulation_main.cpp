#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "handyman_rebuild_ros2/coordinator_node.hpp"
#include "handyman_rebuild_ros2/simulated_moderator_node.hpp"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions coordinator_options;
  coordinator_options.parameter_overrides({
    rclcpp::Parameter("simulate_modules", true),
    rclcpp::Parameter("simulation_step_ms", 20),
  });
  auto coordinator =
    std::make_shared<handyman_rebuild_ros2::CoordinatorNode>(coordinator_options);
  auto moderator = std::make_shared<handyman_rebuild_ros2::SimulatedModeratorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(coordinator);
  executor.add_node(moderator);
  while (rclcpp::ok() && !moderator->finished()) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  const auto flush_until = std::chrono::steady_clock::now() + 300ms;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < flush_until) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return moderator->succeeded() ? 0 : 1;
}
