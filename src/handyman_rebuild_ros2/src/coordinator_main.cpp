#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "handyman_rebuild_ros2/coordinator_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handyman_rebuild_ros2::CoordinatorNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
