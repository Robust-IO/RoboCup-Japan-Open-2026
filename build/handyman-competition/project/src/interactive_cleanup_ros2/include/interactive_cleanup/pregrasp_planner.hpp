#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <string>

#include "interactive_cleanup/hsr_geometry.hpp"

namespace interactive_cleanup
{

struct PregraspPlan
{
  bool valid{false};
  std::string grasp_mode;
  HsrArmPose arm_pose;
  double nav_standoff{0.0};
  double approach_stop_distance{0.0};
  double desired_hand_camera_height{0.0};
  std::string approach_profile;
};

PregraspPlan planPregrasp(
  const geometry_msgs::msg::Point &target_in_base,
  const std::string &grasp_mode);

}  // namespace interactive_cleanup
