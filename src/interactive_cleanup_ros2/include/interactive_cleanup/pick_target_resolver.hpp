#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>
#include <vector>

namespace interactive_cleanup
{

struct PickCandidate
{
  std::string class_name;
  double confidence{0.0};
  double bbox_cx{0.0};
  double bbox_cy{0.0};
  double bbox_w{0.0};
  double bbox_h{0.0};
  geometry_msgs::msg::Point position;
  bool has_3d_position{false};
};

struct PointingObservation
{
  bool is_valid{false};
  double confidence{0.0};
  geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Vector3 direction;
  double wrist_pixel_x{0.0};
  double wrist_pixel_y{0.0};
  double point_pixel_x{0.0};
  double point_pixel_y{0.0};
};

struct PickResolution
{
  bool valid{false};
  std::string class_name;
  geometry_msgs::msg::Point position;
  double score{0.0};
  std::string grasp_mode;
};

PickResolution resolvePickTarget(
  const std::vector<PickCandidate> &candidates,
  const std::vector<PointingObservation> &pointings);

std::string classifyGraspMode(double target_height);

}  // namespace interactive_cleanup
