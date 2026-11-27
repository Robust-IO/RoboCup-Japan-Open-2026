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
  // True only for a synchronized RGB-D arm/finger fit. Pose/image-derived
  // world rays remain useful fallbacks but must not veto image evidence as if
  // they were measured 3-D geometry.
  bool measured_depth{false};
  bool plane_valid{false};
  geometry_msgs::msg::Point plane_origin;
  geometry_msgs::msg::Vector3 plane_normal;
  geometry_msgs::msg::Vector3 plane_forward;
  geometry_msgs::msg::Vector3 camera_forward;
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
