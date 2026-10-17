#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <string>
#include <vector>

#include "interactive_cleanup/pick_target_resolver.hpp"

namespace interactive_cleanup
{

struct DestinationRegion
{
  std::string name;
  std::string type;
  geometry_msgs::msg::Point center;
  double allowed_radius{0.4};
  double sector_center_yaw{0.0};
  double sector_half_width{3.14159265358979323846};
  double preferred_facing_yaw{0.0};
  std::string placement_mode{"surface_place"};
  // Optional image-space evidence, populated by the controller by projecting
  // this region into the held destination-gesture camera view. The distance
  // is measured from the projected trigger centre to the visible arm ray.
  bool image_projection_valid{false};
  double image_ray_distance_px{0.0};
  int image_supporting_samples{0};
};

struct DestinationResolution
{
  bool valid{false};
  std::string name;
  geometry_msgs::msg::Point position;
  std::string placement_mode;
  double score{0.0};
  double ray_distance{0.0};
  double three_d_ray_distance{0.0};
  double surface_intersection_distance{0.0};
  double image_ray_distance_px{0.0};
  double along_distance{0.0};
  int supporting_samples{0};
  int usable_samples{0};
  bool image_evidence_used{false};
  bool reversed{false};
  std::string runner_up_name;
  double runner_up_cost{0.0};
  std::string rejection_reason;
};

DestinationRegion inferDestinationRegion(
  const std::string &name,
  double center_x,
  double center_y);

std::vector<DestinationRegion> loadDestinationRegions(
  const std::string &yaml_path,
  std::string *error = nullptr);

DestinationResolution resolvePlaceDestination(
  const std::vector<DestinationRegion> &regions,
  const std::vector<PointingObservation> &pointings,
  const std::string &target_class);

}  // namespace interactive_cleanup
