#include "interactive_cleanup/place_destination_resolver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <stdexcept>

namespace interactive_cleanup
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kInvalidDistance = 9999.0;
constexpr double kMinPointingConfidence = 0.35;

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

bool containsToken(const std::string &name, const char *token)
{
  return toLower(name).find(token) != std::string::npos;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double angleDistance(double lhs, double rhs)
{
  return std::abs(normalizeAngle(lhs - rhs));
}

double pointRayDist2D(
  double px,
  double py,
  double ox,
  double oy,
  double dx,
  double dy,
  double *along = nullptr)
{
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) {
    return kInvalidDistance;
  }

  dx /= len;
  dy /= len;
  const double vx = px - ox;
  const double vy = py - oy;
  const double proj = vx * dx + vy * dy;
  if (along != nullptr) {
    *along = proj;
  }
  if (proj < 0.0) {
    return kInvalidDistance;
  }

  const double perp_x = vx - proj * dx;
  const double perp_y = vy - proj * dy;
  return std::hypot(perp_x, perp_y);
}

struct AggregatedPointing
{
  bool valid{false};
  geometry_msgs::msg::Point origin;
  double yaw{0.0};
  double angular_stability{0.0};
};

AggregatedPointing aggregatePointings(const std::vector<PointingObservation> &pointings)
{
  AggregatedPointing aggregate;
  double origin_sum_x = 0.0;
  double origin_sum_y = 0.0;
  double dir_sum_x = 0.0;
  double dir_sum_y = 0.0;
  std::vector<double> sample_yaws;
  std::vector<double> sample_weights;
  double weight_sum = 0.0;

  for (const auto &pointing : pointings) {
    if (!pointing.is_valid || pointing.confidence < kMinPointingConfidence) {
      continue;
    }

    const double planar_norm = std::hypot(pointing.direction.x, pointing.direction.y);
    if (planar_norm < 1e-6) {
      continue;
    }

    const double weight = std::clamp(pointing.confidence, 0.0, 1.0);
    weight_sum += weight;
    origin_sum_x += pointing.origin.x * weight;
    origin_sum_y += pointing.origin.y * weight;
    dir_sum_x += (pointing.direction.x / planar_norm) * weight;
    dir_sum_y += (pointing.direction.y / planar_norm) * weight;
    sample_yaws.push_back(std::atan2(pointing.direction.y, pointing.direction.x));
    sample_weights.push_back(weight);
  }

  if (sample_yaws.empty() || weight_sum <= 1e-6) {
    return aggregate;
  }

  const double avg_dir_norm = std::hypot(dir_sum_x, dir_sum_y);
  if (avg_dir_norm < 1e-6) {
    return aggregate;
  }

  aggregate.valid = true;
  aggregate.origin.x = origin_sum_x / weight_sum;
  aggregate.origin.y = origin_sum_y / weight_sum;
  aggregate.yaw = std::atan2(dir_sum_y, dir_sum_x);

  double total_error = 0.0;
  for (std::size_t index = 0; index < sample_yaws.size(); ++index) {
    total_error += angleDistance(sample_yaws[index], aggregate.yaw) * sample_weights[index];
  }
  aggregate.angular_stability = total_error / weight_sum;
  return aggregate;
}

std::string inferDestinationType(const std::string &name)
{
  const std::string lowered = toLower(name);
  if (lowered.find("trash_box") != std::string::npos) {
    return "trash_box";
  }
  if (lowered.find("table") != std::string::npos) {
    return "table";
  }
  if (lowered.find("wagon") != std::string::npos) {
    return "wagon";
  }
  if (lowered.find("shelf") != std::string::npos) {
    return "shelf";
  }
  if (lowered.find("box") != std::string::npos) {
    return "box";
  }
  return "generic";
}

double defaultRadiusForType(const std::string &type)
{
  if (type == "trash_box") {
    return 0.35;
  }
  if (type == "table") {
    return 0.50;
  }
  if (type == "wagon") {
    return 0.55;
  }
  if (type == "shelf") {
    return 0.45;
  }
  if (type == "box") {
    return 0.40;
  }
  return 0.45;
}

std::string defaultPlacementModeForType(const std::string &type)
{
  if (type == "trash_box" || type == "box") {
    return "drop";
  }
  if (type == "shelf") {
    return "shelf_place";
  }
  return "surface_place";
}

double defaultPreferredFacingYaw(double center_x, double center_y)
{
  return std::atan2(-center_y, -center_x);
}

double taskPriorForRegionType(const std::string &type, const std::string &target_class)
{
  const std::string lowered = toLower(target_class);
  if (lowered.empty()) {
    return 0.0;
  }

  if (type == "trash_box" &&
      (containsToken(lowered, "can") || containsToken(lowered, "bottle"))) {
    return 0.25;
  }

  if ((type == "table" || type == "wagon") && containsToken(lowered, "soysauce")) {
    return 0.10;
  }

  return 0.0;
}

double sectorCompatibility(const DestinationRegion &region, double bearing_yaw)
{
  if (region.sector_half_width >= kPi - 1e-3) {
    return 1.0;
  }

  const double delta = angleDistance(bearing_yaw, region.sector_center_yaw);
  if (delta <= region.sector_half_width) {
    return 1.0;
  }

  return std::max(0.0, 1.0 - (delta - region.sector_half_width) / (kPi * 0.5));
}

DestinationRegion regionFromYaml(const YAML::Node &node)
{
  if (!node["name"] || !node["center"]) {
    throw std::runtime_error("destination region entry requires 'name' and 'center'");
  }

  const auto center = node["center"];
  if (!center["x"] || !center["y"]) {
    throw std::runtime_error("destination region center requires 'x' and 'y'");
  }

  DestinationRegion region = inferDestinationRegion(
    node["name"].as<std::string>(),
    center["x"].as<double>(),
    center["y"].as<double>());

  if (node["type"]) {
    region.type = node["type"].as<std::string>();
  }
  if (node["allowed_radius"]) {
    region.allowed_radius = node["allowed_radius"].as<double>();
  }
  if (node["sector_center_yaw"]) {
    region.sector_center_yaw = node["sector_center_yaw"].as<double>();
  }
  if (node["sector_half_width"]) {
    region.sector_half_width = node["sector_half_width"].as<double>();
  }
  if (node["preferred_facing_yaw"]) {
    region.preferred_facing_yaw = node["preferred_facing_yaw"].as<double>();
  }
  if (node["placement_mode"]) {
    region.placement_mode = node["placement_mode"].as<std::string>();
  }

  return region;
}

}  // namespace

DestinationRegion inferDestinationRegion(
  const std::string &name,
  double center_x,
  double center_y)
{
  DestinationRegion region;
  region.name = name;
  region.type = inferDestinationType(name);
  region.center.x = center_x;
  region.center.y = center_y;
  region.center.z = 0.0;
  region.allowed_radius = defaultRadiusForType(region.type);
  region.sector_center_yaw = defaultPreferredFacingYaw(center_x, center_y);
  region.sector_half_width = kPi;
  region.preferred_facing_yaw = defaultPreferredFacingYaw(center_x, center_y);
  region.placement_mode = defaultPlacementModeForType(region.type);
  return region;
}

std::vector<DestinationRegion> loadDestinationRegions(
  const std::string &yaml_path,
  std::string *error)
{
  if (error != nullptr) {
    error->clear();
  }

  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node regions_node = root["regions"];
    if (!regions_node || !regions_node.IsSequence()) {
      throw std::runtime_error("destination_regions.yaml requires a 'regions' sequence");
    }

    std::vector<DestinationRegion> regions;
    regions.reserve(regions_node.size());
    for (const auto &node : regions_node) {
      regions.push_back(regionFromYaml(node));
    }
    return regions;
  } catch (const std::exception &ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return {};
  }
}

DestinationResolution resolvePlaceDestination(
  const std::vector<DestinationRegion> &regions,
  const std::vector<PointingObservation> &pointings,
  const std::string &target_class)
{
  DestinationResolution result;
  const auto aggregate = aggregatePointings(pointings);
  if (!aggregate.valid) {
    return result;
  }

  const double ray_dx = std::cos(aggregate.yaw);
  const double ray_dy = std::sin(aggregate.yaw);
  const double stability_score = 1.0 / (1.0 + aggregate.angular_stability / 0.35);
  double best_score = -std::numeric_limits<double>::infinity();

  for (const auto &region : regions) {
    double along = 0.0;
    const double ray_distance = pointRayDist2D(
      region.center.x,
      region.center.y,
      aggregate.origin.x,
      aggregate.origin.y,
      ray_dx,
      ray_dy,
      &along);
    if (ray_distance >= kInvalidDistance - 1.0 || along <= 0.0) {
      continue;
    }

    const double avatar_distance = std::hypot(
      region.center.x - aggregate.origin.x,
      region.center.y - aggregate.origin.y);
    const double bearing = std::atan2(
      region.center.y - aggregate.origin.y,
      region.center.x - aggregate.origin.x);
    const double region_scale = std::max(0.10, region.allowed_radius);
    const double alignment_score = 4.0 / (1.0 + ray_distance / region_scale);
    const double membership_score =
      ray_distance <= region.allowed_radius ?
      2.0 :
      2.0 / (1.0 + ray_distance / region_scale);
    const double distance_score =
      avatar_distance < 0.35 ? 0.0 : 1.0 / (1.0 + std::abs(avatar_distance - 2.5) / 2.0);
    const double sector_score = sectorCompatibility(region, bearing);
    const double task_prior = taskPriorForRegionType(region.type, target_class);

    const double score =
      alignment_score +
      membership_score +
      0.75 * sector_score +
      0.5 * distance_score +
      0.5 * stability_score +
      task_prior;

    if (score <= best_score) {
      continue;
    }

    best_score = score;
    result.valid = true;
    result.name = region.name;
    result.position = region.center;
    result.placement_mode = region.placement_mode;
    result.score = score;
  }

  return result;
}

}  // namespace interactive_cleanup
