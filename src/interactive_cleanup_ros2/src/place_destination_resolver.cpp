#include "interactive_cleanup/place_destination_resolver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace interactive_cleanup
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kInvalidDistance = 9999.0;
constexpr double kMinPointingConfidence = 0.25;
constexpr double kDirectionClusterHalfAngle = 35.0 * kPi / 180.0;
constexpr double kMinAlongDistance = 0.15;
constexpr double kAmbiguityMargin = 0.12;
constexpr double kCameraFocalLengthPx = 554.0;
constexpr double kMaxAnchoredImageAngle = 8.0 * kPi / 180.0;

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

double pointRayDist3D(
  double px,
  double py,
  double pz,
  double ox,
  double oy,
  double oz,
  double dx,
  double dy,
  double dz,
  double *along = nullptr)
{
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 1e-6) {
    return kInvalidDistance;
  }

  dx /= len;
  dy /= len;
  dz /= len;
  const double vx = px - ox;
  const double vy = py - oy;
  const double vz = pz - oz;
  const double proj = vx * dx + vy * dy + vz * dz;
  if (along != nullptr) {
    *along = proj;
  }
  if (proj < 0.0) {
    return kInvalidDistance;
  }

  const double perp_x = vx - proj * dx;
  const double perp_y = vy - proj * dy;
  const double perp_z = vz - proj * dz;
  return std::sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z);
}

double pointRayDist2D(
  double px, double py, double ox, double oy,
  double dx, double dy, double *along = nullptr)
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
  return std::hypot(vx - proj * dx, vy - proj * dy);
}

struct NormalizedPointing
{
  geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Vector3 direction;
  double confidence{0.0};
};

double planarDirectionDot(const NormalizedPointing &lhs, const NormalizedPointing &rhs)
{
  const double lhs_norm = std::hypot(lhs.direction.x, lhs.direction.y);
  const double rhs_norm = std::hypot(rhs.direction.x, rhs.direction.y);
  if (lhs_norm < 1e-6 || rhs_norm < 1e-6) {
    return 0.0;
  }
  return (lhs.direction.x * rhs.direction.x +
          lhs.direction.y * rhs.direction.y) / (lhs_norm * rhs_norm);
}

std::vector<NormalizedPointing> coherentPointingCluster(
  const std::vector<PointingObservation> &pointings)
{
  std::vector<NormalizedPointing> normalized;

  for (const auto &pointing : pointings) {
    if (!pointing.is_valid || pointing.confidence < kMinPointingConfidence) {
      continue;
    }

    const double norm = std::sqrt(
      pointing.direction.x * pointing.direction.x +
      pointing.direction.y * pointing.direction.y +
      pointing.direction.z * pointing.direction.z);
    if (norm < 1e-6) {
      continue;
    }
    NormalizedPointing sample;
    sample.origin = pointing.origin;
    sample.direction.x = pointing.direction.x / norm;
    sample.direction.y = pointing.direction.y / norm;
    sample.direction.z = pointing.direction.z / norm;
    sample.confidence = std::clamp(pointing.confidence, 0.0, 1.0);
    normalized.push_back(sample);
  }

  if (normalized.empty()) {
    return {};
  }

  // Cluster by the pointing axis.  MediaPipe can reverse depth polarity for
  // frontal poses, so polarity is resolved once for the complete gesture.
  const double min_dot = std::cos(kDirectionClusterHalfAngle);
  std::size_t best_seed = 0;
  int best_support = 0;
  for (std::size_t seed = 0; seed < normalized.size(); ++seed) {
    int support = 0;
    for (const auto &sample : normalized) {
      if (std::abs(planarDirectionDot(normalized[seed], sample)) >= min_dot) {
        ++support;
      }
    }
    if (support > best_support) {
      best_support = support;
      best_seed = seed;
    }
  }

  std::vector<NormalizedPointing> cluster;
  for (auto sample : normalized) {
    const double dot = planarDirectionDot(normalized[best_seed], sample);
    if (std::abs(dot) < min_dot) {
      continue;
    }
    if (dot < 0.0) {
      sample.direction.x *= -1.0;
      sample.direction.y *= -1.0;
      sample.direction.z *= -1.0;
    }
    cluster.push_back(sample);
  }
  return cluster;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return kInvalidDistance;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    result = 0.5 * (
      result + *std::max_element(values.begin(), values.begin() + middle));
  }
  return result;
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
  (void)target_class;
  DestinationResolution result;
  const auto cluster = coherentPointingCluster(pointings);
  result.usable_samples = static_cast<int>(cluster.size());
  if (cluster.empty()) {
    result.rejection_reason = "no valid pointing ray";
    return result;
  }

  struct Candidate
  {
    const DestinationRegion *region{nullptr};
    double cost{kInvalidDistance};
    double median_planar_ray{kInvalidDistance};
    double median_three_d_ray{kInvalidDistance};
    double median_surface_intersection{kInvalidDistance};
    double image_ray_distance_px{kInvalidDistance};
    double median_along{kInvalidDistance};
    int support{0};
    bool reversed{false};
  };
  std::vector<Candidate> candidates;
  const int required_support = std::max(
    1, static_cast<int>(std::ceil(cluster.size() * 0.50)));
  double best_image_distance = kInvalidDistance;
  for (const auto &region : regions) {
    if (region.image_projection_valid) {
      best_image_distance = std::min(
        best_image_distance, region.image_ray_distance_px);
    }
  }
  // An image line is useful only when it actually passes close to at least
  // one mapped destination. If every candidate is far from that line (as in
  // task 1 with all relevant places outside the image), extrapolating it for
  // thousands of pixels amplifies a small landmark error and must not
  // override the depth-independent pose-world arm direction.
  const bool use_image_evidence =
    std::atan2(best_image_distance, kCameraFocalLengthPx) <=
    kMaxAnchoredImageAngle;

  // Evaluate both polarities globally.  The previous controller flipped each
  // frame independently toward whichever known region happened to be close,
  // which allowed a mixed gesture to manufacture a false consensus.
  for (int sign : {1, -1}) {
    for (const auto &region : regions) {
      std::vector<double> planar_ray_distances;
      std::vector<double> three_d_ray_distances;
      std::vector<double> surface_intersection_distances;
      std::vector<double> along_distances;
      for (const auto &sample : cluster) {
        double planar_along = 0.0;
        const double planar_ray_distance = pointRayDist2D(
          region.center.x, region.center.y,
          sample.origin.x, sample.origin.y,
          sign * sample.direction.x,
          sign * sample.direction.y,
          &planar_along);
        if (planar_along < kMinAlongDistance) {
          continue;
        }
        const double three_d_ray_distance = pointRayDist3D(
          region.center.x, region.center.y, region.center.z,
          sample.origin.x, sample.origin.y, sample.origin.z,
          sign * sample.direction.x,
          sign * sample.direction.y,
          sign * sample.direction.z);
        planar_ray_distances.push_back(planar_ray_distance);
        three_d_ray_distances.push_back(three_d_ray_distance);
        along_distances.push_back(planar_along);

        // A perpendicular distance to an infinite horizontal ray cannot
        // distinguish destinations that are almost collinear (notably the
        // white side table and cardboard box).  Intersect the 3-D gesture
        // with each destination's actual placement height and compare the
        // landing point in XY.  This uses the full arm elevation without
        // imposing another pass/fail threshold; noisy samples are handled by
        // the median below.
        const double signed_dz = sign * sample.direction.z;
        if (std::abs(signed_dz) > 1e-3) {
          const double t = (region.center.z - sample.origin.z) / signed_dz;
          if (t > 0.0) {
            const double hit_x = sample.origin.x +
              t * sign * sample.direction.x;
            const double hit_y = sample.origin.y +
              t * sign * sample.direction.y;
            surface_intersection_distances.push_back(
              std::hypot(region.center.x - hit_x, region.center.y - hit_y));
          }
        }
      }
      if (static_cast<int>(planar_ray_distances.size()) < required_support) {
        continue;
      }

      const double median_planar_ray = median(planar_ray_distances);
      const double median_three_d_ray = median(three_d_ray_distances);
      const double median_surface_intersection =
        median(surface_intersection_distances);
      const double median_along = median(along_distances);
      const double mean_planar_ray = std::accumulate(
        planar_ray_distances.begin(), planar_ray_distances.end(), 0.0) /
        static_cast<double>(planar_ray_distances.size());
      // Prefer the physical landing point when elevation is available.  For
      // a nearly horizontal gesture, fall back to ordinary 3-D/planar ray
      // distance.  Distances remain soft costs rather than brittle gates.
      const bool has_surface_intersection =
        median_surface_intersection < kInvalidDistance;
      const double world_cost = has_surface_intersection
        ? median_planar_ray + 0.30 * median_surface_intersection +
          0.10 * median_three_d_ray + 0.05 * mean_planar_ray
        : median_three_d_ray + 0.25 * median_planar_ray +
          0.05 * mean_planar_ray;
      double cost = world_cost;
      if (use_image_evidence && region.image_projection_valid) {
        // Depth sampled on a thin synthetic arm is easily contaminated by
        // the wall or furniture behind it.  The 2-D arm line itself remains
        // stable, so compare it with the known region projected into the same
        // held camera view. Convert pixels to an angular error and make that
        // the primary cue while retaining world geometry as a soft tie-break.
        const double image_angle = std::atan2(
          std::max(0.0, region.image_ray_distance_px),
          kCameraFocalLengthPx);
        cost = 4.0 * image_angle + 0.20 * world_cost;
      }
      candidates.push_back(Candidate{
        &region, cost, median_planar_ray, median_three_d_ray,
        median_surface_intersection,
        region.image_projection_valid
          ? region.image_ray_distance_px : kInvalidDistance,
        median_along,
        static_cast<int>(planar_ray_distances.size()), sign < 0});
    }
  }

  if (candidates.empty()) {
    result.rejection_reason = "no region has enough horizontal-ray support";
    return result;
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate &lhs, const Candidate &rhs) { return lhs.cost < rhs.cost; });
  const Candidate &best = candidates.front();
  const auto second = std::find_if(
    candidates.begin() + 1, candidates.end(),
    [&best](const Candidate &candidate) {
      return candidate.region->name != best.region->name;
    });
  if (second != candidates.end()) {
    result.runner_up_name = second->region->name;
    result.runner_up_cost = second->cost;
  }
  if (second != candidates.end() && second->cost - best.cost < kAmbiguityMargin) {
    result.rejection_reason = "best and second-best destination rays are ambiguous: " +
      best.region->name + " vs " + second->region->name;
    return result;
  }

  result.valid = true;
  result.name = best.region->name;
  result.position = best.region->center;
  result.placement_mode = best.region->placement_mode;
  result.score = 1.0 / (1.0 + best.cost);
  result.ray_distance = best.median_planar_ray;
  result.three_d_ray_distance = best.median_three_d_ray;
  result.surface_intersection_distance = best.median_surface_intersection;
  result.image_ray_distance_px = best.image_ray_distance_px;
  result.along_distance = best.median_along;
  result.supporting_samples = best.support;
  result.image_evidence_used = use_image_evidence;
  result.reversed = best.reversed;
  return result;
}

}  // namespace interactive_cleanup
