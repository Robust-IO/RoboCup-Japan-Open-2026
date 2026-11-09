#include "interactive_cleanup/place_destination_resolver.hpp"
#include "interactive_cleanup/pointing_geometry.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iterator>
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
// A stable ten-frame image ray in task 1 passes about 10.7 degrees from the
// intended side-table trigger.  Eight degrees discarded that useful evidence
// and let a physically invalid pose-world ray select the cardboard box.
constexpr double kMaxAnchoredImageAngle = 12.0 * kPi / 180.0;
constexpr int kMinAnchoredImageSamples = 1;
constexpr int kMinImageDepthOverrideSamples = 2;
constexpr double kImageAmbiguityMargin = 3.0 * kPi / 180.0;
constexpr double kWorldRayRadiusSlack = 0.55;
constexpr double kSurfaceHitRadiusSlack = 0.65;

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
  bool measured_depth{false};
};

double directionAxisDot(const NormalizedPointing &lhs, const NormalizedPointing &rhs)
{
  return (lhs.direction.x * rhs.direction.x +
          lhs.direction.y * rhs.direction.y +
          lhs.direction.z * rhs.direction.z);
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
    sample.measured_depth = pointing.measured_depth;
    normalized.push_back(sample);
  }

  if (normalized.empty()) {
    return {};
  }

  // Cluster directed rays. Opposite directions are different observations.
  const double min_dot = std::cos(kDirectionClusterHalfAngle);
  std::size_t best_seed = 0;
  int best_support = 0;
  for (std::size_t seed = 0; seed < normalized.size(); ++seed) {
    int support = 0;
    for (const auto &sample : normalized) {
      if (directionAxisDot(normalized[seed], sample) >= min_dot) {
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
    const double dot = directionAxisDot(normalized[best_seed], sample);
    if (dot < min_dot) {
      continue;
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
  // Production observations carry the image plane captured with the same TF
  // as their measured arm ray. Use that geometry directly, across head poses.
  struct PlaneCandidate { const DestinationRegion *region; double cost; int support; };
  std::vector<PlaneCandidate> plane_candidates;
  const int plane_count = std::count_if(
    pointings.begin(), pointings.end(), [](const PointingObservation &p) {
      return p.is_valid && p.confidence >= kMinPointingConfidence && p.plane_valid;
    });
  if (plane_count > 0) {
    const int measured_count = std::count_if(
      pointings.begin(), pointings.end(), [](const PointingObservation &p) {
        return p.is_valid && p.confidence >= kMinPointingConfidence &&
               p.plane_valid && p.measured_depth;
      });
    // Depth fits and pose-only transition frames are different evidence.
    // Once depth is available, form its consensus without allowing missing
    // depth / moving-hand frames to inflate the required vote denominator.
    const int consensus_count = measured_count > 0 ? measured_count : plane_count;
    for (const auto &region : regions) {
      std::vector<double> costs;
      int measured_support = 0;
      for (const auto &p : pointings) {
        if (!p.is_valid || p.confidence < kMinPointingConfidence || !p.plane_valid)
          continue;
        if (measured_count > 0 && !p.measured_depth) continue;
        const double angle = pointingPlaneAngle(region.center, p.plane_origin, p.plane_normal);
        if (angle > 0.12) continue;
        if (!pointingHalfspaceContains(region.center, p.plane_origin, p.plane_forward, 0.03) ||
            !pointingHalfspaceContains(region.center, p.plane_origin, p.camera_forward)) continue;
        double depth_cost = 0.0;
        if (p.measured_depth) {
          const double distance = pointRayDist3D(
            region.center.x, region.center.y, region.center.z,
            p.origin.x, p.origin.y, p.origin.z,
            p.direction.x, p.direction.y, p.direction.z);
          if (distance > region.allowed_radius + kWorldRayRadiusSlack) continue;
          depth_cost = distance / 3.0;
          ++measured_support;
        }
        costs.push_back(angle + depth_cost);
      }
      if (static_cast<int>(costs.size()) >= std::max(2, (consensus_count + 1) / 2) &&
          measured_support >= (measured_count + 1) / 2) {
        plane_candidates.push_back({&region, median(costs), static_cast<int>(costs.size())});
      }
    }
    result.usable_samples = consensus_count;
    if (plane_candidates.empty()) {
      // A nearly level arm indicates a furniture bearing, not necessarily
      // its low release point. Only measured, near-horizontal rays may use
      // this secondary interpretation. Never reverse rays or use pose-only
      // estimates here; retain the existing 3-D distance and forward gates.
      struct BearingCandidate { const DestinationRegion *region; double cost; int support; };
      std::vector<BearingCandidate> bearings;
      const int required = std::max(3, (2 * consensus_count + 2) / 3);
      for (const auto &region : regions) {
        std::vector<double> costs;
        for (const auto &p : pointings) {
          if (!p.is_valid || p.confidence < kMinPointingConfidence ||
              !p.plane_valid || !p.measured_depth) continue;
          const double norm = std::sqrt(p.direction.x*p.direction.x +
            p.direction.y*p.direction.y + p.direction.z*p.direction.z);
          const double pn = std::sqrt(p.plane_normal.x*p.plane_normal.x +
            p.plane_normal.y*p.plane_normal.y + p.plane_normal.z*p.plane_normal.z);
          if (!std::isfinite(norm) || !std::isfinite(pn) || norm < 1e-6 || pn < 1e-6 ||
              std::abs(p.direction.z)/norm > 0.20 ||
              std::abs(p.plane_normal.z)/pn < 0.95) continue;
          if (!pointingHalfspaceContains(region.center, p.plane_origin, p.plane_forward, 0.03) ||
              !pointingHalfspaceContains(region.center, p.plane_origin, p.camera_forward)) continue;
          auto bearing_point = region.center;
          bearing_point.z = p.origin.z;
          if (pointingPlaneAngle(bearing_point, p.plane_origin, p.plane_normal) > 0.12)
            continue;
          const double distance = pointRayDist3D(
            region.center.x, region.center.y, region.center.z,
            p.origin.x, p.origin.y, p.origin.z,
            p.direction.x, p.direction.y, p.direction.z);
          if (distance > region.allowed_radius + kWorldRayRadiusSlack) continue;
          double along = 0;
          const double planar = pointRayDist2D(region.center.x, region.center.y,
            p.origin.x, p.origin.y, p.direction.x, p.direction.y, &along);
          if (along < kMinAlongDistance) continue;
          const double angle = std::atan2(planar, along);
          if (angle > 0.30) continue;
          costs.push_back(angle);
        }
        if (static_cast<int>(costs.size()) >= required)
          bearings.push_back({&region, median(costs), static_cast<int>(costs.size())});
      }
      std::sort(bearings.begin(), bearings.end(),
        [](const auto &a, const auto &b) { return a.cost < b.cost; });
      if (!bearings.empty() &&
          (bearings.size() == 1 || bearings[1].cost - bearings[0].cost >= 0.05)) {
        const auto &best = bearings.front();
        result.valid = true;
        result.name = best.region->name;
        result.position = best.region->center;
        result.placement_mode = best.region->placement_mode;
        result.score = 1.0/(1.0 + best.cost);
        result.supporting_samples = best.support;
        result.horizontal_bearing_used = true;
        return result;
      }
      result.rejection_reason = "image plane and measured depth have no consistent destination";
      return result;
    }
    std::sort(plane_candidates.begin(), plane_candidates.end(),
      [](const PlaneCandidate &a, const PlaneCandidate &b) { return a.cost < b.cost; });
    const auto &best = plane_candidates.front();
    if (plane_candidates.size() > 1 && plane_candidates[1].cost - best.cost < 0.03) {
      result.rejection_reason = "destination geometry is ambiguous";
      return result;
    }
    result.valid = true;
    result.name = best.region->name;
    result.position = best.region->center;
    result.placement_mode = best.region->placement_mode;
    result.score = 1.0 / (1.0 + best.cost);
    result.supporting_samples = best.support;
    result.image_evidence_used = true;
    return result;
  }
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
  double best_image_distance = kInvalidDistance;
  int best_image_support = 0;
  for (const auto &region : regions) {
    if (region.image_projection_valid &&
        region.image_ray_distance_px < best_image_distance) {
      best_image_distance = region.image_ray_distance_px;
      best_image_support = region.image_supporting_samples;
    }
  }
  // An image line is useful only when it actually passes close to at least
  // one mapped destination. If every candidate is far from that line (as in
  // task 1 with all relevant places outside the image), extrapolating it for
  // thousands of pixels amplifies a small landmark error and must not
  // override the depth-independent pose-world arm direction.
  const bool use_image_evidence =
    best_image_support >= kMinAnchoredImageSamples &&
    std::atan2(best_image_distance, kCameraFocalLengthPx) <=
      kMaxAnchoredImageAngle;

  std::vector<NormalizedPointing> measured_cluster;
  std::copy_if(
    cluster.begin(), cluster.end(), std::back_inserter(measured_cluster),
    [](const NormalizedPointing &sample) { return sample.measured_depth; });
  const bool has_measured_world_geometry = !measured_cluster.empty();

  // The image ray and projected destination share the same camera frame. A
  // measured 3-D ray that projects somewhere else is therefore contaminated
  // (typically the thin synthetic arm was mixed with background depth). A
  // stable, unique image match is a hard directional constraint; RGB-D still
  // supplies the preferred 3-D path when it agrees, and remains the fallback
  // when image evidence is absent or ambiguous.
  if (!has_measured_world_geometry && use_image_evidence) {
    std::vector<const DestinationRegion *> image_candidates;
    for (const auto &region : regions) {
      const double image_angle = std::atan2(
        std::max(0.0, region.image_ray_distance_px),
        kCameraFocalLengthPx);
      if (region.image_projection_valid &&
          region.image_supporting_samples >= kMinAnchoredImageSamples &&
          image_angle <= kMaxAnchoredImageAngle) {
        image_candidates.push_back(&region);
      }
    }
    std::sort(
      image_candidates.begin(), image_candidates.end(),
      [](const DestinationRegion *lhs, const DestinationRegion *rhs) {
        return lhs->image_ray_distance_px < rhs->image_ray_distance_px;
      });
    if (!image_candidates.empty()) {
      const auto *best = image_candidates.front();
      const double best_angle = std::atan2(
        std::max(0.0, best->image_ray_distance_px), kCameraFocalLengthPx);
      if (image_candidates.size() > 1) {
        const auto *runner_up = image_candidates[1];
        const double runner_up_angle = std::atan2(
          std::max(0.0, runner_up->image_ray_distance_px),
          kCameraFocalLengthPx);
        result.runner_up_name = runner_up->name;
        result.runner_up_cost = runner_up_angle;
        if (runner_up_angle - best_angle < kImageAmbiguityMargin) {
          result.rejection_reason =
            "best and second-best image rays are ambiguous: " +
            best->name + " vs " + runner_up->name;
          return result;
        }
      }
      if (!has_measured_world_geometry ||
          best->image_supporting_samples >= kMinImageDepthOverrideSamples) {
        result.valid = true;
        result.name = best->name;
        result.position = best->center;
        result.placement_mode = best->placement_mode;
        result.score = 1.0 / (1.0 + best_angle);
        result.ray_distance = kInvalidDistance;
        result.three_d_ray_distance = kInvalidDistance;
        result.surface_intersection_distance = kInvalidDistance;
        result.image_ray_distance_px = best->image_ray_distance_px;
        result.along_distance = kInvalidDistance;
        result.supporting_samples = best->image_supporting_samples;
        result.image_evidence_used = true;
        return result;
      }
    }
  }

  const auto &geometry_cluster = has_measured_world_geometry
    ? measured_cluster : cluster;
  const int required_support = std::max(
    1, static_cast<int>(std::ceil(geometry_cluster.size() * 0.50)));

  // Preserve the measured direction; candidates cannot reverse an arm ray.
  for (int sign : {1}) {
    for (const auto &region : regions) {
      if (use_image_evidence && region.image_projection_valid &&
          std::atan2(std::max(0.0, region.image_ray_distance_px),
            kCameraFocalLengthPx) > kMaxAnchoredImageAngle) continue;
      std::vector<double> planar_ray_distances;
      std::vector<double> three_d_ray_distances;
      std::vector<double> surface_intersection_distances;
      std::vector<double> along_distances;
      for (const auto &sample : geometry_cluster) {
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
      const bool has_surface_intersection =
        median_surface_intersection < kInvalidDistance;
      // Image evidence may rank two physically plausible regions, but may
      // never rescue a region which the measured world ray misses by metres.
      if (median_three_d_ray > region.allowed_radius + kWorldRayRadiusSlack) {
        continue;
      }
      if (has_surface_intersection &&
          median_surface_intersection >
            region.allowed_radius + kSurfaceHitRadiusSlack) {
        continue;
      }
      const double world_cost = has_surface_intersection
        ? median_planar_ray + 0.30 * median_surface_intersection +
          0.10 * median_three_d_ray + 0.05 * mean_planar_ray
        : median_three_d_ray + 0.25 * median_planar_ray +
          0.05 * mean_planar_ray;
      double cost = world_cost;
      if (use_image_evidence && region.image_projection_valid) {
        // Bearing-only image geometry is a tie-break, never a 3-D veto bypass.
        const double image_angle = std::atan2(
          std::max(0.0, region.image_ray_distance_px),
          kCameraFocalLengthPx);
        cost = world_cost + 0.75 * image_angle;
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
    result.rejection_reason =
      "no destination is physically consistent with the 3-D gesture";
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
  result.image_evidence_used =
    use_image_evidence && best.region->image_projection_valid &&
    std::atan2(
      std::max(0.0, best.region->image_ray_distance_px),
      kCameraFocalLengthPx) <= kMaxAnchoredImageAngle;
  result.reversed = best.reversed;
  return result;
}

}  // namespace interactive_cleanup
