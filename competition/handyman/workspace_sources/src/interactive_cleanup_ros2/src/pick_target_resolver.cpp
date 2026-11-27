#include "interactive_cleanup/pick_target_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace interactive_cleanup
{

namespace
{

constexpr double kInvalidDistance = 9999.0;
constexpr double kMinPointingConfidence = 0.35;
constexpr double kConfidenceWeight = 0.25;
constexpr double kPointing3DWeight = 5.0;
constexpr double kPointing2DWeight = 0.75;
constexpr double kPointing3DScale = 0.25;
constexpr double kPointing2DScale = 60.0;

double pointRayDist2D(
  double px,
  double py,
  double ox,
  double oy,
  double dx,
  double dy)
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
  if (proj < 0.0) {
    return kInvalidDistance;
  }

  const double perp_x = vx - proj * dx;
  const double perp_y = vy - proj * dy;
  return std::hypot(perp_x, perp_y);
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
  double dz)
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
  if (proj < 0.0) {
    return kInvalidDistance;
  }

  const double perp_x = vx - proj * dx;
  const double perp_y = vy - proj * dy;
  const double perp_z = vz - proj * dz;
  return std::sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z);
}

double scoreCandidate(
  const PickCandidate &candidate,
  const std::vector<PointingObservation> &pointings)
{
  double best_score = kConfidenceWeight * candidate.confidence;
  bool has_pointing_score = false;

  for (const auto &pointing : pointings) {
    if (
      !pointing.is_valid ||
      pointing.confidence < kMinPointingConfidence ||
      !candidate.has_3d_position)
    {
      continue;
    }

    const double ray_d_3d = pointRayDist3D(
      candidate.position.x, candidate.position.y, candidate.position.z,
      pointing.origin.x, pointing.origin.y, pointing.origin.z,
      pointing.direction.x, pointing.direction.y, pointing.direction.z);
    const double ray_d_2d = pointRayDist2D(
      candidate.bbox_cx, candidate.bbox_cy,
      pointing.wrist_pixel_x, pointing.wrist_pixel_y,
      pointing.point_pixel_x - pointing.wrist_pixel_x,
      pointing.point_pixel_y - pointing.wrist_pixel_y);

    const double pointing_weight = std::clamp(pointing.confidence, 0.0, 1.0);
    const double score = kConfidenceWeight * candidate.confidence +
      (ray_d_3d < kInvalidDistance - 1.0 ?
      (kPointing3DWeight * pointing_weight) / (1.0 + ray_d_3d / kPointing3DScale) : 0.0) +
      (ray_d_2d < kInvalidDistance - 1.0 ?
      (kPointing2DWeight * pointing_weight) / (1.0 + ray_d_2d / kPointing2DScale) : 0.0);

    best_score = std::max(best_score, score);
    has_pointing_score = true;
  }

  if (has_pointing_score) {
    return best_score;
  }

  return kConfidenceWeight * candidate.confidence;
}

}  // namespace

PickResolution resolvePickTarget(
  const std::vector<PickCandidate> &candidates,
  const std::vector<PointingObservation> &pointings)
{
  PickResolution result;
  double best_score = -std::numeric_limits<double>::infinity();

  for (const auto &candidate : candidates) {
    const double score = scoreCandidate(candidate, pointings);
    if (score <= best_score) {
      continue;
    }

    best_score = score;
    result.valid = true;
    result.class_name = candidate.class_name;
    result.position = candidate.position;
    result.score = score;
    result.grasp_mode = classifyGraspMode(candidate.position.z);
  }

  return result;
}

std::string classifyGraspMode(double target_height)
{
  if (target_height < 0.08) {
    return "floor_pick";
  }
  if (target_height < 0.22) {
    return "table_low_pick";
  }
  return "table_mid_pick";
}

}  // namespace interactive_cleanup
