#include "interactive_cleanup/pointing_alignment.hpp"

#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

namespace
{

constexpr double kMinPointingConfidence = 0.35;

double normalizeAngleLocal(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double angleDistance(double lhs, double rhs)
{
  return std::abs(normalizeAngleLocal(lhs - rhs));
}

}  // namespace

PointingAlignmentResult evaluatePointingAlignment(
  const std::vector<PointingObservation> &pointings,
  double robot_yaw,
  double angle_threshold,
  double stability_threshold,
  double angular_gain,
  double max_angular,
  std::size_t min_samples)
{
  PointingAlignmentResult result;

  double dir_sum_x = 0.0;
  double dir_sum_y = 0.0;
  std::vector<double> sample_yaws;
  std::vector<double> sample_weights;
  double weight_sum = 0.0;
  sample_yaws.reserve(pointings.size());
  sample_weights.reserve(pointings.size());

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
    dir_sum_x += (pointing.direction.x / planar_norm) * weight;
    dir_sum_y += (pointing.direction.y / planar_norm) * weight;
    sample_yaws.push_back(std::atan2(pointing.direction.y, pointing.direction.x));
    sample_weights.push_back(weight);
  }

  result.sample_count = sample_yaws.size();
  if (sample_yaws.empty()) {
    return result;
  }

  const double aggregate_norm = std::hypot(dir_sum_x, dir_sum_y);
  if (aggregate_norm < 1e-6) {
    return result;
  }

  result.has_measurement = true;
  result.desired_yaw = std::atan2(dir_sum_y, dir_sum_x);
  result.yaw_error = normalizeAngleLocal(result.desired_yaw - robot_yaw);
  result.within_angle_threshold = std::abs(result.yaw_error) <= std::max(0.0, angle_threshold);

  double total_error = 0.0;
  for (std::size_t index = 0; index < sample_yaws.size(); ++index) {
    total_error += angleDistance(sample_yaws[index], result.desired_yaw) * sample_weights[index];
  }
  result.angular_stability = total_error / std::max(weight_sum, 1e-6);

  if (!result.within_angle_threshold) {
    result.command_angular = std::clamp(
      result.yaw_error * angular_gain,
      -std::abs(max_angular),
      std::abs(max_angular));
  }

  result.aligned =
    result.within_angle_threshold &&
    result.sample_count >= min_samples &&
    result.angular_stability <= std::max(0.0, stability_threshold);

  return result;
}

double computePointingAlignmentTimeout(
  double initial_yaw_error,
  double max_angular,
  double min_timeout,
  double timeout_overhead,
  double max_timeout)
{
  const double speed = std::max(0.05, std::abs(max_angular));
  const double raw_timeout =
    std::abs(initial_yaw_error) / speed + std::max(0.0, timeout_overhead);
  return std::clamp(raw_timeout, std::max(0.0, min_timeout), std::max(min_timeout, max_timeout));
}

}  // namespace interactive_cleanup
