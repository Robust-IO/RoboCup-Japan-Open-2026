#pragma once

#include <cstddef>
#include <vector>

#include "interactive_cleanup/pick_target_resolver.hpp"

namespace interactive_cleanup
{

struct PointingAlignmentResult
{
  bool has_measurement{false};
  bool aligned{false};
  bool within_angle_threshold{false};
  std::size_t sample_count{0};
  double desired_yaw{0.0};
  double yaw_error{0.0};
  double angular_stability{0.0};
  double command_angular{0.0};
};

PointingAlignmentResult evaluatePointingAlignment(
  const std::vector<PointingObservation> &pointings,
  double robot_yaw,
  double angle_threshold,
  double stability_threshold,
  double angular_gain,
  double max_angular,
  std::size_t min_samples);

double computePointingAlignmentTimeout(
  double initial_yaw_error,
  double max_angular,
  double min_timeout,
  double timeout_overhead,
  double max_timeout);

}  // namespace interactive_cleanup
