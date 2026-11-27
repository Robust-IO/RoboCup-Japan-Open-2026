#pragma once

namespace interactive_cleanup
{

struct HandServoInput
{
  bool target_found{false};
  double pixel_error_x{0.0};
  double pixel_error_y{0.0};
  double bbox_area_ratio{0.0};
  bool in_grasp_window{false};
  double confidence{0.0};
};

struct HandServoCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double lift_delta{0.0};
  bool aligned{false};
  bool should_close{false};
};

HandServoCommand computeHandServoCommand(
  const HandServoInput &input,
  double max_linear_x,
  double max_linear_y,
  double max_lift_delta);

bool shouldRetryHandApproach(
  bool in_grasp_window,
  bool target_found,
  int consecutive_missing_frames,
  int max_missing_frames,
  double elapsed,
  double timeout);

}  // namespace interactive_cleanup
