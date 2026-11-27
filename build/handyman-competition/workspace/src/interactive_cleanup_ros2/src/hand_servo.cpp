#include "interactive_cleanup/hand_servo.hpp"

#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

HandServoCommand computeHandServoCommand(
  const HandServoInput &input,
  double max_linear_x,
  double max_linear_y,
  double max_lift_delta)
{
  HandServoCommand command;
  if (!input.target_found || input.confidence < 0.2) {
    return command;
  }

  if (input.in_grasp_window) {
    command.aligned = true;
    command.should_close = true;
    return command;
  }

  const double desired_area = 0.12;
  command.linear_x = std::clamp(
    (desired_area - input.bbox_area_ratio) * 0.7,
    -max_linear_x,
    max_linear_x);
  command.linear_y = std::clamp(
    -input.pixel_error_x / 320.0 * max_linear_y,
    -max_linear_y,
    max_linear_y);
  command.lift_delta = std::clamp(
    -input.pixel_error_y / 240.0 * max_lift_delta,
    -max_lift_delta,
    max_lift_delta);

  const bool centered =
    std::abs(input.pixel_error_x) <= 18.0 &&
    std::abs(input.pixel_error_y) <= 18.0;
  const bool close_enough =
    input.bbox_area_ratio >= 0.09 && input.bbox_area_ratio <= 0.16;
  command.aligned = centered && close_enough;
  return command;
}

bool shouldRetryHandApproach(
  bool in_grasp_window,
  bool target_found,
  int consecutive_missing_frames,
  int max_missing_frames,
  double elapsed,
  double timeout)
{
  if (in_grasp_window) {
    return false;
  }
  if (elapsed > timeout) {
    return true;
  }
  return !target_found && consecutive_missing_frames > max_missing_frames;
}

}  // namespace interactive_cleanup
