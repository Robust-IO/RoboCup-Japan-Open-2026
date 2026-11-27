#include "interactive_cleanup/avatar_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace interactive_cleanup
{

AvatarTrackCommand computeAvatarTrackCommand(
  double center_error_x,
  double deadband,
  double gain,
  double max_angular)
{
  AvatarTrackCommand cmd;
  if (std::abs(center_error_x) <= std::max(0.0, deadband) ||
      gain <= 0.0 ||
      max_angular <= 0.0) {
    return cmd;
  }

  cmd.tracking = true;
  cmd.angular_z = std::clamp(-center_error_x * gain, -max_angular, max_angular);
  return cmd;
}

}  // namespace interactive_cleanup
