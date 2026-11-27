#pragma once

namespace interactive_cleanup
{

struct AvatarTrackCommand
{
  double angular_z{0.0};
  bool tracking{false};
};

AvatarTrackCommand computeAvatarTrackCommand(
  double center_error_x,
  double deadband,
  double gain,
  double max_angular);

}  // namespace interactive_cleanup
