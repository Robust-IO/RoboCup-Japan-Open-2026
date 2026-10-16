#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace handyman_rebuild_ros2 {
inline double turnError(double target, double current) {
  return std::atan2(std::sin(target-current), std::cos(target-current));
}
// No minimum-speed floor and no reversal near arrival. A 0.45 rad/s cap with
// 0.8 rad/s^2 acceleration gives ~2.3 seconds for an ideal 45-degree turn.
inline double turnSpeed(double remaining, double previous, double dt) {
  if (!std::isfinite(remaining) || !std::isfinite(previous) ||
      !std::isfinite(dt) || previous<0 || dt<=0 || dt>.5)
    throw std::invalid_argument("invalid turn feedback interval");
  if (remaining<=.035) return 0.;
  // Proportional approach damps delayed feedback; braking bound and ramp-up
  // bound apply independently. Emergency deceleration is never rate limited.
  return std::min({.45, previous+.8*dt, 1.8*(remaining-.025),
                   std::sqrt(2.*.8*(remaining-.035))});
}
}
