#pragma once

#include <algorithm>
#include <cmath>
#include "interactive_cleanup/pick_target_resolver.hpp"

namespace interactive_cleanup
{
// Centre the visible person and indicated arm together. Tracking only the
// torso leaves an extended hand outside the image even when the torso is
// perfectly centred. Invalid landmarks must not change the tracking target.
inline double gestureFramingCenter(double person, double start, double end)
{
  if (!std::isfinite(start) || !std::isfinite(end)) return person;
  return (std::min({person, start, end}) + std::max({person, start, end})) * 0.5;
}

inline bool pointingHalfspaceContains(
  const geometry_msgs::msg::Point &point,
  const geometry_msgs::msg::Point &origin,
  const geometry_msgs::msg::Vector3 &normal, double angular_slack = 0.0)
{
  const double x = point.x-origin.x, y = point.y-origin.y, z = point.z-origin.z;
  const double n = std::sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
  if (n < 1e-6) return true;
  return x*normal.x + y*normal.y + z*normal.z >=
    -std::sin(angular_slack)*n*std::sqrt(x*x+y*y+z*z);
}
// Angular distance from a 3-D candidate to the captured image-line plane.
// Unlike pixel distances this is invariant to head motion and range.
inline double pointingPlaneAngle(
  const geometry_msgs::msg::Point &point,
  const geometry_msgs::msg::Point &origin,
  const geometry_msgs::msg::Vector3 &normal)
{
  const double x = point.x - origin.x;
  const double y = point.y - origin.y;
  const double z = point.z - origin.z;
  const double denominator = std::sqrt(x*x + y*y + z*z) *
    std::sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
  if (!std::isfinite(denominator) || denominator < 1e-6) return 3.141592653589793;
  return std::asin(std::clamp(
    std::abs(x*normal.x + y*normal.y + z*normal.z) / denominator, 0.0, 1.0));
}
}  // namespace interactive_cleanup
