#pragma once
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cleanup_vision_ros2/msg/pointing_direction.hpp>

namespace interactive_cleanup
{
inline bool stablePickupPreview(
  const std::vector<cleanup_vision_ros2::msg::PointingDirection> &samples)
{
  if (samples.size() < 3) return false;
  const auto &last = samples.back();
  const auto ns = [](const auto &p) -> int64_t {
    return int64_t(p.header.stamp.sec)*1000000000LL+p.header.stamp.nanosec;
  };
  if (!last.is_valid || last.confidence < 0.5) return false;
  int support = 0;
  int64_t oldest = ns(last);
  for (const auto &p : samples) {
    const auto age = ns(last)-ns(p);
    if (!p.is_valid || p.confidence < 0.5 || age < 0 || age > 1500000000LL)
      continue;
    const auto &a = last.direction;
    const auto &b = p.direction;
    const double norm = std::sqrt((a.x*a.x+a.y*a.y+a.z*a.z)*
                                 (b.x*b.x+b.y*b.y+b.z*b.z));
    const double displacement = std::sqrt(
      std::pow(last.origin.x-p.origin.x, 2)+
      std::pow(last.origin.y-p.origin.y, 2)+
      std::pow(last.origin.z-p.origin.z, 2));
    if (norm < 1e-6 || (a.x*b.x+a.y*b.y+a.z*b.z)/norm < std::cos(0.20) ||
        displacement > 0.15) continue;
    ++support;
    oldest = std::min(oldest, ns(p));
  }
  return support >= 3 && ns(last)-oldest >= 600000000LL;
}

// Capture timestamps, rather than inference completion order, determine phase.
struct GestureCaptureWindow
{
  int64_t begin_ns{0};
  int64_t end_ns{0};
  void open(int64_t stamp) { begin_ns = stamp; end_ns = 0; }
  void close(int64_t stamp) { if (end_ns == 0) end_ns = stamp; }
  bool accepts(int64_t stamp) const
  {
    return begin_ns > 0 && stamp >= begin_ns && (end_ns == 0 || stamp <= end_ns);
  }
};
}  // namespace interactive_cleanup
