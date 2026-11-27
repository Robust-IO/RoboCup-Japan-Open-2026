#pragma once
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/types.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace sigverse_bridge {
// Unity Header.Update publishes Unix time (sec/nanosec), not simulation uptime.
// A zero stamp means timestamps are disabled. Never silently mix that clock
// with receive-time stamps when the source-time contract was requested.
inline builtin_interfaces::msg::Time sourceStamp(bsoncxx::document::view header)
{
  const auto stamp = header["stamp"].get_document().value;
  auto integer = [](bsoncxx::document::element e) -> int64_t {
    if (e.type() == bsoncxx::type::k_int32) return e.get_int32().value;
    if (e.type() == bsoncxx::type::k_int64) return e.get_int64().value;
    throw std::invalid_argument("timestamp field must be an integer");
  };
  const int64_t sec = integer(stamp["sec"]);
  const int64_t ns = integer(stamp["nanosec"]);
  if (sec < 0 || sec > std::numeric_limits<int32_t>::max() ||
      ns < 0 || ns >= 1000000000LL || (sec == 0 && ns == 0))
    throw std::invalid_argument("invalid/zero source timestamp; enable Unity ROS timestamps");
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(sec);
  result.nanosec = static_cast<uint32_t>(ns);
  return result;
}
}  // namespace sigverse_bridge
