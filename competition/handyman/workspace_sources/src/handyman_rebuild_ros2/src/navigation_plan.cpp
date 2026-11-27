#include "handyman_rebuild_ros2/navigation_plan.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace bg = boost::geometry;

namespace handyman_rebuild_ros2
{
namespace
{

using GeometryPoint = bg::model::d2::point_xy<double>;
using GeometryPolygon = bg::model::polygon<GeometryPoint>;

GeometryPolygon makePolygon(const RoomConfig & room)
{
  GeometryPolygon polygon;
  for (const auto & point : room.region) {
    bg::append(polygon.outer(), GeometryPoint(point.first, point.second));
  }
  bg::correct(polygon);
  return polygon;
}

void orderByDistance(
  std::vector<NavigationCandidate> & candidates,
  const std::optional<Pose2D> & robot_pose)
{
  if (!robot_pose) {
    return;
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [&robot_pose](const NavigationCandidate & left, const NavigationCandidate & right) {
      return std::hypot(left.pose.x - robot_pose->x, left.pose.y - robot_pose->y) <
             std::hypot(right.pose.x - robot_pose->x, right.pose.y - robot_pose->y);
    });
}

std::vector<const RoomRoute *> routesForTarget(
  const EnvironmentConfig & environment,
  const std::string & target_room,
  const std::optional<Pose2D> & robot_pose)
{
  if (!robot_pose) {
    return {};
  }
  std::string source_room;
  for (const auto & item : environment.rooms) {
    if (isPoseInsideRoom(item.second, *robot_pose, 0.20)) {
      source_room = item.first;
      break;
    }
  }
  if (source_room.empty() || source_room == target_room) {
    return {};
  }
  std::vector<const RoomRoute *> matches;
  for (const auto & route : environment.routes) {
    if (route.from_room == source_room && route.to_room == target_room) {
      matches.push_back(&route);
    }
  }
  return matches;
}

void expandRouteCandidates(
  std::vector<NavigationCandidate> & candidates,
  const std::vector<const RoomRoute *> & routes)
{
  if (routes.empty()) {
    return;
  }
  std::vector<NavigationCandidate> expanded;
  expanded.reserve(candidates.size() * routes.size());
  // Keep the preferred destination/search pose fixed while trying alternate
  // doorways. A failed primary entrance therefore advances to the second
  // entrance before changing the final target pose.
  for (const auto & candidate : candidates) {
    for (const RoomRoute * route : routes) {
      NavigationCandidate copy = candidate;
      copy.route_waypoints = route->waypoints;
      expanded.push_back(std::move(copy));
    }
  }
  candidates = std::move(expanded);
}

double distanceToSegment(
  const Pose2D & pose,
  const std::pair<double, double> & start,
  const std::pair<double, double> & end)
{
  const double dx = end.first - start.first;
  const double dy = end.second - start.second;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) {
    return std::hypot(pose.x - start.first, pose.y - start.second);
  }
  const double projection = std::clamp(
    ((pose.x - start.first) * dx + (pose.y - start.second) * dy) / length_squared,
    0.0, 1.0);
  return std::hypot(
    pose.x - (start.first + projection * dx),
    pose.y - (start.second + projection * dy));
}

}  // namespace

NavigationPlan::NavigationPlan(
  std::vector<NavigationCandidate> candidates,
  std::size_t maximum_attempts,
  std::string error)
: candidates_(std::move(candidates)),
  maximum_attempts_(maximum_attempts),
  error_(std::move(error))
{
}

NavigationPlan NavigationPlan::forRoom(
  const EnvironmentConfig & environment,
  const std::string & room,
  std::size_t maximum_attempts,
  const std::optional<Pose2D> & robot_pose)
{
  const auto found = environment.rooms.find(room);
  if (found == environment.rooms.end()) {
    return {{}, 0, "Unknown room: " + room};
  }
  std::vector<NavigationCandidate> candidates;
  candidates.reserve(found->second.search_points.size());
  for (std::size_t index = 0; index < found->second.search_points.size(); ++index) {
    candidates.push_back({found->second.search_points[index], room, index, {}});
  }
  const auto routes = routesForTarget(environment, room, robot_pose);
  if (!routes.empty()) {
    expandRouteCandidates(candidates, routes);
  } else {
    orderByDistance(candidates, robot_pose);
  }
  return {std::move(candidates), maximum_attempts, {}};
}

NavigationPlan NavigationPlan::forSearchPoint(
  const EnvironmentConfig & environment, const std::string & room,
  std::size_t index, std::size_t maximum_attempts,
  const std::optional<Pose2D> & robot_pose)
{
  const auto found = environment.rooms.find(room);
  if (found == environment.rooms.end() || index >= found->second.search_points.size()) {
    return {{}, 0, "Unknown search point"};
  }
  const auto & pose = found->second.search_points[index];
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    !isPoseInsideRoom(found->second, pose))
  {
    return {{}, 0, "Invalid search point pose"};
  }
  std::vector<NavigationCandidate> candidates{{pose, room, index, {}}};
  expandRouteCandidates(candidates, routesForTarget(environment, room, robot_pose));
  return {std::move(candidates), maximum_attempts, {}};
}

NavigationPlan NavigationPlan::forDestination(
  const EnvironmentConfig & environment,
  const std::string & destination,
  const std::string & destination_room,
  std::size_t maximum_attempts,
  const std::optional<Pose2D> & robot_pose)
{
  const auto found = environment.destinations.find(destination);
  if (found == environment.destinations.end()) {
    return {{}, 0, "Unknown destination: " + destination};
  }
  std::vector<NavigationCandidate> candidates;
  for (std::size_t index = 0; index < found->second.size(); ++index) {
    const auto & candidate = found->second[index];
    if (destination_room.empty() || candidate.room == destination_room) {
      candidates.push_back({candidate.pose, candidate.room, index, {}});
    }
  }
  if (candidates.empty()) {
    return {{}, 0, "Destination has no candidate in room: " + destination_room};
  }
  const auto routes = routesForTarget(
    environment, destination_room, robot_pose);
  if (!routes.empty()) {
    expandRouteCandidates(candidates, routes);
  } else {
    orderByDistance(candidates, robot_pose);
  }
  return {std::move(candidates), maximum_attempts, {}};
}

bool NavigationPlan::valid() const noexcept
{
  return error_.empty() && !candidates_.empty() && maximum_attempts_ > 0;
}

bool NavigationPlan::exhausted() const noexcept
{
  return !valid() || attempted_ >= maximum_attempts_;
}

const NavigationCandidate * NavigationPlan::current() const noexcept
{
  if (exhausted()) {
    return nullptr;
  }
  return &candidates_[attempted_ % candidates_.size()];
}

bool NavigationPlan::advance() noexcept
{
  if (!exhausted()) {
    ++attempted_;
  }
  return !exhausted();
}

std::size_t NavigationPlan::attemptNumber() const noexcept
{
  return maximum_attempts_ == 0 ? 0 : std::min(attempted_ + 1, maximum_attempts_);
}
std::size_t NavigationPlan::maximumAttempts() const noexcept {return maximum_attempts_;}
const std::string & NavigationPlan::error() const noexcept {return error_;}

bool isPoseInsideRoom(
  const RoomConfig & room,
  const Pose2D & pose,
  double boundary_tolerance_m)
{
  if (room.region.size() < 3) {
    return false;
  }
  const GeometryPoint point(pose.x, pose.y);
  const GeometryPolygon polygon = makePolygon(room);
  if (bg::covered_by(point, polygon)) {
    return true;
  }
  if (boundary_tolerance_m <= 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < room.region.size(); ++index) {
    const auto & start = room.region[index];
    const auto & end = room.region[(index + 1) % room.region.size()];
    if (distanceToSegment(pose, start, end) <= boundary_tolerance_m) {
      return true;
    }
  }
  return false;
}

}  // namespace handyman_rebuild_ros2
