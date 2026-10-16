#include "handyman_rebuild_ros2/environment_config.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <yaml-cpp/yaml.h>

namespace bg = boost::geometry;

namespace handyman_rebuild_ros2
{
namespace
{

Pose2D parsePose(const YAML::Node & node)
{
  return {node["x"].as<double>(), node["y"].as<double>(), node["yaw"].as<double>()};
}

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

bool validateMapResource(const std::string & uri, std::string & error)
{
  constexpr char prefix[] = "package://";
  if (uri.rfind(prefix, 0) != 0) {
    error = "Map must use a package:// URI: " + uri;
    return false;
  }
  const std::string resource = uri.substr(sizeof(prefix) - 1);
  const std::size_t separator = resource.find('/');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= resource.size()) {
    error = "Invalid package map URI: " + uri;
    return false;
  }
  const std::string package = resource.substr(0, separator);
  const std::filesystem::path relative_path = resource.substr(separator + 1);
  try {
    const std::filesystem::path map_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) / relative_path;
    if (!std::filesystem::is_regular_file(map_path)) {
      error = "Map YAML does not exist: " + map_path.string();
      return false;
    }
    const YAML::Node map = YAML::LoadFile(map_path.string());
    const YAML::Node origin = map["origin"];
    if (!map["image"] || !map["resolution"] || !origin || !origin.IsSequence() ||
      origin.size() != 3 || !map["negate"] || !map["occupied_thresh"] ||
      !map["free_thresh"])
    {
      error = "Map YAML is missing required fields: " + map_path.string();
      return false;
    }
    map["resolution"].as<double>();
    map["negate"].as<int>();
    map["occupied_thresh"].as<double>();
    map["free_thresh"].as<double>();
    for (const auto & value : origin) {
      value.as<double>();
    }
    std::filesystem::path image_path = map["image"].as<std::string>();
    if (image_path.is_relative()) {
      image_path = map_path.parent_path() / image_path;
    }
    if (!std::filesystem::is_regular_file(image_path)) {
      error = "Map image does not exist: " + image_path.string();
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = "Unable to resolve map resource " + uri + ": " + exception.what();
    return false;
  }
}

bool validateRoomGeometry(
  const std::string & environment_name,
  const std::unordered_map<std::string, RoomConfig> & rooms,
  std::string & error)
{
  std::vector<std::pair<std::string, GeometryPolygon>> polygons;
  for (const auto & item : rooms) {
    GeometryPolygon polygon = makePolygon(item.second);
    std::string reason;
    if (!bg::is_valid(polygon, reason) || bg::area(polygon) <= 0.0) {
      error = environment_name + "." + item.first + " has invalid region geometry: " + reason;
      return false;
    }
    for (const auto & search_point : item.second.search_points) {
      if (!bg::covered_by(GeometryPoint(search_point.x, search_point.y), polygon)) {
        error = environment_name + "." + item.first + " has a search point outside its region";
        return false;
      }
    }
    polygons.emplace_back(item.first, std::move(polygon));
  }
  for (std::size_t left = 0; left < polygons.size(); ++left) {
    for (std::size_t right = left + 1; right < polygons.size(); ++right) {
      std::vector<GeometryPolygon> overlap;
      bg::intersection(polygons[left].second, polygons[right].second, overlap);
      double overlap_area = 0.0;
      for (const auto & polygon : overlap) {
        overlap_area += std::abs(bg::area(polygon));
      }
      if (overlap_area > 1e-6) {
        error = environment_name + " room regions overlap: " + polygons[left].first +
          " and " + polygons[right].first;
        return false;
      }
    }
  }
  return true;
}

bool parseEnvironment(
  const std::string & name, const YAML::Node & root,
  EnvironmentConfig & environment, std::string & error)
{
  if (!root["map"] || !root["internal_name"] || !root["initial_pose"]) {
    error = name + " is missing map, internal_name or initial_pose";
    return false;
  }
  environment.name = name;
  environment.internal_name = root["internal_name"].as<std::string>();
  environment.map = root["map"].as<std::string>();
  environment.initial_pose = parsePose(root["initial_pose"]);
  if (!validateMapResource(environment.map, error)) {
    return false;
  }

  const YAML::Node rooms = root["rooms"];
  if (!rooms || !rooms.IsMap() || rooms.size() == 0) {
    error = name + " has no rooms";
    return false;
  }
  for (const auto & item : rooms) {
    RoomConfig room;
    const std::string room_name = item.first.as<std::string>();
    const YAML::Node region = item.second["region"];
    const YAML::Node search_points = item.second["search_points"];
    if (!region || !region.IsSequence() || region.size() < 3) {
      error = name + "." + room_name + " needs at least three region points";
      return false;
    }
    for (const auto & point : region) {
      if (!point.IsSequence() || point.size() != 2) {
        error = name + "." + room_name + " has an invalid region point";
        return false;
      }
      room.region.emplace_back(point[0].as<double>(), point[1].as<double>());
    }
    if (!search_points || !search_points.IsSequence() || search_points.size() == 0) {
      error = name + "." + room_name + " has no search points";
      return false;
    }
    for (const auto & point : search_points) {
      room.search_points.push_back(parsePose(point));
    }
    environment.rooms.emplace(room_name, std::move(room));
  }
  if (!validateRoomGeometry(name, environment.rooms, error)) {
    return false;
  }

  const YAML::Node routes = root["routes"];
  if (routes) {
    if (!routes.IsSequence()) {
      error = name + ".routes must be a sequence";
      return false;
    }
    for (const auto & item : routes) {
      if (!item["from"] || !item["to"] || !item["waypoints"] ||
        !item["waypoints"].IsSequence())
      {
        error = name + " has an invalid room route";
        return false;
      }
      RoomRoute route;
      route.from_room = item["from"].as<std::string>();
      route.to_room = item["to"].as<std::string>();
      if (environment.rooms.count(route.from_room) == 0 ||
        environment.rooms.count(route.to_room) == 0)
      {
        error = name + " route references an unknown room";
        return false;
      }
      for (const auto & waypoint : item["waypoints"]) {
        route.waypoints.push_back(parsePose(waypoint));
      }
      environment.routes.push_back(std::move(route));
    }
  }

  const YAML::Node destinations = root["destinations"];
  if (!destinations || !destinations.IsMap() || destinations.size() == 0) {
    error = name + " has no destinations";
    return false;
  }
  for (const auto & item : destinations) {
    const std::string destination_name = item.first.as<std::string>();
    if (!item.second.IsSequence() || item.second.size() == 0) {
      error = name + "." + destination_name + " has no candidates";
      return false;
    }
    auto & candidates = environment.destinations[destination_name];
    for (const auto & candidate : item.second) {
      DestinationCandidate parsed;
      if (!candidate["room"]) {
        error = name + "." + destination_name + " candidate is missing its room";
        return false;
      }
      parsed.room = candidate["room"].as<std::string>();
      parsed.pose = parsePose(candidate);
      const auto room = environment.rooms.find(parsed.room);
      if (room == environment.rooms.end()) {
        error = name + "." + destination_name + " references unknown room " + parsed.room;
        return false;
      }
      if (!bg::covered_by(GeometryPoint(parsed.pose.x, parsed.pose.y), makePolygon(room->second))) {
        error = name + "." + destination_name + " candidate is outside room " + parsed.room;
        return false;
      }
      candidates.push_back(parsed);
    }
  }
  return true;
}

}  // namespace

bool EnvironmentCatalog::loadFromFile(const std::string & catalog_path, std::string & error)
{
  try {
    const YAML::Node catalog = YAML::LoadFile(catalog_path);
    const YAML::Node files = catalog["environment_files"];
    environments_.clear();
    if (!files || !files.IsMap() || files.size() != 4) {
      error = "Environment catalog must contain exactly four layouts";
      return false;
    }
    const std::filesystem::path base = std::filesystem::path(catalog_path).parent_path();
    for (const auto & item : files) {
      const std::string name = item.first.as<std::string>();
      const std::filesystem::path path = base / item.second.as<std::string>();
      EnvironmentConfig environment;
      if (!parseEnvironment(name, YAML::LoadFile(path.string()), environment, error)) {
        environments_.clear();
        return false;
      }
      environments_.emplace(name, std::move(environment));
    }
    return true;
  } catch (const YAML::Exception & exception) {
    environments_.clear();
    error = exception.what();
    return false;
  }
}

const EnvironmentConfig * EnvironmentCatalog::find(const std::string & name) const noexcept
{
  const auto direct = environments_.find(name);
  if (direct != environments_.end()) {
    return &direct->second;
  }
  for (const auto & item : environments_) {
    if (item.second.internal_name == name) {
      return &item.second;
    }
  }
  return nullptr;
}

bool EnvironmentCatalog::resolveTask(HandymanTask & task, std::string & error) const
{
  const EnvironmentConfig * environment = find(task.environment);
  if (environment == nullptr) {
    error = "Unknown task environment: " + task.environment;
    return false;
  }
  task.environment = environment->name;
  if (environment->rooms.count(task.pickup_room) == 0) {
    error = environment->name + " has no pickup room " + task.pickup_room;
    return false;
  }
  if (task.destination_is_avatar) {
    task.destination_room.clear();
    return true;
  }
  const auto destination = environment->destinations.find(task.destination);
  if (destination == environment->destinations.end()) {
    error = environment->name + " has no destination " + task.destination;
    return false;
  }
  if (!task.destination_room.empty()) {
    const bool matching_candidate = std::any_of(
      destination->second.begin(), destination->second.end(),
      [&task](const DestinationCandidate & candidate) {
        return candidate.room == task.destination_room;
      });
    if (!matching_candidate) {
      error = environment->name + "." + task.destination +
        " has no candidate in room " + task.destination_room;
      return false;
    }
    return true;
  }
  std::unordered_set<std::string> candidate_rooms;
  for (const auto & candidate : destination->second) {
    candidate_rooms.insert(candidate.room);
  }
  if (candidate_rooms.size() != 1) {
    // Some competition instructions name only the pickup room and disambiguate a
    // repeated destination relationally (for example, "the trash box close to
    // the kettle").  The exported navigation configuration does not yet carry
    // those object-to-object relations, but a candidate in the explicitly named
    // pickup room is still unambiguous at room level and is the correct room for
    // same-room pickup/place tasks.
    if (candidate_rooms.count(task.pickup_room) != 0) {
      task.destination_room = task.pickup_room;
      return true;
    }
    error = environment->name + "." + task.destination +
      " is ambiguous and has no candidate in pickup room " + task.pickup_room +
      "; the instruction must include its destination room";
    return false;
  }
  task.destination_room = *candidate_rooms.begin();
  return true;
}

const std::unordered_map<std::string, EnvironmentConfig> &
EnvironmentCatalog::environments() const noexcept { return environments_; }

}  // namespace handyman_rebuild_ros2
