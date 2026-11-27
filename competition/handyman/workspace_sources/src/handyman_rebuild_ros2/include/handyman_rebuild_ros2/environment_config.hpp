#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "handyman_rebuild_ros2/task.hpp"

namespace handyman_rebuild_ros2
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RoomConfig
{
  std::vector<std::pair<double, double>> region;
  std::vector<Pose2D> search_points;
};

struct DestinationCandidate
{
  std::string room;
  Pose2D pose;
};

struct RoomRoute
{
  std::string from_room;
  std::string to_room;
  std::vector<Pose2D> waypoints;
};

struct EnvironmentConfig
{
  std::string name;
  std::string internal_name;
  std::string map;
  Pose2D initial_pose;
  std::unordered_map<std::string, RoomConfig> rooms;
  std::unordered_map<std::string, std::vector<DestinationCandidate>> destinations;
  std::vector<RoomRoute> routes;
};

class EnvironmentCatalog
{
public:
  bool loadFromFile(const std::string & catalog_path, std::string & error);
  const EnvironmentConfig * find(const std::string & name) const noexcept;
  bool resolveTask(HandymanTask & task, std::string & error) const;
  const std::unordered_map<std::string, EnvironmentConfig> & environments() const noexcept;

private:
  std::unordered_map<std::string, EnvironmentConfig> environments_;
};

}  // namespace handyman_rebuild_ros2
