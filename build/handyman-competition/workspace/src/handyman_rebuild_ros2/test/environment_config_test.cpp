#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_map_server/map_io.hpp>
#include <yaml-cpp/yaml.h>

#include "handyman_rebuild_ros2/environment_config.hpp"

using handyman_rebuild_ros2::HandymanTask;
using handyman_rebuild_ros2::EnvironmentCatalog;

namespace
{

std::string resolvePackageUri(const std::string & uri)
{
  constexpr char prefix[] = "package://";
  const std::string resource = uri.substr(sizeof(prefix) - 1);
  const std::size_t separator = resource.find('/');
  const std::string package = resource.substr(0, separator);
  return (
    std::filesystem::path(ament_index_cpp::get_package_share_directory(package)) /
    resource.substr(separator + 1)).string();
}

}  // namespace

TEST(EnvironmentConfigTest, LoadsExactlyFourLayouts)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  EXPECT_EQ(catalog.environments().size(), 4u);
  EXPECT_NE(catalog.find("LayoutA"), nullptr);
  EXPECT_NE(catalog.find("LayoutB"), nullptr);
  EXPECT_NE(catalog.find("LayoutC"), nullptr);
  EXPECT_NE(catalog.find("LayoutD"), nullptr);
}

TEST(EnvironmentConfigTest, ResolvesUnityAndInternalEnvironmentNames)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  const auto * unity = catalog.find("LayoutC");
  const auto * internal = catalog.find("LayoutC");
  ASSERT_NE(unity, nullptr);
  ASSERT_NE(internal, nullptr);
  EXPECT_EQ(unity, internal);
}

TEST(EnvironmentConfigTest, EveryLayoutHasMapRoomsSearchPointsAndDestinations)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  for (const auto & item : catalog.environments()) {
    const auto & environment = item.second;
    EXPECT_EQ(environment.map.rfind("package://", 0), 0u);
    EXPECT_FALSE(environment.rooms.empty());
    EXPECT_FALSE(environment.destinations.empty());
    EXPECT_FALSE(environment.routes.empty());
    for (const auto & room : environment.rooms) {
      EXPECT_GE(room.second.region.size(), 3u);
      EXPECT_FALSE(room.second.search_points.empty());
    }
    for (const auto & destination : environment.destinations) {
      EXPECT_FALSE(destination.second.empty());
    }
  }
}

TEST(EnvironmentConfigTest, Nav2LoadsEveryConfiguredMapIntoANonEmptyOccupancyGrid)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;

  for (const auto & item : catalog.environments()) {
    const auto & environment = item.second;
    nav_msgs::msg::OccupancyGrid map;
    const std::string map_yaml = resolvePackageUri(environment.map);
    ASSERT_EQ(nav2_map_server::loadMapFromYaml(map_yaml, map), nav2_map_server::LOAD_MAP_SUCCESS)
      << environment.name << " failed to load " << map_yaml;
    EXPECT_GT(map.info.width, 0u) << environment.name;
    EXPECT_GT(map.info.height, 0u) << environment.name;
    EXPECT_GT(map.info.resolution, 0.0F) << environment.name;
    EXPECT_EQ(
      map.data.size(),
      static_cast<std::size_t>(map.info.width) * static_cast<std::size_t>(map.info.height))
      << environment.name;
  }
}

TEST(EnvironmentConfigTest, KeepsMultipleCandidatesForAmbiguousDestinations)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  const auto * layout = catalog.find("LayoutD");
  ASSERT_NE(layout, nullptr);
  EXPECT_GT(layout->destinations.at("white_side_table").size(), 1u);
  EXPECT_GT(layout->destinations.at("wagon").size(), 1u);
}

TEST(EnvironmentConfigTest, ResolvesUniqueDestinationRoomFromCurrentLayout)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  HandymanTask task;
  task.environment = "LayoutC";
  task.pickup_room = "kitchen";
  task.destination = "corner_sofa";
  ASSERT_TRUE(catalog.resolveTask(task, error)) << error;
  EXPECT_EQ(task.destination_room, "living_room");
}

TEST(EnvironmentConfigTest, RejectsEntitiesMissingFromCurrentLayout)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  HandymanTask missing_room;
  missing_room.environment = "LayoutB";
  missing_room.pickup_room = "bedroom";
  missing_room.destination = "wagon";
  EXPECT_FALSE(catalog.resolveTask(missing_room, error));

  HandymanTask missing_destination;
  missing_destination.environment = "LayoutA";
  missing_destination.pickup_room = "kitchen";
  missing_destination.destination = "iron_bed";
  EXPECT_FALSE(catalog.resolveTask(missing_destination, error));
}

TEST(EnvironmentConfigTest, ResolvesAmbiguousDestinationInExplicitPickupRoom)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  HandymanTask task;
  task.environment = "LayoutC";
  task.pickup_room = "lobby";
  task.destination = "trash_box_for_burnable";
  ASSERT_TRUE(catalog.resolveTask(task, error)) << error;
  EXPECT_EQ(task.destination_room, "lobby");
}

TEST(EnvironmentConfigTest, RequiresRoomWhenAmbiguousDestinationIsOutsidePickupRoom)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  HandymanTask task;
  task.environment = "LayoutC";
  task.pickup_room = "kitchen";
  task.destination = "trash_box_for_burnable";
  EXPECT_FALSE(catalog.resolveTask(task, error));
  task.destination_room = "lobby";
  EXPECT_TRUE(catalog.resolveTask(task, error)) << error;
}

TEST(EnvironmentConfigTest, LayoutCBedroomRoutesStageWithinExportedOpening)
{
  EnvironmentCatalog catalog;
  std::string error;
  ASSERT_TRUE(catalog.loadFromFile(
    std::string(HANDYMAN_CONFIG_DIR) + "/environments.yaml", error)) << error;
  const auto * layout = catalog.find("LayoutC");
  ASSERT_NE(layout, nullptr);

  const auto semantic = YAML::LoadFile(
    (std::filesystem::path(resolvePackageUri(layout->map))
    .parent_path() / "semantic_map.json").string());
  YAML::Node door;
  for (std::size_t index = 0; index < semantic["doorways"].size(); ++index) {
    const YAML::Node item = semantic["doorways"][index];
    if (item["room"].as<std::string>() == "bedroom") {door = item; break;}
  }
  ASSERT_TRUE(door.IsDefined());
  const double center_x = door["center"]["x"].as<double>();
  const double inside_y = door["inside_pose"]["y"].as<double>();
  const double outside_y = door["outside_pose"]["y"].as<double>();
  const double lateral_limit = door["width"].as<double>() / 2.0 - 0.30;
  std::size_t checked_routes = 0;
  for (const auto & route : layout->routes) {
    if (route.from_room != "bedroom" && route.to_room != "bedroom") {
      continue;
    }
    ++checked_routes;
    // The repaired fallback must retain the exact centerline and outward yaw.
    if (route.from_room == "bedroom" && route.to_room == "living_room") {
      ASSERT_GE(route.waypoints.size(), 2u);
      EXPECT_NEAR(route.waypoints[0].x, center_x, 1e-5);
      EXPECT_NEAR(route.waypoints[1].x, center_x, 1e-5);
      EXPECT_NEAR(route.waypoints[0].y, inside_y, 1e-5);
      EXPECT_NEAR(route.waypoints[1].y, outside_y, 1e-5);
      EXPECT_NEAR(route.waypoints[0].yaw, 1.57079632679, 1e-5);
      EXPECT_NEAR(route.waypoints[1].yaw, 1.57079632679, 1e-5);
    }
    bool has_inside_stage = false;
    bool has_outside_stage = false;
    for (const auto & waypoint : route.waypoints) {
      if (std::abs(waypoint.x - center_x) > lateral_limit) {
        continue;
      }
      has_inside_stage = has_inside_stage || std::abs(waypoint.y - inside_y) <= 0.05;
      has_outside_stage = has_outside_stage || std::abs(waypoint.y - outside_y) <= 0.05;
    }
    EXPECT_TRUE(has_inside_stage) << route.from_room << " -> " << route.to_room;
    EXPECT_TRUE(has_outside_stage) << route.from_room << " -> " << route.to_room;
  }
  EXPECT_EQ(checked_routes, 4u);
}
