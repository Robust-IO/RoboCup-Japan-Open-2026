cmake_minimum_required(VERSION 3.8)
project(search_binding_fixture)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_executable(search_binding_fixture search_binding_fixture.cpp)
target_include_directories(search_binding_fixture PRIVATE
 /home/crazylearner/RoboCup-Japan-Open-2026/src/handyman_rebuild_ros2/include)
target_link_libraries(search_binding_fixture
 /tmp/handyman-search-nav-build/handyman_rebuild_ros2/libhandyman_nodes.a
 /tmp/handyman-search-nav-build/handyman_rebuild_ros2/libhandyman_core.a yaml-cpp)
ament_target_dependencies(search_binding_fixture rclcpp rclcpp_action handyman_msgs nav2_msgs tf2_ros ament_index_cpp tf2_geometry_msgs)
