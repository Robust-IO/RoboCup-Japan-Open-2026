cmake_minimum_required(VERSION 3.8)
project(handyman_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(handyman_sample
  src/handyman_sample.cpp
  src/room_navigation.cpp
)
ament_target_dependencies(handyman_sample
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  action_msgs
  handyman_msgs
  ament_index_cpp
  OpenCV
)

add_executable(teleop_key_handyman src/teleop_key_handyman.cpp)
ament_target_dependencies(teleop_key_handyman
  rclcpp
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  handyman_msgs
)

install(TARGETS
  handyman_sample
  teleop_key_handyman
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  launch
  maps
  param
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

add_library(placement_navigation_lib
  src/placement_navigation.cpp
)
target_include_directories(placement_navigation_lib PUBLIC include)

target_link_libraries(handyman_sample
  placement_navigation_lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(room_navigation_test
    test/room_navigation_test.cpp
    src/room_navigation.cpp
  )
  target_include_directories(room_navigation_test PUBLIC include)

  ament_add_gtest(placement_navigation_test
    test/placement_navigation_test.cpp
  )
  target_include_directories(placement_navigation_test PRIVATE include)
  target_link_libraries(placement_navigation_test
    placement_navigation_lib
  )
endif()

ament_package()
