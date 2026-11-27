cmake_minimum_required(VERSION 3.5)
project(competition_test_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CURSES_NEED_WIDE TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Curses REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(interactive_cleanup_msgs REQUIRED)
find_package(human_navigation_msgs REQUIRED)

add_executable(teleop_handyman src/teleop_handyman.cpp)
ament_target_dependencies(teleop_handyman rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs handyman_msgs)
target_link_libraries(teleop_handyman ${CURSES_LIBRARIES})
target_include_directories(teleop_handyman PRIVATE include ${CURSES_INCLUDE_DIR})

add_executable(teleop_interactive_cleanup src/teleop_interactive_cleanup.cpp)
ament_target_dependencies(teleop_interactive_cleanup rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs interactive_cleanup_msgs)
target_link_libraries(teleop_interactive_cleanup ${CURSES_LIBRARIES})
target_include_directories(teleop_interactive_cleanup PRIVATE include ${CURSES_INCLUDE_DIR})

add_executable(teleop_human_navigation src/teleop_human_navigation.cpp)
ament_target_dependencies(teleop_human_navigation rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs human_navigation_msgs)
target_link_libraries(teleop_human_navigation ${CURSES_LIBRARIES})
target_include_directories(teleop_human_navigation PRIVATE include ${CURSES_INCLUDE_DIR})

add_executable(handyman_sample src/handyman_sample.cpp)
ament_target_dependencies(handyman_sample rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs handyman_msgs)
target_link_libraries(handyman_sample ${CURSES_LIBRARIES})
target_include_directories(handyman_sample PRIVATE include ${CURSES_INCLUDE_DIR})

add_executable(interactive_cleanup_sample src/interactive_cleanup_sample.cpp)
ament_target_dependencies(interactive_cleanup_sample rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs interactive_cleanup_msgs)
target_link_libraries(interactive_cleanup_sample ${CURSES_LIBRARIES})
target_include_directories(interactive_cleanup_sample PRIVATE include ${CURSES_INCLUDE_DIR})

add_executable(human_navigation_sample src/human_navigation_sample.cpp)
ament_target_dependencies(human_navigation_sample rclcpp std_msgs geometry_msgs sensor_msgs trajectory_msgs tf2_ros tf2_geometry_msgs human_navigation_msgs)
target_link_libraries(human_navigation_sample ${CURSES_LIBRARIES})
target_include_directories(human_navigation_sample PRIVATE include ${CURSES_INCLUDE_DIR})

install(
  TARGETS teleop_handyman teleop_interactive_cleanup teleop_human_navigation handyman_sample interactive_cleanup_sample human_navigation_sample
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

