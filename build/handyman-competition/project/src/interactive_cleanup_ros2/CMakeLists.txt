cmake_minimum_required(VERSION 3.8)
project(interactive_cleanup)

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
find_package(yaml_cpp_vendor REQUIRED)
find_package(cleanup_vision_ros2 REQUIRED)
find_package(interactive_cleanup_msgs REQUIRED)

# Main controller node
add_executable(interactive_cleanup_sample
  src/interactive_cleanup_sample.cpp
  src/navigation_utils.cpp
  src/grasp_utils.cpp
  src/avatar_tracker.cpp
  src/observation_head_sweep.cpp
  src/pick_target_resolver.cpp
  src/pointing_alignment.cpp
  src/place_destination_resolver.cpp
  src/hsr_geometry.cpp
  src/pregrasp_planner.cpp
  src/hand_servo.cpp
)
target_include_directories(interactive_cleanup_sample PRIVATE include)
ament_target_dependencies(interactive_cleanup_sample
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
  cleanup_vision_ros2
  interactive_cleanup_msgs
)
target_link_libraries(interactive_cleanup_sample yaml-cpp)

install(TARGETS
  interactive_cleanup_sample
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/placeholder_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  map
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(navigation_utils_test
    test/navigation_utils_test.cpp
    src/navigation_utils.cpp
  )
  target_include_directories(navigation_utils_test PRIVATE include)
  ament_target_dependencies(navigation_utils_test
    geometry_msgs
    tf2
    tf2_geometry_msgs
  )

  ament_add_gtest(grasp_utils_test
    test/grasp_utils_test.cpp
    src/grasp_utils.cpp
  )
  target_include_directories(grasp_utils_test PRIVATE include)

  ament_add_gtest(avatar_tracker_test
    test/avatar_tracker_test.cpp
    src/avatar_tracker.cpp
  )
  target_include_directories(avatar_tracker_test PRIVATE include)

  ament_add_gtest(pick_target_resolver_test
    test/pick_target_resolver_test.cpp
    src/pick_target_resolver.cpp
  )
  target_include_directories(pick_target_resolver_test PRIVATE include)
  ament_target_dependencies(pick_target_resolver_test
    geometry_msgs
  )

  ament_add_gtest(place_destination_resolver_test
    test/place_destination_resolver_test.cpp
    src/place_destination_resolver.cpp
  )
  target_include_directories(place_destination_resolver_test PRIVATE include)
  ament_target_dependencies(place_destination_resolver_test
    geometry_msgs
  )
  target_link_libraries(place_destination_resolver_test yaml-cpp)

  ament_add_gtest(hsr_geometry_test
    test/hsr_geometry_test.cpp
    src/hsr_geometry.cpp
  )
  target_include_directories(hsr_geometry_test PRIVATE include)
  ament_target_dependencies(hsr_geometry_test
    geometry_msgs
  )

  ament_add_gtest(pregrasp_planner_test
    test/pregrasp_planner_test.cpp
    src/pregrasp_planner.cpp
    src/hsr_geometry.cpp
  )
  target_include_directories(pregrasp_planner_test PRIVATE include)
  ament_target_dependencies(pregrasp_planner_test
    geometry_msgs
  )

  ament_add_gtest(hand_servo_test
    test/hand_servo_test.cpp
    src/hand_servo.cpp
  )
  target_include_directories(hand_servo_test PRIVATE include)

  ament_add_gtest(pointing_alignment_test
    test/pointing_alignment_test.cpp
    src/pointing_alignment.cpp
  )
  target_include_directories(pointing_alignment_test PRIVATE include)
  ament_target_dependencies(pointing_alignment_test
    geometry_msgs
  )

  ament_add_gtest(observation_head_sweep_test
    test/observation_head_sweep_test.cpp
    src/observation_head_sweep.cpp
  )
  target_include_directories(observation_head_sweep_test PRIVATE include)

  add_test(
    NAME nav2_bt_xml_test
    COMMAND ${CMAKE_COMMAND}
      -DINPUT_FILE=${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/validate_nav2_bt_xml.cmake
  )

  add_test(
    NAME cleanup_launch_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/cleanup_launch_test.py
  )

  add_test(
    NAME placeholder_tf_shutdown_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/placeholder_tf_shutdown_test.py
  )

  add_test(
    NAME interactive_cleanup_behavior_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/interactive_cleanup_behavior_test.py
  )
endif()

ament_package()
