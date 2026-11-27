cmake_minimum_required(VERSION 3.8)
project(handyman_rebuild_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(handyman_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

add_library(handyman_search_turn_bt SHARED src/search_turn_bt.cpp)
target_include_directories(handyman_search_turn_bt PRIVATE include)
target_compile_definitions(handyman_search_turn_bt PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(handyman_search_turn_bt behaviortree_cpp_v3 nav2_behavior_tree rclcpp geometry_msgs tf2 tf2_geometry_msgs tf2_ros)
add_library(handyman_search_spin SHARED src/search_spin.cpp)
target_include_directories(handyman_search_spin PRIVATE include)
ament_target_dependencies(handyman_search_spin nav2_behaviors nav2_core nav2_util pluginlib tf2 tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core search_spin_plugin.xml)
install(TARGETS handyman_search_turn_bt handyman_search_spin LIBRARY DESTINATION lib)

add_library(handyman_core
  src/competition_protocol.cpp
  src/environment_config.cpp
  src/instruction_parser.cpp
  src/navigation_plan.cpp
  src/task_state_machine.cpp
)
ament_target_dependencies(handyman_core
  ament_index_cpp
  geometry_msgs
  handyman_msgs
  yaml_cpp_vendor
)
target_link_libraries(handyman_core yaml-cpp)
target_include_directories(handyman_core SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_compile_definitions(handyman_core PRIVATE BOOST_ALLOW_DEPRECATED_HEADERS)
target_include_directories(handyman_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(handyman_nodes
  src/coordinator_node.cpp
  src/navigation_executor.cpp
  src/simulated_moderator_node.cpp
)
target_link_libraries(handyman_nodes handyman_core yaml-cpp)
ament_target_dependencies(handyman_nodes
  ament_index_cpp
  rclcpp
  rclcpp_action
  handyman_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_include_directories(handyman_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(handyman_coordinator src/coordinator_main.cpp)
add_executable(handyman_search_worker src/search_worker_main.cpp)
target_link_libraries(handyman_search_worker handyman_nodes)
ament_target_dependencies(handyman_search_worker rclcpp)
target_link_libraries(handyman_coordinator handyman_nodes)
ament_target_dependencies(handyman_coordinator rclcpp)

add_executable(handyman_simulated_moderator
  src/simulated_moderator_main.cpp
)
target_link_libraries(handyman_simulated_moderator handyman_nodes)
ament_target_dependencies(handyman_simulated_moderator rclcpp)

add_executable(handyman_phase1_simulation
  src/phase1_simulation_main.cpp
)
target_link_libraries(handyman_phase1_simulation handyman_nodes)
ament_target_dependencies(handyman_phase1_simulation rclcpp)

add_executable(handyman_initial_pose_localizer src/initial_pose_localizer.cpp)
ament_target_dependencies(handyman_initial_pose_localizer
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  handyman_core
  handyman_nodes
  handyman_coordinator
  handyman_search_worker
  handyman_simulated_moderator
  handyman_phase1_simulation
  handyman_initial_pose_localizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY launch config behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES 阶段3导航.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/analyze_navigation_bag.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(search_turn_profile_test test/search_turn_profile_test.cpp)
  target_include_directories(search_turn_profile_test PRIVATE include)
  target_compile_options(search_turn_profile_test PRIVATE -UNDEBUG)
  add_test(NAME search_turn_profile_test COMMAND search_turn_profile_test)
  add_executable(search_turn_bt_test test/search_turn_bt_test.cpp)
  ament_target_dependencies(search_turn_bt_test behaviortree_cpp_v3 rclcpp geometry_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_compile_options(search_turn_bt_test PRIVATE -UNDEBUG)
  add_test(NAME search_turn_bt_test COMMAND search_turn_bt_test $<TARGET_FILE:handyman_search_turn_bt>)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav2_map_server REQUIRED)
  ament_add_gtest(task_state_machine_test test/task_state_machine_test.cpp)
  target_link_libraries(task_state_machine_test handyman_core)
  target_include_directories(task_state_machine_test PRIVATE include)

  ament_add_gtest(competition_protocol_test test/competition_protocol_test.cpp)
  target_link_libraries(competition_protocol_test handyman_core)
  target_include_directories(competition_protocol_test PRIVATE include)
  ament_target_dependencies(competition_protocol_test handyman_msgs)

  ament_add_gtest(instruction_parser_test test/instruction_parser_test.cpp)
  target_link_libraries(instruction_parser_test handyman_core)
  target_include_directories(instruction_parser_test PRIVATE include)
  target_compile_definitions(instruction_parser_test PRIVATE
    HANDYMAN_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )

  ament_add_gtest(environment_config_test test/environment_config_test.cpp)
  target_link_libraries(environment_config_test handyman_core)
  target_include_directories(environment_config_test PRIVATE include)
  target_compile_definitions(environment_config_test PRIVATE
    HANDYMAN_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  )
  ament_target_dependencies(environment_config_test
    ament_index_cpp
    nav2_map_server
  )

  ament_add_gtest(navigation_plan_test test/navigation_plan_test.cpp)
  target_link_libraries(navigation_plan_test handyman_core)
  target_include_directories(navigation_plan_test PRIVATE include)

  add_launch_test(
    test/protocol_roundtrip_launch_test.py
    TIMEOUT 30
  )
  add_launch_test(
    test/navigation_roundtrip_launch_test.py
    TIMEOUT 40
  )
endif()

ament_package()
