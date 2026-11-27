cmake_minimum_required(VERSION 3.8)
project(cleanup_vision_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/PointingDirection.msg"
  "msg/AvatarObservation.msg"
  "msg/SceneObject.msg"
  "msg/SceneObjectArray.msg"
  "msg/HandTargetAlignment.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(FILES
  scripts/perception_common.py
  scripts/perception_visualization.py
  scripts/pointing_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cleanup_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleanup_detection_node
)

install(PROGRAMS
  scripts/head_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME head_perception_node
)

install(PROGRAMS
  scripts/hand_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hand_perception_node
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME cleanup_vision_perception_common_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/perception_common_test.py
  )

  add_test(
    NAME cleanup_vision_perception_visualization_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/perception_visualization_test.py
  )

  add_test(
    NAME cleanup_vision_head_pointing_geometry_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/head_pointing_geometry_test.py
  )
endif()

ament_package()
