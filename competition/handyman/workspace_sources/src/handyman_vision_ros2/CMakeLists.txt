cmake_minimum_required(VERSION 3.8)
project(handyman_vision_ros2)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/object_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_detection_node
)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
