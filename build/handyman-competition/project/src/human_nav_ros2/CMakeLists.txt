cmake_minimum_required(VERSION 3.8)
project(human_nav_ros2)

# 编译选项（保持简单即可）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(human_navigation_msgs REQUIRED)

add_executable(human_navigation_sample
  src/human_navigation_sample.cpp
)

ament_target_dependencies(human_navigation_sample
  rclcpp
  geometry_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  human_navigation_msgs
)

add_executable(hsr_key_teleop
  src/hsr_key_teleop.cpp
)

ament_target_dependencies(hsr_key_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  human_navigation_msgs
)

add_executable(human_navigation_sample_origin
  src/human_navigation_sample_origin.cpp
)

ament_target_dependencies(human_navigation_sample_origin
  rclcpp
  human_navigation_msgs
)

add_executable(human_navigation_sample_copy
  src/human_navigation_sample_copy.cpp
)

ament_target_dependencies(human_navigation_sample_copy
  rclcpp
  geometry_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
  human_navigation_msgs
)

install(TARGETS
  human_navigation_sample
  human_navigation_sample_origin
  human_navigation_sample_copy
  hsr_key_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# 原 ROS1: launch 随包安装 -> ROS2: 安装 launch 目录到 share 下，供 ros2 launch 使用
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
# 原 ROS1: launch/hsr.rviz -> ROS2: 安装到 share/human_nav_ros2/rviz，供 sample_with_rviz_launch 使用
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()