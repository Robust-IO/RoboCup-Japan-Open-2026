> [!NOTE]
> This repository is for ROS 2.


# ROS 2 Packages for Competition

This repository contains the ROS 2 messages used in the competition and the ROS 2 nodes used for testing.

Please see the [wiki page](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki) for details.


## Prerequisites

Please install ROS 2 Humble Hawksbill.

The following command examples assume your ROS 2 workspace is `~/ros2_ws`.

Also install the required libraries below, such as rosbridge-suite.

#### Install Additional Dependencies
```bash:
sudo rosdep init
rosdep update
sudo apt install -y git
sudo apt install -y libncurses-dev
sudo apt install -y python3-pip
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-$ROS_DISTRO-xacro
sudo apt install -y ros-$ROS_DISTRO-octomap
sudo apt install -y ros-$ROS_DISTRO-hardware-interface
sudo apt install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-controller-manager
sudo apt install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-ros-perception ros-$ROS_DISTRO-moveit-ros-occupancy-map-monitor
```

#### Install Mongo C Driver
```bash:
cd ~/Downloads
wget https://github.com/mongodb/mongo-c-driver/releases/download/2.0.2/mongo-c-driver-2.0.2.tar.gz
tar zxf mongo-c-driver-2.0.2.tar.gz
cd mongo-c-driver-2.0.2/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_UNINSTALL=ON
cmake --build .
sudo cmake --install .
```

#### Install Mongo C++ Driver
```bash:
cd ~/Downloads
wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.1.1/mongo-cxx-driver-r4.1.1.tar.gz
tar zxf mongo-cxx-driver-r4.1.1.tar.gz
cd mongo-cxx-driver-r4.1.1/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local
cmake --build .
sudo cmake --install .
sudo ldconfig
```

#### Install SIGVerse Rosbridge

```bash:
cd ~/ros2_ws/src
git clone https://github.com/SIGVerse/sigverse_ros_package.git
```

## Install this ROS 2 packages and Build

```bash:
cd ~/ros2_ws/src
git clone https://github.com/RoboCupatHomeSim/ros2-competition-msgs
cd ~/ros2_ws/
colcon build --symlink-install --packages-skip sigverse_turtlebot3
source ~/ros2_ws/install/setup.bash
```

## How to Run Simple Samples

You can run the simple sample apps.  
Commands vary by task.

```bash:
ros2 launch competition_test_tools handyman_sample_launch.xml
```
```bash:
ros2 launch competition_test_tools interactive_cleanup_sample_launch.xml
```
```bash:
ros2 launch competition_test_tools human_navigation_sample_launch.xml
```

## How to Run Teleoperation Tool

You can operate the robot with keyboard operation.  
It is for debugging.  
Commands vary by task.

```bash:
ros2 launch competition_test_tools teleop_handyman_launch.xml
```
```bash:
ros2 launch competition_test_tools teleop_interactive_cleanup_launch.xml
```
```bash:
ros2 launch competition_test_tools teleop_human_navigation_launch.xml
```
