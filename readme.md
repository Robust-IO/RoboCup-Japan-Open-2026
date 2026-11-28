# RoboCup@Home Simulation 2026 Japan Open

本仓库为 **RoboCup@Home Simulation** 竞赛的 ROS 2 工作空间，基于 [SIGVerse](http://www.sigverse.org/) 仿真平台（Unity + ROS 2 Humble），使用 Toyota HSR（Human Support Robot）完成三项家庭服务任务。

## 竞赛任务概览


| 任务                      | 说明                                          |
| ----------------------- | ------------------------------------------- |
| **Handyman**            | 机器人根据 Avatar 的自然语言指令，前往指定房间抓取目标物体并搬运回来      |
| **Human Navigation**    | 机器人通过生成自然语言引导指令，指导佩戴 VR 头显的测试者完成物体搬运        |
| **Interactive Cleanup** | 机器人根据 Avatar 的肢体指向，识别需要清理的物体和目标位置，自主完成抓取与放置 |


## 目录结构

以下为仓库中**受版本控制**的目录与文件，不包含 `.gitignore` 中忽略的 `build/`、`install/`、`log/`、`.pixi/`、`*.pt`、`*.task` 等。

```
RoboCup-Japan-Open-2026/
├── readme.md
├── .gitignore
├── pixi.toml                    # pixi 环境配置（Python 依赖，见下方「使用 pixi 配置环境」）
├── pixi.lock                    # 依赖锁定
├── rule_reference/              # 竞赛规则参考文档
│   ├── handyman/
│   ├── human_navigation/
│   └── interactive_cleanup/
│
└── src/                         # ROS 2 工作空间源码
    ├── handyman_ros2/           # Handyman 控制器（主控 + 键盘遥控）
    ├── vision_ros2/             # Handyman 视觉感知（YOLOv12 物体检测）
    ├── human_nav_ros2/          # Human Navigation 控制器
    ├── interactive_cleanup_ros2/# Interactive Cleanup 控制器
    ├── ros2-competition-msgs/    # 竞赛消息定义与测试工具（官方）
    │   ├── handyman_msgs/
    │   ├── human_navigation_msgs/
    │   ├── interactive_cleanup_msgs/
    │   └── competition_test_tools/
    └── sigverse_ros_package/     # SIGVerse ROS Bridge 与示例（官方）
```

## 各 Package 说明

### 自建 Package


| Package                    | 说明                                                    |
| -------------------------- | ----------------------------------------------------- |
| `handyman_ros2`            | Handyman 任务控制器（主控状态机、键盘遥控），依赖 `vision_ros2` 做物体检测     |
| `vision_ros2`              | Handyman 视觉模块（YOLOv12，ultralytics），Python 依赖统一由 `pixi` 管理 |
| `human_nav_ros2`           | Human Navigation 任务控制器                                |
| `interactive_cleanup_ros2` | Interactive Cleanup 任务控制器，依赖 `cleanup_vision_ros2` 的视觉结果 |


### 官方 Package


| Package                 | 说明                                                                                               |
| ----------------------- | ------------------------------------------------------------------------------------------------ |
| `ros2-competition-msgs` | 竞赛消息类型（`handyman_msgs`、`human_navigation_msgs`、`interactive_cleanup_msgs`）及 sample / teleop 测试节点 |
| `sigverse_ros_package`  | SIGVerse ROS Bridge（Unity ↔ ROS 2）及 HSR、TIAGo、TurtleBot3 等示例                                     |


## 系统架构

```
┌──────────────────────┐          ┌──────────────────────────────────┐
│   Windows (Unity)    │          │         Ubuntu (ROS 2)           │
│                      │  TCP/IP  │                                  │
│  SIGVerse 仿真环境    │◄────────►│  rosbridge + sigverse_ros_bridge │
│  (HSR + 场景 + Avatar)│          │  + 机器人控制器节点               │
└──────────────────────┘          └──────────────────────────────────┘
```

- **Windows 端**：运行 Unity 仿真程序，渲染场景、物理引擎、Avatar 交互  
- **Ubuntu 端**：运行 ROS 2 节点（rosbridge、SIGVerse ROS Bridge、任务控制器）

## HSR 机器人 ROS 话题


| 类别     | 话题                                                  | 方向             |
| ------ | --------------------------------------------------- | -------------- |
| 底盘速度   | `/hsrb/command_velocity`                            | ROS → SIGVerse |
| 头部控制   | `/hsrb/head_trajectory_controller/command`          | ROS → SIGVerse |
| 手臂控制   | `/hsrb/arm_trajectory_controller/command`           | ROS → SIGVerse |
| 夹爪控制   | `/hsrb/gripper_controller/command`                  | ROS → SIGVerse |
| 关节状态   | `/hsrb/joint_states`                                | SIGVerse → ROS |
| 激光雷达   | `/hsrb/base_scan`                                   | SIGVerse → ROS |
| RGB 相机 | `/hsrb/head_rgbd_sensor/rgb/image_raw`              | SIGVerse → ROS |
| 深度相机   | `/hsrb/head_rgbd_sensor/depth_registered/image_raw` | SIGVerse → ROS |
| 手部相机   | `/hsrb/hand_camera/image_raw`                       | SIGVerse → ROS |


## 环境搭建

### 1. 系统依赖（apt）

- ROS 2 Humble  
- rosbridge、slam-toolbox、Nav2、MoveIt 等  
- Mongo C/C++ Driver（SIGVerse ROS Bridge 依赖）

```bash
sudo rosdep init
rosdep update
sudo apt install -y git libncurses-dev
sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite
sudo apt install -y ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-octomap
sudo apt install -y ros-$ROS_DISTRO-hardware-interface
sudo apt install -y ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-controller-manager
sudo apt install -y ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-ros-perception ros-$ROS_DISTRO-moveit-ros-occupancy-map-monitor
```

**Mongo C/C++ 驱动安装**（SIGVerse ROS Bridge 依赖，主目录为 `~/下载`）：

```bash
cd ~/下载
wget https://github.com/mongodb/mongo-c-driver/releases/download/2.0.2/mongo-c-driver-2.0.2.tar.gz
tar zxf mongo-c-driver-2.0.2.tar.gz
cd mongo-c-driver-2.0.2/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DENABLE_UNINSTALL=ON
cmake --build .
sudo cmake --install .

cd ~/下载
wget https://github.com/mongodb/mongo-cxx-driver/releases/download/r4.1.1/mongo-cxx-driver-r4.1.1.tar.gz
tar zxf mongo-cxx-driver-r4.1.1.tar.gz
cd mongo-cxx-driver-r4.1.1/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_PREFIX_PATH=/usr/local
cmake --build .
sudo cmake --install .
sudo ldconfig
```

### 2. Python 依赖（vision_ros2 + cleanup_vision_ros2 + rosbridge）

仓库中的 Python 依赖统一由 `pixi` 管理，避免与系统 ROS 2 Python 冲突。`handyman_ros2`、`cleanup_vision_ros2` 与 `rosbridge_server` 使用的第三方包都已在根目录 `pixi.toml` 中声明。

[pixi](https://pixi.sh/) 在项目根目录管理 Python 环境，使用 Python 3.10 且 `numpy<2`，与 ROS Humble 的 `cv_bridge`、`rclpy` 兼容。`pixi.toml` 中已配置：

- **vision_ros2**：`ultralytics`、`opencv-python`、`numpy`、`pillow`、`transforms3d`
- **cleanup_vision_ros2**：`mediapipe`（Tasks API）
- **rosbridge_server**：`tornado`、`pymongo`（bson）、`cbor2`

```bash
# 安装 pixi
curl -fsSL https://pixi.sh/install.sh | bash

# 在仓库根目录安装依赖
cd /path/to/RoboCup-Japan-Open-2026
pixi install
```

运行 `hsr_nav.launch.py` 时，**先** source ROS2 与工作空间，**再**进入 pixi 环境并执行 launch：

```bash
cd /path/to/RoboCup-Japan-Open-2026
source /opt/ros/humble/setup.bash   # 或 setup.zsh
source install/setup.bash
pixi shell
ros2 launch handyman_ros2 hsr_nav.launch.py
```

完整六轮 Handyman 比赛包使用独立调度入口（需先生成运行包），始终从 session1 开始：

```bash
cd build/handyman-competition
./start_dynamic.sh N:=2
# 固定模式：./start_fixed.sh N:=2
```

`N` 默认 6；前 N 轮正常执行，后续轮次收到首次 `Instruction` 后等待 5 秒发送 `Give_up`。
`N:=0` 表示全部放弃，范围 0～4294967295。入口也接受 `--N 2`，不再接受起始轮次位置参数。
详见 `competition/handyman/launch/使用说明.md`。

`pixi` 只管理 Python 包本身，不会下载比赛所需的运行时模型资产。请手动准备以下文件：

- `src/vision_ros2/models/last.pt` 或 `src/vision_ros2/models/yolo12n.pt`
- `src/cleanup_vision_ros2/models/pose_landmarker.task`

### 3. 编译工作空间

```bash
cd /path/to/RoboCup-Japan-Open-2026
colcon build --symlink-install --packages-skip sigverse_turtlebot3
source install/setup.bash
```

可选：将上述两条 `source` 写入 `~/.bashrc`，避免每次手动执行。

## 快速启动

**竞赛官方 sample / teleop（competition_test_tools）：**

```bash
ros2 launch competition_test_tools handyman_sample_launch.xml
```
```bash
ros2 launch competition_test_tools interactive_cleanup_sample_launch.xml
```
```bash
ros2 launch competition_test_tools human_navigation_sample_launch.xml
```

键盘控制：

```bash
ros2 launch competition_test_tools teleop_handyman_launch.xml
```
```bash
ros2 launch competition_test_tools teleop_interactive_cleanup_launch.xml
```
```bash
ros2 launch competition_test_tools teleop_human_navigation_launch.xml
```

**自建 Handyman 全栈（主控 + 视觉 + RViz2 + rosbridge）：**

```bash
# 使用 pixi 时：先 source /opt/ros/humble/setup.bash、source install/setup.bash，再 pixi shell，然后：
ros2 launch handyman_ros2 hsr_nav.launch.py
```

**自建 Interactive cleanup 全栈 **

```bash
ros2 launch interactive_cleanup cleanup.launch.py use_nav2:=true use_rviz:=true
```

可以通过 `N` 设置完整执行的 session 数量（无符号 32 位整数，范围 0～4294967295）。例如先完整执行两轮，从第三轮开始正常处理指令，收到 `Clean_up!` 后发送 `Give_up`：

```bash
ros2 launch interactive_cleanup cleanup.launch.py N:=2 use_nav2:=false use_rviz:=false
```

不传 `N` 时默认为 `4294967295`。`N:=0` 表示从第一轮开始在收到 `Clean_up!` 后放弃。计数从进程启动后的首个任务开始，每次重启进程重新计数。启动前需加载 ROS 和本工作区环境，并启用本仓库的 pixi Python 环境。

更多 launch 与键盘遥控说明见 `src/handyman_ros2/README.md`。

## 参考链接

- [SIGVerse 官方文档](http://www.sigverse.org/)
- [ros2-competition-msgs](https://github.com/RoboCupatHomeSim/ros2-competition-msgs)
- [sigverse_ros_package](https://github.com/SIGVerse/sigverse_ros_package)
- [RoboCup@Home Simulation Wiki](https://github.com/RoboCupatHomeSim)

---

# 检测代码步骤

使用网线将两台电脑进行连接：一台 Ubuntu 电脑跑 ROS2 代码，一台 Windows 电脑跑 Unity 和 Quest3 的 Horizon Link。



## Windows 电脑操作

1. 首先关闭两台设备防火墙。
2. 在 Windows 的 **运行**（Win+R）中输入 `ncpa.cpl`，选择 **以太网** 右键 → **属性** → **IPv4 属性** 里面，手动设置：
   - IP：`192.168.0.1`
   - 网关：`255.255.255.0`
3. 设置好后执行 `ping 192.168.0.2`，看是否丢包，两端都设置好再ping。
4. 设置 Quest3 与电脑 Horizon Link 连接（Windows 开启虚拟网卡，打开 App，与 Quest 进行有线连接）。
5. 打开初始界面右下角 Quest 设置，或者点击左下框 WiFi 也可打开设置，在设置的左边栏与wifi同级的找到 **Link**，设置 Quest 与电脑 Link，点击后会进入另一个待机界面，有human navigation和interactive clean up两个unity项目可选，但是还是往下进行不要直接点。
6. 在文件管理器打开想要进行 task 的unity包的文件夹进入，找到 **build** 文件夹，里面有项目的 exe 文件（找到 task 名字的 exe，等 ROS2 端启动代码后再点 exe）。----这个build文件夹是初始配置unity时就进行过的步骤
7. 让 Quest 进行交互（Quest 只有 **Human Navigation** 和 **Interactive Clean Up** 需要 Quest）。



## Ubuntu 电脑操作

1. 关闭防火墙。
2. 右上角连接找到 **有线网**，对有线网设置，手动设置 IPv4：
   - IP：`192.168.0.2`
   - 默认网关：`192.168.0.2`
3. 执行 `ping 192.168.0.2`，看是否丢包。
4. 不丢包后，在控制台 / Cursor 里启动控制台，`cd` 到仓库（或从仓库文件夹右键点“在终端中打开”）。
5. 之前没有编译过的，问大模型自己的task怎么编译。编译后，通过 `source install/setup.bash` 启动 ROS2。
6. 对特定 task 的 launch 进行启动：
   - **Handyman**：输入 `ros2 launch handyman_ros2 hsr_nav.launch.py` 启动，看到 `initialized` 或 `setup` 即可。
   - **Human Navigation**：输入 `ros2 launch human_nav_ros2 sample_launch.py` 启动。



## Debug

如果ubuntu运行ros2时出现 **address already in use**，可以：
- 关闭当前终端、关闭 Cursor ，打开后从source再来一遍，或
- 重启电脑。


### Human Navigation 提示开关

两个入口 `sample_launch.py` 和 `sample_with_rviz_launch.py` 均支持布尔参数 `prompt`，默认 `true`：

```bash
ros2 launch human_nav_ros2 sample_launch.py prompt:=false
# 启用提示（也可省略 prompt 参数）
ros2 launch human_nav_ros2 sample_launch.py prompt:=true
```

`prompt:=false` 时不生成导航提示、不向 `/human_navigation/message/guidance_message` 发布任何提示，
包括主动请求、错误纠正和任务完成提示，也不轮询语音播报状态。任务握手、状态更新和机器人控制流程保留。
该开关在启动时读取。
