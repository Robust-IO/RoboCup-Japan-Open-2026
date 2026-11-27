# Handyman ROS 项目

## 项目概述

本项目是为 **RoboCup@Home Simulation Malaysia 2025 Handyman 比赛** 开发的 ROS1 代码包。该项目实现了一个家庭服务机器人，能够执行物体抓取和放置任务，支持多种家庭环境布局。

## 项目结构

```
handyman-ros/
├── CMakeLists.txt              # 构建配置文件
├── package.xml                 # 包依赖信息
├── launch/                     # 启动文件目录
│   ├── hsr_nav.launch         # 主要启动文件
│   ├── amcl.launch            # AMCL定位启动文件
│   ├── move_base.launch       # 导航启动文件
│   ├── make_map.launch        # 建图启动文件
│   └── *.launch               # 其他功能启动文件
├── maps/                       # 地图文件目录
│   ├── 2019HM01.pgm/.yaml     # 2019年HM01布局地图
│   ├── 2019HM02.pgm/.yaml     # 2019年HM02布局地图
│   ├── 2020HM01.pgm/.yaml     # 2020年HM01布局地图
│   └── 2021HM01.pgm/.yaml     # 2021年HM01布局地图
├── msg/                        # 自定义消息类型
│   └── HandymanMsg.msg        # Handyman消息定义
├── param/                      # 参数文件目录
│   ├── base_local_planner_params.yaml
│   ├── costmap_common_params_hsr.yaml
│   ├── dwa_local_planner_params.yaml
│   ├── global_costmap_params.yaml
│   ├── local_costmap_params.yaml
│   └── move_base_params.yaml
├── src/                        # 源代码目录
│   ├── handyman_sample.cpp     # 主要控制逻辑
│   └── teleop_key_handyman.cpp # 键盘遥控节点
├── include/                    # 头文件目录
└── license/                    # 许可证文件
```

## 主要功能

### 1. 环境支持
- 支持4种不同的家庭环境布局：
  - Layout2019HM01
  - Layout2019HM02
  - Layout2020HM01
  - Layout2021HM01
- 每个环境包含多个房间：客厅(living)、卧室(bedroom)、大厅(lobby)、厨房(kitchen)

### 2. 物体识别
- 支持识别30+种日常物品：
  - 水果类：苹果(apple)
  - 玩具类：玩具企鹅(toy_penguin)、兔子娃娃(rabbit_doll)、熊娃娃(bear_doll)等
  - 日用品：罐装果汁(canned_juice)、糖(sugar)、酱油(soysauce)等
  - 厨具：白杯(white_cup)、粉红杯(pink_cup)、番茄酱(ketchup)等

### 3. 目标位置
- 支持15+种放置位置：
  - 家具类：白边桌(white_side_table)、转角沙发(corner_sofa)、圆桌(round_low_table)等
  - 床类：木床(wooden_bed)、铁床(iron_bed)
  - 垃圾分类箱：可回收、可燃、瓶罐类
  - 其他：手推车(wagon)、纸箱(cardboard_box)

## 使用方式

### 1. 环境准备
```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆项目
git clone https://gitee.com/robocup-malaysia-2025-nankai/handyman-ros.git

# 克隆依赖的rcup_vision项目
git clone https://gitee.com/robocup-malaysia-2025-nankai/rcup_vision.git

# 编译
cd ~/catkin_ws
catkin_make
```

### 2. 配置转接服务器
#### 安装 Rosbridge Server 请参考
<http://wiki.ros.org/rosbridge_suite>

#### 安装 SIGVerse Rosbridge Server 请参考
<https://github.com/SIGVerse/ros_package/tree/master/sigverse_ros_bridge>

### 3. 启动系统
```bash
# 加载环境变量
source ~/catkin_ws/devel/setup.bash

# 启动主系统
roslaunch handyman hsr_nav.launch
```

### 4. 系统组件
主启动文件 `hsr_nav.launch` 会启动以下组件：
- **SIGVerse ROS Bridge**：与仿真环境通信
- **ROS Bridge WebSocket**：提供Web接口
- **RCUP Vision**：物体识别系统
- **RViz**：可视化界面
- **Handyman Sample**：主控制节点

### 5. 工作流程
1. **初始化**：系统启动并等待环境信息
2. **环境切换**：根据接收到的环境信息加载对应地图
3. **任务接收**：等待比赛指令
4. **指令解析**：解析指令中的房间、物体、目标位置
5. **任务执行**：
   - 导航到指定房间
   - 巡逻搜索目标物体
   - 视觉识别和定位
   - 机械臂抓取
   - 导航到目标位置
   - 放置物体
6. **任务完成**：发送完成信号

## 系统架构

### 1. 消息通信
- **HandymanMsg**：自定义消息类型，包含message和detail字段
- **主要话题**：
  - `/handyman/message/to_robot`：接收来自裁判的指令
  - `/handyman/message/to_moderator`：发送状态给裁判
  - `/vision`：接收视觉识别结果
  - `/hand_detection`：接收手部检测信息
  - `/detection_target`：发送目标物体给视觉系统

### 2. 状态机
系统采用状态机设计，主要状态包括：
- **Initialize**：初始化状态
- **Ready**：准备就绪
- **WaitForInstruction**：等待指令
- **GoToRoom1**：前往第一个房间
- **GoToRoom2**：前往第二个房间
- **MoveToInFrontOfTarget**：移动到目标物体前
- **Grasp**：抓取物体
- **Release**：释放物体
- **TaskFinished**：任务完成

### 3. 导航系统
- 使用ROS标准导航栈
- 支持动态地图切换
- 集成AMCL定位
- 配置DWA局部规划器

## 依赖项

### 必需依赖
- **ROS1 Noetic**（或其他兼容版本）
- **标准ROS包**：
  - roscpp
  - rospy
  - std_msgs
  - move_base_msgs
  - geometry_msgs
  - trajectory_msgs
  - tf
  - nav_msgs
  - actionlib

### 非标准依赖
- **rcup_vision**：视觉识别包
  - 用于物体检测和识别
  - 提供目标物体位置信息
- **sigverse_ros_bridge**：与仿真环境通信
- **rosbridge_server**：WebSocket支持

## 已知问题与不足

### 1. 视觉识别限制
- **近距离识别不稳定**：在物体距离较近时，识别准确率显著下降
- **角度敏感**：仰拍和俯拍情况下识别性能不佳
- **遮挡问题**：物体部分遮挡时识别困难

### 2. 指令处理简单
- **有限的语义理解**：只能处理简单的关键词匹配
- **缺乏上下文理解**：无法理解复杂的指令逻辑
- **固定格式依赖**：指令需要包含特定的关键词

### 3. 导航与操作
- **固定路径规划**：缺乏动态避障和路径优化
- **机械臂控制简单**：抓取和放置动作较为粗糙
- **精度问题**：定位和操作精度有待提高
- **环境适应性**：对环境变化的适应能力有限

### 4. 系统稳定性
- **错误处理不足**：部分异常情况处理不完善
- **错误恢复能力弱**：遇到异常情况时恢复能力有限


## 许可证

本项目采用 SIGVerse 许可证，详见 `license/` 目录。

## 联系方式

- 维护者：双角斧 来自ISI Lab
- 邮箱：horned_axe@proton.me

---

**注意**：此项目专为 RoboCup@Home Simulation Malaysia 2025 比赛设计，可能需要根据具体比赛规则进行调整。