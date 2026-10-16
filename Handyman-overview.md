# Handyman Overview

## 1. 目的

这份文档用于让新的 Codex 进程或做过上下文压缩后的大模型，快速建立对当前 `Handyman` 项目的业务与代码认知。

目标不是复述全部源码，而是回答下面这些关键问题：

- 比赛官方期望机器人完成什么流程
- 这个仓库里 `Handyman` 相关代码实际实现了什么
- 当前系统由哪些 ROS 2 package、节点、launch、topic 和消息构成
- 当前实现与官方规则、官方 sample、HSR 接口之间有哪些一致点和偏差
- 后续继续开发时，应该先看哪些文件，哪些地方风险最大

---

## 2. 资料来源与可信度

本总览基于以下资料整理，按“优先级”理解：

### 2.1 官方/半官方资料

- `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
- `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`
- 在线 wiki:
  - <https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview>
- HSR 描述仓库:
  - <https://github.com/ToyotaResearchInstitute/hsr_description>

### 2.2 当前仓库里的真实实现

- `src/handyman_ros2/src/handyman_sample.cpp`
- `src/handyman_ros2/src/teleop_key_handyman.cpp`
- `src/handyman_ros2/launch/*.launch.py`
- `src/handyman_ros2/param/nav2_params.yaml`
- `src/handyman_vision_ros2/scripts/object_detection_node.py`
- `src/handyman_vision_ros2/launch/vision.launch.py`

### 2.3 需要特别注意

- 仓库内 `README` 和部分说明文档存在“旧命名/旧赛季/旧能力描述”残留。
- 以后分析行为时，应优先相信源码，其次相信 `rule_reference`，最后再看 README。

---

## 3. 项目一句话总结

当前 `Handyman` 系统是一个基于 ROS 2 Humble 的 HSR 仿真控制器：

- 通过 `sigverse_ros_bridge + rosbridge_server` 与 Unity/SIGVerse 比赛环境通信
- 通过 `handyman_msgs/msg/HandymanMsg` 与 Avatar/Moderator 交换比赛事件消息
- 通过 `Nav2 + AMCL + map_server` 在不同布局地图间切换并导航
- 通过一个 Python YOLO 节点从 HSR 手部相机上做目标检测
- 由一个 C++ 单节点状态机串起“收指令 -> 去目标房间 -> 搜物体 -> 靠近抓取 -> 去目标位置 -> 放置 -> 上报完成”

从业务上看，它已经具备一个可跑通的端到端骨架，但距离一个完整、稳健、规则覆盖充分的比赛系统还有明显差距。

---

## 4. 官方比赛视角下的 Handyman 任务

根据 `SystemOverview` 和 `RosMessage Handyman`，官方定义的流程大致如下：

1. Ubuntu 侧启动机器人控制器、SIGVerse rosbridge 等
2. Windows 侧启动 Handyman Unity app
3. 初始化机器人与目标物体位置
4. Avatar 发送 `Are_you_ready?`，同时发送 `Environment`
5. 机器人回复 `I_am_ready`
6. Avatar 发送 `Instruction`
7. 机器人前往指定房间
8. 机器人发送 `Room_reached`
9. Avatar 检查第一阶段是否成功
10. 机器人寻找目标物体
11. 找到并抓取后发送 `Object_grasped`
12. Avatar 检查抓取是否正确
13. 机器人执行抓取后的搬运/放置任务
14. 机器人发送 `Task_finished`
15. Avatar 检查最终任务是否完成
16. Avatar 发送 `Task_succeeded` / `Task_failed` / `Mission_complete`

官方还明确支持这几个分支：

- 机器人找不到物体时可发送 `Does_not_exist`
- Avatar 可能发 `Corrected_instruction`
- 机器人无法完成时可发送 `Give_up`
- 超时会收到 `Task_failed`，detail 常见为 `Time_is_up`

### 4.1 当前实现与官方流程的关系

当前代码只覆盖了“主干 happy path”的大部分步骤：

- 支持接收 `Environment`
- 支持接收 `Are_you_ready?`
- 支持接收 `Instruction`
- 支持发送 `I_am_ready`
- 支持发送 `Room_reached`
- 支持发送 `Object_grasped`
- 支持发送 `Task_finished`
- 支持接收 `Task_succeeded` / `Task_failed` / `Mission_complete`

但以下官方分支当前主控**没有真正实现**：

- `Does_not_exist`
- `Corrected_instruction`
- `Give_up`
- 与“人类作为目标放置点”相关的特殊 handover 判定逻辑

---

## 5. 官方消息接口

官方 `HandymanMsg` 很简单：

```text
string message
string detail
```

比赛事件 topic：

- `/handyman/message/to_robot`
  - 方向：SIGVerse/Avatar -> ROS
  - 消息类型：`handyman_msgs/msg/HandymanMsg`
- `/handyman/message/to_moderator`
  - 方向：ROS -> SIGVerse/Avatar
  - 消息类型：`handyman_msgs/msg/HandymanMsg`

当前仓库中消息定义位于：

- `src/ros2-competition-msgs/handyman_msgs/msg/HandymanMsg.msg`

---

## 6. HSR 机器人接口背景

`ToyotaResearchInstitute/hsr_description` 提供了 HSR 的 URDF 和 Gazebo 传感器/关节命名。当前项目依赖的关键接口与官方描述基本一致：

### 6.1 当前代码依赖的核心关节/控制接口

- `arm_lift_joint`
- `arm_flex_joint`
- `arm_roll_joint`
- `wrist_flex_joint`
- `wrist_roll_joint`
- `hand_motor_joint`

主控/teleop 实际发布：

- `/hsrb/command_velocity`
- `/hsrb/arm_trajectory_controller/command`
- `/hsrb/gripper_controller/command`

### 6.2 当前代码依赖的传感器接口

- `/hsrb/base_scan`
- `/hsrb/head_rgbd_sensor/rgb/...`
- `/hsrb/head_rgbd_sensor/depth_registered/...`
- `/hsrb/hand_camera/image_raw`

### 6.3 一个重要偏差

HSR 描述仓库里的 Gazebo 插件里，头部 RGBD 相机常见话题名是：

- `/hsrb/head_rgbd_sensor/rgb/image_rect_color`
- `/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw`

但当前视觉代码订阅的是：

- `/hsrb/head_rgbd_sensor/rgb/image_raw`
- `/hsrb/head_rgbd_sensor/depth_registered/image_raw`

这说明当前工程很可能依赖 SIGVerse/比赛环境自己的 topic 映射，而不是直接照搬 `hsr_description` 的 Gazebo 默认话题。

手部相机方面，URDF 里插件名称与当前代码更一致，都是 `hand_camera`，且图像话题为 `image_raw`。

---

## 7. 当前仓库中的 Handyman 相关包

## 7.1 `src/handyman_ros2`

这是主控包，核心内容如下：

- `src/handyman_sample.cpp`
  - 比赛主状态机
  - 负责与 Moderator 通信
  - 负责切图、启动 Nav2/AMCL/map_server
  - 负责导航、抓取、放置状态串联
- `src/teleop_key_handyman.cpp`
  - 键盘遥控/测试工具
- `launch/hsr_nav.launch.py`
  - 一键拉起主控、视觉、RViz、rosbridge、sigverse bridge
- `launch/handyman.launch.py`
  - 不带视觉和 RViz 的轻量启动
- `launch/amcl.launch.py`
  - AMCL
- `launch/move_base.launch.py`
  - Nav2 controller/planner/behavior/bt_navigator
- `maps/*.yaml`
  - 各布局静态地图
- `param/nav2_params.yaml`
  - Nav2 参数

## 7.2 `src/handyman_vision_ros2`

这是当前 Handyman 的视觉包，核心内容如下：

- `scripts/object_detection_node.py`
  - Python 检测节点
  - 使用 Ultralytics YOLO
  - 目前主要对手部相机做目标检测
- `launch/vision.launch.py`
  - 启动检测节点
- `models/`
  - 模型目录

### 7.3 命名上的注意事项

仓库根 README 中有时把视觉包写成 `vision_ros2`，但当前实际包名是：

- `handyman_vision_ros2`

分析、构建、launch 时应以实际 package 名为准。

---

## 8. 整体系统架构

可以把当前系统理解成 5 层：

### 8.1 比赛环境层

- Windows 上运行 Unity/SIGVerse Handyman app
- 里面有场景、Avatar、对象、HSR 仿真

### 8.2 桥接通信层

- `sigverse_ros_bridge`
- `rosbridge_websocket`

用途：

- 比赛事件消息
- 机器人控制命令
- 传感器数据传输

### 8.3 主控决策层

- `handyman_sample` 节点

负责：

- 比赛消息处理
- 环境切换
- 任务状态机
- 目标房间/目标位置/操作时序控制

### 8.4 导航层

- `map_server`
- `amcl`
- `Nav2`:
  - `controller_server`
  - `planner_server`
  - `behavior_server`
  - `bt_navigator`

### 8.5 感知层

- `object_detection_node`

目前负责：

- 接收目标物体名
- 从手部相机检测目标物体
- 发布 bounding box 中心与尺寸

当前“感知层”远弱于 README 里宣称的能力，真实实现更接近“近距离对准抓取辅助”。

---

## 9. 当前启动方式

最常用全栈启动是：

```bash
ros2 launch handyman_ros2 hsr_nav.launch.py
```

这个 launch 会启动：

- `rosbridge_server`
- `sigverse_ros_bridge`
- `rviz2`
- `handyman_vision_ros2/object_detection_node`
- `handyman_ros2/handyman_sample`

如果只想跑桥接 + 主控，可用：

```bash
ros2 launch handyman_ros2 handyman.launch.py
```

注意：

- `map_server`、`amcl`、`Nav2` 不在 `hsr_nav.launch.py` 里一次性常驻启动
- 当前主控会在收到环境并进入 ready 流程后，**通过 `system()` 调 shell 命令**动态启动/停止这些导航组件

这意味着当前导航栈是“按环境切换时重启”的，而不是一开始就固定常驻。

---

## 10. 当前主控节点 `handyman_sample` 的真实行为

核心文件：

- `src/handyman_ros2/src/handyman_sample.cpp`

这个文件很大，但主线很清楚：它是一个单线程主循环 + callback 驱动的状态机。

### 10.1 内部状态

主状态枚举：

- `Initialize`
- `Ready`
- `WaitForInstruction`
- `GoToRoom1`
- `GoToRoom2`
- `MoveToInFrontOfTarget`
- `MoveToInFrontOfDest`
- `Grasp`
- `Release`
- `ComeBack`
- `TaskFinished`

实际用到的主线状态主要是：

- `Initialize`
- `Ready`
- `WaitForInstruction`
- `GoToRoom1`
- `MoveToInFrontOfTarget`
- `Grasp`
- `GoToRoom2`
- `Release`
- `TaskFinished`

其中：

- `MoveToInFrontOfDest` 基本没进入主线
- `ComeBack` 是空的

### 10.2 消息回调逻辑

主控订阅：

- `/handyman/message/to_robot`
- `/vision`
- `/hand_detection`

`messageCallback` 当前处理：

- `Environment`
  - 记录环境名
- `Are_you_ready?`
  - 如果状态合适，置 `is_started_ = true`
- `Instruction`
  - 仅在 `WaitForInstruction` 下缓存文本指令
- `Task_succeeded`
  - 在 `TaskFinished` 下标记结束
- `Task_failed`
  - 置失败标记，主循环会跳回 `Initialize`
- `Mission_complete`
  - 直接退出进程

### 10.3 `Environment` 的映射

当前代码把 Unity 的简写映射到内部环境名：

- `LayoutA` -> `Layout2019HM01`
- `LayoutB` -> `Layout2019HM02`
- `LayoutC` -> `Layout2020HM01`
- `LayoutD` -> `Layout2021HM01`

之后 `LoadMapManager` 再把 `Layout2019HM01` 这种名字转为地图 YAML 路径。

---

## 11. 地图与导航机制

## 11.1 `LoadMapManager` 的职责

`handyman_sample.cpp` 前半段定义了 `LoadMapManager`，它承担了环境切换时的导航系统重建：

- 停掉旧的 `map_server`
- 停掉旧的 `amcl`
- 停掉旧的 Nav2 节点
- 启动新的 `map_server`
- 通过 lifecycle 切换 map_server 到 active
- 启动 `amcl`
- 发布 `/initialpose`
- 启动 Nav2
- 清空 costmap
- 检查 `/map` 和 TF 是否可用

### 11.2 这部分实现的特点

- 不是通过 launch API 管理，而是通过 `system("ros2 ... &")` 与 `pkill`
- 这很直接，但工程上比较脆弱
- 如果后续有并发启动、进程残留、名字冲突，这里会是高风险点

### 11.3 当前支持的地图

- `2019HM01`
- `2019HM02`
- `2020HM01`
- `2021HM01`

地图文件位于：

- `src/handyman_ros2/maps/*.yaml`

### 11.4 Nav2 参数特征

`src/handyman_ros2/param/nav2_params.yaml` 显示：

- 全局坐标系 `map`
- 底座坐标系 `base_footprint`
- 雷达 topic `/hsrb/base_scan`
- `controller_frequency: 5.0`
- local/global costmap 都把 robot footprint 和 inflation radius 明确写死
- local planner 使用 `dwb_core::DWBLocalPlanner`

整体上是一个比较保守的、低速的室内二维导航配置。

---

## 12. 指令理解方式

当前主控的 NLP 很简单，不是语义解析，而是**基于字符串切词和子串匹配**。

相关逻辑：

- `extractInfo(...)`

它会把指令按空格拆开，然后分别在这些 token 里找：

- 房间关键词：`living` / `bedroom` / `lobby` / `kitchen`
- 物体关键词：一长串对象名
- 目标位置关键词：`white_side_table` / `corner_sofa` / `dining_table` / `wooden_bed` / `Avatar` 等

### 12.1 这意味着什么

- 代码依赖英文关键词与官方指令文本高度一致
- 没有句法分析
- 没有错误恢复
- 没有歧义消解
- 没有处理 `Corrected_instruction`

所以当前系统的“语言理解”能力非常有限，本质上仍是 rule-based keyword spotting。

---

## 13. 房间巡航与目标位置逻辑

当前主控没有实时建图定位目标位置，而是使用**手工硬编码的房间巡航点与放置点**。

### 13.1 `roomLocation(room, variation)`

对每个环境和房间，都写死了一组巡航点：

- living
- bedroom
- lobby
- kitchen

不同环境有不同数量的 patrol point，例如：

- `Layout2019HM01` 多数房间最多 2 个点
- `Layout2020HM01` 的 living 最多 4 个点

机器人在 `GoToRoom1` 状态中会：

- 先判断自己是否已经在目标房间附近
- 否则用 Nav2 前往当前 patrol 点
- 到点后若还没发现目标，就切到下一个 patrol 点

### 13.2 `destLocation(dest, room)`

放置目标同样按环境写死坐标。

例如会根据环境和 room 决定：

- `white_side_table`
- `corner_sofa`
- `armchair`
- `trash_box_for_recycle`
- `trash_box_for_bottle_can`
- `dining_table`
- `wooden_bed`
- `wagon`
- `cardboard_box`

### 13.3 当前实现的实际含义

当前系统对“环境理解”的核心不是在线感知，而是：

- 先根据官方 `Environment` 选择预制地图
- 再根据指令中的 room/dest 走硬编码 waypoint

这是一种典型的比赛工程做法，优点是简单可控，缺点是对地图与场景假设非常强。

---

## 14. 当前视觉模块的真实能力

核心文件：

- `src/handyman_vision_ros2/scripts/object_detection_node.py`

### 14.1 视觉节点订阅

- `/hsrb/head_rgbd_sensor/rgb/image_raw`
- `/hsrb/hand_camera/image_raw`
- `/hsrb/head_rgbd_sensor/depth_registered/image_raw`
- `/detection_target`

### 14.2 视觉节点发布

- `/detection_depth`
- `/vision`
- `/hand_detection`

### 14.3 真实执行逻辑

虽然它订阅了头部 RGB 和深度图，也准备了 `/vision` 和 `/detection_depth` publisher，但当前主逻辑几乎只做一件事：

- 从 `/detection_target` 接收目标类别名
- 在 `hand_image` 上跑 YOLO
- 只取目标类别，`max_det=1`
- 发布 `Int32MultiArray` 到 `/hand_detection`
  - 内容是检测框 `xywh`

### 14.4 当前没有真正做成的部分

- 没有把头部 RGB 检测串到主流程
- 没有使用深度图估距离
- 没有稳定地产生可用于抓取位姿的 `/vision` 目标位姿
- `handyman_sample` 虽然订阅了 `/vision` 并缓存 `object_pose`，但主流程实际并不依赖它

### 14.5 这部分的真实定位

当前视觉模块更像：

- “基于手部相机的近距离目标对准辅助”

而不是：

- “完整的房间级搜索、检测、3D 定位感知系统”

---

## 15. 抓取与放置逻辑

当前代码的抓取不是 MoveIt 级别的精细操作，更像规则化时序控制。

### 15.1 抓取前对准

在 `MoveToInFrontOfTarget` 状态：

- 如果最近 1 秒内收到 `/hand_detection`
  - 根据框中心 `x_det/y_det` 调底盘和手臂高度
  - 目标是让框中心接近图像参考点
- 若框宽足够大
  - 认为“ready_to_grasp”
- 对准后：
  - 不够近就微微前进
  - 足够近则进入 `Grasp`

### 15.2 抓取动作

`Grasp` 状态本质是：

- 发送 gripper close
- 等 8 秒
- 发送 `Object_grasped`
- 然后转入去目标位置导航

这里没有：

- 力反馈验证
- 夹爪闭合状态验证
- 抓取失败判定
- “抓错物体”自主检查

### 15.3 放置动作

`Release` 状态本质是：

- 到目标点后把手臂伸到一个固定姿态
- 等 4 秒
- 张开 gripper
- 再等 8 秒
- 发送 `Task_finished`

同样没有：

- 放置是否真的成功的感知闭环
- 如果目标是 Avatar 时的专门 handover 控制

---

## 16. 当前 topic / interface 总表

下面这张表对新进程最有用。

| 类别 | Topic | 方向 | 当前用途 |
| --- | --- | --- | --- |
| 比赛消息 | `/handyman/message/to_robot` | Avatar -> 主控 | 接收 `Are_you_ready?` / `Environment` / `Instruction` / `Task_*` |
| 比赛消息 | `/handyman/message/to_moderator` | 主控 -> Avatar | 回传 `I_am_ready` / `Room_reached` / `Object_grasped` / `Task_finished` |
| 目标检测目标 | `/detection_target` | 主控 -> 视觉 | 告诉 YOLO 当前只关注哪个类别 |
| 手部检测结果 | `/hand_detection` | 视觉 -> 主控 | 发布 bbox `xywh`，供近距离对准 |
| 目标位姿 | `/vision` | 视觉 -> 主控 | 当前订阅了，但主流程基本没实际用起来 |
| 深度 | `/detection_depth` | 视觉 -> 其他 | 当前主流程未依赖 |
| 底盘控制 | `/hsrb/command_velocity` | 主控 -> HSR | 底盘速度控制 |
| 手臂轨迹 | `/hsrb/arm_trajectory_controller/command` | 主控 -> HSR | 手臂姿态控制 |
| 夹爪控制 | `/hsrb/gripper_controller/command` | 主控 -> HSR | 开合夹爪 |
| 激光 | `/hsrb/base_scan` | HSR -> Nav2/AMCL | 导航定位 |
| 地图 | `/map` | map_server -> 全局 | 静态地图 |
| 导航动作 | `navigate_to_pose` | 主控 <-> Nav2 | 前往房间和放置点 |
| 初始位姿 | `/initialpose` | 主控 -> AMCL | 切图后重定位 |

---

## 17. 与官方 sample 的关系

仓库里其实已经包含官方 `ros2-competition-msgs/competition_test_tools` 的 `handyman_sample.cpp`。

当前自建 `handyman_ros2/src/handyman_sample.cpp` 可以看成是官方 sample 的扩展版，主要新增了：

- 环境切图与地图管理
- Nav2 导航
- 房间 patrol 点
- 与视觉节点联动
- 简单抓取与放置流程

但也保留了 sample 的一些风格：

- 大状态机单文件实现
- 事件消息主导流程
- 较少抽象层次

这意味着如果后续要重构，官方 sample 是一个很好的“协议最小基线”，而当前版本是“可比赛的工程化分支”。

---

## 18. 当前实现与文档/README 的主要不一致

这一节很重要，后续新模型最容易被这些地方误导。

### 18.1 ROS1/旧项目表述残留

`src/handyman_ros2/README.md` 中仍有 ROS1 / `catkin_make` / 旧包名表述，不应作为当前实现事实来源。

### 18.2 视觉能力描述偏乐观

README 里描述成：

- 头相机、手相机、深度相机全面协同
- 物体定位较完整

但真实代码目前主要用的是：

- 手相机目标检测 + bbox 对准

### 18.3 包名不一致

部分文档写的是：

- `vision_ros2`

实际当前路径是：

- `src/handyman_vision_ros2`

### 18.4 topic 命名可能依赖 SIGVerse 环境而非标准 Gazebo 默认值

尤其是头部 RGBD 的话题名，当前代码和 `hsr_description` 的 Gazebo 默认值并不完全一致。

---

## 19. 当前实现的明显短板

后续继续开发时，优先级最高的问题基本都在这里。

### 19.1 规则覆盖不完整

未真正处理：

- `Does_not_exist`
- `Corrected_instruction`
- `Give_up`
- Avatar handover 的特殊要求

### 19.2 感知链不完整

- 主流程缺少房间级目标搜索感知闭环
- `/vision` 的 3D 位姿没有真正支撑抓取
- 深度信息几乎没用上

### 19.3 抓取和放置没有成功验证

- 没有确认是否抓住
- 没有确认抓的是不是对的
- 没有确认是否放置成功

### 19.4 工程结构偏脆弱

- `handyman_sample.cpp` 单文件过大
- 通过 `system()` + `pkill` 管理导航进程
- 状态和副作用耦合重

### 19.5 语言理解过于脆弱

- 靠关键词匹配
- 不适合复杂自然语言

### 19.6 场景知识全靠硬编码坐标

- 一旦环境或对象布局有偏移，鲁棒性会迅速下降

---

## 20. 以后继续开发时，建议的阅读顺序

如果新的 Codex 进程需要快速进入状态，推荐按这个顺序看：

1. `handyman-overview.md`
2. `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
3. `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`
4. `src/handyman_ros2/src/handyman_sample.cpp`
5. `src/handyman_vision_ros2/scripts/object_detection_node.py`
6. `src/handyman_ros2/launch/hsr_nav.launch.py`
7. `src/handyman_ros2/launch/amcl.launch.py`
8. `src/handyman_ros2/launch/move_base.launch.py`
9. `src/handyman_ros2/param/nav2_params.yaml`

如果要查官方消息/基线 sample，再看：

- `src/ros2-competition-msgs/handyman_msgs/msg/HandymanMsg.msg`
- `src/ros2-competition-msgs/competition_test_tools/src/handyman_sample.cpp`

---

## 21. 后续模型最该记住的结论

给未来新进程的压缩版要点如下：

- 当前 Handyman 是“官方消息协议 + 自定义导航切图 + 手相机 YOLO 对准 + 规则式抓放”的系统。
- 主控核心只有一个：`src/handyman_ros2/src/handyman_sample.cpp`。
- 真正的业务主线是：
  - 等 `Environment`
  - 等 `Are_you_ready?`
  - 切地图并起 Nav2/AMCL
  - 发 `I_am_ready`
  - 收 `Instruction`
  - 关键词提取 room/object/dest
  - 发 `/detection_target`
  - 去房间 patrol
  - 用 `/hand_detection` 做近距离对准
  - 抓取
  - 去目标位置
  - 放置
  - 发 `Task_finished`
- 当前最重要的缺口不是“再调一点参数”，而是：
  - 规则分支没补齐
  - 感知闭环没补齐
  - 抓放成功判定没补齐
  - 代码结构需要拆分

---

## 22. 对后续工程任务的直接建议

如果后续要继续做比赛准备，优先方向建议是：

1. 先补规则闭环
   - `Does_not_exist`
   - `Corrected_instruction`
   - `Give_up`
2. 再补感知闭环
   - 明确房间级搜索策略
   - 明确 `/vision` 是否真的输出可用 3D 位姿
3. 再补抓放验证
   - 抓取成功判定
   - 目标正确性判定
   - 放置成功判定
4. 最后做结构化重构
   - 拆分状态机、地图管理、任务解析、操作控制、视觉接口

如果只是为了短期比赛可跑，短期价值最高的通常是：

- 修 topic / launch / 模型路径 / 环境切换问题
- 提高检测稳定性
- 提高抓取前对准稳定性
- 处理常见 `Task_failed` 分支

---

## 23. 关键文件清单

- `src/handyman_ros2/src/handyman_sample.cpp`
- `src/handyman_ros2/src/teleop_key_handyman.cpp`
- `src/handyman_ros2/launch/hsr_nav.launch.py`
- `src/handyman_ros2/launch/handyman.launch.py`
- `src/handyman_ros2/launch/amcl.launch.py`
- `src/handyman_ros2/launch/move_base.launch.py`
- `src/handyman_ros2/param/nav2_params.yaml`
- `src/handyman_ros2/maps/2019HM01.yaml`
- `src/handyman_ros2/maps/2019HM02.yaml`
- `src/handyman_ros2/maps/2020HM01.yaml`
- `src/handyman_ros2/maps/2021HM01.yaml`
- `src/handyman_vision_ros2/scripts/object_detection_node.py`
- `src/handyman_vision_ros2/launch/vision.launch.py`
- `src/ros2-competition-msgs/handyman_msgs/msg/HandymanMsg.msg`
- `rule_reference/handyman/SystemOverview · RoboCupatHomeSimhandyman-unity2 Wiki.md`
- `rule_reference/handyman/RosMessage Handyman · RoboCupatHomeSimros2-competition-msgs Wiki.md`

