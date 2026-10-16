# Interactive Cleanup 总览

## 1. 文档目的

这份文档是给后续开发者、后续 Codex 会话、以及可能的新进程做交接用的总览文档，目标不是替代源码，而是让接手者能在较短时间内同时弄清楚下面几件事：

- 比赛本身要求机器人完成什么；
- 官方系统架构、消息流和 HSR 接口是什么；
- 当前仓库里 `interactive_cleanup` 与 `cleanup_vision_ros2` 的真实实现结构是什么；
- 重构方案中哪些设计已经落地，哪些还只是方向；
- 当前系统已经推进到哪里，主要风险和瓶颈又在哪里；
- 如果对文档内容有怀疑，应当回到哪些本地文件或官方资料核实。

维护原则：

- 只要 `src/interactive_cleanup_ros2`、`src/cleanup_vision_ros2`、launch 文件、消息定义、状态机流程、目的地数据库、或相关设计结论发生变化，这份文档就应当一起更新；
- 文档与代码冲突时，以代码为准，然后修正文档；
- 文档中凡是带有“推断”的内容，表示它不是官方文字原样结论，而是根据当前源码或资料归纳得到。

本次整理主要依据：

- 本地规则镜像：
  - `rule_reference/interactive_cleanup/RoboCupatHomeSiminteractive-cleanup-unity2：交互式清理 Unity 项目 --- RoboCupatHomeSiminteractive-cleanup-unity2 Interactive Cleanup Unity project.md`
  - `rule_reference/interactive_cleanup/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for interactive clean up.md`
  - `rule_reference/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for competitions.md`
- 官方外部资料：
  - [https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview)
  - [https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR)
  - [https://github.com/ToyotaResearchInstitute/hsr_description](https://github.com/ToyotaResearchInstitute/hsr_description)
- 本地设计方案：
  - `Interactive cleanup重构方案.md`
- 本地实现源码：
  - `src/interactive_cleanup_ros2/`
  - `src/cleanup_vision_ros2/`

## 2. 比赛任务与系统概览

### 2.1 任务目标

Interactive Cleanup 的核心任务是：

- 机器人根据 Avatar 的指向与比赛消息，理解“要抓什么物体”；
- 根据 Avatar 的指向与比赛消息，理解“要把物体放到哪里或丢到哪里”；
- 完成抓取；
- 把物体放到 Avatar 指向的桌面，或者丢进 Avatar 指向的垃圾桶。

官方成功条件可以概括为：

- 如果待清理物体最终被放到了 Avatar 指向的桌面上，或者被丢进了 Avatar 指向的垃圾箱里，则该次任务成功。

信息来源：

- `rule_reference/interactive_cleanup/RoboCupatHomeSiminteractive-cleanup-unity2：交互式清理 Unity 项目 --- RoboCupatHomeSiminteractive-cleanup-unity2 Interactive Cleanup Unity project.md`

### 2.2 官方比赛流程

官方流程压缩后如下：

1. Ubuntu 侧启动机器人控制器、rosbridge、SIGVerse bridge 以及相关 ROS 节点。
2. Windows 侧启动 Unity 的 Interactive Cleanup 程序。
3. 初始化机器人和目标物体的姿态。
4. Avatar 向机器人发送 `Are_you_ready?`。
5. 机器人回复 `I_am_ready`。
6. Avatar 指向待抓物体并发送 `Pick_it_up!`。
7. Avatar 指向清理目的地并发送 `Clean_up!`。
8. 机器人可以选择发送 `Point_it_again` 要求重指。
9. 机器人抓取物体，必要时还可以发送 `Is_this_correct?`。
10. 机器人发送 `Object_grasped`。
11. 机器人将物体移动到目的地并释放。
12. 机器人发送 `Task_finished`。
13. Avatar 根据结果返回 `Task_succeeded`、`Task_failed` 或 `Mission_complete`。

对当前实现很重要的规则含义：

- `Point_it_again` 是官方允许的流程，不是自定义扩展；
- `Is_this_correct?` 是官方允许的可选确认步骤，但会扣分；
- `Give_up` 也是官方允许的失败退出方式。

信息来源：

- `rule_reference/interactive_cleanup/RoboCupatHomeSiminteractive-cleanup-unity2：交互式清理 Unity 项目 --- RoboCupatHomeSiminteractive-cleanup-unity2 Interactive Cleanup Unity project.md`
- `rule_reference/interactive_cleanup/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for interactive clean up.md`

### 2.3 官方系统拓扑

官方系统是双机结构：

- Windows 电脑运行 Unity 写成的 Interactive Cleanup 仿真程序；
- Ubuntu 电脑运行 ROS 2 控制侧，包括机器人控制器、rosbridge server、SIGVerse rosbridge server。

通信上分两层：

- 较轻量的控制和事件消息走 rosbridge；
- 大量传感器数据走 SIGVerse rosbridge server。

机器人动作接口层面：

- ROS 侧通过 `Twist`、`JointTrajectory` 等消息驱动 Unity 中的 HSR；
- Unity 会周期性回传 `JointState`、TF 以及各类传感器数据。

信息来源：

- `rule_reference/interactive_cleanup/RoboCupatHomeSiminteractive-cleanup-unity2：交互式清理 Unity 项目 --- RoboCupatHomeSiminteractive-cleanup-unity2 Interactive Cleanup Unity project.md`
- [https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview](https://github.com/RoboCupatHomeSim/handyman-unity2/wiki/SystemOverview)

## 3. 与本任务直接相关的官方 ROS 接口

### 3.1 比赛消息 Topic

比赛事件消息使用 `interactive_cleanup_msgs/msg/InteractiveCleanupMsg`：

```text
string message
string detail
```

当前任务的消息 Topic：

- `/interactive_cleanup/message/to_robot`
  - 方向：Avatar 或 SIGVerse 侧发给机器人控制器
- `/interactive_cleanup/message/to_moderator`
  - 方向：机器人控制器发回 Avatar 或 SIGVerse 侧

当前任务里最关键的消息值：

- `Are_you_ready?`
- `I_am_ready`
- `Pick_it_up!`
- `Clean_up!`
- `Point_it_again`
- `Is_this_correct?`
- `Yes`
- `No`
- `Object_grasped`
- `Task_finished`
- `Task_succeeded`
- `Task_failed`
- `Mission_complete`
- `Give_up`

信息来源：

- `rule_reference/interactive_cleanup/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for interactive clean up.md`

### 3.2 官方 HSR ROS Topic

本仓库实际用到、且官方文档明确给出的 HSR 接口主要有：

控制侧：

- `/hsrb/command_velocity`
  - 类型：`geometry_msgs/Twist`
  - 用于底盘运动
- `/hsrb/head_trajectory_controller/command`
  - 类型：`trajectory_msgs/JointTrajectory`
  - 用于头部云台关节
- `/hsrb/arm_trajectory_controller/command`
  - 类型：`trajectory_msgs/JointTrajectory`
  - 用于机械臂与腕部关节
- `/hsrb/gripper_controller/command`
  - 类型：`trajectory_msgs/JointTrajectory`
  - 用于夹爪

状态与传感器：

- `/hsrb/joint_states`
- `/hsrb/base_scan`
- `/hsrb/head_rgbd_sensor/rgb/image_raw`
- `/hsrb/head_rgbd_sensor/rgb/camera_info`
- `/hsrb/head_rgbd_sensor/depth_registered/image_raw`
- `/hsrb/head_rgbd_sensor/depth_registered/camera_info`
- `/hsrb/head_center_camera/image_raw`
- `/hsrb/head_center_camera/camera_info`
- `/hsrb/hand_camera/image_raw`
- `/hsrb/hand_camera/camera_info`

官方还特别说明：

- 机器人 TF 会被周期性发布。

信息来源：

- `rule_reference/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for competitions.md`
- [https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR](https://github.com/RoboCupatHomeSim/ros2-competition-msgs/wiki/RosMessage-HSR)

## 4. HSR 结构信息与本仓库的建模方式

### 4.1 官方结构资料来源

官方竞赛消息页面直接给出的 HSR 结构描述资料来源是：

- [https://github.com/ToyotaResearchInstitute/hsr_description](https://github.com/ToyotaResearchInstitute/hsr_description)

当前仓库并没有直接 vendor 这套包，而是自己在 `interactive_cleanup` 里维护了一套轻量化的局部几何模型，用于预抓取规划和手爪相机位置估计。

信息来源：

- `rule_reference/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for competitions.md`
- [https://github.com/ToyotaResearchInstitute/hsr_description](https://github.com/ToyotaResearchInstitute/hsr_description)

### 4.2 当前任务中最关键的关节

官方消息表和本地控制器里都直接相关的关节包括：

机械臂与腕部：

- `arm_lift_joint`
- `arm_flex_joint`
- `arm_roll_joint`
- `wrist_flex_joint`
- `wrist_roll_joint`

头部：

- `head_pan_joint`
- `head_tilt_joint`

夹爪：

- `hand_l_proximal_joint`
- `hand_r_proximal_joint`

此外，本地控制器还会读取：

- `hand_motor_joint`
  - 用于抓取验证，判断夹爪是否“没能完全闭死”，从而推断夹爪里可能夹住了物体。

信息来源：

- `rule_reference/RoboCupatHomeSimros2-competition-msgs ROS 2 message packages for competitions.md`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

### 4.3 当前本地几何模型

`src/interactive_cleanup_ros2/src/hsr_geometry.cpp` 实现了一套简化的 HSR 几何模型，主要包括：

- 对 `arm_lift`、`arm_flex`、`arm_roll`、`wrist_flex`、`wrist_roll` 做 joint limit clamp；
- 基于固定几何参数估计：
  - shoulder 位置；
  - palm 位置；
  - hand camera 位置；
- `estimateHandPitch()` 采用 `-arm_flex - wrist_flex`；
- `handCameraOffsetFromWristRoll()` 返回 `(-0.027, 0.0, 0.136)`。

这里必须明确一个关键判断：

- 推断：这不是完整的 URDF 级正运动学；
- 它只是当前预抓取规划使用的一套轻量近似模型；
- 因此，如果 RViz 中看到末端腕部、夹爪、或者最后一节链条姿态不对，问题可能出在：
  - 简化几何假设本身；
  - `pregrasp_planner` 选出的姿态模板；
  - 控制器给机械臂下发轨迹的方式；
  - 或者本地简化模型与仿真里完整 HSR 模型不一致。

本地测试目前把这些 joint limit 和 hand camera offset 视作与官方 HSR 描述相匹配的实现目标。

信息来源：

- `src/interactive_cleanup_ros2/src/hsr_geometry.cpp`
- `src/interactive_cleanup_ros2/include/interactive_cleanup/hsr_geometry.hpp`
- `src/interactive_cleanup_ros2/test/hsr_geometry_test.cpp`
- [https://github.com/ToyotaResearchInstitute/hsr_description](https://github.com/ToyotaResearchInstitute/hsr_description)

## 5. 仓库中与 Interactive Cleanup 最相关的目录

最需要优先知道的目录和文件：

- `Interactive cleanup重构方案.md`
  - 当前整套重构的设计动机、阶段目标和验收标准
- `rule_reference/interactive_cleanup/`
  - 本地镜像的比赛规则与消息说明
- `src/interactive_cleanup_ros2/`
  - C++ 控制器主体
- `src/cleanup_vision_ros2/`
  - Python 感知层与自定义消息
- `pixi.toml`
  - Python 运行时依赖

两个 package 的职责边界：

- `interactive_cleanup`
  - 状态机、比赛消息、导航、目标物解析、目的地解析、预抓取规划、近场伺服、抓取验证
- `cleanup_vision_ros2`
  - 头部 RGB-D 感知、指向估计、手爪相机感知、调试可视化、自定义消息

信息来源：

- `readme.md`
- `src/interactive_cleanup_ros2/package.xml`
- `src/cleanup_vision_ros2/package.xml`
- `pixi.toml`

## 6. 当前 `interactive_cleanup` 包的架构

### 6.1 编译组成

`interactive_cleanup_sample` 由下面这些源文件一起构成：

- `interactive_cleanup_sample.cpp`
- `navigation_utils.cpp`
- `grasp_utils.cpp`
- `avatar_tracker.cpp`
- `observation_head_sweep.cpp`
- `pick_target_resolver.cpp`
- `pointing_alignment.cpp`
- `place_destination_resolver.cpp`
- `hsr_geometry.cpp`
- `pregrasp_planner.cpp`
- `hand_servo.cpp`

这说明当前实现已经不是“所有逻辑都堆进一个 sample.cpp”的旧模式，核心计算逻辑已经被拆到了多个独立模块里。

信息来源：

- `src/interactive_cleanup_ros2/CMakeLists.txt`

### 6.2 Launch 拓扑

`src/interactive_cleanup_ros2/launch/cleanup.launch.py` 当前会启动：

- `interactive_cleanup_sample`
- `head_perception_node`
- `hand_perception_node`
- `sigverse_ros_bridge`
- `rosbridge_websocket`
- 可选 Nav2 定位与导航栈
- 可选 RViz
- 若干 TF 相关占位节点

比较关键的运行时细节：

- `placeholder_tf_publisher.py` 会在桥接真正接管之前，低频发布 `odom -> base_footprint`；
- 一旦收到 `/hsrb/base_scan`，说明 Unity bridge 已经开始工作，它就会停止发布，避免 TF 冲突；
- 这样 Nav2 可以更早初始化。

另有一个只起控制器、不带视觉的 launch：

- `src/interactive_cleanup_ros2/launch/sample.launch.py`

信息来源：

- `src/interactive_cleanup_ros2/launch/cleanup.launch.py`
- `src/interactive_cleanup_ros2/launch/sample.launch.py`
- `src/interactive_cleanup_ros2/scripts/placeholder_tf_publisher.py`
- `src/interactive_cleanup_ros2/test/cleanup_launch_test.py`
- `src/interactive_cleanup_ros2/test/placeholder_tf_shutdown_test.py`

### 6.3 控制器的主要 ROS 接口

`interactive_cleanup_sample` 当前创建的 publisher：

- `/interactive_cleanup/message/to_moderator`
- `/interactive_cleanup/message/to_human`
- `/hsrb/message/to_human`
- `/hsrb/command_velocity`
- `/hsrb/arm_trajectory_controller/command`
- `/hsrb/gripper_controller/command`
- `/hsrb/head_trajectory_controller/command`
- `/cleanup_perception/mode`

subscriber：

- `/interactive_cleanup/message/to_robot`
- `/hsrb/message/to_robot`
- `/hsrb/joint_states`
- `/cleanup_perception/head/avatar`
- `/cleanup_perception/head/objects`
- `/cleanup_perception/head/pointing`
- `/cleanup_perception/hand/target_alignment`

导航侧 action client：

- `navigate_to_pose`

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

### 6.4 当前状态机

当前状态机包含如下状态：

1. `Initialize`
2. `Ready`
3. `WaitForPickTrackAvatar`
4. `ResolvePickTarget`
5. `WaitForCleanTrackAvatar`
6. `ResolvePlaceDestination`
7. `PlanPregrasp`
8. `NavigateToPregrasp`
9. `DeployArmForApproach`
10. `HandCameraApproach`
11. `CloseAndVerifyGrasp`
12. `SendObjectGrasped`
13. `MoveToDestination`
14. `ReleaseObject`
15. `SendTaskFinished`
16. `WaitForResult`

按阶段理解：

- 等待阶段：
  - 一边持续跟踪 Avatar，一边缓存 cue window；
- 解析阶段：
  - 先解析 pick cue，再解析 place cue；
- 抓取阶段：
  - 规划预抓取
  - 导航到预抓取位
  - 部署机械臂
  - 手爪相机近场伺服
  - 闭合夹爪并验证；
- 任务完成阶段：
  - 发送 `Object_grasped`
  - 导航去目的地
  - 释放物体
  - 发送 `Task_finished`
  - 等待比赛结果。

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/test/interactive_cleanup_behavior_test.py`

## 7. 当前 Pick / Place 解析逻辑

### 7.1 Cue 锁存机制

在 `WaitForPickTrackAvatar` 和 `WaitForCleanTrackAvatar` 两个状态中，控制器会持续维护最近若干秒的观测窗口：

- 来自 `/cleanup_perception/head/objects` 的 recent object frames；
- 来自 `/cleanup_perception/head/pointing` 的 recent pointing frames。

当比赛消息到达时：

- 收到 `Pick_it_up!` 就锁存 `latched_pick_objects`_ 与 `latched_pick_pointings_`；
- 收到 `Clean_up!` 就锁存 `latched_place_objects_` 与 `latched_place_pointings_`。

当前锁存窗口长度：

- `CUE_CAPTURE_WINDOW_SEC = 3.0`

这和旧逻辑相比是一个关键架构升级：

- 旧思路是“消息到了才开始临时观察”；
- 现在是“消息到达时锁存最近已观测到的 cue”。

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/test/interactive_cleanup_behavior_test.py`
- `Interactive cleanup重构方案.md`

### 7.2 `ResolvePickTarget`：当前三段式工作法

这里记录的是当前代码里的真实行为，不是旧版本，也不是只看设计文档得到的理想流程。

第一段：直接用锁存 cue 解码目标物

- 输入：
  - `latched_pick_objects_`
  - `latched_pick_pointings_`
- 处理：
  - 从 object frames 里过滤掉 `person`；
  - 生成 `PickCandidate`；
  - 用 `resolvePickTarget()` 评分选目标。

这里要明确两个事实：

- 这一步不是“纯 pointing”；
- 它依然需要锁存窗口里存在可用的非 `person` 物体检测；
- 如果锁存窗口里只有 pointing，没有可用的 object candidate，这一步就会失败。

第二段：基于 pointing 的底盘对齐

- 如果第一段失败，状态机会进入主动重观察；
- 对齐阶段的 seed 不是空白开始，而是直接使用已经锁存下来的 `latched_pick_pointings_`；
- `evaluatePointingAlignment()` 会聚合有效 pointing，控制底盘旋转，使机器人朝向与 pointing 方向对齐。

这一步也要明确：

- 它不是“只靠 fresh 视觉、完全丢掉锁存 pointing”；
- 它明确复用了之前锁存的 pointing；
- 也就是说，现在的实现已经不再是“第二阶段把前面锁存信息丢掉重来”。

第三段：受限 head micro-sweep 加合并重解码

- 对齐完成后，会执行一个受限的五阶段 head micro-sweep；
- 这个 sweep 负责采集新的 object 观测；
- 最终目标解析时使用：
  - 新采集到的 `obs_objects_`
  - `collectPickResolutionPointings()` 合并后的 pointing 集合，其中包括：
    - `latched_pick_pointings_`
    - 对齐开始后收集到的 `obs_pointings_`

因此，当前 `ResolvePickTarget` 的真实结构是：

- 第一段：锁存 object + 锁存 pointing
- 第二段：锁存 pointing + fresh pointing 做底盘对齐
- 第三段：fresh objects + 锁存与 fresh pointing 融合做最终解析

结论：

- 当前实现已经不是单一模态；
- 它是一个分阶段的融合流程，而不是“第一段只用一种信息、第二段只用另一种信息”的简单切换。

但仍然存在一个根本限制：

- pick 目标最终仍然离不开视觉检测；
- pointing 只能帮助筛选和对齐，不能在完全没有 object detection 的情况下凭空确定抓取目标类别和 3D 位置。

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/src/pick_target_resolver.cpp`
- `src/interactive_cleanup_ros2/src/pointing_alignment.cpp`
- `src/interactive_cleanup_ros2/src/observation_head_sweep.cpp`
- `src/interactive_cleanup_ros2/test/pick_target_resolver_test.cpp`
- `src/interactive_cleanup_ros2/test/pointing_alignment_test.cpp`
- `src/interactive_cleanup_ros2/test/observation_head_sweep_test.cpp`
- `src/interactive_cleanup_ros2/test/interactive_cleanup_behavior_test.py`

### 7.3 `pick_target_resolver` 当前评分项

当前评分主要依赖：

- detection confidence；
- 候选物体到 pointing 3D 射线的距离；
- 候选 bbox 中心到图像 pointing 射线的 2D 距离；
- pointing confidence 的过滤与加权。

几个实现细节：

- pointing 置信度低于 `0.35` 时会被忽略；
- 当前不是等权投票，而是带权评分；
- 抓取模式只按目标高度分：
  - `< 0.08` -> `floor_pick`
  - `< 0.22` -> `table_low_pick`
  - 其他 -> `table_mid_pick`

信息来源：

- `src/interactive_cleanup_ros2/src/pick_target_resolver.cpp`
- `src/interactive_cleanup_ros2/test/pick_target_resolver_test.cpp`
- `Interactive cleanup重构方案.md`

### 7.4 `ResolvePlaceDestination`

当前 place 解析是几何驱动的，不依赖对家具或容器做视觉检测。

输入：

- `latched_place_pointings_`
- destination region 数据库
- `target_class_` 作为弱先验

当前算法核心：

- 聚合有效 pointing 的 origin 和平面方向；
- 求平均 yaw 与角稳定性；
- 对每个 `DestinationRegion` 按以下项打分：
  - 射线对齐程度
  - 是否落在区域内
  - sector 兼容度
  - 距离合理性
  - pointing 稳定性
  - 基于目标类别的弱 task prior

这与重构方案中的方向一致：

- 目的地解析不依赖家具检测；
- 而是依赖 pointing 几何 + 目的地区域数据库。

但这里也有一个实现层面的注意点：

- 头部感知节点虽然有 `RESOLVE_DEST` 模式；
- 但当前控制器并没有在 place 解析中真正使用 head object detection；
- 当前 place 解析实质上就是“pointing + destination database”。

信息来源：

- `src/interactive_cleanup_ros2/src/place_destination_resolver.cpp`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/test/place_destination_resolver_test.cpp`
- `Interactive cleanup重构方案.md`

## 8. 目的地数据库：设计意图与当前实现的差异

### 8.1 当前 YAML 文件里实际存了什么

当前目的地配置文件是：

- `src/interactive_cleanup_ros2/config/destination_regions.yaml`

当前这个文件实际上只写了：

- name
- center `(x, y)`

它并没有显式写出下面这些 richer metadata：

- `type`
- `allowed_radius`
- `sector_center_yaw`
- `sector_half_width`
- `preferred_facing_yaw`
- `placement_mode`

信息来源：

- `src/interactive_cleanup_ros2/config/destination_regions.yaml`

### 8.2 运行时是如何补全这些信息的

`loadDestinationRegions()` 读取 YAML 后，会通过 `inferDestinationRegion()` 按名称补全默认值。

当前默认推断规则：

- type 推断：
  - `trash_box_*` -> `trash_box`
  - `*table*` -> `table`
  - `wagon*` -> `wagon`
  - `*shelf*` -> `shelf`
  - `*box*` -> `box`
- 默认半径：
  - `trash_box` -> `0.35`
  - `table` -> `0.50`
  - `wagon` -> `0.55`
  - `shelf` -> `0.45`
  - `box` -> `0.40`
- 默认放置模式：
  - `trash_box` -> `drop`
  - `box` -> `drop`
  - `shelf` -> `shelf_place`
  - 其他 -> `surface_place`
- 默认扇区：
  - 基本等于“全开扇区”，约束很弱

这里存在一个重要的“设计文档 vs 当前实现”偏差：

- 设计文档希望 destination region 最终拥有更细粒度、可调的参数；
- 设计文档对 `cardboard_box` 的语义更偏向表面放置；
- 但当前实现里，因为 YAML 没有覆写，`cardboard_box` 会被当成 `box -> drop`。

这是一个真实的架构分歧点，后续如果比赛策略依赖精细目的地语义，这部分应优先回头修。

信息来源：

- `src/interactive_cleanup_ros2/src/place_destination_resolver.cpp`
- `src/interactive_cleanup_ros2/config/destination_regions.yaml`
- `Interactive cleanup重构方案.md`

## 9. 预抓取规划、机械臂部署与近场接近

### 9.1 当前 `pregrasp_planner`

`planPregrasp()` 的输入：

- 目标在 `base_footprint` 下的位置
- 抓取模式

输出：

- arm pose
- navigation stand-off
- approach stop distance
- desired hand camera height
- approach profile 名称

当前支持的抓取模式 profile：

- `floor_pick`
- `table_low_pick`
- `table_mid_pick`

当前规划方法可以概括为：

- 先根据目标高度估一个期望的 hand camera 高度；
- 在 `arm_lift` 和 `arm_flex` 上做网格搜索；
- 再根据期望 hand pitch 推出 `wrist_flex`；
- 用 `hsr_geometry.cpp` 的简化几何模型计算代价；
- 选代价最低的姿态；
- 如果目标横向偏得更厉害，还会略微增大 nav standoff。

这里必须明确：

- 推断：这仍然是 workspace heuristic，不是完整 IK，也不是带碰撞约束的 motion planning。

信息来源：

- `src/interactive_cleanup_ros2/src/pregrasp_planner.cpp`
- `src/interactive_cleanup_ros2/src/hsr_geometry.cpp`
- `src/interactive_cleanup_ros2/test/pregrasp_planner_test.cpp`
- `Interactive cleanup重构方案.md`

### 9.2 规划后的执行流程

当前控制器在抓取侧的顺序是：

1. `PlanPregrasp`
  - 把 odom 下目标转到 `base_footprint`
  - 调 `planPregrasp()`
2. `NavigateToPregrasp`
  - 由 stand-off 反推出 map 下导航目标
  - 发送 Nav2 goal
3. `DeployArmForApproach`
  - 切换 perception mode 到 `HAND_APPROACH`
  - 下发预抓取 arm trajectory
4. `HandCameraApproach`
  - 使用手爪相机结果做近场伺服
5. `CloseAndVerifyGrasp`
  - 闭合夹爪并验证抓取

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

### 9.3 当前手爪相机近场伺服

`HandCameraApproach` 当前依赖：

- `/cleanup_perception/hand/target_alignment` 上的 `latest_hand_alignment_`
- 本地 `computeHandServoCommand()` 重新计算的伺服命令

当前控制器在这个阶段会同时输出：

- 底盘前后运动
- 底盘横向微调
- arm lift 的小幅修正

一个很容易被忽略但实际很重要的实现细节：

- `HandTargetAlignment.msg` 虽然本身就带有：
  - `recommended_linear_x`
  - `recommended_linear_y`
  - `recommended_lift_delta`
- 但当前控制器并没有直接使用这些 recommended 值；
- 它只把 hand node 输出当作观测，再在控制器内部重新算一遍伺服命令。

重试逻辑：

- 如果目标没找到，或对准超时，会先轻微后退，再返回 `PlanPregrasp`；
- 重试次数是有限的。

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/src/hand_servo.cpp`
- `src/cleanup_vision_ros2/msg/HandTargetAlignment.msg`
- `src/interactive_cleanup_ros2/test/hand_servo_test.cpp`
- `Interactive cleanup重构方案.md`

### 9.4 当前最突出的未解问题区域

截至当前开发会话，问题重心已经明显从 cue 解析前半段移动到了预抓取和近场操作阶段。

已经取得明显进展的部分：

- Avatar 跟踪明显更稳；
- cue 锁存生效；
- 底盘对齐 + bounded re-observation 的 pick 重解析链路已经接回主流程；
- pick target 解析日志已经能在不少情况下选到正确物体。

当前新的瓶颈：

- operator 反馈在预抓取阶段，夹爪末端最后一节在 RViz 里没有保持平行地面；
- 视觉上更像是“往地里直插”，说明末端姿态很可能不对；
- 这强烈暗示问题可能出在：
  - `pregrasp_planner` 选出的姿态；
  - `hsr_geometry` 的简化假设；
  - 机械臂轨迹部署方式；
  - 或者本地简化模型与真实仿真模型的差异。

这部分信息来自当前会话中 operator 给出的实跑日志与现象描述，不是仓库里某个已提交文件里的内容。

信息来源：

- 当前会话中的 operator 运行日志与描述
- `src/interactive_cleanup_ros2/src/pregrasp_planner.cpp`
- `src/interactive_cleanup_ros2/src/hsr_geometry.cpp`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

## 10. 当前抓取验证逻辑

当前抓取验证融合了几个证据：

- `hand_motor_joint` 反馈，判断夹爪是否没有完全闭死；
- pickup 附近是否还能在头部视觉里看到目标；
- 手爪相机是否还能看到与目标类一致、且面积足够的物体。

相关 helper 逻辑：

- `gripperLikelyHoldingObject()`
  - 如果夹爪没有完全闭死，则认为“可能抓到东西了”
- `isGraspVerificationSuccessful()`
  - 当前成功条件是以下三者之一成立：
    - gripper 像是夹到了东西
    - hand camera 确认到了抓取后目标
    - pickup 附近已经看不到目标了

这和设计文档里设想的“三层证据融合”相比，还属于较轻的启发式实现，不算最终形态。

信息来源：

- `src/interactive_cleanup_ros2/src/grasp_utils.cpp`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/test/grasp_utils_test.cpp`
- `Interactive cleanup重构方案.md`

## 11. 当前 `cleanup_vision_ros2` 包的架构

### 11.1 这个包负责什么

它目前负责：

- 头部相机 Avatar 观测
- 头部相机物体检测和 3D 定位
- Avatar pointing 估计
- 手爪相机近场目标检测
- 手爪相机对准消息输出
- 调试图像和 marker
- Interactive Cleanup 用到的自定义消息

信息来源：

- `src/cleanup_vision_ros2/package.xml`
- `src/cleanup_vision_ros2/CMakeLists.txt`

### 11.2 当前自定义消息

关键消息类型：

- `AvatarObservation.msg`
  - Avatar bbox
  - `center_error_x`
  - `origin_odom`
  - `body_yaw`
  - confidence
- `PointingDirection.msg`
  - `odom` 下的 3D origin 和 direction
  - 图像中的 wrist pixel 与 projected point
  - confidence
  - `is_valid`
- `SceneObject.msg`
  - class、confidence、bbox
  - 若可得则包含 3D position
  - depth
- `SceneObjectArray.msg`
  - objects 数组
  - `source_camera`
- `HandTargetAlignment.msg`
  - 是否找到目标
  - 目标类别
  - 归一化后的像素误差
  - bbox 面积占比
  - 是否进入 grasp window
  - 推荐运动
  - confidence

信息来源：

- `src/cleanup_vision_ros2/msg/AvatarObservation.msg`
- `src/cleanup_vision_ros2/msg/PointingDirection.msg`
- `src/cleanup_vision_ros2/msg/SceneObject.msg`
- `src/cleanup_vision_ros2/msg/SceneObjectArray.msg`
- `src/cleanup_vision_ros2/msg/HandTargetAlignment.msg`

### 11.3 头部感知节点

主文件：

- `src/cleanup_vision_ros2/scripts/head_perception_node.py`

订阅：

- 头部 RGB 图像
- 头部 depth 图像
- head camera info
- `/cleanup_perception/mode`

发布：

- `/cleanup_perception/head/avatar`
- `/cleanup_perception/head/objects`
- `/cleanup_perception/head/pointing`
- `/cleanup_perception/debug/head_image`
- `/cleanup_perception/debug/head_markers`

当前 active modes：

- `TRACK_AVATAR`
- `RESOLVE_PICK`
- `RESOLVE_DEST`

当前算法概括：

- 用 YOLO 在 head RGB 上检测 `person` 和任务物体；
- 用 depth + TF 将物体中心抬到 `odom`；
- 用 MediaPipe PoseLandmarker 或 Pose fallback 估计人体姿态和 pointing；
- pointing 的稳健性处理包括：
  - 先对 person 区域做 crop，再跑 pose；
  - 用 arm visibility 和 straightness 做约束；
  - 使用左右手 sticky selection；
  - depth 或 wrist-depth endpoint 的选择逻辑；
  - 时间平滑与低置信度跳变拒绝。

几个输出语义上的要点：

- head object detection 可以带 `odom` 下的 3D 坐标；
- pointing 方向也发布在 `odom` 下；
- 即使完整 pointing 不可用，`AvatarObservation` 仍可能有效，这时仍能用于 Avatar 居中跟踪。

信息来源：

- `src/cleanup_vision_ros2/scripts/head_perception_node.py`
- `src/cleanup_vision_ros2/scripts/perception_common.py`
- `src/cleanup_vision_ros2/scripts/pointing_utils.py`
- `src/cleanup_vision_ros2/test/head_pointing_geometry_test.py`
- `src/cleanup_vision_ros2/test/perception_common_test.py`

### 11.4 手爪感知节点

主文件：

- `src/cleanup_vision_ros2/scripts/hand_perception_node.py`

订阅：

- `/hsrb/hand_camera/image_raw`
- `/hsrb/hand_camera/camera_info`
- `/cleanup_perception/mode`

发布：

- `/cleanup_perception/hand/objects`
- `/cleanup_perception/hand/target_alignment`
- `/cleanup_perception/debug/hand_image`
- `/cleanup_perception/debug/hand_markers`

active modes：

- `HAND_APPROACH`
- `HAND_VERIFY`

当前算法概括：

- 在 hand camera RGB 上跑 YOLO；
- 忽略 `person`；
- 从所有非 `person` 检测框中按以下项选“当前最优框”：
  - detection confidence
  - bbox area ratio
  - center bias
- 输出像素误差、bbox 面积占比、是否进入 grasp window 等近场控制信息。

两个很重要的限制：

- 当前没有 hand camera depth；
- 所以 near-field approach 本质上还是 2D 图像空间控制。

另一个对后续更重要的限制：

- hand node 只选“当前最优的非 person 检测框”；
- 控制器再检查这个框的 `target_class` 是否等于 `target_class_`；
- 这意味着当后续换成更完整、更强的多类模型后，场景里一旦有别的物体更大、更居中、置信度更高，就可能导致：
  - hand node 持续给出 fresh 结果；
  - 但控制器认为这些结果“usable = false”；
  - 于是系统表现成“明明有检测，但一直找不到目标”。

信息来源：

- `src/cleanup_vision_ros2/scripts/hand_perception_node.py`
- `src/cleanup_vision_ros2/msg/HandTargetAlignment.msg`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

### 11.5 旧的单节点实现

包里仍保留：

- `src/cleanup_vision_ros2/scripts/cleanup_detection_node.py`

这个节点使用的是旧话题：

- `/cleanup_vision/detected_objects`
- `/cleanup_vision/pointing_direction`
- `/cleanup_vision/enable`

它已经不是当前重构后主路径的运行节点，更多属于过渡期遗留实现。

信息来源：

- `src/cleanup_vision_ros2/scripts/cleanup_detection_node.py`
- `Interactive cleanup重构方案.md`

## 12. 当前 Topic 与运行时数据流

当前端到端数据流可以概括为：

1. Unity Avatar 与模拟器发出比赛消息和传感器数据。
2. `interactive_cleanup_sample` 从 `/interactive_cleanup/message/to_robot` 接收比赛流程消息。
3. `interactive_cleanup_sample` 通过 `/cleanup_perception/mode` 控制当前感知模式。
4. `head_perception_node` 输出：
  - Avatar 居中观测
  - pointing
  - 3D scene objects
5. 等待状态把这些结果积累到 recent cue window。
6. `ResolvePickTarget` 与 `ResolvePlaceDestination` 消耗这些锁存 cue。
7. `PlanPregrasp` 把目标点转成局部 arm pose 与 nav stand-off。
8. Nav2 把底盘移动到预抓取位。
9. `hand_perception_node` 输出图像空间近场对准信息。
10. `HandCameraApproach` 一边调底盘，一边调 arm lift。
11. `CloseAndVerifyGrasp` 负责闭爪与抓取验证。
12. 之后控制器再通过比赛消息和导航，把物体送到目标区域。

信息来源：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/cleanup_vision_ros2/scripts/head_perception_node.py`
- `src/cleanup_vision_ros2/scripts/hand_perception_node.py`
- `src/interactive_cleanup_ros2/launch/cleanup.launch.py`

## 13. 重构方案的落地状态：哪些已经完成，哪些还是半成品

### 13.1 已经落地或大体落地的部分

`Interactive cleanup重构方案.md` 里的下列设计，当前已经较明显地反映在代码中：

- 头部节点与手爪节点分离
- `/cleanup_perception/*` 新话题体系
- 等待阶段 Avatar 跟踪
- pick/place cue 锁存
- `pick_target_resolver`
- `place_destination_resolver`
- bounded head micro-sweep 替代旧的大范围 head scan
- confidence-aware pointing 聚合
- destination region 的加载与默认推断
- `hsr_geometry`
- `pregrasp_planner`
- `hand_servo`
- `Object_grasped` 前的抓取验证 gate

### 13.2 已有实现，但仍然比较简化的部分

这些部分虽然已经存在，但离“最终理想版”还有明显距离：

- `hsr_geometry`
  - 是简化几何，不是完整 URDF FK
- `pregrasp_planner`
  - 是 profile + heuristic search，不是 IK / motion planner
- hand-camera final approach
  - 仍是 2D 伺服，没有深度
- grasp verification
  - 仍是较轻量的启发式，而不是设计文档里的更完整证据融合
- destination region database
  - YAML 里只存 center，很多 richer metadata 仍靠运行时推断

### 13.3 当前实现与设计方案的关键偏差

需要特别记住的偏差：

- 设计文档希望目的地区域拥有更丰富的可调参数，但当前 YAML 还没有真正写进去；
- 设计文档希望有更强的近场重试与闭环，但当前手爪相机链路还比较简化；
- 头部节点虽然有 `RESOLVE_DEST`，但当前控制器 place 解析并不使用 destination object vision；
- `cardboard_box` 目前若不在 YAML 中显式覆写，会沿用 `box -> drop` 的默认规则。

信息来源：

- `Interactive cleanup重构方案.md`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/src/place_destination_resolver.cpp`
- `src/interactive_cleanup_ros2/config/destination_regions.yaml`

## 14. 当前已经存在的测试保护网

控制器侧测试：

- `avatar_tracker_test.cpp`
- `navigation_utils_test.cpp`
- `grasp_utils_test.cpp`
- `pick_target_resolver_test.cpp`
- `place_destination_resolver_test.cpp`
- `pointing_alignment_test.cpp`
- `observation_head_sweep_test.cpp`
- `hsr_geometry_test.cpp`
- `pregrasp_planner_test.cpp`
- `hand_servo_test.cpp`
- `cleanup_launch_test.py`
- `placeholder_tf_shutdown_test.py`
- `interactive_cleanup_behavior_test.py`

视觉侧测试：

- `perception_common_test.py`
- `perception_visualization_test.py`
- `head_pointing_geometry_test.py`

其中最值得记住的行为测试断言包括：

- 控制器已经迁移到 `/cleanup_perception/*` 话题；
- 旧的大范围 head scan 流程已经被移除；
- 等待阶段确实在做 Avatar 跟踪；
- pick 解析会在 base alignment 之后执行 bounded head micro-sweep；
- active pick resolution 会合并 latched pointing 和 fresh pointing；
- pick/place cue 锁存是正式流程的一部分；
- grasp path 中已经引入 pregrasp planner 和 hand servo；
- `Object_grasped` 只有在验证成功后才会发送。

信息来源：

- `src/interactive_cleanup_ros2/test/`
- `src/cleanup_vision_ros2/test/`
- `src/interactive_cleanup_ros2/CMakeLists.txt`
- `src/cleanup_vision_ros2/CMakeLists.txt`

## 15. 构建、运行与测试命令

环境层面：

- Python 依赖通过 `pixi` 管理；
- 运行时模型不会自动下载；
- 当前视觉链路至少期望存在如下模型资产：
  - `src/cleanup_vision_ros2/models/cleanup_model.pt`
  - `src/cleanup_vision_ros2/models/pose_landmarker.task`

编译：

```bash
source /opt/ros/humble/setup.bash
cd /home/navi/RoboCup-Japan-Open-2026
colcon build --symlink-install --packages-skip turtlebot
source install/setup.bash
```

Python 环境：

```bash
cd /home/navi/RoboCup-Japan-Open-2026
pixi install
pixi shell
```

启动完整 Interactive Cleanup 栈：

```bash
source /opt/ros/humble/setup.bash
cd /home/navi/RoboCup-Japan-Open-2026
source install/setup.bash
pixi shell
ros2 launch interactive_cleanup cleanup.launch.py use_nav2:=true use_rviz:=true
```

当前工程约定的推荐启动方式不是直接在终端里裸跑 `ros2 launch`，而是通过仓库根目录下的 `run_and_log.sh` 前台运行并同步记录日志到 `log.txt`。

推荐命令：

```bash
cd /home/navi/RoboCup-Japan-Open-2026
./run_and_log.sh "pixi run ros2 launch interactive_cleanup cleanup.launch.py use_nav2:=true use_rviz:=true"
```

`run_and_log.sh` 的当前行为：

- 接收一整条命令字符串作为唯一参数；
- 在前台运行该命令；
- 通过 `tee` 把标准输出和标准错误同时追加到当前目录下的 `log.txt`；
- 在 `log.txt` 中记录执行命令和退出码。

因此，后续分析 Interactive Cleanup 的运行日志时，默认应优先查看：

- `log.txt`

而不是依赖一次性终端输出回忆现场信息。

关于效果验证还有一个重要约束：

- 这个任务依赖 Windows 侧 Unity 程序与 Ubuntu 侧 ROS 栈联动；
- 因此智能体不应把“仅在本地直接执行 `ros2 launch`”当作完整效果验证；
- 在没有 Windows Unity 端、Avatar 交互和 SIGVerse 联动的情况下，本地启动最多只能用于：
  - 静态检查 launch 是否能起；
  - 查看节点是否报立即性错误；
  - 读取日志结构；
- 不能据此断言指向识别、抓取、放置、导航闭环或整体任务行为已经正确。

只启动控制器：

```bash
source /opt/ros/humble/setup.bash
cd /home/navi/RoboCup-Japan-Open-2026
source install/setup.bash
ros2 launch interactive_cleanup sample.launch.py
```

常用测试命令：

```bash
source /opt/ros/humble/setup.bash
cd /home/navi/RoboCup-Japan-Open-2026
source install/setup.bash
colcon test --packages-select interactive_cleanup cleanup_vision_ros2
colcon test-result --verbose
```

信息来源：

- `readme.md`
- `pixi.toml`
- `run_and_log.sh`
- `src/interactive_cleanup_ros2/README.md`
- `src/interactive_cleanup_ros2/launch/cleanup.launch.py`
- `src/interactive_cleanup_ros2/launch/sample.launch.py`

## 16. 当前文档漂移与易误导文件

下面这些文件仍然有参考价值，但不能再被视为“当前运行时的直接真相”：

- `src/interactive_cleanup_ros2/README.md`
  - 里面仍然提到旧的 `/cleanup_vision/*` 话题和 `/cleanup_vision/enable`
- `src/cleanup_vision_ros2/config/cleanup_vision_params.yaml`
  - 仍然是旧的 `cleanup_detection` 单节点配置风格
- `src/cleanup_vision_ros2/scripts/cleanup_detection_node.py`
  - 也是旧链路，不是当前主运行路径

后续若有人根据这些旧文件修改系统，很容易误判当前架构。

信息来源：

- `src/interactive_cleanup_ros2/README.md`
- `src/cleanup_vision_ros2/config/cleanup_vision_params.yaml`
- `src/cleanup_vision_ros2/scripts/cleanup_detection_node.py`
- `src/interactive_cleanup_ros2/launch/cleanup.launch.py`

## 17. 当前主要风险与调试优先级

当前最需要优先记住的风险：

1. 预抓取末端姿态可能仍然不对，即使前端目标解析已经接近正确；
2. hand-camera final approach 仍然是 2D 控制，深度缺失带来天然局限；
3. hand node 选“最佳非 person 目标”的策略，在未来多类别完整模型下会明显变脆；
4. destination YAML 仍然比设计方案里设想的稀疏得多；
5. 本地 `hsr_geometry` 是近似模型，可能与仿真里的真实 HSR 几何不完全一致。

结合近期调试进展来看：

- 前半段 cue 锁存、pointing 对齐、active re-observation 这条链已经明显推进；
- 当前最主要的难点已经移动到预抓取姿态正确性与 hand-camera close approach 稳定性上。

信息来源：

- `Interactive cleanup重构方案.md`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/src/pregrasp_planner.cpp`
- `src/interactive_cleanup_ros2/src/hsr_geometry.cpp`
- `src/cleanup_vision_ros2/scripts/hand_perception_node.py`
- 当前会话中的 operator 实跑反馈

## 18. 后续修改前建议优先阅读的文件

如果下一步要改状态机或整体流程：

- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `Interactive cleanup重构方案.md`

如果下一步要改 pointing、pick target 解析、base alignment：

- `src/interactive_cleanup_ros2/src/pick_target_resolver.cpp`
- `src/interactive_cleanup_ros2/src/pointing_alignment.cpp`
- `src/cleanup_vision_ros2/scripts/head_perception_node.py`
- `src/cleanup_vision_ros2/scripts/pointing_utils.py`

如果下一步要改 place destination 解析：

- `src/interactive_cleanup_ros2/src/place_destination_resolver.cpp`
- `src/interactive_cleanup_ros2/config/destination_regions.yaml`

如果下一步要改预抓取、末端姿态、腕部角度、hand camera 相对位姿：

- `src/interactive_cleanup_ros2/src/pregrasp_planner.cpp`
- `src/interactive_cleanup_ros2/src/hsr_geometry.cpp`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`
- `src/interactive_cleanup_ros2/test/pregrasp_planner_test.cpp`
- [https://github.com/ToyotaResearchInstitute/hsr_description](https://github.com/ToyotaResearchInstitute/hsr_description)

如果下一步要改 hand-camera approach 或抓取验证：

- `src/cleanup_vision_ros2/scripts/hand_perception_node.py`
- `src/interactive_cleanup_ros2/src/hand_servo.cpp`
- `src/interactive_cleanup_ros2/src/grasp_utils.cpp`
- `src/interactive_cleanup_ros2/src/interactive_cleanup_sample.cpp`

## 19. 简短结论

当前 Interactive Cleanup 系统已经明显完成了从旧式单体控制器向“分层感知 + cue 锁存 + pointing 对齐 + pregrasp 规划 + hand-camera 近场链路”的升级：

- 等待阶段会跟踪 Avatar，并提前锁存 cue；
- pick 解析已经重新接回“底盘旋转 + bounded re-observation”；
- place 解析已经转成“pointing 几何 + 目的地数据库”；
- `hsr_geometry` 和 `pregrasp_planner` 已经存在；
- final approach 已经迁移到手爪相机；
- `Object_grasped` 发送前也已经加入验证 gate。

当前最核心的未解点，不再是前半段“看懂指向”，而是后半段“让机械臂和末端姿态真正以正确几何完成预抓取与近场接近”。
