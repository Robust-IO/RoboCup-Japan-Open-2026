# Handyman Rebuild ROS 2

阶段 3 的启动、验证和故障排查见 [阶段3导航.md](阶段3导航.md)。

这是一个独立于旧 `handyman_ros2` 的重构起点。它以官方
`handyman_msgs/msg/HandymanMsg`、官方 topic 和事件字符串为协议边界，逐步实现可靠的
导航、搜索、抓取、验证和放置闭环。

## 当前范围

已经提供：

- 官方比赛 topic、类型化事件和唯一映射表；
- 输入方向、必填/空 `detail` 与未知事件校验；
- 只允许构造合法 Robot → Moderator 消息的输出接口；
- 结构化 `HandymanTask`；
- 可单元测试的任务状态机；
- 最小 Coordinator 节点；
- ROS bridge/SIGVerse bridge 启动入口；
- 环境、名称和恢复策略配置模板；
- 后续模块的抽象接口；
- 显式隔离的无机器人模块模拟模式；
- 可自动验收完整 happy path 的模拟 Moderator。
- 四布局地图与语义配置加载；
- Nav2 房间/目的地多候选导航、超时恢复和 TF 区域验证。

尚未实现：

- 头部 RGBD 搜索和三维定位；
- 抓取、抓取验证；
- 放置、Avatar handover 和放置验证；
- `Does_not_exist`、`Object_grasped`、`Task_finished` 的真实模块自动上报。

正常模式下，未完成模块不会伪造成功事件。只有阶段 1 验收专用的
`simulate_modules` 参数会使用假模块结果，而且默认关闭。

## 目录职责

```text
include/handyman_rebuild_ros2/
  competition_protocol.hpp  官方协议类型与转换接口
  task.hpp                  任务与状态数据模型
  task_state_machine.hpp    纯状态机
  module_interfaces.hpp     后续模块边界
src/
  competition_protocol.cpp  唯一映射表、输入解析和输出构造
  coordinator_node.cpp      比赛消息入口和总调度
  task_state_machine.cpp    状态转换实现
config/
  environments.yaml         地图、房间、搜索点和目的地区域
  name_aliases.yaml         自然语言/Unity/视觉名称统一
  recovery.yaml             超时和重试策略
behavior_trees/
  handyman_task.xml         后续行为树入口
test/
  task_state_machine_test.cpp
```

## 编译与测试

```bash
colcon build --base-paths src --symlink-install --packages-select handyman_msgs sigverse_ros_bridge handyman_rebuild_ros2
colcon test --base-paths src --packages-select handyman_rebuild_ros2
colcon test-result --verbose
```

这里显式限定 `src`，可避免工作区根目录中的外部/临时 ROS 包副本被重复发现。

## 阶段 1 完整协议验收

下面的 launch 不连接 Unity，也不会发送机器人控制命令。它会启动 Coordinator、
假模块结果和模拟 Moderator，自动验证一轮完整协议的消息内容与顺序：

```bash
source install/setup.bash
ros2 launch handyman_rebuild_ros2 phase1_simulation.launch.py
```

看到 `PHASE 1 SIMULATION PASSED` 且两个节点正常退出即为通过。同一轮回归也已加入
`colcon test`，其中任何消息错序、内容错误或 10 秒超时都会失败。

## 启动

连接 Unity/SIGVerse 时：

```bash
ros2 launch handyman_rebuild_ros2 handyman_rebuild.launch.py
```

只启动 Coordinator 做本地消息测试时：

```bash
ros2 launch handyman_rebuild_ros2 handyman_rebuild.launch.py start_bridges:=false
```

## 推荐实现顺序

1. Instruction Parser 与官方指令测试集；
2. Environment/Navigation Manager；
3. 房间搜索和 `Does_not_exist`；
4. RGBD 定位与抓取验证；
5. 目的地导航、放置验证和 Avatar handover；
6. 四种 Layout 的完整比赛回归。

## 官方协议基线

- 消息：`handyman_msgs/msg/HandymanMsg`
- 输入 topic：`/handyman/message/to_robot`
- 输出 topic：`/handyman/message/to_moderator`
- 官方参考：`src/ros2-competition-msgs/competition_test_tools`
