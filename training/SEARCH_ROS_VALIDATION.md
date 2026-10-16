# 搜索 ROS 隔离联调记录（2026-09-13）

## 结论与范围

在临时解包的 Cyclone DDS 下，5 项真实 ROS 进程间模拟消息测试通过；
53 项搜索单元测试通过。尚未完成 Unity 实际导航、相机和里程计联调，
未发送机器人移动指令或比赛回应，不能据此宣告第四阶段完成。

默认 Fast DDS 路径仍有故障，未修复或替换系统默认中间件。
同机官方 C++ talker/listener 对照：Fast DDS 发布 7 条、订阅 0 条；
Cyclone DDS 发布 7 条、订阅全部 7 条。网络调用记录显示 Fast DDS 接收
进程已收到 UDP 数据，但应用订阅回调未触发。此证据不足以确定具体内部根因。

## 集成测试

使用隔离 ROS_DOMAIN_ID=73、ROS_LOCALHOST_ONLY=1，人工生成导航状态、
map→base_footprint TF、里程计及视觉诊断；没有动作服务器或速度命令发布器。

| 场景 | 结果 |
| --- | --- |
| 匹配导航成功、持续停稳、新视觉目标 | found |
| 观察期间移动 | incomplete / movement_during_observation |
| 观察期间里程计断流 | incomplete / odometry_stream_timeout |
| 仅收到旧成功状态，未观察到 active | 不开始观察，最终超时 |
| 精确时间 TF 缺失 | 拒绝位姿，最终超时 |

所有输出 actionable=false、does_not_exist_authorized=false。
缺失 TF 测试没有模拟延迟 TF 重试；后续应补充这一场景。

## 可追溯文件

- 持久化证据：`training/validation/search-ros-20260913/`。
- 原始集成输出：`/tmp/search-ros-integration-akjzj0pm/`。
- Fast DDS 官方对照：`/tmp/ros-official-probe-e4n073ia/`。
- Cyclone DDS 官方对照：`/tmp/ros-official-probe-pk68s98i/`。
- 临时库及环境清单：`/tmp/handyman-cyclone-comparison-6jepckey/`。
- 已核验包：Windows `C:\Users\wpb15\handyman-cyclone-debs-MBAQns`，
  SHA-256 与远端 APT 元数据一致。只解包，不执行 apt install。

临时目录可能在重启后消失。复现时重新解包，不要把临时库路径写进 .bashrc。

```bash
source /opt/ros/humble/setup.bash
cd ~/RoboCup-Japan-Open-2026
ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  training/tests/run_search_with_test_middleware.py \
  /tmp/handyman-cyclone-comparison-6jepckey/environment.json
```

`probe_ros_official.py` 是记录型探针；仅进程正常退出不代表消息收到，必须检查
listener 日志。`run_search_ros_integration.py` 则对 5 项结果执行断言。

## 下一步

先验证 Fast DDS 与 Cyclone DDS 的双向通信兼容性，再决定正式节点的中间件
配置。现有 Unity 桥接节点仍使用原配置，不应假设混用已经可行。
不应为了让测试通过放宽停稳或目标确认阈值。
