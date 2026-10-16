# 单次头部观察测试

用途：比较当前搜索点平视和低头后的 RGBD/检测效果，不把头部运动本身算作搜索成功。
本工具只移动头部；调用前必须停止底盘导航、机械臂动作和其他头部控制源。
不需要修改 Unity 工程或重新生成数据集。现有地图和自动搜索点保持不变。

## 接口依据

- 当前项目 `interactive_cleanup_sample.cpp` 的 `headLookDown()` 为 pan=0、tilt=-0.5。
- Unity `HSRCommon.cs` 给出的 tilt 限位为 [-1.570, 0.523]；工具使用更小的测试范围 [-0.8, 0]。
- 已核对话题 `/hsrb/head_trajectory_controller/command` 和 `/hsrb/joint_states`。

## 操作

先暂停 Unity 完成准备；仅正式执行时恢复 Unity，不拖动机器人或物品。
默认只打印计划，不导入 ROS、不发送命令：

```bash
cd ~/RoboCup-Japan-Open-2026
python3 training/scripts/head_view_trial.py
```

确认没有其他控制源后执行一次（输出目录必须是新的）：

```bash
source /opt/ros/humble/setup.bash
python3 training/scripts/cyclone_test_env.py --domain 71 \
  --dds-config training/config/cyclone_search_stack.xml --run -- \
  python3 training/scripts/head_view_trial.py --run --confirm-head-motion \
  --pan 0 --tilt -0.5 --output /tmp/head-view-down-01
```

工具等待新鲜关节反馈和一个命令订阅者，只发布一条 2 秒轨迹。
确认头部到达且稳定至少 0.6 秒后，保存 3 张新 RGB 图及关节时间戳。
最多运行 12 秒，无反馈则不发命令；超时不是已发送轨迹的取消。
不自动回正；下一轮底盘导航前应明确选择是否用 `--tilt 0` 回正并验证。

`passed` 仅表示关节反馈和图像采集完成，不表示深度有效、识别成功或物品不存在。
现场测试须同时运行已有 RGBD diagnostics，并比较有效深度比例、目标大小和识别结果。
先比较视角，再决定是否更换靠近家具的搜索点；不直接降低深度质量阈值。

## 隔离验证

`training/tests/run_head_view_trial.py` 仅允许 domain 73，模拟正确反馈、错误反馈和无反馈；不连接 Unity。
测试工具的 `--synthetic-test` 也仅允许 domain 73，不能用于 live domain 71。

## 首次现场测试与下一次比较

2026-09-17 在 session1 初始位置，低头前有效深度约 59.9%，低头至 -0.5 后约 95.6%。
诊断模型连续检测到 filled_ketchup；这些结果来自独立 RGBD 诊断，不能代替本工具的稳定验证。
关节后续读数为 -0.49999997，但工具本身超时、未采图，因此不记作完整通过。
旧日志未保存逐帧关节反馈，尚不能判定超时根因；源码及 prefab 的发送周期为 100ms。

现版本新增 `feedback.json`（最多 1000 条），记录到达间隔、反馈年龄、实际角度和拒绝原因；
`summary.json` 记录图像拒绝原因。原 0.3 秒新鲜度/间隔和 0.6 秒稳定窗口门槛不变。
下一次在原位置使用 `--tilt -0.25`、全新输出目录，比较更小俯角，避免目标贴近画面上缘。

## 小俯角实测及时间偏差修正

- -0.25 俯角已通过现场头部验证，采到 3 张新图；番茄酱通过独立多帧定位，有效深度约 84.3%。
- 实测 61 条反馈中，11 条因时间戳轻微领先而拒绝，最大领先约 0.732 毫秒。
- 仅头部 JointState 校验允许最多 1 毫秒领先；0.3 秒旧帧期限、消息顺序和 0.6 秒稳定窗口不变。RGBD 和底盘 TF 校验未改。
- 记录时间序列、假定固定到位姿态的离线对比：修正前 11 次时间拒绝，修正后 0 次；这不是完整运动回放，也不是新增现场实测。
- 原始关节角度现保存到反馈日志，非有限值保存为 null 并仍被拒绝，避免诊断输出失败。
- 下一步是自动搜索中的“到达→调整头部→验证静止→新帧观察”衔接；当前还没有将头部动作自动接入运行器，不能声称自动扫描已完成。
