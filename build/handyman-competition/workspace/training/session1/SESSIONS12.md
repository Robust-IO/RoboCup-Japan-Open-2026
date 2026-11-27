# Session1 → Session2 连续自动运行

## 操作

先停止Unity，恢复从Session1开始的赛事配置，确保后续会自然进入Session2。不要保留跳过Session1的开发配置。保持ROS桥接和模型服务在线。

Ubuntu终端执行（可以在Unity进入前启动，程序只等待握手）：

```bash
bash /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1/start_sessions12.sh --execute --from-session1-initial
```

随后从Unity的Session1初始位置开始。不要另外启动start_session1.sh或start_session2.sh。日志根目录记录于D:\handyman-training-runs\sessions12-latest.txt；每轮独立的live.log、task/live-state.json及运行证据均在该目录下。

## 自动行为

- 只从LayoutA的Are_you_ready?/Environment真实握手启动，不把仍在运行的LayoutB误当Session1。
- 每轮先停旧地图/初始定位进程组，加载正确地图，用该地图initial_pose重建定位，打开初始空夹爪，然后交由原任务流程握手执行。
- 第一轮任务进程退出后等待LayoutB握手，不使用定时器猜测切换。
- 若Unity提前进入LayoutB，先中断并等旧任务进程完整退出，再准备第二轮；不并行控制机器人。
- Session1成功、失败或本地执行停止后，仍等待Unity实际进入第二轮。程序不会发送Give_up来强迫切轮，也不会把本地停止当成比赛成功。
- 第二轮结束/进程退出或Mission_complete后停止，不启动Session3。
- 两轮共享地点约束>50%识别、侧抓失败改上抓、试提后再次闭爪确认逻辑。

## 任务

1. LayoutA：客厅white_side_table上的dog_doll，放到客厅square_low_table。
2. LayoutB：客厅wooden_side_table上的canned_juice，丢入客厅trash_box_for_bottle_can。

## 验证与限制

2026-11-15：163项离线测试通过；双轮连续Unity实测尚未进行。当前仅串联这两个固定任务配置，不是六轮通用指令解析器。地图与定位服务会由入口切换，不修改Unity。准备阶段异常则报错停止，不允许旧控制器与新任务重叠。Ctrl+C会请求结束任务，等待其停止后退出；保留外部桥接、模型服务及地图进程。
