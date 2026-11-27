# Session1 → Session2 → Session3 连续自动化

在 Ubuntu 终端先启动监听，再从 Unity 的 Session1 初始状态开始比赛：

```bash
cd /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1
bash start_sessions123.sh --execute --from-session1-initial
```

仅从 Session2 初始状态测试第2、3轮：

```bash
bash start_sessions123.sh --execute --resume-session2-initial
```

旧的 start_sessions12.sh 入口仍可使用，现在也会继续执行第三轮。

顺序：
1. Session1 / LayoutA：原 Session1 自动搬运任务。
2. Session2 / LayoutB：canned_juice → trash_box_for_bottle_can。
3. Session3 / LayoutA：搜索 bear_doll；收到官方修正指令后，从 corner_sofa 抓取 dog_doll 并递交给 Moderator。

成功或时间耗尽都不能凭本地计时直接启动下一轮。监听新的 Are_you_ready? 与 Environment；旧任务先取消并退出，然后切换地图/定位、准备新轮次初始空手状态，启动独立任务进程。视觉、抓取上下文不跨轮复用。

Session3 接入本次已实测成功的持物轮廓导航与视觉持物复核。第三轮结束后本启动器退出，尚未连接 Session4～6。

日志全部保存在 D 盘：
- `/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/sessions123-latest.txt` 指向最近运行目录。
- 运行目录的 `events.jsonl`、`supervisor-state.json` 记录轮次切换。
- `session1/`、`session2/`、`session3/` 分别保存各轮日志与证据。

本次已通过离线状态转换与入口路由测试；Session3 单轮已有官方 Task_succeeded，三轮连续切换仍需一次完整实测。
