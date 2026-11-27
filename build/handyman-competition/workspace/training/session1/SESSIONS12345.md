# Session1～5连续自动流程

```bash
cd /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1
bash start_sessions12345.sh --execute --from-session1-initial
```

先启动监督器，再启动Unity Session1比赛。顺序：LayoutA → LayoutB → LayoutA → LayoutC → LayoutD。
每轮结束或超时后等待新鲜的Are_you_ready?和对应Environment握手，清理上一轮控制进程后切换地图和初始定位，再执行下一轮。
第五轮任务：厨房木色柜body上的rabbit_doll → 客厅square_low_table。沿用真实识别后的家具正向抓取位姿；桌面松爪、后退、收臂完成后上报Task_finished，最终成功以Moderator为准。
第五轮结束或Mission_complete后停止，Session6尚未接入。
日志：D:\handyman-training-runs\sessions12345-时间戳；指针sessions12345-latest.txt。
旧start_sessions12/123/1234.sh均指向同一监督器，现在也执行五轮。
离线切换回归已验证，完整五轮连续实测尚未完成。
