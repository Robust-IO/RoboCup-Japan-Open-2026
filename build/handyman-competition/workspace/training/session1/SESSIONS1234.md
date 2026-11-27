# Session1～4 连续自动流程

在 Unity Session1 初始场景启动前运行：

```bash
cd /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1
bash start_sessions1234.sh --execute --from-session1-initial
```

顺序为 Session1 LayoutA → Session2 LayoutB → Session3 LayoutA → Session4 LayoutC。
第三轮结束或超时后不退出监督进程；等待新一轮 Are_you_ready? 和 Environment LayoutC，再退出旧任务、切换地图和初始定位，启动第四轮。不会仅凭第三轮成功消息提前执行第四轮。

Session4 使用 session4-live-config.json：门厅 white_side_table 的 tumbler → 门厅 trash_box_for_burnable，包含已校准桶口数据。保留该配置与派生地图，不重新覆盖。
第四轮结束或 Mission_complete 后退出；尚未接入 Session5、6。旧 start_sessions12.sh / start_sessions123.sh 入口也调用同一四轮监督器。

日志在 D:\handyman-training-runs\sessions1234-时间戳，各轮独立子目录；最新路径见 sessions1234-latest.txt。

Session4 已在单轮收到 Task_succeeded；四轮连续切换尚待整场实测。最新 9.5cm 侧抓尝试阈值尚待实测。
