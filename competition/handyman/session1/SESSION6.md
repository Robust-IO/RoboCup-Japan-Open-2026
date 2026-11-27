# Session6 独立全自动开发

官方任务：Go to the lobby, grasp the filled_ketchup and put the filled_ketchup on the wooden_shelf in the living room.
布局：LayoutD。目标：filled_ketchup。目的地：wooden_shelf#living_room。
源家具已由用户确认：门厅 dining_table#lobby。使用 prepare_session6_config.py --source dining_table#lobby 生成 session6-live-config.json；不从物品真值坐标推断。
配置基于原始 LayoutD 地图，候选位姿按静态可达路径和家具视线筛选，再按路径距离排序。静态检查记录在 session6-route-audit.json。

沿用先靠近家具观察，再按实际目标选正对家具抓取位置的流程。跨房间搬运到客厅，测量木架可用支撑面，完成松爪、后退、收臂后上报，最终以Moderator判定为准。

配置完成并进入Session6初始位置后：
```bash
cd /__HANDYMAN_PACKAGE__/workspace/training/session1
bash start_session6_round.sh --execute --from-session6-initial
```

已通过断点抓取、搬运和放置验证，收到 Task_succeeded / Mission_complete；尚未完成从初始位置的一次无干预整轮回归。
已加入六轮连续监督器。Session5 与 Session6 同属 LayoutD：Session5 收到 Instruction 或官方成功/失败消息前，重复准备握手不能触发 Session6；之后需收到新鲜的 Are_you_ready? 与 Environment LayoutD 才切换，且等待旧任务进程退出再启动下一轮。
木架已完成一次放置实测。松爪前沿用规划支撑面，以当前夹爪 TF 和物品底部偏移判断高度，不重新通过视觉选层。

## 六轮连续启动
先在 Ubuntu 执行，再启动 Unity Session1：
```bash
cd /__HANDYMAN_PACKAGE__/workspace/training/session1
bash start_sessions123456.sh --execute --from-session1-initial
```
顺序 Session1 → Session2 → Session3 → Session4 → Session5 → Session6。
旧的 start_sessions12/123/1234/12345.sh 都调用同一个监督器，也随之执行六轮。
最新日志路径写入 D:\handyman-training-runs\sessions123456-latest.txt。
六轮连续切换仍待整体验证。启动脚本不会自动重启 Unity，也不能将已结束的比赛恢复为 Session1。
