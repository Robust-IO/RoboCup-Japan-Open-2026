# Session4：门厅 tumbler → 可燃垃圾桶

- LayoutC；源家具 `white_side_table#lobby`。
- 目标 `tumbler` 的坐标仍由实际相机/RGBD识别，不使用任务答案坐标。
- 目的地 `trash_box_for_burnable#lobby`。
- 官方指令：`Go to the lobby, grasp the tumbler and throw it in the trash_box_for_burnable close to the kettle.`

复用现有容器任务：进入房间→源家具观察→识别抓取→撤离收臂→目标垃圾桶导航→夹爪到桶口上方→松爪→撤离→检查外部地面→发送Task_finished→等待官方Task_succeeded。

抓取复用当前侧抓/上抓、夹爪闭合及抬升复核、纠偏规则。投放沿用Session2规则，不增加桌面支撑圆或再次识别物品身份的条件。

## 初次开发的桶口测量
Session2是另一种垃圾桶，坐标及桶口不能直接复制。Session4任务定义还会移动门厅垃圾桶，旧导出位置与定义约差38cm。应以当前RGBD观测为准。

```bash
cd /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/workspace/training/session1
bash start_session4_round.sh --inspect-container --from-session4-initial
```

只在Unity确实处于Session4初始位置时执行。检查入口不发送I_am_ready，不抓取或投放；导航后保存两个头部俯角的RGBD图像。

`measure_container_rim.py`根据审核过的桶口/桶沿像素得到实际地图坐标和桶沿高度，不依赖蓝色桶沿。测量结果传给`prepare_session4_config.py --opening ...`后，清除`container_geometry_pending`。未完成测量时完整任务明确拒绝执行，不用Session2桶口冒充。

## 整轮启动
桶口测量完成后，Unity重置到Session4初始位置：

```bash
bash start_session4_round.sh --execute --from-session4-initial
```

日志位于D盘，`/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/session4-latest.txt`指向本次目录。开发中的Session4尚未接在前三轮连续启动器后；单轮验证通过后再连接。

## 当前限制
外部地面没有检测到目标只是用户要求的本地投放判据，最终由Moderator判分；自动地面重新抓取仍未实现。若检测到桶外物品，记录失败，不伪造成功。
