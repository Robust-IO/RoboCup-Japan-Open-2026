# 六轮任务共用底盘转向检查（2026-11-15）

## 覆盖入口

- Session1/2/4/5/6：assembly 安装 SharedTurnGuard。
- Session3：run_session3_search 安装同一个 SharedTurnGuard。
- 导航起步、途中绕障、门口转弯、终点朝向：所有导航 child 统一使用 carry_navigation_runtime。Nav2 只能向 `/handyman/navigation_velocity_unchecked` 发速度；NavigationTurnRelay 检查后才向 `/hsrb/command_velocity` 发布。
- 家具朝向调整和抓取位姿导航：同一导航后端，自动覆盖。
- motion.adjust 以及 runtime.base_move 的旋转：通过 SharedTurnGuard 选择完整左右旋转路径，保留所选方向。混合平移/旋转拆为原 XY 平移后定向旋转。
- 撤离与放置调整当前是无旋转平移，保持原逻辑；以后若请求旋转，同一入口会检查。
- 连续搜索：开转前检查完整 360 度扫掠，放不下则先平移；每次发角速度前再检查前方扫掠。
- 手腕自身旋转不等于底盘转向，本修改不改变抓取纠偏策略。

## 选择与执行

用当前机械臂/夹爪、持物点云以及底盘构造二维包络。手臂模型来自已导入机器人几何和当前 TF；持物只在当前握持状态下加入。
完整比较顺时针、逆时针两条路径，排除碰撞路径，再比较剩余空间。两侧都有至少 15cm 余量时优先短路径，避免空房间无谓绕圈。

导航途中需要反向绕行时锁定选择，先停止平移再转，不接受导航器随后的反方向小修正而来回摇摆。每次检查实际反馈；输入或定位过期即输出零速度。当前激光点可以阻止向走动客人旋转，等空间恢复后继续。

静态地图判断两侧都无法通过时：先停止、取消本次导航目标、确认导航结束及参数恢复，再保持朝向平移腾空间，重新前往原目标。纯旋转近距离调整使用同样的退让逻辑。

## 启动要求

新旧导航发布方式不能混用。更新后必须重新启动本仓库的 session_services.launch.py 或 session1_services.launch.py；它们会把 cmd_vel_topic 设置为检查层的输入。
导航恢复模块 navigation_entry.py 同样使用新主题。
如发现 Nav2 仍直接发布 `/hsrb/command_velocity`，在发送导航目标前报 `navigation_velocity_bypasses_turn_guard_restart_services`，不静默绕过。

单元测试不需要 Unity。ROS 消息接线测试使用独立 domain 183，明确拒绝 domain 71：
`ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=1 PYTHONPATH=. /usr/bin/python3 tests/ros_turn_relay_check.py`

## 验收与边界

代码覆盖不等于已通过 Unity 六轮实测。至少回归一次家具旁持物左右转、门口转向、终点旋转与搜索转圈，再跑六轮。
静态地图可能遗漏突出的家具细节；二维激光也不能看见所有高度的障碍。此功能减少转向碰撞，不构成零碰撞保证。
物品已经掉落时不得从旧持物断点直接恢复。重新获取物品或重启 session 后再测试。
