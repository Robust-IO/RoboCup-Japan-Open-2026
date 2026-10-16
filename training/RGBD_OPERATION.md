# 视觉诊断操作

当前范围：头部六类分割、RGBD 可见表面定位、同时间 TF、三帧稳定、目标移除与断流撤销。
不包含搜索扫描、全房间不存在判定、抓取或比赛计分；所有输出 actionable=false。

## 1. Ubuntu 保持桥接（Unity 可暂停）

在两个保持打开的 Ubuntu/WSL 终端分别运行。若端口已被现有桥接监听，不重复启动。
SSH 临时启动后退出可能导致 WSL 服务消失，建议保留终端。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

```bash
source /opt/ros/humble/setup.bash
source ~/RoboCup-Japan-Open-2026/install/setup.bash
export ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1
ros2 run sigverse_ros_bridge sigverse_ros_bridge 50001
```

检查 `ss -ltn`：9090 和 50001 应为 LISTEN。桥接重启后 Unity 可能需要退出并重新进入 Play。

## 2. 离线预检查（不需要 Unity 运行）

```bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/start_rgbd_diagnostics.py
```

默认只检查文件、已审阅相机报告哈希和监听端口；不加载模型、不订阅相机。
端口正常不代表 Unity 已连接；`ss -tn` 可查看来自 Unity 的已建立连接。
报告默认路径 `/mnt/c/Users/wpb15/Downloads/handyman-camera-audit.json`。
新报告须重新审阅，不允许简单替换哈希绕过检查。

## 3. 实测（此时才恢复 Unity）

饮料罐放在桌面、相机可见；先保持机器人及物品静止。

```bash
/usr/bin/python3 training/scripts/start_rgbd_diagnostics.py --run --seconds 15 --target canned_juice
```

启动会打印本次唯一日志目录，位于 `~/handyman-datasets/live/rgbd-日期时间-编号`。
模型启动额外耗时；采样默认 15 秒，允许 1～120 秒。结束后可立即暂停 Unity。
Ctrl+C 可提前停止；退出清理只处理本次推理进程，不停止桥接。

## 4. 看什么

- `diagnostics.jsonl`：累积 count=1、2 后 stable=true；坐标为 odom 中可见表面统计，不是抓取点。
- 每个编号目录：rgb.png、depth.bin、request.json、result.json、detection.jpg。
- 目标移除：target_not_in_current_frame，stable=false，不保留旧坐标。不是 Does_not_exist。
- 暂停/断流：约 2 秒无结果后 waiting_for_fresh_result；恢复需重新累计三帧。
- TF 等待上限 0.5 秒；过期结果或低置信度/无效深度不得通过。
- 正常停止：diagnostic_stopped；如果无该记录或有清理异常，保留日志检查。

唯一 ROS 输出：`/handyman/vision/diagnostics`，std_msgs/String JSON。
相机范围受当前审阅配置约束：640x480、16UC1 毫米、近距离 0.4 米盲区。
当前位置稳定性验收主要针对饮料罐；支持其他类别不等于这些类别已完成定位验收。

## 5. 已完成验证与剩余工作

已实测：静止稳定、目标移除撤销、暂停/恢复重新累计、SIGINT 正常退出。
已有离线回归测试；模型/场景/相机配置改变后应重新核对并复测。
尚未完成阶段四整体验收：多类别三维定位、搜索扫描、房间覆盖及 Does_not_exist 判定。
