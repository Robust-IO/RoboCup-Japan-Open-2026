# DDS 双向通信对照（2026-09-13）

在同一 WSL、ROS_DOMAIN_ID=73、ROS_LOCALHOST_ONLY=1 下，使用系统自带
demo_nodes_cpp/talker 与 listener。每组运行约 8 秒，分别清理退出。
没有连接 Unity，没有发送机器人控制指令，没有改动现有桥接节点。

| 发布端 | 订阅端 | 发布数 | 收到数 |
| --- | --- | --- | --- |
| Fast DDS | Fast DDS | 7 | 0 |
| Fast DDS | Cyclone DDS | 7 | 0 |
| Cyclone DDS | Fast DDS | 7 | 0 |
| Cyclone DDS | Cyclone DDS | 7 | 7 |

四组示例进程都成功启动。通过与否根据接收到的匹配消息编号判断，不以
进程退出码代替消息验证。该脚本完成全部对照后可退出 0，即使部分方向不通。

此结果仅说明当前环境和 localhost 配置下混用未通过，不证明两种中间件
在所有配置下都不兼容，也不能单独确定失败发生在发现、传输或消息处理的哪一步。

原始日志：`/tmp/search-mixed-dds-etjookfu/`。
持久化副本：`training/validation/search-mixed-dds-20260913/`。
脚本：`training/tests/probe_ros_mixed_dds.py`。
临时 Cyclone 库清单：`/tmp/handyman-cyclone-comparison-6jepckey/environment.json`。

```bash
source /opt/ros/humble/setup.bash
cd ~/RoboCup-Japan-Open-2026
ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  training/tests/probe_ros_mixed_dds.py \
  --cyclone-manifest /tmp/handyman-cyclone-comparison-6jepckey/environment.json
```

下一步：准备独立、可回退的统一 Cyclone DDS 测试入口。所有参与节点均应
使用一致的环境，包括桥接、定位、导航、视觉与搜索。先验证可执行文件和依赖，
再进行真实节点的消息联调。不得仅切换视觉端并假定现有 Fast DDS 桥接可用。
当前没有切换比赛运行环境，实际相机、TF、里程计、导航动作仍需分别验收。
