# 模拟 Unity BSON 图像桥接验收（2026-09-13）

使用现有已编译 SIGVerse 桥接程序，在独立域 73、端口 51001 接收合法合成
BSON 图像。未修改桥接源码、Unity 文件或生产桥接端口 50001。

测试链路：两条 TCP 连接分别发送 RGB/深度 → BSON 解析 → ROS Image 发布 →
独立 ROS 节点接收。发布的是 `/handyman_test/rgb` 和 `/handyman_test/depth`，
不包含机器人控制话题。

先等待四路订阅均实际收到小图预热，不仅依赖发现/匹配计数，然后发送正式样本：

| 图像 | 大小 | 发送 | best-effort | reliable |
| --- | --- | --- | --- | --- |
| RGB | 640×480 rgb8 / 921,600 字节 | 20 | 20 | 20 |
| 深度 | 640×480 16UC1 / 614,400 字节 | 20 | 20 | 20 |

20 个唯一序号均收到。每帧核对宽高、step、编码、字节序和全部像素字节，
没有内容不一致。退出后测试端口释放。原有生产桥接保持运行。

合成深度的字节仅用于传输一致性检查，不代表真实距离；没有运行定位模型。
测试先后建立两条 TCP 连接，并以约 0.12 秒一组发送；不能代替真实 Unity
全部相机并发、实际帧率、网络抖动或 TCP 分段异常的验证。没有证明实际图像
正在持续进入生产桥接，也没有解释此前真实 RGB/深度为 0 帧的原因。

脚本：`training/tests/probe_bson_image_bridge.py`。
持久化证据：`training/validation/bson-image-bridge-20260913/`。
原始输出：`/tmp/bson-image-bridge-4srr4q6y/`。

```bash
source /opt/ros/humble/setup.bash
source ~/RoboCup-Japan-Open-2026/install/setup.bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/cyclone_test_env.py --run -- \
  /usr/bin/python3 training/tests/probe_bson_image_bridge.py
```

下一步：检查真实 Unity 图像连接上的输入活动与多路图像负载。需要短时恢复
Unity 时先通知用户，不在用户暂停期间把零输入判作故障。
