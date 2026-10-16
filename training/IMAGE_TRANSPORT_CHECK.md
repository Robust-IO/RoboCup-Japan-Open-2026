# 图像传输隔离对照（2026-09-13）

Unity 暂停，域 73，Cyclone DDS，两个独立 Python ROS 进程。
模拟发布 sensor_msgs/Image，发布端 reliable，接收端同时使用 best-effort 和 reliable。
没有连接 Unity 或生产桥接端口，没有机器人控制命令。

第二轮在发布端匹配到两个订阅者后开始发送：

| 测试载荷 | 发布 | best-effort 接收 | reliable 接收 |
| --- | --- | --- | --- |
| 64×64 RGB（12,288 字节） | 25 | 1 | 1 |
| 640×480 RGB（921,600 字节） | 25 | 25 | 25 |
| 640×480 深度（614,400 字节） | 25 | 25 | 25 |

小图阶段仍有丢帧，因此不能将整轮测试标记为全部通过；匹配计数不保证应用
已能即时接收所有首帧。第一轮结果也保留用于比较，不能用第二轮覆盖它。

证据支持：该配置可以传递测试尺寸的单路图像；不是任何此尺寸的大消息都无法到达。
不能据此排除多相机并发、实际帧率、丢包/重组、桥接发布、Unity 图像生成等问题。
此测试绕过了 Unity TCP/BSON → SIGVerse 桥接路径。

后续应验证独立测试桥接的 BSON 图像输入到 ROS 发布链路，并增加必要的
接收/发布计数，再安排一次短时 Unity 验证。不要仅因发布节点存在就断言
图像持续发布，也不要直接修改训练模型或放宽视觉质量阈值。

脚本：`training/tests/probe_image_transport.py`。脚本输出统计，不以退出码
代替完整性验收。
持久化日志：`training/validation/image-transport-20260913/`。
原始第二轮：`/tmp/image-transport-ri3f84xm/`；第一轮：`/tmp/image-transport-1ipod49t/`。

```bash
source /opt/ros/humble/setup.bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/cyclone_test_env.py --run -- \
  /usr/bin/python3 training/tests/probe_image_transport.py
```
