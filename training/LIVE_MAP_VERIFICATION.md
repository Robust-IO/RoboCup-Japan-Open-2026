# 只读活动地图内容校验

使用已安装 Nav2 map_io 库解码本地地图，避免重写阈值、翻转等解析规则。
文件解码程序不创建 ROS 节点；序列化参考 OccupancyGrid 到独占创建的临时文件。
Python 校验器订阅地图并逐项比较 frame_id、尺寸、分辨率、原点及全部栅格。
静态地图允许 latched 消息，不能以消息时间旧就认定失效。

```bash
source /opt/ros/humble/setup.bash
cd ~/RoboCup-Japan-Open-2026
cmake -S training/tools/map_snapshot -B /tmp/handyman-map-snapshot-build
cmake --build /tmp/handyman-map-snapshot-build -j2
```

`verify_live_map.py` 参数：--map-yaml、--decoder、--topic（默认 /map）、
--seconds（默认 8，最多 30）、--output（独占创建 JSON）。需使用与发布端一致
的 ROS 域和 DDS 配置；不要为模拟测试改动域 71 上的桥接进程。

consistent=true 要求采样期间至少收到地图、所有接收地图匹配、本地文件前后摘要
不变、结束时恰有一个发布者。不订阅 costmap，不调用 load_map，不发布控制命令。
这只证明所收到 /map 的内容，不证明 Nav2 costmap 已同步、定位正确或导航准备好。
消息比较成功仍 actionable=false、costmap_verified=false。
已作为可选 --decoder 模式接入请求消费端，详见 COORDINATOR_SEARCH_REQUEST.md；
生产使用前必须重新校验，而非永久复用旧 JSON 结果。

8 项单元测试通过。隔离 ROS 域 73 用真实 Nav2 解码的 LayoutA 参考地图模拟发布，
匹配场景 consistent=true；修改一个栅格后拒绝。未对当前 Unity 会话 /map 做实测。
证据目录：training/validation/map-equality-20260913。
