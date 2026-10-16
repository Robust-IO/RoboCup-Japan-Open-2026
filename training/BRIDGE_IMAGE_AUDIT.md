# 可选桥接图像计数（2026-09-13）

源码：`src/sigverse_ros_package/sigverse_ros_bridge/src/sigverse_ros_bridge.cpp`。
开关：仅 `HANDYMAN_IMAGE_AUDIT=1` 开启；默认关闭，不改变图像内容、话题或 QoS。
首次 3 帧和之后约每秒记录发布前后两条日志，避免每帧输出。日志按行加锁，
以免多个图像连接的内容混在一行。不记录像素内容。

日志前缀 `HANDYMAN_IMAGE_AUDIT`：

- topic / fd / tid：连接与话题对应关系。
- received：完整 BSON 读取并进入 Image 分支的次数。
- started / completed：publish 调用开始与正常返回次数。
- bson_bytes / binary_bytes / data_bytes：累计 BSON 字节、本帧二进制长度与 ROS 图像长度。
- width / height / step / encoding：真实发布图像规格。
- subscribers：匹配订阅数量，不是收到图像的确认。
- publish_ms：本次发布路径耗时（包含该次前置诊断输出开销）。

注意：计数在 publish 前后采样输出，不是每帧流水日志；如果在图像解析或
复制阶段异常退出，不能凭缺少计数日志认定没有 TCP 输入。
publish 正常返回不代表订阅端已收到数据，必须与轻量接收计数对照。

## 构建与验收

独立可执行文件：
`~/handyman-tools/bridge-audit-build-20260913/sigverse_ros_bridge`。
没有执行安装，没有替换 `install/sigverse_ros_bridge` 内的程序，
当前生产桥接没有重启，因此当前进程不会输出新增计数。

关闭与开启开关各跑一次隔离 BSON 图像测试（域 73 / 端口 51001）：
每次 RGB、深度各 20 帧，两种订阅 QoS 均完整收到，字节校验无差异。
关闭时诊断日志 0 行；开启时 20 行，两个话题均有 after_publish。
两轮均正常释放测试端口。未测试真实 Unity 多相机负载下的日志开销。

证据：`training/validation/bridge-image-audit-20260913/`。
原源码备份：`~/handyman-tool-backups/before-image-audit-msheio1t/`。

```bash
source /opt/ros/humble/setup.bash
source ~/RoboCup-Japan-Open-2026/install/setup.bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/cyclone_test_env.py --run -- \
  /usr/bin/python3 training/tests/probe_bson_image_bridge.py \
  --bridge-executable ~/handyman-tools/bridge-audit-build-20260913/sigverse_ros_bridge --audit
```

下一步需要显式切换实际桥接到此构建并启用开关，然后短时采样；切换会断开
Unity 连接，应先让用户停止或暂停 Unity，桥接就绪后再提示重新进入 Play。
旧程序保留，可回退；不要用叠加启动的方式争抢生产端口。
