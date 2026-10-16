# 小数据报对照配置（2026-09-13）

背景：实际 Unity 采样时头部图像已进入桥接且 publish 返回、订阅已匹配，
但接收端 0 帧；同期系统 IP 重组失败增加。系统计数不是话题专属，尚不能
证明这些失败全部来自头部图像，也不能仅凭此确定根因。

新增 `training/config/cyclone_small_datagrams.xml`：
General/MaxMessageSize=1200B，General/FragmentSize=1024B。
意图是使用较小 DDS 数据报，减少依赖 IP 分片重组；并非降低图像分辨率或压缩图像。
DDS 分片与 IP 分片是不同层次，参考
[Cyclone DDS 0.10.5 分片说明](https://cyclonedds.io/docs/cyclonedds/0.10.5/config/cyclonedds_specifics.html)。
实际最大包长仍须抓取测试数据报或跟踪系统调用验证，配置值不等于已测得的包长。

入口新增 `--dds-config 路径`，只把配置传给本次子进程及后代。默认不启用。
启动前的 ctypes 预检查只验证库加载，XML 是否有效由实际 DDS 节点创建验证。

隔离域 73 / 端口 51001，以诊断桥接进行双路 BSON 图像测试：
RGB/深度各 20 帧，best-effort 与 reliable 四路均完整收到，字节校验无差异；
诊断计数开启，测试结束端口释放。本次未复现实际 Unity 多路负载，不能宣告修复。

证据：`training/validation/dds-small-datagrams-20260913/`。
原始日志：`/tmp/bson-image-bridge-62qnjbsi/`。
入口旧版本：`~/handyman-tool-backups/before-small-datagrams-te5czji6/`。

```bash
source /opt/ros/humble/setup.bash
source ~/RoboCup-Japan-Open-2026/install/setup.bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/cyclone_test_env.py \
  --dds-config training/config/cyclone_small_datagrams.xml --run -- \
  /usr/bin/python3 training/tests/probe_bson_image_bridge.py \
  --bridge-executable ~/handyman-tools/bridge-audit-build-20260913/sigverse_ros_bridge --audit
```

下一步为真实桥接和接收诊断同时显式启用该配置，进行短时对照。启用在发送端
才可能改变发送数据报；仅更改订阅端不够。本轮没有切换正在运行的桥接、
没有修改 sysctl、防火墙或 Unity。切换前通知用户保持暂停，切换后重新进入 Play。
