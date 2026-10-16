# 独立 Cyclone DDS 测试入口

运行库位于 `~/handyman-tools/cyclonedds-20260913/opt/ros/humble`。
只从已校验的软件包解包复制，没有系统安装，没有修改 `.bashrc`、旧启动文件或比赛配置。

## 默认只做预检查

```bash
source /opt/ros/humble/setup.bash
source ~/RoboCup-Japan-Open-2026/install/setup.bash
cd ~/RoboCup-Japan-Open-2026
/usr/bin/python3 training/scripts/cyclone_test_env.py
```

应显示 `preflight: passed`、`domain: 73`、`started: false`。
检查库文件、Python ROS 模块和 RMW 动态库加载；不创建 ROS 节点。
此检查不能证明桥接、导航插件或实时消息全部可用。

## 显式运行隔离测试

```bash
/usr/bin/python3 training/scripts/cyclone_test_env.py --run -- \
  /usr/bin/python3 training/tests/probe_ros_official.py
```

只有 `--run -- 命令` 才启动指定程序；环境只作用于这个程序及其子进程。
默认域 73，与现有域 71 分离，使用 localhost 模式。官方示例仅发布测试字符串。
查看 listener 日志是否有 `I heard`，不要仅凭退出码判断消息通过。

## 后续整套节点联调的约束

- 所有参与节点（桥接、定位、导航、视觉、搜索）都须经同一入口启动。
- 本入口本身不启动整套节点，也不会自动停止或替换现有桥接。
- 子程序若自行覆盖 ROS_DOMAIN_ID 或 RMW_IMPLEMENTATION，会破坏统一环境。
  `start_rgbd_diagnostics.py` 已适配：默认继承 ROS_DOMAIN_ID，未设置时用 71，
  可显式指定 --domain；保留继承的中间件和库路径。端口检查可用
  --rosbridge-port 与 --sigverse-port 指定，不改变 Unity 的连接配置。
- 实际切换域 71 需显式 `--domain 71`；必须先确认旧参与节点已停止，
  避免两套协调器或导航同时控制机器人。目前尚未执行这一切换。
- 不直接调用完整比赛 launch 来做只读验收：它可能启动协调器并发送协议消息。
- 先验证桥接节点加载和端口，再取得用户恢复 Unity 的确认，进行实际相机/TF/里程计检查。

## 回退

停止通过新入口启动的进程，再使用原启动命令。没有全局环境改动需要撤销。
独立运行库目录可以暂留；不要在仍有进程使用它时移动或删除。

## 2026-09-13 桥接启动检查

`training/tests/check_cyclone_bridges.py` 默认只列出命令；显式 --run 才启动
域 73 的 rosbridge:19090 与 sigverse:51001。两端口须空闲，最多等待 15 秒，
检查成功后运行视觉默认预检查，最后关闭本次进程并检查端口释放。
不会启动协调器、导航或相机订阅，不探测二进制桥接协议，不连接 Unity。

```bash
/usr/bin/python3 training/scripts/cyclone_test_env.py --run -- \
  /usr/bin/python3 training/tests/check_cyclone_bridges.py --run
```

本次结果：两桥接监听成功，视觉预检查继承域 73 / rmw_cyclonedds_cpp，
退出后测试端口释放；7 项视觉启动器测试通过。
日志：`training/validation/cyclone-bridge-startup-20260913/`。
旧启动器备份：`~/handyman-tool-backups/before-domain-launcher-xvs1igx3/`。
此结果仅验证启动及预检查，不证明 Unity 消息已传入或 GPU 模型已加载。
