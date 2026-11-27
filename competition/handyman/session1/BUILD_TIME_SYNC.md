# Build exe 的 Ubuntu 自动校时

不依赖 Unity Editor，不要求改动或重新构建比赛 exe，不修改操作系统时钟，不改写 ROS 消息时间戳。

已验证示例 exe 使用 SIGVerse 原生 `TimeSync` 连接：Unity 首次请求偏移，且在运行中持续接收 `time_gap,秒,毫秒` 返回。新版 Ubuntu 桥接沿用该接口。

六轮启动器先确保新版桥接运行。每轮导航服务重置后、初始化夹爪前，`startup_clock_ready.py` 检查关节数据。机器人静止且连续数据证明源时间超前时，调用 `/sigverse/recalibrate_startup_clock`，由桥接通过原生 TCP 连接设置新的绝对偏移；随后必须重新采样验证成功才启动动作。

不会把网络延迟当成时钟偏移：只凭数据落后不自动校准；重复、倒退、停止更新、强抖动、机器人移动时不会用这些样本计算修正。校准时不允许其他运动发布者。单向协议有传输误差，因此保留短时间裕量并依靠校准后的实测验证，不宣称硬件级时钟同步。

第一次桥接升级需关闭再打开 exe。以后先执行原有 `start_sessions123456.sh --execute --from-session1-initial`，再从 Session1 启动比赛。启动器自动复用/启动新版桥接，六轮之间不需要点击同步。

新桥接二进制及 SHA256 记录在 D 盘 `handyman-training-runs/clock-bridge-active.json`；桥接日志位置记录在 `bridge-clean-latest.json`。每轮目录包含 `clock-ready.log`、`clock-ready.json`。

桥接保留既有 BSON 分帧与相机传输修复。旧二进制没有覆盖，换用独立构建目录，原桥接配置保存于新桥接目录的 `previous-bridge.json`。

若正式 exe 移除了原生时间同步组件，自动校时会明确报告缺少连接，不能保证适用于该 exe；应在比赛准备阶段验证接口。已经观察到本示例 exe 有该连接。

测试：C++ 偏移策略、Python 时钟窗口测试、独立 ROS domain 84 + TCP 55091 的 BSON/ROS 原生往返测试。真正 exe 的自动校时验收必须在重新连接后完成。
