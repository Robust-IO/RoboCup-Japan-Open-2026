# 搜索目标绑定通信验证

2026-09-13：隔离 ROS_DOMAIN_ID=73、ROS_LOCALHOST_ONLY=1、Cyclone DDS。
真实 NavigationExecutor 向模拟 Nav2 ActionServer 发送指定搜索点。
SearchBindingPublisher 在最终目标接受回调中发布 HandymanMsg；不是比赛回应，
测试 topic 为 `/handyman_test/search_binding`。
detail 使用 YAML，schema 为 handyman-search-binding-v1。

观察器通过 --binding-topic、--task-id、--point-id、--map-sha256 和指定 x/y/yaw
核对消息；接收时消息时龄不得超过 2 秒。任务标识必须每次运行新建，不能重复使用。
Nav2 接受回调是 active 证据；允许随后匹配同一 UUID 的成功状态，包含已缓存状态。
观察器仍要求新鲜 TF、精确地图位姿、持续停稳及新视觉证据。

结果：ActionServer 实际 UUID、C++ 发布 UUID 和 Python 绑定 UUID 一致，最终 found；
所有日志 actionable=false、does_not_exist_authorized=false。没有连接 Unity 域或
发布速度指令。测试视觉和 TF 均为人工数据，不代表真实相机或导航成功率。

源代码：include/handyman_rebuild_ros2/search_binding_publisher.hpp（相对 ROS 包）。
测试：training/tests/run_binding_action_integration.py；C++ fixture 与独立 CMake
保存在 training/tests/binding_fixture/。CMake 当前引用 /tmp/handyman-search-nav-build
中的库，重启后复现需要先重建上一轮独立导航包，再编译 fixture。

默认一个观察器只绑定一次。新增可选 `--max-bound-attempts 2`（范围 1..5），
只有匹配目标 ABORTED 后，才允许以新 UUID、更大执行序号重新创建 SearchScan、
SearchArrival 和 TfSearchObserver；这不是重启进程，而是重建每次尝试的状态对象。
清空旧停稳窗口、视觉计数和位姿，整体 --seconds 超时保持不变；CANCELED 不恢复。
失败历史在 JSONL 中保留，新尝试的状态只记录本次证据。没有新绑定则等待总超时，
不会主动发送或重试导航。多个入口中的中间途经目标仍不触发观察。

模拟 retry_recover 验证：首次 ABORTED，第二次不同 UUID 成功，重新收集 TF 后 found。
关闭该选项的原 retry 场景仍保持失败即停止。70 项原搜索测试加 5 项重试边界测试通过。
取消、旧 UUID 成功消息、旧执行序号等目前为单元测试，未全部做进程间故障注入。

新增乱序处理：当前目标尚无终态时，可以暂存一个通过身份校验的新 UUID/更大序号
绑定，最多 2 秒。重复消息不延长寿命，冲突候选被拒绝。收到旧目标 ABORTED 后
再次检查消息时龄及身份，才重建观察状态；旧目标成功或取消会清除暂存绑定。
隔离 retry_reorder 场景刻意将失败状态延迟到第二次绑定之后，日志验证顺序为
retry_binding_deferred → waiting_for_new_retry_binding → 第二次 search_goal_bound → found。
80 项搜索单元测试、17 项 TF 相关测试通过。超期与冲突目前为单元测试覆盖。

限制：尚未接入正式 coordinator；仅在明确 ABORTED 后切换。不猜测丢失的失败
状态；缓存过期或失败消息一直不来，仍按总超时安全结束。
本次地图标识为测试配置文件摘要，由测试两端提供；生产端仍需实际地图包哈希校验。
消息仅做一致性检查，不提供恶意节点身份认证。途经点不发布绑定已在 4 个途经点加
1 个最终点的模拟链验证；不代表真实门洞通行效果。
原有 --goal-id 手动观察模式保留。Unity 无需为此测试运行。
