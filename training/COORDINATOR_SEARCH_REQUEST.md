# 协调器搜索请求出口

默认关闭参数 `search.publish_requests`。不得与 `simulate_modules` 同时启用。
仅在真实房间导航验证成功并发送 Room_reached 后发布请求，topic 为
`/handyman/search/request`，HandymanMsg，reliable/transient_local/depth=1。
detail 为 YAML，schema=handyman-search-request-v1。

search_requested 包含本次随机 task_id、ROS stamp_ns、environment、layout、room、
target、map_uri、frame_id=map、带稳定编号的搜索点。actionable=false，
requires_map_verification=true：此消息不是导航授权，也未验证实际加载地图版本。
接收者仍需校验当前任务、消息新鲜度及地图包摘要，不能把缓存的历史请求直接执行。

修正/新指令被接受、Task_failed、Task_succeeded、Mission_complete 等分支使旧请求
失效，发布 search_cancelled，携带原 task_id 和原因，随后清空请求标识。
取消通知本身不会停止尚未接入的外部导航程序；正式执行端必须处理它并确认取消。
目前已有下述只读消费端，但没有新搜索导航、结果回传状态机或物品不存在回应。
既有导航执行代码及默认启动行为保持原路径，独立构建未替换常用 install。

验证：run_coordinator_search_request.py 在隔离域 73 上运行新协调器、模拟 Moderator、
厨房内 TF。实际收到 I_am_ready、Room_reached、含 4 点的苹果搜索请求，以及
Task_failed 对应同 task_id 的取消。没有 Task_finished/Object_grasped/Does_not_exist/Give_up。
这不是 Unity 实测，也没有验证修正指令与取消通知丢失的完整恢复流程。

## 只读消费端

`training/scripts/search_request_consumer.py` 默认从 ament 获取当前 overlay 的包
share 路径，也可显式指定 --package-share。读取 catalog/environment YAML，并核对
layout、room、map_uri、搜索点 ID 和位姿；计算环境文件摘要与地图 YAML+图像摘要。
资源路径不得越出包目录，前后重读发现文件变化则拒绝。尚未解码栅格验证地图内容，
也未比较 Nav2 当前 /map；prepared_readonly 不表示地图可导航或房间覆盖完整。

请求必须新鲜（2 秒内），一次只保留一个。匹配取消会清空它；取消先于请求到达时
记录已取消任务，防止迟到请求复活。重复请求不刷新有效期，准备状态 30 秒过期。
最多记录 1000 个失效任务，达到上限拒绝新请求。取消不依赖新鲜度，避免延迟取消
被忽略；任务 ID 校验不是发布者身份认证。进程停机清空准备状态。

无发布器、动作客户端或服务客户端，全程 actionable=false。
实际协调器→消费端隔离联调验证了 prepared_readonly→cancelled，89 项搜索单元
测试通过。地图摘要只用于本地快照，连接导航之前仍必须核对活动地图并重新校验文件。

## 可选的地图消息校验接入

传入 --decoder /tmp/handyman-map-snapshot-build/map_snapshot 启用，--map-topic 默认 /map。
后台解码参考地图，不阻塞请求取消；解码完成后再次核对 task_id 和地图摘要，
已取消、过期的请求不会因旧解码结果恢复。只缓存一张最新地图、一个解码作业。
与单一发现发布者的地图内容匹配后输出 map_content_verified/live_map_verified=true。
这里 live_map_verified 只代表收到的地图消息，不代表 costmap、定位或执行权限。

周期 0.5 秒重查本地地图/环境摘要及 ROS 图端点；变化、地图不匹配或发布者缺失/
歧义都会撤销请求并记录为失效。不能自动恢复，需要新的任务请求。瞬时变化或检测
间隙无法排除，故未来实际发导航前仍需再校验。当前 Humble 不提供此回调的 MessageInfo，
端点标识来自 ROS 图快照，不是逐条消息的来源认证。

新增 7 项测试，总计 96 项搜索测试。模拟协调器链验证地图匹配→取消、匹配→单栅格
变化→撤销；首次联调发现回调签名不兼容，修正后通过，保留失败和重测日志。

## 单搜索点整链隔离测试

`run_coordinator_search_request.py --execute-search` 仅允许域 73/localhost。
真实协调器发苹果搜索请求，消费端验证地图；测试程序选择第一个厨房搜索点，
复核地图摘要、启动观察器并等待订阅就绪，然后调用 C++ NavigationExecutor 测试入口。
假 Nav2 接受目标，实际 UUID 经绑定工具送到观察器，停稳和模拟视觉证据满足后 found。
task_id/target/map_bundle_sha256 来自同一次请求准备结果，不手工替换成另一目标物。

已通过：请求→地图验证→真实执行器/假 Nav2→UUID 绑定→停稳→模拟苹果 found→取消请求。
仅发出 I_am_ready 和 Room_reached，没有提前回应抓取或任务完成。首次清理等待时间
短于消费端剩余运行时长而超时，修正测试等待后完整通过，未放宽业务阈值。

这是测试脚本内的连接，不是正式执行端。仍无生产导航授权、完整搜索覆盖或结果回传
协调器；取消测试发生在导航结束后，尚未验证导航过程中取消与地图撤销能停止对应目标。
96 项搜索和 8 项地图单元测试通过，证据目录 training/validation/full-search-chain-20260913。

## 导航进行中的定向取消验证

整链测试新增 --execute-search --stop-during cancel 或 --stop-during map。
假 Nav2 保持目标执行中，测试程序等待消费端取消/地图撤销，再向隔离 C++ 测试
入口发 task_id 对应的取消消息。先发错误 task_id，断言没有取消；再发正确标识，
核对收到取消请求和完成取消的 UUID 均与当前导航目标相同。观察器最终 incomplete，
不得 observe/found。两场景通过，5 个 C++ 核心测试套件通过。

本次修改 navigation_executor.cpp：失效 generation 的迟到接受回调若携带句柄，
定向取消该句柄，避免 cancel() 发生在接受之前时遗留目标。此边界仅完成编译，
还需单独延迟接受响应的故障注入。未修改既有导航超时中 cancel_all 的历史分支。

隔离入口收到取消后继续 spin 2 秒以发送请求；测试另外核对服务器实际取消确认，
不能把入口进程正常退出等同于机器人已停。没有验证取消被拒绝、确认丢失或服务器
无响应的恢复，也未注入同时运行的其他目标。生产系统仍需独立取消确认和超时处理。
执行端转发仍由测试程序完成，不是正式运行连接。常用 install 未替换。
证据目录：training/validation/navigation-stop-20260913。

## 取消先于接受响应

--execute-search --stop-during late_accept：假 Nav2 扣住接受响应，测试等待 C++
入口发出 executor_cancel_called 确认后才放行。记录并断言顺序为：
executor_cancel_ack_before_acceptance → server_acceptance_released → server_cancel_received。
实际目标 UUID、取消请求 UUID、服务器完成取消 UUID 一致；没有 search_goal_bound、
observe 或 found。验证了 navigation_executor.cpp 对失效 generation 迟到句柄的取消。
96 项搜索单元测试仍通过。

确认消息只说明 cancel() 已调用，不等于已停止；最终结论另外依赖假服务器确认。
当前未绑定观察器会等待 search_timeout 后退出，尚未直接订阅请求取消通知；
这是未接通的退出优化，不应将超时退出描述为即时取消观察器。服务器延迟超过测试
入口 2 秒退出窗口的情况未覆盖，正式取消确认与拒绝/超时处理仍待实现。
证据目录：training/validation/late-accept-cancel-20260913。

## 观察器直接响应任务取消（更新上述等待超时的限制）

search_arrival_ros.py 新增 --cancel-topic，需同时给出非零 32 位十六进制 task-id。
订阅 HandymanMsg（reliable/transient_local/depth=1），仅处理 schema 和 task_id
匹配的 search_cancelled。延迟取消仍有效，不因消息旧而忽略；不提供发布者认证。
清空停稳窗口、视觉 adapter、位姿、暂存重试绑定和状态缓存，退出为 cancelled。
未绑定 UUID 时同样生效，不等待 search_timeout。该节点仍没有导航取消客户端，
机器人目标是否取消必须由执行器和服务器确认，不能凭观察器退出认定停止。

新增 4 项单元测试，总计 100 项搜索测试通过。整链 --stop-during late_accept
验证未绑定取消，--stop-during cancel 验证已绑定导航中取消；错误任务通知不退出，
正确通知退出为 cancelled，原目标取消链仍分别核对实际 UUID。未绑定场景测得约
0.25 秒（从测试发送 Task_failed 至观察到进程退出，包含轮询），不是实时性保证。
首次故障注入发布者 QoS 配错导致消息不送达，已修正并保留失败日志。
证据目录：training/validation/observer-task-cancel-20260913。

## 取消确认状态接口

NavigationExecutor 新增可选 setCancellationObserver 回调。启用后，对有句柄的定向
取消按 UUID 跟踪：cancel_requested → cancel_accepted_waiting_terminal → cancel_confirmed。
cancel_confirmed 只来自该 UUID 的 CANCELED 动作结果，不等同于 TF 已实测停稳。
目标以其他终态结束报告 goal_ended_otherwise；拒绝或未确认该 UUID 报告
cancel_rejected_or_not_acknowledged；1 秒内未收到终态报告 cancel_confirmation_timeout。
迟到的取消响应不会把已经超时的跟踪记录改成成功。

回调默认不安装，当前仅隔离入口启用；正式协调器尚未消费这些状态，也没有因超时
自动恢复、重发或急停。1 秒是当前诊断阈值，后续应配置化并基于实测调整。旧的
无句柄 cancel_all 超时分支不在此跟踪范围内。目标未接受前等待也没有独立确认时限，
直到迟到句柄出现后才开始定向取消计时。

故障注入：--execute-search --stop-during cancel --cancel-fault reject / no_terminal /
no_response，分别让假服务器拒绝、接受但不结束、将响应推迟 2.5 秒，均不误报完成。
no_response 是响应晚于确认时限，不是永久断网。服务器稍后可能真正取消，跟踪结果
仍保守保留超时。正常取消另做回归，核对同 UUID 的终态确认。
100 项搜索测试和 5 个 C++ 核心测试套件通过。隔离测试不能证明真实底盘已停止。
证据目录：training/validation/cancel-confirmation-20260913。

## 协调器自身导航的取消屏障（更新上述接口接入状态）

正式协调器现已安装取消状态回调，但仅覆盖它自身持有的 NavigationExecutor，
尚未接通独立搜索执行进程的取消状态。CancellationBarrier 在活动导航取消时锁定，
等待同 UUID 的 cancel_confirmed；接受取消请求不等于完成取消。锁定期间拒绝新的
Are_you_ready 握手、房间导航入口和搜索请求发布，不排队自动重放。
拒绝取消、其他终态或确认超时进入持久故障；迟到确认不解锁。

新增 cancellation_barrier.hpp，修改 coordinator_node.hpp/cpp，增加独立 C++
断言测试 training/tests/cancellation_barrier_test.cpp 及隔离 ROS 联调脚本
training/tests/run_coordinator_cancel_barrier.py。后者 --case confirmed / rejected /
timeout 在 domain 73、本机假 Nav2 下均通过：确认前不发送第二轮就绪；正常确认后
允许下一轮就绪；拒绝和超时后重复消息仍不能产生第二个目标。C++ 断言测试及
5 个核心 C++ 测试套件通过。证据：training/validation/coordinator-cancel-barrier-20260913。

仅编译到 /tmp/handyman-search-nav-install，常用 install 未替换；没有启动 Unity
测试或控制比赛机器人。故障尚无在线恢复入口，不能靠盲目重启认定旧目标已停止。
目标接受前的等待没有独立期限，Mission_complete 关闭时的取消排空及执行器内部
重试不在本次屏障验证范围。下一步接通跨进程搜索执行状态，再进行隔离故障测试。

## 跨进程取消：协调器接收端（2026-09-14）

启用 search.publish_requests 时，协调器在 search_cancelled 中新增随机 cancel_id，
发布前锁定独立搜索取消屏障。订阅 /handyman/search/execution_status，HandymanMsg，
reliable/volatile/depth=10；message 为 search_cancel_status，detail 为 YAML：

```yaml
schema: handyman-search-cancel-status-v1
task_id: "原请求的任务 ID"
cancel_id: "本次 search_cancelled 的取消 ID"
state: cancel_received
```

cancel_received 只说明收到请求，不解锁。cancel_drained 是执行进程对整个任务的
取消排空确认：不再提交新目标，所有已接受及迟到接受的目标均已终结，才能发布；
不能仅凭观察器退出、调用 cancel() 或单个目标的取消请求被接受而发送。
cancel_failed 或发出取消后 3 秒仍未排空将保持故障锁定；迟到确认不解锁。
时限使用 steady_clock，在状态接收及派发检查时判定，不依赖 Unity 仿真时钟。
错误 task_id/cancel_id、旧消息和畸形 YAML 不解锁。无 DDS 发布者身份认证，
该接口依赖可信执行进程，不提供对恶意伪造确认的防护。

本步只实现正式协调器的接收端及纯 C++ 屏障；测试进程模拟执行状态，未将真实
搜索执行器接成生产发布端。没有执行进程回应时会保守锁定，包括原来的只读消费端。
正常取消后也不自动重放被拦截的修正指令；排队恢复、关闭时排空、在线故障恢复
仍待实现。不得因此启用正式无人值守运行。日常 install 和 Unity 均未修改。

测试入口 training/tests/run_external_cancel_barrier.py --case confirmed / failed /
timeout，仅 domain 73、localhost，假 TF 位于厨房，不创建 Nav2 动作客户端。
覆盖错误任务、错误取消 ID、仅收到请求、畸形消息、正常排空、失败及超时后迟到确认。
首次脚本漏发 Task_failed 后的新 Environment 消息，导致解锁后仍不能就绪；
已补齐协议并保留初始失败记录。证据位于
training/validation/external-cancel-barrier-20260914。

## 执行器真实取消汇总回报（2026-09-14）

新增 search_cancel_reporter.hpp，并在隔离 search_binding_fixture 中通过
HANDYMAN_TEST_WORKER_STATUS 启用。--worker-status 的整链测试不再由 Python 转发
Task_failed 的导航取消：执行进程直接订阅 search_cancelled，按 task_id/cancel_id
回报 search_cancel_status，正式协调器消费回报并决定是否允许下一轮就绪。

NavigationExecutor 记录每个待返回接受响应，以及所有已接受、未收到结果的目标，
包括路线中间点和超时重试时脱离当前 goal_handle_ 的旧目标。
sealAndCancel() 永久禁止该实例开始新任务，停止原计时器并定向取消全部持有目标。
迟到接受仍按 UUID 取消；不能因当前句柄为空就认定完成。
cancellationDrainState() 只有在封闭、无活动计划、无待接受响应、无未终结目标、
无待确认取消且没有取消故障时返回 cancel_drained。拒绝、取消终态异常和确认超时
保守返回 cancel_failed，不会被后续终态清除。

Reporter 使用同一单线程 ROS 回调上下文；每个任务配一个执行器实例，不能跨任务
复用已封闭实例。所有者须安装逐目标 CancellationObserver（隔离入口已安装）。
收到合法取消后先回报 cancel_received，再每 50 ms 检查执行器；2 秒内没有排空
则回报持久 cancel_failed。重复取消不延长时限。最终状态重复发布以容忍短暂丢包，
不等于已验证真实底盘停稳。封闭接口之外的历史普通导航重试策略没有在此改写。

隔离构建入口改为 /tmp/handyman-drain-fixture-build/search_binding_fixture；旧构建
目录绑定 Windows 暂存源码，首次配置提示源目录不一致，故新建目录而非覆盖缓存。
测试使用真实 C++ NavigationExecutor 和假 Nav2，仍只允许 domain 73 / localhost。
日常 install 未替换，Unity 无需运行。生产任务进程管理、正常搜索完成后的取消
监听存活、多目标同时在途的故障注入、进程崩溃恢复、修正指令自动续接及真实停稳
验证尚未完成，不能把本节当作正式比赛搜索模块已上线。

测试命令在已有 Cyclone 隔离环境中运行：

```bash
python3 training/tests/run_coordinator_search_request.py --execute-search --stop-during cancel --worker-status
python3 training/tests/run_coordinator_search_request.py --execute-search --stop-during late_accept --worker-status
# 对第一条分别增加 --cancel-fault reject / no_terminal / no_response
```

核对实际目标 UUID、取消 UUID、任务 ID 和 cancel_id；接受响应放行之前不得排空。
正常确认后协调器允许第二次 I_am_ready，故障时保持一次，且不产生第二个导航目标。
证据目录：training/validation/worker-cancel-drain-20260914。

## 搜索完成后保留取消监听（2026-09-14）

本步修改的是隔离 search_binding_fixture 的生命周期，不是新增正式比赛进程管理器。
启用 reporter 时，导航完成不再立即退出；它继续 spin，供独立视觉观察器完成，
随后仍能响应同一任务的取消。旧的不启用 reporter 的测试模式保留原退出方式。
原 20 秒总 watchdog 仍仅用于测试，不能作为比赛任务的进程存活时限。

新增正常完成场景：run_coordinator_search_request.py --execute-search --worker-status。
假 Nav2 成功、只读观察器达到 found 后，断言执行进程仍存活；发送带合法 cancel_id
但错误 task_id 的取消，进程保持存活且不回报；再由 Task_failed 触发正确取消。
执行器已无未终结目标，应回报 cancel_drained，而不是再次取消已经完成的目标。
协调器允许第二次 I_am_ready，整轮仅一个导航目标。found 使用测试视觉消息，
不代表本次测试使用了 Unity 实时检测，也不触发 Object_grasped/Task_finished。

该场景仅验证取消协议可用性，不将 Task_failed 当作正常任务完成语义；真实正常
完成、进程崩溃/超时恢复以及修正指令续接仍需后续进程管理工作。
证据目录：training/validation/post-search-lifecycle-20260914。

## 进程退出与存活超时监视组件（2026-09-14）

新增 training/scripts/search_process_watchdog.py，纯 Python、无 ROS、无信号发送和
导航调用。一个 SearchProcessWatchdog 对应一个被所有者持有的子进程及 task_id。
使用 monotonic 存活期限；尚未通过所有者的任务释放流程就退出时，无论返回码为
0、非零还是信号退出，都记录 worker_exited_without_retirement。进程仍在运行但
期限已到时记录 worker_lifetime_exceeded。故障保持，不因后续采样或重复取消清除。

该组件不产生 cancel_drained。只有收到匹配 task_id、合法 cancel_id 的取消上下文
后，failure_report 才生成可送入既有协调器接口的 cancel_failed。无取消上下文时
仅供所有者读取故障，尚无自动通知协调器、发起取消或恢复的生产连接。存活期限
由调用者显式给定，单元测试/联调阈值不作为比赛超时配置。

新增 7 项 test_search_process_watchdog.py 单元测试。扩展
run_external_cancel_barrier.py --case process_exit / process_zero / process_timeout：
测试实际独立子进程退出 7、退出 0、以及仍存活但超时，使用监视器输出送入实际
协调器。校验错误任务取消不生成报告；正确故障报告后，即使再收到 cancel_drained
仍不进入第二轮就绪。超时测试还检查监视器未终止子进程；仅测试清理终止自己创建
的睡眠进程。没有 Nav2 目标，没有 Unity 或机器人控制，不证明真实机器人已停。

本步接入的是隔离测试父进程，尚不是正式搜索进程管理器。生产还需持有进程的
唯一所有者、明确的任务释放条件、故障到取消的触发流程及不能确认停机时的恢复
操作；禁止把重启工作进程或退出码 0 当作可以继续派发的依据。
证据目录：training/validation/process-watchdog-20260914。

## 内部故障主动触发取消（2026-09-14）

新增 search_process_supervisor.py：所有者将已持有子进程的 Watchdog 交给 ROS
适配器，每 100 ms 采样。尚无取消上下文时发布 HandymanMsg.message=search_worker_fault，
detail 使用 handyman-search-worker-fault-v1，携带 task_id、reason、returncode。
接到匹配 search_cancelled 后转为发布 cancel_failed。它不启动、重启、终止子进程，
也不直接访问 Nav2；正式启动器仍待接入，当前测试父进程实例化此适配器。

协调器在已有 execution_status 通道接收内部故障，校验 schema、当前 search_request_id
及 reason 白名单。先调用 SearchCancellationBarrier::latchFault，再用
invalidateSearchRequest 发布 worker_fault:<reason> 原因的取消，生成 cancel_id。
清除当前请求 ID 后重复故障不再产生第二份取消。无匹配任务、未知原因、畸形消息
不触发流程；DDS 发布者身份仍依赖可信运行环境，无新增身份认证。

这不是 Moderator 的 Task_failed，不冒充比赛消息，不发送 Give_up、Does_not_exist
或 Task_finished，也不自动改变比赛协议到下一轮。异常进程无法证明旧目标已终结，
因此保持故障锁定；即使后续回报 cancel_drained 也不自动继续。

联调新增 --case supervised_exit / supervised_zero / supervised_timeout：真实测试
子进程退出 7、提前退出 0、存活超时，经适配器使实际协调器主动取消。测试先核对
worker_fault 取消原因，再发 Moderator Task_failed 模拟状态重置，以免只因状态机
未就绪而误以为屏障生效。最终一次取消、一次 I_am_ready，无新搜索请求。
正常 confirmed 另做回归。增加 fault_notice 单元用例及 C++ 先锁定后确认断言。

范围：实际子进程 + ROS 监督适配器 + 实际协调器，仍无 Nav2 目标，不验证运动中
进程崩溃后底盘停止；下一步需要带在途目标的故障注入。日常 install、Unity 未修改。
证据目录：training/validation/supervised-cancel-20260914。

## 在途导航中的进程故障注入（2026-09-14）

扩展 run_coordinator_search_request.py：--execute-search --stop-during cancel
--worker-status --process-fault crash / deadline。仅允许 ROS domain 73 / localhost；
仍由真实 C++ NavigationExecutor 向假 Nav2 发送一个目标，等待观察器绑定其 UUID，
断言目标在执行，再注入故障。没有修改生产导航/协调器源码，仅扩展隔离测试。

crash 只对该测试 Popen 创建的 C++ 子进程调用 kill，模拟突然退出。父进程监督器
检测退出，通知协调器内部取消；观察器退出 cancelled，协调器保持锁定。测试特别
核对退出后假 Nav2 中同一 UUID 仍在执行、没有收到目标取消，也没有 cancel_drained。
这是危险残留的证据，不是停止成功；假服务器之后由测试自带 8 秒期限结束目标，
不能把该期限当作生产安全机制。

deadline 在目标已接受后启动 0.3 秒测试存活期限，不终止进程；故障触发内部取消，
活着的执行器收到通知后取消同一 UUID。执行器的真实排空回报与监督器故障报告可
同时存在，协调器必须保持故障锁定，不能靠后来的排空回报自动恢复。

两种场景均先由内部故障产生取消，再模拟 Moderator Task_failed 重置协议状态并
尝试就绪/新指令，断言没有第二个就绪及第二个目标，观察器不得 observe/found。
原正常取消另行回归。此处“在途”指假动作服务器保持执行，不是 Unity 底盘实测移动。

剩余关键问题：搜索进程已死时，目前监督器不会接管 Nav2 目标取消；需要独立于
该进程的、严格核对任务/UUID 的取消持有者及终态确认，不能用重启或解除锁定代替。
日常 install 与 Unity 未改动。证据：training/validation/inflight-worker-fault-20260914。

## 独立进程的单目标精确取消（2026-09-14）

新增 independent_goal_cancel.py，并为 SearchProcessSupervisor 增加可选
goal_canceller。监督端在目标派发前启动订阅，复用 validate_binding 核对任务、
搜索点、位姿、地图摘要、角色、generation 及接收时 2 秒新鲜度，保存第一个最终
搜索目标的非零 UUID。匹配故障取消通知到达后，使用
NavigateToPose 的 /_action/cancel_goal（指定 UUID，时间戳为零）及
/_action/get_result（同一 UUID），不依赖已退出工作进程的 GoalHandle。

必须同时收到成功接受该 UUID 的取消回复及该目标 CANCELED 结果才记录
goal_cancel_confirmed。拒绝/UUID 不符、其他结果、服务错误或 2 秒超时均不算成功；
迟到回复不能清除失败。结果只存在于诊断事件中，不发布 cancel_drained，也不解除
协调器故障锁定。监督器仍回报 cancel_failed，因为单个目标终结不是整任务排空。

硬限制：构造时仅允许 domain 73、localhost、/handyman_test/navigate_to_pose。
没有 cancel-all、发新目标或重启接口；生产启用未授权也未实现。只覆盖已记录的
一个最终搜索点 UUID，不覆盖路线中间点、第二个重试目标、接受前崩溃或记录丢失。
更多绑定只记为 additional_binding_not_covered，不声称覆盖全部遗留目标。

联调在原 crash 命令上增加 --independent-cancel；同一假服务器另放一个无关目标。
结束测试搜索子进程后核对目标 UUID、独立取消确认以及无关目标仍在执行，协调器
不进入第二次就绪。另增加 --cancel-fault reject / no_terminal / no_response 测试
拒绝、没有终态及响应超过期限。仅测试自己创建的进程与假动作服务器，无 Unity。

首次联调发现绑定订阅请求 transient_local，而既有发布端是 volatile，导致未接收
UUID；已改为 reliable/volatile，并保留失败日志。组件必须在派发前启动，不能用
晚启动代替目标登记；未记录到目标时记录 no_verified_goal，不猜测 UUID。
证据目录：training/validation/independent-goal-cancel-20260914。

## 中间路点和重试目标登记（2026-09-14）

NavigationExecutor 新增可选 OwnedGoalObserver。每个搜索动作接受回调在检查旧
generation 前报告实际 UUID、generation、派发时捕获的 pose 和是否中间路点。
因此正常最终目标、中间路点、重试和迟到接受均可登记；现有最终搜索点绑定接口
不改角色语义。新 observer 不随 cancel 清空，要求一个实例/观察器生命周期只属于
一个任务；生产协调器没有安装该发布回调，目前由隔离 fixture 发布 owned_goal_accepted。

消息 schema=handyman-owned-goal-v1，含 task_id、point_id、map_sha256、goal_id、
generation、stamp_ns、frame_id、role、pose。role 为 route_waypoint 或 final_search_point。
订阅先于派发，reliable/volatile/depth=100。OwnedGoalRegistry 逐一校验身份、2 秒
新鲜度、非零 UUID、唯一 generation、有限位姿及调用者提供的地图位姿白名单。
相同登记幂等，冲突/错误归属拒绝；允许较旧 generation 的迟到登记，最多 128 项。

RegisteredGoalsCanceller 对每个登记 UUID 查询 GetResult，确认已有终态的旧目标
记录 already_terminal。收到匹配取消后，对未确认终态的登记逐个使用独立 UUID
取消组件，不调用 cancel-all。取消后的 3 秒登记窗口内仍可处理迟到登记；更晚的
登记明确记为未覆盖。每个取消自身有 2 秒确认期限。结果只作逐目标诊断，始终
task_drained=false，既不证明登记完整，也不自动解锁协调器。

新增 run_owned_goal_registry.py --case waypoint / retry：使用真实 LayoutA 配置，
假 TF 和假 Nav2。waypoint 从 living_room 开始导航 kitchen，在中间路点保持执行；
retry 从 kitchen 开始，假服务器先 ABORT 第一个最终目标，再保持第二个重试目标。
登记到位后只结束测试创建的 worker，再由测试提供匹配取消上下文；前者取消中间
路点，后者保留两条 UUID 记录、跳过已确认 ABORTED 的旧目标、取消新的在途目标。
本次组件联调不带协调器，取消上下文由测试提供，不应描述成多目标正式整链已接通。

限制仍在：接受前崩溃或登记消息丢失时无法恢复未知 UUID；尚无持久化派发意图和
登记确认握手，尚未验证多个同时在途目标、迟到登记的 ROS 时序、最大容量故障及
地图切换。需要这些条件完善后才能声明整个任务排空。常用 install 与 Unity 未改。
证据目录：training/validation/owned-goal-registry-20260914。

## 多 UUID 管理器接入监督整链（2026-09-14）

run_owned_goal_registry.py 新增 --supervised：由实际协调器产生 search_requested
及 task_id，测试父进程创建 SearchProcessSupervisor，并将 RegisteredGoalsCanceller
作为 goal_canceller 注入。进程被测试结束后，监督器的内部故障通知使协调器生成
search_cancelled/cancel_id；管理器直接经监督器收到上下文，不再由测试直接调用
request_cancel。现有组件接口无需改动，本步修改的是隔离整链的组装与断言。

两种正常接管场景：--case waypoint --supervised，以及 --case retry --supervised。
前者确认中间路点被取消；后者保留已 ABORTED 和仍执行的两个 UUID，跳过前者，
只取消后者。错误任务的故障通知不触发取消。另对 retry 分别增加
--cancel-fault reject / no_terminal，验证拒绝及无终态不报告目标取消完成。

在内部取消发生后才发送 Moderator Task_failed、Environment、Are_you_ready? 和
新 Instruction，断言日志存在 Ready handshake blocked，只有一份搜索请求、一份
任务取消、一次 I_am_ready，且没有新增导航目标或比赛结果。所有进程异常场景仍
回报 cancel_failed；逐目标取消成功也不能自动放行。

测试使用假 TF/Nav2，不运行 Unity。为在协调器生成厨房请求后覆盖跨房间路点，
waypoint 场景在启动搜索 fixture 前将假 TF 从厨房设置到 living_room；这只是测试
布置，不代表机器人真实移动或正式导航闭环。未接入地图在线校验与视觉观察器，
此次整链指故障监督和取消链，而不是完整视觉搜索任务。生产启动器仍未启用。

尚未解决登记之前崩溃、登记丢失及持久化恢复。下一步应补登记可靠性/确认机制，
不能将本次已登记 UUID 的取消成功当作整个任务无未知目标。日常 install 未替换。
证据目录：training/validation/supervised-owned-goals-20260914。

## 登记确认与有限重发（2026-09-14）

RegisteredGoalsCanceller 在校验登记并写入内存表之后，通过 owned_goal_registered
回复 handyman-owned-goal-ack-v1，包含 task_id、goal_id 和收到的原始登记字符串
registration。合法重复登记也回 ACK；错误/过期登记不确认。这不是持久化收据。

隔离 fixture 在 HANDYMAN_TEST_REG_ACK 开关下缓存尚未确认的登记，每 100 ms
重发完全相同的内容，不刷新 stamp_ns；按任务、UUID 和原始内容精确匹配 ACK 后
删除缓存并停止重发。每条登记使用独立 steady_clock 1.5 秒期限，过期或内容被
篡改的 ACK 不清除记录。到期清理待确认项，并仅一次调用 nav.sealAndCancel，
禁止该执行器后续派发并取消其持有目标；不得据此宣称目标已停。

发送端仍只在隔离测试开关下启用，使用 raw 主题经测试转发器接到原登记主题。
日常运行版本不启用此握手。确认是接受后确认，不是派发前授权；确认等待期间
导航已经可能执行，接收者内存丢失、接受后尚未发登记就崩溃等空档仍未解决。

run_owned_goal_registry.py --case waypoint --supervised --registration-fault
drop_registration / drop_ack / wrong_ack，分别丢弃首条登记、首条确认、篡改首条
确认内容。断言重发内容完全相同、恢复确认后重发停止，随后原故障取消监督链仍
可运行。--case waypoint --registration-fault drop_all_ack 不带 --supervised：
丢弃全部确认，验证本地期限触发 sealAndCancel、同 UUID 到达 CANCELED，之后再发
迟到 ACK 仍不恢复确认。该超时场景是组件级测试，未将登记超时本身接入协调器
故障通知；测试之后结束 worker 并提供取消上下文，已终结目标不重复取消。

上述是受控消息丢弃，不等同于任意真实网络断连的证明。尚无持久化目标日志、
接受前目标 UUID 登记或启动恢复流程，不发送 task_drained 或自动解除故障锁定。
证据目录：training/validation/registration-ack-20260914。

### 登记超时主动通知协调器（2026-09-14）

补齐上一节的组件级缺口：隔离 fixture 登记期限到达后 sealAndCancel，并在
存活期间重复发布 goal_registration_timeout。实际协调器接受匹配当前 task_id
的该故障，先锁定再发布 search_cancelled；迟到 ACK、重复通知、协议重置均不解锁。
监督适配器同时修复先收到取消、后发现进程故障的顺序：tick 在故障发生后补做
已匹配 cancel_id 的目标取消，底层按 cancel_id 去重。

验证命令（使用 cyclone_test_env.py 的 domain 73 / localhost 隔离包装）：

```text
python3 training/tests/run_owned_goal_registry.py --case waypoint --supervised --registration-fault drop_all_ack
python3 training/tests/run_owned_goal_registry.py --case retry --supervised
ctest --test-dir /tmp/handyman-search-nav-build/handyman_rebuild_ros2 --output-on-failure
```

两项故障链测试通过；超时通知在测试结束 worker 之前已触发协调器取消及拒绝
新握手，当前导航 UUID 仅取消一次。正常重试保留旧 ABORTED 结果，仅取消新目标。
C++/launch 回归首次未使用该 DDS 包装，导航测试未收到 I_am_ready（6/7）；
随后使用隔离配置重跑 7/7 通过。保留初次失败日志，不将失败隐藏为全程通过。

证据：training/validation/registration-fault-20260914。源码修改前备份：
/home/crazylearner/handyman-tool-backups/before-registration-fault-oeq6_66w。
登记故障发送端仍是隔离 fixture，尚非正式搜索 worker；没有替换正式 install、
没有 Unity 实测，没有持久化恢复或自动解除锁定。监督端自身失联仍需后续防护。

### 监督端回应失联租约（2026-09-14，隔离开关）

HANDYMAN_TEST_SUPERVISOR_LEASE 在 fixture 开启任务级询问/回应。使用 steady_clock：
启动等待 2 秒，收到有效回应后租约 1 秒；每次询问递增 challenge，单次回应窗口
400 毫秒，检查周期 100 毫秒。只有匹配 task_id、schema、当前 challenge 且在
期限内的回应可续期；重复、错误任务、迟到回应无效。首次有效回应前不发导航。
此任务 ID 必须单次使用，不支持同 ID 重启恢复；此机制不是 DDS 身份认证。

租约超时永久 sealAndCancel，重复上报 supervisor_lease_timeout。协调器先锁定
再发任务取消；恢复回应不自动解锁。Python 监督适配器 lease_test 仅允许
domain 73 + localhost，且仅在自身 watchdog 无故障、未收到取消时回应。
正式 worker 尚未接入；阈值是隔离测试参数，不能直接视为 Unity 安全停止期限。

隔离运行 run_owned_goal_registry.py --case waypoint --supervised --lease-fault
loss / replay / wrong_task / missing，四项通过。前三项导航开始后分别丢弃回应、
重放旧回应、修改任务 ID：当前 UUID 仅取消一次并到达 CANCELED。missing 从
启动丢弃全部回应，目标数量始终为零。迟到有效消息后仍无新目标，协调器拒绝
新握手。测试模拟通信失联，未实际杀掉独立监督进程；监督适配器与故障注入器
仍在同一测试父进程，Nav2 和 TF 为模拟。未证明执行器卡死或 Nav2 失联时停止。

7 项 C++/launch 回归在隔离 DDS 下通过。证据目录：
training/validation/supervisor-lease-20260914。
修改前备份：/home/crazylearner/handyman-tool-backups/before-supervisor-lease-pfpfhydj。
没有修改 Unity、旧数据、正式 install 或远程 Git 提交。

### 独立监督进程 SIGKILL 验证（2026-09-14）

新增 training/tests/lease_supervisor_fixture.py：独立 ROS 进程实例化真实
SearchProcessSupervisor 适配器。测试父进程仍拥有 worker；监督进程通过只读
/proc/PID/stat 的 starttime 和状态观察它，避免把 PID 复用当作原 worker。
此探针不是生产进程管理器，不启动、停止或恢复 worker。

在隔离 DDS 包装下运行：

```text
python3 training/tests/run_owned_goal_registry.py --case waypoint --supervised --lease-fault process_crash
```

真实执行器开始目标、收到监督回应后，测试只 SIGKILL 自己 Popen 创建的监督
子进程，并确认退出码 -9。没有人工触发任务取消，也没有向执行器发停止命令：
执行器租约自行超时，精确取消当前 UUID 一次，假 Nav2 返回 CANCELED；执行器
仍活着，协调器收到 supervisor_lease_timeout 并拒绝下一次握手。迟到回应不
恢复派发。证据：training/validation/lease-process-crash-20260914。

本轮只改两个测试脚本和本说明，未改生产 C++ 或替换正式 install。
仍有测试父进程、假 Nav2/TF 和回应转发器；注册表留在父进程作观察，未触发
外部取消。此结果不代表执行器自身卡死、整机断电或 Nav2 失联时能实际停止。
尚需正式 worker 集成与 Unity 停止验证。原文件备份：
/home/crazylearner/handyman-tool-backups/before-lease-process-3mmm28pw。

### 租约公共组件提取（2026-09-14，正式接入的第一步）

检查 CMake 和 search_request_consumer.py 后确认：目前没有正式的搜索 worker
可直接接入；consumer 仍为只读地图/请求校验器。本轮不改变 actionable:false。

新增包公共头文件：
src/handyman_rebuild_ros2/include/handyman_rebuild_ros2/search_supervisor_lease.hpp。
SearchSupervisorLease 接收 node、NavigationExecutor、唯一任务 ID、三个显式
主题名；构造时验证 ID/非空主题。无测试域名称和环境开关内嵌。调用者必须在
同一个单线程 executor 中，在导航派发前检查 dispatchAllowed()，并保持 node、
nav、租约对象存活至任务明确退役。dispatchAllowed 和接收回应均立即检查
steady_clock 期限；超时 sealAndCancel，周期上报故障，不支持重置或复制。
析构并非退役协议，不能依赖析构确认导航停止。固定期限暂与既有测试一致。

fixture 删除自身租约实现，改为实例化公共组件；其隔离开关和主题映射仍留在
fixture。经已有 CMake include 安装规则进入隔离 install，源文件与已安装头文件
字节一致。process_crash、missing、replay 三项隔离回归通过；证据：
training/validation/lease-component-20260914。
源码备份：/home/crazylearner/handyman-tool-backups/before-lease-component-ciuyw57s。

这是组件提取，不是正式自动搜索闭环。登记/ACK 发送端仍在 fixture，后续须
统一封装目标登记与取消报告，建立受地图校验、任务取消和监督生命周期共同约束
的正式 worker 入口，再进行 Unity 验证。未启用生产导航、未改正式 install。

### 登记公共组件与取消联合判定（2026-09-14）

新增 include/handyman_rebuild_ros2/search_goal_registration.hpp，fixture 改用
SearchGoalRegistration，不再自行实现登记发布、ACK 校验、100ms 重发、1.5s
期限和失败上报。组件默认 require_ack=true；false 仅用于原有无 ACK 的兼容
测试，不可作为正式入口配置。任务 ID/地图摘要/主题在构造时校验；原始登记
内容不随重发变化，ACK 到达和健康查询都会先检查超时。对象独占 nav 的
OwnedGoalObserver 槽，析构解除回调；nav 必须比组件活得久，析构不是安全退役。

SearchCancelReporter 新增可选 protection_state 回调。fixture 将登记状态及
租约故障接入：登记失败/租约失败 -> cancel_failed；登记仍等待 -> 不报告
cancel_drained；登记确认完成且 nav drain 完成才允许成功报告。无回调维持
原接口兼容，但正式入口必须接入该联合判定。单线程使用约束不变。

隔离验证通过：

- drop_all_ack：实际 nav UUID 已 CANCELED，再将相同协调器取消上下文交给
  worker reporter，仍只报告 cancel_received/cancel_failed，绝不 cancel_drained。
- drop_ack：首次确认丢失后原内容重发、确认恢复，后续崩溃取消正常。
- run_coordinator_search_request.py --execute-search --stop-during cancel --worker-status：
  原兼容模式正常取消，cancel_drained 后第二次握手允许；不是开启所有防护的正常路径验证。
- 隔离 DDS 下 C++/launch 7 项回归通过。

证据：training/validation/registration-component-20260914；备份：
/home/crazylearner/handyman-tool-backups/before-registration-component-d4442y4k。
公共组件已安装到隔离 install，正式搜索入口仍未创建；还需强制组合防护、
地图门禁和任务生命周期，不能把本轮组件提取称为生产接入完成。

### 强制组合执行防护（2026-09-14）

新增 search_execution_guard.hpp：SearchExecutionGuard 构造时必建监督租约、
默认强制 ACK 的登记器、联合取消报告器，没有关闭其中一项的接口。ready 与
navigate 检查租约、登记健康和取消状态，每个实例最多派发一次搜索导航。
fixture 的 HANDYMAN_TEST_UNIFIED_GUARD 模式只使用这个组合，不再并行创建旧
单项组件。旧模式保留供已有测试回归，不是正式入口的关闭防护开关。

正常取消时，reporter 已 sealAndCancel 后才 retireAfterSeal 结束监督续期；
该操作不可恢复派发，也不会清除既有租约故障，避免监督端按协议停止回应后
把已正常取消的任务误判为失联。超时后的取消仍保持 cancel_failed。

隔离运行 run_owned_goal_registry.py --case waypoint --supervised --unified：
正常 ACK/续期、取消一次、cancel_drained 后等待超过原租约期限无故障，再次
握手成功。加 --lease-fault process_crash：独立监督 SIGKILL 后取消并保持锁定；
加 --lease-fault missing：没有监督回应，导航派发为零。三项均通过。
证据：training/validation/unified-guard-20260914；备份：
/home/crazylearner/handyman-tool-backups/before-unified-guard-0l82z5ma。

边界：这是进程内组合入口，仍非正式 worker 可执行程序。外部 nav 引用必须
不被调用方直接派发或替换 observer；调用方仍需保证传入的环境/房间/索引与
构造时 point_id 一致，尚未将配置目标绑定到类型接口。地图实时校验、实际
视觉搜索、进程生命周期及最终退役尚待正式入口接入。未启用 Unity/正式导航。

### 固定目标与地图门禁接口（2026-09-14）

SearchExecutionGuard 构造改用 Target{environment,room,point_id,index}，校验
point_id 为 environment/room/index。navigate 仅接受绑定与结果回调，不再允许
导航时更换环境、房间或索引，登记信息和导航目标共用同一固定 Target。

新增必填 MapCheck 回调，返回 pending/verified/revoked 及 task_id/point_id/digest。
verified 必须与本任务、固定点和登记摘要完全匹配；派发前 pending 不派发，
派发后失去 verified、明确 revoked、匹配错误或回调异常均 sealAndCancel 并
永久锁定。100ms 定时检查及派发前检查。协调器接受 map_verification_revoked
并按已有先锁定后取消机制处理。合法取消后停止新的地图检查，不把任务退役
误判为地图故障。MapCheck 必须非阻塞、保持实时状态，不能使用缓存常量充当
正式地图校验；此组件本身不读取 /map 或计算摘要。

隔离 fixture 注入地图状态：--unified --map-fault pending 验证零派发；
--unified --map-fault revoke 验证导航中撤销后一次取消、cancel_failed、拒绝
下次握手；--unified 正常路径回归。前两项测试为注入证据，不是实际地图内容
校验。初次 fixture 编译因 Humble 不支持泛型订阅回调失败，改为显式消息类型
后重建成功。

证据：training/validation/bound-map-guard-20260914；备份：
/home/crazylearner/handyman-tool-backups/before-bound-map-guard-deg6dae3。
仍需将现有 RequestMapGate 的真实校验状态跨进程接入，带 freshness/失联撤销，
再创建正式 worker 入口。不得把本轮门禁接口称为真实 /map 集成完成。

### 真实地图校验状态跨进程连接（2026-09-14，隔离话题）

新增 search_map_responder.py：使用现有 RequestConsumer/RequestMapGate，针对
每次地图询问重新检查请求期限、本地文件摘要及当前地图发布者图信息，只有
匹配任务、搜索点、地图摘要且 live_map_verified 才回应 verified；准备中回应
pending，已退役任务回应 revoked，异常不伪造成功。调用方仍负责订阅地图和
解码基准，地图栅格比较仍由现有 gate 完成。未修改只读 consumer 的默认行为。

新增 search_map_evidence_client.hpp：询问绑定递增 challenge，400ms 回应窗口，
只接收匹配当前任务/点/摘要的响应；已验证状态最多保留 1 秒 steady-clock，
超时永久 revoked，迟到消息不能恢复。初次未验证始终 pending、不允许导航。
guard 定时及派发前通过该客户端获取证据，不使用常量 verified。

隔离测试 --case waypoint --supervised --unified --live-map
normal/change/disconnect/silence 四项通过：真实解码文件地图，通过 ROS 发布
OccupancyGrid，再由 RequestMapGate 比较；修改一个栅格、销毁地图发布者、销毁
校验请求订阅分别触发一次精确取消、cancel_failed 和拒绝下次握手；正常取消
保持 cancel_drained、允许第二次握手。期望地图使用独立副本，不随注入地图变化。

证据：training/validation/live-map-evidence-20260914；备份：
/home/crazylearner/handyman-tool-backups/before-live-map-evidence-bj0ctrji。
边界：已连接真实校验逻辑到独立 C++ 进程，但地图话题仍是隔离 guard_map，
Nav2/TF 为模拟，地图校验响应器由测试父进程创建；尚未部署正式 worker 或
连接比赛 /map。silence 是停止回应测试，并未杀死独立地图校验进程。
状态新鲜不要求静态地图重复发布；依赖当前发布者图及最近已接收地图内容，
不认证每条地图的发送者，不能证明发布者内部正确或 costmap 一致。未改 Unity。

### 独立搜索导航 worker（2026-09-17）

新增 src/handyman_rebuild_ros2/src/search_worker_main.cpp，CMake 构建/安装
handyman_search_worker。默认 execution.enabled=false，退出码 2，尚未创建
NavigationExecutor。显式启用后要求 catalog/environment/room/task_id/point_id/
point_index/map_sha256，校验房间与索引存在，使用固定 Target、地图证据客户端、
强制登记 ACK、监督租约和取消报告。没有 fixture 的模拟地图开关。

每进程最多启动一个搜索点；导航完成仅记录结果并保留任务状态，绝不发送
Does_not_exist/Task_finished。lifetime_sec 默认 30（允许 3～600），到期先
sealAndCancel 并重复报告 worker_lifetime_exceeded，继续处理回调 3 秒后失败
退出。到期路径尚未专项验证；期限不是停止保证。正常退出/信号/异常也不是
退役证据，必须由外部监督负责；本轮未创建正式进程启动/退役调度器。

所有 topic 使用 /handyman/search/...，测试显式 remap 到 domain 73 的隔离
主题，navigation.action_name 设为 /handyman_test/navigate_to_pose。普通运行
不得仅设置 execution.enabled 后直接比赛：仍需上层地图校验器、登记接收者、
监督器、请求生命周期和视觉观察器。C++ 加载的环境配置与校验端的配置一致性
仍由启动器保证，尚未增加环境摘要绑定。

验证命令（cyclone_test_env.py 隔离包装）：

```text
python3 training/tests/run_owned_goal_registry.py --case waypoint --supervised --unified --live-map normal --runtime-worker
python3 training/tests/run_owned_goal_registry.py --case waypoint --supervised --unified --live-map change --runtime-worker
python3 training/tests/run_search_worker_startup.py
```

前两项通过：使用新安装的可执行程序，不再使用 fixture；正常取消后第二次
握手成功，地图变化时一次取消并保持锁定。启动测试默认禁用退出 2、缺少合法
索引拒绝退出 3。初次编译有缩进警告，修正后重编译成功。
证据：training/validation/search-worker-20260917；源码备份：
/home/crazylearner/handyman-tool-backups/before-search-worker-k2x9c5rq。
仅安装到 /tmp/handyman-search-nav-install；未替换正式 install，未启动 Unity，
未连接生产 Nav2，未进行视觉搜索或抓取。下一步应补任务调度/明确退役与视觉
结果衔接，而不是将当前单点导航 worker 称为完整自主搜索。

### 确认清理后的进程退役（2026-09-17）

新增 /handyman/search/retire 的 search_retire 握手（schema
handyman-search-retire-v1，task_id/cancel_id）。worker 仅在 reporter 已对相同
取消编号报告 cancel_drained、内部 nav 仍 drained、登记 settled、地图与租约
未故障、进程期限未到时接受；提前/错误任务/错误取消编号的请求不允许退出。
这是已取消任务的进程清理，不是视觉找到物体或比赛任务完成。

SearchProcessSupervisor 可选配置 retire_topic + 必填 retirement_check；测试
检查已登记 UUID 均有终态结果，收到匹配 cancel_drained 后，先在 watchdog
登记期待退出，再发送退役请求。重复请求不延长期限；2 秒内观察到退出码 0
才标记 retired。之前退出、非零退出、超时均仍故障；已有故障不能被迟到清理
消息洗掉。只确认当前已知目标，不引入持久化或消息发布者身份认证。

13 项纯 watchdog 单元测试通过。新 worker 的正常地图路径中，提前退出请求
被拒绝，随后正常取消产生 cancel_drained，worker 自行退出 0，watchdog.retired
为 true 且无故障，第二次握手成功；地图变化路径仍 cancel_failed 并锁定。
证据：training/validation/worker-retirement-20260917；备份：
/home/crazylearner/handyman-tool-backups/before-worker-retirement-vfmvy9ef。

本轮完成的是退役握手，不是自动调度器或视觉接入。当前由测试父进程启动
worker，正常退役不要求 kill/terminate；故障测试结束时仍由测试清理进程。
观察器本身及它的结果生命周期尚未纳入退役条件，下一步需把只读视觉观察器
与任务绑定后接入。未运行 Unity、未替换正式 install。

### 单点搜索生命周期接入（2026-09-17）

新增默认禁用的 `search_session_runtime.py`，管理地图证据、导航 worker、
实际只读视觉观察器及二者的清理。内部 `search_stop_requested` 只触发取消
屏障，不推进比赛抓取状态。运行器只有在观察器合法结束、worker 确认退役后
才转发 `cancel_drained` 并输出任务绑定的只读三维结果。故障立即停止租约，
独立撤销已登记目标，不依赖协调器回复；故障状态不自动重置。

复测入口：`training/tests/run_session_runtime_regression.py`。
命令、结果解读和限制见 [搜索运行器与故障验证](搜索运行器与故障验证.md)。
证据位于 `training/validation/session-runtime-20260917/verified-r2`。
特别注意：登记前崩溃仍可能留下未知目标；测试只证明故障锁定，不保证撤销。
这不是完整房间自动搜索或 Unity 实时验收完成，不得直接用于比赛。

### 派发前持久化确认与恢复（2026-09-17，后续修复）

上节的登记前窗口限制由新版本搜索保护器处理：拦截 Humble 客户端原始 SendGoal
请求，使用其真实 UUID，先发送 `owned_goal_intent`，监督端 SQLite 同步提交后
回复 `owned_goal_intent_recorded`，再放行原请求。没有确认就不发送。
接受与拒绝证据仍分别确认；不能把 UNKNOWN 查询结果当拒绝或终态。
取消未收到接受回复的目标时不再使用 cancel-all；根据已持久化 UUID 重试精确
取消，服务未知/延迟不伪造成功。所有由 SearchExecutionGuard 管理的中间点和
重试都经过该握手，普通导航调用不因此获得重启恢复保证。

运行器执行必须显式指定稳定的 `--journal` 路径；启动时发现未结束记录则进入
只恢复、禁止派发状态。即使收到全部终态也不会自动解除本轮恢复锁定。
新证据位于 `training/validation/dispatch-intent-20260917/verified-r3`，具体命令、
故障场景和存储要求见 [搜索运行器与故障验证](搜索运行器与故障验证.md)。
恢复不意味着机器人实际静止，也不覆盖日志被删除、底层存储失效、ROS 消息
伪造或 Nav2 终态记录丢失。Unity 仍未运行，正式 install 未替换。

持久化确认放行时重新核查地图和租约；每次运行的内部通道带独立随机运行编号，
避免同一任务编号重放时旧 worker 收到新运行器的续租或确认。
