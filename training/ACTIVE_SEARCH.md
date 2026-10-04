# 主动搜索实现进度

分支：`feat/handyman-active-search`。2026-09-19 第一批实现。

## 当前已实现（离线策略，不连接 ROS）

`scripts/active_search_policy.py` 提供：

- 环视方向生成：默认 8 个绝对朝向，45 度间隔；按实测 59.99 度水平视场，重叠 14.99 度。位置由调用方提供，尚不计算安全主搜索点。
- 每任务/地图/房间独立的候选、方向和区域记录。
- 指定类别过滤、低阈值候选保留、至少 3 个不同时间戳支持后才提出复核；同一视角拒绝重复和乱序帧。
- 有三维位置时按支撑区域和位置关联；同帧相邻实例不合并，关联歧义保守保留。无深度时仅在同一连续视角内按局部跟踪标识关联。
- 高分稳定候选优先复核，低分候选在待扫描方向完成后排序处理。候选分数使用上中位数而非单帧历史峰值，不解释成概率。
- 已排除候选不因同位置的普通重复检测而重新激活；未完成的环视不会因一次复核失败丢失。
- 遮挡、深度无效、导航失败记录为 deferred，而不是 rejected。同一复核视点不重复处理。
- 传感器无效、区域未覆盖、未决候选都阻止完成审查；环视结束不授权不存在。
- session 到期返回 incomplete；取消后不继续接收候选。

全部返回值均 `actionable=false`、`does_not_exist_authorized=false`。`review_completion` 只是请求上层审查，不是不存在结论；`found` 也不是抓取授权。

## 调用边界

上层必须验证消息身份、到达与稳定、图像新鲜度、RGBD 对齐、坐标统一及候选几何。`healthy` 不是此模块从原始相机计算的结果。连续跟踪 ID 不能跨断流重用。

`verify(..., confirmed)` 需要三维位置，但此模块不独立证明类别或姿态正确，必须由近距离确认适配器提供可靠结果。`mark_region(..., reviewed)` 也只能来自经过质量检查的区域覆盖模块。

默认低/高分阈值 0.15/0.8、关联半径 0.12 m 都是可配置实验初值，尚未经类别验证集标定。当前固定半径关联不能解决密集同类物体，歧义时可能多建候选，不允许据此宣称最终去重完成。

## 尚未接入

1. 安全主搜索点的在线 Nav2 路径、当前位姿与动态障碍验证（离线外接圆筛查已实现，见下节）。
2. 在现有导航/头部运行器中执行环视；绝不通过放宽旧请求身份和地图验证来接入。
3. 图像时间间隔采样、近距离多视角确认、方向候选补深度、带不确定度的跨视角关联。
4. 按新增覆盖/路径成本排序、备用视点生成及 deferred 候选重调度。
5. 遮挡覆盖的真实证据计算、日志恢复与 checkpoint、任务修正时缓存失效。
6. 模型常驻、相机服务生命周期协调、正式不存在授权。

当前策略没有运动副作用，不改变原单点搜索链路。开发和离线测试不需要 Unity；实测前另行通知恢复及剩余 session 秒数。

## 验证

```bash
cd ~/RoboCup-Japan-Open-2026
.pixi/envs/default/bin/python -m unittest discover -s training/tests -p 'test_active_search_policy.py'
```

测试包括重叠角度、无效输入、假高分候选回退、低分候选保留、跨视角关联、同帧相邻实例、重复帧、无深度、导航失败、无效观察、取消和 session 到期。

## 第二批：离线主搜索点生成

`scripts/build_active_search_points.py` 读取地图、房间边界及 Nav2 footprint，以机器人外接圆加 padding 和栅格半对角线进行保守障碍膨胀。未知格和地图外视为障碍；使用四连通路径，禁止斜穿墙角。不修改原 YAML 或地图。

候选评分由距房间包围盒中心距离、从配置初始位置出发的栅格路径长度、安全余量组成。中心只用于排序，中心有家具时选别处。每房间输出一个主点、最多两个相隔至少 1 m 的备用点，以及主点的八个绝对环视方向。

这是静态平面安全筛查，不是家具可见性优化；外接圆可能过于保守，未选出点不能证明房间不连通。地图 footprint 未表示的机械臂、头部及高处障碍仍需执行前检查。环视初始朝向目前指向房间包围盒中心，尚未按桌面排序。

```bash
cd ~/RoboCup-Japan-Open-2026
.pixi/envs/default/bin/python training/scripts/build_active_search_points.py \
  --repo . --output /tmp/active-search-points-new.json
```

输出文件必须不存在，避免覆盖已有结果。报告包含地图、环境 YAML 和 footprint 配置指纹，全部 `actionable=false`、`does_not_exist_authorized=false`。

2026-09-19 实际四套地图离线结果：16 个房间中 14 个提出候选；LayoutB/lobby 和 LayoutC/kitchen 未产生保守候选，符合此前暂不测试这两个房间的范围，但不据此授权不存在。LayoutA 客厅主点约 `(2.755, 2.733)`，8 个环视朝向；尚未现场导航验证。结果存于 `training/validation/active-search-points-20260919/points.json`。

新增测试覆盖：地图边界、障碍物旋转余量、斜角不可穿越、中心被占用、分离房间、无效起点、旋转地图坐标和无效分辨率。

## 第三批：隔离环视执行链路

`tests/run_search_session_runtime.py --active-ring` 将生成器提出的 LayoutA 厨房主点及八个朝向写入测试专用临时 package/config。通过临时 ament 索引让协调器、请求验证器和 C++ worker 读取同一份配置；不改项目原 YAML、安装目录或比赛环境。

使用真实 coordinator/runtime/observer/worker 与头部稳定和停止确认逻辑，TF、Nav2、RGBD 和关节反馈是合成的。每个方向有独立 goal UUID 和注册记录；上一组清理完才进入下一组。八组共享同一 session 截止时间，不在组间续期。模拟初始位姿直接在搜索点，所以此测试不证明从入口导航到主点成功。

运行器只对隔离多点模式允许 session budget；现场 `--allow-live --multi-point` 仍拒绝。不能将临时测试配置当作现场地图部署方法。当前真实执行链路仍是负证据续扫，没有接入 ActiveSearch 候选列表驱动的接近与复核。

测试入口（须经过项目 Cyclone wrapper，domain 73、localhost only）：

```bash
python3 training/tests/run_search_session_runtime.py --case sequence \
  --multi-point --auto-head --session-budget-seconds 160 \
  --exhaust-points --active-ring
```

把 `--exhaust-points` 改为 `--cancel-at-transition` 可测试第一组结束切换时取消，要求不得发送第二组导航。

首轮测试发现：图像回调先推进到 observation_timeout，再调用 ViewEvidence 时会清空刚才有效的未检出帧。修复仅保留截止前已有证据，不计入截止后的图像，不延长证据有效期；其他失败、数据无效和过期仍不允许续扫。不得回填旧现场记录或把之前未完成的点 3 记录改成通过。

验证结果：442 项 Python 单元测试通过；八向全程隔离测试通过（8 个独立导航目标、8 次头部动作与8次保持停止），末端仍为 incomplete/coverage_unverified，未授权不存在。第一组切换取消测试通过，只发送一个导航目标，无搜索结果。证据归档 `training/validation/active-ring-20260919/`；包含首轮失败记录，未将其算作通过。模拟导航立即更新位姿，不能用本次总耗时估计真实转向速度或比赛耗时。

## 第四批：任务配置包与只读路径检查入口

`prepare_active_search.py` 在新目录创建地图/config 副本，把指定房间的八个环视位姿写入副本，并使用现有请求验证器核对任务和配置一致性。保留原始环境和地图；仅允许当前六类模型支持的目标。地图、环境 YAML 或 footprint 与生成报告不一致时拒绝；输出已存在时拒绝覆盖。

```bash
.pixi/envs/default/bin/python training/scripts/prepare_active_search.py \
  --repo . --output /tmp/active-search-layouta-living-new \
  --layout LayoutA --room living_room --target canned_juice
```

`manifest.json` 保存 task ID、目标、房间、八点请求、每个副本文件校验和，以及原始地图和 footprint 指纹。这是防止意外串任务或文件漂移的完整性检查，不是签名或运动授权。失败时可能留下不完整输出目录；保留诊断，重试使用新目录。

下一步在 Unity 恢复且现有地图定位/规划服务正常时，可以通过 domain 71 wrapper 运行下列**只读**路径检查（不会发 NavigateToPose 或速度指令）：

```bash
python3 training/scripts/probe_search_path.py \
  --active-bundle /tmp/active-search-layouta-living-new/manifest.json \
  --output /tmp/active-search-path-new.json
```

规划使用 Nav2 当前机器人 TF 作起点；要求 action 成功、map 坐标系、有限路径坐标且末端位置误差不超过 0.15 m。查询前后验证配置包未变化。输出绑定 task ID、配置包摘要和地图摘要，但尚未证明运行中的地图与摘要一致，`live_map_identity_verified=false`、`motion_authorized=false`。规划耗时限制仅限制只读规划请求，不恢复旧导航短超时。

尚不能运行现场完整环视：还需将实时地图验证、持久化目标归属和多点执行入口绑定；当前 `--allow-live --multi-point` 禁止条件未移除。配置包本身也不能作为执行许可重复使用。

## 第五批：运行器任务绑定

`active_run_binding.py` 与运行器 `--active-bundle` 绑定配置包的 task ID、完整请求（只允许传输时间戳不同）、package 路径和地图摘要。运行器首次接收请求、每次启动 worker、切换方向和输出结果时执行对应绑定检查。文件完整性与 footprint 源配置发生变化会失败，原 RequestMapGate 仍负责实时地图内容、发布者和配置的持续检查。

同一 binding 实例只接受一次请求。运行器还会在复用的 DispatchJournal 中事务写入 `active_requests(task PRIMARY KEY, bundle_sha256)`，写入失败不放行。同一 task 在进程重启后仍拒绝；成功、失败和取消均不删除消费记录。必须复用原 journal，不能换数据库或删记录绕过此约束。合法新任务应重新生成配置包和 task ID。

私有请求入口新增只读配置包预检：

```bash
python3 training/scripts/single_search_request.py \
  --package-share /tmp/active-search-layouta-living-20260919-01/package \
  --active-bundle /tmp/active-search-layouta-living-20260919-01/manifest.json \
  --point-index 0 --output /tmp/unused-active-preflight.jsonl
```

`--run` 配合 active-bundle 仍明确拒绝，运行器现场多点限制也仍保留。绑定模式的隔离端到端验证见下节；不代表现场运动已经通过。

## 第六批：带绑定的隔离执行与重启防重放

测试新增 `--bound-ring`：使用真实 `prepare_active_search.snapshot()` 生成配置包，再由测试私有 owner 发送其精确请求。该模式不启动比赛 coordinator，不伪称测试了比赛协议；runtime、worker、observer、binding 和 SQLite 消费记录是真实实现，Nav2/TF/RGBD/头部反馈是合成的。

```bash
python3 training/tests/run_search_session_runtime.py --case sequence \
  --multi-point --auto-head --session-budget-seconds 160 \
  --active-ring --bound-ring --exhaust-points --replay-bound
```

`--replay-bound` 在第一轮结束后真正退出旧运行器进程，再启动新进程、复用 journal，重新发送带新时间戳的旧任务。要求得到 `active_request_previously_consumed`，没有新的 worker 或导航目标。不能用拒绝过期时间戳代替持久化防重放验证。

取消测试改用 `--cancel-at-transition`，不带 `--exhaust-points --replay-bound`；检查取消后无第二个导航。文件变更的单元测试保留，现场执行仍未开放。

2026-09-19 验证结果：466 项 Python 单元测试通过；带绑定的 8 向全程、运行器重启后拒绝旧任务、第一组切换取消三个检查通过。重放使用新时间戳，仍被数据库消费记录拒绝，新增导航为 0。取消测试只产生 1 个导航目标且无搜索结果。归档 `training/validation/bound-ring-20260919/`。

## 第七批：显式现场入口（当前接口，以本节为准）

新增 `--enable-active-ring`，不带此开关仍禁止现场多点模式。开关要求：domain 71、localhost-only、`--allow-live --multi-point`、绑定配置包、point-index 0、明确 head-tilt/view-seconds/session-budget，以及既定 `/handyman/manual_single_search` 私有请求/状态/结果话题。不会发送比赛答案或抓取命令。

收到精确绑定请求并消费任务后，运行器在线程池中发起只读 ComputePathToPose（当前位置 TF），验证结果归属，再解码配置地图；原 RequestMapGate 验证实时地图内容与唯一发布者后才启动 observer/worker。路径失败、配置变化或地图不一致均不放行。路径检查仅发生在主点接近前，各组仍保留原 Nav2 和地图检查；它不是动态障碍和当前机械臂外形的完整安全证明。

运行器增加 `--check-current-path` 供 domain 73 隔离测试走相同前置规划路径。bound-ring 测试提供模拟 ComputePathToPose 服务，验证请求使用 `use_start=false`；不把模拟路径当现场可达证据。

请求入口允许 `--active-bundle ... --enable-active-ring --run --confirm-motion`，结果按绑定任务、地图摘要和八点范围检查，不再仅接收第一个点的结果。仍必须由独立运行器负责导航注册和取消清理。

相机入口新增 `--session-budget-seconds N`，替代固定短采集时长；相机节点从启动起计算绝对截止时刻，模型加载也占用预算，模型就绪后不续期。旧 `--seconds` 模式仍保留 120 秒上限。运行器与相机各自使用启动时的真实剩余时间，不能沿用过去 session 的 N。相机提前失效不得解释为目标不存在。

现场运行顺序：

1. Unity 恢复，确认是否重启及当前剩余时间；新 session 重新对齐定位，确认头部仅一个订阅者。
2. 启动相机 session 模式，等待新鲜 RGBD；确认当前地图与 Layout 匹配。
3. 启动原导航控制服务和绑定运行器，使用持久化 live journal，不换路径绕过旧任务。
4. 私有请求入口发送同一配置包任务，运行器自行重新做当前路径检查。
5. 停止确认后归档结果。八个方向未找到仍不代表整个房间不存在目标。

本版本尚未接入候选列表驱动的自动接近、盲区补看或家具覆盖，首次现场环视只验证执行链路。不要将本节视为完整视觉搜索阶段验收。

本轮验证：474 项单元测试通过；带当前路径检查的完整八向隔离执行、重启拒绝旧任务、组间取消均通过。相机 300 秒 session 模式完成只读配置预检，尚未进行持续 300 秒的现场运行验证。证据归档 `training/validation/live-entry-20260919/`。

## 8. 首次现场环视与相机子进程寿命修复

现场任务 `ce2c3f569af8482da6ce6c007799004e` 成功到达新主搜索点，前三方向观察后自动切换，第四方向因无有效观察证据结束，清理已确认，未授权 Does_not_exist。证据在 `training/validation/active-ring-live-20260919/`。这不是完整八向通过。

发现 GPU worker 仍有预热后 300 秒的旧限制，而 ROS 相机父进程使用 470 秒预算。最后 RGB 时间戳早于第四方向到达时间，随后相机停止；该子进程限制与停止时间吻合。现将父进程的绝对 monotonic 截止时刻传给 GPU worker，共享预算、预热不续期；独立旧模式保留 300 秒限制。新增 worker-exit.json 和父进程 stop_reason，明确区分预算结束、推理超时、子进程退出与中断。

新增五项离线寿命回归检查：旧模式、超过旧 300 秒限制、加载不续期、过期不续期、非法值。仍需现场重测完整八向与超过 300 秒的相机持续运行。未改 Unity、原地图、检测阈值和旧数据；旧任务 ID 已消耗，重测必须生成新绑定包，不能清除 journal 重用旧任务。

## 9. 第四方向观察证据与有限补采

第二轮现场 `7500211c49704b8eb9b21523a14ebf05` 相机仍运行，但第四方向再次结束。日志确认 TF 间隔 0.302 秒触发重新稳定并重置证据；恢复后的完整三秒窗口收到四帧可用图像。旧规则因接收间隔 0.574、0.599 秒分别重新累计，最后两帧又在结束时因超过 0.5 秒被隐藏。相机子进程寿命不是本次原因。

修复后，在同一稳定观察阶段内累计经过适配器验证的不同新帧，不再因接收间隔单独清空。`current_data_fresh` 独立报告当前新鲜度；`view_data_usable` 表示本阶段累计至少三帧有效未检出记录，仍不是房间覆盖或不存在证明。无效数据、重复/逆序帧、重新到位会清空证据；现有运动、TF 和头部检查保持不变。

三秒正常观察窗口结束时，已有三帧则正常结束；不足则最多补采三秒，达到要求即可结束。总 session 截止优先，补采不续期。该可选策略仅由 ROS 观察器绑定，原离线 SearchScan 默认行为不变。新增八项回归包括真实四帧间隔、补采成功/耗尽、总截止、取消、无效帧、新到位和超时后帧。487 项单元测试通过；现场完整八向尚待复测。

## 10. TF 低频接收与稳定窗口

第三轮 `85d86ef9bb144915ba47860be169e993` 导航成功，但第一方向在 TF 重新稳定阶段结束。恢复窗口仍收到样本，间隔约 0.18～0.42 秒；原 0.3 秒阈值反复清空窗口。日志不能区分 Unity 发送端与传输端卡顿。

现场 ROS 观察器显式使用 0.5 秒 TF 间隔上限，与原到位检查和数据年龄上限一致；其他调用默认仍为 0.3 秒。接收与传感器时间间隔均检查。滑动窗口至少保留六帧且覆盖一秒，避免低频情况下只保留约一秒却永远不足六帧。运动速度、位姿范围、重复/逆序、过期数据、四秒恢复期限和总 session 截止均未取消。

493 项单元测试通过。新增测试使用真实接收间隔配合合成静止位姿，不是现场位姿回放。隔离 TF 中断恢复测试通过（重新稳定、重新取图、同一个导航目标、正常清理），归档 `training/validation/tf-cadence-20260919/search-session-runtime-kq0jljuc`。此前完整八向隔离与旧任务重放拒绝通过，归档 `training/validation/view-supplement-20260919/search-session-runtime-m2ze2e04`。现场八向仍待复测，不代表视觉搜索阶段完成。

## 11. 集成头部反馈时限一致性

第四轮首方向反馈年龄约 0.31～0.33 秒触发旧 0.3 秒限制。日志显示到位后因无效反馈被误命名为 head_changed_after_ready；hold 发出后约 4.79 秒才获得停止确认，任务整体清理失败，不应改写为成功。

集成 HeadStage、StopConfirmation 和独立观察器共同采用最大 0.5 秒数据年龄/间隔；独立 head_view_trial 默认仍为 0.3 秒。未来时间戳上限 20 ms、角度容差、0.6 秒稳定窗口、hold 后新反馈要求、2.5 秒取消清理期限均保持。失效反馈单独标记 head_feedback_invalid_after_ready，不再一概称为角度变化。

498 项单元测试通过。上一轮取消前 50 帧真实角度/时间戳回放：旧规则 6 帧无效，新规则 0 帧无效，最终稳定。隔离头部+TF恢复+清理测试通过，归档 `training/validation/head-latency-20260919/search-session-runtime-kn9bkte_`；不是现场八向验收。

## 12. 首次现场完整八向完成

任务 `7e0713eabd84462a9b8a2619a02a515f` 在 LayoutA 客厅、同一 session 恢复后从搜索点附近执行，约 133 秒完成八向流程及清理。各方向有效未检出帧数依次为 17、18、17、22、21、17、17、19；八个方向均有 head-stop.json，未触发 TF 恢复，最终 cleanup_verified=true。归档 `training/validation/active-ring-live-20260919/active-ring-live-20260919-05` 与对应 owner 日志。

本轮未确认 canned_juice，未授权 Does_not_exist，未接近或抓取物品。通过的是该次八向执行与观察链路，不是整个房间覆盖、模型召回率或第四阶段完整验收；也不是从初始位姿导航的重复验证。下一步依据实际视图检查距离与遮挡，接入补看及候选复核。

## 13. 目标房间归属过滤

用户确认：白色高柜上的绿色饮料瓶属于 lobby，不能作为 living_room 任务目标。集成运行器现在从已绑定环境读取所有房间多边形，传给独立观察器。目标确认前，以该帧深度时间戳查询 map←odom，将有效三维可见表面位置转换到 map，再判定房间；不是用机器人所在房间代替物品归属。

只有唯一落在指定房间内部且距房间边界超过 0.15 米的候选可以参与三帧找到确认。边界、重叠、区域外和变换不可用均不确认；明确其他房间输出 outside_target_room。0.15 米是保守边界余量，不是定位误差的统计保证。结果记录 room_membership 和地图坐标；依旧不是抓取授权。

低置信度或几何不稳定候选仍先由原检测质量规则拒绝，不伪造其房间归属。其他房间或归属未明的目标不能变成目标房间的不存在证据。本次没有实现候选自动接近、家具补看或边界候选持久队列；拒绝原因已记录，后续调度必须消费这些状态。独立未提供房间参数的旧诊断入口保持原行为，集成 search_session_runtime 始终传入房间约束。

新增11项离线测试覆盖跨房间、边界、重叠、坐标旋转/平移、无效变换、正确目标确认及缺失TF。四套实际地图、16个房间配置加载通过。旧隔离集成测试的合成目标坐标不是房间内保证，后续正例集成测试需使用指定房间内位置，不能删掉房间约束让旧假数据通过。尚未进行带房间约束的现场复测。

## 14. 房间归属端到端隔离验证（2026-09-20）

运行器测试新增 `--room-case inside/lobby/boundary`。使用 LayoutA 实际房间多边形、真实 coordinator/runtime/worker/observer，合成 TF、导航、头部反馈和 apple 检测，域73；不是 Unity 检测准确率测试。目标位置根据真实区域选择，替换旧的任意 `[1,2,3]` 正例坐标。

每场景连续执行两任务，检查任务切换、房间归属记录、最终结果及停止确认。inside 应 found 且 room_membership.target_room=living_room；lobby 必须出现 outside_target_room 且不能 found；boundary 必须出现 boundary_uncertain 且不能 found。所有情况不得授权 Does_not_exist。

当前跨房间/边界候选会让该观察最终 incomplete，而不是当作“房间已搜索完”。后续需要将这些结果交给补看调度，不能把本次通过理解为自动补看已实现。运行命令：`python3 training/tests/run_search_session_runtime.py --case sequence --auto-head --room-case inside`，须先使用域73隔离启动环境；另外两场景替换最后参数。

三场景各两轮全部通过，509项单元测试通过。归档 `training/validation/room-integration-20260920/`：inside=`search-session-runtime-on_9b4i4`、lobby=`search-session-runtime-ftgnuke5`、boundary=`search-session-runtime-vpq6wbfd`。测试传输发现 WSL 无法读取 D 盘暂存文件（Invalid argument），改用 `C:\Users\wpb15\Downloads` 暂存；未尝试修复或重新挂载 D 盘。

## 15. 房外候选继续与有限换点补看

ViewEvidence 新增 continuation_stamps、outside_room_stamps。经原质量检查及明确房间归属过滤的房外目标帧，可用于当前视角结束后继续下一视角，但不写入 negative_stamps。需要三帧，且整个视图数据可用；边界不明、缺失变换、不稳定候选或错误图像仍不能放行。SearchPointSequence 仅接受显式 view_only_not_room_absence 范围，保持不存在授权为 false。

prepare_active_search.py 新增 `--supplement-points 0/1/2`（默认0保持旧行为）。主点八向后附加所选备用点的八向，复用同一任务、地图绑定、session截止、导航验证及逐点停止确认。备用点来自原保守地图膨胀/连通候选，须唯一落在目标房间内部、距边界超过0.15米、与已选点至少1米；最多两个备用点，每个点只访问一圈，找到目标则提前结束。示例：`python3 training/scripts/prepare_active_search.py --repo ~/RoboCup-Japan-Open-2026 --output /tmp/new-unique-bundle --layout LayoutA --room living_room --target canned_juice --supplement-points 1`。

这是有限的空间换点补看，不是家具表面的可见性或信息增益规划。它不能保证消除全部盲区，也不能证明不存在。尚未实现按家具高度调整俯仰、动态跳过已覆盖表面、边界候选复核队列；无有效补看点则只保留主点。路径现场是否可达仍由当前地图门控与Nav2验证，生成配置不是运动授权。

512项单元测试通过。已生成客厅16视角配置 `/tmp/active-search-living-supplement-20260920-01/manifest.json`，未启动现场运动。带房外目标的16视角隔离集成测试另行记录结果。

隔离集成 `/tmp/search-session-runtime-n2063ead` 已通过：LayoutA kitchen任务持续收到lobby中的合成apple，完成16次导航/头部观察、进入备用点、最终清理确认，未误报找到或不存在。使用真实执行程序、合成传感器/导航，不是客厅现场覆盖验收。归档 `training/validation/supplement-dispatch-20260920/`。

## 16. 搜索点原地转向（2026-09-20）

现场番茄酱任务 `fa002de6647f47bfaecfe9cfed87cbd3` 中，方向3→4用了71.86秒，其他相邻方向约6.86～7.47秒。控制器在该段两次报 Failed to make progress，分别约25秒后重试；原搜索点XY容差0.10米，第四方向开始时日志四舍五入位置误差约0.104米。重复位置纠偏是有力嫌疑，但旧日志没有完整速度轨迹，不能声称已逐次还原摇晃原因。

新入口保留 NavigateToPose 的目标身份、登记、租约、取消与地图校验，但同点切换使用独立 `search_turn.xml`，不调用 ComputePathToPose/FollowPath，也不执行平移进度恢复。运行器只有在上一观察完整退休、前后XY相同且为正向不超过90度的换向时才选择该树。初始导航及备用搜索点移动仍使用原导航树；第三阶段配置没有改动。

启用时在原有、已绑定的多点运行器命令上增加：

```bash
--turn-behavior-tree /tmp/handyman-search-nav-install/handyman_rebuild_ros2/share/handyman_rebuild_ros2/behavior_trees/search_turn.xml
```

需先重新编译并启动新版 `phase4_search_control.launch.py`，它加载 `handyman_search_turn_bt` 和独立 `search_spin` 行为插件。不开启该参数则保留旧路径，便于对照；不要对旧/已消费任务重复发请求。新测试应准备新的唯一任务bundle，使用原会话截止时间，不续期。

`PrepareSearchTurn` 在执行时核对实时 map→base_footprint：反馈年龄≤0.5秒、位置距目标≤0.14米、转角≤1.35rad。角度使用最短有符号差，跨±π不会绕远路。Humble原生Spin BT在构造时读取角度，因此使用执行时读取角度的 `SearchSpinAction`，避免黑板尚未赋值或后续方向沿用旧角度。参考上游源码：<https://github.com/ros-navigation/navigation2/blob/humble/nav2_behavior_tree/plugins/action/spin_action.cpp>。

`SearchSpin` 保留Nav2行为生命周期、精确取消和碰撞检查器；只发布角速度，最大0.45rad/s、加速上限0.8rad/s²，提前减速，约2度内停止，不反向微调。每周期检查剩余转角（另加0.05rad余量）的完整扫掠轮廓；障碍/无有效碰撞检查、TF过期、单周期超过0.5秒、平移漂移超过3厘米或过大过冲均停止失败，不作不存在判断。正常到位后的独立静止/头部/视觉证据校验不变。原生Spin的安全生命周期被复用，速度与扫掠计算为搜索专用实现，并非只改原生Spin的参数。

离线验证：517项Python单元测试、转向速度曲线/BT门控的两个C++测试通过。实际Nav2+合成TF/代价地图测试 `run_search_turn_nav2.py` 的方向3→4、跨π转向均成功，约3.45秒（含末端额外0.25秒测试等待），无反向速度；取消、障碍、TF中断、超出位置容差均正确停止。证据 `/tmp/search-turn-nav2-vh15h1zs`。这是合成反馈下的约3.2秒转身，不代表Unity实测，也不包含头部准备、静止校验、图像观察、逐点进程切换时间。

隔离调度回归使用 `run_search_session_runtime.py --case sequence --multi-point --exhaust-points --auto-head --active-ring --bound-ring --supplement-points 1 --outside-room-ring --turn-only --session-budget-seconds 260`；它使用假导航，只验证每个目标选择了正确的行为树，不能替代上面的实际Nav2旋转测试或Unity现场验收。

最终回归记录：上述16方向调度已通过，初始点/备用点走正常导航，其余14个同点换向选择转向树；停止清理验证通过，未误报房外目标或不存在。原导航规划、导航往返及两个转向C++测试也通过。证据归档 `training/validation/search-turn-20260920/`。

派发前增加位置回退：若当前TF位置距目标超过0.14米，则该次目标使用原导航树回位，而非强行旋转；TF缺失则不派发。转向树执行时仍独立复查新鲜TF与距离，防止派发后的状态变化。转向过程出现漂移/障碍/反馈故障则停止，不把失败改写成找到或不存在。Unity现场转速、延迟与到位稳定性尚待验证。

## 17. 转向现场验证与启动竞争修复（2026-09-20）

首轮因关节时间戳落后约1.05秒而被头部门控拒绝，没有旋转。停止Unity后重启桥接，新Play实测反馈时差恢复到约-17～19毫秒。桥接进程的同步计数不是每次Unity重连都重置，后续全新Play前应检查时钟同步，不能放宽头部阈值绕过。

第二轮在新worker尚未收到TF时提前返回失败；这是新增位置检查的启动竞争。修复为100ms异步重试，保留原15秒启动就绪期限、任务总截止与取消机制；收到TF后才登记派发目标。导航最终失败明确上报search_worker_fault，不再仅写日志等待会话结束。隔离测试新增`--turn-tf-delay`，每次新方向的TF故意延后1.5秒，八向流程、正确树选择及清理全部通过，证据`training/validation/search-turn-startup-20260920/search-session-runtime-_mfz8r87`。原导航及转向四项CTest回归通过。

第三轮现场任务`f6a6671fe0fc4ee8ae4fd44526794062`成功：LayoutA living_room，第六方向确认filled_ketchup且room_membership=inside，cleanup_verified=true。五次search_spin动作耗时依次2.701、2.950、3.000、3.000、2.900秒，不包含静止/头部/视觉等待。方向3→4的完整速度与姿态片段：62条角速度命令，无负向命令，上限0.45rad/s，26条姿态样本，实测转角约44.75度，没有超过0.002rad的反向步进。

录包在读取进行中的SQLite数据库时发生database-is-locked异常退出，后两次旋转只有行为服务器成功与到位/观察日志，缺少完整速度轨迹；不能将缺失数据解释为零速度或无摇晃。后续必须先停止录包再读取数据库。本轮运行器与目标确认不受录包退出影响。现场证据归档`training/validation/search-turn-live-20260920-03/`，转向改动仍未提交或推送。
