Session1 自动化核心，2026-11-14

最新任务调整：用户已明确玩具狗在客厅 white_side_table。
本轮使用 known_source_task.json 的 source_table 创建 Session1 实例：
  Session1(session_id, source_table='white_side_table#living_room')
流程为进入客厅 -> 导航取物桌 -> 现场观察定位 -> 抓取 -> 收臂 ->
导航方形矮桌 -> 放置 -> 核验 -> 等待判分，跳过 search 环视阶段。
导航家具位置可使用地图，但不能把家具坐标当作玩具狗精确抓取坐标。
若桌面上未识别到玩具狗，保留未完成结果；本轮不自动启动房间环视。
known_source_task.json 是交接配置，真实执行适配器仍待接入。
离线回放可传 --source-table 'white_side_table#living_room'，回放事件需含
navigate_source（arrived/room/source_table）及 observe_target
（confirmed/target/room/source_table/evidence_ref）阶段结果。
最新测试15项通过，包含已知取物桌、错误房间和未检测到目标的负例。

任务：LayoutA，客厅 dog_doll -> square_low_table#living_room。
本目录实现固定任务调度与协议效果输出，暂不做通用语言解析。
原任务文字只做大小写、空白和末尾句点归一后的匹配；未知或修正任务停止。
没有读取 EnvironmentInfo 中实际物品 XYZ，未固定抓取坐标。

当前交付
  session1.py：无 ROS 依赖的事件驱动任务核心。
  run_session1.py：仅离线 JSONL 回放，绝不发布协议或运动命令。
  tests/test_session1.py：流程、负例、取消、任务隔离和去重测试。
  example_success.jsonl：明确 synthetic 的合成成功样例。
  example_success_result.json：离线回放结果，不是机器人现场成功。

流程
  新 Session 建立实例并完成真实启动检查；收到 Environment=LayoutA 和
  Are_you_ready? 后产生 I_am_ready。收到匹配任务后依次请求：
  navigate_room -> search -> grasp -> tuck -> navigate_destination -> place
  -> verify_placement -> 等待 Task_succeeded。
  到位且房间正确后产生 Room_reached；真实抓取核验通过后产生 Object_grasped；
  真实放置核验通过后产生 Task_finished。协议 detail 保持空字符串。
  Task_failed 或 Mission_complete 终止本实例。启动下一 Session 必须新建实例。
  本固定 Session1 没有 Does_not_exist 分支：未找到保留未完成原因，不伪报不存在。

运行（不需要 Unity，不操作 ROS）
  cd 本目录
  python3 -m unittest discover -s tests -v
  python3 run_session1.py --events example_success.jsonl --output /tmp/session1-replay-new
  输出目录必须是新目录；命令没有 --run 或 --execute 运动模式。

真实执行器接入合同（尚未完成，不可把离线通过当成现场可运行）
  一个主终端拥有机器人控制权。读取 drain() 的效果并按顺序处理。
  operation 携带 session_id、run_id、operation_id、stage、task 和前阶段 evidence。
  回调 result() 必须回传相同身份；不可在真实接入时使用回放的 $pending/$run。
  客户端必须持续处理 Moderator 中断；停止效果需要取消原操作并停止运动，
  保持不确定持物，不自动张爪。已启动程序的取消不能仅清空本地队列。
  真实操作未退休前不能启动下一操作；状态机不负责 ROS 动作取消确认。
  跨进程重启不要重放进行中的抓取或松爪：先核验现场状态并由主终端恢复。

各阶段结果字段
  所有成功结果 status=ok。
  navigate_room：arrived=true、room=living_room；实际 TF 房间归属核验。
  search：target=dog_doll、room=living_room、confirmed=true、evidence_ref；
    该引用指向本任务实际 RGBD/TF 目标证据，不能取旧帧或 Unity 真值。
  grasp：target=dog_doll、grasp_verified=true；由真实提起验证产生。
  tuck：holding_verified=true、carry_ready=true。
  navigate_destination：arrived=true、destination=square_low_table#living_room。
  place：release_sent=true；只代表完成松爪操作，尚不能宣告任务完成。
  verify_placement：target=dog_doll、destination=square_low_table#living_room、
    placement_verified=true；来自松爪后实际支持/落位观测。
  失败返回 status=incomplete/failed 和 reason；核心停止后续派发，保留证据。
  抓取已有最多5次纠偏应在抓取适配器内部复用，不在总调度层嵌套5次重新抓取。

现有代码接入线索与明确缺口
  导航：复用原 NavigateToPose/路径与门洞配置；不改已验证速度和到位参数。
  搜索：training/scripts/search_session_runtime.py。旧 prepare_active_search.py
    尚有六类白名单，不含 dog_doll；需在独立工作区接通30类名称与当前检测器。
  抓取：autograsp-integration-20261107-181039/run_autograsp.py 可调用通用
    不规则物体模板，但其 source-table 参数必填；搜索结果必须提供经过测量或
    语义匹配的承载面上下文，不能把目标放置桌冒充取物桌。
  玩具狗侧躺的真实抓取尚未验证。不能直接复用企鹅的固定头部抓取高度。
  搬运/放置：common-placement-v1-20261107-162514 是已有部署线索；尚需将
    通用抓取的当前持物几何转换为其实际输入，并提供玩具狗松爪后定位核验。
  共享核心只接受结果字段；证据真实性由真实适配器负责。进程退出码0或生成
    plan.json 不能自动映射为 grasp_verified/placement_verified。
  I_am_ready/Room_reached/Object_grasped/Task_finished 必须通过现有消息网关
    发送一次并遵守反馈时序；不得在发布瞬间把 Unity 判分当作成功。

后续最短接入顺序
  1. 主终端先接现场任务接收及只读计划输出，核对 LayoutA/固定任务。
  2. 接客厅导航和 dog_doll 搜索，得到当前目标与承载面证据。
  3. 用侧躺玩具狗验证通用抓取和持物收臂，输出统一结果。
  4. 接客厅方形矮桌导航、桌面测量放置、放置核验。
  5. 接四条协议消息与 Unity 判分，再运行一次全程无人工步骤验证。

隔离说明
  开发分支 feat/session1-automation-20261114。
  远端工作区 /__HANDYMAN_PACKAGE__/workspace。
  原测试仓库未切换分支、未修改代码、未启停 ROS。
  不新增全局导航超时，不读取当前持物的历史检查点作为现场状态。

2026-11-14 已知取物桌适配增量
  execution_plan.py 提供按家具名+房间筛选候选位姿、复制任务配置、
  导航/抓取 argv 构建和提起结果转换。不会启动子进程或 ROS。
  玩具狗明确使用 templates 模式，保留原导航参数，导航候选沿用导入顺序。
  候选顺序不等于实时路径可达验证；真实导航需复用原路径和门洞执行器。
  抓取返回 candidate_selected 不算抓住；必须读取真实 lift_evidence。
  argv 默认不执行；注意抓取程序即使无 --execute 仍可能转动头部，
  因此本轮没有调用该程序。主终端接入时必须拥有控制权。
  当前20项离线测试通过；尚未接现场观察、收臂、放置和协议运行循环。

现场几何适配增量
  perception_bridge.py：目标分割点云的房间核对、当前持物点云到放置朝向的
  坐标转换、复用 table_transfer_core 的桌面放置高度计算。
  当前点云必须由实时 RGBD/TF 提供；证据引用和承载面匹配由现场采集器提供。
  不使用苹果尺寸；可见点云未覆盖底部时不能声称已测得完整最低点。
  本模块是纯函数适配，不是相机订阅器或运动执行器。
  24项离线测试通过，包含旋转后底部高度、房外目标和真实公共放置模块。
  收臂动作、实时点云采集、放置支撑验证及协议循环仍待接线与实测。

执行调度层增量
  executor.py 将状态机效果交给非阻塞 start/poll/cancel 执行端口，
  按顺序发布协议和执行动作；接到 Moderator 中断后取消当前动作并保留夹持。
  执行端口必须在动作真正结束后才返回结果；发布成功不等于判分成功。
  29项离线测试通过，含整条模拟流程、动作中断、协议失败和抓取失败。
  此调度器没有默认实机端口，缺任一阶段端口会明确报错，不是假成功占位。
  尚未交付现场完整搬运：当前 generic grasp 未导出完整 held-context，
  已有放置入口使用各类物品适配，不能直接把玩具狗传给 flat_box_place。
  下一步应在隔离副本中添加抓取后当前持物数据导出，再接通狗的收臂及
  公共桌面放置入口；实时观察和松爪后核验也需实际端口。

持物数据交接已接入隔离抓取器
  grasp_runtime 是从当前 D 盘通用抓取部署只读复制的独立源码副本。
  成功完成三帧提起核验后，同一最终深度帧的实测表面与掌心 TF 转换为
  hand_palm_link 坐标，写入本次运行目录 held-context.json。
  result.json 包含 held_context_path；失败不会输出可用交接结果。
  execution_plan.py 的抓取命令已指向独立副本，原共享抓取器未修改。
  此数据是可见表面，不伪称完整模型；不是收臂完成或放置授权。
  收臂前必须重读当前状态，放置入口还需实测桌面与松爪后物品核验。
  单元测试累计33项通过，未执行现场抓取；旧训练数据和运行记录不变。

收臂与公共放置动作增量
  transfer_actions.py 实现同步 tuck/place 动作函数，必须由独占运行时工作线程
  调用，不可直接阻塞 Executor.tick。当前没有启动或自动部署到实机入口。
  tuck：当前持物核对、按现场源桌几何退出、保持夹爪开度折臂和降臂。
  place：掌心点云转换至放置朝向、实时桌面规划、桌外展臂、接近、
  当前支撑核验、松爪一次、抬臂退出收臂。复用原 table_transfer_core。
  已松爪后的清理失败保留 release_sent，继续独立放置核验，不重放松爪。
  当前39项离线测试通过，包含原公共模块动作调用；全部为模拟运行时。
  仍需现场实现 observe_hold、measure_and_plan、prepare_standoff、approach、
  verify_support、withdraw 等回调及工作线程取消，不能将这些离线测试
  视为已经完成实机接线或完整搬运。没有默认伪成功现场回调。

目标桌实时观测适配
  destination_observer.py 调用现有 runtime.sample、palm_pose、fresh_measurement、
  save，以及 can_destination_surface.measure / table_approach_reachability.plan_placement。
  将当前深度帧与同帧桌面平面传入既有规划器，按当前掌心 TF 转换持物表面。
  标明持物是运动学估计，不伪称新测目标；不得仅凭此模块认定持物仍在。
  measure_and_plan 可直接作为 transfer_actions.place 的同名回调。
  42项离线测试通过；本轮没有初始化 ROS、抓图或运动。
  尚缺实时持物核验、桌外退让/接近回调、独立放置核验和整机运行入口。

放置核验增量
  placement_verification.py 复用既有桌面支撑模型与高度/稳定性容差。
  DestinationObserver.verify_support 已接同帧桌面及当前掌心 TF，作为
  place 的 verify_support 回调；仍需与实时持物核验共同使用。
  verify_placement 接收松爪后独立目标 RGBD 点云，不接受持物运动学估计；
  检查目标与目的地、不同深度帧、支撑、离桌高度及稳定性，输出状态机
  所需结果。落位无需等于规划坐标，不增加新的导航误差要求。
  49项离线测试通过，使用真实 tabletop_support 模块及合成点云。
  尚未实现独立目标分割采集器及完整实机运行入口，未取得现场放置成功。

松爪后目标点云采集器
  target_capture.py 复用通用抓取 backend._sample/_cloud 和常驻 Worker，
  保存新 RGBD、检测结果、目的桌上连通目标点云；不用抓取模板代替新观测。
  同一连通表面的重复检测框合并，多个不同同类目标则保留歧义。
  capture 输出 placement_verification 所需记录；verify 采集不同帧后调用
  落位核验，结果可回传状态机 verify_placement 阶段。
  release_stamp_ns 必须来自同一会话传感器时钟的松爪确认；不能用系统
  墙钟或上次运行的时间。现场采集未执行，本轮53项离线测试通过。
  仍需实时持物核验、桌外退让及接近回调、执行线程取消和整机入口接线。

实时持物及接近适配增量
  独立 grasp_runtime 现为提起表面导出同帧 RGB 颜色；旧无颜色交接文件
  不会被当作实时持物核验输入。HoldObserver 复用 register_surface，
  用当前掌心 TF、新 RGBD 和夹爪反馈核对持物，缺少观测则不伪报成功。
  PlacementApproach 接入现有 adjust：完整实测平移，初次动作后最多
  再纠偏5次；高度按实测间隙修正，不改 Nav2 参数。
  58项离线测试通过，含真实配准算法、无物体深度、完整移动和纠偏上限。
  未实测收臂后的可见性；头部无法看到持物时，当前核验仍可能停住。
  仍缺源桌/目的桌退让适配、工作线程取消以及整机运行入口的最终接线。

退让动作增量
  table_withdrawal.py 按当前手部/持物包围范围和桌面边界计算整段退让距离，
  执行后重测；PlacementStandoff 复用既有 extension_clearance，退让后重规划。
  只验证桌面投影关系，不声明完整机械臂扫掠无碰撞。
  62项离线测试通过；integration_status.json 列明当前实现与剩余整机接线。
  未创建可运行的现场一键入口，未完成控制权/协议/取消/松爪时间的整合。

应用调度入口骨架
  application.py 组装 Session1 + Executor + 单动作线程；主循环持续处理
  Moderator 消息。取消必须等到动作线程退休且现场 stop Future 确认停止，
  不用杀线程冒充机器人已停。协议发送 Future 不是 Unity 判分确认。
  release_record.py 定义当前会话传感器时钟的开爪确认记录，调用方须在
  松爪反馈后取得新鲜帧，不能把系统墙钟或历史记录当作本次松爪时刻。
  67项离线测试通过，含整轮线程化流程及运行中取消；尚未接好真实 ROS
  订阅/发布/动作取消和全部阶段函数，不提供伪称可用的实机启动命令。

真实协议适配已实现，未启动
  ros_protocol.py 使用 handyman_msgs.msg.HandymanMsg，输入 /handyman/message/to_robot，
  输出 /handyman/message/to_moderator。bind_application 创建订阅、发布和调度定时器，
  必须显式提供完整阶段 handlers 与 request_stop，不自动填模拟成功函数。
  publish 返回只说明调用成功，不代表 Unity 收到或判分；最终仍等待 Task_succeeded。
  导入模块不会初始化 ROS，本轮仅假节点测试；累计72项离线测试通过。
  官方消息不带 session ID，Unity 重启须退休旧 Application 并建立新实例，
  不可把旧任务状态直接接到新一轮。完整现场入口和动作取消适配仍待完成。

五个真实后端阶段绑定
  stage_bindings.py 的 navigate_room/navigate_source/observe_target/grasp/
  navigate_destination 调用既有 motion.navigate 和通用 Coordinator，
  不重新实现路线或修改导航参数。已在客厅时 navigate_room 只核对定位。
  现场识别使用 Scene.target 和源桌范围，不使用 Unity 物品实际坐标。
  handlers 仍要求外部提供三个真实搬运阶段，不提供假成功占位。
  77项离线测试通过；主测试工作区仍为 feat/handyman-phase5-grasping。
  现有有限机械臂轨迹可能在子进程中断后继续，取消必须结合现场反馈，
  不能将 stop_group 或进程退出直接映射为 motion_stopped=True。

三个搬运阶段绑定
  transfer_bindings.py 连接当前抓取 context、HoldObserver、DestinationObserver、
  PlacementStandoff、PlacementApproach、transfer_actions 和 TargetCapture。
  松爪动作完成后采集新帧并写 release-record.json，再清理退出；落位核验
  仅使用此确认时间之后的独立帧。放置阶段不自动重复，同一绑定不跨运行复用。
  transfer_actions.place 新增 on_release 回调；记录失败保留 release_sent，
  返回 incomplete，不自动再松爪，也不提前报告 Task_finished。
  81项离线测试通过；仍需现场运行实例、退让包围范围和底部可观察性输入，
  以及真正的运动中断确认。八阶段有绑定代码不等于已经一键实机可用。

八阶段组装与退让观测
  assembly.assemble 接入八阶段、当前共享 runtime/worker、协议节点及源/目的
  桌退让观察器。调用方仍须创建并独占实际运行资源，不能直接无条件运行。
  WithdrawalObserver 从当前夹爪开度采样实体手部包围范围，结合当前掌心 TF、
  同帧桌面与持物点云计算地图坐标包围范围；空手退出不带旧物品模型。
  已只读加载7个远端实际依赖，无 rclpy.init，无节点/相机/运动。
  83项离线测试通过；真实停止确认和放置底部可观察性仍需接线，不填固定True。

运行资源生命周期
  runtime_resources.RuntimeResources.open 显式获取 RobotLock、创建 ROS 上下文、
  runtime、常驻模型客户端、通用抓取后端、动作后端和协议节点；只允许新D盘
  输出目录与当前 Session1 配置，不使用共享苹果默认任务。
  失败时按逆序清理，动作未退休或子进程仍活跃时不释放控制锁。不会自动发
  I_am_ready 或执行任务。默认 ROS 环境不改，临时模型环境标志清理时恢复。
  87项离线测试通过，包含部分启动失败和退出顺序；本轮未调用真实 open。
  尚无已验收现场CLI：须先接真实停止和底部观测接口，再完成整体验证。

取消与停止确认代码
  motion_stop.py 安装发布前取消门控，记录已发送有限轨迹的执行时长；
  仅对当前任务的导航子进程发送 SIGINT，沿用原 Nav2 清理。
  运行中的有限动作不通过开爪或杀进程强行终止；动作线程退出后，在同一
  动作线程采样新关节/里程计，至少3帧跨0.3秒静止才确认停止。
  有限子动作异常且终点未确认时，停止确认失败并保留控制权。
  assembly 默认连接 StopController；拥有者退出时还需关闭其 supervisor。
  91项离线测试通过，未现场验证取消与反馈时序；没有宣称紧急制动能力。

底部估计与命令行草稿
  support_reference.py 记录提起前测得的支撑平面相对掌心的位置和法向；
  只有放置时保留原支撑面朝向（0.5度内）才复用支撑高度，且标为推算，
  不声称直接看见底面。翻转或缺少接触高度带时不能默认通过。
  run_live_session1.py 默认只检查配置文件，--execute 才会创建资源并执行。
  prepare_live_config.py 从原配置生成独立狗搬运配置，不改共享苹果配置。
  累计97项离线测试通过；此启动入口仍是未现场验收草稿，不应理解为
  本轮已完成自动比赛。下一步先进行组装集成回归，再交主测试终端实测。

完整阶段串联回归
  实际 StageBindings、TransferBindings、公共放置动作和 Application 已
  在模拟硬件输入下串联八阶段，松爪一次，发送Task_finished后等待Moderator。
  修复失败阶段未等待停止证明，以及退休判断漏算尚未发送effects的问题。
  累计101项离线测试通过，不代表实机搬运成功。下一步由主终端做现场预检。

现场预检交接
  --preflight 与 --execute 互斥。预检禁用动作发布，只读当前状态/RGBD并
  推理一次，输出 preflight-result.json；默认无模式仍只检查文件。
  start_session1.sh 设置子进程的既有 domain71 环境，具体命令见
  MAIN_TERMINAL_HANDOFF.txt。本终端未执行现场预检。103项离线测试通过。

2026-11-14 候选位姿选择更新
  正式组装同时为源桌和目的桌接入 CandidateSelector：读取现有 map.yaml/PGM，
  根据既有 Nav2 footprint 的外接半径和 padding 构建保守可通行栅格，
  排除静态不可达点，以及候选点至目标桌之间被地图障碍阻挡的点；
  对剩余点按栅格路径长度排序。未知栅格不视为自由空间，不改 Nav2 参数。
  二维遮挡检查是保守的家具接近/观察预筛，不表示已完成三维视线证明；
  活动客人仍由现场导航感知处理。目标桌边界允许一个栅格对角线误差，
  避免把目标桌自身边缘误当作其他家具。
  LayoutA 初始位姿审计：源桌候选 2 优先（2.638m），其次候选 0（4.290m），
  候选 1 路径仅0.724m但桌前通道静态遮挡，予以排除。
  审计位于 D:/handyman-training-runs/session1-candidate-selection-audit-20261114.json。
  111项本地离线测试通过；新排序尚未做现场导航抓放整轮验证。
  上一轮 resumed-20261114-210058 已停止，grasp_not_verified、holding=empty；
  机器人不是初始姿态，勿直接使用初始定位重锚或假定机械臂收起。
