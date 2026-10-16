# 自动头部观察阶段：单点实测准备版

## 当前范围

搜索运行器可通过 `--head-tilt -0.25` 启用自动头部阶段。
隔离测试使用 ROS domain 73；现场必须同时显式启用 `--allow-live`、domain 71 和 localhost。
默认模式不发头部命令；发指令前要求唯一订阅者且无其他头部命令发布者。
Unity、地图、模型和既有导航参数没有修改。

## 顺序与验证

1. 原工作进程完成导航，原只读观察器确认搜索点到达并静止。
2. 运行器收到匹配当前任务和点位的到达记录，等待新鲜关节反馈及唯一头部订阅者。
3. 运行器只发送一次 2 秒头部轨迹。
4. 运行器检查头部到位和稳定，在当前任务目录写入 `head-ready.json`。
5. 观察器独立订阅关节状态，检查证明中的任务、点位、角度和时间，并确认关节稳定。
6. 清空低头前的视觉证据，仅接受头部就绪时间之后的新图像，再进行原多帧识别和 RGBD 定位。

头部阶段最多 8 秒；任务总期限不变。只延后单视角观察计时，不续期整个任务。
头部运动期间不会累计识别结果，不授权 `Does_not_exist`。
取消后不再启动新的搜索动作；若已发过头部轨迹，则发一次 0.2 秒保持当前反馈角度的指令。
只用保持指令之后的新鲜关节反馈确认连续 0.6 秒稳定，并写入 `head-stop.json`。
停止后无反馈、反馈过期、角度继续变化均不能报告清理完成；观察器退出和头部停止分别判断。

## 现场验证边界

原按轨迹时长判断停止的逻辑已替换为关节反馈确认。
已核对当前 Unity `HSRSubJointTrajectory.SetJointTrajectoryRotation` 会用新轨迹替换旧轨迹。
完整流程、动作中取消、停止反馈丢失、忽略停止等隔离测试已通过。首次 Unity 自动串联实测（20260917-06）导航成功，但头部阶段失败，未完成搜索或停止确认，不能算现场通过。
现场入口已显式开放供下一次受控单点测试使用；不能把合成测试通过当成现场验证通过。
当前只支持一个指定俯角，不代表左右扫描、多点覆盖或房间搜索已完成。

### 首次现场失败后的诊断更新

相机 TF 记录显示头部发生了转动，但旧日志没有原始关节反馈，尚不能判定是时间戳、反馈间隔还是角度稳定性导致失败。
新增每任务 `head-trace.jsonl`：记录命令、关节角度、传感器时间差、接收间隔、稳定性拒绝原因、保持命令及关闭状态。
最多保留 2000 条反馈，生命周期和故障记录不受该上限影响；运行器事件另保留具体 `head_stage_failed` 原因。
本次只修复诊断信息缺失，不扩大时间容差、不跳过反馈验证、不把超时视为停止成功。实际故障原因仍需带新日志的受控复测确认。

### r7 时间偏差修正（2026-09-17）

后续实测 r7 确认：导航成功，头部实际到达 -0.25 rad，但关节时间戳提前约 14.105 ms，超过旧 1 ms 上限，导致连续稳定窗口无法成立。
已核对 Unity `HSRPubJointState` 调用 `Header.Update()`：使用毫秒级系统 UTC 减同步偏移；`SIGVerseRosBridgeTimeSynchronizer` 默认发起一次同步，收到时间差后写入 Header。这说明两端并非同一时钟，但不能仅凭源码断定本次偏差的全部来源。

修正仅针对头部：提前量固定上限 20 ms，超出仍拒绝。不改原消息时间戳，不自动学习无限偏移，不修改 Unity 或系统时间。
300 ms 过期限制、严格递增、连续 0.6 s 双时间窗口、角度限制、8 s 阶段超时均不变。
停止证据必须满足 `sensor_stamp > hold_command_ros_stamp + 20 ms`，排除允许偏差下可能在停止前采集的帧；仍需保持后连续稳定反馈。
头部 ready 证明使用同一有界时差校验；视觉帧截断仍保留原始头部时间戳，不让旧图像越过门控。

`training/tests/replay_head_feedback.py TRACE` 可对原始反馈比较新旧策略，不连接 ROS，也不生成 ready/stop 证明；离线回放通过不等于 Unity 全流程通过。
隔离集成测试追加 `--head-skew-ms 14` 可注入头部时间提前量。

### r8：TF 恢复后的头部证明续期

r8 Unity 实测已通过导航、头部到位、观察门控和停止反馈确认。之后一次 TF 短暂中断恢复成功，但重开观察时首次头部证明超过 1 s，导致 `invalid_head_proof`；本轮结果是 incomplete，不是目标不存在。

修正：头部证明由一次性文件改为当前任务内、由每条有效稳定关节反馈续期的证明。仍保留 1 s 有效期、身份和角度校验；没有新反馈、反馈异常、任务失败或取消后都不续期。不重新发头部运动，也不延长搜索总期限。
观察器继续独立验证关节稳定与机器人到达，并在 TF 恢复后用新的时间戳截断旧图像、重新收集证据。
隔离测试 `--case sequence --auto-head --head-skew-ms 14 --head-tf-recovery` 在首次观察后断开 TF 0.65 s，要求重新进入观察、每任务只有一次低头和一次保持命令，且停止得到确认。

## 文件

- `training/scripts/search_head_stage.py`：运行器拥有的头部阶段。
- `search_session_runtime.py`：启动条件、取消和生命周期衔接。
- `search_arrival_ros.py`：只读头部验证和新图像门控。
- `search_runtime_state.py`：匹配身份的到达记录。
- `training/tests/run_search_session_runtime.py`：`--auto-head` 和三类头部故障测试。
- `training/tests/test_search_head_stage.py`：证明文件的身份、角度、时间校验。

## 隔离测试

```bash
source /opt/ros/humble/setup.bash
source /tmp/handyman-search-nav-install/setup.bash
cd ~/RoboCup-Japan-Open-2026
python3 training/scripts/cyclone_test_env.py --domain 73 \
  --dds-config training/config/cyclone_search_stack.xml --run -- \
  python3 training/tests/run_search_session_runtime.py --case sequence --auto-head
```

追加 `--head-fault missing_feedback`、`--head-fault wrong_angle`、
`--head-fault cancel_during_motion`、`--head-fault stop_feedback_lost` 或 `--head-fault stop_ignored`
可验证无反馈、角度错误、动作中取消、停止后失去反馈和未保持目标角度。
这使用真实运行器、观察器、工作进程和协调器，导航、TF、关节和视觉数据均为合成数据，不连接 Unity。
