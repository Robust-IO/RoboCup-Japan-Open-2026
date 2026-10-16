# 搜索点观察：短时 TF 中断恢复（2026-09-17）

## 现场依据与限制

session1 / LayoutA 的第 4 次单点测试已一次完成导航并通过到达、朝向和静止检查。
观察随后因 `tf_stream_timeout` 结束，未授权 `Does_not_exist`。
同一时间的 RGBD 诊断中，相邻有效深度样本相差约 0.59 秒，之后恢复；后续探测亦收到图像。
这支持存在短时数据间隔，但不能确定是 Unity 停顿、传输延迟还是接收处理造成；没有原始 TF 录包，不能宣称已修复底层断流。

## 本次行为调整

- 仍在超过 0.3 秒没有有效 TF 时立即停止采信视觉证据，没有放宽到达和静止阈值。
- 运行器显式使用 `--recover-transient-tf`；独立观察器默认仍是直接终止策略。
- 每个观察器只允许一次恢复，最多 4 秒，且受原搜索总截止时间约束。
- 清除旧视觉适配器和点位证据，保留已成功的同一个导航目标，不重新发送导航。
- 恢复期间状态显示 `navigate`，语义为重新验证到达，而非要求机器人再次移动。
- 必须重新获得完整 TF 静止窗口、通过目标位姿检查并完成到达稳定检查，才开始新的观察窗口。
- 新观察只接收重新到达时间之后的图像；总任务时限不续期。
- 持续中断、恢复后再次中断仍终止；正常观察中的移动或位姿跳变仍终止；取消后不可恢复。
- 日志增加单调时钟、距上次有效 TF 的时间和是否用过恢复，方便下轮定位实际间隔。

## 改动范围

生产代码：`training/scripts/tf_motion_gate.py`、`search_arrival_ros.py`、`search_session_runtime.py`。
测试：`training/tests/test_search_tf_observer.py`、`run_search_ros_integration.py`。
未修改 Unity、地图、模型、导航参数、比赛消息逻辑或抓取逻辑。
远端旧文件备份：`/home/crazylearner/handyman-tool-backups/before-tf-reacquire-20260917/`。

## 验证

Python 单元测试覆盖重新稳定、固定恢复期限、二次中断、总期限、取消、错误位姿、移动、旧图像以及恢复消息先于计时器回调。
ROS 隔离测试（domain 73）注入 0.65 秒中断：重新到达后使用新帧得到 `found`；持续中断得到 `incomplete/tf_stream_timeout`。
完整运行器双点回归通过；这些是合成 TF/Nav2/视觉数据，不能替代 Unity 实测。

```bash
source /opt/ros/humble/setup.bash
source /tmp/handyman-search-nav-install/setup.bash
cd ~/RoboCup-Japan-Open-2026
.pixi/envs/default/bin/python -m unittest discover -s training/tests -p 'test_*.py'
python3 training/scripts/cyclone_test_env.py --domain 73 \
  --dds-config training/config/cyclone_search_stack.xml --run -- \
  python3 training/tests/run_search_ros_integration.py --motion-source tf --tf-recovery-cases
```

下一次 Unity 实测：保留上一轮搜索点位置，不拖动机器人；只恢复 Play 即可。
若重启了 session，需要明确告知并重新对齐初始定位，不能在搜索点重新按初始坐标对齐。
现场应记录是否完成观察、是否触发恢复、具体间隔及新图像证据；单点未发现不等于房间内不存在。
