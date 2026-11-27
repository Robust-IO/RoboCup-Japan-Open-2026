# Session3 搜索接入

当前范围：真实协议 + 主/备用点搜索熊 + 修正后直接到 corner_sofa#lobby 确认并抓取狗。
抓取验证通过才发送 Object_grasped；撤离收臂后导航到 Moderator 前方，持物伸臂递交。
物品进入交付区域且夹持反馈仍有效后发送 Task_finished，只有收到 Task_succeeded 才记为成功。递交不松爪。
现有 Session1→Session2 连续流程保持两轮，Session3 单独实测后再加入连续运行器。

## 选点
复用原项目 build_active_search_points.select_points（包含遮挡分区备用点）和 active_search_policy.ring_views。
按房间凸包的最小面积旋转包围矩形计算长宽比；阈值在 profiles/session3.json 的 search.aspect_ratio_threshold，默认1.5。
不超过1.5只选主点；超过则加入最多2个原算法生成的备用点。
当前 LayoutA lobby 长宽约5.70×4.77m，比值1.196，使用主点 (1.905,-4.417038)，不启用备用点。
默认 continuous 模式：每个俯仰角连续转一圈，角速度0.4 rad/s；12个角度分区，每30度一组，实际帧须落在分区中心±7.5度内，保留约60度视场的重叠。
RGBD 使用拍摄时刻的历史 TF；采集与单个模型推理线程并行，最多2个在途推理任务，避免旧帧积压。
高置信度候选提前停转复查；缺帧、无效帧或疑似候选停稳补拍两帧。只有全部有效覆盖且无未决候选才允许报告不存在。
原 stopped 模式保留作回退，新 continuous 模式尚未 Unity 实测。
目标使用图像和深度定位，按大厅区域过滤房外物品；未使用 EnvironmentInfo03 的物品答案坐标。
完整配置视角全部有效且没有未决候选时允许报告本搜索策略下不存在。这不是无遮挡覆盖房间每一处的数学证明。

## 协议
Room_reached → 搜索 bear_doll → Does_not_exist → 等待真实 Corrected_instruction → 导航 corner_sofa#lobby → RGBD 确认 dog_doll → 抓取验证 → 收臂 → Moderator → 持物伸臂 → Task_finished → 等待官方结果。
修正前不直接抓狗；提前/内容异常的修正停止本次动作，旧异步结果按 operation/run ID 丢弃。
不对狗做全房扫描；沙发位置由用户明确提供，狗的位置仍须现场测量。不复用熊的负证据。若确实找到了熊，停止并保留冲突证据，不伪报不存在。

## 启动（需要 Unity 正在 Session3 初始位置）
在 Ubuntu：
```bash
cd /__HANDYMAN_PACKAGE__/workspace/training/session1
bash start_session3_round.sh --execute --from-session3-initial
```
该命令清理旧地图服务、加载 LayoutA 并按初始位置定位，准备空手后启动搜索监听。
不能在已移动的机器人上当作断点恢复命令。Session1和Session3同为LayoutA，必须由操作者明确选Session3。
已有正确地图和定位时可用 start_session3_search.sh --execute --output /__HANDYMAN_PACKAGE__/runtime/新的目录；此入口不重置定位。

## 配置与结果
运行配置为远端 session3-search-config.json；原料是已有 session1-live-config.json 的ROS/模型路径、当前地图和 profiles/session3.json。
source_table_id 为用户提供的 corner_sofa#lobby，仅修正指令后的抓取阶段使用；熊的搜索仍按大厅主点扫描。
地图变化后重新运行 prepare_session3_search.py；运行前校验环境与地图哈希。
日志位于 D:\handyman-training-runs\session3-search-*\task。
search-1-bear_doll.json 保存扫描结果；source-target-observation.json、grasp-result.json 保存修正后狗的定位和抓取结果；captures 保存图片证据。
暂无 Session3 Unity 实测通过记录；搜索、抓取、递交已串联，需从 Session3 初始位置实测。


## Moderator 递交（2026-11-15）

官方实现 `HandymanModeratorTool.IsDeliverySucceeded` 判断：物品 Rigidbody 质心到
`DeliveryPosition` 的三维距离 <= 0.3 m，并且仍由 HSR 抓持正确物品。因此不能先松爪。

`prepare_session3_handover.py` 从示例场景 Moderator 的 DeliveryPosition 完整父层级变换
计算地图坐标，不读取目标物体答案坐标。当前 AvatarMotions03.dat 为空，采用固定位置。
人物移动、任务重定位或换场景时需要重新配置/接入人物跟踪，不能沿用固定配置。

运行端根据抓取时测得的手掌坐标系物体表面估计中心，用当前 TF 更新位置。
这不是 Unity 的真实质心；采用 0.20 m 的估计中心容差，给官方 0.30 m 阈值留余量。
导航复用现有参数；收臂持物到达前方可达位姿后升臂、伸臂，根据反馈修正位置。
成功报告只来自 Moderator，离线测试不代表实际递交已成功。

生成搜索配置后，补充递交配置（以下在 Ubuntu 执行）：

```bash
cd /__HANDYMAN_PACKAGE__/workspace/training/session1
python3 prepare_session3_handover.py \
  --unity-root /mnt/d/handyman/handyman-unity2/handyman-unity2 \
  --config session3-search-config.json
python3 run_session3_search.py --config session3-search-config.json
```

后一条不加 `--execute`，只做离线检查。已有 Session3 启动器继续适用。
日志 `handover-evidence.json` 保存估计距离、持物状态和交付点；
`live-state.json` 只有在收到官方 Task_succeeded 后才显示 succeeded。
现有 Session1/2 连续运行器尚未扩展为三轮，单独 Session3 验证后再串联。

离线验证：本地 225 项测试通过；远端递交 7 项、Session3 协议 11 项通过。
示例交付点 ROS 坐标约 (0.28454, -0.00006, 0.98875) m，生成了 6 个地图可达的前方候选位姿。


## 本轮问题和修复

- 20261115-030952 运行，Room_reached 到 Does_not_exist 用时156.09秒；实际收到 dog_doll 修正指令。
- 旧导航候选 (2.305,-4.617038) 距错误绑定的普通双人沙发前沿约1.79m。
- 导出器排除了带 DestinationCandidates 标签的真实家具自身，corner_sofa#lobby 被误绑到 lobby/sofa，中心偏差2.21m。
- 已生成 D:\handyman-updates\HandymanMapExporter-corner-sofa-fix-20261115.zip，仅一个 C# 文件；需 Unity 重导 LayoutA 后重新导入。
- 尚未把未知真实网格边界用猜测值覆盖到运行地图；禁止用当前旧绑定继续抓取。
- Session3 协议节点已使用独立 SingleThreadedExecutor，避免与运动采样共用全局执行器。

修正后 Session3 使用 near_furniture_front：在真实家具前沿外0.55/0.70/0.85m生成观察候选，经地图可达性和视线检查后按路径距离选取；Session1/2不启用此策略。
连续扫描与取消清理、角度环绕、覆盖缺口补看已有离线测试；须重导后实测。
