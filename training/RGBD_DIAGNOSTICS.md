# RGBD 实时只读诊断

新增入口：`scripts/rgbd_diagnostic_node.py`。系统 Python 运行 ROS，独立 Pixi 子进程
`rgbd_inference_worker.py` 加载当前六类模型，执行 640/1280 双尺度分割。

唯一发布话题为 `/handyman/vision/diagnostics`（std_msgs/String JSON）；不发布
`/vision`、目标位姿、导航指令、关节指令或比赛消息。未修改旧视觉节点及启动文件。

运行需要先 source ROS Humble，明确指定隔离的 ROS_DOMAIN_ID=71 和
ROS_LOCALHOST_ONLY=1，并提供 `--repo`、`--pixi`、`--weights`、新的 `--output` 路径。
`--seconds` 默认 30，最大 120；GPU 启动另有 120 秒超时。进程结束回收其专属推理子进程。

输出目录保存每次 RGB、原始深度、时间戳、检测图、推理结果及 diagnostics.jsonl。
只允许一个推理请求在途，避免旧图排队。时间配对上限 50 ms；旧结果、缺失目标、
目标歧义、TF 缺失和推理超时输出拒绝状态。单视野无目标不代表 Does_not_exist。

深度按 16UC1 毫米或 32FC1 米解析，掩膜向内收缩两像素后计算可见表面位置统计。
这些不是物品中心或抓取位姿。场景未动时重复输出也不等于跨场景精度验收。

## 当前限制

TF 查询在推理完成后最多等待 0.5 秒，等待期间继续处理 ROS 回调；始终查询
深度图原时间戳，不退回最新 TF。超过等待上限输出 tf_wait_timeout；若图像已
过期则立即拒绝。诊断记录 tf_wait_status 和 tf_wait_ms。

`geometry_verified=false` 和 `actionable=false` 固定保持；当前实时节点不积累
可用于动作的 stable 状态。图像配准与光学坐标约定仍需源代码/标定证据确认。
`rgbd_quality.py` 的多帧门限已独立测试，尚未启用为实时动作许可条件。
实时发布的是候选表面统计和同时间 TF，不擅自把候选坐标转换为机器人抓取目标。
后续须完成几何确认、接入完整质量门限及断流/目标移除实测，再讨论动作接口。
