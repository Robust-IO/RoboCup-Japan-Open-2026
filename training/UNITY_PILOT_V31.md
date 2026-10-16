# V3.1：pilot07 背景图案验证

只修改生成器私有 PreviewScene 的克隆几何，不修改比赛场景、机器人相机、旧数据或模型。

## 变化

- 垃圾桶克隆放在桌子侧前方，使用旋转后的真实包围盒确保与桌面水平区域分离，桶底仍贴预览地面。
- 每三帧有一帧专门观察背景垃圾桶：隐藏五类目标，相机低角度对准桶身，按包围球自动确定距离。八个朝向扇区轮换，覆盖不同模型的正面方向，不假定模型 +Z 就是图案正面。
- 其余帧保留五类物品、九区域位置及近中远采样。背景 ID 始终为 0；背景专用帧 distanceBand 为 background-fit。
- 记录 backgroundFocused、furniturePosition、furnitureEuler。背景专用帧 requestedViewport 指桶中心，不是目标物。
- 30 帧中有 10 张背景专用图；这只是验证批，不是可直接证明训练效果的大数据集。

## 操作

1. 退出 Play，备份现有 Assets/HandymanDatasetGenerator。
2. 解压 HandymanDatasetGenerator-v3.1.zip，将 Assets 下的工具文件夹与同级 .meta 覆盖到实际 Unity 项目 Assets 下。
3. 等待编译无红色错误，打开 Tools > Handyman Dataset Generator，确认顶部 V3.1。可运行原有 Self Check；其日志仍标 v3，只验证命名和投影数学，不验证 V3.1 垃圾桶摆放。
4. 保留五类物品、墙地板和 RGB 相机选择。背景家具继续使用 pilot06 的真实垃圾桶模型。
5. Frame count=30，Batch seed=20260914，Output directory=D:/handyman-dataset/pilot07。必须使用新目录，保留 pilot06。
6. Generate / Resume 完成后，把 pilot07 整个目录传回 ROS 电脑 D:/handyman-dataset/pilot07。

## 验收

先看 rgb/000002.png、000005.png、000008.png 等每第三帧：桶身应清楚出现，至少部分朝向能看到目标图案。这些背景专用帧的 instance 图应全黑；真实目标帧必须有完整可见区域标注。

Ubuntu 端再用 prepare_unity_pilot.py --inspect 检查全批完整性、mask/标签及图像预览。尚未在 Unity 实际编译与渲染验证，若画面被遮挡或材质报错，停止扩量，保留 Console 信息和样本。不要修改比赛场景材质来绕过错误。
