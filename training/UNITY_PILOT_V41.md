# V4.1 番茄酱透明材质：显式不透明核心标签

检查的工程副本：ketchup_full.prefab 使用一个 MeshFilter/MeshRenderer，四个材质槽。槽 2 对应 GUID c2c237c9018062a43b32d3597d2b1b25 的 phong1；Alpha 100/255、URP/Lit、双面、Alpha 混合、无 BaseMap、无 AlphaClip。用户截图 RGB 与副本颜色不同，因此不写入任何替代颜色。Unity 导入后的四个子网格及槽对应关系由运行时再次检查，Console 输出槽 2 的索引数量。

## 策略与限制

- 必须勾选 Ketchup opaque-core mask 才启用；默认仍拒绝透明材质。
- RGB 原样渲染全部四个材质，不修改源场景、材质、网格或相机。
- 仅验证通过的 filled_ketchup 槽 2 在 mask pass 中丢弃片元，不写颜色/深度。其他不透明部分正常生成 ID 6。
- 标签表示不透明核心，不是玻璃/塑料层的完整物体轮廓，不表示光学可见度，也不可直接用来确定抓取边界。透过透明层看到的其他目标仍按其自身 ID 标记；不模拟光学衰减。
- batch/frame 的 transparencyPolicy 明确记为 filled-ketchup-opaque-core-v1；原始帧元数据随转换保留。
- 不默许其他透明、裁剪、双面材质。不匹配材质 GUID、槽数、网格子网格数或 Alpha 时继续报错，并显示材质名及槽号。
- 若不透明核心断裂，现有转换器会拒绝该帧；不要以丢弃部分核心的方式强行通过。不与另一种轮廓定义的数据无说明混合。

## 验证导出

1. 退出 Play，备份工具。用 HandymanDatasetGenerator-v4.1.zip 内 Assets 内容覆盖同名文件，包含 InstanceId.shader。保持原比赛材质不变。
2. 等待编译无红色错误，打开生成器确认 V4.1，六个物品已指定，勾选 Ketchup opaque-core mask。
3. 新目录 D:/handyman-dataset/pilot09，90 帧，seed=20260919。pilot08 即使失败也不要覆盖。
4. 完成后把 pilot09 传回 ROS 电脑同一路径。先运行六类 --inspect，不进行训练。
5. 检查 RGB 仍有原透明外观，青色核心 mask 对齐番茄酱的实体部分；透明边缘不在标签中是明确策略，不是完整轮廓标注。检查六类计数、断裂拒绝率及同框遮挡样本。

Python 检查不能证明 Unity Shader 编译或 RGB/mask 渲染正确；必须完成上述真实导出验证才能考虑扩量。
