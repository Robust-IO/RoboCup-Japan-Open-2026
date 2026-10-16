# Unity 采集器 v2：50 帧试采

本版是房间风格的静态试点，不是完整房间采集、动画人物仿真或已完成的训练系统。
继续使用独立 Preview Scene，不需要 Play Mode，不修改比赛场景、材质、地图或导航。

## 安装和检查

退出 Play Mode，关闭采集窗口。先把旧 `Assets/HandymanDatasetGenerator` 和同级 `.meta`
备份到项目外；将 v2 压缩包的 `Assets` 内容合并进当前项目的 `Assets`。
不要放在 HandymanMapExporter 内，也不要同时保留两个版本的 C# 脚本。
原文件的 GUID 未改变，新增 `DatasetSelfCheck.cs` 和配套 `.meta`。

等待编译后运行 `Tools > Handyman Dataset Self Check`。
应看到 `12 naming checks + metadata roundtrip PASS`。它只检查名称与序列化，不验证渲染。
本机无 Unity Editor，v2 的编译和渲染仍需在 Unity 电脑验收。

打开 `Tools > Handyman Dataset Generator`：

1. `Find Missing Sources (keep assigned)` 支持 `apple#1`、`apple_01` 等数字后缀，
   优先选择带 Rigidbody 的根节点；已赋值的字段不会改变。仍应确认五类物品外观。
2. 引用 `HeadRGBCamera` 并复制 FOV；设置 **640×480、46.82°**。
3. 设置 **Frame count=50、Seed=20260909、输出 D:/handyman-dataset/pilot03**。
   Unity 可能保留旧窗口的序列化值，不能只依赖新代码默认值。
4. Optional floor/wall material 可从 Project 拖入实际地板/墙面材质。
   也可以保持空白，使用默认纯色；空白时不能宣称已有比赛纹理背景。
   工具复制材质，不修改原材质。贴图的平铺比例仍需查看预览确认。
5. Optional background furniture 可选一个**空的小家具模型根节点**，不是整个 Layout。
   宽≤1.5m、深≤0.8m、高≤2.2m；放在桌后方，地面位于 y=-0.8m。
   不要选带杯子、苹果等物品的家具，否则会产生漏标；工具只对标准类别名进行检查，
   非标准命名的嵌入物品仍必须由人工检查排除。
   不支持透明/镂空材质、SkinnedMeshRenderer 或特殊材质覆盖。
6. Generate / Resume，先只导出50帧，完成后传回 ROS 电脑的同名 D 盘目录。

## 变化和限制

- 近距离0.55–0.85m，中距离0.85–1.3m，远距离1.3–2m，每三帧轮换。
  距离是相机至观察点的距离，不是机器人抓取距离。
- 限制相机方位角±55°，使后墙处于背景侧；俯视角18–50°。
- 桌面缩为1.8×1.2m，增加地板与后墙；背景写入深度，但实例标签为0。
- 保持目标材质与尺寸，仅旋转偏航；多目标会产生一定遮挡，不保证每帧遮挡或每个目标可见。
- 新帧 JSON 记录 `cameraDistance` 和 `distanceBand`。
- `batch.json` 记录生成器2.0、背景来源和采样设置；不能向旧批次续写。
- 更近视角可能裁切物品或产生更多小碎片；转换器依然隔离不可靠多边形，不降低0.95阈值。
- RGB与掩膜采用同一相机设置；实际对齐、光照、模型落地状态必须查看新试采。
- EXE实拍已验证640×480/rgb8、画面直立，尚未完成左右镜像和同物品颜色的对照。

## Ubuntu 检查

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python training/scripts/prepare_unity_pilot.py \
  --inspect /mnt/d/handyman-dataset/pilot03 \
  --output ~/handyman-datasets/pilot03-qa
```

查看 `previews/inspect` 和 `qa_report.json`。检查物品是否穿桌、漂浮、变粉，轮廓是否对齐，
近中远样本是否齐全。通过后再规划独立种子和独立背景的训练/验证批次。
50帧不是正式训练集。未自动训练或推送代码。
