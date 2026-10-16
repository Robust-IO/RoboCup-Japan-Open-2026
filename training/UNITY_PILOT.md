# Handyman 第四阶段：Unity 五类视觉数据试点

## 状态与范围

首批类别：`apple`、`canned_juice`、`rabbit_doll`、`pink_cup`、`white_cup`。
目的分别是验证圆形、圆柱、不规则轮廓和相似杯子的分类。
这是 **独立桌面采集 MVP**，不是完整房间数据生成器，也不是已经训练好的视觉系统。
不需要启动 ROS、Handyman.exe、Play Mode 或逐个等待 6 个 session。
不会修改比赛成绩、Moderator、地图导出器、导航参数或原始材质。

本机没有 Unity Editor，因此 Python 转换可自动测试，C# 编译、URP 渲染、RGB/掩膜对齐必须在 Unity 电脑上验证。
首批只生成 30 张检查图，不要直接生成几万张。

## 1. Windows Unity 端安装（仅传小工具）

仓库内维护源代码：`unity_tools/HandymanDatasetGenerator/` 和同级 `.meta`。
把该文件夹和 `.meta` 复制到实际使用的 Unity 工程：

```text
D:\handyman\handyman-unity2\handyman-unity2\Assets\HandymanDatasetGenerator
D:\handyman\handyman-unity2\handyman-unity2\Assets\HandymanDatasetGenerator.meta
```

也可将交付的 `HandymanDatasetGenerator-pilot-v1.zip` 解压到项目根目录，压缩包内已经包含 `Assets/`。
无需传输完整 Unity 工程、Library 或 Build；不要放到 Build 文件夹。
首次添加的文件只有编辑器脚本、实例 ID shader 和 `.meta`。

等待 Unity 编译完成，Console 不能有红色编译错误。
当前目标环境是 Unity `6000.0.62f1`、URP `17.0.4`。
本工具不支持旧 Built-in 工程，也不需要安装已停止维护的 Perception 包。

## 2. 找到实际模型

1. 打开比赛场景，例如 `Assets/Competition/Handyman/Handyman.unity`，但不要点击 Play。
2. 打开菜单 `Tools > Handyman Dataset Generator`。
3. 点击 `1. Find Sources in Loaded Scenes`。
4. 确认五个字段都指向正确物品的模型根节点。
   自动查找会包括未激活的 Layout，但跳过没有渲染器的空节点。
5. 如果缺失或选错，从 Hierarchy 拖入实际模型根节点；不要拖 `GraspingCandidates` 中的空坐标节点。
6. 若场景有未保存修改，先自行决定保存还是撤销。工具不会代你保存或撤销场景。

类别名固定，不依赖 FBX 文件名。局部训练编号与原 27 类编号映射如下：

| 试点编号 | 类别 | 原 27 类编号 |
|---|---|---|
| 0 | apple | 0 |
| 1 | canned_juice | 3 |
| 2 | rabbit_doll | 15 |
| 3 | pink_cup | 14 |
| 4 | white_cup | 26 |

试点权重不能直接替换现有 27 类运行时权重：后续节点必须按名称/显式映射处理。
现有 `training/classes.txt` 和 `training/dataset.yaml` 不会被覆盖。

## 3. 先生成 30 张

默认输出：`D:/handyman-dataset/pilot01`，Frame count：`30`。
默认 `640x480`、垂直 FOV `60` **只是试点值，尚未校准为机器人相机参数**。
如果找到机器人头部 RGB Camera，可拖到 Optional RGB camera 并复制垂直 FOV。
图像分辨率仍需手工设为 ROS 实际 RGB 图像尺寸；此版本不复制镜头偏移、畸变、后处理。

点击 `2. Generate / Resume`：

- 每次 Editor update 采一帧，UI 可取消；完成帧保留。
- 在独立 Preview Scene 中只复制静态网格、变换和材质引用，不实例化原始脚本。
- 保持真实物理尺寸与材质，随机视角、物品偏航、组合、桌面颜色和照明。
- 每十帧有一帧空桌面负样本。主目标轮换，其余物品作为同样被标注的遮挡物。
- 几何边界防止桌面物品相互穿透，但不进行动态物理仿真或自动稳定放置。
- 同一相机、同一静止场景依次输出 RGB 和 ID 掩膜；桌面参与深度遮挡。
- ID shader 使用二进制 RGB 颜色，关闭后处理、抗锯齿和 HDR。

目录结构：

```text
pilot01/
  batch.json               # 参数、种子、来源资源标识
  rgb/000000.png           # 原图，无框线
  instance/000000.png      # 实例颜色掩膜，黑色为背景
  frames/000000.json       # 类别、相机/物品位姿；该文件代表此帧已完成
```

采集时用 Unity 真值自动标注；比赛运行时不读取这里的物体位置。
工具不判断竞赛规则是否允许使用赛事资源训练，正式提交前仍需核对规则。

续采要求输出目录、种子、帧数、相机参数、来源资源完全一致。
设置或保存的来源资源变化时，必须选择新的输出目录。
同样种子在相同环境下重复布局；不同 GPU/Unity 版本不保证逐像素一致。
未完成帧的图片可重新生成；已存在帧 JSON 但图片丢失会停止。

## 4. 两台电脑之间传什么

- Ubuntu/开发端 → Unity 端：首次仅传 `HandymanDatasetGenerator` 工具包。
- Unity 端 → Ubuntu 端：传 `pilot01` 整个采集批次，而不是 Unity 工程。
- 如果文件放在 ROS 电脑的 Windows D 盘，Ubuntu 路径是 `/mnt/d/handyman-dataset/pilot01`。
- **另一台电脑的 D 盘不会自动出现在本机 `/mnt/d`**，需要先复制或通过局域网传入。

为了降低 Windows 挂载盘 I/O 开销，批次可复制到：

```bash
mkdir -p ~/handyman-datasets/raw
cp -a /mnt/d/handyman-dataset/pilot01 ~/handyman-datasets/raw/
```

目标已存在时不要反复覆盖，优先使用新的批次名称。

## 5. 首批检查、独立验证批次与转换

只有第一批 30 张时即可检查，不必等待第二批：

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python training/scripts/prepare_unity_pilot.py \
  --inspect /mnt/d/handyman-dataset/pilot01 \
  --output ~/handyman-datasets/pilot01-qa
```

查看 `~/handyman-datasets/pilot01-qa/previews/inspect/` 和 `qa_report.json`。
inspect 模式只检查和输出叠加图，不生成训练 YAML。

另采 `pilot02`，把 seed 改为 `20260908`，仍先用 30 张。
不同采集批次分别分到 train/val，不能从同一序列随机抽帧混分。
更正式的验证还需要独立房间背景、视角集合，以及机器人实际相机数据。

在 Ubuntu：

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python training/scripts/prepare_unity_pilot.py \
  --train /mnt/d/handyman-dataset/pilot01 \
  --val /mnt/d/handyman-dataset/pilot02 \
  --output ~/handyman-datasets/pilot-v1
```

输出目录必须不存在。转换器生成：

- `dataset-seg.yaml`：分割训练配置；`labels/` 为 YOLO 多边形。
- `dataset-detect.yaml`：检测训练配置；`detect/` 为独立图片/框标签目录。
- `masks/`：原始实例掩膜，不丢失杯柄孔洞等细节。
- `previews/{train,val}/`：掩膜与框叠加预览，每个 split 最多 50 张。
- `qa_report.json`：各类数量、拒绝帧及原因。

自动检查颜色编码、图像尺寸、类别编号、文件配对、重复图片、重复种子、无可见目标、退化轮廓。
不把完全遮挡的物体画框。可见目标过小会隔离整帧，避免丢标签后成为错误负样本。
YOLO 单多边形不能精确表达孔洞/多个断开的区域；轮廓回填与原掩膜 IoU 低于 0.95 时隔离整帧。
这可能使杯子或遮挡图被较多拒绝：先查看报告，不要简单降低阈值；原始掩膜仍保留供改进转换。
缺少任一类别时不生成训练 YAML，应增加采样并使用新输出目录重跑。

**人工必须检查：**

1. RGB 不是全黑/粉色，五种物品外观和实际比赛一致。
2. 叠加轮廓不左右翻转、不上下颠倒，严格贴合物品。
3. 桌面、物品间遮挡正常；被挡住的区域不能透出标签。
4. pink_cup 与 white_cup 没有混标，兔子头、耳朵、身体属于同一个实例。
5. 负样本确实没有五类目标；RGB 原图没有被画框。
6. QA 中每类都有样本。30 张只用于链路检查，不足以证明模型可靠。

## 6. 通过检查后才训练

先扩充至几百到一两千张试点数据，再做小模型实验。实际数量根据机器人相机验证结果决定。
现有训练脚本可显式传入分割预训练权重，不改变原检测默认参数：

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python training/scripts/train.py \
  --data ~/handyman-datasets/pilot-v1/dataset-seg.yaml \
  --model yolo26n-seg.pt --device 0 \
  --epochs 30 --batch 4 --workers 2 --imgsz 640 \
  --hsv-h 0 --hsv-s 0.15 --hsv-v 0.2 --fliplr 0 \
  --name handyman_pilot_seg
```

首次运行可能下载预训练权重。8 GB 显存先从 batch=4 开始，OOM 时降为 2 或 1。
这条命令目前仅作为后续步骤，没有自动启动训练。
最终精度需在独立真实 ROS 相机帧、远距离、遮挡和目标不存在场景中验证。
模型未检测到物品不能直接发送 `Does_not_exist`，还需房间扫描覆盖、多帧确认和重试逻辑。

## 常见问题

- 没有菜单：路径必须包含 `Assets/HandymanDatasetGenerator/Editor`，先解决 Console 红色编译错误。
- Found 2/5：检查当前打开的是比赛场景；手动拖正确模型根节点，不要给空节点改名凑数。
- Source scene has unsaved changes：自行确认并保存/撤销；工具不会改成绩或保存比赛场景。
- Unsupported material / Skinned mesh：第一版有意拒绝未支持的渲染方式；提供物品和材质截图后扩展，不可删掉校验硬导。
- Nonbinary mask colors：检查 ID shader、渲染器特性、后处理；不要用 JPEG 保存掩膜。
- 正常 RGB 但标签全空：检查 Shader 编译、相机朝向、URP 兼容性；先停止批量采集。
- 框对但轮廓不对：检查原始 ID 图及上下翻转，优先修复采集端，不用随意扩大框掩盖问题。
- 报告拒绝很多帧：逐项查看原因，尤其小目标、孔洞、遮挡碎片；不要把它们无标签加入训练。
- 已存在输出目录：选择新目录。转换器不执行删除或覆盖旧数据。

## 自动测试

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python -m unittest discover \
  -s training/tests -p test_unity_pilot.py -v
```

实现依据：Unity 6 [PreviewRenderUtility 源码](https://github.com/Unity-Technologies/UnityCsReference/blob/6000.0/Editor/Mono/Inspector/PreviewRenderUtility.cs)
与 [Ultralytics 实例分割文档](https://docs.ultralytics.com/tasks/segment/)。
