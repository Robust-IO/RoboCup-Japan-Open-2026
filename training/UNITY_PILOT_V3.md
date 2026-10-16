# V3 小批量验证：边缘漏检与图案误检

本次真实画面：饮料罐在中央置信度约 0.979，移到右侧约 0.077，移回中央恢复约 0.979；垃圾桶图案仍误检。不能据此认定只有位置是原因。

## 修改范围

- 主物品中心随机投影到 3×3 画面区域，保持原有随机朝向和近中远距离。
- 每帧记录 requestedViewport、actualAnchorViewport、viewportCell 和 anchorClassId。Unity viewport 原点在左下，PNG 显示方向仍须实测核对。
- 背景家具接口原本已存在。本次明确提示选择带图案的真实空垃圾桶模型，背景 mask ID 为 0；没有自动查找垃圾桶，也不保证它每帧都可见。
- 不修改机器人相机、导航、旧模型或训练增强配置。RGB 与 mask 使用同一相机。

## Unity 操作（不进入 Play）

1. 解压 HandymanDatasetGenerator-v3.zip，把其中 Assets/HandymanDatasetGenerator 文件夹及同级 .meta 复制到实际 Unity 项目的 Assets 下，覆盖同名工具文件，先保留旧工具备份。
2. 等待编译，Console 不应有红色错误。运行 Tools > Handyman Dataset Self Check，应出现 v3 检查通过。该自检不替代渲染与标签检查。
3. 打开 Tools > Handyman Dataset Generator，确认顶部 V3。保留原有五类物品、RGB 相机、地板和墙材质选择。
4. Optional background furniture 选择实际空垃圾桶模型根节点（应含图案 MeshRenderer），不要选 DestinationCandidates 空节点、整个 Layout 或含可抓取物的家具。工具不支持透明、镂空、双面及蒙皮材质；报错时不要为导出而修改原场景材质。
5. Frame count = 100，Batch seed = 20260913，输出新目录 D:/handyman-dataset/pilot06。不要覆盖 pilot05、train01、val01。
6. Generate / Resume。完成后把 pilot06 整个目录传到 ROS 电脑 D:/handyman-dataset/pilot06；只传数据，无需 Unity 工程。

## 验收后才能扩量

- 先检查 batch.json 的 generatorVersion 为 3.0，三类距离及画面区域覆盖。
- 检查实际 RGB 中物品位置与 mask 对齐；边缘截断不是天然错误，但必须保证剩余可见部分标签正确。
- 检查垃圾桶图案是否真实可见，且没有被标为可抓取物；不能把存在其他目标的帧直接当作无标签负样本。
- 运行现有转换器的 --inspect，检查小物体、断裂 mask、轮廓 IoU 和预览图；未通过前不生成大批量。
- 当前现场对照图只用于诊断，后续独立回归必须重新采集，不能把用于训练的图像又作为独立测试。
- V3 尚需在实际 Unity 6/URP 环境编译、导出验证；不承诺本次调整已解决误检。
