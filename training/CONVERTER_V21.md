# 标签转换器 2.1：保留孔洞

本说明取代 UNITY_PILOT.md / UNITY_PILOT_V2.md 中“孔洞只取外轮廓并拒绝”的旧实现描述。
Unity 工具不变，不必重新采集、传输工程或覆盖原始图片。

原实现只取最大外轮廓，杯柄内孔被填实；与原掩膜 IoU < 0.95 就拒绝整帧。
新实现读取外轮廓和内孔，通过往返连接边编码为同一条轮廓路径。
这种路径包含重复顶点，不是简单多边形；已经针对本项目安装的 Ultralytics/OpenCV
填充实现验证，不能假定任何其他训练框架都能正确处理。

每个可见实例均检查：

1. 原掩膜仍是源数据，不修改、不补洞。
2. 导出归一化坐标，按最终文本重新读取 float32，再生成掩膜。
3. 调用安装版本的 `resample_segments` 与 `polygon2mask` 重建训练端掩膜。
4. 第2、3项与源掩膜的全分辨率 IoU 均须 >=0.95，否则仍拒绝整帧。
5. 目前不支持断开的多个连通区域，明确拒绝，不擅自丢掉碎片或拆成多个物品。

报告记录 converterVersion、ultralyticsVersion 和阈值。
这是几何增强之前、mask_ratio=1 的检查；不代表已验证旋转、裁切、Mosaic 和默认
mask_ratio=4 下所有样本的孔洞仍完整。正式训练前仍需检查训练可视化；
第一次试训可关闭复杂几何增强并使用 mask_ratio=1（显存占用会增加）。
原训练脚本尚未新增 mask_ratio 参数，不能直接给它传这个参数。

## pilot05 回归（2026-09-10）

- 原50帧：旧转换41通过、9拒绝。
- 新转换：48通过、2拒绝。
- 原来9帧杯子孔洞拒绝全部恢复。
- 000001、000041 在新策略下因断开区域拒绝；它们旧版通过，因此净增7帧，
  不是简单在原通过集上增加9帧。
- 原始数据及旧QA目录保持不变。

新报告目录：`~/handyman-datasets/pilot05-qa-holes-v21/`。

最终验证：23项训练/转换回归测试通过。实际 YOLODataset 加载48帧（含5帧负样本）、
108个实例，0损坏标签。在 `augment=False, imgsz=640, mask_ratio=1, overlap_mask=False`
设置下，经过训练加载器重采样和letterbox后的掩膜与原掩膜最小IoU为0.99894。
完整结果为同目录 `loader_audit.json`；该数值不是模型识别精度。

检查新批次仍使用原命令，只需指定不存在的输出目录：

```bash
cd ~/RoboCup-Japan-Open-2026
~/.pixi/bin/pixi run --frozen python training/scripts/prepare_unity_pilot.py \
  --inspect /mnt/d/handyman-dataset/pilot05 \
  --output ~/handyman-datasets/pilot05-qa-holes-v21-final
```

通过帧数量不是识别准确率；尚未开始训练。
