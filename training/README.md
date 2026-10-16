# Unified YOLO Training Workspace

这个 `training/` 目录现在只维护一套统一的 YOLO 检测工程，服务于：

- Handyman
- Interactive Cleanup

最终只训练一个 27 类检测模型，并分别部署到两个任务的运行时包。

## 目录结构

```text
training/
  README.md
  DATA_COLLECTION.md
  classes.txt
  dataset.yaml
  data/
    raw/
      images/
      labels/
      classes.txt
      notes.json
    images/{train,val,test}/
    labels/{train,val,test}/
  scripts/
    extract_frames.py
    split_dataset.py
    train.py
  tests/
    test_training_scripts.py
  runs/
  exports/
  weights/
```

## 快速开始

1. 准备原始图片和 Label Studio 标注导出：

   ```text
   training/data/raw/
     images/
     labels/
     classes.txt
     notes.json
   ```

2. 确保 `training/data/raw/classes.txt` 与 `training/classes.txt` 完全一致。

3. 划分数据集：

   ```bash
   pixi run python training/scripts/split_dataset.py --clean
   ```

4. 使用 YOLO26 基线开始训练：

   ```bash
   pixi run python training/scripts/train.py \
     --epochs 120 \
     --batch 16 \
     --name unified_baseline
   ```

5. 训练完成后，将最佳权重分别复制到两个运行时包：

   ```bash
   cp training/runs/unified_baseline/weights/best.pt \
     src/handyman_vision_ros2/models/handyman_model.pt

   cp training/runs/unified_baseline/weights/best.pt \
     src/cleanup_vision_ros2/models/cleanup_model.pt
   ```

## YOLO26 基线

- `train.py` 默认模型是 `yolo26n.pt`
- 你可以直接使用 Ultralytics 模型别名，让框架自动解析/下载
- 如果你已经手动下载了权重，也可以显式传入：

  ```bash
  pixi run python training/scripts/train.py \
    --model training/weights/yolo26n.pt
  ```

## 27 类列表

统一类别表是 Handyman 26 类与 Cleanup 独有的 `spray_bottle` 的并集：

`apple, bear_doll, camera, canned_juice, cigarette, cubic_clock, dog_doll, empty_ketchup, empty_plastic_bottle, filled_ketchup, filled_plastic_bottle, ground_pepper, hourglass, nursing_bottle, pink_cup, rabbit_doll, rubiks_cube, salt, sauce, soysauce, spray_bottle, sugar, toy_car, toy_duck, toy_penguin, tumbler, white_cup`

## 注意事项

- 现在不再使用 `session` 子目录。
- `split_dataset.py` 会按图像级别随机切分 `train/val/test`。
- 因为取消了 session 切分，导出前必须主动删除大量近重复帧，否则验证集会偏乐观。
- `training/data/raw/notes.json` 目前主要作为 Label Studio 导出的伴随文件保留，训练脚本不依赖其中的字段。

# Label Interface模板
```html
<View>
  <Image name="image" value="$image"/>
  <RectangleLabels name="label" toName="image">
    <Label value="apple" background="#E63946" category="0"/>
    <Label value="bear_doll" background="#457B9D" category="1"/>
    <Label value="camera" background="#2A9D8F" category="2"/>
    <Label value="canned_juice" background="#E9C46A" category="3"/>
    <Label value="cigarette" background="#F4A261" category="4"/>
    <Label value="cubic_clock" background="#264653" category="5"/>
    <Label value="dog_doll" background="#E76F51" category="6"/>
    <Label value="empty_ketchup" background="#2EC4B6" category="7"/>
    <Label value="empty_plastic_bottle" background="#06D6A0" category="8"/>
    <Label value="filled_ketchup" background="#118AB2" category="9"/>
    <Label value="filled_plastic_bottle" background="#EF476F" category="10"/>
    <Label value="ground_pepper" background="#FFD166" category="11"/>
    <Label value="hourglass" background="#073B4C" category="12"/>
    <Label value="nursing_bottle" background="#669BBC" category="13"/>
    <Label value="pink_cup" background="#C1121F" category="14"/>
    <Label value="rabbit_doll" background="#52B788" category="15"/>
    <Label value="rubiks_cube" background="#FF9F1C" category="16"/>
    <Label value="salt" background="#9B5DE5" category="17"/>
    <Label value="sauce" background="#00BBF9" category="18"/>
    <Label value="soysauce" background="#00F5D4" category="19"/>
    <Label value="spray_bottle" background="#F15BB5" category="20"/>
    <Label value="sugar" background="#FEE440" category="21"/>
    <Label value="toy_car" background="#7B2CBF" category="22"/>
    <Label value="toy_duck" background="#3A0CA3" category="23"/>
    <Label value="toy_penguin" background="#4361EE" category="24"/>
    <Label value="tumbler" background="#4CC9F0" category="25"/>
    <Label value="white_cup" background="#7209B7" category="26"/>
  </RectangleLabels>
</View>
```