# Unified YOLO Training Guide

本文件只说明统一 `training/` 工程的使用方法、目录结构、27 类编号和注意事项。

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
    images/
      train/
      val/
      test/
    labels/
      train/
      val/
      test/
  scripts/
    extract_frames.py
    split_dataset.py
    train.py
  runs/
  exports/
  weights/
```

## 使用说明

### 1. 准备原始数据

原始数据目录直接使用 Label Studio 导出的 YOLO 结构：

```text
training/data/raw/
  images/
  labels/
  classes.txt
  notes.json
```

要求：

- `images/` 中存放全部待训练图片
- `labels/` 中存放同名 YOLO 标签文件
- `classes.txt` 必须与 `training/classes.txt` 完全一致
- `notes.json` 保留 Label Studio 导出元数据

### 2. 从视频抽帧

如果你先从视频生成图片，可以使用：

```bash
pixi run python training/scripts/extract_frames.py \
  --video /path/to/video.mp4 \
  --prefix sugar_table_near
```

默认输出到：

```text
training/data/raw/images/
```

如果要一次处理多个视频，可以重复传入 `--video`。

### 3. 划分训练集/验证集/测试集

在 `training/data/raw/` 准备完成后，运行：

```bash
pixi run python training/scripts/split_dataset.py --clean
```

默认会：

- 从 `training/data/raw/images/` 读取图片
- 从 `training/data/raw/labels/` 读取标签
- 校验 `training/data/raw/classes.txt` 与 `training/classes.txt`
- 将数据划分到：

```text
training/data/images/{train,val,test}/
training/data/labels/{train,val,test}/
```

如果需要自定义比例，可以使用：

```bash
pixi run python training/scripts/split_dataset.py \
  --clean \
  --train-ratio 0.7 \
  --val-ratio 0.2 \
  --test-ratio 0.1
```

### 4. 开始训练

统一训练入口：

```bash
pixi run python training/scripts/train.py \
  --epochs 120 \
  --batch 16 \
  --name unified_baseline
```

默认设置：

- 数据配置：`training/dataset.yaml`
- 输出目录：`training/runs`
- 基准模型：`yolo26n.pt`

如果你已经手动准备了本地权重，也可以显式指定：

```bash
pixi run python training/scripts/train.py \
  --model training/weights/yolo26n.pt \
  --epochs 120 \
  --batch 16 \
  --name unified_baseline
```

### 5. 训练结果位置

训练输出默认位于：

```text
training/runs/<name>/
```

常用文件：

- `training/runs/<name>/weights/best.pt`
- `training/runs/<name>/weights/last.pt`

如果要部署到两个任务的运行时包，可分别复制：

```bash
cp training/runs/unified_baseline/weights/best.pt \
  src/handyman_vision_ros2/models/handyman_model.pt

cp training/runs/unified_baseline/weights/best.pt \
  src/cleanup_vision_ros2/models/cleanup_model.pt
```

### 6. 如何阅读训练结果

如果你第一次看 YOLO 训练输出，建议先记住一件事：

- `best.pt` 是训练过程中验证集表现最好的权重，通常优先拿它做部署和测试。
- `last.pt` 是最后一轮训练结束时的权重，只表示“训练停在这里”，不一定是最优。

常见结果文件及含义如下。

#### `results.csv`

这是每个 epoch 的完整数值记录，适合精确比较。

重点列：

- `train/box_loss`：训练集框回归损失，越低通常越好。
- `train/cls_loss`：训练集分类损失，越低通常越好。
- `train/dfl_loss`：训练集定位分布损失，越低通常越好。
- `val/box_loss`、`val/cls_loss`、`val/dfl_loss`：验证集对应损失，通常比训练集更值得关注。
- `metrics/precision(B)`：查准率。模型报出来的框里，有多少是真的。
- `metrics/recall(B)`：查全率。真实目标里，有多少被找到了。
- `metrics/mAP50(B)`：在 IoU=0.50 条件下的平均检测精度，比较宽松。
- `metrics/mAP50-95(B)`：在 IoU=0.50 到 0.95 多个阈值上平均后的 mAP，更严格，也更适合拿来比较模型质量。

入门时可以这样理解：

- `precision` 低：误检多，模型容易“看错”。
- `recall` 低：漏检多，模型容易“看漏”。
- `mAP50` 高但 `mAP50-95` 一般：说明大致能找到目标，但框的位置还不够准。
- 训练损失持续下降而验证指标不再提升：可能已经接近收敛，继续训练收益不大。

#### `results.png`

这是 `results.csv` 的可视化版本，适合先快速看趋势。

建议这样读：

1. 先看三条训练损失和三条验证损失是否整体下降。
2. 再看 `precision`、`recall`、`mAP50`、`mAP50-95` 是否整体上升并逐渐趋于平稳。
3. 如果后期曲线已经明显变平，通常说明模型基本收敛。
4. 如果训练损失继续明显下降，但验证指标开始震荡甚至变差，要警惕过拟合。

#### `BoxPR_curve.png`

这是 Precision-Recall 曲线。

- 曲线越靠右上角越好。
- 图例中的每个数字通常表示该类别在 `mAP@0.5` 下的表现。
- 如果某条类别曲线明显比其他类别差，说明该类更难，或者该类数据质量/数量不足。

#### `BoxF1_curve.png`

这是 F1 与置信度阈值的关系图。

- F1 综合了 `precision` 和 `recall`。
- 曲线峰值对应一个比较均衡的置信度阈值。
- 如果之后你要手动调部署时的 `conf` 参数，这张图很有参考价值。

简单说：

- 想少误检，可以把置信度阈值调高。
- 想少漏检，可以把置信度阈值调低。
- 想先找一个折中点，可以从 F1 峰值附近开始试。

#### `BoxP_curve.png` 和 `BoxR_curve.png`

这两张图分别展示 Precision 和 Recall 随置信度阈值变化的情况。

- `BoxP_curve` 通常随着阈值变高而上升。
- `BoxR_curve` 通常随着阈值变高而下降。

它们本质上是在告诉你：

- 阈值不是越高越好，也不是越低越好。
- 部署时要根据任务目标选择取舍。
- RoboCup 场景如果更怕漏掉物体，通常会更重视 `recall`。
- 如果更怕把背景误识别成物体，通常会更重视 `precision`。

#### `confusion_matrix.png` 和 `confusion_matrix_normalized.png`

这两张图用来查看类别之间的混淆情况。

- 对角线越深，说明该类越容易被正确识别。
- 非对角线有明显颜色，说明两个类别容易互相混淆。
- `normalized` 版本更适合看比例，便于发现“哪一类最容易错”。

阅读时要特别注意 `background`：

- 最下面一行如果某列数值较高，表示该真实类别经常被漏检成背景。
- 最右边一列如果某行数值较高，表示模型经常凭空报出该类别，也就是误检偏多。

#### `val_batch*_labels.jpg` 和 `val_batch*_pred.jpg`

这两类图最适合做直观检查。

- `labels` 图显示验证集真实标注。
- `pred` 图显示模型预测结果。

对比时重点看：

- 框有没有明显偏移。
- 小物体是否容易漏掉。
- 遮挡、远距离、边缘位置的目标是否稳定。
- 相似类别是否会被认错。
- 置信度是否普遍合理。

#### `labels.jpg`

这张图用于看数据分布，而不是看模型本身。

- 左上角柱状图反映各类别样本数量是否均衡。
- 右上角框分布图反映目标框的大致尺寸和长宽比。
- 左下和右下反映目标中心点与大小分布。

如果你看到某些类别样本明显太少，即使总指标很好，也要谨慎解读那一类的结果。

### 7. 建议的阅读顺序

第一次读一个训练结果时，建议按下面顺序：

1. 先看 `args.yaml`，确认这次训练的 `model`、`epochs`、`batch`、`imgsz` 是什么，避免拿不同配置硬比较。
2. 再看 `results.png`，判断训练有没有正常收敛。
3. 然后看 `results.csv`，记录最好的一轮和最后一轮的 `precision`、`recall`、`mAP50`、`mAP50-95`。
4. 接着看 `BoxF1_curve.png`，了解部署时置信度阈值大概该从哪里起步。
5. 再看 `BoxPR_curve.png`，找出表现最弱的类别。
6. 然后看 `confusion_matrix_normalized.png`，确认是“漏检多”还是“混类多”。
7. 最后看 `val_batch*_pred.jpg`，确认模型在真实图像上的框是否看起来可靠。

如果你只想用 1 分钟做快速判断，最少看这四个：

- `results.png`
- `results.csv`
- `confusion_matrix_normalized.png`
- `val_batch0_pred.jpg`

### 8. 导出到 X-AnyLabeling 自动标注

如果你想把训练好的检测模型拿到 X-AnyLabeling 里做自动化标注，不能只给它一个 `.pt` 文件。

对当前使用的 `yolo26` 适配器来说，X-AnyLabeling 需要的是：

- 一个 `ONNX` 模型文件
- 一个描述模型信息的 `config.yaml`

也就是说，实际流程是：

```text
训练得到的 best.pt -> 导出为 model.onnx -> 生成 config.yaml
```

这里要特别注意：

- 导出的输入应该是训练结果权重，例如 `training/runs/simple-20/weights/best.pt`
- 不是训练基座模型，例如主目录下的 `yolo26n.pt`
- `yolo26n.pt` 只是训练起点，`best.pt` 才是你真正训练后的模型

#### 导出命令

使用下面的命令可以把 `simple-20` 的最佳权重导出成 X-AnyLabeling 可加载的模型目录：

```bash
pixi run python training/scripts/export_xanylabeling.py \
  --weights training/runs/simple-20/weights/best.pt \
  --output-dir training/exports/xanylabeling/simple-20 \
  --display-name "simple-20 auto labeler"
```

如果你想按训练运行名来导出，也可以使用：

```bash
pixi run python training/scripts/export_xanylabeling.py \
  --run-name simple-20 \
  --output-dir training/exports/xanylabeling/simple-20 \
  --display-name "simple-20 auto labeler"
```

默认类别列表来自：

- `training/classes.txt`

导出脚本会把类别名写入 `config.yaml`，因此这里的类顺序必须和训练时保持完全一致。

#### 导出结果结构

导出完成后，目录应类似于：

```text
training/exports/xanylabeling/simple-20/
  model.onnx
  config.yaml
```

其中：

- `model.onnx` 是给 X-AnyLabeling 加载的检测模型
- `config.yaml` 是模型配置文件

生成的 `config.yaml` 会按 X-AnyLabeling 的 `yolo26` 自定义模型格式写入以下关键字段：

- `type: yolo26`
- `display_name`
- `model_path: model.onnx`
- `iou_threshold`
- `conf_threshold`
- `max_det`
- `classes`

#### 在 X-AnyLabeling 中使用

1. 打开 X-AnyLabeling。
2. 选择加载自定义模型。
3. 选中导出目录中的 `config.yaml`。
4. 软件会根据 `config.yaml` 继续加载同目录下的 `model.onnx`。

如果加载失败，优先检查这几项：

- `config.yaml` 中的 `model_path` 是否仍然指向 `model.onnx`
- `model.onnx` 是否和 `config.yaml` 位于同一个目录
- `classes` 顺序是否与训练时一致
- 导出时是否确实使用了训练后的 `best.pt`，而不是基座模型

## 27 类物品编号

统一类别顺序必须固定如下：

| ID | 类别 | ID | 类别 | ID | 类别 |
|----|------|----|------|----|------|
| 0 | apple | 9 | filled_ketchup | 18 | sauce |
| 1 | bear_doll | 10 | filled_plastic_bottle | 19 | soysauce |
| 2 | camera | 11 | ground_pepper | 20 | spray_bottle |
| 3 | canned_juice | 12 | hourglass | 21 | sugar |
| 4 | cigarette | 13 | nursing_bottle | 22 | toy_car |
| 5 | cubic_clock | 14 | pink_cup | 23 | toy_duck |
| 6 | dog_doll | 15 | rabbit_doll | 24 | toy_penguin |
| 7 | empty_ketchup | 16 | rubiks_cube | 25 | tumbler |
| 8 | empty_plastic_bottle | 17 | salt | 26 | white_cup |

对应的完整类表文件见：

- `training/classes.txt`
- `training/data/raw/classes.txt`
- `training/dataset.yaml`

## 标注格式

统一使用 YOLO 检测格式：

```text
class_id center_x center_y width height
```

其中：

- `class_id` 必须使用上面的 27 类编号
- 坐标必须归一化到 `[0, 1]`
- 每张图片对应一个同名 `.txt`
- 没有目标的负样本可以保留空 `.txt`

## 注意事项

- 现在不再使用 `session/` 目录结构。
- `training/data/raw/` 必须是扁平的 Label Studio 导出结构。
- `split_dataset.py` 现在按图像级别随机切分，不再按 session 切分。
- 因为取消了 session 切分，导出前必须主动删除大量近重复帧，否则 `val/test` 会过于乐观。
- `raw/classes.txt` 与 `training/classes.txt` 只要顺序或命名有任何不一致，`split_dataset.py` 就会直接报错。
- 默认训练基准模型已经改为 `yolo26n.pt`，不再使用 `yolo12n.pt`。
- 所有命令都应在 `pixi` 环境中运行，例如使用 `pixi run python ...`。
