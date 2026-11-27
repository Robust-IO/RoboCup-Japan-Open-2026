# vision_ros2 — ROS 2 Humble 移植版

原稿（ROS1 Noetic + YOLOv8）完整移植到 ROS2 Humble，YOLO 升级为 YOLOv12。

本包负责物体视觉检测，配合 `handyman` 包使用。

---

## 目录

1. [功能说明](#功能说明)
2. [前置条件与安装](#前置条件与安装)
3. [准备模型文件](#准备模型文件)
4. [编译](#编译)
5. [运行](#运行)
6. [话题接口](#话题接口)
7. [支持的物体类别](#支持的物体类别)
8. [配置参数](#配置参数)
9. [常见问题](#常见问题)
10. [包结构](#包结构)

---

## 功能说明

- **手部相机实时检测**：订阅 `/hsrb/hand_camera/image_raw`，对画面中的目标物体进行 YOLOv12 检测，发布检测框坐标到 `/hand_detection`
- **深度相机集成**：订阅深度图像，可计算目标距离并发布 3D 位姿到 `/vision`
- **头部相机**：订阅头部 RGB 相机，检测代码已保留但默认注释（可按需启用）
- **动态目标切换**：通过 `/detection_target` 话题随时切换要检测的物体类别
- **30 类物体识别**

---

## 前置条件与安装

### 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ament-index-python \
  python3-tf-transformations
```

### Python 依赖（pixi）

```bash
# 在工作空间根目录安装 Python 依赖
cd ~/ros2_ws
pixi install
```

主要依赖：
- `ultralytics>=8.3.0` — YOLOv12 推理框架
- `opencv-python>=4.8.0` — 图像处理
- `numpy>=1.24,<2` — 与 ROS Humble 兼容的数值库

运行节点时，先 `source` ROS 与工作空间，再进入 `pixi shell`：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
pixi shell
```

---

## 准备模型文件

视觉节点需要训练好的 YOLOv12 权重文件。

**放置位置：**

```bash
~/ros2_ws/src/vision_ros2/models/last.pt
```

**放置步骤：**

```bash
# 将模型文件复制到 models 目录
cp /path/to/your/trained_model.pt ~/ros2_ws/src/vision_ros2/models/last.pt

# 放置后必须重新编译，让 colcon 将模型安装到正确路径
cd ~/ros2_ws
colcon build --packages-select vision_ros2
source install/setup.bash
```

**没有模型文件时的行为：**

节点会自动回落到以下位置查找：
1. `share/vision_ros2/models/last.pt`（colcon install 路径）
2. 脚本同级 `../models/last.pt`（开发源码目录）
3. `share/vision_ros2/models/yolo12n.pt` 或源码目录中的 `yolo12n.pt`
4. 以上都不存在则尝试在线下载 `yolo12n.pt` 预训练权重（需联网，检测精度低于自定义模型）

---

## 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_ros2
source install/setup.bash
```

或与 handyman 一起编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 运行

### 单独运行视觉节点

```bash
ros2 launch vision_ros2 vision.launch.py
```

### 设置检测目标（另开终端）

```bash
# 告诉节点要检测什么物体
ros2 topic pub /detection_target std_msgs/msg/String "data: 'apple'" --once
```

### 查看检测结果

```bash
# 检测框坐标 [cx, cy, w, h]（像素单位，手部相机画面坐标）
ros2 topic echo /hand_detection

# 目标 3D 位姿（map 坐标系，需深度图像）
ros2 topic echo /vision

# 目标距离（毫米）
ros2 topic echo /detection_depth
```

### 与 handyman 联合运行（正式比赛）

视觉节点已集成在 `hsr_nav.launch.py` 中，无需单独启动：

```bash
ros2 launch handyman hsr_nav.launch.py
```

---

## 话题接口

### 订阅

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/hsrb/hand_camera/image_raw` | `sensor_msgs/Image` | 手部相机图像（主要检测源） |
| `/hsrb/head_rgbd_sensor/rgb/image_raw` | `sensor_msgs/Image` | 头部 RGB 相机（检测代码已保留，默认注释） |
| `/hsrb/head_rgbd_sensor/depth_registered/image_raw` | `sensor_msgs/Image` | 深度图像，用于计算 3D 位姿 |
| `/detection_target` | `std_msgs/String` | 设置要检测的物体名称（见类别列表） |

### 发布

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/hand_detection` | `std_msgs/Int32MultiArray` | 检测框 `[cx, cy, w, h]`（像素），handyman 用此驱动视觉伺服 |
| `/vision` | `geometry_msgs/PoseStamped` | 目标 3D 位姿（map 坐标系） |
| `/detection_depth` | `std_msgs/Float32` | 目标距离（毫米） |

**`/hand_detection` 数据格式说明：**

```
[cx, cy, w, h]
 cx — 检测框中心 x 坐标（手部相机画面，0~480 像素）
 cy — 检测框中心 y 坐标（0~640 像素，相机已旋转90度矫正）
  w — 检测框宽度
  h — 检测框高度
```

handyman 用 `w >= 450` 判断是否足够近可以抓取。

---

## 支持的物体类别

共 30 类，检测目标名称需与以下列表精确匹配（区分大小写）：

| 序号 | 类别名 | 序号 | 类别名 |
|------|--------|------|--------|
| 0 | apple | 15 | piggy_bank |
| 1 | bear_doll | 16 | pink_cup |
| 2 | canned_juice | 17 | rabbit_doll |
| 3 | cigarette | 18 | rubik-s_cube |
| 4 | clock | 19 | salt |
| 5 | dog_doll | 20 | sauce |
| 6 | empty_ketchup | 21 | soysauce |
| 7 | empty_plastic_bottle | 22 | spray_bottle |
| 8 | filled_ketchup | 23 | sugar |
| 9 | filled_plastic_bottle | 24 | toy_car |
| 10 | game_controller | 25 | toy_duck |
| 11 | ground_pepper | 26 | toy_penguin |
| 12 | hourglass | 27 | tumbler |
| 13 | matryoshka | 28 | white_cup |
| 14 | nursing_bottle | 29 | white_side_table |

> **注意：** `rubik-s_cube` 含连字符，发送检测目标时需精确匹配：
> ```bash
> ros2 topic pub /detection_target std_msgs/msg/String "data: 'rubik-s_cube'" --once
> ```

---

## 配置参数

配置文件位于 `config/vision_params.yaml`，主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `confidence_threshold` | 0.5 | 头部相机置信度阈值 |
| `hand_confidence_threshold` | 0.1 | 手部相机置信度阈值（较低以提高召回率） |
| `max_detections` | 1 | 每帧最多检测数量 |
| `use_gpu` | true | 是否使用 GPU 加速（无 GPU 时自动降级到 CPU） |

> 注意：`config/vision_params.yaml` 目前仅作参考，节点代码中参数为硬编码，如需修改请直接编辑 `scripts/object_detection_node.py`。

---

## 常见问题

**Q: 节点报错 `cv_bridge` 找不到**  
A: 执行 `sudo apt install ros-humble-cv-bridge`。

**Q: 报错 `No module named 'tf_transformations'`**  
A: 执行 `sudo apt install python3-tf-transformations`。

**Q: 节点启动后没有检测结果输出**  
A: 检查以下几点：
1. 是否已通过 `/detection_target` 设置了检测目标
2. 手部相机话题 `/hsrb/hand_camera/image_raw` 是否有数据（`ros2 topic hz /hsrb/hand_camera/image_raw`）
3. 模型文件是否正确放置

**Q: 报错 `DISPLAY not set` 或 `imshow` 崩溃**  
A: 节点会自动检测 `$DISPLAY` 环境变量，无显示器时自动跳过 `imshow`，不影响话题发布。

**Q: GPU 不可用 / CUDA 报错**  
A: ultralytics 会自动降级到 CPU，无需额外处理。若要强制 CPU，编辑 `object_detection_node.py` 在 `YOLO(MODEL_PATH)` 后添加 `model.to('cpu')`。

**Q: 检测精度低 / 检测不到目标**  
A: 当前使用的模型精度取决于训练数据。可调低 `conf=0.1` 阈值（已是默认值），或检查手部相机是否对准目标。

---

## 包结构

```
vision-ros2/              （复制到工作空间时命名为 vision_ros2）
├── scripts/
│   └── object_detection_node.py   # 主检测节点（YOLOv12，订阅3路相机，发布检测结果）
├── launch/
│   └── vision.launch.py           # 单独启动视觉节点
├── config/
│   └── vision_params.yaml         # 参数配置参考文件
├── models/
│   └── last.pt                    # 放置训练好的 YOLO 模型（需手动添加）
├── CMakeLists.txt
├── package.xml
└── README.md
```
