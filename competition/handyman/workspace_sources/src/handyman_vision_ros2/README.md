# RCUP Vision 项目

## 项目概述

RCUP Vision 是为 **RoboCup@Home Simulation Malaysia 2025** 比赛开发的视觉识别包，作为 handyman-ros 项目的核心依赖，提供物体检测和识别功能。

## 与 handyman-ros 项目的关系

本包是 handyman-ros 项目的关键依赖组件，负责：

- **物体检测**：识别比赛中的30+种日常物品
- **位置定位**：提供检测到的物体位置信息
- **手部检测**：为抓取操作提供视觉反馈
- **目标识别**：根据 handyman-ros 发送的目标信息进行针对性检测

## 主要功能

### 1. 物体检测
- 使用 YOLO 模型进行实时物体检测
- 支持多种物品类别：水果、玩具、日用品、厨具等
- 提供检测框的坐标和置信度信息

### 2. 多摄像头支持
- **头部摄像头**：用于环境感知和物体搜索
- **手部摄像头**：用于近距离抓取操作
- **深度摄像头**：提供深度信息用于距离计算

### 3. ROS 接口
- **图像订阅**：从 HSR 机器人获取摄像头数据
- **位置发布**：发布检测到的物体位置（`/vision` 话题）
- **手部检测**：发布手部检测信息（`/hand_detection` 话题）
- **目标接收**：接收 handyman-ros 的目标物体信息（`/detection_target` 话题）

## 项目结构

```
rcup_vision/
├── CMakeLists.txt          # 构建配置文件
├── package.xml             # 包依赖信息
├── requirements.txt        # Python 依赖
└── scripts/
    ├── last.pt            # YOLO 模型文件
    └── object_detection_node.py  # 主检测节点
```

## 使用方式

### 1. 安装依赖
```bash
pip install -r requirements.txt
```

### 2. 启动节点
本包由 handyman-ros 的 `hsr_nav.launch` 自动启动，无需手动启动。

### 3. 话题列表
- `/hsrb/head_rgbd_sensor/rgb/image_raw`：头部摄像头图像
- `/hsrb/hand_camera/image_raw`：手部摄像头图像
- `/vision`：检测结果位置信息
- `/hand_detection`：手部检测信息
- `/detection_target`：目标物体信息

## 依赖项

### ROS 依赖
- rospy
- sensor_msgs
- std_msgs
- cv_bridge
- tf2_ros
- geometry_msgs

### Python 依赖
- opencv-python
- numpy
- Pillow
- ultralytics (YOLO)
- PyYAML

## 已知问题

由于视觉识别技术的限制，本包存在以下问题：

1. **近距离识别不稳定**：物体距离较近时检测精度下降
2. **角度敏感**：仰拍和俯拍情况下识别性能不佳
4. **遮挡问题**：物体部分遮挡时识别困难

这些限制直接影响了 handyman-ros 项目的抓取成功率。

## 许可证

AGPL-3.0 (GNU Affero General Public License v3.0)

**注意**：本项目使用了 Ultralytics (AGPL-3.0)，因此整个项目采用 AGPL-3.0 许可证。

---

**注意**：此包专为 handyman-ros 项目设计，作为其视觉识别模块，不独立使用。