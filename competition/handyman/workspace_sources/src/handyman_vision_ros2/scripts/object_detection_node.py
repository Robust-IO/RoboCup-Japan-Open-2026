#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32MultiArray
from cv_bridge import CvBridge
from math import *
import numpy as np
import os
import tempfile
from PIL import Image as PILImage
import cv2

import tf2_ros
from transforms3d.euler import quat2euler
from geometry_msgs.msg import Pose, PoseStamped

from ultralytics import YOLO

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def _find_model_path():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    src_models = os.path.normpath(os.path.join(script_dir, '..', 'models'))

    share_models = None
    if get_package_share_directory is not None:
        try:
            share_models = os.path.join(
                get_package_share_directory('handyman_vision_ros2'), 'models')
        except Exception:
            pass

    vision_src_models = os.path.normpath(
        os.path.join(script_dir, '..', '..', 'vision_ros2', 'models'))

    search_dirs = []
    for d in (share_models, src_models, script_dir, vision_src_models):
        if d and os.path.normpath(d) not in [os.path.normpath(x) for x in search_dirs]:
            search_dirs.append(d)

    for name in ('last.pt', 'yolo12n.pt'):
        for d in search_dirs:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p

    return os.path.join(src_models, 'yolo12n.pt')


model = YOLO(_find_model_path())
print(f"Model loaded successfully: {model.model_name}")

head_cam_topic = '/hsrb/head_rgbd_sensor/rgb/image_raw'
hand_cam_topic = '/hsrb/hand_camera/image_raw'
depth_cam_topic = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'

class_names = [
    "apple",
    "bear_doll",
    "canned_juice",
    "cigarette",
    "clock",
    "dog_doll",
    "empty_ketchup",
    "empty_plastic_bottle",
    "filled_ketchup",
    "filled_plastic_bottle",
    "game_controller",
    "ground_pepper",
    "hourglass",
    "matryoshka",
    "nursing_bottle",
    "piggy_bank",
    "pink_cup",
    "rabbit_doll",
    "rubik-s_cube",
    "salt",
    "sauce",
    "soysauce",
    "spray_bottle",
    "sugar",
    "toy_car",
    "toy_duck",
    "toy_penguin",
    "tumbler",
    "white_cup",
    "white_side_table"
]

name_to_index = {name: index for index, name in enumerate(class_names)}
index_to_name = {index: name for index, name in enumerate(class_names)}


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.rgb_image = None
        self.hand_image = None
        self.depth_image = None
        self.unused_depth = False
        self.hand_ready = False
        self.detected = False

        self.detection_target = 'sugar'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, head_cam_topic, self.image_callback, 1)
        self.create_subscription(Image, hand_cam_topic, self.hand_cam_callback, 1)
        self.create_subscription(Image, depth_cam_topic, self.depth_image_callback, 1)
        self.create_subscription(String, '/detection_target', self.set_detection_target, 10)

        self.depth_pub = self.create_publisher(Float32, '/detection_depth', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/vision', 10)
        self.hand_pub = self.create_publisher(Int32MultiArray, '/hand_detection', 10)

        self.model = model
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.detection_loop)

    def image_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

        except Exception as e:
            self.get_logger().error('Error processing image: {}'.format(e))

    def hand_cam_callback(self, msg):
        try:
            self.hand_ready = False
            bombed_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.hand_image = cv2.rotate(bombed_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.hand_image = cv2.cvtColor(self.hand_image, cv2.COLOR_RGB2BGR)
            self.hand_ready = True
            self.detected = False

        except Exception as e:
            self.get_logger().error(f'Hand cam error: {e}')

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.unused_depth = True

    def set_detection_target(self, target):
        self.detection_target = target.data
        self.get_logger().info("target set to %s" % self.detection_target)

    def detection_loop(self):
        if self.hand_ready and not self.detected:
            try:
                target_class = int(name_to_index.get(self.detection_target))
                predictions = self.model.predict(self.hand_image, conf=0.1, classes=target_class, verbose=False, max_det=1)
                self.get_logger().info("Hand cam detection...")
                prediction = predictions[0]

                for box in prediction.boxes:
                    cls_name = index_to_name.get(int(box.cls[0].item()))
                    conf = round(box.conf[0].item(), 2)
                    box_dims = [int(x) for x in box.xywh[0].tolist()]
                    box_debug = [int(x) for x in box.xyxy[0].tolist()]

                    detection = Int32MultiArray()
                    detection.data = box_dims[:4]
                    cv2.rectangle(self.hand_image, (box_debug[0], box_debug[1]), (box_debug[2], box_debug[3]), (255, 0, 0))
                    self.hand_pub.publish(detection)

                cv2.imshow("detection_hand", self.hand_image)
                if cv2.waitKey(1) == ord('c'):
                    cv2.destroyAllWindows()

                self.detected = True
            except Exception as e:
                self.get_logger().error(f"Detection error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
