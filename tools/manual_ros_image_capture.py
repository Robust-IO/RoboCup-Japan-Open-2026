#!/usr/bin/env python3
"""Manually save selected frames from the HSR head RGB camera.

Space: save the frame currently visible in the OpenCV window.
Q or Esc: quit.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManualImageCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__('manual_image_capture')
        self.bridge = CvBridge()
        self.frame = None
        self.subscription = self.create_subscription(Image, topic, self.callback, 10)
        self.head_publisher = self.create_publisher(
            JointTrajectory, '/hsrb/head_trajectory_controller/command', 10)
        self.pan = 0.0
        self.tilt = -0.35

    def callback(self, message: Image) -> None:
        try:
            self.frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warning(f'Cannot convert camera frame: {error}')

    def move_head(self, pan_delta: float = 0.0, tilt_delta: float = 0.0,
                  reset: bool = False) -> None:
        if reset:
            self.pan, self.tilt = 0.0, -0.35
        else:
            self.pan = max(-1.5, min(1.5, self.pan + pan_delta))
            self.tilt = max(-0.8, min(0.3, self.tilt + tilt_delta))
        command = JointTrajectory()
        command.joint_names = ['head_pan_joint', 'head_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.pan, self.tilt]
        point.time_from_start = Duration(sec=1)
        command.points = [point]
        self.head_publisher.publish(command)
        self.get_logger().info(f'Head command: pan={self.pan:.2f}, tilt={self.tilt:.2f}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='Directory where selected PNG images are written')
    parser.add_argument('--topic', default='/hsrb/head_rgbd_sensor/rgb/image_rect_color')
    args = parser.parse_args()

    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    next_index = max((int(path.stem.split('_')[-1]) + 1
                      for path in output.glob('spray_*.png')
                      if path.stem.split('_')[-1].isdigit()), default=0)

    rclpy.init()
    node = ManualImageCapture(args.topic)
    window = 'HSR head RGB - A/D pan W/S tilt Space save R reset Q quit'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    node.get_logger().info(f'Waiting for {args.topic}; saving to {output}')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame is None:
                # OpenCV does not paint a newly-created window until its event
                # loop is serviced.  Keep a visible status window while ROS is
                # still waiting for the first camera message.
                waiting = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting, 'Waiting for HSR head-camera frames...',
                            (55, 225), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 220, 255), 2)
                cv2.putText(waiting, args.topic, (55, 265),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.52, (210, 210, 210), 1)
                cv2.imshow(window, waiting)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord('q'), 27):
                    break
                continue
            preview = node.frame.copy()
            cv2.putText(preview,
                        f'Saved:{next_index} Pan:{node.pan:.2f} Tilt:{node.tilt:.2f}  '
                        '[A/D]pan [W/S]tilt [Space]save [R]reset [Q]quit',
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord(' '):
                path = output / f'spray_{next_index:04d}.png'
                if cv2.imwrite(str(path), node.frame):
                    node.get_logger().info(f'Saved {path.name}')
                    next_index += 1
                else:
                    node.get_logger().error(f'Failed to save {path}')
            elif key == ord('a'):
                node.move_head(pan_delta=0.12)
            elif key == ord('d'):
                node.move_head(pan_delta=-0.12)
            elif key == ord('w'):
                node.move_head(tilt_delta=0.08)
            elif key == ord('s'):
                node.move_head(tilt_delta=-0.08)
            elif key == ord('r'):
                node.move_head(reset=True)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
