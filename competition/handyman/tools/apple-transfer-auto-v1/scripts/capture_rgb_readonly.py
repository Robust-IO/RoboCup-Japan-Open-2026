"""System ROS Python subscriber. Never publishes control or competition messages."""
import argparse
import json
from pathlib import Path
import struct
import time
import zlib


def encode_png(msg):
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
    if channels is None:
        raise ValueError('Unsupported encoding: ' + msg.encoding)
    if not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096:
        raise ValueError('Invalid dimensions')
    data = bytes(msg.data)
    if msg.step < msg.width * channels or len(data) < msg.step * msg.height:
        raise ValueError('Invalid stride or payload')
    rows = bytearray()
    for y in range(msg.height):
        row = data[y * msg.step:y * msg.step + msg.width * channels]
        rows.append(0)
        if msg.encoding == 'rgb8':
            rows.extend(row)
        else:
            for x in range(msg.width):
                pixel = row[x * channels:x * channels + 3]
                rows.extend(pixel[::-1] if msg.encoding.startswith('bgr') else pixel)
    def chunk(kind, content):
        return struct.pack('>I', len(content)) + kind + content + struct.pack('>I', zlib.crc32(kind + content) & 0xffffffff)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', msg.width, msg.height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(bytes(rows))) + chunk(b'IEND', b''))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--seconds', type=int, default=1800)
    parser.add_argument('--max-frames', type=int, default=300)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    rclpy.init()
    node = Node('handyman_rgb_readonly_capture')
    count, last = 0, -10.0
    def image_callback(msg):
        nonlocal count, last
        now = time.monotonic()
        if now - last < 2 or count >= args.max_frames:
            return
        stem = f'{count:06d}'
        png = encode_png(msg)
        (args.output / (stem + '.png')).write_bytes(png)
        metadata = {'image': stem + '.png', 'width': msg.width, 'height': msg.height,
                    'encoding': msg.encoding, 'frame_id': msg.header.frame_id,
                    'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec,
                    'received_unix': time.time(), 'orientation': 'ROS rows unchanged'}
        pending = args.output / (stem + '.tmp')
        pending.write_text(json.dumps(metadata, indent=2))
        pending.replace(args.output / (stem + '.json'))
        count += 1
        last = now
        print('Captured', stem, flush=True)
    def info_callback(msg):
        if (args.output / 'camera_info.json').exists():
            return
        (args.output / 'camera_info.json').write_text(json.dumps({'width': msg.width, 'height': msg.height,
            'frame_id': msg.header.frame_id, 'k': list(msg.k), 'd': list(msg.d)}, indent=2))
    node.create_subscription(Image, '/hsrb/head_rgbd_sensor/rgb/image_raw', image_callback, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/hsrb/head_rgbd_sensor/rgb/camera_info', info_callback, qos_profile_sensor_data)
    print('READY: read-only RGB capture, every 2 seconds, max', args.max_frames, 'frames, timeout', args.seconds, flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline and count < args.max_frames:
            rclpy.spin_once(node, timeout_sec=1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print('Capture finished:', count, flush=True)


if __name__ == '__main__':
    main()
