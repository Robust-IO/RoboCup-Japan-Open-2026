#!/usr/bin/env python3
"""Summarize localization and base commands in a phase-3 rosbag."""

import argparse
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {name: get_message(kind) for name, kind in topic_types.items()}
    transform_counts = Counter()
    base_trajectory = []
    amcl_trajectory = []
    commands = []

    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        seconds = timestamp * 1e-9
        if topic == "/tf":
            for transform in message.transforms:
                edge = (transform.header.frame_id, transform.child_frame_id)
                transform_counts[edge] += 1
                if edge == ("odom", "base_footprint"):
                    base_trajectory.append(
                        (seconds, transform.transform.translation.x,
                         transform.transform.translation.y,
                         yaw(transform.transform.rotation)))
        elif topic == "/amcl_pose":
            pose = message.pose.pose
            covariance = message.pose.covariance
            amcl_trajectory.append(
                (seconds, pose.position.x, pose.position.y, yaw(pose.orientation),
                 covariance[0], covariance[7], covariance[35]))
        elif topic == "/hsrb/command_velocity":
            commands.append(
                (seconds, message.linear.x, message.linear.y, message.angular.z))

    print("topics:")
    for name in sorted(topic_types):
        print(f"  {name}: {topic_types[name]}")
    print("tf edges:")
    for edge, count in transform_counts.most_common():
        print(f"  {edge[0]} -> {edge[1]}: {count}")

    def print_samples(label, samples, formatter):
        print(f"{label}: {len(samples)}")
        if not samples:
            return
        origin = samples[0][0]
        selected = [samples[0]]
        next_time = origin + 10.0
        for sample in samples[1:-1]:
            if sample[0] >= next_time:
                selected.append(sample)
                next_time = sample[0] + 10.0
        if len(samples) > 1:
            selected.append(samples[-1])
        for sample in selected:
            print(f"  t={sample[0] - origin:7.2f}s {formatter(sample)}")

    print_samples(
        "odom -> base_footprint samples", base_trajectory,
        lambda value: f"x={value[1]:8.3f} y={value[2]:8.3f} yaw={value[3]:7.3f}",
    )
    print_samples(
        "AMCL samples", amcl_trajectory,
        lambda value: (
            f"x={value[1]:8.3f} y={value[2]:8.3f} yaw={value[3]:7.3f} "
            f"var=({value[4]:.4f},{value[5]:.4f},{value[6]:.4f})"
        ),
    )
    print_samples(
        "velocity command samples", commands,
        lambda value: f"vx={value[1]:7.3f} vy={value[2]:7.3f} wz={value[3]:7.3f}",
    )
    if commands:
        nonzero = sum(
            abs(value[1]) + abs(value[2]) + abs(value[3]) > 1e-4
            for value in commands)
        print(f"nonzero velocity commands: {nonzero}/{len(commands)}")


if __name__ == "__main__":
    main()
