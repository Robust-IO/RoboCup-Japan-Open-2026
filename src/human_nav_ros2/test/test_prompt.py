"""Offline launch wiring and isolated ROS protocol tests; no Unity bridges."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
sys.dont_write_bytecode = True
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

# Keep the test node and child controller away from competition participants.
os.environ["ROS_DOMAIN_ID"] = "96"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
import rclpy
from ament_index_python.packages import get_package_prefix
from human_navigation_msgs.msg import (
    HumanNaviMsg, HumanNaviGuidanceMsg, HumanNaviTaskInfo,
    HumanNaviAvatarStatus, HumanNaviObjectInfo, HumanNaviObjectStatus,
)


class PromptTests(unittest.TestCase):
    def test_both_launch_files_forward_boolean_and_default_true(self):
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        for name in ("sample_launch.py", "sample_with_rviz_launch.py"):
            path = Path(__file__).resolve().parents[1] / "launch" / name
            spec = importlib.util.spec_from_file_location("launch_under_test", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            declarations = [x for x in module.generate_launch_description().entities
                            if isinstance(x, DeclareLaunchArgument)]
            for value in (None, "true", "false", "True", "False"):
                context = LaunchContext()
                for declaration in declarations:
                    declaration.execute(context)
                if value is not None:
                    context.launch_configurations["prompt"] = value
                captured = []
                with patch.object(module, "Node", side_effect=lambda **kwargs: captured.append(kwargs)):
                    module._launch_setup(context)
                controller = next(x for x in captured if x.get("executable") == "human_navigation_sample")
                actual = controller["parameters"][0]["prompt"].evaluate(context)
                self.assertIs(actual, value is None or value.lower() == "true")

    def run_controller(self, enabled):
        rclpy.init()
        node = rclpy.create_node("prompt_test_moderator")
        guidance, protocol = [], []
        to_robot = node.create_publisher(HumanNaviMsg, "/human_navigation/message/to_robot", 10)
        tasks = node.create_publisher(HumanNaviTaskInfo, "/human_navigation/message/task_info", 10)
        avatars = node.create_publisher(HumanNaviAvatarStatus, "/human_navigation/message/avatar_status", 10)
        objects = node.create_publisher(HumanNaviObjectStatus, "/human_navigation/message/object_status", 10)
        def send(name):
            to_robot.publish(HumanNaviMsg(message=name, detail=""))
        def receive(msg):
            protocol.append(msg.message)
            if msg.message == "Get_speech_state":
                to_robot.publish(HumanNaviMsg(message="Speech_state", detail="Not_speaking"))
            if msg.message == "Get_object_status":
                status = HumanNaviObjectStatus()
                status.target_object.name = "cup"
                status.target_object.position.x = 100.
                objects.publish(status)
        node.create_subscription(HumanNaviMsg, "/human_navigation/message/to_moderator", receive, 10)
        node.create_subscription(HumanNaviGuidanceMsg, "/human_navigation/message/guidance_message",
                                 lambda msg: guidance.append(msg.message), 10)
        avatar = HumanNaviAvatarStatus()
        avatar.body.orientation.w = 1.
        def pump(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                avatars.publish(avatar)
                rclpy.spin_once(node, timeout_sec=.05)
        exe = Path(get_package_prefix("human_nav_ros2")) / "lib/human_nav_ros2/human_navigation_sample"
        command = [str(exe), "--ros-args", "-p", "guidance_min_interval_sec:=0.01",
                   "-p", "speech_state_poll_interval_sec:=0.01",
                   "-p", "direction_hint_interval_sec:=30.0"]
        if enabled is not None:
            command += ["-p", "prompt:=" + str(enabled).lower()]
        with tempfile.TemporaryFile(mode="w+") as log:
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 10
                while "I_am_ready" not in protocol and time.monotonic() < deadline:
                    send("Are_you_ready?")
                    pump(.2)
                    self.assertIsNone(child.poll(), "controller exited during startup")
                log.flush(); log.seek(0)
                self.assertIn("I_am_ready", protocol, log.read())
                task = HumanNaviTaskInfo()
                task.environment_id = "test"
                task.target_object.name = "cup"
                task.target_object.position.x = 1.
                task.target_object.position.z = .8
                task.destination.position.x = 3.
                task.destination.position.z = .8
                task.destination.orientation.w = 1.
                task.destination.size.x = task.destination.size.y = task.destination.size.z = 1.
                table = HumanNaviObjectInfo(name="table")
                table.position.x = 1.
                table.position.z = .5
                task.furniture = [table]
                tasks.publish(task)
                pump(1.5)
                send("Guidance_request")
                pump(.7)
                avatar.object_in_left_hand = "wrong_bottle"
                pump(.7)
                avatar.object_in_left_hand = ""
                pump(.5)
                avatar.object_in_left_hand = "cup"
                avatar.is_target_object_in_left_hand = True
                pump(2.5)
                avatar.object_in_left_hand = ""
                avatar.is_target_object_in_left_hand = False
                pump(2.)
                send("Task_succeeded")
                pump(.7)
                # A new session must retain the startup switch.
                send("Go_to_next_session")
                pump(.3)
                send("Are_you_ready?")
                pump(.3)
                tasks.publish(task)
                send("Guidance_request")
                pump(.7)
                self.assertIsNone(child.poll())
                if enabled is False:
                    self.assertEqual(guidance, [])
                    self.assertNotIn("Get_speech_state", protocol)
                else:
                    self.assertTrue(any("meters away" in text for text in guidance), guidance)
                    self.assertTrue(any("not the object" in text for text in guidance), guidance)
                    self.assertIn("Task completed.", guidance)
            finally:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                node.destroy_node()
                rclpy.shutdown()
            log.seek(0)
            text = log.read()
            if enabled is False:
                self.assertNotIn("Polished guidance bases", text)
                self.assertNotIn("Send guide message:", text)
            self.assertEqual(child.returncode, 0, text[-4000:])

    def test_default_prompts_enabled(self):
        self.run_controller(None)

    def test_explicit_true_preserves_prompts(self):
        self.run_controller(True)

    def test_false_suppresses_all_prompt_events(self):
        self.run_controller(False)


if __name__ == "__main__":
    unittest.main()
