import math
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.util
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from handyman_msgs.msg import HandymanMsg
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


@pytest.mark.launch_test
def generate_test_description():
    coordinator = launch_ros.actions.Node(
        package='handyman_rebuild_ros2',
        executable='handyman_coordinator',
        output='screen',
        parameters=[{
            'navigation.maximum_attempts': 3,
            'navigation.goal_timeout_sec': 0.5,
            'navigation.server_wait_timeout_sec': 2.0,
        }],
    )
    return launch.LaunchDescription([
        coordinator,
        launch_testing.actions.ReadyToTest(),
        launch_testing.util.KeepAliveProc(),
    ]), {'coordinator': coordinator}


class TestNavigationRoundTrip(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fake_nav2_and_moderator')
        cls.executor = MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        cls.goal_positions = []
        cls.received = []
        cls.transform_broadcaster = StaticTransformBroadcaster(cls.node)
        cls.action_server = ActionServer(
            cls.node,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=cls.execute_goal,
        )
        cls.publisher = cls.node.create_publisher(
            HandymanMsg, '/handyman/message/to_robot', 10)
        cls.subscription = cls.node.create_subscription(
            HandymanMsg,
            '/handyman/message/to_moderator',
            lambda message: cls.received.append(message.message),
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.action_server.destroy()
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=2.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def execute_goal(cls, goal_handle):
        pose = goal_handle.request.pose.pose
        cls.goal_positions.append((pose.position.x, pose.position.y))
        result = NavigateToPose.Result()
        if len(cls.goal_positions) == 1:
            time.sleep(1.5)
            goal_handle.abort()
            return result

        transform = TransformStamped()
        transform.header.stamp = cls.node.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.rotation = pose.orientation
        cls.transform_broadcaster.sendTransform(transform)
        time.sleep(0.2)
        goal_handle.succeed()
        return result

    @classmethod
    def publish(cls, event, detail=''):
        message = HandymanMsg()
        message.message = event
        message.detail = detail
        cls.publisher.publish(message)

    @classmethod
    def wait_for(cls, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def test_timed_out_goal_uses_second_candidate_before_room_reached(
            self, proc_info, coordinator):
        self.assertTrue(
            self.wait_for(lambda: (
                self.publisher.get_subscription_count() > 0 and
                self.node.count_publishers('/handyman/message/to_moderator') > 0)),
            'Coordinator topic discovery did not finish')
        time.sleep(0.2)
        # The real Moderator repeats its startup handshake while ROS discovery
        # settles. Mirror that behavior instead of relying on one volatile
        # publication immediately after endpoint discovery.
        handshake_deadline = time.monotonic() + 8.0
        while (
            'I_am_ready' not in self.received
            and time.monotonic() < handshake_deadline
        ):
            self.publish('Environment', 'LayoutA')
            self.publish('Are_you_ready?')
            self.wait_for(lambda: 'I_am_ready' in self.received, timeout=0.4)
        self.assertIn('I_am_ready', self.received)
        self.publish(
            'Instruction',
            'Go to the kitchen, grasp the apple and bring it to the dining table.')
        self.assertTrue(
            self.wait_for(lambda: 'Room_reached' in self.received),
            f'Room_reached was not received; events={self.received}')
        self.assertGreaterEqual(len(self.goal_positions), 2)
        self.assertFalse(
            math.isclose(self.goal_positions[0][0], self.goal_positions[1][0]) and
            math.isclose(self.goal_positions[0][1], self.goal_positions[1][1]),
            'Recovery resent the same candidate instead of trying another one')
        self.publish('Mission_complete')
        proc_info.assertWaitForShutdown(process=coordinator, timeout=10)


@launch_testing.post_shutdown_test()
class TestNavigationRoundTripExitCodes(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
