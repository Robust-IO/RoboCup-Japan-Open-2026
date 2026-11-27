import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import launch_ros.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    simulation = launch_ros.actions.Node(
        package='handyman_rebuild_ros2',
        executable='handyman_phase1_simulation',
        output='screen',
        additional_env={'ROS_LOCALHOST_ONLY': '1'},
    )

    return (
        launch.LaunchDescription([
            simulation,
            launch_testing.actions.ReadyToTest(),
            launch_testing.util.KeepAliveProc(),
        ]),
        {'simulation': simulation},
    )


class TestProtocolRoundTrip(unittest.TestCase):

    def test_simulation_finishes(self, proc_info, simulation):
        proc_info.assertWaitForShutdown(process=simulation, timeout=15)


@launch_testing.post_shutdown_test()
class TestProtocolRoundTripExitCodes(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
