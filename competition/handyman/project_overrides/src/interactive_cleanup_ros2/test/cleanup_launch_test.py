#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node


def load_launch_description():
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'cleanup.launch.py'
    spec = importlib.util.spec_from_file_location('cleanup_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def walk_actions(actions):
    for action in actions:
        yield action
        if isinstance(action, GroupAction):
            yield from walk_actions(action.get_sub_entities())


def node_key(node):
    return (
        getattr(node, '_Node__package', None),
        getattr(node, '_Node__node_executable', None),
    )


def node_name(node):
    return getattr(node, '_Node__node_name', None)


def node_remappings(node):
    return list(getattr(node, '_Node__remappings', []))


def render_substitution(value):
    if isinstance(value, tuple):
        return ''.join(render_substitution(item) for item in value)

    text = getattr(value, '_TextSubstitution__text', None)
    if text is not None:
        return text

    return str(value)


def normalized_remappings(node):
    return {
        (render_substitution(src), render_substitution(dst))
        for src, dst in node_remappings(node)
    }


class CleanupLaunchTest(unittest.TestCase):
    def test_main_stack_launches_split_perception_nodes(self):
        ld = load_launch_description()
        actions = list(walk_actions(ld.entities))
        nodes = {
            node_key(action): action
            for action in actions
            if isinstance(action, Node)
        }

        self.assertIn(
            ('cleanup_vision_ros2', 'head_perception_node'),
            nodes,
        )
        self.assertIn(
            ('cleanup_vision_ros2', 'hand_perception_node'),
            nodes,
        )
        self.assertNotIn(
            ('cleanup_vision_ros2', 'cleanup_detection_node'),
            nodes,
        )

    def test_nav2_navigation_nodes_are_explicit_and_remapped_to_hsr(self):
        ld = load_launch_description()
        actions = list(walk_actions(ld.entities))

        include_locations = [
            repr(getattr(action.launch_description_source, '_LaunchDescriptionSource__location', None))
            for action in actions
            if isinstance(action, IncludeLaunchDescription)
        ]

        self.assertTrue(
            any('localization_launch.py' in location for location in include_locations),
            f'Expected localization_launch.py include, got: {include_locations}',
        )
        self.assertFalse(
            any('bringup_launch.py' in location for location in include_locations),
            f'Unexpected bringup_launch.py include: {include_locations}',
        )

        nodes = {
            node_key(action): action
            for action in actions
            if isinstance(action, Node)
        }

        expected_nodes = {
            ('nav2_controller', 'controller_server'),
            ('nav2_planner', 'planner_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
            ('nav2_lifecycle_manager', 'lifecycle_manager'),
        }

        for expected_node in expected_nodes:
            self.assertIn(expected_node, nodes, f'Missing node: {expected_node}')

        self.assertIn(
            ('cmd_vel', '/hsrb/command_velocity'),
            normalized_remappings(nodes[('nav2_controller', 'controller_server')]),
        )
        self.assertIn(
            ('cmd_vel', '/hsrb/command_velocity'),
            normalized_remappings(nodes[('nav2_behaviors', 'behavior_server')]),
        )
        self.assertEqual(
            'lifecycle_manager_navigation',
            node_name(nodes[('nav2_lifecycle_manager', 'lifecycle_manager')]),
        )


if __name__ == '__main__':
    unittest.main()
