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
    def test_main_stack_launches_validated_cleanup_detector(self):
        ld = load_launch_description()
        actions = list(walk_actions(ld.entities))
        nodes = {
            node_key(action): action
            for action in actions
            if isinstance(action, Node)
        }

        self.assertIn(
            ('cleanup_vision_ros2', 'cleanup_detection_node'),
            nodes,
        )
        self.assertIn(
            ('interactive_cleanup', 'interactive_cleanup_sample'),
            nodes,
        )

    def test_nav2_bringup_remains_optional(self):
        ld = load_launch_description()
        actions = list(walk_actions(ld.entities))

        include_locations = [
            repr(getattr(action.launch_description_source, '_LaunchDescriptionSource__location', None))
            for action in actions
            if isinstance(action, IncludeLaunchDescription)
        ]

        self.assertTrue(
            any('bringup_launch.py' in location for location in include_locations),
            f'Expected conditional Nav2 bringup, got: {include_locations}',
        )


if __name__ == '__main__':
    unittest.main()
