#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


def load_module():
    script_file = (
        Path(__file__).resolve().parents[1]
        / 'scripts'
        / 'placeholder_tf_publisher.py'
    )
    spec = importlib.util.spec_from_file_location('placeholder_tf_publisher', script_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PlaceholderTfShutdownTest(unittest.TestCase):
    def test_main_does_not_shutdown_twice_when_context_is_already_stopped(self):
        module = load_module()
        fake_node = Mock()

        with patch.object(module, 'PlaceholderTfPublisher', return_value=fake_node), \
                patch.object(module.rclpy, 'init'), \
                patch.object(module.rclpy, 'spin', side_effect=KeyboardInterrupt), \
                patch.object(module.rclpy, 'ok', return_value=False), \
                patch.object(module.rclpy, 'shutdown') as shutdown:
            module.main()

        fake_node.destroy_node.assert_called_once_with()
        shutdown.assert_not_called()


if __name__ == '__main__':
    unittest.main()
