"""Navigation must not report ready with missing plugins or a dead planner."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav'))
from stack import NavStack, MODE_SLAMNAV


class NavigationStartupTests(unittest.TestCase):
    def setUp(self):
        self.stack = NavStack('/tmp/transbot-nav-test')

    def test_missing_plugin_refuses_before_taking_control(self):
        self.stack._sh = Mock(return_value=(1, 'not found'))
        self.stack._step_robot = Mock()
        self.stack.stop = Mock()
        with self.assertRaisesRegex(RuntimeError, 'global_planner'):
            self.stack.start(MODE_SLAMNAV)
        self.stack._step_robot.assert_not_called()
        self.stack.stop.assert_not_called()
        self.assertFalse(self.stack.running())

    def test_stale_registration_cannot_pass_verification(self):
        self.stack._sh = Mock(return_value=(0, '/slam_gmapping\n/move_base'))
        self.stack._alive = lambda name: name != '/move_base'
        with self.assertRaisesRegex(RuntimeError, 'nodes unavailable: /move_base'):
            self.stack._step_verify(MODE_SLAMNAV)

    def test_alive_nodes_without_planning_service_are_not_ready(self):
        self.stack._sh = Mock(side_effect=[(0, '/slam_gmapping\n/move_base'), (1, '')])
        self.stack._alive = Mock(return_value=True)
        with self.assertRaisesRegex(RuntimeError, 'planning service is unavailable'):
            self.stack._step_verify(MODE_SLAMNAV)


if __name__ == '__main__':
    unittest.main()
