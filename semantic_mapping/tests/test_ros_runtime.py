import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros_runtime import ros_setup, ros_python, ros_python_major, ros_command


class RosRuntimeTests(unittest.TestCase):
    def test_active_distro_selects_its_own_interpreter(self):
        with patch('ros_runtime.os.path.isfile', return_value=True):
            for distro, python in [('melodic', 'python2'), ('noetic', 'python3')]:
                with patch.dict(os.environ, {'ROS_DISTRO': distro}, clear=True):
                    self.assertEqual(ros_setup(), '/opt/ros/' + distro + '/setup.bash')
                    self.assertEqual(ros_python(), python)
                    self.assertEqual(ros_python_major(), int(python[-1]))

    def test_explicit_setup_and_python_override_are_quoted(self):
        with patch.dict(os.environ, {'TRANSBOT_ROS_SETUP': '/opt/custom ROS/setup.bash',
                                    'BAGTOOLS_ROS_PYTHON': '/opt/local env/bin/python'}, clear=True):
            command = ros_command(['/home/Raymond/Desktop/SCC Lab/read.py', '/tmp/session one.bag'])
            self.assertEqual(command[:2], ['bash', '-c'])
            import shlex
            words = shlex.split(command[2])
            self.assertEqual(words[-3:], ['/opt/local env/bin/python',
                                         '/home/Raymond/Desktop/SCC Lab/read.py', '/tmp/session one.bag'])

    def test_missing_ros_keeps_melodic_default_for_offline_fixtures(self):
        with patch.dict(os.environ, {}, clear=True), patch('ros_runtime.os.path.isfile', return_value=False):
            self.assertEqual(ros_python(), 'python2')
