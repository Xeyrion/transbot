"""Select the local ROS bag interpreter, independently of robot-side scripts."""
from __future__ import print_function
import argparse
import os
try:
    from shlex import quote
except ImportError:
    from pipes import quote


def ros_setup():
    explicit = os.environ.get('TRANSBOT_ROS_SETUP')
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    distro = os.environ.get('ROS_DISTRO')
    candidates = ([distro] if distro else []) + ['melodic', 'noetic']
    for name in candidates:
        path = '/opt/ros/' + name + '/setup.bash'
        if os.path.isfile(path):
            return path
    return '/opt/ros/melodic/setup.bash'


def ros_python():
    explicit = os.environ.get('BAGTOOLS_ROS_PYTHON')
    if explicit:
        return explicit
    return 'python%d' % ros_python_major()


def ros_python_major():
    setup = ros_setup()
    if '/noetic/' in setup:
        return 3
    if '/melodic/' in setup:
        return 2
    return 3 if os.environ.get('ROS_PYTHON_VERSION') == '3' else 2


def ros_command(arguments, python=None):
    command = [python or ros_python()] + list(arguments)
    return ['bash', '-c', 'source %s 2>/dev/null; exec %s' % (
        quote(ros_setup()), ' '.join(quote(str(arg)) for arg in command))]


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--setup', action='store_true')
    args = parser.parse_args()
    print(ros_setup() if args.setup else ros_python())
