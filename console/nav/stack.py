#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Three ways to use the robot, all launched from this machine.

    map       gmapping only        -> drive around, save the map
    nav       map_server + AMCL + move_base on a saved map
    slamnav   gmapping + move_base on the live map (no map_server, no AMCL)

Nothing here runs on the robot. The robot only publishes /scan, /odom and
/tf; the planners run locally, so one process tree owns the whole stack and
stopping it cannot leave half a stack behind.

Every start runs the same pre-flight, in this order, because each step has
already caused a failed run at least once:

  1. robot answers            a stale ROS_IP makes the robot beep but never
                              receive a command
  2. clock                    the Jetson has no RTC; a gap beyond
                              transform_tolerance freezes AMCL silently
  3. leftovers killed         two map->odom publishers break both stacks.
                              pkill only sees this PC, so the rest are killed
                              by node name on the master -- that is how a
                              stack left running on another console dies
  4. scan filter              /scan_filtered drops the robot's own body
                              returns, which gmapping would draw as a ring.
                              If one is already publishing, adopt it; a
                              second one feeds gmapping every scan twice
  5. the stack itself
  6. angular gain relay       the chassis turns 1.41x faster than commanded
  7. verify                   node present + /map has data

The GUI owns the log; this module only calls the callback.
"""

import json
import os
import re
import shutil
import signal
import subprocess
import threading
import time

MODE_MAP = 'map'
MODE_NAV = 'nav'
MODE_SLAMNAV = 'slamnav'
MODES = (MODE_MAP, MODE_NAV, MODE_SLAMNAV)

# leftovers that would fight for /cmd_vel, /map or map->odom
KILL_PATTERNS = (
    '[m]ove_base', '[a]mcl', '[m]ap_server', '[s]lam_gmapping',
    '[s]can_filter.py', '[a]mcl_keepalive',
    '[n]av.launch', '[m]ap.launch', '[s]lam_nav.launch',
    '[t]ransbot_navigation.launch', '[t]ransbot_map.launch',
)

# vendor nodes that publish /cmd_vel on their own and quietly override every
# command the GUI sends (they come back when the vendor app restarts)
CMD_VEL_NODES = ('/TransbotPatrol', '/BigTransbot')

# Nodes that fight over /map, map->odom or /cmd_vel. These are matched by
# node NAME on the master, not by process on this machine: a console running
# on another PC joins the same master, and pkill here cannot see its
# processes at all.
LEFTOVER_NODES = ('/move_base', '/amcl', '/map_server', '/slam_gmapping')
# helpers started with rospy anonymous=True -> /scan_filter_1234_5678
LEFTOVER_PREFIXES = ('/scan_filter', '/amcl_keepalive')

# lines that are technically warnings/info but repeat every second and bury
# the preflight steps in the log. Anything containing ERROR or FATAL is always
# kept.
NOISE = (
    'Map update loop missed its desired rate',
    'update frame',
    'update ld=',
    'Laser Pose=',
    'm count',
    'Registering First Scan',
    'average scan matching score',
)


def _strip_ansi(text):
    return re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', text)


class NavStack(object):
    def __init__(self, base_dir, log=None, host=None, user='jetson',
                 password='yahboom'):
        self.base = os.path.abspath(base_dir)
        self.launch_dir = os.path.join(self.base, 'launch')
        self.param_dir = os.path.join(self.base, 'param')
        self.helpers = os.path.join(self.base, 'helpers')
        self.maps_dir = os.path.join(os.path.dirname(self.base), 'maps')
        self.log = log or (lambda _m: None)
        self.host = host or self._host_from_env()
        self.user = user
        self.password = password
        self.mode = None
        self._procs = []          # [(tag, Popen)]
        self._lock = threading.Lock()

    # -- environment ---------------------------------------------------
    @staticmethod
    def _host_from_env():
        m = re.match(r'^\w+://([^:/]+)', os.environ.get('ROS_MASTER_URI', ''))
        if m:
            return m.group(1)
        return os.environ.get('ROBOT_IP', '192.168.1.11')

    def running(self):
        return self.mode is not None

    # -- small shell helpers -------------------------------------------
    def _sh(self, cmd, timeout=15):
        """Run a command, return (rc, stdout+stderr). Never raises."""
        try:
            r = subprocess.run(cmd, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT,
                               universal_newlines=True, timeout=timeout)
            return r.returncode, (r.stdout or '').strip()
        except subprocess.TimeoutExpired:
            return 124, 'timeout'
        except Exception as e:
            return 1, str(e)

    def _pkill(self, patterns=KILL_PATTERNS):
        for p in patterns:
            self._sh(['pkill', '-f', p], timeout=6)
        time.sleep(1.0)

    def _spawn(self, tag, argv, extra_env=None):
        env = dict(os.environ)
        if extra_env:
            env.update(extra_env)
        try:
            p = subprocess.Popen(argv, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 universal_newlines=True, bufsize=1,
                                 start_new_session=True, env=env)
        except Exception as e:
            raise RuntimeError('cannot start %s: %s' % (tag, e))
        with self._lock:
            self._procs.append((tag, p))
        threading.Thread(target=self._pump, args=(tag, p),
                         daemon=True).start()
        return p

    def _pump(self, tag, proc):
        try:
            for line in proc.stdout:
                line = _strip_ansi(line).rstrip()
                if not line:
                    continue
                if ('ERROR' not in line and 'FATAL' not in line
                        and any(n in line for n in NOISE)):
                    continue
                self.log('%s| %s' % (tag, line))
        except Exception:
            pass

    def _wait_topic(self, topic, timeout=20.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rc, out = self._sh(['rostopic', 'list'], timeout=8)
            if rc == 0 and topic in out.split():
                return True
            time.sleep(1.0)
        return False

    def _wait_node(self, name, timeout=25.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rc, out = self._sh(['rosnode', 'list'], timeout=8)
            if rc == 0 and name in out.split():
                return True
            time.sleep(1.0)
        return False

    def _find_node(self, name):
        """Full node name on the master, or None. Anonymous suffixes count."""
        rc, out = self._sh(['rosnode', 'list'], timeout=8)
        if rc != 0:
            return None
        for n in out.split():
            if n == name or n.startswith(name + '_'):
                return n
        return None

    def _node_up(self, name):
        """True if `name` is registered -- anonymous suffixes count.

        The console starts the relay with rospy anonymous=True, so the node is
        /cmd_vel_scale_1234_5678, not /cmd_vel_scale. Matching the bare name
        only would miss it and the stack would start a second relay writing
        the same topic.
        """
        return self._find_node(name) is not None

    def _node_host(self, name):
        """Which machine runs `name`, for messages that need to be actionable."""
        rc, out = self._sh(['rosnode', 'info', name], timeout=8)
        m = re.search(r'contacting node http://([^:/]+)', out)
        return m.group(1) if m else 'unknown host'

    def _publishers(self, topic):
        """Hosts registered as publishing `topic` (may include stale ones)."""
        rc, out = self._sh(['rostopic', 'info', topic], timeout=8)
        if rc != 0:
            return []
        head = out.split('Subscribers')[0]
        return re.findall(r'http://([^:/]+):', head)

    def _alive(self, name):
        """True if the node answers, not merely registered.

        Killing a process leaves its registration on the master until
        something pings it. Treating that entry as a live stack is how the
        console used to skip steps it should have run ("relay already
        running" while nothing was publishing /cmd_vel).
        """
        rc, _ = self._sh(['rosnode', 'ping', '-c', '1', name], timeout=8)
        return rc == 0

    def _topic_live(self, topic, wait=4.0):
        """True if a message actually arrives -- registration proves nothing."""
        rc, _ = self._sh(['timeout', str(int(wait)), 'rostopic', 'echo', '-n1',
                          '--noarr', topic], timeout=wait + 5)
        return rc == 0

    def _leftovers(self):
        """Nav nodes still running on the master, whoever started them."""
        rc, out = self._sh(['rosnode', 'list'], timeout=8)
        if rc != 0:
            return []
        names = [n for n in out.split()
                 if n in LEFTOVER_NODES
                 or any(n.startswith(p) for p in LEFTOVER_PREFIXES)]
        return [n for n in names if self._alive(n)]

    def _refuse_if_running(self, name, what):
        """Fail loudly instead of starting a second owner of /map or /cmd_vel.

        _step_clean normally gets rid of these; reaching here means the kill
        did not take (a frozen node, a launch with respawn="true", or a
        master that will not honour the shutdown request).
        """
        if not (self._node_up(name) and self._alive(name)):
            return
        raise RuntimeError(
            '%s is already running on %s; close that console (or stop its '
            'stack) before starting this one' % (what, self._node_host(name)))

    # -- maps ----------------------------------------------------------
    def ensure_maps_dir(self):
        if not os.path.isdir(self.maps_dir):
            os.makedirs(self.maps_dir)

    def list_maps(self):
        self.ensure_maps_dir()
        names = []
        for f in sorted(os.listdir(self.maps_dir)):
            if f.endswith('.yaml'):
                names.append(f[:-5])
        return names

    def map_path(self, name):
        return os.path.join(self.maps_dir, '%s.yaml' % name)

    def map_exists(self, name):
        return os.path.isfile(self.map_path(name))

    @staticmethod
    def default_name(prefix='lab'):
        return '%s_%s' % (prefix, time.strftime('%Y%m%d_%H%M%S'))

    def map_meta(self, name):
        """Metadata written next to the map when it was saved, or {}."""
        try:
            with open(os.path.join(self.maps_dir, '%s.json' % name)) as fh:
                return json.load(fh)
        except Exception:
            return {}

    def save_map(self, name, stats=None, pose=None, odom_m=None):
        """map_saver in this machine's maps/ directory.

        map_saver writes an ABSOLUTE image path into the yaml; that makes the
        folder unportable (copy maps/ elsewhere and map_server silently
        fails). Rewrite it to the bare file name: map_server resolves it
        relative to the yaml's own directory.
        """
        if not re.match(r'^[\w][\w\-. ]*$', name or '') or '/' in (name or ''):
            raise RuntimeError('invalid map name: %s' % name)
        self.ensure_maps_dir()
        target = os.path.join(self.maps_dir, name)
        rc, out = self._sh(['rosrun', 'map_server', 'map_saver',
                            '-f', target], timeout=60)
        if rc != 0 or not os.path.isfile(target + '.yaml'):
            raise RuntimeError('map_saver failed: %s' % (out or 'no output'))
        self._rewrite_image_field(name)
        meta = {'name': name, 'saved': time.strftime('%Y-%m-%d %H:%M:%S'),
                'resolution': None, 'width': None, 'height': None}
        try:
            with open(target + '.yaml') as fh:
                for line in fh:
                    if line.startswith('resolution:'):
                        meta['resolution'] = float(line.split(':')[1])
            meta['width'], meta['height'] = self._pgm_size(target + '.pgm')
        except Exception:
            pass
        if stats:
            meta['area_m2'] = round(stats.get('area', 0.0), 1)
            meta['coverage_pct'] = round(stats.get('cover', 0.0), 1)
            meta['free_pct'] = round(stats.get('free', 0.0), 1)
            meta['size_m'] = [round(stats.get('w', 0.0), 1),
                              round(stats.get('h', 0.0), 1)]
        if pose:
            meta['pose'] = [round(pose[0], 3), round(pose[1], 3),
                            round(pose[2], 3)]
        if odom_m is not None:
            meta['odom_m'] = round(odom_m, 1)
        try:
            with open(target + '.json', 'w') as fh:
                json.dump(meta, fh, indent=1)
        except Exception:
            pass
        return target

    @staticmethod
    def _pgm_size(path):
        """(width, height) from a binary PGM header, skipping '#' comments.

        map_saver writes 'P5\\n# CREATOR: map_saver.cpp ...\\n800 800\\n255', so
        splitting the first N bytes gives ['P5', '#', 'CREATOR:', ...] and the
        width ends up in the comment, not at index 1.
        """
        with open(path, 'rb') as fh:
            magic = fh.readline().strip()
            if magic != b'P5':
                raise ValueError('not a binary PGM')
            line = fh.readline()
            while line.startswith(b'#'):
                line = fh.readline()
            w, h = line.split()[:2]
        return int(w), int(h)

    def _rewrite_image_field(self, name):
        path = self.map_path(name)
        try:
            with open(path) as fh:
                lines = fh.readlines()
            out = []
            for line in lines:
                if line.startswith('image:'):
                    out.append('image: %s.pgm\n' % name)
                else:
                    out.append(line)
            with open(path, 'w') as fh:
                fh.writelines(out)
        except Exception as e:
            self.log('warn| cannot rewrite image path: %s' % e)

    def rename_map(self, old, new):
        if not self.map_exists(old):
            raise RuntimeError('no such map: %s' % old)
        if self.map_exists(new) or os.path.isfile(os.path.join(
                self.maps_dir, new + '.pgm')):
            raise RuntimeError('a map called %s already exists' % new)
        if not re.match(r'^[\w][\w\-. ]*$', new or '') or '/' in (new or ''):
            raise RuntimeError('invalid map name: %s' % new)
        for ext in ('.yaml', '.pgm', '.json'):
            src = os.path.join(self.maps_dir, old + ext)
            if os.path.isfile(src):
                os.rename(src, os.path.join(self.maps_dir, new + ext))
        self._rewrite_image_field(new)
        meta = self.map_meta(new)
        if meta:
            meta['name'] = new
            try:
                with open(os.path.join(self.maps_dir, new + '.json'), 'w') as fh:
                    json.dump(meta, fh, indent=1)
            except Exception:
                pass
        return new

    # -- lifecycle -----------------------------------------------------
    def start(self, mode, map_name=None, pose=(0.0, 0.0, 0.0)):
        if mode not in MODES:
            raise RuntimeError('unknown mode: %s' % mode)
        if self.running():
            raise RuntimeError('a stack is already running (%s)' % self.mode)
        if mode == MODE_NAV:
            if not map_name:
                raise RuntimeError('navigation needs a map')
            if not self.map_exists(map_name):
                raise RuntimeError('map not found: %s' % self.map_path(map_name))
        self.log('== %s ==' % self._mode_title(mode))
        try:
            self._step_robot(mode)
            self._step_clock()
            self._step_clean()
            if mode in (MODE_MAP, MODE_SLAMNAV):
                self._step_scan_filter()
            if mode == MODE_NAV:
                self._step_nav(map_name, pose)
            elif mode == MODE_MAP:
                self._step_map()
            else:
                self._step_slam_nav()
            if mode in (MODE_NAV, MODE_SLAMNAV):
                self._step_scale()
            if mode == MODE_NAV:
                self._step_keepalive()
            self._step_verify(mode)
        except Exception:
            self.stop(quiet=True)
            raise
        self.mode = mode
        self.log('== ready: %s ==' % self._mode_title(mode))
        return True

    def _mode_title(self, mode):
        return {MODE_MAP: 'mapping', MODE_NAV: 'navigation',
                MODE_SLAMNAV: 'map + navigate'}.get(mode, mode)

    def _step_robot(self, mode):
        self.log('1/7 robot reachable')
        rc, out = self._sh(['ping', '-c', '2', '-W', '2', self.host], timeout=10)
        if rc != 0:
            raise RuntimeError(
                'no ping to %s -- is the robot hotspot connected? '
                '(nmcli con up id Transbot)' % self.host)
        if mode in (MODE_MAP, MODE_SLAMNAV) and not self._wait_topic('/scan', 8):
            raise RuntimeError('/scan has no data; the robot stack is not up')
        self.log('   ok  %s' % self.host)

    def _step_clock(self):
        self.log('2/7 clock')
        script = os.path.join(self.helpers, 'clock_guard.py')
        if not os.path.isfile(script):
            self.log('   skip  clock_guard.py missing')
            return
        if shutil.which('sshpass') is None:
            self.log('   skip  sshpass not installed')
            return
        rc, out = self._sh(['python3', script, '--sync', '--user', self.user,
                            '--password', self.password], timeout=45)
        tail = [l for l in out.splitlines() if l.strip()][-2:]
        for line in tail:
            self.log('   %s' % line.strip())
        if rc != 0:
            self.log('   warn  clock sync did not complete (rc=%d)' % rc)

    def _step_clean(self):
        self.log('3/7 stop leftovers')
        self._pkill()
        # pkill only sees processes on THIS machine. A console on another PC
        # joins the same master and keeps publishing /scan_filtered, /map and
        # map->odom; the only way to stop it is rosnode kill, which rospy
        # nodes honour wherever they run.
        killed = []
        for name in self._leftovers():
            host = self._node_host(name)
            self._sh(['rosnode', 'kill', name], timeout=10)
            killed.append('%s@%s' % (name, host))
        if killed:
            time.sleep(2.0)
            self.log('   killed leftovers: %s' % ', '.join(killed))
        # The vendor's patrol/demo apps publish /cmd_vel whenever they feel
        # like it, and the driver obeys the last message it sees. They also
        # come back when the vendor app restarts, so this runs every time.
        rc, out = self._sh(['rosnode', 'list'], timeout=8)
        nodes = out.split()
        for name in CMD_VEL_NODES:
            if name in nodes:
                self._sh(['rosnode', 'kill', name], timeout=10)
                self.log('   killed %s (it publishes /cmd_vel)' % name)
                time.sleep(1.5)
        # Two consoles on one master is the usual cause of everything below
        # going wrong, and it is invisible unless someone says it out loud.
        guis = [n for n in nodes if n.startswith('/transbot_gui')]
        if len(guis) > 1:
            self.log('   note  %d consoles are connected to this master; only '
                     'one of them should run the stack' % len(guis))
        left = self._leftovers()
        if left:
            self.log('   warn  still registered: %s' % ', '.join(
                '%s@%s' % (n, self._node_host(n)) for n in left))

    def _step_scan_filter(self):
        self.log('4/7 scan filter')
        # Never add a second filter. Two filters on one /scan publish two
        # LaserScans per sweep with identical header stamps; gmapping matches
        # the same scan twice, stops growing the map, and every goal click is
        # then rejected as "unknown" because the map is still empty.
        # The test is "does a message arrive", not "is a publisher
        # registered": a killed filter leaves its registration behind.
        if self._topic_live('/scan_filtered'):
            hosts = sorted(set(self._publishers('/scan_filtered')))
            self.log('   ok  /scan_filtered already has data%s; not starting '
                     'another' % (' (from %s)' % ', '.join(hosts) if hosts
                                  else ''))
            return
        self._spawn('scan', ['python3', os.path.join(self.helpers,
                                                    'scan_filter.py')])
        if self._wait_topic('/scan_filtered', 12):
            self.log('   ok  /scan_filtered')
        else:
            self.log('   warn  /scan_filtered missing, gmapping may see the '
                     'robot body as an obstacle')

    def _step_map(self):
        self.log('5/7 gmapping')
        self._refuse_if_running('/slam_gmapping', 'gmapping')
        self._spawn('map', ['roslaunch', os.path.join(self.launch_dir,
                                                     'map.launch')])
        if not self._wait_topic('/map', 30):
            raise RuntimeError('/map never appeared (see the map| lines above)')
        self.log('   ok  /map')

    def _check_planners(self):
        """The launch file names plugins that live in separate apt packages.

        On a machine without them move_base dies with
        "class global_planner/GlobalPlanner ... does not exist" and the goal
        is accepted but never executed, which looks like a navigation bug.
        """
        distro = os.environ.get('ROS_DISTRO', 'melodic')
        for pkg in ('global_planner', 'dwa_local_planner'):
            rc, _ = self._sh(['rospack', 'find', pkg], timeout=8)
            if rc == 0:
                continue
            self.log('   warn  %s is not installed on this machine; move_base '
                     'will fail to create its planner' % pkg)
            self.log('         fix: sudo apt install ros-%s-%s'
                     % (distro, pkg.replace('_', '-')))
            if pkg == 'global_planner':
                # navfn ships with move_base itself and takes the same
                # defaults, so this works with no install and no sudo.
                self.log('         or, without root: set base_global_planner '
                         'to "navfn/NavfnROS" in move_base_params.yaml')

    def _step_nav(self, map_name, pose):
        self.log('5/7 navigation stack (map %s)' % map_name)
        self._refuse_if_running('/move_base', 'move_base')
        self._check_planners()
        self._spawn('nav', ['roslaunch',
                            os.path.join(self.launch_dir, 'nav.launch'),
                            'map_file:=%s' % self.map_path(map_name),
                            'param_dir:=%s' % self.param_dir,
                            'initial_pose_x:=%f' % pose[0],
                            'initial_pose_y:=%f' % pose[1],
                            'initial_pose_a:=%f' % pose[2]])
        if not self._wait_node('/move_base', 40):
            raise RuntimeError('move_base did not come up')
        if not self._wait_topic('/map', 20):
            raise RuntimeError('/map has no data; map_server failed to load %s'
                               % self.map_path(map_name))
        self.log('   ok  move_base + map_server + amcl')

    def _step_slam_nav(self):
        self.log('5/7 gmapping + move_base')
        self._refuse_if_running('/slam_gmapping', 'gmapping')
        self._refuse_if_running('/move_base', 'move_base')
        self._check_planners()
        self._spawn('slamnav', ['roslaunch',
                                os.path.join(self.launch_dir,
                                             'slam_nav.launch'),
                                'param_dir:=%s' % self.param_dir])
        if not self._wait_node('/move_base', 40):
            raise RuntimeError('move_base did not come up')
        if not self._wait_topic('/map', 30):
            raise RuntimeError('/map never appeared')
        self.log('   ok  /map + move_base')

    def _spawn_scale(self):
        return self._spawn('scale', ['python3', os.path.join(self.helpers,
                                                            'cmd_vel_scale.py'),
                                     '--in-topic', '/cmd_vel_raw',
                                     '--out-topic', '/cmd_vel',
                                     '--angular-scale', '0.709',
                                     '--angular-deadzone', '0.0',
                                     '--teleop-topic', '/cmd_vel_teleop'])

    def _step_scale(self):
        self.log('6/7 angular gain relay')
        relay = self._find_node('/cmd_vel_scale')
        if relay and self._alive(relay):
            # The console starts one at launch and keeps it alive; two relays
            # would both write /cmd_vel.
            self.log('   ok  relay already running (console-owned)')
            return
        self._spawn_scale()
        time.sleep(2.0)
        if self._proc_alive('scale'):
            self.log('   ok  angular x0.709')
        else:
            self.log('   warn  relay died -- the robot will not move')

    def _step_keepalive(self):
        self.log('7/7 AMCL keepalive')
        script = os.path.join(self.helpers, 'amcl_keepalive.py')
        if not os.path.isfile(script):
            return
        self._spawn('keepalive', ['python3', script, '--quiet'])
        time.sleep(2.0)
        if self._proc_alive('keepalive'):
            self.log('   ok  AMCL keeps updating while idle')
        else:
            self.log('   warn  keepalive died; planning stalls when idle')

    def _step_verify(self, mode):
        self.log('verify')
        rc, out = self._sh(['rosnode', 'list'], timeout=8)
        nodes = out.split()
        want = ['/map_server', '/amcl', '/move_base'] if mode == MODE_NAV \
            else (['/slam_gmapping'] if mode == MODE_MAP
                  else ['/slam_gmapping', '/move_base'])
        missing = [n for n in want if n not in nodes]
        if missing:
            self.log('   warn  missing: %s' % ' '.join(missing))
        else:
            self.log('   ok  %s' % ' '.join(want))
        if mode in (MODE_NAV, MODE_SLAMNAV):
            rc, out = self._sh(['rostopic', 'info', '/cmd_vel'], timeout=8)
            names = re.findall(r'^\s*\*\s*(/\S+)', out, re.M)
            # our own node and the relay are expected; anything else can
            # override a goal without telling anyone
            others = [n for n in names
                      if not n.startswith('/transbot_gui')
                      and not n.startswith('/cmd_vel_scale')]
            if others:
                self.log('   note  /cmd_vel is also published by: %s'
                         % ' '.join(others))

    def _proc_alive(self, tag):
        with self._lock:
            for t, p in self._procs:
                if t == tag and p.poll() is None:
                    return True
        return False

    def stop(self, quiet=False):
        if not quiet:
            self.log('== stopping ==')
        with self._lock:
            procs = list(self._procs)
            self._procs = []
        for tag, p in reversed(procs):
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        deadline = time.time() + 6
        for tag, p in procs:
            while p.poll() is None and time.time() < deadline:
                time.sleep(0.1)
        for tag, p in procs:
            try:
                if p.poll() is None:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        self._pkill()
        self.mode = None
        if not quiet:
            self.log('== stopped ==')

    def shutdown(self):
        if self.running():
            self.stop(quiet=True)
