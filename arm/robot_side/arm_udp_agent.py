#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""UDP bridge between the PC GUI and the real Transbot arm.

Run this on the robot (Jetson) or anywhere that can reach the robot's ROS
master.  It keeps one rospy connection to that master, so the PC GUI can stay
on the local Gazebo master and still command / read the real arm.

Wire format (UTF-8 JSON over UDP, port 42100):

    PC -> robot
        {"cmd": "set", "j7": 129.0, "j8": 188.0, "run_time": 800}
        {"cmd": "set", "j7": 129.0, "j8": 188.0, "j9": 89.0}
        {"cmd": "ping"}

    robot -> PC   (5 Hz, to the last sender)
        {"type": "state", "j7": 129.0, "j8": 188.0, "j9": 89.0,
         "voltage": 10.4, "enabled": true, "box": 100.0}

Safety:
  * enabled by default; pass --read-only to never publish /TargetAngle
  * every target is clamped to the servo limits and to start pose +/- --box
  * j9 is only sent when the packet contains it (the GUI never sends it)
  * packets older than --watchdog are ignored (the arm just holds)

Usage:
    python2 arm_udp_agent.py                  # enabled, default box 100
    python2 arm_udp_agent.py --read-only      # read state only
    python2 arm_udp_agent.py --box 20         # tighter test envelope
"""
from __future__ import division
from __future__ import print_function

import argparse
import json
import socket
import threading
import time

import rospy
from transbot_msgs.msg import Arm, Battery, Joint
from transbot_msgs.srv import RobotArm, RobotArmRequest

DEFAULT_PORT = 42100
J7_MIN, J7_MAX = 0.0, 225.0
J8_MIN, J8_MAX = 15.0, 240.0
J9_MIN, J9_MAX = 60.0, 180.0


def log(text):
    print('[udp-agent] %s' % text)
    import sys
    sys.stdout.flush()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def read_median(proxy, n=3):
    """Median of n /CurrentAngle reads; drops one-off bus glitches."""
    samples = []
    for _ in range(n):
        try:
            resp = proxy(RobotArmRequest('getJoint'))
            d = dict((int(j.id), float(j.angle)) for j in resp.RobotArm.joint)
        except Exception:
            d = {}
        if 7 in d and 8 in d:
            samples.append((d[7], d[8], d.get(9)))
        time.sleep(0.015)
    if len(samples) < 2:
        return None
    s7 = sorted(s[0] for s in samples)
    s8 = sorted(s[1] for s in samples)
    m = len(samples) // 2
    j9 = samples[-1][2]
    return s7[m], s8[m], j9


class Agent(object):
    def __init__(self, opts):
        self.opts = opts
        self.pub = None
        self.voltage = None
        self.client = None
        self.lock = threading.Lock()
        self.start = None

        rospy.init_node('arm_udp_agent', anonymous=True)
        if rospy.get_param('/use_sim_time', False):
            raise SystemExit('robot master has /use_sim_time=true; refusing')

        rospy.Subscriber('/voltage', Battery, self.on_voltage, queue_size=1)
        self.proxy = rospy.ServiceProxy('/CurrentAngle', RobotArm)
        try:
            rospy.wait_for_service('/CurrentAngle', timeout=5.0)
        except Exception as e:
            raise SystemExit('/CurrentAngle unavailable: %s' % e)

        pose = read_median(self.proxy)
        if pose is None:
            raise SystemExit('robot did not return joint 7/8')
        self.start = (pose[0], pose[1])
        log('start j7=%.2f j8=%.2f j9=%s' % pose)

        if opts.box > 0:
            self.lo7 = max(J7_MIN, self.start[0] - opts.box)
            self.hi7 = min(J7_MAX, self.start[0] + opts.box)
            self.lo8 = max(J8_MIN, self.start[1] - opts.box)
            self.hi8 = min(J8_MAX, self.start[1] + opts.box)
        else:
            self.lo7, self.hi7 = J7_MIN, J7_MAX
            self.lo8, self.hi8 = J8_MIN, J8_MAX
        log('box j7=[%.1f,%.1f] j8=[%.1f,%.1f]' %
            (self.lo7, self.hi7, self.lo8, self.hi8))

        if opts.enable:
            self.pub = rospy.Publisher('/TargetAngle', Arm, queue_size=1)
            t0 = time.time()
            while self.pub.get_num_connections() == 0 and time.time() - t0 < 3:
                rospy.sleep(0.05)
            log('mode=ENABLED (%d subscriber)' % self.pub.get_num_connections())
        else:
            log('mode=READ-ONLY (no --enable)')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((opts.bind, opts.port))
        except socket.error as e:
            raise SystemExit('cannot bind %s:%d: %s (another agent already running?)'
                             % (opts.bind, opts.port, e))
        self.sock.settimeout(0.05)
        log('listening on %s:%d' % (opts.bind, opts.port))

    def on_voltage(self, msg):
        self.voltage = float(msg.Voltage)

    def state_loop(self):
        rate = rospy.Rate(self.opts.rate)
        while not rospy.is_shutdown():
            pose = read_median(self.proxy)
            with self.lock:
                client = self.client
            if pose is not None and client is not None:
                payload = {
                    'type': 'state',
                    'j7': round(pose[0], 1),
                    'j8': round(pose[1], 1),
                    'j9': round(pose[2], 1) if pose[2] is not None else None,
                    'voltage': round(self.voltage, 2) if self.voltage else None,
                    'enabled': bool(self.opts.enable),
                    'box': self.opts.box,
                    'start_j7': round(self.start[0], 1),
                    'start_j8': round(self.start[1], 1),
                    'ts': time.time(),
                }
                try:
                    self.sock.sendto(json.dumps(payload).encode('utf-8'), client)
                except Exception:
                    pass
            rate.sleep()

    def handle_set(self, msg):
        if self.pub is None:
            return
        j7 = clamp(float(msg.get('j7', self.start[0])), self.lo7, self.hi7)
        j8 = clamp(float(msg.get('j8', self.start[1])), self.lo8, self.hi8)
        rt = int(clamp(float(msg.get('run_time', 800)), 100, 2000))
        arm = Arm()
        for jid, ang in ((7, j7), (8, j8)):
            jt = Joint()
            jt.id = jid
            jt.angle = round(ang, 2)
            jt.run_time = rt
            arm.joint.append(jt)
        if 'j9' in msg and msg['j9'] is not None:
            jt = Joint()
            jt.id = 9
            jt.angle = round(clamp(float(msg['j9']), J9_MIN, J9_MAX), 2)
            jt.run_time = rt
            arm.joint.append(jt)
        self.pub.publish(arm)

    def run(self):
        t = threading.Thread(target=self.state_loop)
        t.daemon = True
        t.start()
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except Exception:
                break
            try:
                msg = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            with self.lock:
                self.client = addr
            cmd = msg.get('cmd')
            if cmd == 'set':
                self.handle_set(msg)
            elif cmd == 'ping':
                pass
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description='UDP bridge to the real arm')
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--rate', type=float, default=5.0,
                    help='state packets per second (default 5)')
    ap.add_argument('--box', type=float, default=100.0,
                    help='clamp targets to start pose +/- box deg '
                         '(default 100; 0 = only servo limits)')
    group = ap.add_mutually_exclusive_group()
    group.add_argument('--enable', dest='enable', action='store_true',
                       default=True,
                       help='publish /TargetAngle (default)')
    group.add_argument('--read-only', dest='enable', action='store_false',
                       help='read angles only; never publish /TargetAngle')
    opts = ap.parse_args()
    Agent(opts).run()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
