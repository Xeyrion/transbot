#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_client.py -- Transbot 2R arm: read the pose, move slowly, draw a line.

Why a separate module
---------------------
The GUI must never block on a servo.  /CurrentAngle is a ROS *service*, so a
call takes as long as the bus round-trip; doing that in a Tk callback freezes
the window.  Everything here therefore runs on its own thread, and the GUI
only ever reads a cached dict.

Conventions
-----------
* Caller owns rospy.init_node().
* Angles are servo degrees (j7 0..225, j8 15..240), same as line_tool.
* Every move is closed-loop on the measured pose and is allowed to abort at
  any point via a threading.Event.
* The servo ignores commands smaller than roughly 1 deg, so each step grows
  its lead when the measured pose does not change (same trick as the arm
  panel's real-robot loop).
"""
import math
import os
import sys
import threading
import time

import rospy
from transbot_msgs.msg import Arm, Joint
from transbot_msgs.srv import RobotArm

# The patched line_tool (top-down capable) lives next door, not in arm/.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'wallline'))
import line_tool as L          # noqa: E402

J7, J8, J9 = 7, 8, 9

# Hard servo limits, identical to console/transbot_gui.py and line_tool.py.
LIMITS = {J7: (0.0, 225.0), J8: (15.0, 240.0), J9: (60.0, 180.0)}

POSE_TIMEOUT = 1.5     # s, a pose older than this is treated as unknown
MAX_STEP_DEG = 6.0     # deg per command; bigger steps overshoot on the real arm


class ArmClient(object):
    def __init__(self, poll_hz=2.0):
        self._pub = None
        self._srv = None
        self._lock = threading.Lock()
        self._pose = None          # {7: deg, 8: deg, 9: deg}
        self._pose_t = 0.0
        self._err = ''
        self._stop = threading.Event()
        self._poll_hz = poll_hz
        self._thread = None

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        self._pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def shutdown(self):
        self._stop.set()

    @property
    def error(self):
        return self._err

    @property
    def pose(self):
        """Latest measured {joint: deg}, or None if it is stale/missing."""
        with self._lock:
            if self._pose is None:
                return None
            if time.time() - self._pose_t > POSE_TIMEOUT:
                return None
            return dict(self._pose)

    def wait_pose(self, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.pose
            if p is not None:
                return p
            time.sleep(0.1)
        return None

    def _poll_loop(self):
        while not self._stop.is_set():
            try:
                if self._srv is None:
                    rospy.wait_for_service('/CurrentAngle', timeout=2.0)
                    self._srv = rospy.ServiceProxy('/CurrentAngle', RobotArm)
                res = self._srv(apply='')
                joints = getattr(getattr(res, 'RobotArm', res), 'joint', [])
                if joints:
                    got = dict((int(j.id), float(j.angle)) for j in joints)
                    with self._lock:
                        # The servo bus sometimes answers only two of the
                        # three joints.  Merge instead of replacing, so a
                        # short reply does not blank out the missing angle.
                        merged = dict(self._pose or {})
                        merged.update(got)
                        self._pose = merged
                        self._pose_t = time.time()
                    self._err = ''
                    if len(got) < 3:
                        # Ask again soon; three good reads matter more than a
                        # fixed poll rate here.  The service call itself takes
                        # ~0.25 s, so this is not a busy loop.
                        time.sleep(0.1)
                        continue
            except Exception as exc:      # service missing / robot offline
                self._err = str(exc)
                self._srv = None
            time.sleep(1.0 / max(0.2, self._poll_hz))

    # -- commands ----------------------------------------------------------
    def send(self, angles, run_time=800):
        """angles: {joint_id: degrees}.  Out-of-range values are clamped."""
        if self._pub is None:
            return
        msg = Arm()
        for jid, ang in angles.items():
            lo, hi = LIMITS[jid]
            msg.joint.append(Joint(id=int(jid), run_time=int(run_time),
                                   angle=float(max(lo, min(hi, ang)))))
        self._pub.publish(msg)

    def goto(self, j7, j8, settle=1.0, tol=0.6, abort=None):
        """Move to (j7, j8) in small closed-loop steps.  True if reached."""
        return self._step_to(j7, j8, settle=settle, tol=tol, abort=abort)

    def _step_to(self, t7, t8, settle=1.0, tol=0.6, abort=None, tries=8):
        prev = None
        for k in range(tries):
            if abort is not None and abort.is_set():
                return False
            cur = self.pose
            if cur is None:
                return False
            e7 = t7 - cur[J7]
            e8 = t8 - cur[J8]
            if max(abs(e7), abs(e8)) <= tol:
                return True
            # Grow the lead when the arm sits still: this is what gets the
            # command past the ~1 deg servo dead band without overshooting.
            lead = min(2.2, 1.0 + 0.35 * k)
            d7 = e7 * lead
            d8 = e8 * lead
            # If the measured pose did not change at all since the last try,
            # the command was inside the dead band.  Ask for 2.5 deg instead
            # of giving up -- that is what made "Go to start" report
            # "not reached" while the arm had not moved a single degree.
            stuck = (prev is not None
                     and abs(cur[J7] - prev[J7]) < 0.3
                     and abs(cur[J8] - prev[J8]) < 0.3)
            prev = cur
            if stuck:
                if 0.0 < abs(e7) and abs(d7) < 2.5:
                    d7 = 2.5 if e7 > 0 else -2.5
                if 0.0 < abs(e8) and abs(d8) < 2.5:
                    d8 = 2.5 if e8 > 0 else -2.5
            c7 = cur[J7] + max(-MAX_STEP_DEG, min(MAX_STEP_DEG, d7))
            c8 = cur[J8] + max(-MAX_STEP_DEG, min(MAX_STEP_DEG, d8))
            self.send({J7: c7, J8: c8}, run_time=int(settle * 1000) + 300)
            self._sleep(settle, abort)
        return False

    def _sleep(self, seconds, abort):
        end = time.time() + seconds
        while time.time() < end:
            if abort is not None and abort.is_set():
                return
            time.sleep(0.05)

    # -- the wall line -----------------------------------------------------
    def draw_line(self, dist, z_top, z_bot, step_cm=0.5, settle=0.9,
                  abort=None, progress=None,
                  look=1.2, look_max=2.5, look_step=0.4):
        """Draw the vertical line in a Cartesian closed loop.

        The first version walked a list of absolute joint targets from
        draw_points().  At 26 cm a 1 cm step is only ~1.6 deg, which is inside
        the servo dead band, so the arm simply did not move, _step_to burned
        its eight retries and the run aborted -- "it stopped without moving".

        This follows the tip instead: FK the measured pose, project it onto
        the path, aim `look` centimetres further along, IK that point and
        send it.  When the arm sits still the lookahead grows (1.2 -> 2.5 cm),
        which is what pushes the command past the dead band without ever
        asking for a jump.

        dist / z_top / z_bot are line_tool's convention (centimetres, z
        relative to the j7 axis).  progress(fraction) is called every step.
        """
        cur = self.pose
        if cur is None:
            raise RuntimeError('no arm pose -- /CurrentAngle unreachable')
        ref = (cur[J7], cur[J8])
        cart, _ = L.cartesian_path(dist, z_top, z_bot, ref, step_cm=step_cm)
        if len(cart) < 2:
            return True
        cum = [0.0]
        for a, b in zip(cart, cart[1:]):
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        total = cum[-1]
        if total < 1e-6:
            return True

        down = ref[1] < L.J8_ZERO
        prev_s = -1.0
        still = 0
        for _ in range(600):
            if abort is not None and abort.is_set():
                return False
            p = self.pose
            if p is None:
                return False
            lx, lz = L.fk(p[J7], p[J8])
            s = self._project(cart, cum, lx, lz)
            if abs(s - prev_s) < 0.03:
                still += 1
            else:
                still = 0
            prev_s = s
            if progress is not None:
                progress(min(1.0, s / total))
            if s >= total - 0.05:
                return True
            aim = min(total, s + min(look_max, look + look_step * still))
            tx, tz = self._point_at(cart, cum, aim)
            q = self._ik_on_branch(tx, tz, down, p)
            if q is None:
                return False
            self.send({J7: q[0], J8: q[1]}, run_time=int(settle * 1000) + 300)
            self._sleep(settle, abort)
        return False

    @staticmethod
    def _project(cart, cum, lx, lz):
        """Arc length of the point on the polyline closest to (lx, lz)."""
        best = 0.0
        best_d = None
        for i in range(len(cart) - 1):
            ax, az = cart[i]
            bx, bz = cart[i + 1]
            dx, dz = bx - ax, bz - az
            seg = math.hypot(dx, dz)
            if seg < 1e-9:
                continue
            t = ((lx - ax) * dx + (lz - az) * dz) / (seg * seg)
            t = max(0.0, min(1.0, t))
            px, pz = ax + t * dx, az + t * dz
            d = math.hypot(lx - px, lz - pz)
            if best_d is None or d < best_d:
                best_d = d
                best = cum[i] + t * seg
        return best

    @staticmethod
    def _point_at(cart, cum, s):
        """Tip position at arc length s."""
        for i in range(len(cart) - 1):
            if cum[i + 1] >= s:
                seg = cum[i + 1] - cum[i]
                if seg < 1e-9:
                    return cart[i]
                t = (s - cum[i]) / seg
                ax, az = cart[i]
                bx, bz = cart[i + 1]
                return (ax + t * (bx - ax), az + t * (bz - az))
        return cart[-1]

    @staticmethod
    def _ik_on_branch(tx, tz, down, cur):
        """IK keeping the elbow branch; closest solution to the current pose."""
        sols = L.ik(tx, tz, margin=0.0)
        if not sols:
            return None
        same = [s for s in sols if (s[1] < L.J8_ZERO) == down]
        pick = same or sols
        return min(pick, key=lambda s: abs(s[0] - cur[J7]) + abs(s[1] - cur[J8]))

    def retract(self, cm=1.0, abort=None):
        """Back the tip straight away from the wall (decrease x), then stop.

        Used after drawing: the pen is still on the wall, so the chassis must
        not move until this has run.
        """
        cur = self.pose
        if cur is None:
            return False
        lx, lz = L.fk(cur[J7], cur[J8])
        sols = L.ik(max(0.0, lx - cm), lz, margin=0.0)
        if not sols:
            return False
        # keep the current elbow branch
        down = cur[J8] < L.J8_ZERO
        pick = [s for s in sols if (s[1] < L.J8_ZERO) == down] or sols
        q = min(pick, key=lambda s: abs(s[0] - cur[J7]) + abs(s[1] - cur[J8]))
        return self._step_to(q[0], q[1], settle=1.0, abort=abort)
