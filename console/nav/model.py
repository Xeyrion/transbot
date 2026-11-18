# -*- coding: utf-8 -*-
"""Everything in the navigation tab that is not a widget.

The panel is rebuilt from scratch when the UI language changes (Tk cannot
retranslate live widgets), so anything that must survive that -- the running
stack, the ROS subscriptions, the map image, the goal -- lives here instead
of on the widget tree.

The model does not publish velocity itself: teleop goes through
`app.robot.manual()`, which already owns the 20 Hz republish and the
motion-generation guard.
"""

import math
import threading
import time

import rospy
import nav_msgs.msg
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

try:
    from tf2_msgs.msg import TFMessage
    _HAS_TF_MSG = True
except Exception:
    TFMessage = None
    _HAS_TF_MSG = False

try:
    from transbot_msgs.msg import Battery
    _HAS_BATTERY = True
except Exception:
    Battery = None
    _HAS_BATTERY = False

from .mapview import LiveMap, TFLite
from .stack import NavStack, MODE_MAP, MODE_NAV, MODE_SLAMNAV


class NavState(object):
    def __init__(self, nav_dir, log=None):
        self.stack = NavStack(nav_dir, log=log)
        self.livemap = LiveMap('dark')
        self.tf = TFLite()

        # UI selections, kept across language rebuilds
        self.mode = MODE_MAP
        self.name = self.stack.default_name()
        self.map_name = ''
        self.teleop = False
        self.maps = []

        # live data
        self.pose = None
        self.amcl_pose = None
        self.sigma = None          # (sx, sy, sth) from /amcl_pose covariance
        self.voltage = None
        self.scan = None
        self.plan = None
        self.goal = None
        self.goal_state = None
        self.trail = []
        self.odom_m = 0.0
        self._last_odom_xy = None
        self._lock = threading.Lock()

        # keyboard teleop
        self.keys = set()
        self.last_cmd = (0.0, 0.0)

        self.subs_done = False
        self.last_error = ''
        self.pub_goal = None
        self.pub_cancel = None
        self.pub_pose = None

    def set_log(self, log):
        self.stack.log = log

    # -- subscriptions ------------------------------------------------
    def ensure_subs(self):
        """Create the subscriptions once. Returns True when they are up.

        Called from the panel's tick rather than at import time: rospy needs
        the GUI's node to exist first, and the panel is built before the
        connection is confirmed.
        """
        if self.subs_done:
            return True
        self.last_error = ''
        try:
            rospy.Subscriber('/map', OccupancyGrid, self._cb_map)
            rospy.Subscriber('/scan', LaserScan, self._cb_scan)
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                             self._cb_amcl)
            rospy.Subscriber('/odom', nav_msgs.msg.Odometry, self._cb_odom)
            rospy.Subscriber('/move_base/status', GoalStatusArray,
                             self._cb_status)
            rospy.Subscriber('/move_base/GlobalPlanner/plan',
                             nav_msgs.msg.Path, self._cb_plan)
            if _HAS_TF_MSG:
                rospy.Subscriber('/tf', TFMessage, self.tf.cb)
                rospy.Subscriber('/tf_static', TFMessage, self.tf.cb)
            if _HAS_BATTERY:
                rospy.Subscriber('/voltage', Battery, self._cb_voltage)
            self.pub_goal = rospy.Publisher('/move_base_simple/goal',
                                            PoseStamped, queue_size=1)
            self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID,
                                              queue_size=1)
            self.pub_pose = rospy.Publisher('/initialpose',
                                            PoseWithCovarianceStamped,
                                            queue_size=1)
            self.subs_done = True
            return True
        except Exception as e:
            self.last_error = str(e)
            return False

    def _cb_map(self, msg):
        self.livemap.update(msg)

    def _cb_scan(self, msg):
        self.scan = msg

    def _cb_amcl(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z),
                         1.0 - 2.0 * p.orientation.z * p.orientation.z)
        self.amcl_pose = (p.position.x, p.position.y, yaw)
        c = msg.pose.covariance
        # AMCL reports the filter's own confidence. It is not ground truth, but
        # a sudden jump in it is the cheapest available "the pose went wrong"
        # signal -- and it is the only one when the map has a twin solution.
        try:
            self.sigma = (math.sqrt(max(c[0], 0.0)),
                          math.sqrt(max(c[7], 0.0)),
                          math.degrees(math.sqrt(max(c[35], 0.0))))
        except Exception:
            self.sigma = None

    def _cb_voltage(self, msg):
        try:
            self.voltage = float(msg.Voltage)
        except Exception:
            pass

    def _cb_odom(self, msg):
        with self._lock:
            xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self._last_odom_xy is not None:
                d = math.hypot(xy[0] - self._last_odom_xy[0],
                               xy[1] - self._last_odom_xy[1])
                if d < 1.0:          # a jump means a new session, not travel
                    self.odom_m += d
            self._last_odom_xy = xy

    def _cb_status(self, msg):
        if not msg.status_list:
            self.goal_state = None
            return
        st = msg.status_list[-1]
        self.goal_state = (st.status, getattr(st, 'text', ''))

    def _cb_plan(self, msg):
        self.plan = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]

    # -- derived ------------------------------------------------------
    def pose_now(self):
        if _HAS_TF_MSG:
            t = self.tf.lookup('map', 'base_footprint')
            if t is not None:
                return t
        return self.amcl_pose

    def scan_points(self):
        msg = self.scan
        if msg is None or not _HAS_TF_MSG:
            return None
        t = self.tf.lookup('map', msg.header.frame_id)
        if t is None:
            return None
        x0, y0, a0 = t
        ca, sa = math.cos(a0), math.sin(a0)
        out = []
        ang = msg.angle_min
        step = msg.angle_increment
        for r in msg.ranges:
            if msg.range_min < r < min(msg.range_max, 12.0):
                lx = r * math.cos(ang)
                ly = r * math.sin(ang)
                out.append((x0 + ca * lx - sa * ly, y0 + sa * lx + ca * ly))
            ang += step
        return out

    def push_trail(self):
        pose = self.pose_now()
        self.pose = pose
        if pose is None:
            return None
        if not self.trail or math.hypot(pose[0] - self.trail[-1][0],
                                        pose[1] - self.trail[-1][1]) > 0.03:
            self.trail.append((pose[0], pose[1]))
            if len(self.trail) > 6000:
                del self.trail[:2000]
        return pose

    # -- actions ------------------------------------------------------
    def send_goal(self, x, y, yaw):
        if self.pub_goal is None:
            return False
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(m)
        self.goal = (x, y, yaw)
        return True

    def cancel_goal(self):
        if self.pub_cancel is not None:
            try:
                self.pub_cancel.publish(GoalID())
            except Exception:
                pass
        self.goal = None
        self.plan = None

    def set_initial_pose(self, x, y, yaw):
        if self.pub_pose is None:
            return False
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
        for i, v in enumerate(cov):
            m.pose.covariance[i] = v
        self.pub_pose.publish(m)
        return True

    def reset_view_state(self):
        self.trail = []
        self.plan = None
        self.goal = None
        self.goal_state = None
