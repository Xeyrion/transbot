#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""arm_simple_gui.py -- simplest Tkinter panel for the arm-only Gazebo scene.

Two sliders map straight onto the two joints:

    arm_joint1 = radians(j7 - 95)
    arm_joint2 = radians(j8 - 180)

Dragging a slider sends the target to /arm_controller/command in small
ramped steps.  [Read current] copies /joint_states back into the sliders.
Nothing else: no presets and no calibration file. Optional real-robot sync
is enabled with --real; the sliders then also drive the real arm.
"""
from __future__ import division, print_function

import argparse
import math
import os
import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import Tkinter as tk

from arm_udp import ArmUdpClient, DEFAULT_PORT

import sys as _sys
_sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', '..', '..'))
import line_tool as L  # noqa: E402

J7_ZERO = 95.0        # j7 of a horizontal first link
J8_ZERO = 180.0       # j8 of collinear links (relative elbow angle)
J7_MIN, J7_MAX = 0.0, 225.0
J8_MIN, J8_MAX = 15.0, 240.0

# Single source of truth: measured values live in line_tool.py and are
# documented in CALIBRATION.md.  Do not copy the vendor URDF numbers here.
L1 = L.L1             # j7 axis -> j8 axis [cm]  (measured 10.0)
L2 = L.L2             # j8 axis -> pen tip [cm]  (measured 24.0)
J7_AXIS_X = L.J7_AXIS_X  # j7 axis world x [m]
J7_AXIS_Z = L.J7_AXIS_Z  # j7 axis world height [m]

Q1_MIN = math.radians(J7_MIN - J7_ZERO)
Q1_MAX = math.radians(J7_MAX - J7_ZERO)
Q2_MIN = math.radians(J8_MIN - J8_ZERO)
Q2_MAX = math.radians(J8_MAX - J8_ZERO)

STEP_RAD = 0.05       # max joint motion per tick [rad]
TICK_MS = 80

# Real-arm slow stream: one small target every interval, instead of jumping
# straight to the slider's final angle.  run_time is interval + 100 ms.
# The loop never gives up: if the servo does not move, the same target is
# re-sent every interval.  After a few still packets the lead grows a little
# (up to REAL_SLOW_LEAD_MAX) so a 1-2 deg servo deadband cannot stop the arm.
REAL_SLOW_INTERVAL = 0.5   # seconds between UDP set packets
REAL_SLOW_STEP = 2.0       # max joint change per packet [deg]
REAL_SLOW_TOL = 2.0        # reached within this error; covers 1-2 deg deadband
REAL_SLOW_LEAD_MAX = 5.0   # max lead used to break servo deadband [deg]

# Real line drawing is a Cartesian closed loop: every packet reads the real
# j7/j8, computes the measured pen tip, and aims at a point this far ahead
# along the planned tip path.  If the arm does not move, the lookahead grows
# (so the joint command also grows past the servo deadband) instead of the
# stream stopping.  The tip path is still checked against the table and the
# 2 cm reach margin before anything is sent.
REAL_CART_LOOKAHEAD = 1.2       # cm along the tip path (> 2 deg at 26 cm)
REAL_CART_LOOKAHEAD_STEP = 0.25  # extra cm per still packet
REAL_CART_LOOKAHEAD_MAX = 2.5   # cm
REAL_CART_END_TOL = 0.3         # cm from the final tip point

# UI language.  'zh' is the default and keeps the original panel exactly;
# 'en' is the same panel with English labels/messages for demos.
LANG = 'zh'


def tr(zh, en):
    """Pick the UI string for the current language."""
    return en if LANG == 'en' else zh


def wrap_pi(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def from_q(q1, q2):
    return math.degrees(q1) + J7_ZERO, math.degrees(q2) + J8_ZERO


def nearest_q2(target, current):
    cands = [target + 2.0 * math.pi * k for k in (-1, 0, 1)]
    cands = [c for c in cands if Q2_MIN - 1e-9 <= c <= Q2_MAX + 1e-9]
    if not cands:
        return min(max(target, Q2_MIN), Q2_MAX)
    return min(cands, key=lambda c: abs(c - current))


def slow_step(cur, goal, step):
    """Move from cur toward goal by at most step degrees (both joints)."""
    d7 = goal[0] - cur[0]
    d8 = goal[1] - cur[1]
    m = max(abs(d7), abs(d8))
    if m < 1e-9:
        return goal[0], goal[1]
    k = min(1.0, max(0.0, float(step)) / m)
    return cur[0] + d7 * k, cur[1] + d8 * k


class ArmSimpleGui(object):
    def __init__(self, root, args):
        self.root = root
        self.root.title(tr('Transbot 机械臂 — 最简控制面板',
                   'Transbot Arm — Simple Control Panel'))
        rospy.init_node('arm_simple_gui', anonymous=True)

        self.latest = {}
        self.target_q = None
        self.cmd_q = None
        self.waypoints = []
        self._ui_sync = False  # set_pose() 更新滑块时屏蔽 on_slide

        self.cmd_pub = rospy.Publisher(
            '/arm_controller/command', Float64MultiArray, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.on_joints,
                         queue_size=1)

        self.info_var = tk.StringVar(value='')
        self.status_var = tk.StringVar(value=tr('就绪', 'Ready'))
        self.real_var = tk.StringVar(value='')
        # Real sync is on by default when --real is given.  It only starts
        # sending after _startup_sync() aligns the sim to the real pose.
        self.real_sync_var = tk.BooleanVar(value=bool(args.real))
        self.real_slow_var = tk.BooleanVar(value=True)  # 真机默认慢慢跟
        self.real_slow_info = tk.StringVar(value='')
        self.follow_var = tk.BooleanVar(value=True)  # 默认只看仿真，不发命令
        self.real = None
        self.real_ready = False    # 对齐真机前绝不发 set
        self.last_real_send = 0.0
        self.last_real_target = None
        self.real_goal = None      # 真机慢速流最终想去的角
        self.real_cmd = None       # 最近发出去的真机目标
        self.real_last_pose = None  # 上次发包时的真机实测
        self.real_stall = 0        # 连续多少次实测没动（只用来加大 lead）
        self.real_waypoints = []   # 慢速画线：真机专用航点队列（旧路径）
        self.real_line_active = False  # 真机画线中：仿真跟随真机实测
        self.real_cart_path = []   # 笛卡尔闭环：笔尖路径 [(lx,lz),...]
        self.real_cart_cum = []    # 路径累计长度
        self.real_cart_idx = 0     # 当前进度（只增不减）
        self.real_cart_down = None  # 锁定的肘部支链
        if args.real:
            self.real = ArmUdpClient(args.real, args.real_port)
            self.real.start()

        tk.Label(root, text=tr('j7 根部 (°)', 'j7 base (°)'), anchor='w').pack(fill='x')
        self.s_j7 = tk.Scale(root, from_=J7_MIN, to=J7_MAX, resolution=0.1,
                             orient='horizontal', length=300)
        self.s_j7.pack(fill='x')
        tk.Label(root, text=tr('j8 小臂 (°)', 'j8 forearm (°)'), anchor='w').pack(fill='x')
        self.s_j8 = tk.Scale(root, from_=J8_MIN, to=J8_MAX, resolution=0.1,
                             orient='horizontal', length=300)
        self.s_j8.pack(fill='x')
        # 只在用户拖动/松开时发命令；set_pose() 的程序化 .set() 不触发 on_slide
        for scale in (self.s_j7, self.s_j8):
            scale.bind('<B1-Motion>', self._scale_moved)
            scale.bind('<ButtonRelease-1>', self._scale_moved)

        row = tk.Frame(root)
        row.pack(fill='x', pady=(4, 0))
        tk.Button(row, text=tr('读取当前', 'Read current'), command=self.read_now).pack(side='left')
        tk.Checkbutton(row, text=tr('跟随仿真', 'Follow sim'),
                       variable=self.follow_var).pack(side='left',
                                                     padx=(6, 0))
        tk.Label(row, textvariable=self.status_var, fg='#555',
                 anchor='w').pack(side='left', padx=(8, 0))

        row = tk.Frame(root)
        row.pack(fill='x')
        tk.Checkbutton(row, text=tr('同步到真机', 'Sync to real'),
                       variable=self.real_sync_var,
                       command=self.on_real_sync).pack(side='left')
        tk.Label(row, textvariable=self.real_var, fg='#555',
                 anchor='w').pack(side='left', padx=(8, 0))

        row = tk.Frame(root)
        row.pack(fill='x', pady=(2, 0))
        tk.Checkbutton(row, text=tr('慢速流', 'Slow stream'), variable=self.real_slow_var,
                       command=self.on_slow_toggle).pack(side='left')
        tk.Label(row, text=tr('间隔', 'Interval')).pack(side='left', padx=(6, 0))
        self.e_real_interval = tk.Entry(row, width=5)
        self.e_real_interval.insert(0, '%.1f' % REAL_SLOW_INTERVAL)
        self.e_real_interval.pack(side='left')
        tk.Label(row, text=tr('s  步长', 's  Step')).pack(side='left')
        self.e_real_step = tk.Entry(row, width=5)
        self.e_real_step.insert(0, '%.1f' % REAL_SLOW_STEP)
        self.e_real_step.pack(side='left')
        tk.Label(row, text=tr('°  容差', '°  Tolerance')).pack(side='left')
        self.e_real_tol = tk.Entry(row, width=5)
        self.e_real_tol.insert(0, '%.1f' % REAL_SLOW_TOL)
        self.e_real_tol.pack(side='left')
        tk.Label(row, text='°').pack(side='left')
        tk.Label(row, textvariable=self.real_slow_info, fg='#666',
                 anchor='w').pack(side='left', padx=(6, 0))

        row = tk.Frame(root)
        row.pack(fill='x', pady=(4, 0))
        tk.Label(row, text=tr('直线 距离', 'Line dist'), anchor='w').pack(side='left')
        self.e_dist = tk.Entry(row, width=7)
        self.e_dist.insert(0, '30.0')
        self.e_dist.pack(side='left')
        tk.Label(row, text=tr('z起', 'z start'), anchor='w').pack(side='left')
        self.e_z0 = tk.Entry(row, width=7)
        self.e_z0.insert(0, '-5.0')
        self.e_z0.pack(side='left')
        tk.Label(row, text=tr('z止', 'z end'), anchor='w').pack(side='left')
        self.e_z1 = tk.Entry(row, width=7)
        self.e_z1.insert(0, '5.0')
        self.e_z1.pack(side='left')
        tk.Button(row, text=tr('范围', 'Range'), command=self.show_range).pack(
            side='left', padx=(4, 0))
        tk.Button(row, text=tr('画线', 'Draw'), command=self.draw_seg).pack(
            side='left', padx=(4, 0))
        tk.Label(root,
                 text=tr('距离/高度都从 j7 轴起算 (cm)，+上/-下；'
                         '台面安全线 z >= %.1f',
                         'Distance/height from the j7 axis (cm), +up/-down; '
                         'table limit z >= %.1f') % L.Z_FLOOR_REL,
                 fg='#666', anchor='w').pack(fill='x')

        tk.Label(root, textvariable=self.info_var, justify='left',
                 anchor='w').pack(fill='x', pady=(4, 0))

        self.canvas = tk.Canvas(root, width=540, height=420, bg='#f7f7f5',
                                highlightthickness=1,
                                highlightbackground='#cccccc')
        self.canvas.pack(fill='both', expand=True, pady=(4, 0))
        self.canvas.bind('<Configure>', lambda _e: self.redraw())

        self.set_pose(args.j7, args.j8)
        self.root.after(TICK_MS, self.tick)
        self.root.after(1200, self._startup_sync)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    # ------------------------------------------------------------ sliders
    def _scale_moved(self, event=None):
        # 上半部分是数值显示区，点它不算拖动，免得误关跟随
        if event is not None:
            h = event.widget.winfo_height()
            if h > 0 and event.y < h * 0.55:
                return
        # 等 Scale 的默认绑定写完值再发，避免读到上一帧
        self.root.after_idle(self.on_slide)

    def on_slide(self, _=None):
        if self._ui_sync:
            return  # 程序化回读/跟随，不算用户拖动
        self.waypoints = []  # 手动接管：停掉正在画的线
        self._clear_real_line()
        if self.follow_var.get():
            self.follow_var.set(False)  # 手动拖动优先，跟随自动关
        self._goto(self.s_j7.get(), self.s_j8.get())

    def _goto(self, j7, j8):
        q1 = math.radians(j7 - J7_ZERO)
        q2 = wrap_pi(math.radians(j8 - J8_ZERO))
        cur = self.cmd_q if self.cmd_q is not None else self.measured_q()
        if cur is not None:
            q2 = nearest_q2(q2, cur[1])
        self.target_q = (q1, q2)
        self.real_goal = (j7, j8)
        self.real_stall = 0
        self.real_last_pose = None
        self.status_var.set(tr('移动中 ... 目标 j7=%.1f  j8=%.1f',
                       'Moving ... target j7=%.1f  j8=%.1f') % (j7, j8))
        self.maybe_send_real()
        self.redraw()

    # ------------------------------------------------------------ line tool
    def _line_inputs(self):
        try:
            return (float(self.e_dist.get()), float(self.e_z0.get()),
                    float(self.e_z1.get()))
        except ValueError:
            return None

    def show_range(self):
        vals = self._line_inputs()
        if vals is None:
            self.status_var.set(tr('距离/z起/z止请输入数字',
                               'Enter numbers for dist / z start / z end'))
            return
        dist, z0, z1 = vals
        # j7 axis is ~8 cm above the table; do not offer the part below the
        # table.  Manual z entries can still go lower if the robot is raised.
        runs, branch = L.line_range(dist, z_lo=L.Z_FLOOR_REL)
        if not runs:
            self.status_var.set(tr('dist %.1f 不可达：没有关节能让笔尖上这条线',
                                    'dist %.1f unreachable: no joint solution puts the tip on this line')
                                % dist)
            return
        a, b = runs[0]
        self.e_z0.delete(0, tk.END)
        self.e_z0.insert(0, '%.2f' % a)
        self.e_z1.delete(0, tk.END)
        self.e_z1.insert(0, '%.2f' % b)
        extra = '' if len(runs) == 1 else tr('（另有 %d 段）', ' (%d more segment(s))') % (len(runs) - 1)
        self.status_var.set(
            tr('dist %.1f 可画 z [%+.2f, %+.2f]（j7起，%s，臂展留 %.1fcm，'
               '台面留 %.1fcm）%s',
               'dist %.1f drawable z [%+.2f, %+.2f] (from j7 axis, %s, '
               'reach margin %.1fcm, table margin %.1fcm)%s')
            % (dist, a, b, branch or '?', L.REACH_MARGIN,
               L.FLOOR_MARGIN, extra))

    def _current_pose_deg(self):
        """Current arm angle: measured if available, else slider value."""
        q = self.measured_q()
        if q is None:
            return self.s_j7.get(), self.s_j8.get()
        return from_q(q[0], q[1])

    def draw_seg(self):
        vals = self._line_inputs()
        if vals is None:
            self.status_var.set(tr('距离/z起/z止请输入数字',
                               'Enter numbers for dist / z start / z end'))
            return
        dist, z0, z1 = vals
        z_start = min(z0, z1)
        real_line = (self.real is not None and self.real_sync_var.get()
                     and self.real_ready and self.real_slow_var.get())
        if real_line and self.real.state is not None:
            # Plan the approach from what the real arm actually reads, not
            # from the simulation, which may lag by one tick.
            ref = (self.real.state['j7'], self.real.state['j8'])
        else:
            ref = self._current_pose_deg()
        try:
            # cart = measured-tip path, path = equivalent joint waypoints.
            # The real loop follows cart in closed loop; the simulation uses
            # path directly when real sync is off.
            cart, path = L.cartesian_path(
                dist, z0, z1, ref,
                floor_z=L.Z_FLOOR_REL if real_line else None)
        except ValueError as e:
            self.status_var.set(
                tr('%s（先点「范围」；肘部支链不对就先手动收臂）',
                   '%s (click Range first; if the elbow branch is wrong, retract the arm manually)') % e)
            return
        if min(z0, z1) < L.Z_FLOOR_REL:
            self.status_var.set(
                tr('警告：z %.1f 低于台面安全线 %.1f；真机会顶到台面（可垫高机器人）',
                   'Warning: z %.1f is below the table limit %.1f; the real arm will hit the table (raise the robot)')
                % (min(z0, z1), L.Z_FLOOR_REL))
        if real_line:
            # Real arm follows the tip path in closed loop.  The simulation
            # does not run its own fast queue: tick() mirrors the real
            # measured angles, so it waits for the real arm.
            self.waypoints = []
            self.target_q = None
            self.real_waypoints = []
            self.real_cart_path = list(cart)
            self.real_cart_cum = self._cumulative(cart)
            self.real_cart_idx = 0
            self.real_cart_down = L.branch_down(ref)
            if self.real_cart_down is None and len(path) > 1:
                # Exactly at full extension: lock whichever branch the
                # planner picked after the first step.
                self.real_cart_down = path[1][1] < L.J8_ZERO
            self.real_line_active = True
            self.real_stall = 0
            self.real_last_pose = None
            self.last_real_send = time.time()
            self.real_slow_info.set(tr('笛卡尔 %.1fcm %d点', 'Cartesian %.1fcm %d pts')
                                    % (self.real_cart_cum[-1],
                                       len(self.real_cart_path)))
        else:
            self.waypoints = list(path)
            self._clear_real_line()
            self.dispatch_next()

    def dispatch_next(self):
        if not self.waypoints:
            return
        j7, j8 = self.waypoints.pop(0)
        self.set_pose(j7, j8)
        self._goto(j7, j8)

    def set_pose(self, j7, j8):
        self._ui_sync = True
        try:
            self.s_j7.set(min(max(j7, J7_MIN), J7_MAX))
            self.s_j8.set(min(max(j8, J8_MIN), J8_MAX))
        finally:
            self._ui_sync = False
        self.redraw()

    def read_now(self):
        if 'arm_joint1' not in self.latest:
            self.status_var.set(tr('还没有 /joint_states 回读', 'No /joint_states reading yet'))
            return
        j7, j8 = from_q(wrap_pi(self.latest['arm_joint1']),
                        wrap_pi(self.latest['arm_joint2']))
        self.set_pose(j7, j8)
        self.cmd_q = None
        self.target_q = None
        self.waypoints = []
        self._clear_real_line()
        self.status_var.set(tr('已从仿真读回  j7=%.1f  j8=%.1f',
                                  'Read back from sim  j7=%.1f  j8=%.1f') % (j7, j8))

    def follow_sim(self):
        """跟随模式：空闲时滑块/预览跟仿真走，只看不动手不发真机。"""
        if not self.follow_var.get():
            return
        if self.target_q is not None or self.waypoints:
            return  # 有正在发的目标时不抢
        if 'arm_joint1' not in self.latest:
            return
        j7, j8 = from_q(wrap_pi(self.latest['arm_joint1']),
                        wrap_pi(self.latest['arm_joint2']))
        if abs(j7 - self.s_j7.get()) + abs(j8 - self.s_j8.get()) < 0.05:
            return
        self.set_pose(j7, j8)
        self.cmd_q = None

    # ------------------------------------------------------------ command
    def measured_q(self):
        if 'arm_joint1' not in self.latest:
            return None
        q1 = wrap_pi(self.latest['arm_joint1'])
        q2 = wrap_pi(self.latest['arm_joint2'])
        return min(max(q1, Q1_MIN), Q1_MAX), q2

    def on_joints(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    # ------------------------------------------------------------ real sync
    def _startup_sync(self):
        """--real 默认勾选，但等 agent 回包后再对齐，避免开机就发目标。"""
        if self.real_sync_var.get():
            self.on_real_sync()

    def on_real_sync(self):
        if self.real is None:
            self.real_sync_var.set(False)
            self.real_ready = False
            self.real_var.set(tr('启动面板时加 --real IP 才有真机同步',
                            'Start the panel with --real IP to enable real sync'))
            return
        if not self.real_sync_var.get():
            self.real_ready = False
            self.real_goal = None
            self.real_cmd = None
            self.real_last_pose = None
            self.real_stall = 0
            self._clear_real_line()
            self.real_slow_info.set('')
            return
        self.follow_var.set(False)  # 真机同步优先，别让仿真回读抢滑块
        # 勾选瞬间不动真机：先让仿真滑块跟上真机当前位形。
        st = self.real.state
        if st is None or not self.real.connected() \
                or st.get('j7') is None or st.get('j8') is None:
            self.real_sync_var.set(False)
            self.real_ready = False
            self.real_var.set(tr('真机未连接：先启动 agent 再勾选',
                            'Real arm not connected: start the agent, then tick this'))
            return
        if not st.get('enabled', False):
            self.real_sync_var.set(False)
            self.real_ready = False
            self.real_var.set(tr('agent 是只读模式：重启为 --box 0 再勾选',
                            'Agent is read-only: restart with --box 0, then tick this'))
            self.real_slow_info.set(tr('只读 agent：不能同步', 'Read-only agent: cannot sync'))
            return
        self.real_ready = True
        self._clear_real_line()
        self.real_goal = (st['j7'], st['j8'])
        self.real_cmd = (st['j7'], st['j8'])
        self.real_last_pose = (st['j7'], st['j8'])
        self.real_stall = 0
        self.last_real_target = (st['j7'], st['j8'])
        self.last_real_send = time.time()
        self.set_pose(st['j7'], st['j8'])
        self.on_slide()  # 仿真跟过去；发回真机的是同一位形无动作
        self.status_var.set(tr('已对齐真机 j7=%.1f j8=%.1f；拖滑块会同时动真机',
                            'Aligned to real j7=%.1f j8=%.1f; sliders now move the real arm too')
                            % (st['j7'], st['j8']))

    def on_slow_toggle(self):
        self.real_cmd = None
        self.real_last_pose = None
        self.real_stall = 0
        self._clear_real_line()
        self.last_real_send = time.time()
        if self.real_sync_var.get():
            self.real_goal = (self.s_j7.get(), self.s_j8.get())
        self.real_slow_info.set('')

    def _clear_real_line(self):
        """Stop any real line/approach and drop its Cartesian path."""
        self.real_waypoints = []
        self.real_cart_path = []
        self.real_cart_cum = []
        self.real_cart_idx = 0
        self.real_cart_down = None
        self.real_line_active = False

    @staticmethod
    def _cumulative(points):
        """Cumulative arc length of a tip polyline."""
        cum = [0.0]
        for i in range(1, len(points)):
            cum.append(cum[-1] + math.hypot(
                points[i][0] - points[i - 1][0],
                points[i][1] - points[i - 1][1]))
        return cum

    @staticmethod
    def _real_limits(st):
        """Same start +/- box envelope the agent uses (0 = servo limits)."""
        box = st.get('box') or 0.0
        s7, s8 = st.get('start_j7'), st.get('start_j8')
        if box and s7 is not None and s8 is not None:
            return (max(J7_MIN, s7 - box), min(J7_MAX, s7 + box),
                    max(J8_MIN, s8 - box), min(J8_MAX, s8 + box))
        return J7_MIN, J7_MAX, J8_MIN, J8_MAX

    def maybe_send_real(self):
        if self.real is None or not self.real_sync_var.get():
            return
        if not self.real_ready:
            return
        if not self.real.connected():
            return
        st = self.real.state
        if st is None or st.get('j7') is None or st.get('j8') is None:
            return
        if self.real_slow_var.get():
            self._send_real_slow(st)
        else:
            self._send_real_direct(st)

    def _send_real_direct(self, st):
        now = time.time()
        if now - self.last_real_send < 0.4:
            return
        j7, j8 = self.s_j7.get(), self.s_j8.get()
        lo7, hi7, lo8, hi8 = self._real_limits(st)
        j7 = min(max(j7, lo7), hi7)
        j8 = min(max(j8, lo8), hi8)
        prev = self.last_real_target
        if prev is not None and max(abs(j7 - prev[0]),
                                   abs(j8 - prev[1])) < 1.0:
            return
        if self.real.send_set(j7, j8):
            self.last_real_send = now
            self.last_real_target = (j7, j8)

    def _cart_index_near(self, tip):
        """Closest path index in a small forward window; progress only grows."""
        path = self.real_cart_path
        n = len(path)
        if n == 0:
            return 0, 0.0
        lo = max(0, self.real_cart_idx - 2)
        # A 3 cm forward window is enough: one packet moves the tip far less.
        # Keeping it small also stops the search from jumping across a short
        # approach straight onto the vertical line.
        hi = min(n - 1, self.real_cart_idx + 6)
        best_i = self.real_cart_idx
        best_d2 = None
        for i in range(lo, hi + 1):
            px, pz = path[i]
            d2 = (px - tip[0]) ** 2 + (pz - tip[1]) ** 2
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_i = i
        # At the bottom of the approach the path reverses (down to the line
        # start, then up).  With a servo deadband the tip can stop a few mm
        # short, making two points on either side of the cusp equally close;
        # without this forward tie-break the controller oscillates there.
        # Pick the farthest point that is still within 1 mm of the closest.
        best_d = math.sqrt(best_d2 or 0.0)
        for i in range(best_i, hi + 1):
            px, pz = path[i]
            if math.hypot(px - tip[0], pz - tip[1]) <= best_d + 0.1:
                best_i = i
        if best_i > self.real_cart_idx:
            self.real_cart_idx = best_i
        return self.real_cart_idx, math.sqrt(best_d2 or 0.0)

    def _cart_point_at(self, s):
        """Interpolate (lx, lz) at arc length s along the tip path."""
        path = self.real_cart_path
        cum = self.real_cart_cum
        if not path:
            return (0.0, 0.0)
        if s <= 0.0:
            return path[0]
        if s >= cum[-1]:
            return path[-1]
        for i in range(1, len(path)):
            if cum[i] >= s:
                seg = cum[i] - cum[i - 1]
                t = 0.0 if seg < 1e-9 else (s - cum[i - 1]) / seg
                return (path[i - 1][0] + (path[i][0] - path[i - 1][0]) * t,
                        path[i - 1][1] + (path[i][1] - path[i - 1][1]) * t)
        return path[-1]

    def _send_real_cartesian(self, st, interval, step, tol, now):
        """One packet of the Cartesian closed loop for a real line.

        Every packet: read real j7/j8 -> FK to the actual pen tip -> find how
        far along the planned tip path we are -> aim at a point lookahead cm
        farther -> IK -> send one small joint step.  If the arm stops moving,
        the lookahead and joint lead grow, so the command itself grows past
        the 1-2 deg servo deadband instead of the stream stopping.
        """
        path = self.real_cart_path
        cum = self.real_cart_cum
        if len(path) < 2 or len(cum) != len(path):
            self._clear_real_line()
            self.real_slow_info.set(tr('line 路径为空', 'line path is empty'))
            return
        cur = (st['j7'], st['j8'])
        lo7, hi7, lo8, hi8 = self._real_limits(st)
        tip = L.fk(cur[0], cur[1])
        idx, _ = self._cart_index_near(tip)
        s = cum[idx]
        total = cum[-1]
        final = path[-1]
        tip_err = math.hypot(tip[0] - final[0], tip[1] - final[1])

        # Still for a packet?  Grow both the Cartesian lookahead and the joint
        # lead.  This is what breaks the servo deadband without overshooting
        # the line by a fixed amount.
        if self.real_last_pose is not None:
            moved = abs(cur[0] - self.real_last_pose[0]) + \
                abs(cur[1] - self.real_last_pose[1])
            if moved < 0.2:
                self.real_stall += 1
            else:
                self.real_stall = 0

        final_sols = L.ik_branch(final[0], final[1], self.real_cart_down,
                                 margin=0.0)
        final_err = None
        box_hit = False
        if final_sols:
            final_clipped = [(min(max(q[0], lo7), hi7),
                              min(max(q[1], lo8), hi8))
                             for q in final_sols]
            box_hit = any(
                abs(q[0] - r[0]) > 1e-6 or abs(q[1] - r[1]) > 1e-6
                for q, r in zip(final_clipped, final_sols))
            final_err = min(max(abs(q[0] - cur[0]), abs(q[1] - cur[1]))
                            for q in final_clipped)
        # Only end after almost the whole path, so starting near the top of a
        # line does not "finish" before the approach.  At the end, the same
        # --容差 as the slider mode covers the servo deadband.
        if s >= total - 0.6 and (
                (final_err is not None and final_err <= tol)
                or tip_err <= REAL_CART_END_TOL):
            self._clear_real_line()
            self.real_stall = 0
            self.real_last_pose = cur
            self.real_slow_info.set(tr('line 到 box 边界', 'line stopped at box limit') if box_hit
                                    else tr('line 完成', 'line done'))
            return

        look = REAL_CART_LOOKAHEAD + REAL_CART_LOOKAHEAD_STEP * \
            max(0, self.real_stall - 1)
        look = min(look, REAL_CART_LOOKAHEAD_MAX)
        tx, tz = self._cart_point_at(min(s + look, total))
        sols = L.ik_branch(tx, tz, self.real_cart_down, margin=0.0)
        if not sols:
            self.real_slow_info.set(tr('笛卡尔目标不可达 x=%.1f z=%.1f',
                                    'Cartesian target unreachable x=%.1f z=%.1f')
                                    % (tx, tz))
            return
        q = min(sols, key=lambda p: (p[0] - cur[0]) ** 2 +
                (p[1] - cur[1]) ** 2)
        # Respect the agent's optional box envelope, not just servo limits.
        q_raw = q
        q = (min(max(q[0], lo7), hi7), min(max(q[1], lo8), hi8))

        lead = step
        if self.real_stall >= 2:
            lead = min(REAL_SLOW_LEAD_MAX,
                       step + 0.5 * (self.real_stall - 1))
            lead = max(lead, step)
        t7, t8 = slow_step(cur, q, lead)
        rt = int(min(max(interval * 1000.0 + 100.0, 100.0), 2000.0))
        if self.real.send_set(t7, t8, run_time=rt):
            self.last_real_send = now
            self.last_real_target = (t7, t8)
            self.real_cmd = (t7, t8)
            self.real_last_pose = cur
            self.real_goal = q
            jerr = max(abs(q[0] - cur[0]), abs(q[1] - cur[1]))
            waiting = (tr(' 等%d包', ' wait %d pkt') % self.real_stall) if self.real_stall else ''
            clipped = (abs(q[0] - q_raw[0]) > 1e-6 or
                       abs(q[1] - q_raw[1]) > 1e-6)
            self.real_slow_info.set(
                tr('笛 %.1f/%.1fcm 差%.1f°%s%s',
                'Cart %.1f/%.1fcm err %.1f°%s%s')
                % (s, total, jerr,
                   ' [box]' if clipped else '', waiting))

    def _send_real_slow(self, st):
        """Keep one small target flowing; never stop just because it is still.

        The measured angle is read every packet and the next target is always
        relative to it.  If the arm does not move, the same target is re-sent,
        and after two still packets the lead grows slowly (up to
        REAL_SLOW_LEAD_MAX) so a 1-2 deg servo deadband cannot park the arm.
        A waypoint is considered reached within --容差, not exactly.
        """
        try:
            interval = float(self.e_real_interval.get())
            step = float(self.e_real_step.get())
            tol = float(self.e_real_tol.get())
        except ValueError:
            self.real_slow_info.set(tr('间隔/步长/容差要数字', 'Interval/step/tolerance must be numbers'))
            return
        interval = min(max(interval, 0.2), 2.0)
        step = min(max(step, 0.2), 10.0)
        tol = min(max(tol, 0.5), 5.0)
        now = time.time()
        if now - self.last_real_send < interval:
            return

        if self.real_line_active and self.real_cart_path:
            self._send_real_cartesian(st, interval, step, tol, now)
            return

        lo7, hi7, lo8, hi8 = self._real_limits(st)
        cur = (st['j7'], st['j8'])

        # Pop every waypoint already inside tolerance before choosing the
        # next target.  A dense approach path can have several points inside
        # 1.5 deg; returning after each one would waste one interval per
        # point and look like the real arm had stopped.
        while self.real_waypoints:
            raw7, raw8 = self.real_waypoints[0]
            g7 = min(max(raw7, lo7), hi7)
            g8 = min(max(raw8, lo8), hi8)
            if max(abs(g7 - cur[0]), abs(g8 - cur[1])) > tol:
                break
            self.real_waypoints.pop(0)
            self.real_stall = 0
            self.real_last_pose = None

        if not self.real_waypoints and self.real_line_active:
            # The whole approach+line is within tolerance of where the arm
            # already is.  Do not fall back to the slider; the line is done.
            self._clear_real_line()
            self.real_stall = 0
            self.real_last_pose = cur
            self.real_slow_info.set(tr('line 完成', 'line done'))
            return

        if self.real_waypoints:
            raw7, raw8 = self.real_waypoints[0]
        else:
            raw7, raw8 = self.s_j7.get(), self.s_j8.get()
            self._clear_real_line()
        g7 = min(max(raw7, lo7), hi7)
        g8 = min(max(raw8, lo8), hi8)
        self.real_goal = (g7, g8)
        clipped = (abs(g7 - raw7) > 1e-6 or abs(g8 - raw8) > 1e-6)

        d7, d8 = g7 - cur[0], g8 - cur[1]
        err = max(abs(d7), abs(d8))

        if not self.real_waypoints and err < 0.3:
            self.real_stall = 0
            self.real_last_pose = cur
            self.real_slow_info.set(tr('已到 box 边界', 'Reached box limit') if clipped
                                    else tr('真机已到位', 'Real arm in position'))
            return

        # Still for two packets?  Grow the lead a little instead of stopping.
        if self.real_last_pose is not None:
            moved = abs(cur[0] - self.real_last_pose[0]) + \
                abs(cur[1] - self.real_last_pose[1])
            if moved < 0.2:
                self.real_stall += 1
            else:
                self.real_stall = 0
        lead = step
        if self.real_stall >= 2:
            lead = min(REAL_SLOW_LEAD_MAX,
                       step + 0.5 * (self.real_stall - 1))
            lead = max(lead, step)

        t7, t8 = slow_step(cur, (g7, g8), lead)
        rt = int(min(max(interval * 1000.0 + 100.0, 100.0), 2000.0))
        if self.real.send_set(t7, t8, run_time=rt):
            self.last_real_send = now
            self.last_real_target = (t7, t8)
            self.real_cmd = (t7, t8)
            self.real_last_pose = cur
            waiting = (tr(' 等%d包', ' wait %d pkt') % self.real_stall) if self.real_stall else ''
            self.real_slow_info.set(
                tr('%.1fs/%.1f° 差%.1f° → %.1f, %.1f%s%s',
                   '%.1fs/%.1f° err %.1f° → %.1f, %.1f%s%s')
                % (interval, lead, err, t7, t8,
                   ' [box]' if clipped else '', waiting))

    def refresh_real(self):
        if self.real is None:
            return
        st = self.real.state
        if st is not None and self.real.connected():
            volt = st.get('voltage')
            vtxt = ('%.1fV' % volt) if isinstance(volt, (int, float)) else '--'
            box = st.get('box')
            btxt = tr('无box限制', 'no box limit') if not box else ('box±%g°' % box)
            mode = '' if st.get('enabled') else tr(' [只读]', ' [read-only]')
            self.real_var.set(tr('真机 j7=%.1f j8=%.1f %s %s%s',
                              'Real j7=%.1f j8=%.1f %s %s%s')
                              % (st.get('j7', 0.0), st.get('j8', 0.0),
                                 vtxt, btxt, mode))
        elif self.real_sync_var.get():
            self.real_var.set(tr('真机未连接（已暂停发送）', 'Real arm disconnected (sending paused)'))

    def _mirror_real(self, st):
        """Real slow line: drive the sim from the measured real angles.

        This is the key to "sim waits for the real arm": there is no separate
        sim waypoint queue racing ahead.  The sim target is refreshed from the
        real state every tick, so if the real arm stalls, the sim stalls too.
        """
        j7, j8 = st['j7'], st['j8']
        self.set_pose(j7, j8)
        q1 = math.radians(j7 - J7_ZERO)
        q2 = wrap_pi(math.radians(j8 - J8_ZERO))
        cur = self.cmd_q if self.cmd_q is not None else self.measured_q()
        if cur is not None:
            q2 = nearest_q2(q2, cur[1])
        self.target_q = (q1, q2)

    def tick(self):
        if rospy.is_shutdown():
            self.on_close()
            return
        if self.real_line_active and self.real_sync_var.get() \
                and self.real_ready:
            st = self.real.state
            if st is not None and self.real.connected() \
                    and st.get('j7') is not None and st.get('j8') is not None:
                self._mirror_real(st)
        if self.target_q is not None:
            if self.cmd_pub.get_num_connections() == 0:
                pass  # 控制器还没起来，保留目标下次再试
            else:
                cur = self.cmd_q
                if cur is None:
                    cur = self.measured_q()
                if cur is None:
                    cur = self.target_q
                tq = self.target_q
                d1, d2 = tq[0] - cur[0], tq[1] - cur[1]
                if max(abs(d1), abs(d2)) <= 1e-4:
                    self.cmd_q = tq
                    self.target_q = None
                    if not self.real_line_active:
                        self.status_var.set(tr('已到位  j7=%.1f  j8=%.1f',
                                            'In position  j7=%.1f  j8=%.1f')
                                            % (self.s_j7.get(),
                                               self.s_j8.get()))
                        self.dispatch_next()  # 画线：继续下一航点
                else:
                    k = min(1.0, STEP_RAD / max(abs(d1), abs(d2)))
                    self.cmd_q = (cur[0] + d1 * k, cur[1] + d2 * k)
                if self.cmd_q is not None:
                    self.cmd_pub.publish(
                        Float64MultiArray(data=list(self.cmd_q)))
        # 补发真机：拖滑块太快被 0.4s 节流丢掉的那一次，在 tick 里补上。
        self.maybe_send_real()
        e1 = self.s_j7.get() - J7_ZERO
        rel = self.s_j8.get() - J8_ZERO
        e2 = e1 + rel
        tipx = L1 * math.cos(math.radians(e1)) \
            + L2 * math.cos(math.radians(e2))
        tipz = L1 * math.sin(math.radians(e1)) \
            + L2 * math.sin(math.radians(e2))
        j = self.latest
        rtxt = ('j7 = %6.1f°   j8 = %6.1f°' % from_q(j['arm_joint1'],
                                                     j['arm_joint2'])
                if 'arm_joint1' in j else tr('等待 /joint_states ...', 'Waiting for /joint_states ...'))
        self.info_var.set(tr(
            'e1 = %+6.1f°  rel = %+6.1f°  e2 = %+6.1f°\n'
            '笔尖 x = %6.2f cm   z = %6.2f cm\n'
            '关节回读     %s',
            'e1 = %+6.1f°  rel = %+6.1f°  e2 = %+6.1f°\n'
            'Tip x = %6.2f cm   z = %6.2f cm\n'
            'Joint feedback  %s') % (e1, rel, e2, tipx, tipz, rtxt))
        self.refresh_real()
        self.follow_sim()
        self.root.after(TICK_MS, self.tick)

    # ---------------------------------------------------------------- draw
    def redraw(self):
        c = self.canvas
        c.delete('all')
        W = max(c.winfo_width(), 540)
        H = max(c.winfo_height(), 420)
        j7, j8 = self.s_j7.get(), self.s_j8.get()

        xmin, xmax = -0.10, 0.50
        zmin, zmax = -0.10, 0.46
        s = min((W - 20.0) / (xmax - xmin), (H - 20.0) / (zmax - zmin))
        ox = 10 - xmin * s
        oy = 10
        m = lambda x, z: (ox + x * s, H - oy - (z - zmin) * s)

        e1 = math.radians(j7 - J7_ZERO)
        e2 = math.radians((j7 - J7_ZERO) + (j8 - J8_ZERO))
        j7x, j7z = J7_AXIS_X, J7_AXIS_Z
        j8x = j7x + L1 / 100.0 * math.cos(e1)
        j8z = j7z + L1 / 100.0 * math.sin(e1)
        tipx = j8x + L2 / 100.0 * math.cos(e2)
        tipz = j8z + L2 / 100.0 * math.sin(e2)

        c.create_line(m(j7x, j7z), m(j8x, j8z), fill='#1a5cd6', width=7,
                      capstyle='round')
        c.create_line(m(j8x, j8z), m(tipx, tipz), fill='#e06a0a', width=6,
                      capstyle='round')
        for x, z, r, col in ((j7x, j7z, 5, '#333333'),
                             (j8x, j8z, 4, '#333333'),
                             (tipx, tipz, 6, '#d00000')):
            px, py = m(x, z)
            c.create_oval(px - r, py - r, px + r, py + r, fill=col,
                          outline='')
        c.create_text(12, 12, anchor='nw',
                      text='j7=%.1f°  j8=%.1f°' % (j7, j8), fill='#333333')

    # ----------------------------------------------------------------- misc
    def on_close(self):
        if self.real is not None:
            try:
                self.real.stop()
            except Exception:
                pass
            self.real = None
        try:
            rospy.signal_shutdown('GUI closed')
        except Exception:
            pass
        self.root.destroy()


def main():
    global LANG
    argv = rospy.myargv(argv=sys.argv)[1:]
    # Pick the language before argparse builds --help, so English demos get
    # English help text too.  argparse validates the value afterwards.
    for i, a in enumerate(argv):
        if a == '--lang' and i + 1 < len(argv):
            LANG = argv[i + 1]
        elif a.startswith('--lang='):
            LANG = a.split('=', 1)[1]
    ap = argparse.ArgumentParser(description='simplest arm-only Gazebo GUI')
    ap.add_argument('--j7', type=float, default=169.6)
    ap.add_argument('--j8', type=float, default=105.4)
    ap.add_argument('--real', default='127.0.0.1',
                    help=tr('真机 UDP agent 的 IP（默认 127.0.0.1，直连本机 agent）；'
                            '设为空字符串则关闭真机同步',
                            'Real-arm UDP agent IP (default 127.0.0.1, local '
                            'agent); empty string disables real sync'))
    ap.add_argument('--real-port', type=int, default=DEFAULT_PORT)
    ap.add_argument('--lang', choices=('zh', 'en'), default='zh',
                    help=tr('面板语言：zh=中文，en=English（默认 zh）',
                            'Panel language: zh=Chinese, en=English '
                            '(default zh)'))
    args = ap.parse_args(argv)
    LANG = args.lang
    ArmSimpleGui(tk.Tk(), args).root.mainloop()


if __name__ == '__main__':
    main()
