#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""task_gui.py -- one window for the whole wall task: navigate, align, draw.

Why one window with stages instead of three programs
----------------------------------------------------
The three steps share the same /cmd_vel and the same robot, and they must not
run at the same time.  A stage machine makes that explicit: the map only
accepts goals in stage 1, the jog pad only works in stage 2, the arm only
moves in stage 3.  Switching stages cancels whatever the previous one was
doing, so you cannot send a goal while the pen is against the wall.

What each stage does
   1 nav    click the map -> move_base goal.  Goals closer than --clear to an
            obstacle are refused, because the arm sticks out ~27 cm in front
            of the chassis and the lidar cannot see it.
   2 align  camera + jog pad.  Arm parked, lidar reads the wall, one button
            converts that reading into line_tool's "distance".
   3 draw   the vertical line: down from z_top to z_bot at x=dist, closed
            loop on the measured joint angles.

Usage
    ./run_task_gui.sh                       # robot at the default address
    ./run_task_gui.sh --map ../maps/fig8.yaml
    ./run_task_gui.sh --offline             # no ROS, just look at the layout

The nav stack must already be up (map_server + amcl + move_base).
"""
import argparse
import json
import math
import os
import queue
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import numpy as np
from PIL import Image, ImageTk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'wallline'))

import line_tool as L          # noqa: E402
from mapdata import MapData    # noqa: E402

try:
    import cv2
    _HAS_CV2 = True
except Exception:
    cv2 = None
    _HAS_CV2 = False

try:
    from cv_bridge import CvBridge
    _BRIDGE = CvBridge()
except Exception:
    _BRIDGE = None

# Astra publishes raw sensor_msgs/Image only; the /compressed topic exists on
# the master as a stale registration but nothing publishes on it.
CAM_TOPIC = '/camera/rgb/image_raw'
# Same three patterns the console uses.  The bracketed first letter keeps
# pkill from matching the ssh session that carries the command.
ASTRA_KILL = ('[a]stra_camera', '[a]strapro.launch', '[c]amera_nodelet_manager')

# base_link -> laser is (-0.03, 0, 0.13) with yaw pi, so the beam that points
# forward sits at +pi in the laser frame and the lidar is 15.9 cm behind the
# j7 axis.  Verified once with a tape; if a map rebuild changes the mount,
# re-measure before trusting the "laser -> distance" button.
LASER_TO_J7 = 0.159        # m, j7 axis minus laser origin, along +x
FRONT_HALF_ANGLE = 6.0     # deg, sector averaged for the wall distance

STAGE_ORDER = ('nav', 'align', 'draw')
STAGE_TITLE = {'nav': '1  导航', 'align': '2  对准', 'draw': '3  画线'}

MAP_W, MAP_H = 920, 450        # left canvas; the map is 2.13:1, so a taller
                               # canvas just leaves empty bands
CAM_W, CAM_H = 560, 400        # 4:3 camera view
RIGHT_W = 580

# Where maps live.  maps/current holds the name the stack loads; the picker
# writes it and restarts the stack (map_server reads the yaml only at start).
MAPS_DIR = os.path.abspath(os.path.join(HERE, '..', 'maps'))

# Servo hard limits, same numbers as console/transbot_gui.py and line_tool.
ARM_LIMITS = {7: (0.0, 225.0), 8: (15.0, 240.0), 9: (60.0, 180.0)}

# ---------------------------------------------------------------------------
# translation
# ---------------------------------------------------------------------------
# Same approach as console/transbot_gui.py: one table keyed by the Chinese
# literal, a single T() lookup.  Static widget text is translated by walking
# the widget tree once after it is built; anything that changes at runtime is
# wrapped in T() where it is formatted.
LANG = 'zh'

_EN = {
    '1  导航': '1  Navigate',
    '2  对准': '2  Align',
    '3  画线': '3  Draw',
    'Transbot 任务台 -- 导航 / 对准 / 画线':
        'Transbot Task Console -- navigate / align / draw',
    '急停  ESC': 'STOP  ESC',
    '相机未连接\n/camera/rgb/image_raw/compressed':
        'Camera not connected\n/camera/rgb/image_raw/compressed',
    '下一步：对准  →': 'Next: align  →',
    '下一步：画线  →': 'Next: draw  →',
    '点地图设目标，右键取消。':
        'Click the map to set a goal; right-click cancels.',
    '取消目标': 'Cancel goal',
    '激光点': 'Laser',
    '清空距离 (m)': 'Clearance (m)',
    '重新定位': 'Re-localise',
    '相机开': 'Camera on',
    '相机关': 'Camera off',
    '设初始位姿（位置→朝向）':
        'Set pose (position → heading)',
    '指定目标朝向（点两下）': 'Set goal heading (two clicks)',
    '记录': 'Record',
    '停止记录': 'Stop rec',
    '离线模式，无法记录': 'Offline: cannot record',
    '找不到 %s': 'Not found: %s',
    '记录启动失败：%s': 'Record failed to start: %s',
    '开始记录 -> %s': 'Recording -> %s',
    '记录完成：%s': 'Recording saved: %s',
    '读不到地图目录：%s': 'Cannot read the map folder: %s',
    '覆盖 %.0f%%': 'cov %.0f%%',
    '当前': 'current',
    '（没有元数据）': '(no metadata)',
    '已经是这张地图': 'Already using this map',
    '已经是这张地图（图已重新加载）': 'Already using this map (picture reloaded)',
    '地图不存在：%s': 'No such map: %s',
    '先切回阶段 1 再换图': 'Switch back to stage 1 to change maps',
    '切换地图': 'Change map',
    '切到 %s 要重启导航栈，约 40 秒。继续？':
        'Switching to %s restarts the nav stack (~40 s). Continue?',
    '写 maps/current 失败：%s': 'Cannot write maps/current: %s',
    '切换地图 -> %s，重启导航栈（约 40 s）…':
        'Map -> %s, restarting the nav stack (~40 s)...',
    '重启失败：%s': 'Restart failed: %s',
    '地图已切换：%s': 'Map changed: %s',
    '看箭头对不对，不对就勾「设初始位姿」点两下':
        'Check the arrow; if it is wrong use Set pose and click twice',
    '电池 %.1f V：底盘只能输出约 20%% 的速度，按下去像没反应，先充电':
        'Battery %.1f V: the chassis only delivers ~20%% of the commanded '
        'speed, so buttons look dead. Charge first.',
    '先点目标位置': 'Click the goal position',
    '再点一下目标朝向': 'Now click the goal heading',
    '激光 --   j7 -- j8 --': 'Laser --   j7 -- j8 --',
    '取距离': 'Use range',
    '速度': 'Speed',
    '停': 'Stop',
    '读当前': 'Read',
    '发送': 'Send',
    '收臂（画完必做）':
        'Back off 1 cm (required)',
    '竖线参数': 'Line parameters',
    '距离 d (cm, j7轴→墙)': 'Distance d (cm, j7 axis → wall)',
    'z 起 (cm, +上)': 'z start (cm, + up)',
    'z 止 (cm, +上)': 'z end (cm, + up)',
    '步长 (cm)': 'Step (cm)',
    '校验': 'Check',
    '摆到起点': 'Go to start',
    '开始画线': 'Start',
    '停止': 'Stop',
    '离线模式：没有连 ROS': 'Offline: no ROS',
    '连不上 ROS master %s，离线模式':
        'Cannot reach ROS master %s, offline mode',
    'ROS 模块导入失败：%s': 'ROS import failed: %s',
    '机械臂模块不可用：%s':
        'Arm module unavailable: %s',
    '阶段 -> %s': 'Stage -> %s',
    '地图加载失败\n%s': 'Map failed to load\n%s',
    '地图加载失败：%s': 'Map failed to load: %s',
    '只有阶段 1 能设目标':
        'Goals can only be set in stage 1',
    '拒绝 (%+.2f, %+.2f)：%s': 'Refused (%+.2f, %+.2f): %s',
    '目标 (%+.2f, %+.2f, %.0f°)':
        'Goal (%+.2f, %+.2f, %.0f deg)',
    '设初始位姿：先点机器人现在的位置':
        'Set pose: click where the robot is now',
    '再点一下机器人朝向的方向':
        'Now click the direction it faces',
    '初始位姿 (%+.2f, %+.2f, %.0f°)':
        'Initial pose (%+.2f, %+.2f, %.0f deg)',
    '机械臂不可用': 'Arm not available',
    '读不到 /CurrentAngle': 'Cannot read /CurrentAngle',
    '读当前：缺 j%s（总线偶尔':
        'Pose read: missing j%s (the bus sometimes',
    '只回两个关节，再按一次）':
        'answers two joints; try again)',
    '读当前：': 'Pose read: ',
    'j%d 不是数字': 'j%d is not a number',
    'j%d=%.1f 超出 %.0f~%.0f': 'j%d=%.1f is outside %.0f..%.0f',
    '发送 j7=%.1f j8=%.1f j9=%.1f  (%.0f°, %d ms)':
        'Sent j7=%.1f j8=%.1f j9=%.1f  (%.0f deg, %d ms)',
    '法向回退 1 cm': 'Back off 1 cm',
    '机械臂正忙': 'Arm is busy',
    '完成': 'done',
    '未完成': 'not reached',
    '%s 失败：%s': '%s failed: %s',
    '读不到激光正前方距离':
        'No laser range straight ahead',
    '激光 %.3f m -> 距离 %.1f cm（已加 15.9 cm 偏移）':
        'Laser %.3f m -> distance %.1f cm (+15.9 cm offset)',
    '离线模式，无法启动相机':
        'Offline: cannot start the camera',
    '启动相机（约 10 s）…':
        'Starting the camera (~10 s)...',
    '相机已启动，等几秒出图':
        'Camera started; the image takes a few seconds',
    '相机启动失败：%s': 'Camera start failed: %s',
    '停止相机…': 'Stopping the camera...',
    '相机已停止': 'Camera stopped',
    '相机停止失败：%s': 'Camera stop failed: %s',
    '离线模式，无法重新定位':
        'Offline: cannot re-localise',
    '重新定位中，约 30-60 s…':
        'Re-localising, 30-60 s...',
    '重新定位失败：%s': 'Re-localise failed: %s',
    '重新定位到 (%+.2f, %+.2f, %.0f°) —— 看地图':
        'Re-localised to (%+.2f, %+.2f, %.0f deg) -- check the',
    '箭头对不对，不对就用「设初始位姿」点两下':
        'arrow; if it is wrong use Set pose and click twice',
    '参数不是数字': 'A parameter is not a number',
    'd=%.1f 完全够不着': 'd=%.1f is out of reach',
    'd=%.1f 可画 z %+.1f..%+.1f (%s)': 'd=%.1f drawable z %+.1f..%+.1f (%s)',
    '  ← 超范围': '  <- out of range',
    '起点 (%.1f, %.1f) 不可达':
        'Start (%.1f, %.1f) is unreachable',
    '摆到起点 j7=%.1f j8=%.1f': 'Moving to start j7=%.1f j8=%.1f',
    '已经在画了': 'Already drawing',
    'move_base 还在执行，先取消目标':
        'move_base is still running; cancel the goal first',
    'z 起必须大于 z 止（从上往下画）':
        'z start must be above z end (top-down)',
    'z 止 %.1f 低于安全线 %.1f':
        'z end %.1f is below the safe limit %.1f',
    '读不到机械臂角度，先确认 /CurrentAngle':
        'No arm pose; check /CurrentAngle',
    '笔尖在 (%.1f, %.1f)，不在起点 (%.1f, %.1f)，先按“摆到起点”':
        'Tip at (%.1f, %.1f), not at the start (%.1f, %.1f); press Go to start',
    '画线%s': 'Drawing %s',
    '中断': 'interrupted',
    '画线失败：%s': 'Drawing failed: %s',
    '画线 %d/%d': 'Drawing %d/%d',
    '画线 %d%%': 'Drawing %d%%',
    '已请求停止': 'Stop requested',
    '急停': 'EMERGENCY STOP',
    '相机无数据\n': 'No camera data\n',
    '没装 opencv': 'opencv not installed',
    '臂未连接': 'arm not connected',
    '尖 %.0f/%.0f cm': 'tip %.0f/%.0f cm',
    '激光 %s  %s': 'Laser %s  %s',
    '%s  等待 /amcl_pose  --  导航栈起了吗？':
        '%s  waiting for /amcl_pose -- is the nav stack up?',
    '%s  x=%+.2f y=%+.2f yaw=%+.1f  |  σ %.2f/%.2f m %.1f°  |  ':
        '%s  x=%+.2f y=%+.2f yaw=%+.0f | σ %.2f/%.2f %.1f° | ',
    'move_base: %s  |  激光 %s': 'nav: %s | laser %s',
    '【%s】': '[%s]',
}


def T(text):
    """Translate one literal.  Unknown text passes through unchanged."""
    if LANG == 'en':
        return _EN.get(text, text)
    return text



# ---------------------------------------------------------------------------
# map
# ---------------------------------------------------------------------------
class Shared(object):
    """Everything the ROS callbacks and the Tk thread both touch."""

    def __init__(self):
        self.lock = threading.Lock()
        self.amcl = None        # (x, y, yaw, sx, sy, syaw_deg)
        self.scan = None        # (N,2) in map frame
        self.plan = None
        self.status = 'idle'
        self.front = None       # m, lidar -> wall straight ahead
        self.img = None         # CompressedImage
        self.img_t = 0.0
        self.volts = None       # battery, V
        self.volts_warned = False


STATUS_NAMES = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded',
                4: 'aborted', 5: 'rejected', 6: 'preempting', 7: 'recalling',
                8: 'recalled', 9: 'lost'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ---------------------------------------------------------------------------
# the window
# ---------------------------------------------------------------------------
class App(object):
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.shared = Shared()
        self.ui_q = queue.Queue()
        self.stage = 'nav'
        self.jog = {'fwd': False, 'back': False, 'left': False,
                    'right': False, 'ccw': False, 'cw': False}
        self._pose_first = None   # first click of the two-click pose setter
        self._jv_filled = False   # joint boxes seeded from the measured pose
        self.tip_txt = ''
        self.jog_t = {}       # when each direction was pressed
        self.jog_until = {}   # a click shorter than one tick still nudges
        self.ros = None
        self.arm = None
        self.draw_abort = threading.Event()
        self.draw_busy = False
        self.last_cmd_t = 0.0
        self.tick_n = 0
        self.map = None
        self.offline = bool(args.offline)
        self.goal_px = None
        self.goal_yaw = None      # None = plain circle marker
        self._goal_first = None   # first click of the two-click goal setter
        self.trace_proc = None    # nav_trace.py while a recording is running
        self._cam_t = 0.0
        self._closing = False
        # Mirrored into a plain bool: the ROS callback thread must never touch
        # a Tk variable, even to read one.
        self.laser_on = False

        root.title(T('Transbot 任务台 -- 导航 / 对准 / 画线'))
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        root.bind('<FocusOut>', lambda e: self.jog_clear())
        root.bind('<Escape>', lambda e: self.estop())

        self._build_top()
        self._build_body()
        self._build_status()

        self._load_map(args.map)
        self.reload_maps()
        self._setup_ros()

        self._translate_tree(self.root)
        self.set_stage(args.stage, force=True)
        self.tick()

    # -- layout ------------------------------------------------------------
    def _translate_tree(self, w):
        """Translate every static `text` option once, after building.

        Runtime-updated strings (status line, readout, log) are wrapped in
        T() where they are formatted instead; this pass only touches what is
        set once and never changed.
        """
        for child in w.winfo_children():
            try:
                if 'text' in child.keys():
                    cur = child.cget('text')
                    if cur:
                        child.config(text=T(cur))
            except Exception:
                pass
            self._translate_tree(child)

    def _build_top(self):
        bar = tk.Frame(self.root, bg='#f1efe8')
        bar.pack(side=tk.TOP, fill=tk.X)
        self.stage_btn = {}
        for name in STAGE_ORDER:
            b = tk.Button(bar, text=STAGE_TITLE[name], width=12, height=1,
                          font=('TkDefaultFont', 11),
                          command=lambda n=name: self.set_stage(n))
            b.pack(side=tk.LEFT, padx=(10 if name == 'nav' else 4, 0), pady=8)
            self.stage_btn[name] = b
        tk.Button(bar, text='急停  ESC', bg='#f7c1c1', activebackground='#f09595',
                  font=('TkDefaultFont', 11, 'bold'), width=12,
                  command=self.estop).pack(side=tk.RIGHT, padx=10, pady=8)
        self.banner = tk.Label(bar, text='', fg='#a32d2d', bg='#f1efe8',
                               font=('TkDefaultFont', 10))
        self.banner.pack(side=tk.RIGHT, padx=10)

    def _build_body(self):
        # Grid with an explicit weight on the map column: pack's expand=True
        # let the column grow to the sum of every child's requested width,
        # which pushed the right column off the window.
        body = tk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=6)
        # Both columns share the extra width.  With weight 0 on the right,
        # maximising the window only stretched the map and the panel stayed
        # 580 px wide -- which is exactly what "放大看看但是没用" was.
        body.columnconfigure(0, weight=1, minsize=MAP_W)
        body.columnconfigure(1, weight=1, minsize=RIGHT_W)
        body.rowconfigure(0, weight=1)

        left = tk.Frame(body)
        left.grid(row=0, column=0, sticky='nsew')
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)      # the map takes the spare height
        left.rowconfigure(1, weight=0)      # the log keeps its 8 rows

        self.canvas = tk.Canvas(left, width=MAP_W, height=MAP_H, bg='#faf9f6',
                                highlightthickness=1,
                                highlightbackground='#d3d1c7')
        self.canvas.grid(row=0, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', self._on_canvas_resize)
        self.canvas.bind('<Button-1>', self.on_map_click)
        self.canvas.bind('<Button-3>', lambda e: self.cancel_goal())
        self.canvas.bind('<Motion>', self.on_map_motion)

        # The map is 14.8 x 7.0 m (2.13:1), so a full-height canvas would be
        # mostly empty.  Put the event history there instead of wasting it.
        logf = tk.Frame(left)
        logf.grid(row=1, column=0, sticky='nsew', pady=(6, 0))
        logf.columnconfigure(0, weight=1)
        logf.rowconfigure(0, weight=0)
        self.logtext = tk.Text(logf, height=6, width=1, wrap=tk.WORD,
                               state=tk.DISABLED, font=('TkFixedFont', 10),
                               bg='#ffffff', relief=tk.FLAT,
                               highlightthickness=1,
                               highlightbackground='#d3d1c7')
        sb = tk.Scrollbar(logf, command=self.logtext.yview)
        self.logtext.config(yscrollcommand=sb.set)
        self.logtext.grid(row=0, column=0, sticky='nsew')
        sb.grid(row=0, column=1, sticky='ns')

        # No fixed width and no grid_propagate(False): with either of those
        # the frame stayed 580 px inside a cell that had already grown, which
        # is why maximising never made the panel bigger.  The column's
        # minsize keeps it from collapsing; sticky lets it fill.
        right = tk.Frame(body)
        right.grid(row=0, column=1, sticky='nsew', padx=(8, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=2)

        self.cam = tk.Canvas(right, width=CAM_W, height=CAM_H, bg='#f1efe8',
                             highlightthickness=1,
                             highlightbackground='#d3d1c7')
        self.cam.grid(row=0, column=0, sticky='nsew')
        self.cam_text = self.cam.create_text(
            CAM_W // 2, CAM_H // 2, text='相机未连接\n/camera/rgb/image_raw/compressed',
            fill='#888780', justify=tk.CENTER, font=('TkDefaultFont', 11))

        self.panels = {}
        holder = tk.Frame(right)
        holder.grid(row=1, column=0, sticky='nsew', pady=(8, 0))
        holder.columnconfigure(0, weight=1)
        holder.rowconfigure(0, weight=1)
        for name in STAGE_ORDER:
            f = tk.Frame(holder)
            self.panels[name] = f

        self._build_nav_panel(self.panels['nav'])
        self._build_align_panel(self.panels['align'])
        self._build_draw_panel(self.panels['draw'])

    def _build_status(self):
        bar = tk.Frame(self.root, bg='#f1efe8')
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        self.status = tk.Label(bar, text='', anchor=tk.W, bg='#f1efe8',
                               font=('TkFixedFont', 10))
        self.status.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(4, 0))
        self.logline = tk.Label(bar, text='', anchor=tk.W, bg='#f1efe8',
                                fg='#5f5e5a', font=('TkFixedFont', 10))
        self.logline.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 4))

    # -- panels ------------------------------------------------------------
    def _build_nav_panel(self, f):
        # Packed before the content: a side=BOTTOM widget packed last only
        # gets the leftover space, which is zero once the labels grow.
        tk.Button(f, text='下一步：对准  →', height=1,
                  command=lambda: self.set_stage('align')).pack(
            side=tk.BOTTOM, fill=tk.X, pady=6)

        # Map picker.  Switching means rewriting maps/current and restarting
        # the stack: map_server reads the yaml once, at startup.
        rowm = tk.Frame(f)
        rowm.pack(fill=tk.X, pady=(0, 2))
        tk.Label(rowm, text='地图').pack(side=tk.LEFT)
        self.map_var = tk.StringVar()
        self.map_box = ttk.Combobox(rowm, textvariable=self.map_var,
                                    state='readonly', width=14)
        self.map_box.pack(side=tk.LEFT, padx=6)
        self.map_box.bind('<<ComboboxSelected>>', lambda e: self._map_detail())
        tk.Button(rowm, text='刷新', width=4,
                  command=self.reload_maps).pack(side=tk.LEFT)
        self.switch_btn = tk.Button(rowm, text='切换', width=4,
                                    command=self.switch_map)
        self.switch_btn.pack(side=tk.LEFT, padx=(6, 0))
        self.map_info = tk.Label(f, text='', fg='#888780', anchor=tk.W)
        self.map_info.pack(fill=tk.X)

        tk.Label(f, text='点地图设目标，右键取消。',
                 fg='#5f5e5a').pack(anchor=tk.W)

        # DWA always ends at a yaw, it just used to be whatever direction the
        # last leg happened to have.  For the wall task the final heading is
        # the whole point, so let it be picked explicitly.
        self.goal_mode = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text='指定目标朝向（点两下）',
                       variable=self.goal_mode,
                       command=self._goal_mode_changed).pack(anchor=tk.W,
                                                             pady=(6, 0))

        row = tk.Frame(f)
        row.pack(anchor=tk.W, pady=(6, 0))
        tk.Button(row, text='取消目标', width=10,
                  command=self.cancel_goal).pack(side=tk.LEFT)
        self.laser_var = tk.BooleanVar(value=False)
        tk.Checkbutton(row, text='激光点', variable=self.laser_var,
                       command=self._toggle_laser).pack(side=tk.LEFT, padx=8)

        row3 = tk.Frame(f)
        row3.pack(anchor=tk.W, pady=(6, 0))
        tk.Label(row3, text='清空距离 (m)', fg='#5f5e5a').pack(side=tk.LEFT)
        self.nav_clear = tk.DoubleVar(value=self.args.clear)
        tk.Spinbox(row3, from_=0.15, to=1.20, increment=0.05, width=6,
                   textvariable=self.nav_clear).pack(side=tk.LEFT, padx=6)
        self.trace_btn = tk.Button(row3, text='记录', width=6,
                                   command=self.trace_toggle)
        self.trace_btn.pack(side=tk.LEFT, padx=(6, 0))

        row4 = tk.Frame(f)
        row4.pack(anchor=tk.W, pady=(8, 0))
        tk.Button(row4, text='重新定位', command=self.relocalise).pack(
            side=tk.LEFT)
        tk.Button(row4, text='相机开', command=self.cam_start).pack(
            side=tk.LEFT, padx=4)
        tk.Button(row4, text='相机关', command=self.cam_stop).pack(side=tk.LEFT)

        self.pose_mode = tk.BooleanVar(value=False)
        tk.Checkbutton(f, text='设初始位姿（位置→朝向）',
                       variable=self.pose_mode,
                       command=self._pose_mode_changed).pack(anchor=tk.W,
                                                             pady=(8, 0))

    def _build_align_panel(self, f):
        tk.Button(f, text='下一步：画线  →', height=1,
                  command=lambda: self.set_stage('draw')).pack(
            side=tk.BOTTOM, fill=tk.X, pady=6)

        # grid, not pack: the readout is the only elastic item, so it gives
        # way instead of pushing the button and the speed knob off the panel.
        r0 = tk.Frame(f)
        r0.pack(fill=tk.X, pady=(0, 4))
        r0.columnconfigure(0, weight=1)
        self.readout = tk.Label(r0, text='激光 --   j7 -- j8 --',
                                anchor=tk.W, font=('TkFixedFont', 10))
        self.readout.grid(row=0, column=0, sticky='ew')
        tk.Button(r0, text='取距离', command=self.lidar_to_dist).grid(
            row=0, column=1, padx=(6, 0))
        # One speed knob instead of two: the turn rate is derived from it
        # (x3.5).
        tk.Label(r0, text='速度').grid(row=0, column=2, padx=(10, 2))
        self.spd = tk.DoubleVar(value=0.10)
        tk.Spinbox(r0, from_=0.04, to=0.30, increment=0.02, width=5,
                   textvariable=self.spd).grid(row=0, column=3)

        pad = tk.Frame(f)
        pad.pack(anchor=tk.W)
        spec = [('↑', 'fwd', 0, 1), ('←', 'ccw', 1, 0), ('停', None, 1, 1),
                ('→', 'cw', 1, 2), ('↓', 'back', 2, 1)]
        for text, key, r, c in spec:
            btn = tk.Button(pad, text=text, width=3, height=1)
            if key is None:
                btn.config(command=self.jog_clear)
            else:
                btn.bind('<ButtonPress-1>', lambda e, k=key: self.jog_set(k, True))
                btn.bind('<ButtonRelease-1>',
                         lambda e, k=key: self.jog_set(k, False))
                btn.bind('<Leave>', lambda e, k=key: self.jog_set(k, False))
            btn.grid(row=r, column=c, padx=2, pady=1)

        # Manual joint control instead of a preset "ready" pose: j9 is the
        # gripper, and without it the pen cannot be clamped or released.
        r2 = tk.Frame(f)
        r2.pack(anchor=tk.W, pady=(4, 0))
        self.jv = {}
        for col, (jid, default) in enumerate(((7, '168.0'), (8, '119.0'),
                                              (9, '159.0'))):
            tk.Label(r2, text='j%d' % jid).grid(
                row=0, column=col * 2, padx=(0 if col == 0 else 6, 2))
            v = tk.StringVar(value=default)
            tk.Entry(r2, textvariable=v, width=5).grid(
                row=0, column=col * 2 + 1)
            self.jv[jid] = v
        tk.Button(r2, text='读当前', command=self.arm_read).grid(
            row=0, column=6, padx=(10, 2))
        tk.Button(r2, text='发送', command=self.arm_send).grid(row=0, column=7)

    def _build_draw_panel(self, f):
        tk.Button(f, text='收臂（画完必做）', command=self.arm_retract).pack(
            side=tk.BOTTOM, fill=tk.X, pady=(4, 6))
        # Reserved from the bottom too: packed last it got zero height.
        self.prog = ttk.Progressbar(f, mode='determinate', maximum=100)
        self.prog.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        box = tk.LabelFrame(f, text='竖线参数')
        box.pack(fill=tk.X)
        rows = [('距离 d (cm, j7轴→墙)', 'dist', '26.2'),
                ('z 起 (cm, +上)', 'z_top', '16.0'),
                ('z 止 (cm, +上)', 'z_bot', '-5.5'),
                ('步长 (cm)', 'step', '1.0')]
        self.line_var = {}
        for i, (label, key, default) in enumerate(rows):
            tk.Label(box, text=label, anchor=tk.W).grid(
                row=i, column=0, sticky=tk.W, padx=6, pady=1)
            v = tk.StringVar(value=default)
            tk.Entry(box, textvariable=v, width=8).grid(
                row=i, column=1, padx=6, pady=1)
            self.line_var[key] = v

        r = tk.Frame(f)
        r.pack(fill=tk.X, pady=(6, 0))
        tk.Button(r, text='校验', command=self.check_line).pack(side=tk.LEFT)
        tk.Button(r, text='摆到起点', command=self.goto_start).pack(
            side=tk.LEFT, padx=4)
        tk.Button(r, text='开始画线', command=self.start_draw).pack(
            side=tk.LEFT, padx=(10, 0), expand=True, fill=tk.X)
        tk.Button(r, text='停止', command=self.stop_draw).pack(
            side=tk.LEFT, padx=4)

    # -- ros ---------------------------------------------------------------
    def _master_ok(self):
        host = self.args.master_host
        try:
            s = socket.create_connection((host, 11311), timeout=1.5)
            s.close()
            return True
        except Exception:
            return False

    # -- maps --------------------------------------------------------------
    @staticmethod
    def _current_map_name():
        try:
            with open(os.path.join(MAPS_DIR, 'current')) as f:
                return f.readline().strip()
        except Exception:
            return ''

    def reload_maps(self):
        self.maps = {}
        try:
            for fn in sorted(os.listdir(MAPS_DIR)):
                if not fn.endswith('.yaml'):
                    continue
                name = fn[:-5]
                meta = {}
                try:
                    with open(os.path.join(MAPS_DIR, name + '.json')) as f:
                        meta = json.load(f)
                except Exception:
                    pass
                self.maps[name] = meta
        except Exception as exc:
            self._log(T('读不到地图目录：%s') % exc)
        names = sorted(self.maps)
        self.map_box['values'] = names
        cur = self._current_map_name()
        if cur in self.maps:
            self.map_var.set(cur)
        elif names:
            self.map_var.set(names[0])
        self._map_detail()

    def _map_detail(self):
        name = self.map_var.get()
        m = getattr(self, 'maps', {}).get(name, {})
        bits = []
        if 'area_m2' in m:
            bits.append('%.1f m²' % m['area_m2'])
        if 'coverage_pct' in m:
            bits.append(T('覆盖 %.0f%%') % m['coverage_pct'])
        if 'size_m' in m:
            bits.append('%.1f×%.1f m' % tuple(m['size_m']))
        if 'saved' in m:
            bits.append(m['saved'][5:16])
        if name and name == self._current_map_name():
            bits.append(T('当前'))
        self.map_info.config(text=' · '.join(bits) or T('（没有元数据）'))

    def switch_map(self):
        name = self.map_var.get()
        if not name:
            return
        if self.stage != 'nav':
            self._log(T('先切回阶段 1 再换图'))
            return
        path = os.path.join(MAPS_DIR, name + '.yaml')
        if not os.path.exists(path):
            self._log(T('地图不存在：%s') % path)
            return
        same = (name == self._current_map_name())
        if not same:
            if not messagebox.askyesno(
                    T('切换地图'),
                    T('切到 %s 要重启导航栈，约 40 秒。继续？') % name):
                return
            try:
                os.makedirs(MAPS_DIR, exist_ok=True)
                with open(os.path.join(MAPS_DIR, 'current'), 'w') as f:
                    f.write(name + '\n')
            except Exception as exc:
                self._log(T('写 maps/current 失败：%s') % exc)
                return
        # Repaint first, restart later.  The yaml is just a file; waiting for
        # the 60 s stack restart before showing the new picture is what made
        # "switching" look like it did nothing.
        self._load_map(path)
        self.map_box.set(name)
        self._map_detail()
        if same:
            self._log(T('已经是这张地图（图已重新加载）'))
            return
        self.cancel_goal()
        self.switch_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._restart_stack, args=(name,),
                         daemon=True).start()

    def _restart_stack(self, name):
        self._ui(self._log, T('切换地图 -> %s，重启导航栈（约 40 s）…') % name)
        script = os.path.abspath(os.path.join(HERE, '..', 'nav', 'scripts', 'start_nav.sh'))
        try:
            p = subprocess.Popen([script, '--no-rviz'],
                                 stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT,
                                 env=dict(os.environ))
            for raw in iter(p.stdout.readline, b''):
                line = raw.decode('utf-8', 'replace').rstrip()
                if line:
                    self._ui(self._log, line)
            p.wait()
        except Exception as exc:
            self._ui(self._log, T('重启失败：%s') % exc)
            self._ui(lambda: self.switch_btn.config(state=tk.NORMAL))
            return
        if p.returncode != 0:
            self._ui(self._log, 'Navigation restart failed (exit %s); check the log.' % p.returncode)
            self._ui(lambda: self.switch_btn.config(state=tk.NORMAL))
            return
        self._ui(self._reload_map_after_switch, name)

    def _reload_map_after_switch(self, name):
        self._load_map(os.path.join(MAPS_DIR, name + '.yaml'))
        self.switch_btn.config(state=tk.NORMAL)
        self._log(T('地图已切换：%s') % name)
        self._log(T('看箭头对不对，不对就勾「设初始位姿」点两下'))

    def _setup_ros(self):
        if self.offline:
            self.banner.config(text=T('离线模式：没有连 ROS'))
            return
        if not self._master_ok():
            self.offline = True
            self.banner.config(text=T('连不上 ROS master %s，离线模式')
                                    % self.args.master_host)
            return
        try:
            import rospy
            from actionlib_msgs.msg import GoalID, GoalStatusArray
            from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                                           Twist)
            from nav_msgs.msg import Path
            from sensor_msgs.msg import Image, LaserScan
            from transbot_msgs.msg import Battery
        except Exception as exc:
            self.offline = True
            self.banner.config(text=T('ROS 模块导入失败：%s') % exc)
            return

        rospy.init_node('task_gui', anonymous=True, disable_signals=True)
        self.ros = rospy
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                                        queue_size=1)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID,
                                          queue_size=1)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.cb_amcl, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.cb_scan, queue_size=3)
        rospy.Subscriber('/voltage', Battery, self.cb_voltage, queue_size=5)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path,
                         self.cb_plan, queue_size=2)
        rospy.Subscriber('/move_base/status', GoalStatusArray,
                         self.cb_status, queue_size=5)
        self.pub_init = rospy.Publisher('/initialpose',
                                        PoseWithCovarianceStamped, queue_size=1)
        rospy.Subscriber(CAM_TOPIC, Image, self.cb_img, queue_size=1)
        threading.Thread(target=rospy.spin, daemon=True).start()

        try:
            from arm_client import ArmClient
            self.arm = ArmClient()
            self.arm.start()
        except Exception as exc:
            self._log(T('机械臂模块不可用：%s') % exc)

    # -- callbacks (ROS threads) -------------------------------------------
    def cb_amcl(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        c = m.pose.covariance
        with self.shared.lock:
            self.shared.amcl = (p.x, p.y, yaw_of(q),
                                math.sqrt(max(c[0], 0.0)),
                                math.sqrt(max(c[7], 0.0)),
                                math.degrees(math.sqrt(max(c[35], 0.0))))

    def cb_voltage(self, m):
        # At 10.5 V this chassis delivers about 15-20 % of the commanded
        # speed, which looks exactly like "the buttons do nothing".  Show the
        # number instead of letting it be a mystery.
        with self.shared.lock:
            self.shared.volts = float(m.Voltage)

    def cb_scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        half = math.radians(FRONT_HALF_ANGLE)
        diff = np.abs(((a - math.pi + math.pi) % (2 * math.pi)) - math.pi)
        ok = (diff < half) & np.isfinite(r) & (r > 0.12)
        front = float(np.median(r[ok])) if ok.any() else None
        with self.shared.lock:
            self.shared.front = front
            if not self.laser_on:
                return
            amcl = self.shared.amcl
        if amcl is None:
            return
        good = np.isfinite(r) & (r > 0.2) & (r < 8.0)
        xl = -r[good] * np.cos(a[good]) - 0.03
        yl = -r[good] * np.sin(a[good])
        ca, sa = math.cos(amcl[2]), math.sin(amcl[2])
        with self.shared.lock:
            self.shared.scan = np.column_stack(
                (xl * ca - yl * sa + amcl[0], xl * sa + yl * ca + amcl[1]))

    def cb_plan(self, m):
        with self.shared.lock:
            self.shared.plan = [(p.pose.position.x, p.pose.position.y)
                                for p in m.poses]

    def cb_status(self, m):
        if not m.status_list:
            return
        s = m.status_list[-1].status
        with self.shared.lock:
            self.shared.status = STATUS_NAMES.get(s, str(s))

    def cb_img(self, m):
        with self.shared.lock:
            self.shared.img = m
            self.shared.img_t = time.time()

    # -- thread-safe UI helpers --------------------------------------------
    def _ui(self, fn, *a):
        self.ui_q.put((fn, a))

    def _drain_ui(self):
        for _ in range(20):
            try:
                fn, a = self.ui_q.get_nowait()
            except queue.Empty:
                return
            try:
                fn(*a)
            except Exception:
                pass

    def _log(self, text):
        self.logline.config(text=text)
        try:
            self.logtext.config(state=tk.NORMAL)
            self.logtext.insert(tk.END, text + '\n')
            self.logtext.see(tk.END)
            self.logtext.config(state=tk.DISABLED)
        except Exception:
            pass

    def _toggle_laser(self):
        self.laser_on = bool(self.laser_var.get())
        if not self.laser_on:
            with self.shared.lock:
                self.shared.scan = None

    # -- stage machine -----------------------------------------------------
    def set_stage(self, name, force=False):
        if name == self.stage and not force:
            return
        if self.stage == 'align' and name != 'align':
            self.jog_clear()
        if name in ('align', 'draw'):
            self.cancel_goal()
        if name == 'draw' and self.stage != 'draw':
            self.jog_clear()
        self.stage = name
        for n, b in self.stage_btn.items():
            b.config(relief=tk.SUNKEN if n == name else tk.RAISED,
                     bg='#dbeafe' if n == name else '#f0f0f0')
        for n, f in self.panels.items():
            if n == name:
                f.grid(row=0, column=0, sticky='nsew')
            else:
                f.grid_remove()
        self._log(T('阶段 -> %s') % T(STAGE_TITLE[name]))

    # -- map ---------------------------------------------------------------
    def _load_map(self, path):
        try:
            self.canvas.delete('bg')
            self.map = MapData(os.path.expanduser(path))
            self.map.fit(max(self.canvas.winfo_width(), MAP_W),
                         max(self.canvas.winfo_height(), MAP_H))
            self.canvas.create_image(self.map.off_x, self.map.off_y,
                                     anchor=tk.NW, image=self.map.photo,
                                     tags='bg')
        except Exception as exc:
            self.map = None
            self.canvas.create_text(MAP_W // 2, MAP_H // 2,
                                    text='地图加载失败\n%s' % exc,
                                    fill='#a32d2d', justify=tk.CENTER)
            self._log(T('地图加载失败：%s') % exc)

    def _on_canvas_resize(self, ev):
        """Re-fit the map when the window is resized or maximised."""
        if self.map is None or ev.width < 50 or ev.height < 50:
            return
        if (abs(ev.width - self.map.canvas_w) < 20
                and abs(ev.height - self.map.canvas_h) < 20):
            return
        self.map.fit(ev.width, ev.height)
        self.canvas.delete('bg')
        self.canvas.create_image(self.map.off_x, self.map.off_y,
                                 anchor=tk.NW, image=self.map.photo, tags='bg')
        self.canvas.tag_lower('bg')

    def on_map_motion(self, ev):
        if self.map is None:
            return
        x, y = self.map.to_map(ev.x, ev.y)
        ok, _ = self.map.clear(x, y, self.nav_clear.get())
        self.canvas.config(cursor='tcross' if ok else 'X_cursor')

    def on_map_click(self, ev):
        if self.stage != 'nav':
            self._log(T('只有阶段 1 能设目标'))
            return
        if self.offline or self.map is None:
            return
        x, y = self.map.to_map(ev.x, ev.y)
        if self.pose_mode.get():
            self._pose_click(x, y)
            return
        if self.goal_mode.get():
            self._goal_click(x, y)
            return
        ok, msg = self.map.clear(x, y, self.nav_clear.get())
        if not ok:
            self._log(T('拒绝 (%+.2f, %+.2f)：%s') % (x, y, msg))
            return
        with self.shared.lock:
            amcl = self.shared.amcl
        # Default heading: face away from where the robot is, i.e. the last
        # leg's direction.  Shift-click keeps 0 (map +x) for a fixed heading.
        yaw = 0.0
        if amcl is not None and not (ev.state & 0x0001):
            yaw = math.atan2(y - amcl[1], x - amcl[0])
        self._send_goal(x, y, yaw)

    def _goal_mode_changed(self):
        self._goal_first = None
        if self.goal_mode.get():
            self._log(T('先点目标位置'))

    def _goal_click(self, x, y):
        """Two clicks: goal position, then a point the robot should face."""
        if self._goal_first is None:
            ok, msg = self.map.clear(x, y, self.nav_clear.get())
            if not ok:
                self._log(T('拒绝 (%+.2f, %+.2f)：%s') % (x, y, msg))
                return
            self._goal_first = (x, y)
            self._log(T('再点一下目标朝向'))
            return
        x0, y0 = self._goal_first
        yaw = math.atan2(y - y0, x - x0)
        self._goal_first = None
        self.goal_mode.set(False)
        self._send_goal(x0, y0, yaw)

    def _send_goal(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.ros.Time.now()
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(g)
        self.goal_px = self.map.to_px(x, y)
        self.goal_yaw = yaw
        self._log(T('目标 (%+.2f, %+.2f, %.0f°)') % (x, y, math.degrees(yaw)))

    def _pose_mode_changed(self):
        self._pose_first = None
        self.canvas.delete('posemark')
        if self.pose_mode.get():
            self._log(T('设初始位姿：先点机器人现在的位置'))

    def _pose_click(self, x, y):
        """Two clicks: robot position, then a point it is facing."""
        if self._pose_first is None:
            self._pose_first = (x, y)
            px, py = self.map.to_px(x, y)
            self.canvas.delete('posemark')
            self.canvas.create_oval(px - 7, py - 7, px + 7, py + 7,
                                    outline='#d4537e', width=2,
                                    tags=('dyn', 'posemark'))
            self._log(T('再点一下机器人朝向的方向'))
            return
        x0, y0 = self._pose_first
        yaw = math.atan2(y - y0, x - x0)
        self._publish_initial(x0, y0, yaw)
        self._pose_first = None
        self.pose_mode.set(False)
        self._log(T('初始位姿 (%+.2f, %+.2f, %.0f°)')
                  % (x0, y0, math.degrees(yaw)))

    def _publish_initial(self, x, y, yaw):
        if self.ros is None:
            return
        from geometry_msgs.msg import PoseWithCovarianceStamped
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.ros.Time.now()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        m.pose.covariance[0] = 0.0025
        m.pose.covariance[7] = 0.0025
        m.pose.covariance[35] = 0.0025
        self.pub_init.publish(m)

    def cancel_goal(self):
        if self.ros is None:
            return
        from actionlib_msgs.msg import GoalID
        self.pub_cancel.publish(GoalID())
        self.goal_px = None
        self.goal_yaw = None
        self._goal_first = None

    # -- jog ---------------------------------------------------------------
    def jog_set(self, key, on):
        if self.stage != 'align':
            return
        now = time.time()
        if on:
            self.jog[key] = True
            self.jog_t[key] = now
        else:
            self.jog[key] = False
            # A click is shorter than one 50 ms tick, so the pad would look
            # dead.  Keep the command alive for 0.25 s: a click nudges.
            if now - self.jog_t.get(key, 0.0) < 0.25:
                self.jog_until[key] = now + 0.25

    def jog_clear(self):
        for k in self.jog:
            self.jog[k] = False
        self.jog_until.clear()
        self._publish_cmd(0.0, 0.0, 0.0)

    def _publish_cmd(self, vx, vy, wz):
        if self.ros is None:
            return
        from geometry_msgs.msg import Twist
        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        tw.angular.z = wz
        self.pub_cmd.publish(tw)
        self.last_cmd_t = time.time()

    def _tick_jog(self):
        if self.stage != 'align':
            return
        vx = vy = wz = 0.0
        s = self.spd.get()
        t = s * 3.5          # turn rate follows the speed knob
        now = time.time()

        def on(k):
            return self.jog.get(k) or now < self.jog_until.get(k, 0.0)

        if on('fwd'):
            vx += s
        if on('back'):
            vx -= s
        if on('left'):
            vy += s * 0.7
        if on('right'):
            vy -= s * 0.7
        if on('ccw'):
            wz += t
        if on('cw'):
            wz -= t
        if vx or vy or wz:
            self._publish_cmd(vx, vy, wz)
        elif time.time() - self.last_cmd_t < 0.3:
            self._publish_cmd(0.0, 0.0, 0.0)

    def on_key(self, ev):
        if self.stage != 'align':
            return
        m = {'w': 'fwd', 's': 'back', 'a': 'ccw', 'd': 'cw',
             'q': 'left', 'e': 'right'}
        k = (ev.keysym or '').lower()
        if k in m:
            self.jog_set(m[k], True)

    def on_key_up(self, ev):
        m = {'w': 'fwd', 's': 'back', 'a': 'ccw', 'd': 'cw',
             'q': 'left', 'e': 'right'}
        k = (ev.keysym or '').lower()
        if k in m:
            self.jog_set(m[k], False)

    # -- arm ---------------------------------------------------------------
    def arm_read(self):
        """Load the measured angles into the three entry boxes."""
        if self.arm is None:
            self._log(T('机械臂不可用'))
            return
        p = self.arm.pose
        if p is None:
            self._log(T('读不到 /CurrentAngle'))
            return
        missing = [j for j in self.jv if j not in p]
        for jid, var in self.jv.items():
            if jid in p:
                var.set('%.1f' % p[jid])
        if missing:
            self._log((T('读当前：缺 j%s（总线偶尔')
                       + '只回两个关节，再按一次）')
                      % ','.join(str(j) for j in missing))
            return
        self._jv_filled = True
        self._log(T('读当前：') + '  '.join(
            'j%d=%.1f' % (j, p[j]) for j in sorted(self.jv)))

    def arm_send(self):
        """Send whatever is in the three boxes, all at once."""
        if self.arm is None:
            self._log(T('机械臂不可用'))
            return
        vals = {}
        for jid, var in self.jv.items():
            try:
                v = float(var.get())
            except ValueError:
                self._log(T('j%d 不是数字') % jid)
                return
            lo, hi = ARM_LIMITS[jid]
            if not (lo <= v <= hi):
                self._log(T('j%d=%.1f 超出 %.0f~%.0f') % (jid, v, lo, hi))
                return
            vals[jid] = v
        cur = self.arm.pose
        delta = 0.0
        if cur is not None:
            delta = max(abs(vals[j] - cur.get(j, vals[j])) for j in vals)
        # ~20 ms per degree, clamped: a 1 deg trim should not take 3 s, and a
        # 90 deg swing must not be cut off mid-travel.
        run_time = int(max(400, min(3000, delta * 20)))
        self.arm.send(vals, run_time=run_time)
        self._jv_filled = True
        self._log(T('发送 j7=%.1f j8=%.1f j9=%.1f  (%.0f°, %d ms)')
                  % (vals[7], vals[8], vals[9], delta, run_time))

    def arm_retract(self):
        if self.arm is None:
            self._log(T('机械臂不可用'))
            return
        self._run_arm(lambda: self.arm.retract(1.0), '法向回退 1 cm')

    def _run_arm(self, fn, label):
        if self.draw_busy:
            self._log(T('机械臂正忙'))
            return

        def work():
            try:
                ok = fn()
                self._ui(self._log,
                         T('%s %s') % (label, T('完成') if ok else T('未完成')))
            except Exception as exc:
                self._ui(self._log, T('%s 失败：%s') % (label, exc))

        threading.Thread(target=work, daemon=True).start()

    def lidar_to_dist(self):
        with self.shared.lock:
            front = self.shared.front
        if front is None:
            self._log(T('读不到激光正前方距离'))
            return
        d = (front + LASER_TO_J7) * 100.0
        self.line_var['dist'].set('%.1f' % d)
        self._log(T('激光 %.3f m -> 距离 %.1f cm（已加 15.9 cm 偏移）')
                  % (front, d))

    # -- trace -------------------------------------------------------------
    def trace_toggle(self):
        """Record what move_base decided, so 'why did it spin' is answerable.

        The recorder is a separate process on purpose: it keeps its own
        subscriptions and its own timeline, and killing it cannot disturb
        move_base.  Stopping it makes it write report.md next to trace.csv.
        """
        if self.offline or self.ros is None:
            self._log(T('离线模式，无法记录'))
            return
        if self.trace_proc is not None:
            p = self.trace_proc
            self.trace_proc = None
            try:
                p.send_signal(signal.SIGINT)
            except Exception:
                pass
            self.trace_btn.config(text=T('记录'))
            threading.Thread(target=self._trace_wait, args=(p,),
                             daemon=True).start()
            return

        out = os.path.expanduser('~/.local/share/transbot/wall_task/traces/'
                                 + time.strftime('%Y%m%d-%H%M%S'))
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', 'tools', 'nav_trace.py')
        script = os.path.normpath(script)
        if not os.path.exists(script):
            self._log(T('找不到 %s') % script)
            return
        try:
            self.trace_proc = subprocess.Popen(
                [sys.executable, script, '--out', out],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                env=dict(os.environ))
        except Exception as exc:
            self._log(T('记录启动失败：%s') % exc)
            return
        self.trace_btn.config(text=T('停止记录'))
        self._log(T('开始记录 -> %s') % out)

    def _trace_wait(self, p):
        try:
            out = p.communicate(timeout=15)[0].decode('utf-8', 'replace')
        except Exception:
            out = ''
        where = ''
        for line in out.splitlines():
            if line.startswith('wrote '):
                where = line[6:].strip()
        self._ui(self._log, T('记录完成：%s') % (where or '见终端输出'))

    # -- robot side --------------------------------------------------------
    def _robot_host(self):
        m = re.match(r'^\w+://([^:/]+)', os.environ.get('ROS_MASTER_URI', ''))
        return m.group(1) if m else self.args.master_host

    def _ssh(self, remote, timeout=45):
        env = dict(os.environ, SSHPASS=self.args.robot_pass)
        cmd = ['sshpass', '-e', 'ssh', '-o', 'StrictHostKeyChecking=no',
               '-o', 'ConnectTimeout=8',
               '%s@%s' % (self.args.robot_user, self._robot_host()), remote]
        return subprocess.run(cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=timeout,
                              env=env)

    def cam_start(self):
        if self.offline:
            self._log(T('离线模式，无法启动相机'))
            return
        self._log(T('启动相机（约 10 s）…'))

        def work():
            h = self._robot_host()
            kills = '; '.join('pkill -f "%s" 2>/dev/null' % p for p in ASTRA_KILL)
            src = ('source /opt/ros/melodic/setup.bash; '
                   'source $HOME/software/transbot_library/devel/setup.bash; '
                   'export ROS_MASTER_URI=http://%s:11311; export ROS_IP=%s; '
                   % (h, h))
            try:
                self._ssh(kills + '; sleep 2; echo CLEANED', timeout=30)
                r = self._ssh(
                    src + 'rm -f /tmp/astra_gui.pid; '
                    '(nohup roslaunch astra_camera astrapro.launch '
                    '>/tmp/astra_gui.log 2>&1 & echo $! > /tmp/astra_gui.pid); '
                    'echo LAUNCHED', timeout=45)
                if r.returncode != 0:
                    raise RuntimeError(
                        r.stderr.decode('utf-8', 'replace').strip()
                        or 'ssh failed')
                self._ui(self._log, T('相机已启动，等几秒出图'))
            except Exception as exc:
                self._ui(self._log, T('相机启动失败：%s') % exc)

        threading.Thread(target=work, daemon=True).start()

    def cam_stop(self):
        if self.offline:
            return
        self._log(T('停止相机…'))

        def work():
            kills = '; '.join('pkill -f "%s" 2>/dev/null' % p for p in ASTRA_KILL)
            k9 = '; '.join('pkill -9 -f "%s" 2>/dev/null' % p for p in ASTRA_KILL)
            try:
                self._ssh(kills + '; sleep 2; ' + k9 + '; echo STOPPED',
                          timeout=40)
                self._ui(self._log, T('相机已停止'))
            except Exception as exc:
                self._ui(self._log, T('相机停止失败：%s') % exc)

        threading.Thread(target=work, daemon=True).start()

    def relocalise(self):
        """Robot got picked up: match one scan against the map, then seed AMCL.

        AMCL has no way to recover from a 2 m teleport -- its particles are
        all near the old pose.  coarse_localize does a likelihood-field search
        over the whole map, which is the only thing that works after a move.
        """
        if self.offline or self.ros is None:
            self._log(T('离线模式，无法重新定位'))
            return
        self._log(T('重新定位中，约 30-60 s…'))

        def work():
            try:
                out = subprocess.check_output(
                    [sys.executable, os.path.expanduser(self.args.coarse),
                     '--map', os.path.expanduser(self.args.map),
                     '--pose-only'],
                    stderr=subprocess.DEVNULL, timeout=240).decode().strip()
                parts = out.split()
                if len(parts) != 3:
                    raise ValueError('unexpected output %r' % out)
                x, y, yaw = [float(v) for v in parts]
            except Exception as exc:
                self._ui(self._log, T('重新定位失败：%s') % exc)
                return
            self._publish_initial(x, y, yaw)
            self._ui(self._log,
                     (T('重新定位到 (%+.2f, %+.2f, %.0f°) —— 看地图')
                      + '箭头对不对，不对就用「设初始位姿」点两下')
                     % (x, y, math.degrees(yaw)))

        threading.Thread(target=work, daemon=True).start()

    # -- line --------------------------------------------------------------
    def _line_inputs(self):
        try:
            return (float(self.line_var['dist'].get()),
                    float(self.line_var['z_top'].get()),
                    float(self.line_var['z_bot'].get()),
                    max(0.2, float(self.line_var['step'].get())))
        except ValueError:
            self._log(T('参数不是数字'))
            return None

    def check_line(self):
        v = self._line_inputs()
        if v is None:
            return
        d, zt, zb, _ = v
        runs, branch = L.line_range(d, z_lo=min(zb, L.Z_FLOOR_REL) - 5,
                                    z_hi=max(zt, 0) + 5)
        if not runs:
            self._log(T('d=%.1f 完全够不着') % d)
            return
        lo, hi = runs[0][0], runs[-1][1]
        txt = 'd=%.1f 可画 z %+.1f..%+.1f (%s)' % (d, lo, hi, branch)
        if zt > hi or zb < lo:
            txt += '  ← 超范围'
        self._log(txt)

    def goto_start(self):
        v = self._line_inputs()
        if v is None or self.arm is None:
            return
        d, zt, _, _ = v
        sols = L.ik(d, zt, margin=L.REACH_MARGIN)
        if not sols:
            self._log(T('起点 (%.1f, %.1f) 不可达') % (d, zt))
            return
        q = None
        cur = self.arm.pose
        if cur is not None:
            q = min(sols, key=lambda s: abs(s[0] - cur[7]) + abs(s[1] - cur[8]))
        else:
            q = min(sols, key=lambda s: abs(s[1] - 124.0))
        self._run_arm(lambda: self.arm.goto(q[0], q[1], settle=1.2),
                      T('摆到起点 j7=%.1f j8=%.1f') % (q[0], q[1]))

    def start_draw(self):
        if self.arm is None:
            self._log(T('机械臂不可用'))
            return
        if self.draw_busy:
            self._log(T('已经在画了'))
            return
        with self.shared.lock:
            st = self.shared.status
        if st == 'active':
            self._log(T('move_base 还在执行，先取消目标'))
            return
        v = self._line_inputs()
        if v is None:
            return
        d, zt, zb, step = v
        if zt <= zb:
            self._log(T('z 起必须大于 z 止（从上往下画）'))
            return
        if zb < L.Z_FLOOR_REL - 1e-6:
            self._log(T('z 止 %.1f 低于安全线 %.1f') % (zb, L.Z_FLOOR_REL))
            return
        # Refuse to start anywhere but the top of the line: draw_points()
        # walks the line itself, so a pen that is already on the wall would be
        # dragged straight to the start point across the surface.
        p = self.arm.pose
        if p is None:
            self._log(T('读不到机械臂角度，先确认 /CurrentAngle'))
            return
        lx, lz = L.fk(p.get(7, 0.0), p.get(8, 0.0))
        if math.hypot(lx - d, lz - zt) > 1.5:
            self._log(T('笔尖在 (%.1f, %.1f)，不在起点 (%.1f, %.1f)，先按“摆到起点”')
                      % (lx, lz, d, zt))
            return
        self.draw_abort.clear()
        self.draw_busy = True
        self.prog['value'] = 0

        def work():
            try:
                ok = self.arm.draw_line(
                    d, zt, zb, step_cm=step, abort=self.draw_abort,
                    progress=lambda frac: self._ui(self._on_progress, frac))
                self._ui(self._log,
                         T('画线%s') % (T('完成') if ok else T('中断')))
            except Exception as exc:
                self._ui(self._log, T('画线失败：%s') % exc)
            finally:
                self._ui(self._draw_done)

        threading.Thread(target=work, daemon=True).start()

    def _on_progress(self, frac):
        self.prog['value'] = 100.0 * frac
        self._log(T('画线 %d%%') % int(round(100 * frac)))

    def _draw_done(self):
        self.draw_busy = False

    def stop_draw(self):
        self.draw_abort.set()
        self._log(T('已请求停止'))

    def estop(self):
        self.draw_abort.set()
        self.jog_clear()
        self.cancel_goal()
        for _ in range(10):
            self._publish_cmd(0.0, 0.0, 0.0)
            time.sleep(0.02)
        self._log(T('急停'))

    def on_close(self):
        self._closing = True
        try:
            self.estop()
        except Exception:
            pass
        if self.trace_proc is not None:
            try:
                self.trace_proc.send_signal(signal.SIGINT)
            except Exception:
                pass
        if self.arm is not None:
            self.arm.shutdown()
        # Unregister from the master before exiting.  Killing the process
        # instead leaves a /task_gui_<pid>_<time> publisher behind forever;
        # those zombies made the robot driver retry a dead address and stop
        # accepting new publishers.
        if self.ros is not None:
            try:
                self.ros.signal_shutdown('window closed')
                time.sleep(0.25)
            except Exception:
                pass
        self.root.destroy()

    # -- redraw ------------------------------------------------------------
    def _redraw_map(self):
        if self.map is None:
            return
        self.canvas.delete('dyn')
        with self.shared.lock:
            amcl = self.shared.amcl
            scan = None if self.shared.scan is None else self.shared.scan.copy()
            plan = None if self.shared.plan is None else list(self.shared.plan)

        if self.laser_on and scan is not None and len(scan):
            step = max(1, len(scan) // 400)
            for x, y in scan[::step]:
                px, py = self.map.to_px(x, y)
                if -5 < px < self.map.cw + 5 and -5 < py < self.map.ch + 5:
                    self.canvas.create_oval(px - 1.5, py - 1.5, px + 1.5,
                                            py + 1.5, fill='#e24b4a',
                                            outline='', tags='dyn')
        if plan and len(plan) > 1:
            pts = []
            for x, y in plan:
                pts.extend(self.map.to_px(x, y))
            self.canvas.create_line(*pts, fill='#639922', width=2, tags='dyn')
        if self._goal_first is not None:
            fx, fy = self.map.to_px(*self._goal_first)
            self.canvas.create_oval(fx - 7, fy - 7, fx + 7, fy + 7,
                                    outline='#d4537e', width=2, tags='dyn')
        if self.goal_px is not None:
            gx, gy = self.goal_px
            if self.goal_yaw is None:
                self.canvas.create_oval(gx - 7, gy - 7, gx + 7, gy + 7,
                                        outline='#378add', width=2,
                                        tags='dyn')
            else:
                # Arrow: canvas y grows downwards, map y grows upwards.
                ca, sa = math.cos(self.goal_yaw), math.sin(self.goal_yaw)
                ln = 18
                self.canvas.create_line(gx, gy, gx + ln * ca, gy - ln * sa,
                                        fill='#378add', width=2,
                                        arrow=tk.LAST, tags='dyn')
                self.canvas.create_oval(gx - 3, gy - 3, gx + 3, gy + 3,
                                        fill='#378add', outline='',
                                        tags='dyn')
        if amcl is not None:
            px, py = self.map.to_px(amcl[0], amcl[1])
            ln = 16
            ca, sa = math.cos(amcl[2]), math.sin(amcl[2])
            self.canvas.create_polygon(
                px + ln * ca, py - ln * sa,
                px - ln * 0.6 * ca + ln * 0.45 * sa,
                py + ln * 0.6 * sa + ln * 0.45 * ca,
                px - ln * 0.6 * ca - ln * 0.45 * sa,
                py + ln * 0.6 * sa - ln * 0.45 * ca,
                fill='#ef9f27', outline='#854f0b', tags='dyn')

    @staticmethod
    def _to_pil(msg):
        """sensor_msgs/Image -> PIL.  Astra sends rgb8, so cv_bridge is
        only the fallback for anything else."""
        enc = (getattr(msg, 'encoding', '') or '').lower()
        h, w = int(msg.height), int(msg.width)
        if enc in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            need = h * w * 3
            if arr.size < need:
                return None
            arr = arr[:need].reshape(h, w, 3)
            if enc == 'bgr8':
                arr = arr[:, :, ::-1]
            return Image.fromarray(np.ascontiguousarray(arr))
        if _BRIDGE is not None:
            try:
                return Image.fromarray(
                    _BRIDGE.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
            except Exception:
                return None
        return None

    def _redraw_cam(self):
        with self.shared.lock:
            msg = self.shared.img
            t = self.shared.img_t
        if msg is None or time.time() - t > 3.0:
            self.cam.itemconfig(self.cam_text,
                                text=T('相机无数据\n') + CAM_TOPIC)
            return
        if not _HAS_CV2:
            self.cam.itemconfig(self.cam_text, text=T('没装 opencv'))
            return
        if t <= self._cam_t:
            return              # same frame as last tick; don't decode twice
        self._cam_t = t
        try:
            im = self._to_pil(msg)
            if im is None:
                return
            # Decode to the canvas' current size: the camera pane grows with
            # the window, and thumbnailing to the constructor size would keep
            # the picture small inside a big frame.
            w = self.cam.winfo_width()
            h = self.cam.winfo_height()
            if w < 50 or h < 50:
                w, h = CAM_W, CAM_H
            im.thumbnail((w, h), Image.BILINEAR)
            self.cam_photo = ImageTk.PhotoImage(im)
            self.cam.delete('img')
            self.cam.coords(self.cam_text, w // 2, h // 2)
            self.cam.create_image(w // 2, h // 2,
                                  image=self.cam_photo, tags='img')
            self.cam.itemconfig(self.cam_text, text='')
        except Exception:
            pass

    def _redraw_status(self):
        with self.shared.lock:
            amcl = self.shared.amcl
            status = self.shared.status
            front = self.shared.front

        if (self.arm is not None and not self._jv_filled
                and getattr(self, 'jv', None)):
            p0 = self.arm.pose
            # Wait for a reply that has all three joints: the bus sometimes
            # answers two, and seeding j9 from a missing value wrote 0.0.
            if p0 is not None and all(j in p0 for j in self.jv):
                for jid, var in self.jv.items():
                    var.set('%.1f' % p0[jid])
                self._jv_filled = True

        laser = '--' if front is None else '%.2f m' % front
        self.tip_txt = ''
        if self.arm is None:
            arm = '臂未连接'
        else:
            p = self.arm.pose
            if p is None:
                arm = 'j7 -- j8 --'
            else:
                # The tip position goes to the status bar: this row shares
                # its width with a button and the speed knob.
                self.tip_txt = T('尖 %.0f/%.0f cm') % L.fk(p.get(7, 0.0),
                                                          p.get(8, 0.0))
                arm = 'j7 %.0f j8 %.0f' % (p.get(7, 0.0), p.get(8, 0.0))
        self.readout.config(text=T('激光 %s  %s') % (laser, arm))

        head = T('【%s】') % T(STAGE_TITLE[self.stage])
        if amcl is None:
            txt = T('%s  等待 /amcl_pose  --  导航栈起了吗？') % head
        else:
            txt = ((T('%s  x=%+.2f y=%+.2f yaw=%+.1f  |  σ %.2f/%.2f m %.1f°  |  ')
                    + T('move_base: %s  |  激光 %s'))
                   % (head, amcl[0], amcl[1], math.degrees(amcl[2]),
                      amcl[3], amcl[4], amcl[5], status,
                      '--' if front is None else '%.2f m' % front))
        with self.shared.lock:
            volts = self.shared.volts
            low = (volts is not None and volts < 11.0
                   and not self.shared.volts_warned)
            if low:
                self.shared.volts_warned = True
        if volts is not None:
            txt += '  |  电池 %.1f V%s' % (volts, '  !!' if volts < 11.0 else '')
            if low:
                self._log(T('电池 %.1f V：底盘只能输出约 20%% 的速度，'
                            '按下去像没反应，先充电') % volts)
        if self.tip_txt:
            txt += '  |  ' + self.tip_txt
        self.status.config(text=txt)

    def tick(self):
        if self._closing:
            return
        self.tick_n += 1
        self._drain_ui()
        # 20 Hz: the driver keeps the last /cmd_vel for ~0.15 s, so a slow
        # publisher turns every release into a visible coast.
        self._tick_jog()
        if self.tick_n % 4 == 0:
            self._redraw_map()
        if self.tick_n % 5 == 0:
            self._redraw_cam()
        if self.tick_n % 8 == 0:
            self._redraw_status()
        self.root.after(50, self.tick)


def _default_map():
    """maps/current names the map to load; fig8 is the fallback."""
    base = os.path.abspath(os.path.join(HERE, '..', 'maps'))
    try:
        with open(os.path.join(base, 'current')) as f:
            name = f.readline().strip()
    except Exception:
        name = ''
    if not name:
        name = 'fig8'
    return os.path.join(base, name + '.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=_default_map())
    ap.add_argument('--clear', type=float, default=0.45,
                    help='min clearance for a nav goal (m); the arm sticks '
                         'out ~0.27 m and the lidar cannot see it')
    ap.add_argument('--master-host', default=os.environ.get('ROBOT_IP', '192.168.1.11'))
    ap.add_argument('--robot-user', default='jetson')
    ap.add_argument('--robot-pass', default='yahboom')
    ap.add_argument('--coarse', default=os.path.abspath(os.path.join(HERE, '..', 'nav', 'scripts', 'coarse_localize.py')))
    ap.add_argument('--stage', default='nav', choices=list(STAGE_ORDER),
                    help='stage to open in (default nav)')
    ap.add_argument('--lang', default='zh', choices=('zh', 'en'),
                    help='interface language (default zh)')
    ap.add_argument('--offline', action='store_true')
    args = ap.parse_args()
    global LANG
    LANG = args.lang

    root = tk.Tk()
    root.geometry('1560x880')
    app = App(root, args)

    # Being killed (pkill from the launcher) must go through the same path as
    # closing the window, otherwise the recorder child is left behind and
    # keeps a /cmd_vel subscription alive.
    def _on_term(signum, frame):
        root.after(0, app.on_close)

    signal.signal(signal.SIGTERM, _on_term)
    root.bind('<KeyPress>', app.on_key)
    root.bind('<KeyRelease>', app.on_key_up)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
