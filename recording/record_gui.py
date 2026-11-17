#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_gui.py -- Transbot 录制控制台（控制机端）。

一块窗口里同时做四件事：

  1. 实时状态：在不在录、录了多久、bag 多大、写入速率、磁盘剩余、
               RGB/深度/雷达频率、底盘驱动和 ROS master 还在不在
  2. 录制控制：开始 / 优雅停止 / 拉回校验 / 删除机器人上的包
     （拉回和删除都先弹选包窗口，逐行标出本地有没有、是否已验证）
  3. 键盘遥控：W/S 前后、A/D 转向、Q/E 平移、空格急停
  4. 运行日志：脚本的原话都显示在下面，不用再去开终端

状态是怎么来的
    record_probe.py 跑在机器人上，每秒打一行 JSON；
    本程序用一条常驻 SSH 连接读它。
    好处：图像数据完全不经过 WiFi（只传几百字节/秒），
          而且 ROS 挂掉时照样能看到“谁还活着、盘还剩多少”。

安全设计（今天失控那件事的教训）
    * 遥控必须手动勾选“启用键盘遥控”才生效，避免误碰；
    * 松手就发零速；窗口失焦、最小化、关闭都会自动发零并解除；
    * 空格 / ESC = 急停：本机发零，同时让机器人本地也发零（绕开 WiFi）；
    * 配合 transbot_driver.py 的 0.5 秒看门狗：控制端掉线，车自己停。

用法
    ./record_gui.py
    ./record_gui.py --robot jetson@192.168.1.11
"""

import argparse
import glob
import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))
from i18n import tr, translate_log
from session_catalog import session_label
ROS_SETUP = '/opt/ros/melodic/setup.bash'


def ros_python_paths():
    """Where rospy lives, for whichever distro is installed.

    Hardcoding melodic/python2.7 works on the usual control PC -- Melodic's
    rospy happens to import under python3 too -- but on a Noetic machine the
    path does not exist and teleop dies with "rospy 导入失败". Glob instead.
    """
    return sorted(glob.glob('/opt/ros/*/lib/python*/dist-packages'))


# 导入期就把路径挂上：遥控线程和零速兜底都要用，谁先跑到都能 import。
for _p in ros_python_paths():
    if _p not in sys.path:
        sys.path.append(_p)

# 颜色
BG = '#eef1f4'
CARD = '#ffffff'
INK = '#22313a'
DIM = '#6b7b86'
OK = '#1b8a3a'
WARN = '#c47f00'
BAD = '#c62828'
ACCENT = '#1565c0'


def fmt_hms(sec):
    sec = int(max(0.0, sec))
    return '%d:%02d' % (sec // 60, sec % 60)


def start_args(duration, out, name=''):
    """录制台的 start 参数。

    名字留空就根本不传 -n —— 脚本那边退回纯时间戳命名，不会因为界面上
    空着一个框就拼出个怪名字来。
    """
    args = ['start', '-d', str(duration), '-o', out]
    if name:
        args += ['-n', name]
    return args


def local_ip_towards(host):
    """本机在“通往机器人”的那张网卡上的 IP。ROS_IP 必须填这个。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 22))
        return s.getsockname()[0]
    except Exception:
        return '127.0.0.1'
    finally:
        s.close()


# ======================================================================
#  SSH
# ======================================================================
class Remote:
    def __init__(self, target, password):
        self.target = target
        self.host = target.split('@')[-1]
        self.password = password
        self.has_sshpass = shutil.which('sshpass') is not None

    def _base(self):
        argv = []
        if self.has_sshpass:
            argv += ['sshpass', '-p', self.password]
        return argv

    def ssh_argv(self):
        return self._base() + [
            'ssh', '-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=6',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=3',
            self.target]

    def scp_argv(self):
        return self._base() + [
            'scp', '-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=6']

    def rosenv(self):
        """远端执行 ROS 命令前的环境准备。ROS_IP 必须是机器人自己的 IP。"""
        return ('source %s 2>/dev/null; export ROS_MASTER_URI=http://localhost:11311; '
                'export ROS_IP=%s; unset ROS_HOSTNAME;' % (ROS_SETUP, self.host))


# ======================================================================
#  键盘遥控
# ======================================================================
class Teleop:
    """20 Hz 往 /cmd_vel 发速度。独立线程，界面卡了也不影响发零。"""

    def __init__(self, remote):
        self.remote = remote
        self.lock = threading.Lock()
        self.keys = set()
        self.armed = False
        self.speed = 0.15
        self.stop_flag = False
        self.ready = False
        self.error = ''
        self.pub = None
        self._was_armed = False
        self._thread = None
        self._reset = threading.Event()

    # -- 外部接口 ------------------------------------------------------
    def start(self):
        self._thread = threading.Thread(target=self._loop, name='teleop', daemon=True)
        self._thread.start()

    def set_armed(self, on):
        with self.lock:
            self.armed = bool(on)

    def set_speed(self, v):
        with self.lock:
            self.speed = float(v)

    def key(self, name, down):
        with self.lock:
            if down:
                self.keys.add(name)
            else:
                self.keys.discard(name)

    def clear_keys(self):
        with self.lock:
            self.keys.clear()

    def reconnect(self):
        self.set_armed(False);self.clear_keys();self._reset.set()
        if not self._thread or not self._thread.is_alive():self.start()

    def zero_burst(self, n=5):
        with self.lock:
            pub = self.pub
        if pub is None:
            return
        try:
            from geometry_msgs.msg import Twist
            for _ in range(n):
                pub.publish(Twist())
                time.sleep(0.02)
        except Exception:
            pass

    def shutdown(self):
        self.stop_flag = True
        self.set_armed(False)
        self.zero_burst(10)

    # -- 线程 ----------------------------------------------------------
    def _loop(self):
        # ROS 环境必须在这里设好，再 import rospy
        os.environ['ROS_MASTER_URI'] = 'http://%s:11311' % self.remote.host
        os.environ['ROS_IP'] = local_ip_towards(self.remote.host)
        os.environ.pop('ROS_HOSTNAME', None)
        for _p in ros_python_paths():
            if _p not in sys.path:
                sys.path.append(_p)
        try:
            import rospy
            from geometry_msgs.msg import Twist
        except Exception as exc:
            self.error = tr('rospy 导入失败：%s') % exc
            return

        try:
            if not rospy.core.is_initialized():
                rospy.init_node('record_gui_teleop', anonymous=True, disable_signals=True)
            self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        except Exception as exc:
            self.error = tr('连不上 ROS master：%s') % exc
            return

        self.ready = True
        while not self.stop_flag:
            if self._reset.is_set():
                self._reset.clear()
                try:
                    if self.pub:self.pub.unregister()
                    self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
                    self.ready=True;self.error=''
                except Exception as exc:self.ready=False;self.error=str(exc)
            with self.lock:
                armed = self.armed
                keys = set(self.keys)
                spd = self.speed

            if armed:
                vx = vy = wz = 0.0
                if 'fwd' in keys:
                    vx += spd
                if 'back' in keys:
                    vx -= spd
                if 'left' in keys:
                    vy += spd * 0.7
                if 'right' in keys:
                    vy -= spd * 0.7
                if 'ccw' in keys:
                    wz += spd * 3.0
                if 'cw' in keys:
                    wz -= spd * 3.0
                tw = Twist()
                tw.linear.x = round(vx, 3)
                tw.linear.y = round(vy, 3)
                tw.angular.z = round(wz, 3)
                try:
                    self.pub.publish(tw)
                except Exception:
                    pass
                self._was_armed = True
            elif self._was_armed:
                # 刚解除：补几帧零速，确保机器人收到
                self.zero_burst(3)
                self._was_armed = False
            time.sleep(.05)


# ======================================================================
#  选包对话框（拉回 / 删除共用）
# ======================================================================
LOCAL_LABEL = {
    'missing': ('本地无', BAD),
    'size-mismatch': ('尺寸不同', BAD),
}


def human_size(n):
    n = float(n)
    for unit in ('B', 'KiB', 'MiB', 'GiB'):
        if n < 1024 or unit == 'GiB':
            return ('%.0f %s' if unit == 'B' else '%.1f %s') % (n, unit)
        n /= 1024.0


class SelectDialog(tk.Toplevel):
    """Pick which bags to pull, or which to delete. One widget, two modes.

    Rows come from `pull --list-json`. Delete mode greys out anything that
    cannot have a verified local copy, because --delete-only refuses to remove
    a remote bag it cannot prove is safe to remove.
    """

    def __init__(self, master, rows, mode, meta, on_done):
        tk.Toplevel.__init__(self, master)
        self.result = None
        self.mode = mode
        self.rows = rows
        self.picks = []
        self.on_done = on_done
        pull = mode == 'pull'

        self.title(tr('选择要拉回的包') if pull else tr('选择要删除的包'))
        self.configure(bg=BG)
        self.transient(master)
        sw, sh = master.winfo_screenwidth(), master.winfo_screenheight()
        self.geometry('%dx%d' % (min(840, sw - 80), min(580, sh - 120)))
        self.minsize(660, 380)
        self.bind('<Escape>', lambda e: self._cancel())

        head = tk.Frame(self, bg=BG)
        head.pack(fill=tk.X, padx=16, pady=(14, 2))
        tk.Label(head,
                 text=(tr('从机器人拉回，逐文件校验 SHA-256 后落地')
                       if pull else
                       tr('从机器人删除；本地没有校验一致的副本就跳过')),
                 bg=BG, fg=INK, font=('', 12, 'bold')).pack(anchor='w')
        tk.Label(head, text=tr('机器人：%s') % meta.get('remote', '—'),
                 bg=BG, fg=DIM, font=('', 9)).pack(anchor='w', pady=(4, 0))
        tk.Label(head, text=tr('落地目录：%s') % meta.get('local_root', '—'),
                 bg=BG, fg=DIM, font=('', 9)).pack(anchor='w')

        body = tk.Frame(self, bg=CARD, bd=1, relief=tk.GROOVE)
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=10)

        hdr = tk.Frame(body, bg=CARD)
        hdr.pack(fill=tk.X, padx=10, pady=(8, 2))
        tk.Label(hdr, text=tr('包名'), bg=CARD, fg=DIM, font=('', 9)).pack(side=tk.LEFT)
        tk.Label(hdr, text=tr('本地'), bg=CARD, fg=DIM, font=('', 9),
                 width=10, anchor='e').pack(side=tk.RIGHT)
        tk.Label(hdr, text=tr('大小'), bg=CARD, fg=DIM, font=('', 9),
                 width=10, anchor='e').pack(side=tk.RIGHT)

        wrap = tk.Frame(body, bg=CARD)
        wrap.pack(fill=tk.BOTH, expand=True, padx=(4, 0), pady=(0, 6))
        self.canvas = tk.Canvas(wrap, bg=CARD, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = tk.Frame(self.canvas, bg=CARD)
        win = self.canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: self.canvas.configure(
            scrollregion=self.canvas.bbox('all')))
        self.canvas.bind('<Configure>', lambda e: self.canvas.itemconfigure(
            win, width=e.width))
        for w in (self.canvas, inner):
            w.bind('<MouseWheel>', self._wheel)
            w.bind('<Button-4>', self._wheel)
            w.bind('<Button-5>', self._wheel)

        for i, row in enumerate(rows):
            state = row.get('local')
            if state == 'partial':
                pct = 100.0 * row.get('partial_size', 0) / max(1, row['size'])
                text, colour = tr('传输中 %.0f%%') % pct, WARN
            elif state in LOCAL_LABEL:
                text, colour = tr(LOCAL_LABEL[state][0]), LOCAL_LABEL[state][1]
            elif row.get('recorded'):
                text, colour = tr('已验证'), OK
            else:
                text, colour = tr('本地有'), WARN

            # 半截的 .part 不算本地副本：删不得，但拉回来能接着传。
            blocked = (not pull) and (state in LOCAL_LABEL or state == 'partial')
            var = tk.BooleanVar(value=(True if pull else bool(row.get('recorded'))))
            tk.Checkbutton(inner, variable=var, bg=CARD, activebackground=CARD,
                           highlightthickness=0, bd=0, command=self._recount,
                           state=(tk.DISABLED if blocked else tk.NORMAL)
                           ).grid(row=i, column=0, sticky='w', padx=(8, 0), pady=3)
            tk.Label(inner, text=row['name'], bg=CARD,
                     fg=(DIM if blocked else INK), font=('', 10), anchor='w'
                     ).grid(row=i, column=1, sticky='we', padx=(2, 8))
            tk.Label(inner, text=human_size(row['size']), bg=CARD, fg=DIM,
                     font=('', 9), width=10, anchor='e'
                     ).grid(row=i, column=2, sticky='e')
            tk.Label(inner, text=text, bg=CARD, fg=colour, font=('', 9),
                     width=10, anchor='e'
                     ).grid(row=i, column=3, sticky='e', padx=(0, 10))
            self.picks.append((var, row, blocked))
        inner.columnconfigure(1, weight=1)

        foot = tk.Frame(self, bg=BG)
        foot.pack(fill=tk.X, padx=16, pady=(0, 14))
        for label, cmd in ((tr('全选'), lambda: self._set_all(True)),
                           (tr('全不选'), lambda: self._set_all(False))):
            tk.Button(foot, text=label, relief=tk.FLAT, padx=8,
                      command=cmd).pack(side=tk.LEFT)
        quick = ((tr('只选本地没有的'), self._only_missing) if pull
                 else (tr('只选已验证的'), self._only_recorded))
        tk.Button(foot, text=quick[0], relief=tk.FLAT, padx=8,
                  command=quick[1]).pack(side=tk.LEFT, padx=(6, 0))

        self.lbl_sum = tk.Label(foot, text='', bg=BG, fg=DIM, font=('', 9))
        self.lbl_sum.pack(side=tk.LEFT, padx=12)

        self.btn_ok = tk.Button(foot, text=tr('确定'), bg=ACCENT, fg='white',
                                activebackground='#0d47a1', activeforeground='white',
                                relief=tk.FLAT, font=('', 10, 'bold'),
                                padx=16, pady=5, command=self._ok)
        self.btn_ok.pack(side=tk.RIGHT)
        tk.Button(foot, text=tr('取消'), relief=tk.FLAT, padx=12, pady=5,
                  command=self._cancel).pack(side=tk.RIGHT, padx=8)

        self._recount()
        self.update_idletasks()
        try:
            self.grab_set()
        except tk.TclError:
            pass

    def _wheel(self, ev):
        num = getattr(ev, 'num', None)
        step = -1 if num == 4 else 1 if num == 5 else (-1 if ev.delta > 0 else 1)
        self.canvas.yview_scroll(step, 'units')

    def _set_all(self, on):
        for var, _row, blocked in self.picks:
            var.set(on and not blocked)
        self._recount()

    def _only_missing(self):
        # 半截的 .part 也算“本地没有”——它本来就要接着传完。
        for var, row, blocked in self.picks:
            var.set(row.get('local') in LOCAL_LABEL or row.get('local') == 'partial')
        self._recount()

    def _only_recorded(self):
        for var, row, blocked in self.picks:
            var.set(bool(row.get('recorded')) and not blocked)
        self._recount()

    def _chosen(self):
        return [row['name'] for var, row, blocked in self.picks
                if var.get() and not blocked]

    def _recount(self):
        names = self._chosen()
        total = sum(row['size'] for var, row, blocked in self.picks
                    if var.get() and not blocked)
        self.lbl_sum.configure(
            text=tr('已选 %d 个，共 %s') % (len(names), human_size(total)))
        self.btn_ok.configure(
            text=(tr('拉回选中的 %d 个') if self.mode == 'pull'
                  else tr('删除选中的 %d 个')) % len(names),
            state=(tk.NORMAL if names else tk.DISABLED))

    def _ok(self):
        self.result = self._chosen()
        names = self.result
        self.destroy()
        self.on_done(names)

    def _cancel(self):
        self.result = None
        self.destroy()


# ======================================================================
#  主窗口
# ======================================================================
class App:
    def __init__(self, root, remote):
        self.root = root
        self.remote = remote
        self.q = queue.Queue()
        self.state = {}
        self.state_t = 0.0
        self.online = False
        self.rtt = None
        self.closing = False
        self.busy = None
        self.last_rec = None
        self.log_lines = 0
        self.clock_offset=None

        self.teleop = Teleop(remote)
        self.teleop.start()

        self._build()

        self._probe_thread = threading.Thread(target=self._probe_loop, daemon=True)
        self._probe_thread.start()
        threading.Thread(target=self._ping_loop, daemon=True).start()

        self.root.after(200, self._tick)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    # ---------------------------------------------------------------- UI
    def _build(self):
        r = self.root
        r.title(tr('Transbot 录制控制台'))
        # 1120 太窄：键盘卡片最右一列（D 右转 / E 右平移）会被窗口切掉，
        # 实测 1180 才刚好放下。按屏幕给，别超出去。
        sw, sh = r.winfo_screenwidth(), r.winfo_screenheight()
        w = max(1, min(1400, sw - 40))
        h = max(1, min(900, sh - 80))
        r.geometry('%dx%d+0+0' % (w, h))
        r.minsize(min(960, w), min(480, h))
        r.configure(bg=BG)

        # ---- 顶栏 ----
        head = tk.Frame(r, bg=BG)
        head.pack(fill=tk.X, padx=14, pady=(12, 6))

        tk.Label(head, text=tr('Transbot 录制控制台'), bg=BG, fg=INK,
                 font=('', 17, 'bold')).pack(side=tk.LEFT)

        self.lbl_conn = tk.Label(head, text=tr('● 未连接'), bg=BG, fg=BAD,
                                 font=('', 11, 'bold'))
        self.lbl_conn.pack(side=tk.LEFT, padx=(16, 0))

        self.btn_estop = tk.Button(head, text=tr('急停  ESC'), bg='#e53935', fg='white',
                                   activebackground='#b71c1c', activeforeground='white',
                                   font=('', 13, 'bold'), relief=tk.FLAT,
                                   padx=22, pady=6, command=self.estop)
        self.btn_estop.pack(side=tk.RIGHT)

        self.lbl_rtt = tk.Label(head, text='', bg=BG, fg=DIM, font=('', 9))
        self.lbl_rtt.pack(side=tk.RIGHT, padx=12)

        # ---- 状态卡片 ----
        cards = tk.Frame(r, bg=BG)
        cards.pack(fill=tk.X, padx=14)
        for i in range(4):
            cards.columnconfigure(i, weight=1, uniform='c')

        self.card_rec = self._card(cards, tr('录制'), 0)
        self.v_rec = tk.Label(self.card_rec, text=tr('空闲'), bg=CARD, fg=DIM,
                              font=('', 15, 'bold'), anchor='w')
        self.v_rec.pack(fill=tk.X)
        self.v_rec_sub = tk.Label(self.card_rec, text='—', bg=CARD, fg=INK,
                                  font=('', 10), anchor='w', justify=tk.LEFT)
        self.v_rec_sub.pack(fill=tk.X, pady=(2, 0))
        self.v_rec_sub2 = tk.Label(self.card_rec, text='', bg=CARD, fg=DIM,
                                   font=('', 9), anchor='w', justify=tk.LEFT)
        self.v_rec_sub2.pack(fill=tk.X)
        self.v_rec_sub3 = tk.Label(self.card_rec, text='', bg=CARD, fg=DIM,
                                   font=('', 9), anchor='w', justify=tk.LEFT)
        self.v_rec_sub3.pack(fill=tk.X)

        self.card_disk = self._card(cards, tr('存储'), 1)
        self.v_disk = tk.Label(self.card_disk, text='—', bg=CARD, fg=INK,
                               font=('', 15, 'bold'), anchor='w')
        self.v_disk.pack(fill=tk.X)
        self.v_disk_sub = tk.Label(self.card_disk, text='', bg=CARD, fg=DIM,
                                   font=('', 9), anchor='w', justify=tk.LEFT)
        self.v_disk_sub.pack(fill=tk.X, pady=(2, 0))
        self.v_disk_sub2 = tk.Label(self.card_disk, text='', bg=CARD, fg=DIM,
                                    font=('', 9), anchor='w', justify=tk.LEFT)
        self.v_disk_sub2.pack(fill=tk.X)

        self.card_src = self._card(cards, tr('数据源'), 2)
        self.v_src = {}
        for key, name in (('rgb', tr('彩色 RGB')), ('depth', tr('深度')), ('scan', tr('激光雷达'))):
            row = tk.Frame(self.card_src, bg=CARD)
            row.pack(fill=tk.X)
            tk.Label(row, text=name, bg=CARD, fg=DIM, width=9, anchor='w',
                     font=('', 9)).pack(side=tk.LEFT)
            lab = tk.Label(row, text='--', bg=CARD, fg=INK, font=('', 10, 'bold'),
                           anchor='e')
            lab.pack(side=tk.RIGHT)
            self.v_src[key] = lab

        self.card_link = self._card(cards, tr('链路 / 底盘'), 3)
        self.v_link = {}
        for key, name in (('drv', tr('底盘驱动')), ('master', 'ROS master')):
            row = tk.Frame(self.card_link, bg=CARD)
            row.pack(fill=tk.X)
            tk.Label(row, text=name, bg=CARD, fg=DIM, width=9, anchor='w',
                     font=('', 9)).pack(side=tk.LEFT)
            lab = tk.Label(row, text='--', bg=CARD, fg=INK, font=('', 10, 'bold'),
                           anchor='e')
            lab.pack(side=tk.RIGHT)
            self.v_link[key] = lab
        self.v_tel = tk.Label(self.card_link, text=tr('遥控：未启用'), bg=CARD, fg=DIM,
                              font=('', 9), anchor='w')
        self.v_tel.pack(fill=tk.X, pady=(4, 0))
        self.v_clock=tk.Label(self.card_link,text=tr('时间：等待状态'),bg=CARD,fg=DIM,anchor='w',font=('',9))
        self.v_clock.pack(fill=tk.X)

        prep=tk.LabelFrame(r,text=tr(' 录前准备 '),bg=CARD,fg=INK,padx=10,pady=6)
        prep.pack(fill=tk.X,padx=14,pady=(8,0))
        self.prepare_buttons=[]
        for action,label in [('all','一键准备'),('clock','校准时间'),('camera','启动/恢复相机'),('lidar','启动/恢复雷达')]:
            button=tk.Button(prep,text=tr(label),padx=9,command=lambda a=action:self.on_prepare(a))
            button.pack(side=tk.LEFT,padx=(0,7));self.prepare_buttons.append(button)
        tk.Label(prep,text=tr('仅空闲时使用；校时或恢复驱动可能重启数据源'),bg=CARD,fg=DIM,font=('',9)).pack(side=tk.LEFT)

        # ---- 中间：录制控制 + 键盘遥控 ----
        mid = tk.Frame(r, bg=BG)
        mid.pack(fill=tk.X, padx=14, pady=10)
        mid.columnconfigure(0, weight=3)
        mid.columnconfigure(1, weight=2)

        # 录制控制
        box = tk.LabelFrame(mid, text=tr(' 录制 '), bg=CARD, fg=INK, bd=1, relief=tk.GROOVE,
                            font=('', 10, 'bold'), padx=12, pady=10)
        box.grid(row=0, column=0, sticky='nsew', padx=(0, 10))

        row = tk.Frame(box, bg=CARD)
        row.pack(fill=tk.X)
        tk.Label(row, text=tr('时长'), bg=CARD, fg=DIM).pack(side=tk.LEFT)
        self.var_dur = tk.StringVar(value='300')
        tk.Spinbox(row, from_=10, to=1800, increment=30, width=6,
                   textvariable=self.var_dur, justify='center').pack(side=tk.LEFT, padx=(6, 4))
        tk.Label(row, text=tr('秒   （5 分钟 = 300 秒）'), bg=CARD, fg=DIM).pack(side=tk.LEFT)

        row = tk.Frame(box, bg=CARD)
        row.pack(fill=tk.X, pady=(6, 0))
        tk.Label(row, text=tr('录制根目录'), bg=CARD, fg=DIM).pack(side=tk.LEFT)
        self.var_dir = tk.StringVar(value='/home/jetson/bags')
        tk.Entry(row, textvariable=self.var_dir, width=34).pack(side=tk.LEFT, padx=6)

        row = tk.Frame(box, bg=CARD)
        row.pack(fill=tk.X, pady=(6, 0))
        tk.Label(row, text=tr('回传根目录'), bg=CARD, fg=DIM).pack(side=tk.LEFT)
        self.var_local = tk.StringVar(value=os.environ.get(
            'LOCAL_DIR', os.path.expanduser('~/robot_research/bagtools/bags')))
        tk.Entry(row, textvariable=self.var_local, width=34).pack(side=tk.LEFT, padx=6)

        # 会话名：只影响会话目录名，包名（cam_<时间戳>.bag）保持不变。
        # 留空就是纯时间戳 —— 名字是可选的，不该拦着人开录。
        row = tk.Frame(box, bg=CARD)
        row.pack(fill=tk.X, pady=(6, 0))
        tk.Label(row, text=tr('会话名'), bg=CARD, fg=DIM).pack(side=tk.LEFT)
        self.var_name = tk.StringVar(value='')
        name_entry = tk.Entry(row, textvariable=self.var_name, width=22)
        name_entry.pack(side=tk.LEFT, padx=6)
        tk.Label(row, text=tr('可留空；留空就是纯时间戳'), bg=CARD, fg=DIM,
                 font=('', 9)).pack(side=tk.LEFT)
        self.lbl_name = tk.Label(box, text='', bg=CARD, fg=DIM, anchor='w',
                                 font=('', 9))
        self.lbl_name.pack(fill=tk.X, pady=(2, 0))
        self.var_name.trace('w', lambda *a: self._preview_name())

        row = tk.Frame(box, bg=CARD)
        row.pack(fill=tk.X, pady=(10, 0))
        self.btn_start = tk.Button(row, text=tr('开始录制'), bg='#2e7d32', fg='white',
                                   activebackground='#1b5e20', activeforeground='white',
                                   relief=tk.FLAT, font=('', 10, 'bold'), padx=16, pady=5,
                                   command=self.on_start)
        self.btn_start.pack(side=tk.LEFT)
        self.btn_stop = tk.Button(row, text=tr('停止'), bg='#ef6c00', fg='white',
                                  activebackground='#e65100', activeforeground='white',
                                  relief=tk.FLAT, font=('', 10, 'bold'), padx=16, pady=5,
                                  command=self.on_stop)
        self.btn_stop.pack(side=tk.LEFT, padx=8)
        self.btn_pull = tk.Button(row, text=tr('拉回校验'), relief=tk.FLAT, font=('', 10),
                                  padx=12, pady=5, command=self.on_pull)
        self.btn_pull.pack(side=tk.LEFT)
        self.btn_delete = tk.Button(row, text=tr('删除机器人上的包'), relief=tk.FLAT,
                                    font=('', 10), padx=12, pady=5,
                                    command=self.on_delete_remote)
        self.btn_delete.pack(side=tk.LEFT, padx=8)

        self.v_hint = tk.Label(box, text='', bg=CARD, fg=DIM, anchor='w',
                               font=('', 9), justify=tk.LEFT)
        self.v_hint.pack(fill=tk.X, pady=(8, 0))

        # 键盘遥控
        tb = tk.LabelFrame(mid, text=tr(' 键盘遥控 '), bg=CARD, fg=INK, bd=1, relief=tk.GROOVE,
                           font=('', 10, 'bold'), padx=12, pady=8)
        tb.grid(row=0, column=1, sticky='nsew')

        self.var_arm = tk.BooleanVar(value=False)
        tk.Checkbutton(tb, text=tr('启用键盘遥控（勾选后按键才有效）'), bg=CARD,
                       variable=self.var_arm, command=self.on_arm,
                       font=('', 10)).pack(anchor='w')

        row = tk.Frame(tb, bg=CARD)
        row.pack(fill=tk.X, pady=(4, 2))
        tk.Label(row, text=tr('速度'), bg=CARD, fg=DIM).pack(side=tk.LEFT)
        self.var_spd = tk.DoubleVar(value=0.15)
        self.lbl_spd = tk.Label(row, text=tr('0.15 m/s  转向 0.45 rad/s'), bg=CARD, fg=INK,
                                font=('', 9))
        self.lbl_spd.pack(side=tk.RIGHT)
        tk.Scale(row, from_=0.05, to=0.40, resolution=0.01, orient=tk.HORIZONTAL,
                 variable=self.var_spd, showvalue=False, bg=CARD,
                 highlightthickness=0, length=110,
                 command=self.on_speed).pack(side=tk.LEFT, padx=8)

        pad = tk.Frame(tb, bg=CARD)
        pad.pack(pady=(2, 0))
        b_w = tk.Button(pad, text=tr('W  前进'), width=10, height=2, font=('', 9))
        b_w.grid(row=0, column=1, padx=2, pady=2)
        b_a = tk.Button(pad, text=tr('A  左转'), width=10, height=2, font=('', 9))
        b_a.grid(row=1, column=0, padx=2, pady=2)
        b_s = tk.Button(pad, text=tr('S  后退'), width=10, height=2, font=('', 9))
        b_s.grid(row=1, column=1, padx=2, pady=2)
        b_d = tk.Button(pad, text=tr('D  右转'), width=10, height=2, font=('', 9))
        b_d.grid(row=1, column=2, padx=2, pady=2)
        b_q = tk.Button(pad, text=tr('Q 左平移'), width=10, height=1, font=('', 8))
        b_q.grid(row=2, column=0, padx=2, pady=(6, 2))
        b_e = tk.Button(pad, text=tr('E 右平移'), width=10, height=1, font=('', 8))
        b_e.grid(row=2, column=2, padx=2, pady=(6, 2))

        # 鼠标也能开：按住按钮 = 按住键
        for btn, key in ((b_w, 'fwd'), (b_s, 'back'), (b_a, 'ccw'),
                         (b_d, 'cw'), (b_q, 'left'), (b_e, 'right')):
            btn.bind('<ButtonPress-1>', lambda e, k=key: self._mouse_key(k, True))
            btn.bind('<ButtonRelease-1>', lambda e, k=key: self._mouse_key(k, False))

        self.v_tel_note = tk.Label(tb, text=tr('空格 / ESC = 急停'), bg=CARD, fg=DIM,
                                   font=('', 9))
        self.v_tel_note.pack(anchor='w', pady=(6, 0))

        # ---- 日志 ----
        lf = tk.LabelFrame(r, text=tr(' 运行日志 '), bg=CARD, fg=INK, bd=1, relief=tk.GROOVE,
                           font=('', 10, 'bold'))
        lf.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 12))
        self.log = tk.Text(lf, height=9, bg='#12191f', fg='#d6e2ea',
                           insertbackground='#d6e2ea', relief=tk.FLAT,
                           font=('TkFixedFont', 9), wrap=tk.WORD)
        sb = tk.Scrollbar(lf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.log.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.log.configure(state=tk.DISABLED)

        # ---- 键盘 ----
        r.bind_all('<KeyPress>', self.on_key)
        r.bind_all('<KeyRelease>', self.on_key_up)
        r.bind_all('<Escape>', lambda e: self.estop())
        r.bind_all('<space>', lambda e: self.estop())
        r.bind('<Unmap>', lambda e: self._disarm(tr('窗口最小化')))
        r.bind('<FocusOut>', self._on_focus_out)

        self._log(tr('控制台已启动。机器人 %s') % self.remote.target)
        if not self.remote.has_sshpass:
            self._log(tr('提示：没装 sshpass，将使用 SSH 密钥登录（可能需要输密码）。'))

    def _card(self, parent, title, col):
        f = tk.LabelFrame(parent, text=' %s ' % title, bg=CARD, fg=INK, bd=1,
                          relief=tk.GROOVE, font=('', 10, 'bold'), padx=12, pady=8)
        f.grid(row=0, column=col, sticky='nsew', padx=(0 if col == 0 else 8, 0))
        return f

    # ------------------------------------------------------------ 日志
    def _log(self, msg):
        msg = translate_log(msg)
        ts = time.strftime('%H:%M:%S')
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, '[%s] %s\n' % (ts, msg))
        self.log_lines += 1
        if self.log_lines > 800:
            self.log.delete('1.0', '200.0')
            self.log_lines -= 199
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    # ------------------------------------------------------------ 探针
    def _probe_loop(self):
        # 每次（重）连都上传一次，改过探针就自动生效
        try:
            subprocess.run(self.remote.scp_argv() +
                           [os.path.join(SCRIPT_DIR, 'record_probe.py'),
                            '%s:/home/jetson/' % self.remote.target],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=30)
        except Exception:
            pass

        # 用 bash -c 而不是 bash -lc：登录 shell 会读 .bashrc，实测慢 4~6 秒
        remote_cmd = ("bash -c '%s export ROS_IP=%s; exec python3 -u "
                      "/home/jetson/record_probe.py'" %
                      (self.remote.rosenv(), self.remote.host))

        while not self.closing:
            proc = None
            try:
                proc = subprocess.Popen(self.remote.ssh_argv() + [remote_cmd],
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT,
                                        universal_newlines=True, bufsize=1,
                                        env=dict(os.environ, ROBOT=self.remote.target, ROBOT_PASS=self.remote.password))
                for line in proc.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith('{'):
                        try:
                            self.q.put(('status', json.loads(line)))
                        except Exception:
                            pass
                    else:
                        self.q.put(('raw', line))
                proc.wait()
            except Exception as exc:
                self.q.put(('raw', tr('探针连接异常：%s') % exc))
            finally:
                if proc and proc.poll() is None:
                    proc.terminate()
            self.q.put(('offline', None))
            time.sleep(2)

    def _ping_loop(self):
        while not self.closing:
            try:
                out = subprocess.run(['ping', '-c', '1', '-W', '1', self.remote.host],
                                     stdout=subprocess.PIPE, timeout=4).stdout.decode(
                    'utf-8', 'replace')
                m = None
                for part in out.split():
                    if part.startswith('time='):
                        m = float(part.split('=')[1])
                        break
                self.rtt = m
            except Exception:
                self.rtt = None
            time.sleep(3)

    # ------------------------------------------------------------ 定时刷新
    def _tick(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == 'status':
                    self.state = payload
                    self.state_t = time.monotonic()
                    self._public_show(payload)
                elif kind == 'raw':
                    self._log(payload)
                elif kind == 'offline':
                    if self.online:
                        self._log(tr('与机器人的状态连接断开，正在重连…'))
                        self._disarm(tr('状态连接断开'))
                    self.online = False
                elif kind == 'log':
                    self._log(payload)
                elif kind == 'busy':
                    self.busy = payload
                    self._refresh_buttons()
                elif kind == 'teleop_reconnect':
                    self.teleop.reconnect()
                elif kind == 'listing':
                    self._open_picker(payload)
        except queue.Empty:
            pass

        self._render()
        if not self.closing:
            self.root.after(200, self._tick)

    def _public_show(self, d):
        try:self.clock_offset=float(d['t'])-time.time()
        except (KeyError,TypeError,ValueError):self.clock_offset=None
        if not self.online:
            self.online = True
            self._log(tr('已连上机器人 %s（状态每秒刷新）') % self.remote.target)
        rec = bool(d.get('rec'))
        prev = self.last_rec
        if rec != prev:
            self.last_rec = rec
            if rec:
                self._log(tr('开始录制：%s') % d.get('bag', ''))
            elif prev is True:
                self._log(tr('录制已停止。'))

    def _render(self):
        d = self.state
        stale = (time.monotonic() - self.state_t) > 4.0 or not d

        # 连接状态
        if stale:
            self.online = False
            self.lbl_conn.configure(text=tr('● 未连接'), fg=BAD)
            self.lbl_rtt.configure(text='')
        else:
            self.lbl_conn.configure(text=tr('● 在线'), fg=OK)
            self.lbl_rtt.configure(
                text=('WiFi %d ms' % self.rtt) if self.rtt is not None else 'WiFi --')

        # 录制卡片
        phase=d.get('phase','recording' if d.get('rec') else 'idle')
        if not stale and phase in ('preparing','finalizing'):
            self.v_rec.configure(text=tr('● 准备中') if phase=='preparing' else tr('● 收尾中'),fg=WARN)
            self.v_rec_sub.configure(text=tr('正在检查和启动数据源，请等待') if phase=='preparing' else tr('正在关闭并校验 bag，请等待'))
            self.v_rec_sub2.configure(text='')
        elif not stale and d.get('rec'):
            self.v_rec.configure(text=tr('● 录制中'), fg=OK)
            self.v_rec_sub.configure(
                text=tr('%s    已录 %s') % (fmt_hms(d.get('elapsed', 0)),
                                       '%.1f MB' % d.get('mb', 0)))
            self.v_rec_sub2.configure(text=tr('写入 %.1f MB/s') % d.get('mbps', 0))
            nsess = d.get('sessions', 1)
            if nsess > 1:
                # 两路同时写盘 = 帧率对半砍 + 出现几十秒空洞（11-10 那次）
                # 卡片窄，这里只放"几路"；包名留给日志
                self.v_rec_sub3.configure(
                    text=tr('⚠ %d 路录制同时写盘') % nsess, fg=WARN)
                if not getattr(self, '_multi_warned', False):
                    self._multi_warned = True
                    self._log(tr('⚠ 检测到 %d 路录制在同时写盘（%s），帧率会掉，先停掉多余那路')
                              % (nsess, d.get('bag', '')))
            else:
                self.v_rec_sub3.configure(text=d.get('bag', ''), fg=DIM)
                self._multi_warned = False
        else:
            self.v_rec.configure(text=tr('○ 空闲') if not stale else tr('○ 未知'), fg=DIM)
            self.v_rec_sub.configure(text='—' if stale else tr('没有正在写的 bag'))
            self.v_rec_sub2.configure(text='')
            self.v_rec_sub3.configure(text='')

        # 磁盘
        if not stale:
            free = d.get('disk_free_gb', -1)
            self.v_disk.configure(text=tr('%.1f GB 可用') % free,
                                  fg=OK if free > 5 else (WARN if free > 3 else BAD))
            mins = int(free * 1000 / 20 / 60) if free > 0 else 0
            self.v_disk_sub.configure(text=tr('按 20 MB/s 约可录 %d 分钟') % mins)
            self.v_disk_sub2.configure(
                text=tr('1 分钟约 1.2 GB，5 分钟约 5.9 GB'))
        else:
            self.v_disk.configure(text='—', fg=DIM)
            self.v_disk_sub.configure(text='')
            self.v_disk_sub2.configure(text='')

        # 数据源频率
        if not stale and d.get('ros_ok'):
            for key, good, warn in (('rgb', 20, 8), ('depth', 20, 8), ('scan', 5, 2)):
                hz = d.get(key, -1)
                if hz is None or hz < 0:
                    self.v_src[key].configure(text='--', fg=DIM)
                else:
                    fg = OK if hz >= good else (WARN if hz >= warn else BAD)
                    self.v_src[key].configure(text='%.1f Hz' % hz, fg=fg)
        elif not stale and d.get('health_state')=='not_recording':
            for key in ('rgb','depth','scan'):
                self.v_src[key].configure(text='--',fg=DIM)
        elif stale:
            for key in ('rgb', 'depth', 'scan'):
                self.v_src[key].configure(text='--', fg=DIM)
        else:
            for key in ('rgb', 'depth', 'scan'):
                self.v_src[key].configure(text=tr('ROS 连接失败'), fg=BAD)

        # 链路 / 底盘
        if stale:
            for key in ('drv', 'master'):
                self.v_link[key].configure(text='--', fg=DIM)
        else:
            self.v_link['drv'].configure(
                text=tr('正常') if d.get('driver') else tr('不在运行'),
                fg=OK if d.get('driver') else BAD)
            self.v_link['master'].configure(
                text=tr('正常') if d.get('master') else tr('不在运行'),
                fg=OK if d.get('master') else BAD)

        if self.teleop.ready:
            self.v_tel.configure(
                text=tr('遥控：%s') % (tr('已启用（松手即停）') if self.var_arm.get() else tr('未启用')),
                fg=ACCENT if self.var_arm.get() else DIM)
        else:
            self.v_tel.configure(text=tr('遥控：ROS 未连接'), fg=BAD)
        if stale or self.clock_offset is None:
            self.v_clock.configure(text=tr('时间：等待状态'),fg=DIM)
        else:
            self.v_clock.configure(text=tr('时间差：%+.2f 秒')%self.clock_offset,fg=OK if abs(self.clock_offset)<2 else BAD)

        # 提示行
        if stale:
            self.v_hint.configure(text=tr('看不到机器人状态：检查 WiFi / SSH。'), fg=BAD)
        elif self.busy:
            self.v_hint.configure(text=self.busy, fg=WARN)
        elif phase in ('preparing','finalizing'):
            self.v_hint.configure(text=tr('启动请求正在处理，请勿重复点击') if phase=='preparing' else tr('正在收尾，完成后可开始下一轮'),fg=WARN)
        elif d.get('rec'):
            left = ''
            try:
                total = int(float(self.var_dur.get()))
                left = tr('，还剩 %s') % fmt_hms(total - d.get('elapsed', 0))
            except Exception:
                pass
            self.v_hint.configure(text=tr('录制中%s。到点会自动停。') % left, fg=OK)
        else:
            last=d.get('last_recording',{})
            reasons={'health_observer_failed':tr('监测心跳未及时更新，数据已保留'),
                     'stream_fault':tr('数据流检查失败，数据已保留'),'clock_jump':tr('录制时发生跳时，数据已保留')}
            hint=reasons.get(last.get('stop_reason')) if last.get('complete') is False else None
            self.v_hint.configure(text=hint or tr('机器人最近输出：%s') % (translate_log(d.get('log', '')) or '—'),fg=BAD if hint else DIM)
        self._refresh_buttons()

    def _refresh_buttons(self):
        state = tk.DISABLED if self.busy else tk.NORMAL
        for b in (self.btn_start, self.btn_stop, self.btn_pull, self.btn_delete):
            b.configure(state=state)
        active=self.state.get('phase') in ('preparing','recording','finalizing') or self.state.get('rec')
        if active:self.btn_start.configure(state=tk.DISABLED)
        for button in self.prepare_buttons:
            button.configure(state=tk.DISABLED if self.busy or active or not self.online else tk.NORMAL)

    # ------------------------------------------------------------ 键盘
    def _typing(self):
        w = self.root.focus_get()
        return isinstance(w, (tk.Entry, tk.Spinbox, tk.Text))

    def on_key(self, ev):
        if self._typing() or not self.var_arm.get():
            return
        m = {'w': 'fwd', 's': 'back', 'a': 'ccw', 'd': 'cw',
             'q': 'left', 'e': 'right'}
        k = (ev.keysym or '').lower()
        if k in m:
            self.teleop.key(m[k], True)
            return 'break'

    def on_key_up(self, ev):
        m = {'w': 'fwd', 's': 'back', 'a': 'ccw', 'd': 'cw',
             'q': 'left', 'e': 'right'}
        k = (ev.keysym or '').lower()
        if k in m:
            self.teleop.key(m[k], False)
            return 'break'

    def _mouse_key(self, key, down):
        if not self.var_arm.get():
            return
        self.teleop.key(key, down)

    def _on_focus_out(self, ev):
        if self.root.focus_displayof() is None:
            self._disarm(tr('窗口失焦'))

    def _disarm(self, why=''):
        if self.var_arm.get():
            self.var_arm.set(False)
            self.teleop.set_armed(False)
            self.teleop.clear_keys()
            if why:
                self._log(tr('已自动解除遥控：%s（已发零速）') % why)

    def on_arm(self):
        on = self.var_arm.get()
        self.teleop.set_armed(on)
        if on:
            if not self.teleop.ready:
                self._log(tr('遥控不可用：%s') % (self.teleop.error or tr('ROS 未连接')))
                self.var_arm.set(False)
                self.teleop.set_armed(False)
                return
            self._log(tr('遥控已启用：W/S 前后，A/D 转向，Q/E 平移，松手即停'))
            self.root.focus_force()
        else:
            self.teleop.clear_keys()
            self._log(tr('遥控已关闭（已发零速）'))

    def on_speed(self, _=None):
        v = float(self.var_spd.get())
        self.teleop.set_speed(v)
        self.lbl_spd.configure(text=tr('%.2f m/s  转向 %.2f rad/s') % (v, v * 3.0))

    # ------------------------------------------------------------ 急停
    def estop(self):
        """两条路同时断：本机发零 + 机器人本地发零。"""
        self._disarm(tr('急停'))
        self.teleop.clear_keys()
        self.teleop.zero_burst(10)
        self._log(tr('急停：已从控制机发零速，同时让机器人本地发零…'))

        cmd = (self.remote.rosenv() +
               ' timeout 2 rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '
               '"{linear: {x: 0.0, y: 0.0, z: 0.0}, '
               'angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1; echo ESTOP_OK')

        def work():
            try:
                out = subprocess.run(self.remote.ssh_argv() + [cmd],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     timeout=12).stdout.decode('utf-8', 'replace')
                if 'ESTOP_OK' in out:
                    self.q.put(('log', tr('急停：机器人本地零速已发完。')))
                else:
                    self.q.put(('log', tr('急停：机器人本地发零失败！请立刻断电或拎起车。')))
            except Exception as exc:
                self.q.put(('log', tr('急停：SSH 失败（%s）。请立刻断电或拎起车。') % exc))

        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------ 命令
    def _run_script(self, args, busy_text):
        if self.busy:
            self._log(tr('还在忙（%s），等它结束。') % self.busy)
            return
        self.busy = busy_text
        self._refresh_buttons()
        self._log(tr('开始：%s') % busy_text)

        def work():
            try:
                proc = subprocess.Popen(['bash', os.path.join(SCRIPT_DIR, 'remote_record.sh')] + args,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        universal_newlines=True, bufsize=1,
                                        env=dict(os.environ, ROBOT=self.remote.target, ROBOT_PASS=self.remote.password))
                for line in proc.stdout:
                    line = line.rstrip()
                    if line:
                        self.q.put(('log', line))
                proc.wait()
                if args[0]=='start' and proc.returncode==0:
                    self.q.put(('log',tr('启动请求已提交，正在准备数据源')))
                else:self.q.put(('log', tr('%s 结束（退出码 %d）') % (busy_text, proc.returncode)))
            except Exception as exc:
                self.q.put(('log', tr('%s 出错：%s') % (busy_text, exc)))
            finally:
                if args[0]=='prepare':self.q.put(('teleop_reconnect',None))
                self.q.put(('busy', None))

        threading.Thread(target=work, daemon=True).start()

    def on_prepare(self,action):
        if self.state.get('rec') or self.state.get('phase') in ('preparing','finalizing'):
            self._log(tr('录制或收尾期间不能校时和恢复驱动'))
            return
        self._disarm(tr('录前准备'));self.teleop.clear_keys()
        labels={'all':tr('一键准备'),'clock':tr('校准时间'),'camera':tr('启动/恢复相机'),'lidar':tr('启动/恢复雷达')}
        self._run_script(['prepare',action],labels[action])

    def on_start(self):
        try:
            dur = int(float(self.var_dur.get()))
        except Exception:
            messagebox.showerror(tr('时长不对'), tr('时长要填整数秒。'))
            return
        if dur <= 0 or dur > 1800:
            messagebox.showerror(tr('时长不对'), tr('时长请填 10 ~ 1800 秒。'))
            return
        out = self.var_dir.get().strip() or '/home/jetson/bags'
        raw = self.var_name.get().strip()
        name = session_label(raw)
        if raw and not name:
            messagebox.showerror(tr('会话名不可用'), tr('会话名里没有可用字符。'))
            return
        free = self.state.get('disk_free_gb', -1)
        need = dur * 30 / 1000.0
        if free > 0 and need > free - 1:
            if not messagebox.askyesno(
                    tr('空间可能不够'),
                    tr('预计需要 %.1f GB，机器人只剩 %.1f GB。\n还是要开始吗？')
                    % (need, free)):
                return
        args = start_args(dur, out, name)
        self._run_script(args, tr('启动数据源并录制 %d 秒') % dur)

    def _preview_name(self):
        """把清洗后的目录名当场给出来，别等录完才发现名字被改了。"""
        raw = self.var_name.get().strip()
        label = session_label(raw)
        if not raw:
            self.lbl_name.configure(text=tr('目录名：session_<时间戳>'), fg=DIM)
        elif not label:
            self.lbl_name.configure(text=tr('这个名字里没有可用字符，会退回纯时间戳'), fg=BAD)
        else:
            self.lbl_name.configure(text=tr('目录名：session_<时间戳>_%s') % label, fg=DIM)

    def on_stop(self):
        self._run_script(['stop'], tr('优雅停止录制'))

    def on_pull(self):
        self._pick_and_run('pull')

    def on_delete_remote(self):
        self._pick_and_run('delete')

    def _base_args(self):
        return ['pull',
                '-r', self.var_dir.get().strip() or '/home/jetson/bags',
                '-l', os.path.expanduser(self.var_local.get().strip())]

    def _pick_and_run(self, mode):
        """List first, then let the operator pick.

        Acting on a whole folder blind is how a `-d` run removed bags nobody
        meant to touch; every pull and every delete now goes through the picker.
        """
        if not self.var_local.get().strip():
            messagebox.showerror(tr('回传目录为空'), tr('请填写控制机上的目标目录。'))
            return
        if self.busy:
            self._log(tr('还在忙（%s），等它结束。') % self.busy)
            return
        self.q.put(('busy', tr('读取机器人上的包清单')))
        self.q.put(('log', tr('读取机器人上的包清单…')))
        args = self._base_args() + ['--list-json']

        def work():
            try:
                proc = subprocess.run(
                    ['bash', os.path.join(SCRIPT_DIR, 'remote_record.sh')] + args,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True, timeout=300,
                    env=dict(os.environ, ROBOT=self.remote.target,
                             ROBOT_PASS=self.remote.password))
                if proc.returncode != 0:
                    raise RuntimeError((proc.stderr or proc.stdout or '').strip()[-400:])
                data = json.loads(proc.stdout)
            except Exception as exc:
                self.q.put(('log', tr('读取清单失败：%s') % exc))
                self.q.put(('busy', None))
                return
            self.q.put(('listing', (mode, data)))

        threading.Thread(target=work, daemon=True).start()

    def _open_picker(self, payload):
        mode, data = payload
        self.busy = None
        self._refresh_buttons()
        rows = data.get('files') or []
        if not rows:
            messagebox.showinfo(tr('没有可选的包'), tr('源目录里没有已收尾的 bag。'))
            return
        SelectDialog(self.root, rows, mode, data,
                     lambda names: self._start_transfer(mode, names))

    def _start_transfer(self, mode, names):
        if not names:
            return
        args = self._base_args() + ['--names'] + names
        if mode == 'delete':
            args.append('--delete-only')
            self._run_script(args, tr('删除机器人上的 %d 个包（先核对本地副本）') % len(names))
        else:
            self._run_script(args, tr('拉回 %d 个包并校验 SHA-256') % len(names))

    # ------------------------------------------------------------ 退出
    def on_close(self):
        self.closing = True
        try:
            self._disarm(tr('关闭窗口'))
            self.teleop.shutdown()
        except Exception:
            pass
        self.root.after(50, self.root.destroy)


def main():
    ap = argparse.ArgumentParser(description=tr('Transbot 录制控制台'))
    ap.add_argument('--robot', default=os.environ.get('ROBOT', 'jetson@192.168.1.11'))
    ap.add_argument('--password', default=os.environ.get('ROBOT_PASS', 'yahboom'))
    args = ap.parse_args()

    remote = Remote(args.robot, args.password)
    root = tk.Tk()
    app = App(root, remote)
    root.mainloop()


if __name__ == '__main__':
    main()
