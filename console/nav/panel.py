# -*- coding: utf-8 -*-
"""The map / navigation tab: one mode selector, three behaviours.

    mapping     drive with the keyboard, watch the map grow, save it
    navigation  pick a saved map, click a goal, or drive by hand
    map+nav     gmapping and move_base on the same live map

Teleop and goal-following both write /cmd_vel, so they are mutually
exclusive: turning teleop on cancels the goal, sending a goal turns teleop
off. Nobody has to remember to switch the other one off.

This file is the view only. The stack, the subscriptions and the live map
live in model.NavState so that a language switch -- which destroys and
rebuilds every widget -- does not orphan a running stack.
"""

import math
import os
import re
import subprocess
import sys
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

from .stack import MODE_MAP, MODE_NAV, MODE_SLAMNAV

AUTO_NAME = re.compile(r'^lab_\d{8}_\d{6}$')

STATUS_NAMES = {0: 'pending', 1: 'active', 2: 'preempted', 3: 'succeeded',
                4: 'aborted', 5: 'rejected', 6: 'preempting', 7: 'recalling',
                8: 'recalled', 9: 'lost'}

KEYMAP = {'w': (1, 0), 'up': (1, 0),
          's': (-1, 0), 'down': (-1, 0),
          'a': (0, 1), 'left': (0, 1),
          'd': (0, -1), 'right': (0, -1)}


class MapPanel(ttk.Frame):
    LIN = 0.12          # m/s for keyboard teleop
    ANG = 0.60          # rad/s
    RENDER_HZ = 8.0

    def __init__(self, master, app, L):
        ttk.Frame.__init__(self, master)
        self.app = app
        self.L = L
        self.nav = app.nav
        self.nav.set_log(self._stack_log)

        self.mode_var = tk.StringVar(value=self.nav.mode)
        self.name_var = tk.StringVar(value=self.nav.name)
        self.map_var = tk.StringVar(value=self.nav.map_name)
        self.status_var = tk.StringVar(
            value=self._running_status() if self.nav.stack.running()
            else L('● 未启动'))
        self.info_var = tk.StringVar(value='')
        self.teleop_var = tk.BooleanVar(value=self.nav.teleop)
        self.view_var = tk.StringVar(value='1.0x  +0°')
        self.pose_mode_var = tk.BooleanVar(value=False)
        self.goal_mode_var = tk.BooleanVar(value=False)
        self.cam_var = tk.BooleanVar(value=False)
        self._cam_photo = None
        pose0 = self.nav.pose
        self.px_var = tk.StringVar(value='%.2f' % pose0[0] if pose0 else '0.00')
        self.py_var = tk.StringVar(value='%.2f' % pose0[1] if pose0 else '0.00')
        self.pth_var = tk.StringVar(
            value='%.0f' % math.degrees(pose0[2]) if pose0 else '0')

        self._photo = None
        self._drag = None
        self._press = None
        self._pose_click = None
        self._goal_click = None
        self._busy = False
        self._last_render = 0.0
        self._last_sync = 0.0
        self._low_volt_warned = False
        self._sigma_warned = False
        self._rot_start = None
        self._cam_deadline = 0.0
        self._cam_hinted = False
        # X11 自动重复会成对发 press/release，松键后延迟一点再真停，
        # 否则按住方向键时车每 33 ms 停一次（一冲一顿）。
        self._rel_timers = {}       # key -> after id
        self.KEY_RELEASE_DELAY = 150

        self._build()
        self.refresh_maps(silent=True)
        self._sync()

    # ------------------------------------------------------------------
    # widgets
    # ------------------------------------------------------------------
    def _build(self):
        L = self.L
        # 五排控件 + 两行说明。窗口最小 1000 px，长文案一律另起一行。
        r1 = ttk.Frame(self)
        r1.pack(fill='x', padx=12, pady=(10, 4))
        ttk.Label(r1, text=L('运行模式'), style='H.TLabel').pack(side='left',
                                                            padx=(0, 10))
        for mode, text in ((MODE_MAP, L('纯建图')),
                           (MODE_NAV, L('纯导航')),
                           (MODE_SLAMNAV, L('边建图边导航'))):
            ttk.Radiobutton(r1, text=text, value=mode, variable=self.mode_var,
                            command=self._on_mode).pack(side='left',
                                                        padx=(0, 12))
        self.btn_start = ttk.Button(r1, text=L('启动'), style='Green.TButton',
                                    command=self.start)
        self.btn_start.pack(side='left', padx=4)
        self.btn_stop = ttk.Button(r1, text=L('停止'), style='Red.TButton',
                                   command=self.stop)
        self.btn_stop.pack(side='left', padx=4)
        ttk.Label(r1, textvariable=self.status_var).pack(side='left', padx=12)

        # 保存用 / 使用中 是两个不同的东西，标签必须分开写，否则一定有人
        # 在导航模式下改上面那个输入框然后奇怪为什么没生效。
        r2 = ttk.Frame(self)
        r2.pack(fill='x', padx=12, pady=(0, 4))
        self.lbl_save = ttk.Label(r2, text=L('保存为'))
        self.lbl_save.pack(side='left')
        self.ent_name = ttk.Entry(r2, textvariable=self.name_var, width=22)
        self.ent_name.pack(side='left', padx=(4, 8))
        self.ent_name.bind('<KeyRelease>', lambda e: self._update_name_warn())
        self.btn_save = ttk.Button(r2, text=L('保存地图'),
                                   style='Green.TButton', command=self.save)
        self.btn_save.pack(side='left', padx=4)
        self.btn_rename = ttk.Button(r2, text=L('改名'), command=self.rename)
        self.btn_rename.pack(side='left', padx=4)
        self.name_warn = ttk.Label(r2, text='', foreground='#e8a33d')
        self.name_warn.pack(side='left', padx=8)

        r3 = ttk.Frame(self)
        r3.pack(fill='x', padx=12, pady=(0, 4))
        self.lbl_map = ttk.Label(r3, text=L('使用地图（仅导航）'))
        self.lbl_map.pack(side='left')
        self.combo = ttk.Combobox(r3, textvariable=self.map_var, width=22,
                                  state='readonly', values=self.nav.maps)
        self.combo.pack(side='left', padx=(4, 8))
        self.combo.bind('<<ComboboxSelected>>', lambda e: self._on_pick_map())
        self.btn_refresh = ttk.Button(r3, text=L('刷新列表'),
                                      command=lambda: self.refresh_maps())
        self.btn_refresh.pack(side='left', padx=4)
        ttk.Checkbutton(r3, text=L('设初始位姿'),
                        variable=self.pose_mode_var).pack(side='left',
                                                          padx=(16, 0))
        ttk.Checkbutton(r3, text=L('指定目标朝向'),
                        variable=self.goal_mode_var).pack(side='left',
                                                          padx=(12, 0))

        # 位姿：可以手打坐标，也可以读当前、或者让雷达在全图上重新找一次
        r4 = ttk.Frame(self)
        r4.pack(fill='x', padx=12, pady=(0, 6))
        ttk.Label(r4, text=L('位姿')).pack(side='left')
        ttk.Label(r4, text='X').pack(side='left', padx=(6, 2))
        ttk.Entry(r4, textvariable=self.px_var, width=7).pack(side='left')
        ttk.Label(r4, text='Y').pack(side='left', padx=(6, 2))
        ttk.Entry(r4, textvariable=self.py_var, width=7).pack(side='left')
        ttk.Label(r4, text='θ°').pack(side='left', padx=(6, 2))
        ttk.Entry(r4, textvariable=self.pth_var, width=7).pack(side='left')
        self.btn_readpose = ttk.Button(r4, text=L('读当前'),
                                       command=self.read_pose)
        self.btn_readpose.pack(side='left', padx=(10, 4))
        self.btn_sendpose = ttk.Button(r4, text=L('发送位姿'),
                                       command=self.send_pose)
        self.btn_sendpose.pack(side='left', padx=4)
        self.btn_reloc = ttk.Button(r4, text=L('重新定位'),
                                    command=self.relocalize)
        self.btn_reloc.pack(side='left', padx=4)

        # 地图占左边，右边一列给相机和视角控制：地图横条太长的时候，右边
        # 塞一块 320x240 的画面正好把它压短，顺便省掉来回切页签。
        mid = ttk.Frame(self)
        mid.pack(fill='both', expand=True, padx=12, pady=(0, 4))
        self.canvas = tk.Canvas(mid, bg='#171a1f', highlightthickness=0, bd=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self._img_id = self.canvas.create_image(0, 0, anchor='nw')
        self.canvas.bind('<Configure>', self._on_resize)
        self.canvas.bind('<Button-1>', self._on_press)
        self.canvas.bind('<B1-Motion>', self._on_drag)
        self.canvas.bind('<ButtonRelease-1>', self._on_release)
        self.canvas.bind('<Button-3>', self._on_right)
        self.canvas.bind('<Double-Button-1>', self._on_double)
        self.canvas.bind('<MouseWheel>', self._on_wheel)
        self.canvas.bind('<Button-4>', lambda e: self._zoom(e, 1.2))
        self.canvas.bind('<Button-5>', lambda e: self._zoom(e, 1 / 1.2))
        # Shift + 拖动 = 连续旋转（按住地图"拧"），Shift + 滚轮 = 每格 5°
        self.canvas.bind('<Shift-Button-1>', self._rot_press)
        self.canvas.bind('<Shift-B1-Motion>', self._rot_drag)
        self.canvas.bind('<Shift-ButtonRelease-1>', self._rot_release)
        self.canvas.bind('<Shift-MouseWheel>', self._on_wheel_rot)
        self.canvas.bind('<Shift-Button-4>', lambda e: self._rot_step(5))
        self.canvas.bind('<Shift-Button-5>', lambda e: self._rot_step(-5))

        side = ttk.Frame(mid)
        side.pack(side='right', fill='y', padx=(8, 0))
        cam_box = tk.Frame(side, width=320, height=160, bg='#101318')
        cam_box.pack()
        cam_box.pack_propagate(False)
        self.cam_lb = tk.Label(cam_box, bg='#101318', fg='#6b7480',
                               text=L('相机未开启'), font=('Ubuntu', 10))
        self.cam_lb.pack(fill='both', expand=True)
        ttk.Checkbutton(side, text=L('相机预览'), variable=self.cam_var,
                        command=self._on_cam).pack(anchor='w', pady=(6, 2))
        # 视角用鼠标+键盘操作，不摆一排定值按钮：Shift+拖动连续旋转、
        # 滚轮缩放、[ ] 旋转、+ - 缩放、双击复位。
        vr = ttk.Frame(side)
        vr.pack(fill='x', pady=(2, 0))
        ttk.Label(vr, textvariable=self.view_var,
                  foreground='#9aa5b1').pack(side='left')
        ttk.Button(vr, text=L('复位'),
                   command=self._reset_view).pack(side='right')

        ttk.Label(self, textvariable=self.info_var,
                  foreground='#9aa5b1').pack(fill='x', padx=14)
        ttk.Label(self, foreground='#7d8794', wraplength=1300, justify='left',
                  text=L('拖动 = 平移　滚轮 = 缩放　Shift+拖动 = 旋转　'
                         '[ ] 旋转　+ - 缩放　双击 = 复位')).pack(fill='x', padx=14)
        ttk.Label(self, foreground='#7d8794', wraplength=1300, justify='left',
                  text=L('「设初始位姿」点两下：位置→朝向；'
                         '「重新定位」重找位置')).pack(
                             fill='x', padx=14, pady=(0, 2))

        r5 = ttk.Frame(self)
        r5.pack(fill='x', padx=12, pady=(0, 10))
        ttk.Checkbutton(r5, text=L('键盘遥控'),
                        variable=self.teleop_var,
                        command=self._on_teleop).pack(side='left')
        ttk.Label(r5, text=L('W/S 前后，A/D 转向，空格停'),
                  foreground='#9aa5b1').pack(side='left', padx=10)
        ttk.Button(r5, text=L('紧急停止'), style='Red.TButton',
                   command=self.emergency).pack(side='right')

    # ------------------------------------------------------------------
    # plumbing
    # ------------------------------------------------------------------
    def _log(self, msg):
        try:
            self.app._log(msg)
        except Exception:
            pass

    def _stack_log(self, msg):
        """stack.py talks in `tag| text`; keep that readable in the log."""
        self._log(msg)

    def _ui(self, fn):
        try:
            self.app._ui(fn)
        except Exception:
            pass

    def _on_mode(self):
        self.nav.mode = self.mode_var.get()
        self._update_name_warn()
        self._sync()

    def _on_pick_map(self):
        name = self.map_var.get().strip()
        self.nav.map_name = name
        if name:
            meta = self.nav.stack.map_meta(name)
            bits = []
            if meta.get('area_m2'):
                bits.append('%.1f m²' % meta['area_m2'])
            if meta.get('coverage_pct'):
                bits.append(self.L('覆盖率') + ' %.0f%%' % meta['coverage_pct'])
            if meta.get('size_m'):
                bits.append('%.1f × %.1f m' % tuple(meta['size_m']))
            if meta.get('saved'):
                bits.append(meta['saved'])
            self.info_var.set(' · '.join(bits))
        self._sync()

    def _sync(self):
        mode = self.mode_var.get()
        running = self.nav.stack.running()
        connected = bool(getattr(self.app.robot, 'connected', lambda: False)())
        # 「保存为」整行只属于建图：纯导航时没有东西可存，改名也不该在这里做
        # ——map_server 已经按旧路径加载了地图，改文件名等于把下一启动搞坏。
        build_mode = mode in (MODE_MAP, MODE_SLAMNAV)
        try:
            self.btn_start.configure(
                state='normal' if (connected and not running and not self._busy
                                   and (mode != MODE_NAV
                                        or self.map_var.get().strip()))
                else 'disabled')
            self.btn_stop.configure(
                state='normal' if (running and not self._busy) else 'disabled')
            self.btn_save.configure(
                state='normal' if (running and build_mode and not self._busy)
                else 'disabled')
            self.btn_rename.configure(
                state='normal' if (build_mode and not running and not self._busy)
                else 'disabled')
            self.ent_name.configure(
                state='normal' if build_mode else 'disabled')
            self.lbl_save.configure(
                foreground='#d8dee9' if build_mode else '#5f6b78')
            self.combo.configure(
                state='readonly' if mode == MODE_NAV else 'disabled')
            self.lbl_map.configure(
                foreground='#d8dee9' if mode == MODE_NAV else '#5f6b78')
            self.btn_readpose.configure(
                state='normal' if connected else 'disabled')
            self.btn_sendpose.configure(
                state='normal' if (connected and self.nav.subs_done)
                else 'disabled')
            self.btn_reloc.configure(
                state='normal' if (connected and not self._busy
                                   and mode == MODE_NAV
                                   and self.map_var.get().strip()
                                   and self.nav.stack.running())
                else 'disabled')
        except Exception:
            pass

    def _set_busy(self, busy, status=None):
        """Start/stop 的收尾在后台线程里跑，Tk 只能在主线程碰。"""
        self._busy = busy

        def _do():
            if status is not None:
                self.status_var.set(status)
            self._sync()
        self._ui(_do)

    def _running_status(self):
        return {MODE_MAP: self.L('● 建图中'),
                MODE_NAV: self.L('● 导航中'),
                MODE_SLAMNAV: self.L('● 边建图边导航')}.get(
                    self.nav.stack.mode, self.L('● 未启动'))

    # ------------------------------------------------------------------
    # start / stop
    # ------------------------------------------------------------------
    def start(self):
        if self._busy:
            return
        if not getattr(self.app.robot, 'connected', lambda: False)():
            self._log(self.L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        mode = self.mode_var.get()
        name = self.map_var.get().strip() if mode == MODE_NAV \
            else (self.name_var.get().strip() or self.nav.stack.default_name())
        if mode != MODE_NAV:
            # 同一个窗口里建第二张图时，自动名字会还是上一次的时间戳，
            # 保存时就会盖掉上一张。撞名就换一个新的时间戳。
            if AUTO_NAME.match(name) and self.nav.stack.map_exists(name):
                name = self.nav.stack.default_name()
            self.name_var.set(name)
            self.nav.name = name
        pose = self.nav.pose or (0.0, 0.0, 0.0)
        self.nav.stack.host = type(self.nav.stack)._host_from_env()
        self.nav.reset_view_state()
        self._set_busy(True, self.L('● 正在启动…'))

        def work():
            try:
                self.nav.stack.start(
                    mode, map_name=name if mode == MODE_NAV else None, pose=pose)
                self._set_busy(False, self._running_status())
                if mode != MODE_NAV:
                    # gmapping 没有"接着旧图建"这回事，每次都是空图。把这句话
                    # 写在日志里，省得有人以为下拉框里选的那张图会参与建图。
                    self._log(self.L('○ 从空地图开始建，不会加载或修改已有地图'))
                    self._log(self.L('○ 地图保存名：%s（可随时改名）') % name)
                else:
                    self._log(self.L('○ 地图 %s；定位不准就在地图上点两下'
                                     '设初始位姿') % name)
            except Exception as e:
                self._log(self.L('✗ 启动失败：%s') % e)
                self._set_busy(False, self.L('● 未启动'))
        threading.Thread(target=work, daemon=True).start()

    def stop(self):
        if self._busy:
            return
        self._set_busy(True, self.L('● 正在停止…'))

        def work():
            try:
                self.emergency()
                self.nav.stack.stop()
                self._log(self.L('✓ 已停止'))
            except Exception as e:
                self._log(self.L('✗ 停止失败：%s') % e)
            finally:
                self._set_busy(False, self.L('● 未启动'))
        threading.Thread(target=work, daemon=True).start()

    def emergency(self):
        try:
            self.app.robot.stop_manual()
        except Exception:
            pass
        self.nav.keys.clear()
        self.nav.last_cmd = (0.0, 0.0)

    # ------------------------------------------------------------------
    # maps
    # ------------------------------------------------------------------
    def refresh_maps(self, silent=False):
        cur = self.map_var.get().strip()      # Tk 只能在主线程读

        def work():
            try:
                names = self.nav.stack.list_maps()
                self.nav.maps = names
                self._ui(lambda: self.combo.configure(values=names))
                if (not cur or cur not in names) and names:
                    self._ui(lambda n=names[-1]: self.map_var.set(n))
                    self._ui(self._on_pick_map)
                if not silent:
                    self._log(self.L('○ 地图列表：%s')
                              % (', '.join(names) if names
                                 else self.L('（空）')))
                self._ui(self._update_name_warn)
                self._ui(self._sync)
            except Exception as e:
                self._log(self.L('✗ 读取地图列表失败：%s') % e)
        threading.Thread(target=work, daemon=True).start()

    def save(self):
        name = self.name_var.get().strip()
        if not name:
            self._log(self.L('○ 请先输入地图名'))
            return
        if self.nav.stack.map_exists(name):
            if not messagebox.askyesno(
                    self.L('地图已存在'),
                    self.L('已有同名地图 %s，覆盖它？') % name):
                return
        stats = self.nav.livemap.stats()
        pose = self.nav.pose
        odom_m = self.nav.odom_m
        self._log(self.L('→ 正在保存地图 %s …') % name)

        def work():
            try:
                self.nav.stack.save_map(name, stats=stats, pose=pose,
                                        odom_m=odom_m)
                self._log(self.L('✓ 地图已保存：maps/%s.yaml') % name)
                self._ui(self.refresh_maps)
            except Exception as e:
                self._log(self.L('✗ 保存失败：%s') % e)
        threading.Thread(target=work, daemon=True).start()

    def rename(self):
        old = self.name_var.get().strip()
        if not old:
            self._log(self.L('○ 请先输入地图名'))
            return
        if not self.nav.stack.map_exists(old):
            self._log(self.L('○ 没有叫 %s 的地图，先保存一次') % old)
            return
        new = simpledialog.askstring(self.L('改名'),
                                     self.L('新地图名（%s）') % old,
                                     initialvalue=old, parent=self)
        if not new or new.strip() == old:
            return
        try:
            self.nav.stack.rename_map(old, new.strip())
            self.name_var.set(new.strip())
            self.nav.name = new.strip()
            self._log(self.L('✓ 已改名：%s → %s') % (old, new.strip()))
            self.refresh_maps(silent=True)
        except Exception as e:
            self._log(self.L('✗ 改名失败：%s') % e)

    # ------------------------------------------------------------------
    # pose: read / type / relocalise
    # ------------------------------------------------------------------
    def _update_name_warn(self):
        """同名提示：输入框里打的名字已经存在就立刻显示，不用等到按保存。"""
        try:
            name = self.name_var.get().strip()
            if (self.mode_var.get() != MODE_NAV and name
                    and self.nav.stack.map_exists(name)):
                self.name_warn.configure(
                    text=self.L('⚠ 已有同名地图，保存会覆盖'))
            else:
                self.name_warn.configure(text='')
        except Exception:
            pass

    def read_pose(self):
        """把当前位姿填进输入框，方便微调后再发回去。"""
        pose = self.nav.pose or self.nav.amcl_pose
        if pose is None:
            self._log(self.L('○ 还没有位姿（等 TF 或 /amcl_pose）'))
            return
        self.px_var.set('%.2f' % pose[0])
        self.py_var.set('%.2f' % pose[1])
        self.pth_var.set('%.0f' % math.degrees(pose[2]))
        self._log(self.L('○ 已读当前位姿 %.2f, %.2f, %.0f°')
                  % (pose[0], pose[1], math.degrees(pose[2])))

    def send_pose(self):
        """按输入框的坐标下发 /initialpose（自定义位置）。"""
        try:
            x = float(self.px_var.get())
            y = float(self.py_var.get())
            th = math.radians(float(self.pth_var.get()))
        except ValueError:
            self._log(self.L('✗ 位姿要填数字：X / Y 用米，θ 用度'))
            return
        self._apply_pose(x, y, th, self.L('手动'))

    def _apply_pose(self, x, y, yaw, source):
        if self.nav.pub_pose is None:
            self._log(self.L('✗ 还没连上 ROS，位姿发不出去'))
            return
        try:
            self.nav.set_initial_pose(x, y, yaw)
        except Exception as e:
            self._log(self.L('✗ 位姿下发失败：%s') % e)
            return
        self.px_var.set('%.2f' % x)
        self.py_var.set('%.2f' % y)
        self.pth_var.set('%.0f' % math.degrees(yaw))
        self._log(self.L('✓ 初始位姿 (%.2f, %.2f, %.0f°) 已下发（%s）')
                  % (x, y, math.degrees(yaw), source))

    def relocalize(self):
        """雷达在全图上重新找一次位置，解决"搬动后/定位跳飞"的情况。

        AMCL 没法从"被搬走两米"里恢复——粒子全在旧位置。这里用似然场在整张
        地图上搜一遍，结果作为初始位姿发回去。搜索在长方形房间里可能有两个
        相差 180° 的解，所以同时把 margin 报出来，低于 0.10 就明说别信。
        """
        if self._busy:
            return
        mode = self.mode_var.get()
        name = self.map_var.get().strip()
        if mode != MODE_NAV or not name:
            self._log(self.L('○ 重新定位要在纯导航模式下、并且选好地图'))
            return
        helper = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'helpers', 'relocalize.py')
        if not os.path.isfile(helper):
            self._log(self.L('✗ 找不到 relocalize.py'))
            return
        if self.nav.scan is None:
            self._log(self.L('✗ 还没有 /scan，等雷达出数据再试'))
            return
        self._set_busy(True, self.L('● 正在重新定位…'))
        self._log(self.L('→ 重新定位中，约 30-90 s（全图似然场搜索）'))

        def work():
            try:
                out = subprocess.run(
                    [sys.executable, helper, '--map', self.nav.stack.map_path(name),
                     '--machine'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    universal_newlines=True, timeout=240).stdout.strip()
                parts = out.splitlines()[-1].split()
                if len(parts) != 4:
                    raise ValueError('unexpected output %r' % out[-200:])
                x, y, yaw, margin = [float(v) for v in parts]
                self._ui(lambda: self._apply_pose(
                    x, y, yaw, self.L('重新定位')))
                if margin < 0.10:
                    self._log(self.L('⚠ 似然只领先第二名 %.3f，可能有 180° '
                                     '孪生解，建议改用「设初始位姿」点两下')
                              % margin)
                else:
                    self._log(self.L('○ 似然领先 %.3f，解唯一') % margin)
            except Exception as e:
                self._log(self.L('✗ 重新定位失败：%s') % e)
            finally:
                self._set_busy(False, self._running_status()
                               if self.nav.stack.running()
                               else self.L('● 未启动'))
        threading.Thread(target=work, daemon=True).start()

    # ------------------------------------------------------------------
    # rendering
    # ------------------------------------------------------------------
    def _on_resize(self, _ev):
        self.nav.livemap.need_fit = True
        self._render(force=True)

    def _render(self, force=False):
        if not self.winfo_ismapped():
            return
        now = time.time()
        if not force and now - self._last_render < 1.0 / self.RENDER_HZ:
            return
        self._last_render = now
        cw = max(20, self.canvas.winfo_width())
        ch = max(20, self.canvas.winfo_height())
        pose = self.nav.push_trail()
        try:
            photo = self.nav.livemap.render(cw, ch, scan=self.nav.scan_points(),
                                            pose=pose, trail=self.nav.trail,
                                            plan=self.nav.plan,
                                            goal=self.nav.goal)
        except Exception:
            photo = None
        if photo is not None and photo is not self._photo:
            self._photo = photo
            self.canvas.itemconfigure(self._img_id, image=photo)
        self._redraw_info(pose)

    def _redraw_info(self, pose):
        bits = []
        if pose is not None:
            bits.append(self.L('位姿') + ' %.2f, %.2f, %.0f°'
                        % (pose[0], pose[1], math.degrees(pose[2])))
        sig = self.nav.sigma
        if sig is not None:
            bits.append('σ %.2f/%.2f m %.1f°' % sig)
            if sig[0] > 0.20 or sig[1] > 0.20:
                self._warn_localisation(sig)
            elif sig[0] < 0.12 and sig[1] < 0.12:
                self._sigma_warned = False      # 收敛回来了，允许再次报警
        st = self.nav.livemap.stats()
        if st:
            bits.append(self.L('已知') + ' %.1f m²' % st['area'])
            bits.append(self.L('覆盖率') + ' %.0f%%' % st['cover'])
        if self.nav.goal is not None:
            bits.append(self.L('目标') + ' %.2f, %.2f'
                        % (self.nav.goal[0], self.nav.goal[1]))
        if self.nav.goal_state is not None:
            bits.append('move_base: '
                        + STATUS_NAMES.get(self.nav.goal_state[0], '?'))
        if self.nav.voltage is not None:
            bits.append('%.1f V' % self.nav.voltage)
            if self.nav.voltage < 11.0 and not self._low_volt_warned:
                self._low_volt_warned = True
                self._log(self.L('⚠ 电池 %.1f V，低于 11 V 时底盘只出 25%% 的速度，'
                                 '看着像"按了没反应"') % self.nav.voltage)
        self.info_var.set(' · '.join(bits))
        self.view_var.set('%.1fx  %+.0f°' % (self.nav.livemap.zoom,
                                             self.nav.livemap.rotation_deg()))

    def _warn_localisation(self, sig):
        """AMCL 自己报的协方差涨了：定位大概率已经飘了，只在跨越阈值时报一次。"""
        if self._sigma_warned:
            return
        self._sigma_warned = True
        self._log(self.L('⚠ 定位不确定度 σ=%.2f/%.2f m 偏大：'
                         '先用「重新定位」，或用「设初始位姿」点两下')
                  % (sig[0], sig[1]))

    # ------------------------------------------------------------------
    # mouse
    # ------------------------------------------------------------------
    def _on_press(self, ev):
        if ev.state & 0x1:          # Shift：旋转手势，不是平移
            return
        self._press = (ev.x, ev.y)
        self._drag = (ev.x, ev.y)

    def _on_drag(self, ev):
        if ev.state & 0x1 or self._drag is None:
            return
        dx = ev.x - self._drag[0]
        dy = ev.y - self._drag[1]
        self._drag = (ev.x, ev.y)
        self.nav.livemap.pan(dx, dy)
        self._render(force=True)

    def _on_release(self, ev):
        self._drag = None
        press, self._press = self._press, None
        if press is None:
            return
        if math.hypot(ev.x - press[0], ev.y - press[1]) > 6:
            return                      # that was a pan, not a click
        self._click(ev.x, ev.y)

    def _on_right(self, _ev):
        self.cancel_goal()

    def _on_double(self, _ev):
        self.nav.livemap.reset_view()
        self._render(force=True)

    def _on_wheel(self, ev):
        self._zoom(ev, 1.2 if ev.delta > 0 else 1 / 1.2)

    def _zoom(self, ev, factor):
        self.nav.livemap.zoom_at(ev.x, ev.y, factor)
        self._render(force=True)

    def _on_wheel_rot(self, ev):
        self._rot_step(5 if ev.delta > 0 else -5)

    def _rot_press(self, ev):
        """Shift + 拖动：绕视图中心连续旋转。"""
        cx = self.canvas.winfo_width() / 2.0
        cy = self.canvas.winfo_height() / 2.0
        self._rot_start = (math.atan2(ev.y - cy, ev.x - cx),
                           self.nav.livemap.rotation_deg())
        self.canvas.configure(cursor='exchange')
        return 'break'

    def _rot_drag(self, ev):
        if self._rot_start is None:
            return 'break'
        cx = self.canvas.winfo_width() / 2.0
        cy = self.canvas.winfo_height() / 2.0
        if math.hypot(ev.x - cx, ev.y - cy) < 30:
            return 'break'          # 太靠近中心，角度抖得厉害
        a0, r0 = self._rot_start
        deg = math.degrees(math.atan2(ev.y - cy, ev.x - cx) - a0)
        self.nav.livemap.set_rotation(r0 + deg)
        self._render(force=True)
        return 'break'

    def _rot_release(self, _ev):
        self._rot_start = None
        self.canvas.configure(cursor='')
        return 'break'

    VIEW_KEYS = {'bracketleft': ('rot', -5.0), 'bracketright': ('rot', 5.0),
                 'equal': ('zoom', 1.25), 'plus': ('zoom', 1.25),
                 'kp_add': ('zoom', 1.25), 'minus': ('zoom', 0.8),
                 'kp_subtract': ('zoom', 0.8), '0': ('reset', 0.0)}

    def _view_key(self, ev):
        if not self.winfo_ismapped():
            return False
        try:
            focus = self.app.root.focus_get()
        except Exception:
            focus = None
        if isinstance(focus, (tk.Entry, tk.Text, ttk.Entry, ttk.Combobox)):
            return False
        act = self.VIEW_KEYS.get(ev.keysym.lower())
        if act is None:
            return False
        kind, val = act
        if kind == 'rot':
            self._rot_step(val)
        elif kind == 'zoom':
            self._zoom_step(val)
        else:
            self._reset_view()
        return True

    def _zoom_step(self, factor):
        lm = self.nav.livemap
        lm.zoom_at(max(20, self.canvas.winfo_width()) / 2.0,
                   max(20, self.canvas.winfo_height()) / 2.0, factor)
        self._render(force=True)

    def _rot_step(self, deg):
        lm = self.nav.livemap
        lm.set_rotation(lm.rotation_deg() + deg)
        self._render(force=True)

    def _reset_view(self):
        self.nav.livemap.reset_view()
        self._render(force=True)

    def _on_cam(self):
        """相机预览直接用视觉页那套采集，不另起一条订阅。"""
        if self.cam_var.get():
            self.app._cam_open()
            self._cam_deadline = time.time() + 5.0
            self._cam_hinted = False
        else:
            self.app._cam_close()
            self._cam_photo = None
            self.cam_lb.configure(image='', text=self.L('相机未开启'))

    def _click(self, px, py):
        L = self.L
        if self.nav.livemap.msg is None:
            self._log(L('○ 还没有地图数据'))
            return
        wx, wy = self.nav.livemap.canvas_to_world(px, py)
        if self.pose_mode_var.get():
            # 点在地图外的灰区会得到 (-4.77, -10.21) 这种坐标，发下去 AMCL 会
            # 直接把粒子撒到墙外。位姿只允许落在已知地图里。
            ok, why = self.nav.livemap.clear(wx, wy, 0.0)
            if not ok:
                self._log(L('✗ 这一点不在已知地图内（%s），请点在地图上；'
                            '要精确坐标就直接填 X/Y/θ') % why)
                return
            if self._pose_click is None:
                self._pose_click = (wx, wy)
                self._log(L('○ 初始位置 (%.2f, %.2f)，再点一下定朝向')
                          % (wx, wy))
                return
            yaw = math.atan2(wy - self._pose_click[1], wx - self._pose_click[0])
            self._apply_pose(self._pose_click[0], self._pose_click[1], yaw,
                             L('点选'))
            self._pose_click = None
            return
        if self.nav.stack.mode not in (MODE_NAV, MODE_SLAMNAV):
            self._log(L('○ 当前模式不接收导航目标'))
            return
        ok, why = self.nav.livemap.clear(wx, wy, 0.30)
        if not ok:
            self._log(L('✗ 目标点不可用（%s），换一个已知空闲区域') % why)
            return
        # 指定朝向：第一下定位置，第二下用"从目标点指向第二下"的方向当朝向。
        # 贴墙那类任务里终点朝向才是关键，默认的"从车指向目标"经常是斜的。
        if self.goal_mode_var.get() and self._goal_click is None:
            self._goal_click = (wx, wy)
            self._log(L('○ 目标位置 (%.2f, %.2f)，再点一下定朝向')
                      % (wx, wy))
            return
        if self.goal_mode_var.get() and self._goal_click is not None:
            gx, gy = self._goal_click
            yaw = math.atan2(wy - gy, wx - gx)
            self._goal_click = None
            self._send_goal(gx, gy, yaw)
            return
        yaw = 0.0
        if self.nav.pose is not None:
            yaw = math.atan2(wy - self.nav.pose[1], wx - self.nav.pose[0])
        self._send_goal(wx, wy, yaw)

    def _send_goal(self, x, y, yaw):
        if self.teleop_var.get():
            self.teleop_var.set(False)
            self._on_teleop()
        self.nav.send_goal(x, y, yaw)
        self._log(self.L('→ 目标 (%.2f, %.2f) %.0f°')
                  % (x, y, math.degrees(yaw)))

    def cancel_goal(self):
        self.nav.cancel_goal()
        self._log(self.L('○ 目标已取消'))

    # ------------------------------------------------------------------
    # keyboard teleop (bound by App, which forwards here)
    # ------------------------------------------------------------------
    def teleop_active(self):
        # 不再要求导航栈在跑：键盘是"一个出口"，不导航时也能开。
        # 真正决定谁占用 /cmd_vel 的是中继的优先级，不是这个开关。
        if not self.teleop_var.get():
            return False
        if not getattr(self.app.robot, 'connected', lambda: False)():
            return False
        if not self.winfo_ismapped():
            return False
        try:
            focus = self.app.root.focus_get()
        except Exception:
            focus = None
        if isinstance(focus, (tk.Entry, tk.Text, ttk.Entry, ttk.Combobox)):
            return False
        return True

    def on_key(self, ev):
        # 视角快捷键优先，且不依赖遥控开关（遥控只占 W/S/A/D 和方向键）
        if self._view_key(ev):
            return 'break'
        if not self.teleop_active():
            return None
        key = ev.keysym.lower()
        if key == 'space':
            self.emergency()
            self._log(self.L('○ 已停车'))
            return 'break'
        if key not in KEYMAP:
            return None
        t = self._rel_timers.pop(key, None)
        if t is not None:
            try:
                self.after_cancel(t)
            except Exception:
                pass
        if key not in self.nav.keys:
            self.nav.keys.add(key)
            self._apply_teleop()
        return 'break'

    def on_key_up(self, ev):
        key = ev.keysym.lower()
        if key in self.nav.keys:
            t = self._rel_timers.pop(key, None)
            if t is not None:
                try:
                    self.after_cancel(t)
                except Exception:
                    pass
            self._rel_timers[key] = self.after(
                self.KEY_RELEASE_DELAY, lambda k=key: self._release_key(k))
            return 'break'
        return None

    def _release_key(self, key):
        self._rel_timers.pop(key, None)
        if key in self.nav.keys:
            self.nav.keys.discard(key)
            self._apply_teleop()

    def _apply_teleop(self):
        lx = az = 0.0
        for k in self.nav.keys:
            f, t = KEYMAP[k]
            lx += f * self.LIN
            az += t * self.ANG
        lx = max(-self.LIN, min(self.LIN, lx))
        az = max(-self.ANG, min(self.ANG, az))
        cmd = (round(lx, 3), round(az, 3))
        if cmd == self.nav.last_cmd:
            return
        self.nav.last_cmd = cmd
        try:
            if cmd == (0.0, 0.0):
                self.app.robot.stop_manual()
            else:
                # 按键一落下，导航就必须让路：先取消目标，再交给遥控。
                # 谁真正占用 /cmd_vel 由中继仲裁（键盘优先 0.3 s），
                # 这里不用停任何进程。
                self.nav.cancel_goal()
                self.app.robot.manual(lx, az, hold=0)
        except Exception as e:
            self._log(self.L('✗ 遥控指令失败：%s') % e)

    def _on_teleop(self):
        self.nav.teleop = self.teleop_var.get()
        if self.nav.teleop:
            self._log(self.L('○ 键盘遥控已开启（W/S 前后，A/D 转向，空格停）'))
        else:
            self.emergency()
            self._log(self.L('○ 键盘遥控已关闭'))

    # ------------------------------------------------------------------
    # tick
    # ------------------------------------------------------------------
    def tick(self):
        if not self.nav.subs_done and getattr(
                self.app.robot, 'connected', lambda: False)():
            if self.nav.ensure_subs():
                self._log(self.L('○ 地图订阅已建立'))
            else:
                self._log(self.L('✗ 订阅失败：%s') % self.nav.last_error)
        if self.nav.subs_done:
            self._render()
        self._cam_tick()
        if time.time() - self._last_sync > 2.0:
            self._last_sync = time.time()
            self._sync()

    def _cam_tick(self):
        if not self.cam_var.get():
            return
        try:
            photo = self.app.nav_camera_frame(320, 160)
        except Exception:
            photo = None
        if photo is not None and photo is not self._cam_photo:
            self._cam_photo = photo
            self.cam_lb.configure(image=photo, text='')
        elif photo is None and self._cam_photo is None:
            if str(self.cam_lb.cget('text')) != self.L('相机无数据'):
                self.cam_lb.configure(text=self.L('相机无数据'))
            if time.time() > self._cam_deadline and not self._cam_hinted:
                self._cam_hinted = True
                self._log(self.L('○ 相机没有数据：Astra 驱动多半没起，'
                                 '去「视觉传感器」页点启动 Astra'))
