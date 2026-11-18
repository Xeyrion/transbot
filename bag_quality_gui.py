#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bag quality board -- point it at a folder, see which takes are usable.

Every number here comes from bag_quality_probe.py (python2, because Melodic's
rosbag is py2-only). This window is the py3 side: it shells out, parses one JSON
line per bag, and paints a table.

Verdict rules (tuned on the 2026-11-10 takes):

    good   rgb >= 20 Hz, depth >= 25 Hz, lidar >= 7 Hz, no gap > 1 s,
           rgb/depth pairing > 50 ms under 5 %
    ok     rgb >= 12 Hz, lidar >= 4 Hz, no gap > 3 s
    bad    anything else -- usually two recorders fighting over the disk

Usage:  python3 bag_quality_gui.py [folder]
"""
import json
import math
import os
import queue
import shlex
import subprocess
import sys
import threading
import time
import tempfile
from concurrent.futures import ThreadPoolExecutor
from session_catalog import sessions, bag_names, rename_target
from semantic_replay import draw_detections, discover_runs, read_json, COLORS
from map_display import Viewport, point_bounds, grid_bounds, draw_objects, export_recording_map
from mapping_job import recording_groups, map_artifacts, new_job, read_status
from semantic_presets import PRESETS
from viewer_core import semantic_directory

import tkinter as tk
from i18n import language, tr
from tkinter import ttk, filedialog, messagebox, simpledialog

HERE = os.path.dirname(os.path.abspath(__file__))
PROBE = os.path.join(HERE, 'bag_quality_probe.py')
SERVER = os.path.join(HERE, 'bag_viewer_server.py')
DEFAULT_DIR = os.path.expanduser('~/robot_research/bagtools/bags')
PREVIEW = '/tmp/bag_quality_preview.ppm'
SEMANTIC_PRESETS = {key: english if language() == 'en' else label for key, label, english, _, _ in PRESETS}

BG = '#eef1f4'
CARD = '#ffffff'
DIM = '#6b7b86'
OK = '#1b8a3a'
WARN = '#c47f00'
BAD = '#c62828'

# 地图视图里两层雷达点的颜色。历史用中性灰：它是"扫过的痕迹"，应该退到背景里
# 去，别跟当前帧抢注意力。灰也刚好是唯一能在两种底图上都成立的明度 —— 有栅格
# 地图时底图是白灰，没地图时是近黑，中灰两边都看得见。
#
# 当前帧只用绿色，不加深色描边：历史一旦是灰的，绿色靠色相就够分辨了；而描边
# 在白底上会变成一圈深色方框，比点本身还抢眼，等于把刚压下去的噪音又请回来。
HISTORY_GREY = (125, 133, 140)     # 已浏览过的扫描
CURRENT_GREEN = (34, 197, 94)      # 当前这一帧


def verdict(d):
    """(label, tag) for one probe result."""
    if d.get('err'):
        return tr('打不开'), 'bad'
    rgb = d.get('/camera/rgb/image_raw')
    dep = d.get('/camera/depth/image_raw')
    scan = d.get('/scan')
    pair = d.get('pair') or {}
    if not rgb or not dep:
        return tr('缺相机'), 'bad'
    gaps = max(rgb['max_gap'], dep['max_gap'])
    hz_rgb, hz_dep = rgb['hz'], dep['hz']
    hz_scan = scan['hz'] if scan else 0.0
    over50 = pair.get('over50_pct', 100.0)

    if hz_rgb >= 20 and hz_dep >= 25 and hz_scan >= 7 and gaps <= 1.0 and over50 <= 5:
        return tr('好'), 'good'
    if hz_rgb >= 12 and (hz_scan >= 4 or hz_scan == 0) and gaps <= 3.0:
        return tr('可用'), 'ok'
    return tr('花'), 'bad'


class App(object):
    def __init__(self, root, folder):
        self.root = root
        self.folder = folder
        self.catalog_root = os.path.abspath(folder)
        self.mode = 'sessions'
        self._preview_pool = ThreadPoolExecutor(max_workers=1)
        self._preview_job = None
        self._preview_token = 0
        self._preview_cache = {}
        self._quality_cache = {}
        self._preview_proc = None
        self._closing = False
        self._mapping_proc = None
        self._mapping_job = None
        self._mapping_window = None
        self.results = {}
        self.q = queue.Queue()
        self.busy = False
        self.stop_flag = False
        self._photo = None

        root.title(tr('Bag 质量看板'))
        root.configure(bg=BG)
        # 直接最大化：写死 1320x780 在小屏上会被 WM 压扁，列宽又是固定的，
        # 结果就是挤成一团。最大化 + 列宽自适应才稳。
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry('%dx%d+0+0' % (max(900, sw - 40), max(560, sh - 80)))
        try:
            root.attributes('-zoomed', True)      # X11 下等于最大化
        except Exception:
            pass
        # 再窄列就会被切，给个下限；真到底了还有横向滚动条兜底。
        # 下限不能超过屏幕本身：这台控制机是 1080x648，写死 1260 会让右半边
        # （预览面板、状态栏）直接跑到屏幕外面去。
        root.minsize(min(1260, sw - 40), min(560, sh - 60))
        self._style()
        self._build()
        self.root.bind('<Configure>', self._on_resize)
        self.root.after(120, self._layout_columns)
        self.scan()
        self.root.after(30, self._drain)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

    def _style(self):
        st = ttk.Style(self.root)
        try:
            st.theme_use('clam')
        except Exception:
            pass
        # 行高给足：26 太薄，六行挤在顶上像一条线
        # 行高给足（36），但字号保持 10：11 号会把 78 px 的数值列再顶爆一次
        st.configure('Treeview', rowheight=36, font=('DejaVu Sans', 10))
        st.configure('Treeview.Heading', font=('DejaVu Sans', 10, 'bold'))

    # ---------------------------------------------------------------- UI
    def _build(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill='x', padx=10, pady=(10, 4))

        tk.Label(top, text=tr('目录'), bg=BG, fg=DIM).pack(side='left')
        self.var_dir = tk.StringVar(value=self.folder)
        ent = tk.Entry(top, textvariable=self.var_dir, width=62)
        ent.pack(side='left', padx=6, fill='x', expand=True)
        ent.bind('<Return>', lambda e: self.scan())

        ttk.Button(top, text=tr('选择根目录'), command=self.choose_root).pack(side='left', padx=4)
        ttk.Button(top, text=tr('返回会话'), command=self.back_sessions).pack(side='left', padx=4)
        ttk.Button(top, text=tr('刷新'), command=self.scan).pack(side='left', padx=4)
        actions = tk.Frame(self.root, bg=BG)
        actions.pack(fill='x', padx=10, pady=(0,4))
        self.btn_all = ttk.Button(actions, text=tr('分析全部'), command=self.analyze_all)
        self.btn_all.pack(side='left', padx=4)
        self.btn_one = ttk.Button(actions, text=tr('只看选中'), command=self.analyze_one)
        self.btn_one.pack(side='left', padx=4)
        self.btn_stop = ttk.Button(actions, text=tr('停止'), command=self.stop, state='disabled')
        self.btn_stop.pack(side='left', padx=4)
        self.btn_map = ttk.Button(actions,text=tr('建图 / 重新建图'),command=self.build_map)
        self.btn_map.pack(side='left',padx=4)
        self.btn_semantic = ttk.Menubutton(actions, text=tr('生成语义地图'))
        semantic_menu = tk.Menu(self.btn_semantic, tearoff=False)
        for key, title in SEMANTIC_PRESETS.items():
            semantic_menu.add_command(label=title, command=lambda preset=key: self.generate_semantic(preset))
        self.btn_semantic.configure(menu=semantic_menu)
        self.btn_semantic.pack(side='left', padx=4)

        self.var_status = tk.StringVar(value=tr('就绪'))
        tk.Label(actions, textvariable=self.var_status, bg=BG, fg=DIM).pack(side='left', padx=12)

        self.catalog_frame = tk.Frame(self.root, bg=BG)
        self.catalog = ttk.Treeview(self.catalog_frame, columns=('name','count','size','state'), show='headings')
        catalog_scroll = ttk.Scrollbar(self.catalog_frame, orient='vertical', command=self.catalog.yview)
        catalog_scroll.pack(side='right', fill='y')
        self.catalog.configure(yscrollcommand=catalog_scroll.set)
        for key, title, width in (('name',tr('会话文件夹'),420),('count',tr('分卷数'),90),('size',tr('大小 GiB'),100),('state',tr('状态'),120)):
            self.catalog.heading(key,text=title)
            self.catalog.column(key,width=width,anchor='w' if key=='name' else 'center')
        self.catalog.pack(fill='both',expand=True)
        self.catalog.bind('<Double-1>',self.open_session)
        self.catalog.bind('<Return>',self.open_session)
        catalog_actions = tk.Frame(self.catalog_frame, bg=BG)
        catalog_actions.pack(pady=6)
        ttk.Button(catalog_actions,text=tr('进入选中会话'),command=self.open_session).pack(side='left',padx=4)
        self.btn_rename=ttk.Button(catalog_actions,text=tr('重命名'),command=self.rename_session)
        self.btn_rename.pack(side='left',padx=4)

        body = tk.Frame(self.root, bg=BG)
        self.body = body
        body.pack(fill='both', expand=True, padx=10, pady=(0, 8))

        # 表格
        cols = ('bag', 'dur', 'rgb', 'depth', 'scan', 'gap', 'pair', 'size', 'verdict')
        self.tree = ttk.Treeview(body, columns=cols, show='headings', height=14)
        # 数值列别抠：10 pt 下 "27.7" 要 40 px 左右，加上左右留白给到 78。
        heads = {'bag': (tr('包'), 320), 'dur': (tr('时长'), 78), 'rgb': (tr('彩色'), 78),
                 'depth': (tr('深度'), 78), 'scan': (tr('雷达'), 74),
                 'gap': (tr('断档'), 84), 'pair': (tr('配对'), 78),
                 'size': ('MB', 72), 'verdict': (tr('结论'), 68)}
        # 列宽策略见 _layout_columns：包名有上限，剩下的按比例给数值列。
        # 全部 stretch=False —— 交给 Treeview 自己拉伸的话，包名会把宽度吃光。
        self.COLS_MIN = {'bag': 220, 'dur': 76, 'rgb': 76, 'depth': 76,
                         'scan': 72, 'gap': 84, 'pair': 76, 'size': 70,
                         'verdict': 66}
        if self.root.winfo_screenwidth() < 1300:
            self.COLS_MIN = dict(bag=180,dur=58,rgb=58,depth=58,scan=58,gap=66,pair=60,size=58,verdict=58)
        for c in cols:
            t = heads[c]
            self.tree.heading(c, text=t[0])
            self.tree.column(c, width=t[1], minwidth=self.COLS_MIN[c],
                             stretch=False, anchor='w' if c == 'bag' else 'center')
        self.tree.tag_configure('good', foreground=OK)
        self.tree.tag_configure('ok', foreground=WARN)
        self.tree.tag_configure('bad', foreground=BAD)
        self.tree.bind('<<TreeviewSelect>>', self.on_select)
        self.tree.bind('<Double-1>', self.on_open)
        self.tree.bind('<Return>', self.on_open)
        # 打包顺序很重要：先把右侧固定宽度的面板和滚动条占位，再让表格吃掉
        # 剩下的空间。反过来的话表格 expand 会先把宽度抢光，右栏被压成一条。
        sb = ttk.Scrollbar(body, orient='vertical', command=self.tree.yview)
        sb.pack(side='right', fill='y')
        self.tree.configure(yscrollcommand=sb.set)

        xsb = ttk.Scrollbar(body, orient='horizontal', command=self.tree.xview)
        xsb.pack(side='bottom', fill='x')
        self.tree.configure(xscrollcommand=xsb.set)

        self.preview_width = 280 if self.root.winfo_screenwidth() < 1300 else 380
        right = tk.Frame(body, bg=BG, width=self.preview_width)
        right.pack(side='right', fill='y', padx=(10, 0))
        right.pack_propagate(False)

        self.tree.pack(side='left', fill='both', expand=True)

        tk.Label(right, text=tr('预览（0.5 处）'), bg=BG, fg=DIM).pack(anchor='w')
        self.lbl_img = tk.Label(right, bg='#111', width=40, height=14,
                                text=tr('选中一行看预览'), fg='#888')
        self.lbl_img.pack(pady=(4, 8))

        tk.Label(right, text=tr('详情'), bg=BG, fg=DIM).pack(anchor='w')
        self.txt = tk.Text(right, width=1, height=18, wrap='word',
                           bg=CARD, fg='#222', relief='flat', font=('monospace', 9))
        self.txt.pack(fill='both', expand=True)

    # ---------------------------------------------------------------- scan
    def _layout_columns(self):
        """按当前宽度分配列宽：包名封顶 520，其余平分，避免一边巨宽一边挤。"""
        avail = self.tree.winfo_width() - 4
        if avail < 200:
            return
        mins = self.COLS_MIN
        base = sum(mins.values())
        widths = dict(mins)
        extra = avail - base
        if extra > 0:
            widths['bag'] = min(520, mins['bag'] + int(extra * 0.55))
            left = extra - (widths['bag'] - mins['bag'])
            nums = [c for c in mins if c != 'bag']
            for c in nums:
                widths[c] = mins[c] + int(left / float(len(nums)))
        for c, w in widths.items():
            try:
                self.tree.column(c, width=w, minwidth=mins[c], stretch=False)
            except Exception:
                pass

    def _on_resize(self, ev):
        # 只在宽度明显变化时重算，避免和 Treeview 自己来回打架
        if ev.widget is not self.root:
            return
        if abs(ev.width - getattr(self, '_last_w', 0)) < 40:
            return
        self._last_w = ev.width
        self.root.after_idle(self._layout_columns)

    def choose_root(self):
        if self.busy:
            return
        folder = filedialog.askdirectory(initialdir=self.catalog_root)
        if folder:
            self.catalog_root = folder
            self.mode = 'sessions'
            self.var_dir.set(folder)
            self.scan()

    def back_sessions(self):
        if self.busy:
            self.var_status.set(tr('请先停止分析，再切换会话。'))
            return
        self.mode = 'sessions'
        self.var_dir.set(self.catalog_root)
        self.scan()

    def open_session(self, _event=None):
        selection = self.catalog.selection()
        if not selection or self.busy:
            return
        entry = self._session_entries[int(selection[0])]
        if entry['active']:
            messagebox.showinfo(tr('会话正在录制'), tr('请先停止录制并等待分卷关闭。'))
            return
        self.mode = 'bags'
        self.folder = entry['path']
        self.var_dir.set(self.folder)
        self.scan()

    def rename_session(self):
        """给已经录完的会话改名。

        录制中一律不许动：会话目录的路径已经写进 rosbag 的命令行和 .session.json，
        半路把目录挪走，收尾那一步就找不到自己的分卷了。录完再改是安全的 ——
        里面只有普通文件，改名不动内容。
        """
        selection = self.catalog.selection()
        if not selection or self.busy:
            return
        entry = self._session_entries[int(selection[0])]
        if os.path.abspath(entry['path']) == os.path.abspath(self.catalog_root):
            messagebox.showinfo(tr('这个位置不能改名'),
                                tr('选中的是根目录本身，请选一个会话文件夹。'))
            return
        if entry['active']:
            messagebox.showinfo(tr('会话正在录制'),
                                tr('录制中的会话不能改名，请先停止录制。'))
            return
        current = entry['name']
        answer = simpledialog.askstring(
            tr('重命名会话'),
            tr('新名字。只写标签会接在原来的时间戳后面；\n写 session_... 这种完整名字则原样采用。'),
            initialvalue=current, parent=self.root)
        if answer is None:
            return
        target = rename_target(current, answer)
        if not target:
            messagebox.showerror(tr('名字不可用'), tr('这个名字里没有可用字符。'))
            return
        if target == current:
            return
        dst = os.path.join(self.catalog_root, target)
        if os.path.exists(dst):
            messagebox.showerror(tr('重名了'), tr('已经有一个叫 %s 的会话。') % target)
            return
        try:
            os.rename(entry['path'], dst)
        except OSError as exc:
            messagebox.showerror(tr('改名失败'), str(exc))
            return
        if os.path.abspath(self.folder) == os.path.abspath(entry['path']):
            self.folder = dst
        self.scan()
        self.var_status.set(tr('已改名：%s') % target)

    def scan(self):
        if self.busy:
            return
        folder = os.path.abspath(os.path.expanduser(self.var_dir.get().strip()))
        if not os.path.isdir(folder):
            messagebox.showerror(tr('目录不存在'), folder)
            return
        self._preview_token += 1
        if self.mode != 'bags' or folder != os.path.abspath(self.folder):
            self.mode = 'sessions'
            self.catalog_root = folder
            self.body.pack_forget()
            self.catalog_frame.pack(fill='both',expand=True,padx=10,pady=8)
            self._session_entries = sessions(folder)
            self.catalog.delete(*self.catalog.get_children())
            for i,e in enumerate(self._session_entries):
                self.catalog.insert('', 'end', iid=str(i), values=(e['name'],e['bags'],
                    '%.2f'%(e['size']/2**30),tr('录制中') if e['active'] else tr('可分析')))
            self.btn_all.configure(state='disabled');self.btn_one.configure(state='disabled')
            self.var_status.set(tr('%d 个会话；先进入文件夹，再分析分卷。')%len(self._session_entries))
            return
        self.catalog_frame.pack_forget()
        self.body.pack(fill='both',expand=True,padx=10,pady=(0,8))
        self.folder = folder
        self.tree.delete(*self.tree.get_children());self.results.clear()
        self.lbl_img.configure(image='',text=tr('选中分卷查看预览'))
        self.txt.delete('1.0','end')
        for b in bag_names(folder):
            self.tree.insert('', 'end', iid=b, values=(b,'—','—','—','—','—','—',
                '%.0f'%(os.path.getsize(os.path.join(folder,b))/1e6),tr('待分析')))
            key = self._quality_key(b)
            if key in self._quality_cache:self._apply_row(self._quality_cache[key])
        self.btn_all.configure(state='normal');self.btn_one.configure(state='normal')
        self.var_status.set(tr('%d 个分卷；可分析本会话。')%len(self.tree.get_children()))
        self.root.after_idle(self._layout_columns)

    # ---------------------------------------------------------------- run probe
    def _quality_key(self, bag):
        path = os.path.join(self.folder, bag)
        return path, os.path.getsize(path), os.path.getmtime(path)

    def _probe(self, bag, extra=()):
        key = self._quality_key(bag)
        if not extra and key in self._quality_cache:
            return self._quality_cache[key]
        # 探针是 python2 + rosbag，必须带 ROS 环境；不管窗口是从哪起的都补上
        inner = ' '.join([shlex.quote('python2'), shlex.quote(PROBE),
                          shlex.quote(os.path.join(self.folder, bag))] +
                         [shlex.quote(a) for a in extra])
        cmd = ['bash', '-c', 'source /opt/ros/melodic/setup.bash 2>/dev/null; exec ' + inner]
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=900)
        for line in p.stdout.strip().splitlines():
            line = line.strip()
            if line.startswith('{'):
                data = json.loads(line)
                if not extra:self._quality_cache[key] = data
                return data
        raise RuntimeError((p.stderr or 'no output')[-200:])

    def analyze_all(self):
        if self.mode != 'bags':
            return
        bags = list(self.tree.get_children())
        if not bags:
            return
        if self.busy:
            return
        self.busy = True
        self.stop_flag = False
        self.btn_all.configure(state='disabled')
        self.btn_stop.configure(state='normal')

        def work():
            for i, bag in enumerate(bags):
                if self.stop_flag:
                    break
                self.q.put(('status', tr('分析 %d/%d：%s') % (i + 1, len(bags), bag)))
                try:
                    d = self._probe(bag)
                except Exception as e:
                    d = {'bag': bag, 'err': str(e)}
                self.q.put(('row', d))
            self.q.put(('done', None))

        threading.Thread(target=work, daemon=True).start()

    def analyze_one(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo(tr('先选一行'), tr('在表格里点一个包。'))
            return
        bag = sel[0]
        if self.busy or self.mode != 'bags':
            return
        self.busy = True
        self.btn_all.configure(state='disabled')
        self.btn_stop.configure(state='normal')
        self.var_status.set(tr('分析 %s …') % bag)

        def work():
            try:
                d = self._probe(bag)
            except Exception as e:
                d = {'bag': bag, 'err': str(e)}
            self.q.put(('row', d))
            self.q.put(('done', None))

        threading.Thread(target=work, daemon=True).start()

    def stop(self):
        self.stop_flag = True
        self.var_status.set(tr('已请求停止（当前这个包跑完就停）'))

    def _choose_mapping_group(self, groups):
        dialog=tk.Toplevel(self.root);dialog.title(tr('选择录制'));dialog.transient(self.root)
        tk.Label(dialog,text=tr('文件夹包含多次录制，请选择一组分卷。')).pack(padx=16,pady=12)
        choice=ttk.Combobox(dialog,state='readonly',width=54,
            values=['%s (%d)'%(os.path.basename(g[0]),len(g)) for g in groups]);choice.current(0);choice.pack(padx=16,pady=4)
        result=[]
        def accept():result.append(groups[choice.current()][0]);dialog.destroy()
        ttk.Button(dialog,text=tr('确定'),command=accept).pack(padx=16,pady=12)
        dialog.grab_set();self.root.wait_window(dialog)
        return result[0] if result else None

    def _selected_recording(self, title):
        if self.mode=='sessions':
            selected=self.catalog.selection()
            if not selected:
                messagebox.showinfo(title,tr('请先选中一个会话文件夹。'),parent=self.root);return None
            entry=self._session_entries[int(selected[0])]
            if entry['active']:
                messagebox.showinfo(tr('会话正在录制'),tr('请先停止录制并等待分卷关闭。'),parent=self.root);return None
            folder=entry['path']
        else:
            folder=self.folder;selected=self.tree.selection()
            if selected:
                return os.path.join(folder,selected[0])
        groups=recording_groups(folder)
        if not groups:raise ValueError(tr('没有可建图的录制数据。'))
        return groups[0][0] if len(groups)==1 else self._choose_mapping_group(groups)

    def generate_semantic(self, preset):
        if self.busy or self._mapping_proc is not None:
            return
        title = tr('生成语义地图')
        try:
            bag = self._selected_recording(title)
            if bag is None:
                return
            folder, name = os.path.split(bag)
            BagViewer(self.root, folder, name, preset=preset)
        except (OSError, ValueError) as exc:
            messagebox.showerror(title, str(exc), parent=self.root)

    def build_map(self):
        if self.busy or self._mapping_proc is not None:return
        if self._mapping_window is not None:
            self._mapping_window.destroy();self._mapping_window=None
        try:
            bag = self._selected_recording(tr('建图'))
            if bag is None:return
            folder = os.path.dirname(bag)
            previous=map_artifacts(bag)
            if previous and not messagebox.askyesno(tr('替换已有地图？'),
                    tr('已存在地图。是否用本次数据重新建图并替换当前地图？\n旧地图会保留备份，原始 bag 不会修改。'),parent=self.root):return
            job=new_job(bag)
            command=['bash','-c','source /opt/ros/melodic/setup.bash 2>/dev/null; exec '+
                     ' '.join(shlex.quote(v) for v in [sys.executable,os.path.join(HERE,'mapping_job.py'),bag,'--job',str(job)]+(['--replace'] if previous else []))]
            with (job/'console.log').open('ab') as log:
                self._mapping_proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self._mapping_job=job;self._mapping_bag=bag;self.busy=True
            for button in (self.btn_all,self.btn_one,self.btn_map,self.btn_semantic,self.btn_rename):button.configure(state='disabled')
            self._mapping_window=tk.Toplevel(self.root);self._mapping_window.title(tr('会话建图'))
            self._mapping_window.geometry('660x370');self._mapping_window.transient(self.root)
            tk.Label(self._mapping_window,text=os.path.basename(folder),anchor='w').pack(fill='x',padx=12,pady=10)
            self._mapping_label=tk.StringVar(value=tr('检查雷达、里程计与时间连续性…'))
            tk.Label(self._mapping_window,textvariable=self._mapping_label,anchor='w',wraplength=620).pack(fill='x',padx=12)
            self._mapping_spinner=ttk.Progressbar(self._mapping_window,mode='indeterminate');self._mapping_spinner.pack(fill='x',padx=12,pady=10);self._mapping_spinner.start(15)
            self._mapping_log=tk.Text(self._mapping_window,height=10,wrap='word',font=('monospace',9));self._mapping_log.pack(fill='both',expand=True,padx=12)
            self._mapping_cancel=ttk.Button(self._mapping_window,text=tr('取消建图'),command=self.cancel_mapping);self._mapping_cancel.pack(pady=10)
            self._mapping_window.protocol('WM_DELETE_WINDOW',self.cancel_mapping)
            self._mapping_started=time.monotonic();self._mapping_stage=None
        except Exception as exc:
            messagebox.showerror(tr('建图失败'),str(exc),parent=self.root)

    def cancel_mapping(self):
        if self._mapping_proc is None:
            if self._mapping_window is not None:self._mapping_window.destroy();self._mapping_window=None
            return
        (self._mapping_job/'cancel').touch()
        self._mapping_label.set(tr('正在停止建图；已有地图不会被替换。'))
        self._mapping_cancel.configure(state='disabled')

    def _poll_mapping(self):
        if self._mapping_proc is None:return
        if time.monotonic()<getattr(self,'_mapping_next_poll',0):return
        self._mapping_next_poll=time.monotonic()+.25
        status=read_status(self._mapping_job);stage=status.get('stage','inspect')
        labels={'inspect':tr('检查雷达、里程计与时间连续性…'),'rebuild':tr('根据当前数据重新建图…'),
                'verify':tr('验证地图并更新关联…'),'stopping':tr('正在停止建图…')}
        elapsed=time.monotonic()-self._mapping_started
        label=labels.get(stage,stage)
        text=tr('%s  已用 %.0f 秒')%(label,elapsed)
        if status.get('partial'):text+='  '+tr('仅使用连续片段 %.1f 秒')%status.get('selected_duration_s',0)
        if not (self._mapping_job/'cancel').exists():self._mapping_label.set(text)
        self.var_status.set(text)
        path=self._mapping_job/('inspect.log' if stage=='inspect' else 'rebuild.log')
        live_logs=list((self._mapping_job/'result/navigation').glob('*.mapping.log'))
        if live_logs:path=live_logs[0]
        if path.exists():
            with path.open('rb') as log:
                log.seek(max(0,path.stat().st_size-5000));tail=log.read().decode('utf-8','replace')
            if tail!=getattr(self,'_mapping_tail',None):
                self._mapping_log.delete('1.0','end');self._mapping_log.insert('end',tail);self._mapping_log.see('end');self._mapping_tail=tail
        code=self._mapping_proc.poll()
        if code is None:return
        self._mapping_proc=None;self.busy=False;self._mapping_spinner.stop()
        self.btn_map.configure(state='normal');self.btn_semantic.configure(state='normal');self.btn_rename.configure(state='normal')
        self.btn_all.configure(state='normal' if self.mode=='bags' else 'disabled')
        self.btn_one.configure(state='normal' if self.mode=='bags' else 'disabled')
        if status.get('state')=='complete' and code==0:
            text=tr('建图完成，重新打开分卷即可查看新地图。')
            if status.get('partial'):text+=' '+tr('仅使用连续片段 %.1f 秒')%status.get('selected_duration_s',0)
        elif status.get('state')=='cancelled':text=tr('建图已取消，原地图保留。')
        else:text=tr('建图失败，原地图保留：')+status.get('error',str(self._mapping_job/'console.log'))
        self._mapping_label.set(text);self.var_status.set(text)
        self._mapping_cancel.configure(text=tr('关闭'),state='normal',command=self.cancel_mapping)
        self._mapping_window.protocol('WM_DELETE_WINDOW',self.cancel_mapping)

    # ---------------------------------------------------------------- drain
    def _drain(self):
        if self._closing:
            return
        self._poll_mapping()
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == 'status':
                    self.var_status.set(payload)
                elif kind == 'row':
                    self._apply_row(payload)
                elif kind == 'done':
                    self.busy = False
                    self.btn_all.configure(state='normal')
                    self.btn_stop.configure(state='disabled')
                    n = len(self.results)
                    good = sum(1 for d in self.results.values() if verdict(d)[1] == 'good')
                    self.var_status.set(tr('完成：%d 个包，%d 个好') % (n, good))
                elif kind == 'preview':
                    token, result, error = payload
                    if token == self._preview_token:
                        if error:
                            self.lbl_img.configure(image='', text=tr('预览失败：')+error)
                        else:
                            from PIL import ImageTk
                            self._photo = ImageTk.PhotoImage(result)
                            self.lbl_img.configure(image=self._photo, text='', width=result.width, height=result.height)
        except queue.Empty:
            pass
        self.root.after(30, self._drain)

    def _apply_row(self, d):
        bag = d['bag']
        self.results[bag] = d
        label, tag = verdict(d)
        rgb = d.get('/camera/rgb/image_raw') or {}
        dep = d.get('/camera/depth/image_raw') or {}
        scan = d.get('/scan') or {}
        pair = d.get('pair') or {}
        gaps = max(rgb.get('max_gap', 0), dep.get('max_gap', 0))
        vals = (bag,
                '%ds' % d.get('dur', 0) if d.get('dur') else '—',
                '%.1f' % rgb.get('hz', 0) if rgb else '—',
                '%.1f' % dep.get('hz', 0) if dep else '—',
                '%.1f' % scan.get('hz', 0) if scan else tr('无'),
                '%.1fs' % gaps if gaps else '—',
                '%.0fms' % pair.get('median_ms', 0) if pair else '—',
                '%.0f' % d.get('size_mb', 0),
                label)
        if self.tree.exists(bag):
            self.tree.item(bag, values=vals, tags=(tag,))
        else:
            self.tree.insert('', 'end', iid=bag, values=vals, tags=(tag,))

    # ---------------------------------------------------------------- detail
    def on_select(self, _ev=None):
        sel = self.tree.selection()
        if not sel:
            return
        bag = sel[0]
        d = self.results.get(bag)
        if not d:
            self.txt.delete('1.0', 'end')
            self.txt.insert('end', tr('还没分析这个包。点「只看选中」。'))
            self._show_preview(bag)
            return
        self._show_detail(d)
        self._show_preview(bag)

    def on_open(self, ev=None):
        if ev is not None and getattr(ev, 'y', None) is not None:
            row = self.tree.identify_row(ev.y)
            if row:
                self.tree.selection_set(row)
        sel = self.tree.selection()
        if not sel:
            return
        try:
            BagViewer(self.root, self.folder, sel[0])
        except Exception as e:
            messagebox.showerror(tr('打不开查看器'), str(e))

    def _show_detail(self, d):
        def t(k):
            v = d.get(k)
            return '—' if v is None else v
        lines = [
            d['bag'],
            tr('大小      %.0f MB') % d.get('size_mb', 0),
            tr('时长      %s s') % t('dur'),
            tr('消息数    %s') % t('msgs'),
            '',
        ]
        for key, name in (('/camera/rgb/image_raw', tr('彩色')),
                          ('/camera/depth/image_raw', tr('深度')),
                          ('/scan', tr('雷达')),
                          ('/odom', tr('里程计')),
                          ('/imu/data', 'IMU')):
            v = d.get(key)
            if not v:
                lines.append(tr('%-6s 无') % name)
            else:
                lines.append(tr('%-6s %5d 帧  %5.2f Hz  最大断档 %.2fs  >1s %d 次')
                             % (name, v['n'], v['hz'], v['max_gap'], v['gt1']))
        p = d.get('pair')
        if p:
            lines += ['', tr('彩色↔深度  中位 %.1f ms，>50ms %.1f%%')
                      % (p['median_ms'], p['over50_pct'])]
        inputs = d.get('semantic_inputs')
        if inputs is not None:
            missing = [k for k, count in inputs.items() if not count]
            lines += ['', tr('时间指标依据：bag记录时间'),
                      tr('语义数据清单缺少：')+(', '.join(missing) if missing else tr('无'))]
            lines += [tr('话题齐全仍需检查外参、时间戳与完整TF；地图/轨迹可离线补建。')]
        if d.get('path_m') is not None:
            lines.append(tr('走过      %.2f m') % d['path_m'])
        if d.get('err'):
            lines += ['', tr('错误：') + d['err']]
        label, _ = verdict(d)
        lines += ['', tr('结论：') + label]
        self.txt.delete('1.0', 'end')
        self.txt.insert('end', '\n'.join(lines))

    def _show_preview(self, bag):
        path = os.path.join(self.folder, bag)
        key = (path, os.path.getsize(path), os.path.getmtime(path))
        self._preview_token += 1
        token = self._preview_token
        if self._preview_job:
            self._preview_job.cancel()
        self.lbl_img.configure(image='',text=tr('正在后台加载预览…'))
        def work():
            if self._closing:
                return
            error, image = None, self._preview_cache.get(key)
            if image is None:
                fd, preview = tempfile.mkstemp(suffix='.ppm',prefix='bag-preview-');os.close(fd)
                try:
                    p = subprocess.Popen(['bash','-c','source /opt/ros/melodic/setup.bash 2>/dev/null; exec python2 %s %s --frame %s'
                        %(shlex.quote(PROBE),shlex.quote(path),shlex.quote(preview))],
                        stdout=subprocess.PIPE,stderr=subprocess.PIPE)
                    self._preview_proc = p
                    if self._closing:p.terminate()
                    p.communicate(timeout=120)
                    from PIL import Image
                    with Image.open(preview) as source:
                        image=source.convert('RGB');image.thumbnail((self.preview_width-20,300))
                    if len(self._preview_cache)>=20:self._preview_cache.clear()
                    self._preview_cache[key]=image
                except Exception as exc:error=str(exc)
                finally:
                    if self._preview_proc is not None and self._preview_proc.poll() is None:
                        self._preview_proc.terminate()
                        self._preview_proc.wait()
                    self._preview_proc = None
                    if os.path.exists(preview):os.unlink(preview)
            self.q.put(('preview',(token,image,error)))
        self._preview_job = self._preview_pool.submit(work)

    def close(self):
        if self._mapping_proc is not None:
            self._mapping_window.lift()
            messagebox.showinfo(tr('会话建图'),tr('请先完成或取消建图，再关闭数据阅读。'),parent=self.root)
            return
        self._closing = True
        if self._preview_job:self._preview_job.cancel()
        if self._preview_proc is not None and self._preview_proc.poll() is None:
            self._preview_proc.terminate()
        self._preview_pool.shutdown(wait=True)
        for child in self.root.winfo_children():
            if isinstance(child, BagViewer):child.close()
        self.root.destroy()


class BagViewer(tk.Toplevel):
    """双击包名打开：拖时间轴看那一刻的彩色/深度/雷达/位姿。

    真正读包的是 bag_viewer_server.py（python2 常驻进程），这里只发时间、收
    一帧的结果。常驻是为了不用每次重新打开 bag —— 2 GB 的包重开一次要好几秒。
    """

    def __init__(self, master, folder, bag, preset=None):
        tk.Toplevel.__init__(self, master)
        self.title(tr('查看：') + bag)
        self.configure(bg=BG)
        self.geometry('%dx%d+40+20' % (
            min(1400, self.winfo_screenwidth() - 60),
            min(820, self.winfo_screenheight() - 90)))
        self.bag = bag
        self.folder = folder
        self.dur = 0.0
        self._closing = False
        self._want_t = 0.0
        self._playing = False
        self._play_job = None
        self._photos = []
        self._ph_rgb = None          # 复用同一个 Tk 图像，别每帧新建
        self._ph_depth = None
        self._setting_sl = False     # 程序改滑块时不要触发 _on_slide
        self._last_seek = 0.0
        self._force = False
        self.frame_mode = 'base'     # 'base' 车体系 / 'map' 地图系
        self._map_img = None         # 累积点云画在这张 PIL 图上
        self._map_ox = self._map_oy = 0.0
        self._map_n = 0
        self._ph_map = None
        self._last = {}
        self._q = queue.Queue()
        self._frame_applied = threading.Event()
        self._export_rgb = self._export_depth = None
        self.proc = None
        self._reader_thread = None
        self._loading = True
        self._load_job = self._drain_job = None
        self._load_error = False
        self._reader_log = tempfile.TemporaryFile(mode='w+b')
        self._load_started = time.monotonic()
        self._load_phase = tr('正在打开录制包、准备轨迹')
        self.world = 'odom'
        self.map_info = None
        self.has_map = False
        self._map_panel = None
        self._last_map_t = None
        self._record_t0 = 0.
        self._semantic_run = None
        self._semantic_choices = []
        self._semantic_runs = []
        self._semantic_proc = None
        self._semantic_poll_job = None
        self._semantic_started = None
        self._initial_preset = preset
        self._resize_job = self._export_poll_job = None
        self._layout_job=None
        self._layout_initialized=False
        self._map_popup = self._large_canvas = None
        self._ph_map_large = None
        self._map_span = 12.
        self._map_centre = None
        self._map_overview = True
        self._grid_bounds = None
        self._viewports = {}
        self._export_pool = self._map_export_future = None
        self._build()
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.update_idletasks()
        self._loading_tick()
        try:
            self.proc = subprocess.Popen(
                ['bash', '-c', 'source /opt/ros/melodic/setup.bash 2>/dev/null; '
                               'exec python2 %s %s'
                               % (shlex.quote(SERVER), shlex.quote(os.path.join(folder, bag)))],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=self._reader_log, universal_newlines=True, bufsize=1)
        except Exception as exc:
            self._loading_failed(str(exc))
            return
        self._reader_thread = threading.Thread(target=self._worker, daemon=True)
        self._reader_thread.start()
        self._drain_job = self.after(16, self._drain)
        self.seek(0.0)

    # ------------------------------------------------------------ UI
    def _build(self):
        bar = tk.Frame(self, bg=BG)
        bar.pack(fill='x', padx=10, pady=(10, 4))

        self.var_t = tk.StringVar(value='0.0 s')
        tk.Label(bar, textvariable=self.var_t, bg=BG, fg=DIM, width=12).pack(side='left')

        # 别用 ⏮ ◀ ▶ 这类符号：字体里没有这个字形时 Tk 会抛
        # BadLength (RenderAddGlyphs)，整个窗口直接挂掉。ASCII 最稳。
        self._controls = []
        for txt, cmd in (('|<', lambda: self.seek(0)),
                         ('<', lambda: self.step(-0.5)),
                         (tr('播放'), self.toggle_play),
                         ('>', lambda: self.step(0.5)),
                         ('>|', lambda: self.seek(self.dur))):
            button = ttk.Button(bar, text=txt, width=6, command=cmd)
            button.pack(side='left', padx=2)
            self._controls.append(button)

        self.var_sl = tk.DoubleVar(value=0.0)
        sl = ttk.Scale(bar, from_=0.0, to=max(0.1, self.dur), variable=self.var_sl,
                       command=self._on_slide)
        self.slider = sl
        sl.pack(side='left', fill='x', expand=True, padx=10)
        sl.bind('<ButtonRelease-1>', self._on_slide_release)

        export_button = ttk.Button(bar, text=tr('导出三合一'), width=11,command=self.export_combo)
        export_button.pack(side='left', padx=(8,2))
        self.btn_frame = ttk.Button(bar, text=tr('切到地图系'), width=11,
                                    command=self.toggle_frame)
        self.btn_frame.pack(side='left', padx=2)
        self._controls += [self.slider, export_button, self.btn_frame]

        semantic_bar = tk.Frame(self, bg=BG)
        self.semantic_bar = semantic_bar
        semantic_bar.pack(fill='x', padx=12, pady=3)
        tk.Label(semantic_bar, text=tr('语义回放'), bg=BG, fg=DIM).pack(side='left')
        self.var_semantic = tk.StringVar(value=tr('原始数据'))
        self.semantic_picker = ttk.Combobox(semantic_bar, textvariable=self.var_semantic,
                                            values=[tr('原始数据')], width=28, state='readonly')
        self.semantic_picker.pack(side='left', padx=6)
        self.semantic_picker.bind('<<ComboboxSelected>>', self._select_semantic)
        self._controls.append(self.semantic_picker)
        self.generate_button, generate_menu = self._semantic_menu(semantic_bar, tr('生成语义地图'))
        for preset, title in SEMANTIC_PRESETS.items():
            generate_menu.add_command(label=title, command=lambda key=preset: self._generate_semantic(key))
        gallery = ttk.Button(semantic_bar, text=tr('地图图库'), command=self.open_gallery)
        gallery.pack(side='left', padx=3)
        self._controls.append(gallery)

        _, view_menu = self._semantic_menu(semantic_bar, tr('视图'))
        self.var_boxes = tk.BooleanVar(value=True)
        self.var_points = tk.BooleanVar(value=True)
        self.var_other_classes = tk.BooleanVar(value=False)
        for label, var in ((tr('识别框'), self.var_boxes), (tr('物品点'), self.var_points)):
            view_menu.add_checkbutton(label=label, variable=var, command=self._redraw_semantic)
        view_menu.add_checkbutton(label=tr('显示其它识别类别'), variable=self.var_other_classes,
                                  command=self._toggle_other_classes)
        view_menu.add_separator()
        for label, command in ((tr('均衡布局'), lambda: self.set_layout('balanced')),
                               (tr('画面优先'), lambda: self.set_layout('rgb')),
                               (tr('地图优先'), lambda: self.set_layout('map')),
                               (tr('放大地图'), self.open_map_window),
                               (tr('缩小地图'), lambda: self.zoom_map(1.25)),
                               (tr('放大比例'), lambda: self.zoom_map(.8)),
                               (tr('全图'), self.overview_map), (tr('跟随机器人'), self.follow_map)):
            view_menu.add_command(label=label, command=command)

        _, more_menu = self._semantic_menu(semantic_bar, tr('更多'))
        more_menu.add_command(label=tr('最终地图'), command=self.open_final_map)
        more_menu.add_command(label=tr('结果目录'), command=self.open_results)
        more_menu.add_separator()
        self.var_all_runs = tk.BooleanVar(value=False)
        more_menu.add_checkbutton(label=tr('显示全部历史结果'), variable=self.var_all_runs,
                                 command=self._refresh_semantic_choices)
        more_menu.add_command(label=tr('加载语义结果…'), command=self._load_semantic)
        more_menu.add_separator()
        more_menu.add_command(label=tr('语义生成说明'), command=self._semantic_help)
        self.var_map_scale = tk.StringVar()
        tk.Label(semantic_bar, textvariable=self.var_map_scale, bg=BG, fg=DIM).pack(side='left', padx=5)
        self.var_generation = tk.StringVar()
        # Only occupy a status row after an explicit generation request.
        self.semantic_status = tk.Frame(self, bg=BG)
        self.semantic_progress = ttk.Progressbar(self.semantic_status, mode='indeterminate', length=110)
        self.semantic_status_label = tk.Label(self.semantic_status, textvariable=self.var_generation,
                                             bg=BG, fg=DIM, anchor='w', wraplength=1000)
        self.semantic_status_label.pack(side='left', fill='x', expand=True)
        self._set_controls(False)

        self.var_semantic_note = tk.StringVar()
        tk.Label(self, textvariable=self.var_semantic_note, bg=BG, fg=DIM,
                 anchor='w', wraplength=1200).pack(fill='x', padx=12)

        self.var_info = tk.StringVar(value=tr('正在读取会话 TF，请稍候…'))
        tk.Label(self, textvariable=self.var_info, bg=BG, fg=DIM,
                 anchor='w').pack(fill='x', padx=12)
        self.var_map_note=tk.StringVar()
        self.map_note_label=tk.Label(self,textvariable=self.var_map_note,bg=BG,fg=WARN,anchor='w',wraplength=850)

        self.loading_frame = tk.Frame(self, bg=BG)
        self.loading_frame.pack(fill='x', padx=14, pady=10)
        self.spinner = ttk.Progressbar(self.loading_frame, mode='indeterminate', length=180)
        self.spinner.pack(side='left', padx=(0,12))
        self.spinner.start(12)
        self.var_loading = tk.StringVar(value=tr('正在打开…'))
        self.loading_label = tk.Label(self.loading_frame, textvariable=self.var_loading,bg=BG,fg=DIM)
        self.loading_label.pack(side='left')

        self.body=ttk.Panedwindow(self,orient='horizontal')
        self.body.pack(fill='both',expand=True,padx=10,pady=8)
        self.body.bind('<Configure>',self._initial_layout)
        rgb=tk.Frame(self.body,bg='#111',width=720,height=480);rgb.pack_propagate(False)
        self.lbl_rgb=tk.Label(rgb,bg='#111',text=tr('彩色'),fg='#777',width=1,height=1)
        self.lbl_rgb.pack(fill='both',expand=True)
        self.right_split=ttk.Panedwindow(self.body,orient='vertical',width=440)
        self.body.add(rgb,weight=3);self.body.add(self.right_split,weight=2)
        depth=tk.Frame(self.right_split,bg=BG,width=440,height=200);depth.pack_propagate(False)
        tk.Label(depth,text=tr('深度（0~4 m，近白远黑）'),bg=BG,fg=DIM).pack(anchor='w')
        self.lbl_depth=tk.Label(depth,bg='#111',text=tr('深度'),fg='#777',width=1,height=1)
        self.lbl_depth.pack(fill='both',expand=True)
        mapping=tk.Frame(self.right_split,bg=BG,width=440,height=320);mapping.pack_propagate(False)
        self.right_split.add(depth,weight=1);self.right_split.add(mapping,weight=2)
        self.lbl_scan_title = tk.Label(mapping, text=tr('正在加载全局视图'), bg=BG, fg=DIM)
        self.lbl_scan_title.pack(anchor='w')
        self.cv = tk.Canvas(mapping, width=440, height=320, bg='#0d1117',
                            highlightthickness=0)
        self.cv.pack(fill='both', expand=True)
        for widget in (self.lbl_rgb,self.lbl_depth,self.cv):
            widget.bind('<Configure>',self._queue_redraw)
        self._bind_map(self.cv)

    def _initial_layout(self,event):
        # An idle callback before mapping sees width=1 and collapses the RGB pane.
        if not self._layout_initialized and event.width>300 and event.height>150:
            self._layout_initialized=True
            self._layout_job=self.after_idle(self._apply_initial_layout)

    def _apply_initial_layout(self):
        self._layout_job=None
        self.set_layout('balanced')

    def set_layout(self,mode):
        if self._closing:return
        self.update_idletasks()
        ratio={'balanced':.6,'rgb':.78,'map':.32}[mode]
        self.body.sashpos(0,int(self.body.winfo_width()*ratio))
        self.update_idletasks()
        self.right_split.sashpos(0,int(self.right_split.winfo_height()*(.24 if mode=='map' else .36)))
        self._queue_redraw()

    def _queue_redraw(self,_event=None):
        if self._closing:return
        if self._resize_job:self.after_cancel(self._resize_job)
        self._resize_job=self.after(100,self._redraw_view)

    @staticmethod
    def _canvas_size(canvas):
        return max(32,canvas.winfo_width()),max(32,canvas.winfo_height())

    def _render_images(self):
        from PIL import ImageTk
        for image,label,attribute in ((self._export_rgb,self.lbl_rgb,'_ph_rgb'),
                                      (self._export_depth,self.lbl_depth,'_ph_depth')):
            if image is None:continue
            size=self._canvas_size(label);box=self._fit(image,*size)
            photo=getattr(self,attribute)
            if photo is None or (photo.width(),photo.height())!=size:
                photo=ImageTk.PhotoImage(box);setattr(self,attribute,photo)
            else:photo.paste(box)
            label.configure(image=photo,text='')

    def _redraw_view(self):
        if self._resize_job:self.after_cancel(self._resize_job)
        self._resize_job=None
        if self._closing:return
        self._render_images()
        d=self._last
        if d.get('err'):return
        if d.get('frame') in ('map','odom') and d.get('pose'):
            self._map_ensure(d['pose']);self._draw_map_view(d['pose'],d.get('scan'))
        elif d:self._draw_scan(d.get('scan'))

    def _bind_map(self,canvas):
        canvas.bind('<MouseWheel>',lambda e:self.zoom_map(.8 if e.delta>0 else 1.25,canvas))
        canvas.bind('<Button-4>',lambda e:self.zoom_map(.8,canvas))
        canvas.bind('<Button-5>',lambda e:self.zoom_map(1.25,canvas))
        canvas.bind('<ButtonPress-1>',lambda e:self._start_map_drag(e,canvas))
        canvas.bind('<B1-Motion>',self._drag_map)

    def _start_map_drag(self,event,canvas):
        view=self._viewports.get(canvas)
        self._drag_origin=(event.x,event.y,view) if view else None

    def _drag_map(self,event):
        if not getattr(self,'_drag_origin',None):return
        x,y,view=self._drag_origin
        self._map_centre=(view.centre[0]-(event.x-x)/view.scale,view.centre[1]+(event.y-y)/view.scale)
        self._map_span=view.span;self._map_overview=False
        self._redraw_view()

    def zoom_map(self,factor,canvas=None):
        view=self._viewports.get(canvas or self.cv)
        if view:self._map_centre=view.centre;self._map_span=view.span
        self._map_span=max(.8,min(60.,self._map_span*factor));self._map_overview=False
        self._redraw_view()

    def overview_map(self):
        self._map_overview=True;self._redraw_view()

    def follow_map(self):
        self._map_overview=False;self._map_centre=None;self._redraw_view()

    def open_map_window(self):
        if self._map_popup and self._map_popup.winfo_exists():
            self._map_popup.lift();return
        win=tk.Toplevel(self);self._map_popup=win;win.title(tr('大地图 · 可滚轮缩放、拖动平移'))
        win.geometry('%dx%d+60+40'%(min(1200,self.winfo_screenwidth()-100),min(850,self.winfo_screenheight()-150)))
        bar=tk.Frame(win);bar.pack(fill='x')
        for text,command in (('+',lambda:self.zoom_map(.8,self._large_canvas)),('-',lambda:self.zoom_map(1.25,self._large_canvas)),
                             (tr('全图'),self.overview_map),(tr('跟随机器人'),self.follow_map),(tr('最终地图'),self.open_final_map)):
            ttk.Button(bar,text=text,command=command).pack(side='left',padx=3,pady=4)
        self._large_canvas=tk.Canvas(win,bg='#0d1117',highlightthickness=0)
        self._large_canvas.pack(fill='both',expand=True);self._large_canvas.bind('<Configure>',self._queue_redraw)
        self._bind_map(self._large_canvas);win.protocol('WM_DELETE_WINDOW',self._close_map_window)
        self._queue_redraw()

    def _close_map_window(self):
        if self._large_canvas:self._viewports.pop(self._large_canvas,None)
        if self._map_popup:self._map_popup.destroy()
        self._map_popup=self._large_canvas=self._ph_map_large=None

    @staticmethod
    def _open_path(path):
        subprocess.Popen(['xdg-open',path],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    def open_results(self):
        self._open_path(self._last.get('semantic_run') or self.folder)

    def open_gallery(self):
        path = os.path.join(semantic_directory(os.path.join(self.folder, self.bag)), 'gallery')
        os.makedirs(path, exist_ok=True)
        self._open_path(path)

    def open_final_map(self):
        folder=self._last.get('semantic_run')
        if not folder:
            messagebox.showinfo(tr('最终地图'),tr('请先在语义回放中选择一个版本。'),parent=self);return
        if self._map_export_future and not self._map_export_future.done():return
        if self._playing:self.toggle_play()
        if self._export_pool is None:self._export_pool=ThreadPoolExecutor(max_workers=1)
        self.var_info.set(tr('正在导出完整语义地图…'))
        gallery = os.path.join(semantic_directory(os.path.join(self.folder, self.bag)), 'gallery')
        self._map_export_future = self._export_pool.submit(export_recording_map, folder, gallery)
        self._poll_final_map()

    def _poll_final_map(self):
        self._export_poll_job=None
        if self._closing:return
        if not self._map_export_future.done():
            self._export_poll_job=self.after(80,self._poll_final_map);return
        try:
            path=self._map_export_future.result();self.var_info.set(tr('最终地图已保存：')+path);self._open_path(path)
        except Exception as exc:
            self.var_info.set(tr('最终地图导出失败：')+str(exc))

    def _set_controls(self, enabled):
        for widget in self._controls:
            widget.state(['!disabled'] if enabled else ['disabled'])

    def _loading_tick(self):
        if self._closing or not self._loading:return
        self.var_loading.set(tr('%s… 已等待 %.1f 秒（可随时关闭）') %
                             (self._load_phase,time.monotonic()-self._load_started))
        self._load_job = self.after(100,self._loading_tick)

    def _loading_failed(self, reason):
        self._loading=False;self._load_error=True;self.spinner.stop()
        self.var_loading.set(tr('打开失败：')+reason)
        self.loading_label.configure(fg=BAD)

    # ------------------------------------------------------------ 交互
    def _select_semantic(self, _event=None):
        index = self.semantic_picker.current()
        self._semantic_run = self._semantic_choices[index-1]['path'] if index > 0 else None
        if self._playing:
            self.toggle_play()
        self.var_semantic_note.set(tr('正在加载语义结果…') if self._semantic_run else '')
        self._force = True

    def _semantic_menu(self, parent, label):
        button = ttk.Menubutton(parent, text=label)
        menu = tk.Menu(button, tearoff=False)
        button.configure(menu=menu)
        button.pack(side='left', padx=3)
        self._controls.append(button)
        return button, menu

    def _refresh_semantic_choices(self, select=None):
        selected = select or self._semantic_run
        if self.var_all_runs.get():
            runs = list(self._semantic_runs)
        else:
            runs = []
            for preset, label in SEMANTIC_PRESETS.items():
                matches = [run for run in self._semantic_runs if run.get('preset') == preset]
                if matches:
                    latest = max(matches, key=lambda run: (run.get('mtime', 0), run['path']))
                    runs.append(dict(latest, label=label))
            # Keep an explicitly loaded selection visible when switching back from history.
            if selected and all(run['path'] != selected for run in runs):
                runs.extend(run for run in self._semantic_runs if run['path'] == selected)
        self._semantic_choices = runs
        self.semantic_picker.configure(values=[tr('原始数据')] + [run['label'] for run in runs])
        paths = [run['path'] for run in runs]
        self.semantic_picker.current(paths.index(selected)+1 if selected in paths else 0)
        if select:
            self._select_semantic()

    def _load_semantic(self):
        folder = filedialog.askdirectory(parent=self, title=tr('选择包含 run.json 的语义结果文件夹'),
                                         initialdir=semantic_directory(os.path.join(self.folder, self.bag)))
        if not folder:
            return
        if all(run['path'] != folder for run in self._semantic_runs):
            self._semantic_runs.append(dict(path=folder, label=os.path.basename(folder)))
        self._refresh_semantic_choices(select=folder)

    def _generate_semantic(self, preset):
        if self._semantic_proc is not None:
            return
        title = SEMANTIC_PRESETS[preset]
        bag = os.path.join(self.folder, self.bag)
        self._semantic_started = time.monotonic()
        try:
            results = semantic_directory(bag)
            logs = os.path.join(results, 'logs')
            os.makedirs(logs, exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=logs, prefix=preset+'_', suffix='.log', delete=False) as log:
                self._semantic_log = log.name
                self._semantic_output = os.path.join(results, os.path.splitext(os.path.basename(log.name))[0])
                command = ['bash', os.path.join(HERE, 'semantic', 'run_semantic.sh'), bag,
                           self._semantic_output, '--preset', preset]
                self._semantic_proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                                       stderr=subprocess.STDOUT, start_new_session=True)
        except (OSError, ValueError) as exc:
            self._set_generation_status(tr('语义地图生成失败：') + str(exc))
            return
        self.generate_button.state(['disabled'])
        self._poll_semantic_generation(title, bag)

    def _semantic_help(self):
        messagebox.showinfo(tr('语义生成说明'), tr(
            '栅格底图可选；物品三维定位需要有效深度、相机内外参和机器人位姿。\n'
            '没有底图时使用 odom 坐标，位置可能随里程计累计漂移。\n'
            '生成时显示当前阶段和已用时间；动态进度条表示任务仍在运行，不代表完成百分比。\n'
            '结果保存在当前录制目录，可从“地图图库”或“更多 → 结果目录”打开。'), parent=self)

    def _set_generation_status(self, text, running=False):
        self.var_generation.set(text)
        self.semantic_status.pack(after=self.semantic_bar, fill='x', padx=12, pady=(0, 2))
        if running:
            if not self.semantic_progress.winfo_manager():
                self.semantic_progress.pack(side='left', before=self.semantic_status_label, padx=(0, 8))
                self.semantic_progress.start(15)
        else:
            self.semantic_progress.stop()
            self.semantic_progress.pack_forget()

    def _poll_semantic_generation(self, title, bag):
        self._semantic_poll_job = None
        try:
            run = read_json(os.path.join(self._semantic_output, 'run.json'))
        except (OSError, ValueError):
            run = {}
        code = self._semantic_proc.poll()
        if code is None:
            labels = {'sample': tr('正在配对并检查定位数据'),
                      'detect': tr('正在识别'), 'fuse_render': tr('正在融合并保存')}
            stages = run.get('stages', [])
            stage = stages[-1]['name'] if stages else None
            elapsed = max(0, int(time.monotonic() - self._semantic_started))
            clock = '%d:%02d' % (elapsed // 60, elapsed % 60)
            self._set_generation_status('%s · %s · %s' % (
                title, labels.get(stage, tr('正在准备生成')), tr('已用 %s') % clock), running=True)
            self._semantic_poll_job = self.after(500, self._poll_semantic_generation, title, bag)
            return
        self._semantic_proc = None
        self.semantic_progress.stop()
        self.semantic_progress.pack_forget()
        self.generate_button.state(['!disabled'])
        if code == 0 and run.get('state') == 'complete':
            discovered = discover_runs(bag)
            known = {result['path']: result for result in self._semantic_runs}
            known.update((result['path'], result) for result in discovered)
            self._semantic_runs = list(known.values())
            self._refresh_semantic_choices(select=self._semantic_output)
            if run.get('gallery_error'):
                text = tr('%s 已生成，图库保存失败。可在更多菜单重试最终地图。') % title
            else:
                text = tr('%s 已生成，可在地图图库查看。') % title
            if run.get('world_frame') == 'odom':
                text += ' ' + tr('坐标：odom。')
            bounds = (run.get('geometry', {}).get('map') or {}).get('record_bounds') or []
            if any(value is not None for value in bounds):
                text += ' ' + tr('结果覆盖底图的有效录制片段。')
            self._set_generation_status(text)
            return
        error = run.get('error')
        if not error:
            with open(self._semantic_log, 'rb') as log:
                log.seek(max(0, os.fstat(log.fileno()).st_size - 2000))
                lines = log.read().decode('utf-8', 'replace').strip().splitlines()
                error = lines[-1] if lines else ''
        self._set_generation_status(tr('语义地图生成失败：') + error + '\n' + self._semantic_log)

    def _redraw_semantic(self):
        if self._last.get('semantic') and not self._last.get('err'):
            self._apply(self._last)

    def _toggle_other_classes(self):
        if self.var_other_classes.get():
            self.var_boxes.set(True)
        if self._last.get('semantic'):
            self._redraw_semantic()
        else:
            self._update_semantic_note(None)

    def _update_semantic_note(self, semantic):
        note = ''
        if semantic and not semantic.get('sample'):
            note = tr('此处没有临近语义采样帧；拖动时间轴查看其他位置。')
        elif semantic:
            note = tr('模型推理框（非人工标注）：绿已融合、蓝待融合、橙深度不足、灰排除。地图：绿为本帧关联、蓝为其他候选；全会话融合，非物体中心。')
            if self.var_other_classes.get() and self.var_boxes.get():
                extras = sum(not d.get('included', True) for d in semantic['detections'])
                details = tr('本帧显示 %d 个其它类别框。') % extras if extras else tr('本帧没有其它类别的检测框。')
                note = details + '  ' + note
        elif self.var_other_classes.get():
            note = tr('选择语义回放结果后，可查看其它类别的检测框。')
        self.var_semantic_note.set(note)

    def _on_slide(self, _v):
        """拖动中：只更新文字，最多每 150 ms 真的读一帧。

        以前每移动一像素就 seek 一次，一秒能发几十次请求；每次都新建两个
        Tk 图像对象（820x520 + 340x210），X 服务端的图像资源很快被刷爆，
        进程直接 SIGBUS —— 就是那个 Bus error。
        """
        if self._setting_sl:
            return
        if self._playing:self.toggle_play()
        t = float(self.var_sl.get())
        self.var_t.set('%.1f / %.1f s' % (t, self.dur))
        now = time.time()
        if now - self._last_seek > 0.15:
            self._last_seek = now
            self._want_t = t

    def toggle_frame(self):
        self.frame_mode = self.world if self.frame_mode == 'base' else 'base'
        self.btn_frame.configure(text=tr('切到车体系') if self.frame_mode != 'base' else tr('切到全局视图'))
        self._force = True

    def _on_slide_release(self, _ev):
        # 松手一定补一次，保证停在用户选的那一帧
        self._last_seek = time.time()
        self.seek(float(self.var_sl.get()))

    def step(self, d):
        self.seek(min(self.dur, max(0.0, self._want_t + d)))

    def toggle_play(self):
        self._playing = not self._playing
        if self._play_job:
            self.after_cancel(self._play_job);self._play_job=None
        if self._playing:
            self._play_wall=time.monotonic();self._play_t=self._want_t
            self._tick_play()

    def _tick_play(self):
        self._play_job=None
        if not self._playing or self._closing:return
        t=self._play_t+time.monotonic()-self._play_wall
        if t>=self.dur:
            self._playing=False;self.seek(self.dur);return
        self.seek(t, from_play=True)
        self._play_job=self.after(66,self._tick_play)

    def seek(self,t,from_play=False):
        self._want_t=max(0.,min(self.dur,t))
        if self._playing and not from_play:
            self._play_wall=time.monotonic();self._play_t=self._want_t
        self.var_t.set('%.1f / %.1f s'%(self._want_t,self.dur) if self.dur else '-- / --')
        self._setting_sl=True
        self.var_sl.set(self._want_t)
        self._setting_sl=False

    # ------------------------------------------------------------ 读帧
    def _worker(self):
        try:
            info = json.loads(self.proc.stdout.readline())
            self._q.put({'init': info})
            self.world = info.get('world', 'base')
            self.frame_mode = self.world
        except Exception as exc:
            detail = ''
            try:
                self._reader_log.seek(0)
                detail = self._reader_log.read().decode('utf-8','replace')[-600:]
            except (OSError,ValueError):pass
            self._q.put({'err': tr('初始化失败：')+(detail or str(exc))})
            return
        last = None
        while not self._closing:
            t = self._want_t
            force, self._force = self._force, False
            if not force and last is not None and abs(t - last) < 1e-3:
                time.sleep(0.04)
                continue
            last = t
            try:
                self.proc.stdin.write(json.dumps({'t': t,
                                                  'record_time': self._record_t0+t,
                                                  'semantic_run': self._semantic_run,
                                                  'frame': self.frame_mode}) + '\n')
                self.proc.stdin.flush()
                line = self.proc.stdout.readline()
                if not line:
                    break                     # 服务端已经退出，别再空转
                d = json.loads(line)
            except Exception as e:
                d = {'err': str(e)}
            self._frame_applied.clear()
            self._q.put(d)
            while not self._closing and not self._frame_applied.wait(.1):
                pass

    def _drain(self):
        if self._closing:
            return
        try:
            while True:
                d = self._q.get_nowait()
                try:
                    self._apply(d)
                finally:
                    self._frame_applied.set()
        except queue.Empty:
            pass
        self._drain_job = self.after(16, self._drain)

    def _apply(self, d):
        if d.get('semantic_error'):
            self._semantic_run = d.get('semantic_run')
            paths = [r['path'] for r in self._semantic_choices]
            self.semantic_picker.current(paths.index(self._semantic_run)+1 if self._semantic_run in paths else 0)
            self.var_semantic_note.set(tr('语义结果加载失败：')+d['err'])
            return
        if 'init' in d:
            info = d['init']
            self._semantic_runs = info.get('semantic_runs', [])
            self._refresh_semantic_choices()
            self._update_context(info)
            self._load_phase=tr('正在准备第一帧')
            self._set_controls(True)
            if self._initial_preset:
                preset, self._initial_preset = self._initial_preset, None
                self._generate_semantic(preset)
            return
        context = d.pop('context', None)
        if context:
            self._update_context(context)
            self.seek(d['record_time']-self._record_t0)
        semantic=d.get('semantic')
        if (not context and not d.get('err') and d is not self._last and semantic and semantic.get('sample') is not None
                and d.get('semantic_run')==self._last.get('semantic_run')
                and d.get('frame')==self._last.get('frame')
                and semantic.get('sample')==self._last.get('semantic',{}).get('sample')):
            self._last=d  # Playback within one cached sample needs no image/font work.
            return
        self._last = d
        self._update_semantic_note(semantic)
        self.lbl_scan_title.configure(text={
            'map': tr('地图 · 当前绿 / 历史灰'),
            'odom': tr('里程计 · 当前绿 / 历史灰'),
            'base': tr('车体雷达（6 m 内，车头朝上）')}.get(d.get('frame'), tr('雷达')))
        if d.get('err'):
            if self._loading:self._loading_failed(d['err'])
            self.var_info.set(tr('读取失败：') + d['err'])
            self.cv.delete('all')
            return
        if self._loading or self._load_error:
            self._loading=False;self._load_error=False;self.spinner.stop()
            self.loading_frame.pack_forget()
        try:
            from PIL import Image
            if d.get('rgb'):
                self._export_rgb = Image.open(d['rgb']).convert('RGB')
                if semantic and self.var_boxes.get():
                    self._export_rgb = draw_detections(self._export_rgb, semantic['detections'], self.var_other_classes.get())
            else:
                self._export_rgb = None
                self.lbl_rgb.configure(image='', text=tr('当前时刻无彩色帧'))
            if d.get('depth'):
                self._export_depth = Image.open(d['depth']).convert('RGB')
                if semantic and self.var_boxes.get():
                    self._export_depth = draw_detections(self._export_depth, semantic['detections'], self.var_other_classes.get(), labels=False)
            else:
                self._export_depth = None
                self.lbl_depth.configure(image='', text=tr('当前时刻无深度帧'))
            self._render_images()
        except Exception as e:
            self.var_info.set(tr('解码失败：%s') % e)
            return

        pose = d.get('pose')
        bits = [{'map': tr('地图视图'), 'odom': tr('里程计视图（非地图定位）'), 'base': tr('车体系')}.get(d.get('frame'), ''), 't=%.2fs' % d.get('t', 0)]
        if semantic:
            bits.append(tr('%s · 采样回放帧 %s') % (semantic['model'], semantic.get('sample') or '--'))
            bits.append(tr('紫色深度：无效测距'))
        stamps = d.get('stamps', {})
        if 'rgb' in stamps:
            bits.extend('%s-RGB %+.0fms' % (k, (stamps[k]-stamps['rgb'])*1000) for k in ('depth', 'scan') if k in stamps)
        if pose:
            bits.append(tr('位姿 (%.2f, %.2f) %.0f°') % (pose[0], pose[1], pose[2]))
        scan = d.get('scan')
        bits.append(tr('雷达 %d 点') % len(scan) if scan else tr('无雷达'))
        bits.append(tr('深度 ok') if d.get('depth') else tr('无深度'))
        self.var_info.set('   '.join(bits))

        if d.get('frame') in ('map', 'odom') and pose:
            if self._last_map_t is not None and d['t'] < self._last_map_t:
                self._map_img = None
            self._last_map_t = d['t']
            self._accumulate(scan, pose)
            self._draw_map_view(pose, scan)
        else:
            self._draw_scan(scan)

    def _update_context(self, info):
        self._record_t0 = info.get('t0', 0.)
        self.dur = info.get('dur', 0.)
        self.var_t.set('%.1f / %.1f s'%(self._want_t,self.dur))
        self.slider.configure(to=max(.1, self.dur))
        self.has_map = info.get('has_map', False)
        self.map_info = info.get('map_info')
        if info.get('map_note') or info.get('warning'):
            self.var_map_note.set(info.get('map_note') or info.get('warning'))
            self.map_note_label.pack(fill='x',padx=12,before=self.loading_frame)
        else:
            self.map_note_label.pack_forget()
        if info.get('warning'):
            self.title(tr('查看：%s（地图关联无效，已降级）') % self.bag)
        self.world = info.get('world', 'base')
        self.frame_mode = self.world
        self._map_img = None
        self._map_panel = None
        self._last_map_t = None
        self._map_overview=True
        self._viewports.clear()
        self.btn_frame.configure(text=tr('切到车体系') if self.world != 'base' else tr('切到全局视图'))

    @staticmethod
    def _fit(im, w, h):
        """缩放到固定尺寸并居中留边 —— 复用 PhotoImage 要求每帧尺寸一致。"""
        from PIL import Image
        im = im.convert('RGB')
        scale=min(float(w)/im.width,float(h)/im.height)
        im=im.resize((max(1,int(im.width*scale)),max(1,int(im.height*scale))),Image.BILINEAR)
        box = Image.new('RGB', (w, h), (17, 17, 17))
        box.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
        return box

    MAP_PPM = 20.0        # 地图画布分辨率：像素/米
    MAP_HALF = 30.0       # 覆盖 ±30 m

    def _map_ensure(self, pose):
        if self._map_img is not None:
            return
        from PIL import Image, ImageDraw
        self._grid_bounds=None
        self._history_bounds=None
        side = int(self.MAP_HALF * 2 * self.MAP_PPM)
        self._map_img = Image.new('RGB', (side, side), (13, 17, 23))
        self._map_draw = ImageDraw.Draw(self._map_img)
        self._map_ox = pose[0] - self.MAP_HALF
        self._map_oy = pose[1] - self.MAP_HALF
        if self.map_info and self.frame_mode == 'map':
            try:
                source = Image.open(self.map_info['image']).convert('RGB')
                self._grid_bounds=grid_bounds(self.map_info,source)
                res = float(self.map_info['resolution'])
                ox, oy, yaw = self.map_info['origin']
                c, sn = math.cos(yaw), math.sin(yaw)
                scale = self.MAP_PPM
                # Map raster rows run downward, world y runs upward.
                wx, wy = self._map_ox-ox, self._map_oy+self.MAP_HALF*2-oy
                affine = (c/(scale*res), -sn/(scale*res), (c*wx+sn*wy)/res,
                          sn/(scale*res), c/(scale*res), source.height-(-sn*wx+c*wy)/res)
                self._map_img = source.transform((side, side), Image.AFFINE, affine,
                                                 resample=Image.NEAREST)
                self._map_draw = ImageDraw.Draw(self._map_img)
            except Exception as exc:
                self.var_info.set(tr('地图背景加载失败：')+str(exc))

    def _w2p(self, x, y):
        return ((x - self._map_ox) * self.MAP_PPM,
                self._map_img.height - (y - self._map_oy) * self.MAP_PPM)

    def _accumulate(self, pts, pose):
        """把这一帧的点画进累积图 —— 拖过的扫描会一直留着。

        历史画灰色、当前帧在 _draw_map_view 里画绿色：这样"绿"只出现在此刻这
        一帧上，拖时间轴时一眼就能看出哪一圈是"现在"。
        """
        self._map_ensure(pose)
        if not pts:
            return
        d = self._map_draw
        w, h = self._map_img.size
        for x, y in pts:
            px, py = self._w2p(x, y)
            if 0 <= px < w and 0 <= py < h:
                d.rectangle([px, py, px + 1, py + 1], fill=HISTORY_GREY)
        self._map_n += len(pts)
        bounds=point_bounds(list(pts)+[(pose[0],pose[1])])
        if self._history_bounds:
            old=self._history_bounds;bounds=(min(old[0],bounds[0]),min(old[1],bounds[1]),max(old[2],bounds[2]),max(old[3],bounds[3]))
        self._history_bounds=bounds

    def _draw_map_view(self, pose, current=None):
        from PIL import Image,ImageDraw,ImageTk
        semantic=self._last.get('semantic')
        bounds=semantic.get('bounds') if semantic else None
        if not bounds:
            points=[(o['x'],o['y']) for o in semantic['objects']] if semantic else []
            bounds=point_bounds(points+[(pose[0],pose[1])]) if points else (self._grid_bounds or self._history_bounds or (pose[0]-5,pose[1]-5,pose[0]+5,pose[1]+5))
        for canvas,attribute in ((self.cv,'_ph_map'),(self._large_canvas,'_ph_map_large')):
            if canvas is None:continue
            cw,ch=self._canvas_size(canvas)
            view=(Viewport.fit(bounds,cw,ch) if self._map_overview else
                  Viewport(self._map_centre or pose[:2],self._map_span,cw,ch))
            self._viewports[canvas]=view
            k=self.MAP_PPM/view.scale
            affine=(k,0,(view.left-self._map_ox)*self.MAP_PPM,
                    0,k,self._map_img.height-(view.top-self._map_oy)*self.MAP_PPM)
            image=self._map_img.transform((cw,ch),Image.AFFINE,affine,resample=Image.BILINEAR)
            draw=ImageDraw.Draw(image)
            for x,y in current or ():
                p=view.pixel(x,y)
                if p:draw.rectangle((p[0]-1,p[1]-1,p[0]+1,p[1]+1),fill=CURRENT_GREEN)
            if semantic and self.var_points.get() and self.frame_mode==semantic['world']:
                active=set(d['object_number'] for d in semantic['detections'] if d.get('object_number'))
                draw_objects(image,view,semantic['objects'],active,size=max(11,min(20,int(min(cw,ch)/40))))
            p=view.pixel(pose[0],pose[1])
            if p:
                a=math.radians(pose[2]);ca,sa=math.cos(a),math.sin(a);x,y=p;length=14
                draw.polygon(((x+length*ca,y-length*sa),
                              (x-length*.6*ca+length*.45*sa,y+length*.6*sa+length*.45*ca),
                              (x-length*.6*ca-length*.45*sa,y+length*.6*sa-length*.45*ca)),fill=(245,158,11),outline=(180,83,9))
            photo=getattr(self,attribute)
            if photo is None or (photo.width(),photo.height())!=(cw,ch):
                photo=ImageTk.PhotoImage(image);setattr(self,attribute,photo)
            else:photo.paste(image)
            canvas.delete('all');canvas.create_image(0,0,anchor='nw',image=photo)
            if canvas==self.cv:
                self._map_panel=image.copy();self.var_map_scale.set(tr('范围 %.1f m')%view.span)

    def _draw_scan(self, pts):
        self._draw_scan_canvas(pts,self.cv)
        if self._large_canvas:self._draw_scan_canvas(pts,self._large_canvas)

    def _draw_scan_canvas(self, pts, canvas):
        """雷达俯视图：车在中心、车头朝上，2/4/6 m 距离环。"""
        canvas.delete('all')
        w,h=self._canvas_size(canvas)
        cx, cy = w / 2.0, h / 2.0
        scale = min(w, h) / 2.0 / 6.0        # 6 m 铺满

        for r in (2, 4, 6):
            canvas.create_oval(cx - r * scale, cy - r * scale,
                                cx + r * scale, cy + r * scale,
                                outline='#1b2534')

        if pts:
            for x, y in pts:
                px = cx + x * scale
                py = cy - y * scale
                canvas.create_rectangle(px, py, px + 2, py + 2,
                                         outline='', fill='#4ade80')
        else:
            canvas.create_text(cx, cy + 26, text=tr('无雷达数据'), fill='#555')

        # 只画一个三角形：点已经按 /tf 转到车体系，屏幕上方就是车头，
        # 再加箭头反而多余。
        canvas.create_polygon(cx, cy - 9, cx - 8, cy + 8, cx + 8, cy + 8,
                               fill='#f59e0b', outline='#b45309')

    def export_combo(self):
        """把当前帧的 彩色 / 深度 / 雷达 拼成一张 PNG —— 标注和报告直接用。"""
        d = self._last
        if not d or d.get('err'):
            messagebox.showinfo(tr('还没有帧'), tr('先拖一下时间轴。'))
            return
        try:
            from PIL import Image, ImageDraw
        except Exception as e:
            messagebox.showerror(tr('缺 PIL'), str(e))
            return

        W, H, HEAD, FOOT = 420, 315, 46, 34
        canvas = Image.new('RGB', (W * 3, H + HEAD + FOOT), (238, 241, 244))
        dr = ImageDraw.Draw(canvas)

        bag = self.bag
        # PIL 的默认位图字体只有 latin-1，中文会抛 UnicodeEncodeError，
        # 所以标题自己拼 ASCII，不要直接塞 var_info（里面有"位姿/雷达"）。
        pose = d.get('pose')
        cap = 't=%.2fs' % d.get('t', 0)
        if pose:
            cap += '   pose (%.2f, %.2f) %.0f deg' % (pose[0], pose[1], pose[2])
        cap += '   lidar %d pts' % len(d.get('scan') or [])
        semantic = d.get('semantic')
        if semantic:
            cap += '   %s sample %s | inference / full-session fusion' % (semantic['model'], semantic.get('sample'))
        dr.text((10, 8), bag, fill=(20, 20, 20))
        dr.text((10, 26), cap, fill=(90, 100, 110))

        def fit(im, w, h):
            im = im.convert('RGB')
            im.thumbnail((w, h))
            box = Image.new('RGB', (w, h), (17, 17, 17))
            box.paste(im, ((w - im.width) // 2, (h - im.height) // 2))
            return box

        # 彩色
        if d.get('rgb'):
            canvas.paste(fit(self._export_rgb.copy(), W, H), (0, HEAD))
        else:
            canvas.paste(fit(Image.new('RGB', (W, H), (17, 17, 17)), W, H), (0, HEAD))

        # 深度
        if d.get('depth'):
            canvas.paste(fit(self._export_depth.copy(), W, H), (W, HEAD))

        # 雷达俯视
        panel = Image.new('RGB', (W, H), (13, 17, 23))
        pd = ImageDraw.Draw(panel)
        cx, cy = W / 2.0, H / 2.0
        scale = min(W, H) / 2.0 / 6.0
        for r in (2, 4, 6):
            pd.ellipse([cx - r * scale, cy - r * scale,
                        cx + r * scale, cy + r * scale], outline=(27, 37, 52))
        for x, y in (d.get('scan') or []):
            px, py = cx + x * scale, cy - y * scale
            pd.rectangle([px, py, px + 2, py + 2], fill=(74, 222, 128))
        pd.line([cx, cy - 30, cx, cy - 10], fill=(245, 158, 11), width=2)
        pd.polygon([cx, cy - 8, cx - 7, cy + 8, cx + 7, cy + 8], fill=(245, 158, 11))
        if d.get('frame') in ('map', 'odom') and self._map_panel is not None:
            panel = fit(self._map_panel.copy(), W, H)
        canvas.paste(panel, (W * 2, HEAD))

        for i, name in enumerate(('RGB', 'DEPTH', 'LIDAR')):
            dr.text((i * W + 10, HEAD + H + 8), name, fill=(90, 100, 110))

        out_dir = os.path.join(self.folder, 'exports')
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        suffix = '_'+os.path.basename(semantic.get('run_id',semantic['model'])) if semantic else ''
        out = os.path.join(out_dir, '%s_t%05.1f%s.png' % (bag[:-4], d.get('t', 0), suffix))
        canvas.save(out)
        self.var_info.set(tr('已导出 ') + out)

    def close(self):
        self._closing = True
        self.spinner.stop()
        self.semantic_progress.stop()
        self._close_map_window()
        if self._export_pool:self._export_pool.shutdown(wait=False)
        for job in (self._load_job,self._drain_job,self._play_job,self._resize_job,self._export_poll_job,self._layout_job,self._semantic_poll_job):
            if job:
                try:self.after_cancel(job)
                except tk.TclError:pass
        # 先好好说再见，给它时间自己退；直接 terminate 会让它正写到一半时
        # 撞上 Broken pipe（旧版本就是这么刷 traceback 的）。
        try:
            self.proc.stdin.write('{"cmd":"close"}\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=1.0)
        except Exception:
            pass
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=3)
            except Exception:
                pass
        if self._reader_thread is not None:self._reader_thread.join(timeout=1)
        if self.proc is not None:
            for stream in (self.proc.stdin,self.proc.stdout):
                try:stream.close()
                except (OSError,ValueError):pass
        self._reader_log.close()
        self.destroy()


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DIR
    root = tk.Tk()
    App(root, folder)
    root.mainloop()


if __name__ == '__main__':
    main()
