#!/usr/bin/env python3
"""One entry point for recording and reviewing Transbot sessions."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tkinter as tk
from i18n import tr
from tkinter import messagebox

HERE = Path(__file__).resolve().parent
BG = '#eef1f4'


def default_data_dir():
    return os.path.abspath(os.path.expanduser(os.environ.get('BAGTOOLS_DATA_DIR',
        os.environ.get('LOCAL_DIR', '~/robot_research/bagtools/bags'))))


def launch_spec(kind, data_dir):
    env = dict(os.environ, LOCAL_DIR=str(data_dir), BAGTOOLS_DATA_DIR=str(data_dir))
    if kind == 'record':
        return [sys.executable, str(HERE/'recording/record_gui.py')], env
    if kind == 'review':
        return [sys.executable, str(HERE/'bag_quality_gui.py'), str(data_dir)], env
    raise ValueError('Unknown tool: '+kind)


class Workbench:
    def __init__(self, root, data_dir=None):
        self.root = root
        self.data_dir = data_dir or default_data_dir()
        self.children = {}
        self.buttons = {}
        self.closed = False
        root.title(tr('Transbot 数据工作台'))
        root.configure(bg=BG)
        root.geometry('%dx%d' % (min(760, root.winfo_screenwidth()-60),
                                min(420, root.winfo_screenheight()-100)))
        tk.Label(root,text=tr('Transbot 数据工作台'),bg=BG,fg='#203039',
                 font=('',20,'bold')).pack(anchor='w',padx=24,pady=(24,4))
        tk.Label(root,text=tr('从这里开始录制，或查看已回传的会话。'),bg=BG,fg='#6b7b86',
                 font=('',11)).pack(anchor='w',padx=24)
        cards=tk.Frame(root,bg=BG);cards.pack(fill='both',expand=True,padx=20,pady=20)
        cards.rowconfigure(0,weight=1)
        for i,(kind,title,detail,color) in enumerate([
            ('record',tr('录制控制'),tr('采集 · 停止 · 回传'),'#2e7d32'),
            ('review',tr('数据阅读'),tr('选择会话 · 分析 · 回放'),'#1565a3')]):
            cards.columnconfigure(i,weight=1,uniform='tools')
            card=tk.Frame(cards,bg='white',padx=16,pady=16)
            card.grid(row=0,column=i,sticky='nsew',padx=4)
            button=tk.Button(card,text=title,bg=color,fg='white',relief='flat',
                             font=('',15,'bold'),pady=12,
                             command=lambda k=kind:self.open_tool(k))
            button.pack(fill='x');self.buttons[kind]=button
            tk.Label(card,text=detail,bg='white',fg='#6b7b86',font=('',10)).pack(pady=(12,0))
        bottom=tk.Frame(root,bg=BG);bottom.pack(fill='x',padx=24)
        tk.Label(bottom,text=tr('控制机数据目录'),bg=BG,fg='#6b7b86').pack(anchor='w')
        tk.Label(bottom,text=self.data_dir,bg=BG,fg='#203039',anchor='w').pack(side='left',fill='x',expand=True)
        tk.Button(bottom,text=tr('打开数据文件夹'),command=self.open_folder).pack(side='right')
        self.status=tk.StringVar(value=tr('就绪'))
        tk.Label(root,textvariable=self.status,bg=BG,fg='#6b7b86',anchor='w').pack(fill='x',padx=24,pady=(10,2))
        tk.Label(root,text=tr('录制的开始与停止，请在“录制控制”窗口操作。'),bg=BG,fg='#6b7b86',
                 anchor='w').pack(fill='x',padx=24,pady=(0,16))
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.poll_job=root.after(300,self.poll)

    def open_tool(self, kind):
        previous=self.children.get(kind)
        if previous and previous[0].poll() is None:
            self.status.set(tr('窗口已经打开，请切换到对应窗口。'))
            return
        title=tr('录制控制') if kind=='record' else tr('数据阅读')
        self.status.set(tr('正在打开')+title+'…')
        self.root.update_idletasks()
        cache=Path(os.environ.get('BAGTOOLS_CACHE_DIR',str(Path.home()/'.cache/transbot_bagtools')))/'launcher'
        cache.mkdir(parents=True,exist_ok=True)
        log=cache/(kind+'.log')
        command,env=launch_spec(kind,self.data_dir)
        try:
            with log.open('ab') as output:
                process=subprocess.Popen(command,env=env,cwd=str(HERE),stdout=output,
                                         stderr=subprocess.STDOUT,start_new_session=True)
            self.children[kind]=(process,log)
            self.buttons[kind].configure(state='disabled')
            self.status.set(title+tr('窗口已启动。'))
        except OSError as exc:
            self.status.set(tr('打开失败：')+str(exc))

    def poll(self):
        if self.closed:return
        for kind,(process,log) in list(self.children.items()):
            code=process.poll()
            if code is None:continue
            del self.children[kind]
            self.buttons[kind].configure(state='normal')
            if code:
                self.status.set(tr('窗口启动或运行失败，日志：')+str(log))
                messagebox.showerror(tr('窗口运行失败'),log.read_text(errors='replace')[-1000:],parent=self.root)
            else:self.status.set(tr('窗口已关闭，可再次打开。'))
        self.poll_job=self.root.after(300,self.poll)

    def open_folder(self):
        os.makedirs(self.data_dir,exist_ok=True)
        if not shutil.which('xdg-open'):
            self.status.set(tr('数据目录：')+self.data_dir);return
        subprocess.Popen(['xdg-open',self.data_dir],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    def close(self):
        self.closed=True
        self.root.after_cancel(self.poll_job)
        # Tool windows and robot recording sessions have their own lifetime.
        self.root.destroy()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',nargs='?',help='compatibility: open a data directory directly')
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--record',action='store_true')
    group.add_argument('--review',action='store_true')
    args=parser.parse_args()
    directory=os.path.abspath(os.path.expanduser(args.folder)) if args.folder else default_data_dir()
    if args.record or args.review or args.folder:
        command,env=launch_spec('record' if args.record else 'review',directory)
        os.execvpe(command[0],command,env)
    root=tk.Tk();Workbench(root,directory);root.mainloop()


if __name__=='__main__':main()
