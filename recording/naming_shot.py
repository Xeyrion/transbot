# -*- coding: utf-8 -*-
"""截图验收：录制台的"会话名"，会话列表的"重命名"。

    xvfb-run -a python3 naming_shot.py record /tmp/a.png
    xvfb-run -a python3 naming_shot.py catalog /tmp/b.png
"""
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import tkinter as tk                      # noqa: E402

MODE = sys.argv[1] if len(sys.argv) > 1 else 'record'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/shot.png'


def shoot(root):
    root.update_idletasks()
    root.update()
    os.system('import -window root %s' % OUT)
    root.after(300, root.destroy)


def record_mode():
    import record_gui
    root = tk.Tk()
    root.geometry('1180x740+0+0')
    app = record_gui.App(root, record_gui.Remote('jetson@192.168.1.11', 'yahboom'))
    app.var_name.set(u'走廊建图')
    root.after(2500, lambda: shoot(root))
    root.mainloop()


def catalog_mode():
    import bag_quality_gui
    demo = tempfile.mkdtemp(prefix='sessions_')
    for name, n in [('session_20261111_014312_走廊建图', 3),
                    ('session_20261111_015530', 1),
                    ('session_20261112_004052_MZgKPg', 2),
                    ('session_20261112_005522_FkhF1N', 2)]:
        os.makedirs(os.path.join(demo, name))
        for i in range(n):
            open(os.path.join(demo, name, 'cam_%s_%d.bag' % (name[8:22], i)), 'wb').close()
    root = tk.Tk()
    root.geometry('1100x620+0+0')
    app = bag_quality_gui.App(root, demo)
    root.update()
    kids = app.catalog.get_children()
    if kids:
        app.catalog.selection_set(kids[0])
    root.after(2000, lambda: shoot(root))
    root.mainloop()
    shutil.rmtree(demo, ignore_errors=True)


if __name__ == '__main__':
    (record_mode if MODE == 'record' else catalog_mode)()
