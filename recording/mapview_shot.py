# -*- coding: utf-8 -*-
"""截图：地图系里"当前帧（亮绿）"和"已浏览扫描（暗绿）"的区分。

    xvfb-run -a python3 mapview_shot.py <bag> <out.png>
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import tkinter as tk                      # noqa: E402
import bag_quality_gui                    # noqa: E402

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/mapview.png'
FOLDER = os.path.dirname(os.path.abspath(BAG))


def pump(root, cond, seconds=20):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        root.update()
        time.sleep(.01)
        if cond():
            return True
    return False


def main():
    root = tk.Tk()
    root.geometry('1180x760+0+0')
    v = bag_quality_gui.BagViewer(root, FOLDER, os.path.basename(BAG))
    assert pump(root, lambda: not v._loading), 'viewer did not finish loading'
    print('dur=%.2f world=%s frame=%s' % (v.dur, v.world, v.frame_mode))
    # 隔开一点走，让"历史"和"当前"真的错开（机器人不动的话两层会重合）
    for frac in (0.02, 0.20, 0.40, 0.60, 0.80, 0.95):
        v.seek(v.dur * frac)
        pump(root, lambda: not v._loading, 8)
        root.update()
        time.sleep(.25)
        root.update()
        pose = (v._last or {}).get('pose')
        print('  t=%.2f pose=%s' % (v.dur * frac, pose))
    root.update_idletasks()
    root.update()
    # BagViewer 是 Toplevel，裸 Xvfb 没有窗口管理器，截 root 只会得到空桌面
    v.deiconify()
    v.lift()
    v.update_idletasks()
    v.update()
    time.sleep(.4)
    root.update()
    print('mapped=%s geom=%s id=0x%x' % (v.winfo_ismapped(), v.winfo_geometry(), v.winfo_id()))
    os.system('import -window 0x%x %s' % (v.winfo_id(), OUT))
    print('title:', v.lbl_scan_title.cget('text'))
    print('info:', v.var_info.get())
    print('label w=%d req=%d' % (v.lbl_scan_title.winfo_width(),
                                 v.lbl_scan_title.winfo_reqwidth()))
    # 像素统计：证明"历史"和"当前"两层真的都画上了，不是只有一层
    hist = sum(1 for px in v._map_img.getdata()
               if px == bag_quality_gui.HISTORY_GREY)
    cur = sum(1 for px in v._map_panel.getdata()
              if px == bag_quality_gui.CURRENT_GREEN)
    print('history_px=%d current_px=%d' % (hist, cur))
    root.after(200, root.destroy)
    root.mainloop()


if __name__ == '__main__':
    main()
