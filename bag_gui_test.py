#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smoke test for bag_quality_gui: analyze a folder, open the viewer, screenshot.

    DISPLAY=:0 python3 bag_gui_test.py /tmp/bagtest /tmp/shot [map]

The optional third argument 'map' also flips the viewer into map frame, which
only works when the bag actually contains /map.
"""
import os
import subprocess
import sys
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import bag_quality_gui as B  # noqa: E402


def shot(widget, path):
    widget.update()
    subprocess.call(['import', '-window', hex(widget.winfo_id()), path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print('shot ->', path)


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else '/tmp/bagtest'
    outdir = sys.argv[2] if len(sys.argv) > 2 else '/tmp/shot'
    want_map = len(sys.argv) > 3 and sys.argv[3] == 'map'
    os.makedirs(outdir, exist_ok=True)

    root = tk.Tk()
    app = B.App(root, folder)

    def wait_done():
        if app.busy:
            root.after(500, wait_done)
            return
        root.after(800, after_analyze)

    def after_analyze():
        print('board size:', root.winfo_width(), 'x', root.winfo_height())
        shot(root, os.path.join(outdir, '1_board.png'))
        bags = list(app.tree.get_children())
        print('rows:', bags)
        if not bags:
            root.quit()
            return
        viewer = B.BagViewer(root, folder, bags[0])

        def after_viewer():
            print('viewer size:', viewer.winfo_width(), 'x',
                  viewer.winfo_height())
            print('has_map:', getattr(viewer, 'has_map', None))
            shot(viewer, os.path.join(outdir, '2_viewer.png'))
            if want_map and getattr(viewer, 'has_map', False):
                viewer.toggle_frame()
                root.after(4000, after_map)
                return
            if want_map:
                print('bag has no /map, skip frame toggle')
            root.after(300, root.quit)

        def after_map():
            print('frame_mode:', viewer.frame_mode)
            shot(viewer, os.path.join(outdir, '3_map.png'))
            root.after(300, root.quit)

        root.after(9000, after_viewer)

    def kick():
        app.analyze_all()
        root.after(1000, wait_done)

    root.after(600, kick)
    root.mainloop()
    print('done')


if __name__ == '__main__':
    main()
