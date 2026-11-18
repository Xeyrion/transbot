# -*- coding: utf-8 -*-
"""Navigation package for the Transbot console.

    stack.py    launch / stop the three stacks (mapping, navigation, both)
    mapview.py  OccupancyGrid -> PIL -> Tk canvas, plus the 2D TF shim
    panel.py    the notebook tab: mode selector, map controls, canvas, teleop
    launch/     the roslaunch files the panel starts
    param/      planner parameters (copies, tuned; see README)
    helpers/    scan_filter, cmd_vel_scale, clock_guard, amcl_keepalive
"""
