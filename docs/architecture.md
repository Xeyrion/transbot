# Architecture and the traps behind it

## Three processes

```
                ┌──────────────────────────────┐
                │ bag_quality_gui.py   (py3)   │
                │  tkinter table + viewer      │
                └───────┬──────────────┬───────┘
        one-shot JSON   │              │  {"t": 12.3}  /  one JSON line
                        ▼              ▼
        ┌───────────────────────┐  ┌──────────────────────────────┐
        │ bag_quality_probe.py  │  │ bag_viewer_server.py  (py2)  │
        │ (py2) metrics per bag │  │ keeps ONE bag open, seeks    │
        └───────────────────────┘  └──────────────────────────────┘
```

Both helpers are Python 2 because Melodic's `rosbag` is Python 2 only. The GUI
never imports `rosbag`; it parses JSON instead.

## Why the viewer server is long-lived

Reopening a bag means re-reading its index and the first chunk — around a second
for a 2 GB take, more on a spinning disk. Dragging a slider would then lag a
second behind the mouse. Keeping the process alive makes each seek cost only the
chunk decompression (0.2–0.5 s).

Frames are written to fixed paths (`/tmp/bagview/rgb.ppm`, `depth.pgm`) and
overwritten on each seek. The GUI reads them right after the reply arrives, so
there is no race: one request in flight at a time, driven by a worker thread that
always sends the newest requested time.

## Traps

**Python 2 buffers `for line in sys.stdin`.** The file iterator reads ahead in
8 KB blocks, so on a pipe that stays open the first line is never delivered — the
GUI waits forever for its first frame. It works fine when tested as
`printf … | python2 server.py`, because EOF flushes the buffer, which is exactly
how a working server can look broken. Use `while True: line = sys.stdin.readline()`.

**Emoji in Tk labels can kill the window.** `⏮ ◀ ▶ ⏸` on a button whose font has
no glyph for them makes Tk throw `X Error: BadLength (RenderAddGlyphs)` and the
whole window dies. `|<` `<` `>` `>|` never does.

**`stretch=True` is not "responsive".** It hands the column *all* the leftover
width — a bag-name column grew to 1220 px while eight numeric columns stayed at
78 px. The layout instead sets every column fixed and computes widths from the
tree width: bag name capped at 520, the rest split evenly.

**Changing column widths inside `<Configure>` oscillates.** Setting a width can
trigger another Configure; two handlers fight and settle on nonsense (columns
10 px wide). Recompute only when the window width changed by more than 40 px, and
compute from the width argument, never from the result of the previous pass.

**`winfo_width()` is 1 before the window is mapped.** Drawing a canvas from it
puts the centre at the corner. Use `cget('width')` for a canvas whose size you
set yourself.

**Packing order decides who gets squeezed.** A widget packed with
`fill='both', expand=True` before its siblings swallows the space, and the
right-hand panel collapses to a stripe. Pack the fixed-width side first, the
expanding table last.

**A closed pipe is not an error.** Python ignores `SIGPIPE`, so when the GUI goes
away the next `sys.stdout.write` raises `IOError: [Errno 32] Broken pipe` instead of
killing the process — and an uncaught one prints a traceback every single time you
close the window. `send()` in `bag_viewer_server.py` catches it, returns `False`, and
the loop breaks so the bag gets closed. The GUI also says goodbye (`{"cmd":"close"}`)
and waits up to a second before falling back to `terminate()`, otherwise it kills the
reader mid-write and produces the same error from the other side.

**Dragging the slider killed the process with `Bus error`.** Every pixel of drag
fired a seek, and every seek built two new `ImageTk.PhotoImage` objects
(880×560 and 340×210). A Tk image is an X server resource, not just Python
memory: at ~40 images per second they pile up faster than the collector releases
them and the X server eventually fails the client outright — SIGBUS, no Python
traceback, nothing in the log. The slider now throttles to about 6 seeks per
second while dragging and always seeks once on release, and each panel reuses a
single `PhotoImage` (`paste()` into a fixed-size canvas) instead of allocating a
new one per frame.

**The lidar is not mounted pointing forward.** `/base_link -> /laser` in all three
takes carries `yaw = -180°` (and a small `-0.03, 0, 0.13` offset): the lidar's +x
points at the back of the car. Drawing the raw `/scan` therefore puts the car's
rear at the top of the plot. `bag_viewer_server.py` reads that transform from the
bag's `/tf` at startup and rotates every point into `base_link`, so "up" is the
car's nose. Override it when a bag's TF looks wrong:

```bash
BAGVIEW_LASER_YAW=-90 python2 bag_viewer_server.py some.bag
```

The window title shows the value that was applied, so there is never a question
of which frame you are looking at.

## Recording-side traps

These bite on the robot, not in the GUI, and they all look like "the script did
nothing" from a distance.

**`set -u` plus ROS `setup.bash` is a silent exit.** ROS's setup scripts test
variables that may be unset (`CMAKE_PREFIX_PATH`, `ROS_DISTRO`, …). Under
`set -euo pipefail` the `source` aborts the script *immediately*, before any of
its own output — so `auto_record_cam.sh -c` printed nothing and returned 1, with
no error message anywhere. Any script that sources ROS must bracket it:

```bash
set +eu
source /opt/ros/melodic/setup.bash
set -eu
```

**The Astra driver stops streaming and does not come back.** `astra_camera_node`
starts the stream when something subscribes and stops it when the last subscriber
leaves. It does not reliably restart for the next subscriber. The result is a
bag with one `camera_info` and zero images, and a `rostopic list` that looks
completely healthy. `restart_camera.sh` kills the node and relaunches until a
frame actually arrives; `auto_record_cam.sh` probes each `REQUIRED` topic for one
message before recording, so this fails loudly instead of producing an empty bag.

**`getent hosts Transbot` returns only an IPv6 link-local address.** The Jetson's
hostname resolves to `fe80::…`, which nothing can reach, so `rospy.init_node()`
hangs until it times out with `ROSInitException: init_node interrupted before it
could complete`. Setting `ROS_IP=192.168.1.11` fixes it; both `record_full.sh` and
`auto_record_cam.sh` now default `ROS_IP` to the first IPv4 address.

**The clock is wrong after every reboot.** No RTC battery, no NTP on the hotspot:
the robot came up on 2021-08-26 while the control PC was on 2026-11-11, so every
bag name and stamp was five years off. Check `date` on both sides before a
session.

**`pkill -f <pattern>` kills the shell that runs it.** The pattern appears in the
`bash -c` command line of the very command doing the killing, so an SSH session
dies with status 255 and no output. Use a bracket (`astrapro[.]launch`) *and*
make sure the literal string does not also appear elsewhere in the same command
line — or better, put the logic in a script file on the robot and call that.

**A lock that is only read is not a lock.** `remote_record.sh start` checked
`/tmp/transbot_record.pid` and removed stale entries, but nothing ever wrote it,
so two starts still produced two recorders. The lock is now taken by
`record_full.sh` itself — it exists from the first second, before the stack is
up, and is validated against `/proc/<pid>/cmdline` so a recycled PID cannot wedge
it shut.
