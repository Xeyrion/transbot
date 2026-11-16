# bagtools — recording quality and review

当前修复与入口说明：[控制机后续修复](docs/repairs_complete_20261115.md)。当前算法定义：[语义地图计算流程](docs/semantic_algorithm.md)。旧状态文档中的能力、限制和测试数字属于历史记录，以这两份说明及本轮交接记录为准。

日常从控制机桌面的 **Transbot 数据工作台** 进入，统一打开录制控制和数据阅读。详见 [统一入口](docs/workbench.md)。

当前目录与性能说明：[会话文件夹、加载提示与性能实测](docs/sessions_and_performance.md)。

2026-11-11: see [采集、默认地图与续传工作流](docs/workflow.md) for current behavior.


Two tools for the multimodal recording work (Astra RGB + depth + RPLIDAR):

1. **Quality board** — point it at a folder of bags and get one line per bag:
   duration, rates, worst gap, RGB/depth pairing, verdict. Answers "is this take
   worth keeping" before you spend an hour on it.
2. **Viewer** — double-click a bag to scrub through it: the RGB frame, the depth
   frame, a top-down lidar plot and the odometry pose at that instant. Export the
   three panels as one PNG for annotation or a report.

```
    ┌───────────────────────────────┐        double-click
    │ bag            dur  rgb  …  │ ─────────────────────┐
    │ cam_…_024943_1 59s 28.0 …   好│                      ▼
    │ cam_…_024932_0 224s 10.7 …  花│        ┌──────────────────────────┐
    └───────────────────────────────┘        │ RGB │ DEPTH │ LIDAR      │
                                             │     │       │            │
                                             └──────────────────────────┘
```

## Layout

```
bagtools/
├── workbench.py            daily entry: recording console + session reader in one window
├── bag_quality_gui.py      quality table + preview + double-click viewer (py3)
├── bag_quality_probe.py    one bag -> one JSON line of metrics (py2)
├── bag_viewer_server.py    long-lived reader, serves frames by time (py2)
├── session_catalog.py      folder scan, session grouping, session naming
├── i18n.py                 tr() + translate_log(): one catalogue for the whole tool
├── locale/en.json          every English string, recorder status lines included
├── run.sh / run_en.sh      launchers: Chinese UI / English UI, same code and data
├── install_workbench.sh    puts both desktop entries on the control machine
├── rebuild_map.sh          offline gmapping for one session (private ROS master)
├── build_maps.sh           walk the bags tree and map every session that lacks one
├── extract_map.py          export the /map already inside a bag (py2, no gmapping)
├── recording/              how the bags get made
│   ├── run.sh                launcher for the recording console
│   ├── remote_record.sh      control-side start / status / stop / pull
│   ├── record_full.sh        robot-side: bring up the stack, then record
│   ├── auto_record_cam.sh    the actual rosbag call (lz4, 2 GB splits)
│   ├── restart_camera.sh     robot-side: revive the Astra driver when it stops streaming
│   ├── record_gui.py         window with live status + keyboard driving
│   ├── record_probe.py       robot-side probe, one JSON line per second
│   ├── pull_session.py       resume + SHA-256 manifests + explicit batch deletion
│   ├── transfer_agent.py     robot-side end of the pull protocol
│   ├── pull_bags.sh          thin shim kept for old commands
│   └── record_session.sh     nav-only session bag (no images)
├── semantic/               what the data is for: objects on a map
│   ├── run_semantic.sh       one command: frames -> detections -> objects -> PNG
│   ├── extract_frames.py     sampled RGB + depth + pose per frame (py2)
│   ├── extract_images.py     sampled RGB only, for a labelling dataset (py2)
│   ├── detect_project.py     YOLO26 / custom weights -> map coordinates (py3)
│   ├── cluster_objects.py    merge repeated sightings into objects + draw (py3)
│   ├── calibrate_camera.py   solve base_link->camera_link from two vertical planes (py3)
│   ├── make_yolo_dataset.py  labelled images -> train/val + data.yaml (py3)
│   ├── pose.py               known static offsets + a minimal 3D TF (py2/py3)
│   ├── LABELING.md           how to label your own classes
│   └── models/               YOLO weights (gitignored)
├── tests/                  unittest suite (46 tests; GUI ones need BAGTOOLS_GUI_TEST=1)
├── bags/                   the data (gitignored)
└── docs/                   metrics, architecture, workflow, semantic_map_plan
```

`recording/` and `semantic/` are siblings on purpose: one produces the bags, the other
consumes them, and both share the same `bags/` tree and session naming. The interpreter
split matters — `rosbag` is Python 2 only, so anything that opens a bag is py2 and
everything downstream is py3. Each script says which one it needs in its header, and
the py2 ones refuse to start under py3 rather than failing halfway through.

## Recording

Recording happens **on the robot** — raw images are 20 MB/s and the WiFi does 1–5 MB/s.
The control PC only sends commands and reads a few hundred bytes of status per second.

```bash
cd recording
./deploy_to_robot.sh                # push record_full/auto_record_cam to the robot first
./remote_record.sh start -d 300     # bring up the stack, record 5 minutes
./remote_record.sh status           # rates, size, disk, log tail
./remote_record.sh stop             # graceful stop, prints the resulting bags
./remote_record.sh pull             # pull everything the pattern matches
```

In the recording console, **拉回校验** and **删除机器人上的包** both open a picker
first: one row per bag with its size and whether a copy already sits in the local
folder (本地无 / 传输中 89% / 本地有 / 已验证 / 尺寸不同). Nothing touches the robot
until you confirm a selection, so a mis-click can no longer sweep a whole folder.
A half-finished `.part` shows up as 传输中 with its percentage — it can be resumed,
but it is not a local copy, so delete mode greys it out.

Pulling and deleting are separate actions on purpose. Delete refuses any bag whose
local copy is missing or fails the SHA-256 re-check, and keeps it on the robot
instead of aborting the run:

```bash
./remote_record.sh pull --names cam_20261112_005544_0.bag          # exactly these
./remote_record.sh pull --names A.bag B.bag --delete-only          # delete, only if safe
./remote_record.sh pull --list-json                                # what the picker reads
```

`-d` still exists for scripted batch work, but it deletes everything the batch
listed — including bags that were skipped because you already had them. The
picker's delete path is the narrower, per-bag one.

What gets recorded (`-p full`):

| Topic | Why |
|---|---|
| `/camera/rgb/image_raw`, `/camera/depth/image_raw` | the two image streams |
| `/camera/*/camera_info` | intrinsics — needed to rebuild a point cloud from depth |
| `/scan` | lidar, 720 beams, frame `laser` |
| `/odom`, `/odom_raw` | wheel odometry (raw and the filtered one) |
| `/imu/data`, `/imu/data_raw` | IMU |
| `/tf`, `/tf_static` | the whole frame tree, including `map → odom` |
| `/cmd_vel` | what was actually commanded — without it there is no (observation, action) pair to train on |
| `/map`, `/map_metadata` | the occupancy grid; the 2026-11-10 takes had `map → odom` but no map, so nothing could be overlaid afterwards |
| `/amcl_pose` | localized pose with covariance, when navigation is running |
| `/voltage` | tells a slow run from a low battery |

The recorder subscribes to the entire configured topic list, including topics
that appear later. Map/localisation startup gets a short grace period when a
SLAM/localisation node is running.

A topic can be listed and still carry nothing. So every `REQUIRED` topic is
probed for one real message before recording starts; if one stays silent for 6 s
the run is refused (`-A` overrides, for bench tests). The same check flags any
topic that has subscribers but no publisher.

Measured on the Jetson Nano with everything running: 25 s of `-p full` is 645 MB,
i.e. about 1.5 GB per minute. RGB drops from 30 Hz to ~19 Hz while `gmapping` is
running — worth knowing when a take looks "slow" for no obvious reason.

When recording finishes, `record_full.sh` also saves the occupancy grid next to
the bag (`map_<stamp>.pgm` + `.yaml`) — cheaper than rebuilding it later.

## Mapping a session that has no map

Plenty of sessions were recorded without SLAM running, so they carry `/scan`,
`/odom` and `/tf` but no `/map`. Those can be mapped offline on the control PC —
no robot involved:

```bash
cd ~/robot_research/bagtools
./build_maps.sh --dry-run          # 先看清单：哪些要建、哪些跳过、为什么
./build_maps.sh                    # 扫 bags/，把还没有地图的会话都建一遍
./build_maps.sh bags/session_x     # 只处理一个会话
```

`build_maps.sh` walks the tree, skips sessions that already have a raster or are
still recording, and calls `rebuild_map.sh` once per remaining session. That tool
decides how to get a map:

- the bag already carries `/map` (SLAM was running) → **export** the last
  occupancy grid to `map_<stamp>.pgm/.yaml`. No gmapping: re-running it would
  fight the `map → odom` already in the bag.
- no `/map` but `/scan` present → **rebuild** through a private ROS master at 2×
  playback speed, writing `<split>.mapping.pgm/.yaml/.bag` plus a `.map.json`
  association.
- neither → skip, and say which one is missing.

A failed attempt cleans up its own half-written artifacts, so the next run
retries it instead of treating it as done. The viewer picks up either form: a
`.map.json` association wins, otherwise `map_*.yaml` next to the bag is used.

Two rules the hard way (see `docs/architecture.md` for the post-mortem):

- **One recorder at a time.** `record_full.sh` holds a PID lock for the whole
  session, including the 10–60 s the stack takes to come up — that window is why
  two starts 12 s apart both went through and fought over the disk.
- **Stop kills every recorder, and never with `-9`.** A `SIGKILL` leaves
  `*.bag.active` behind, which `rosbag info` cannot open; one 1.7 GB take was
  lost that way until `rosbag reindex` recovered 93.9 s of it.

### When the camera goes quiet

The Astra driver stops the stream when its last subscriber disconnects, and does
not always restart it for the next one. Symptoms: the topics exist, `rosbag`
records a single `camera_info` and no images at all. Restart the driver:

```bash
ssh jetson@192.168.1.11 '~/restart_camera.sh'      # stops, relaunches, waits for a frame
```

This is also why `record_full.sh` probes for real data before recording, and why
two back-to-back `auto_record_cam.sh` runs used to produce one good bag and one
empty one.

### Before a session: check the clock

The Jetson has no RTC battery. Compare its UTC clock with the control PC before
a session; use the project clock synchronisation tool rather than setting the
Jetson clock from a `date` command evaluated on the Jetson itself. Changing the
clock while recording invalidates time alignment.

## Why two processes

Melodic's `rosbag` is Python 2 only; the GUI is Python 3. So the GUI never
imports `rosbag` — it shells out:

| Process | Language | Job |
|---|---|---|
| `bag_quality_gui.py` | py3 + tkinter | table, preview, viewer window |
| `bag_quality_probe.py` | py2 | one bag → one JSON line of metrics |
| `bag_viewer_server.py` | py2, **long-lived** | keeps one bag open, answers seek requests |

The viewer server stays alive on purpose: reopening a 2 GB bag costs several
seconds, which makes a scrub slider unusable. It reads `{"t": 12.3}` from stdin
and writes one JSON line back, with the frames dumped to `/tmp/bagview/`.

## Usage

One daily entry: the desktop workbench. It opens the recording console or the
session reader with the same default data root. CLI shortcuts remain available:

```bash
cd ~/robot_research/bagtools
./run.sh                              # unified workbench
./run.sh --review                     # session reader directly
./run.sh /path/to/bags
./run_en.sh                           # same workbench, English UI

cd recording
./run.sh                              # recording console
./run.sh --robot jetson@10.0.0.5
```

`run_en.sh` only exports `BAGTOOLS_LANG=en`; code, data roots and recording logic
are the same files. Recorder output is translated at display time, because the
robot-side scripts run behind ssh where no environment variable follows. See
[`docs/english_demo.md`](docs/english_demo.md).

Quality board:

- **扫描** — list the folder
- **分析全部** — index-only pass; 6 bags / 8 GB take about 12 s
- **只看选中** — re-analyse one bag
- **双击一行** — open the viewer; drag the slider, `<` `>` step 0.5 s, `播放` runs at 15 fps
- **导出三合一** — writes `exports/<bag>_t<time>.png` with RGB / depth / lidar side by side

Recording console:

- **速度滑条** (0.05–0.40 m/s) — keyboard teleop only. Motion driven by
  `move_base` follows `max_vel_x` in the DWA params instead.
- Slower driving genuinely improves depth quality and reduces lidar smear
  (7 Hz sweep = 140 ms per revolution; at 0.45 rad/s the scan is twisted by
  3.6°). Frame rate does not: 30 fps is 30 fps, lowering it only shrinks the bag.
- 0.10–0.15 m/s for mapping-grade data, 0.15–0.20 for general collection.
- **会话名** — optional label for the session folder. Folders are
  `session_<timestamp>[_<label>]`; the label keeps letters, digits, CJK and
  `. _ -`, and the console shows the resulting name while you type. Leave it
  empty and you get the bare timestamp. Rename a finished session from the
  session list; a live one refuses, because the path is already baked into
  rosbag's command line and the `.session.json` sidecar.

## Verdict rules

See [`docs/metrics.md`](docs/metrics.md). In short:

| Verdict | Meaning |
|---|---|
| 好 | rgb ≥ 20 Hz, depth ≥ 25 Hz, lidar ≥ 7 Hz, no gap > 1 s, pairing > 50 ms under 5 % |
| 可用 | rgb ≥ 12 Hz, lidar ≥ 4 Hz, no gap > 3 s |
| 花 | anything else — usually two recorders fighting over the disk |

## What the bags contain

`/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/*/camera_info`,
`/scan` (frame `laser`, 720 beams, ±180°), `/odom`, `/odom_raw`, `/imu/data`,
`/imu/data_raw`, `/tf`, `/tf_static`, `/cmd_vel`, `/voltage`, `/rosout`.
Takes recorded while SLAM or navigation was running also carry `/map`,
`/map_metadata`, `/amcl_pose` and `map → odom`.

## Roadmap

- **Default map view is available**: `rebuild_map.sh` saves a map, TF trajectory and source association. See [workflow](docs/workflow.md).
- **Batch export**: render every Nth frame of a bag as 三合一 PNGs for labelling.

## Requirements

`python3-tk`, `python3-pil`, `python2` with ROS Melodic (`rosbag`, `rospy`),
ImageMagick not required.
