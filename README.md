# Transbot Remote Console

Remote control console for the Yahboom Transbot robot (ROS 1 Melodic), running on a control PC over WiFi. All robot interaction goes through the ROS master on the robot — no code needs to be copied onto the robot.

![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue) ![Python 3](https://img.shields.io/badge/Python-3-green) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

```
Control PC (Ubuntu VM, x86_64)      Robot mainboard (Jetson Nano, aarch64)
┌─────────────────────┐   WiFi   ┌──────────────────────┐
│ this repo            │ ◄──────► │ ROS master :11311     │
│ env.sh + GUI/scripts │  topics  │ base / lidar / camera │
│ ROS_IP=<local>       │  services│ /arm stack preinstalled│
└─────────────────────┘          └──────────────────────┘
```

> ![GUI console](docs/images/gui-demo.png)

**Two machines, not one.** The control PC and the robot mainboard are separate
computers with separate accounts, and they are not interchangeable:

| | Control PC (this repo) | Robot mainboard |
|---|---|---|
| Arch / OS | x86_64, Ubuntu 18.04 VM | aarch64, Ubuntu 18.04 / L4T (JetPack) |
| SSH user | your own PC account | `jetson` |
| Owns | GUI, teleop, rviz | `/dev/ttyTHS1`, `/dev/rplidar`, `/scan`, `/odom` |

Anything that touches a serial device — base driver, lidar, mapping and
navigation — must run **on the mainboard**. This repo only publishes topics and
visualizes; see `tasks/10-mapping-ssh.sh` for the exact SSH commands.

## Highlights

1. **Zero-config robot discovery** (`env.sh`) — resolves the robot IP via `$ROBOT_IP` → config file → AP default → mDNS hostname, and filters out mDNS self-hits when the PC shares the robot's hostname. Auto-picks the local `ROS_IP` on the robot's subnet, loads the ROS workspaces first so the GUI still opens when the robot is offline, and falls back to the bundled `transbot_msgs` when no catkin workspace is built.
2. **Safety-first motion** — new velocity commands preempt old ones, 20 Hz command loop, auto-brake after 0.5–5 s idle, hard caps (0.4 m/s, 2.0 rad/s), brake-on-exit.
3. **One GUI for everything** (`transbot_gui.py`, ~2.9k lines, tkinter, zh/en) — six tabs: base drive, RGB/buzzer effects, arm + gimbal sliders with `/CurrentAngle` readback, camera/depth viewer, mapping/navigation launcher, system status.
4. **No catkin build required** — `vendor/python` ships the official `transbot_msgs` Python modules, so a fresh clone can drive the robot with nothing but ROS Melodic and `python3-tk`.

## Quickstart

```bash
git clone https://github.com/xeyrion/transbot-remote-console.git transbot-remote-console
cd transbot-remote-console

source ./env.sh          # auto-detect robot, sets ROS_MASTER_URI / ROS_IP
bash tasks/00-health-check.sh   # all green before continuing
bash tasks/01-keyboard-drive.sh # i / , / j / l / k  (teleop style)
bash run_gui.sh                 # graphical console (6 tabs)
bash tasks/03-view-mapping.sh   # rviz mapping view (needs gmapping on robot)
```

Pin a robot IP (optional):

```bash
mkdir -p ~/.config && echo 'ROBOT_IP=<robot-ip>' > ~/.config/transbot_console.conf
```

`~/.config/transbot-console.conf` and `~/.config/transbot_demo.conf` are still
accepted as legacy names.

## Layout

```
transbot-remote-console/
├── env.sh                # robot IP discovery + ROS_MASTER_URI / ROS_IP
├── run_gui.sh            # GUI launcher (preflight + master probe + exec)
├── transbot_gui.py       # tkinter console (6 tabs)
├── check_robot.sh        # health check (voltage / scan / odom / arm / nodes)
├── config.example.conf   # template for the pinned-IP config
├── scripts/
│   ├── arm_teleop.py     # keyboard teleop: arm+gimbal (left) + base (right)
│   ├── demo_actions.py   # one-shot showroom demo, base never moves
│   └── arm_probe.py      # arm self-test: neutral pose + per-joint probe
├── tasks/                # one-liner entries, 00 first (see tasks/README.md)
│   ├── 00-health-check.sh    01-keyboard-drive.sh   02-view-camera.sh
│   ├── 03-view-mapping.sh    04-view-image.sh       05-lidar-demo.sh
│   ├── 06-line-follow.sh     07-arm-teleop.sh       08-demo-actions.sh
│   └── 09-gui.sh             10-mapping-ssh.sh
├── examples/arm_min/     # /TargetAngle from `rostopic pub` to limit-checked readback
├── maps/                 # sample map (my_map_1001.pgm/.yaml)
├── vendor/
│   ├── python/           # prebuilt transbot_msgs (no catkin needed)
│   └── transbot_msgs_src/  # official msg/srv definitions
├── rviz/camera.rviz  rviz/mapping.rviz
├── docs/setup.md  docs/advanced.md  docs/benchmark.md
├── docs/机械臂控制原理.md   # code-level arm walkthrough (hardware → ROS → GUI)
└── docs/images/gui-demo.png
```

## Arm joint limits

Unified across the GUI, the teleop script and `examples/arm_min`:

| Joint | Servo id | Range |
|---|---|---|
| base (j7) | 7 | 0–225° |
| lift (j8) | 8 | 15–240° |
| gripper (j9) | 9 | 60–180° |

The vendor manual's `30–270°` for j8 is no longer used.

See `docs/setup.md` for the full guide, `docs/advanced.md` for lidar / line-follow / arm / mapping notes, and `docs/benchmark.md` for measured WiFi/video numbers. For the arm internals, `docs/机械臂控制原理.md` walks a command from the GUI click down to the serial servo.

## Requirements

- Ubuntu 18.04 + ROS Melodic on the control PC, robot on the same WiFi (AP or STA mode)
- Vendor ROS packages on the ROS path: `transbot_msgs` (bundled in `vendor/`, catkin build optional), `transbot_laser` / `transbot_linefollow` (only for tasks 05/06), `teleop_twist_keyboard`, `rqt_image_view`, `rviz`
- Python: `rospy`, `python3-tk`, `opencv-python`, `numpy`, `Pillow`, `cv_bridge` (see `requirements.txt`)

This repo contains original glue code (console, scripts, launchers, docs) plus the bundled `transbot_msgs` message definitions. Vendor robot firmware and driver packages are **not** included.

## License

MIT — see `LICENSE`.
