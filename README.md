# Transbot Remote Console

Remote control console for the Yahboom Transbot robot (ROS 1 Melodic), running on a control PC over WiFi. All robot interaction goes through the ROS master on the robot — no code needs to be copied onto the robot.

![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue) ![Python 3](https://img.shields.io/badge/Python-3-green) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

```
Control PC (Ubuntu VM)              Robot (Jetson Nano)
┌─────────────────────┐   WiFi   ┌──────────────────────┐
│ this repo            │ ◄──────► │ ROS master :11311     │
│ env.sh + GUI/scripts │  topics  │ base / lidar / camera │
│ ROS_IP=<local>       │  services│ /arm stack preinstalled│
└─────────────────────┘          └──────────────────────┘
```

> ![GUI console](docs/images/gui-demo.png)

## Highlights

1. **Zero-config robot discovery** (`env.sh`) — resolves the robot IP via `$ROBOT_IP` → config file → AP default → mDNS hostname, and filters out mDNS self-hits when the PC shares the robot's hostname. Auto-picks the local `ROS_IP` on the robot's subnet.
2. **Safety-first motion** — new velocity commands preempt old ones, 20 Hz command loop, auto-brake after 0.5–5 s idle, hard caps (0.4 m/s, 2.0 rad/s), brake-on-exit.
3. **One GUI for everything** (`gui/app.py`, ~1700 lines, tkinter, zh/en) — base drive, RGB/buzzer effects, arm + gimbal sliders with `/CurrentAngle` readback, camera/depth viewer, system status.

## Quickstart

```bash
git clone https://github.com/xeyrion/transbot-remote-console.git transbot-remote-console
cd transbot-remote-console

source ./env.sh          # auto-detect robot, sets ROS_MASTER_URI / ROS_IP
bash tasks/00-health-check.sh   # all green before continuing
bash tasks/01-keyboard-drive.sh # i / , / j / l / k  (teleop style)
bash gui/run.sh                 # graphical console (5 tabs)
bash tasks/03-view-mapping.sh   # rviz mapping view (needs gmapping on robot)
```

Pin a robot IP (optional):

```bash
mkdir -p ~/.config && echo 'ROBOT_IP=<robot-ip>' > ~/.config/transbot-console.conf
```

## Layout

```
transbot-remote-console/
├── env.sh                # robot IP discovery + ROS_MASTER_URI / ROS_IP
├── check_robot.sh        # health check (voltage / scan / odom / arm / nodes)
├── gui/
│   ├── app.py            # tkinter console (drive/lights/arm/vision/status)
│   └── run.sh            # GUI launcher
├── scripts/
│   ├── arm_teleop.py     # keyboard teleop: arm+gimbal (left) + base (right)
│   ├── demo_actions.py   # one-shot showroom demo, base never moves
│   └── arm_probe.py      # arm self-test: neutral pose + per-joint probe
├── tasks/                # one-liner entries, 00 first
│   ├── 00-health-check.sh  01-keyboard-drive.sh
│   ├── 02-view-camera.sh   03-view-mapping.sh  04-view-image.sh
│   └── 05-lidar-demo.sh    06-line-follow.sh
├── rviz/camera.rviz  rviz/mapping.rviz
├── docs/setup.md  docs/advanced.md  docs/benchmark.md
└── docs/images/gui-demo.png
```

See `docs/setup.md` for the full guide, `docs/advanced.md` for lidar / line-follow / mapping-and-navigation notes, and `docs/benchmark.md` for measured WiFi/video numbers.

## Requirements

- Ubuntu 18.04 + ROS Melodic on the control PC, robot on the same WiFi (AP or STA mode)
- Vendor ROS packages on the ROS path: `transbot_msgs` (required), `transbot_laser` / `transbot_linefollow` (only for tasks 05/06), `teleop_twist_keyboard`, `rqt_image_view`, `rviz`
- Python: `rospy`, `python3-tk`, `opencv-python`, `numpy`, `Pillow`, `cv_bridge` (see `requirements.txt`)

This repo contains only original glue code (console, scripts, launchers, docs). Vendor robot firmware/packages are **not** included.

## License

MIT — see `LICENSE`.
