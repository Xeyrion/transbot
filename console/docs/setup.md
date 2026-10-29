# Setup guide (control PC ↔ robot)

## 0. Network model

- The robot runs the ROS master (`:11311`) plus base/lidar/camera stacks (autostarted on power-on, ~18–20 nodes, wait ~1 min after power-on).
- The control PC only publishes topics / calls services over WiFi. Nothing in this repo needs to run on the robot.
- The robot has two WiFi modes: AP (self-hosted hotspot, fixed default IP) and STA (joins an external router, IP assigned by DHCP). `env.sh` handles both.

## 1. Connect (every new terminal)

```bash
source ./env.sh
rostopic echo /voltage -n 1   # a voltage reading means you are connected
```

How `env.sh` resolves the robot IP: `$ROBOT_IP` → `~/.config/transbot_console.conf` → AP default → mDNS hostname. It then picks the local NIC address on the robot's subnet for `ROS_IP`/`ROS_HOSTNAME`. To pin an IP permanently, write it into the config file.

## 2. Drive the base

```bash
bash tasks/01-keyboard-drive.sh
# i forward, , back, j/l turn in place, k stop, q/z speed +/-
```

Notes:

- This publishes `/cmd_vel`, subscribed by the base driver on the robot.
- Vendor keyboard launch files may remap to a private topic — prefer `teleop_twist_keyboard` or remap explicitly (`cmd_vel:=/cmd_vel`).

## 3. Arm / lights / buzzer / gimbal

```bash
python3 scripts/demo_actions.py   # light / buzzer / gimbal / arm demo, base never moves
python3 scripts/arm_teleop.py     # left hand: q/z j1, w/x j2, e/c gripper, r/v gimbal, s home
python3 scripts/arm_probe.py      # self-test with /CurrentAngle readback
bash run_gui.sh                   # full GUI: sliders + readback + effects + video
```

## 4. Cameras

```bash
bash tasks/04-view-image.sh        # quick check (astra default, pass 'usb' for USB cam)
bash tasks/02-view-camera.sh       # rviz: RGB + depth cloud + laser + TF
```

The GUI image panel is off by default to save WiFi bandwidth and falls back across color topics if one goes stale.

## 5. Mapping / navigation (run on the robot, view from here)

Mapping and navigation must run **on the robot mainboard** (stable `/scan`+`/odom` access, the serial lidar device, no WiFi packet loss). SSH to the mainboard as `jetson` (a different account from the control PC) and launch the vendor `transbot_nav` mapping/navigation there; use this repo only for visualization:

```bash
ssh jetson@<robot-ip>            # mainboard, not the control PC
bash tasks/10-mapping-ssh.sh     # prints the full command set, executes nothing
bash tasks/03-view-mapping.sh    # back here: rviz with Fixed Frame=map
```

## 6. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `rostopic list` empty | shell not sourced to robot master | `source ./env.sh` in every new terminal |
| No `/voltage` | robot still booting / serial busy | wait 1 min, check base driver node is up |
| Keyboard drive doesn't move | wrong `cmd_vel` topic | use `teleop_twist_keyboard` or add `cmd_vel:=/cmd_vel` |
| New network, robot unreachable | robot got a new DHCP IP (STA mode) | re-run `source ./env.sh` (mDNS re-resolves); or set `ROBOT_IP=` manually |
| Want a fixed IP | auto-discovery feels uncontrollable | write `ROBOT_IP=<ip>` into `~/.config/transbot_console.conf` |
