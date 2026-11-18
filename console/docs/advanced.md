# Advanced: lidar / line-follow / mapping notes

Hardware demonstrations consume vendor packages on the robot. The GUI's online mapping and navigation stack runs on the control PC; see the [system architecture](../../docs/ARCHITECTURE.md).

## Arm control (`tasks/07`–`08`, `examples/arm_min/`)

```bash
bash tasks/07-arm-teleop.sh              # arm + gimbal + base keyboard teleop
bash tasks/08-demo-actions.sh            # light / buzzer / gimbal / arm demo
python3 examples/arm_min/arm_min.py 8 120 1000   # single joint, with limit check + readback
```

Joint limits are unified across the GUI, the teleop script and the examples:
j7 base `0–225°`, j8 lift `15–240°`, j9 gripper `60–180°`.

## LiDAR (`tasks/05-lidar-demo.sh`)

```bash
bash tasks/00-health-check.sh            # /scan must show frame_id first, keep ~2 m clear
bash tasks/05-lidar-demo.sh avoid        # obstacle avoidance (default)
bash tasks/05-lidar-demo.sh track        # person/object following
bash tasks/05-lidar-demo.sh warn         # guard/warning zone
```

Requires the vendor `transbot_laser` package on the ROS path.

## Line following (`tasks/06-line-follow.sh`)

```bash
bash tasks/04-view-image.sh              # confirm the downward camera sees the line
bash tasks/06-line-follow.sh
```

Place the robot on a black line with the camera facing down before launching. Requires vendor `transbot_linefollow`.

## Mapping & navigation (control PC)

Use the GUI's three modes, implemented by [nav/](../nav/README.md):

- Mapping: gmapping consumes the robot's scan, odometry and TF, with a local scan filter.
- Navigation: map_server, AMCL and move_base use a saved map.
- Map + navigate: gmapping and move_base use the live map.
- `bash tasks/03-view-mapping.sh` adds an optional RViz view.

The vendor procedure printed by `tasks/10-mapping-ssh.sh` is an alternative, not a prerequisite. One online stack should own the robot graph at a time.

## Hardware ownership

- Base driver / bringup launches: they need the robot's serial device and vendor libs; they always fail on the PC.
- Mapping and navigation nodes do not access the serial device. Their WiFi link quality still matters and should be checked during robot validation.
