# Advanced: lidar / line-follow / mapping notes

All of these consume vendor stacks on the robot side; this repo only provides thin launchers plus the GUI.

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

## Mapping & navigation (robot side)

Run the vendor `transbot_nav` launches **on the robot** (mapping needs stable local `/scan`+`/odom`):

- Mapping: vendor map launch with `map_type:=gmapping open_rviz:=false`, then `map_server map_saver -f <name>` to save.
- Navigation: vendor navigation launch with an existing map.
- This PC only visualizes: `bash tasks/03-view-mapping.sh`.

## What NOT to run on the control PC

- Base driver / bringup launches: they need the robot's serial device and vendor libs; they always fail on the PC.
- Mapping nodes over WiFi: works in theory, drops packets in practice — keep them on the robot.
