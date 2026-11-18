# tasks — one-command entry points

Every script sources `../env.sh` itself; you never source it by hand.
Start with `00-health-check.sh` and wait for an all-green run before anything else.

| Script | What it does | Prerequisite / what to look for |
|---|---|---|
| `00-health-check.sh` | ping / voltage / lidar / odom / arm / node count | give the robot about a minute after power-on |
| `01-keyboard-drive.sh` | keyboard driving, `i/,/j/l/k` | the car moves = chassis and network are fine |
| `02-view-camera.sh` | RViz: RGB + depth cloud + lidar + TF | camera stack already running |
| `03-view-mapping.sh` | RViz: `map` frame, Map + lidar | gmapping/map_server running on the robot |
| `04-view-image.sh [usb\|astra]` | quick look at the camera image | defaults to astra; `usb` for the USB camera |
| `05-lidar-demo.sh [avoid\|track\|warn]` | lidar avoid / follow / guard | leave 2 m of clear floor |
| `06-line-follow.sh` | follow a black line | car on the line, camera pointing down |
| `07-arm-teleop.sh` | arm + gimbal + chassis teleop in one | `s` returns to home, the safe key |
| `08-demo-actions.sh` | scripted lights / buzzer / gimbal / arm show | chassis stays still, good for demos |
| `09-gui.sh` | the graphical console (the only GUI) | the one to use day to day |
| `10-mapping-ssh.sh` | prints the robot-side mapping/nav SSH commands | prints only, never executes |

## The console

`09-gui.sh` / `run_gui.sh` opens six tabs: chassis, lights & audio, gimbal & arm,
vision sensors, mapping & navigation, system status. The system tab switches the
interface between Chinese and English.

Arm joint limits are defined by the GUI: j7 `0~225°`, j8 `15~240°`, j9 `60~180°`.

## Mapping / navigation

Mapping and navigation have to run on the robot (they need a steady `/scan` and
`/odom`); this machine only draws RViz. `10-mapping-ssh.sh` prints the commands
and stops there — it never opens an SSH session or starts anything.
