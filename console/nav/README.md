# nav — mapping, navigation, and both at once

Everything in this folder exists so the console can run the three stacks
without the user touching a terminal. It is self-contained: launch files,
planner parameters and helper nodes all live here, and maps go to
`../maps/`.

```
panel.py    the tab (mode selector, map controls, canvas, teleop) — view only
model.py    subscriptions, live pose, goal publishing; survives a language switch
stack.py    preflight + roslaunch + kill; one process tree per mode
mapview.py  OccupancyGrid -> PIL -> Tk canvas, plus a 2D TF shim
launch/     map.launch, nav.launch, slam_nav.launch
param/      tuned planner parameters (copies of the robot-side originals)
helpers/    scan_filter, cmd_vel_scale, clock_guard, amcl_keepalive, relocalize
```

## Why the stack runs here and not on the robot

The robot owns the serial ports, so the base driver and the lidar must stay
there. gmapping, AMCL and move_base do not touch hardware: they only read
`/scan`, `/odom` and `/tf`. Running them on the control machine means

- **Stop means stop.** One process tree, killed as a group; no half-dead
  `move_base` still holding `map->odom` on the robot after the GUI closes.
- **No catkin on the robot.** The vendor's `transbot_nav` package stays
  untouched; `param/` holds the tuned copies.
- **The parameters are visible.** Every value the planner uses is a file in
  this folder, not a build artefact on the Jetson.

The trade-off is that the robot's WiFi link now carries `/scan` at 7 Hz to the
planner. That is the same traffic the console already needed for the map view,
and it measured fine on this machine.

## What each mode starts, in order

Every start runs the same preflight. Each step exists because it has already
caused a failed run at least once.

| Step | What | Why it is there |
|---|---|---|
| 1 | ping the robot, wait for `/scan` | a stale `ROS_IP` makes the robot beep but never receive a command |
| 2 | `clock_guard.py --sync` | the Jetson has no RTC; a gap beyond `transform_tolerance` freezes AMCL silently |
| 3 | kill leftovers | two `map->odom` publishers break both stacks. `pkill` only reaches processes on this PC, so the rest are killed **by node name on the master** — that is how a stack left running on a second console dies |
| 4 | `scan_filter.py` (mapping modes) | drops the returns that land on the robot's own body, which gmapping would draw as a ring. Skipped if `/scan_filtered` already has data: two filters on one `/scan` hand gmapping every sweep twice |
| 5 | `roslaunch map / nav / slam_nav` | the stack itself |
| 6 | `cmd_vel_scale.py` (navigation modes) | the chassis turns 1.41× faster than commanded |
| 7 | verify node + `/map` | "all ok but nothing moves" is usually a missing node, not a missing topic |

Navigation adds `amcl_keepalive.py`: without it AMCL stops updating while the
robot stands still, its `map->odom` stamp freezes, and every costmap logs
"transform timeout" and refuses to plan.

## One console per master

The nav stack runs on the PC, but the robot's master is shared: a second
console on another machine joins the same graph and brings its own
`scan_filter`, `move_base` and `map->odom`. The symptoms are never obvious —
`/scan_filtered` arrives at twice the lidar rate, gmapping stops growing the
map, and every goal click is refused as "unknown".

Starting a stack now clears that up: step 3 kills leftovers by node name
wherever they run and logs which host they were on, step 4 adopts an existing
`/scan_filtered` instead of adding a second one, and steps 5 refuse to start a
second `gmapping`/`move_base` while one is still alive. A note is printed when
more than one console is connected.

## Parameters that differ from the vendor's

| File | Value | Why |
|---|---|---|
| `dwa_local_planner_params.yaml` | `min_vel_x: 0.0` | 21 % of a recorded run was DWA reversing to fix heading, which looks like the robot wandering off |
| | `min_vel_theta: 0.34`, `max_vel_theta: 0.40` | the chassis ignores anything below ~0.10 rad/s; 0.34 × 0.709 lands just past that hole |
| `nav.launch` (AMCL) | `odom_alpha2: 0.005` | the vendor's 0.10 injects ~1.8° of heading noise per 0.1 m step; the chassis fuses an IMU and does not deserve that |
| | `recovery_alpha_slow/fast` non-zero | the vendor sets both to 0.0, which disables recovery from a bad initial guess |
| `nav.launch` (move_base) | `cmd_vel` → `/cmd_vel_raw` | the relay divides the angular rate by the measured 1.41 gain, so DWA's prediction matches reality |
| `map.launch` (gmapping) | `particles: 60`, `temporalUpdate: -1` | 30 particles deplete at the second loop of a figure-8; a still robot does not need map updates |

`cmd_vel_scale.py` is gain-only (`--angular-deadzone 0.0`). Adding the
dead-zone offset made the gait worse, not better: DWA's angular output flips
sign by ±0.03 rad/s at 10 Hz and the offset turns every one of those into a
real rotation.

## When the pose is wrong

AMCL cannot recover from being picked up and put down two metres away: every
particle is still near the old pose, and the filter will happily keep
publishing a confident-looking wrong answer. Four ways out, cheapest first:

| Symptom | Do this |
|---|---|
| Slightly off, still on the right spot | **Read** fills the pose fields, edit them, **Send pose** |
| Know roughly where the robot is | tick **Set pose**, click the map twice: position, then heading |
| Moved the robot, no idea where | **Re-localise**: matches one scan against the whole map |
| σ in the info line is large | Re-localise; if the map is ambiguous, set the pose by hand |

**Re-localise** (`helpers/relocalize.py`) does a likelihood-field search over
every free cell and 5° of heading, then refines the winner. It also reports
how far the best match leads the *next distinct* candidate:

- margin > 0.10 → the answer is unique
- margin ≤ 0.10 → **180° twin likely**. A long rectangular room with two
  similar ends produces this; the GUI says so in the log and you should use
  the two-click method instead of trusting the search

The `σ` in the info line is AMCL's own covariance (m, m, degrees). It is not
ground truth, but a jump from 0.05 to 0.5 m means the filter has lost the
plot, and the panel warns once when it crosses 0.20 m.

## Can I extend an existing map?

**No.** gmapping has no "load a map and keep going" concept: every start is a
fresh, empty grid, and `map+navigate` is no exception — the **Use map**
dropdown is disabled there because nothing reads it. Mapping over a place you
already have produces a *new* map, not an update of the old one.

What that means in practice:

- Re-mapping the same room replaces the file only if you save under the same
  name (the GUI asks first). The old one is gone unless you renamed it.
- To keep both, save the second run under a new name; `Rename` exists for that.
- Cleaning up a stray wall by hand is possible (edit the `.pgm` in any image
  editor, keep the size) but it is a one-off fix, not a workflow.
- Actually *continuing* a map needs a different SLAM backend. On Melodic that
  is `slam_toolbox` (`map_file_name` / `map_start_pose`), which serialises the
  graph instead of throwing it away. That is a swap of the mapping node, not a
  setting in this panel.

## Saving maps, and names

Two different names, deliberately:

- **Save as** — the name a new map is written under. Mapping modes start with
  a timestamp (`lab_20261107_003243`, to the second) and you can edit or
  **Rename** it later.
- **Use map** — the map the navigation stack loads. It only reads from
  `maps/*.yaml`.

If the name in **Save as** already exists, a warning appears next to the field
while you type, and **Save map** asks for confirmation before overwriting.
Starting a second mapping run with an auto-generated name that already exists
bumps it to a fresh timestamp instead of quietly overwriting the first map.

## Teleop and goals are mutually exclusive

Both end up on `/cmd_vel`, so exactly one of them may own it at a time.
With the relay running, `helpers/cmd_vel_scale.py` is the shared writer:

```
move_base ──/cmd_vel_raw────┐
                            ├─ cmd_vel_scale.py ──/cmd_vel──> driver
keyboard ──/cmd_vel_teleop──┘     teleop wins for 0.3 s
```

A teleop message mutes the planner input for `--teleop-hold` seconds, so the
driver never sees two 20 Hz streams alternating (that is what makes the robot
stutter). Switchover is one message period -- no process is started or killed.

The relay is started by the console at launch, not by the nav stack, so
keyboard driving behaves the same whether navigation is running or not. If the
relay dies the GUI notices within a second, restarts it, and meanwhile
publishes teleop straight to `/cmd_vel` so the robot still moves.

That fallback means the single-writer topology is conditional on relay health;
it does not arbitrate a separate wall-task or recording GUI. See the
[system architecture](../../docs/ARCHITECTURE.md#进程与控制权) for cross-tool ownership.

Pressing a direction key cancels any active goal; sending a goal switches
teleop off. Nobody has to remember to turn the other one off.

Teleop is forwarded with `--teleop-scale 1.0`, i.e. it skips the 0.709 angular
correction: hand-driven turns are about 40 % stronger than the number in the
log. That is deliberate -- a hand-driven turn is a direct command, not a DWA
prediction.

## The map view

Controls, no buttons for the common cases:

| Gesture | Does |
|---|---|
| drag | pan |
| wheel | zoom at the cursor |
| **Shift + drag** | rotate continuously (grab the map and twist it) |
| Shift + wheel | rotate in 5° steps |
| `[` / `]` | rotate 5° |
| `+` / `-` | zoom in / out |
| `0` or double click or **Reset** | back to the default fit |

The current zoom and rotation are shown under the camera pane. The view keys
work whether or not teleop is on, and are ignored while a text field has
focus.

Three details are load-bearing:

1. **2× supersampling.** The whole frame (map, laser, trail, plan, goal,
   robot) is composited into one PIL image at 2× and downscaled. Drawing a few
   hundred canvas ovals gives hard jagged dots and a flat picture.
2. **Fit to the known area, in top-down rows.** gmapping's grid grows far
   beyond what has been seen, so fitting the grid leaves the map as a small
   island in a sea of grey. And the occupancy grid counts rows from the bottom
   while the image counts from the top — converting once, in `_bbox_locked`,
   is what keeps a fixed map from `map_server` from being pushed off the bottom
   edge.
3. **The view transform is centre-based and rotates.** The map raster is
   rotated once (cached) and sampled onto the canvas with an affine transform,
   so zooming costs the same at 12× as at 1×: the old "resize the whole raster
   then paste it" approach allocated a 30000×18000 intermediate image the
   moment you zoomed in. The laser, trail, plan and robot markers go through
   the same transform, which is what keeps them glued to the map at any angle.

The right-hand column carries a 320×160 camera preview. It reuses the
Vision Sensors tab's capture (no second subscription) and only resizes the
frame; if the Astra driver is not running the pane says so and the log points
at the tab that starts it.

## Verification status

Verified on the robot (Jetson Nano, 2026-11-06, battery 11.7 V, `/scan` 7.3 Hz):

- ✅ **mapping** — stack up, `● Mapping`, 32.3 m² / 39 % from a single scan
  position, laser and robot marker drawn, stop returns to `● Not running`
- ✅ **save + rename** — `maps/lab_20261106_002737.yaml/.pgm/.json`, the yaml's
  `image:` rewritten to the bare file name, rename rewrites it again and
  updates the json
- ✅ **navigation** — `/map_server` + `/amcl` + `/move_base` + keepalive +
  relay all up, AMCL reports a pose, stop leaves no `move_base`/`amcl` behind
- ✅ **vendor `/cmd_vel` publishers** — `/TransbotPatrol` and `/BigTransbot`
  are killed on every start; they respawn with the vendor app, so this is not
  a one-off
- ✅ all three launch files parse (`roslaunch --nodes`); `py_compile` on the
  control machine's Python 3.6.9
- ✅ **pose handling** — **Read** pulls the live pose into the fields;
  **Send pose** with typed coordinates moved AMCL to (0.50, 0.50, 0°) and the
  filter converged there; a non-numeric field is rejected with a message
  instead of a traceback; a click in the empty grey area is refused instead of
  sending a pose several metres outside the map
- ✅ **re-localise** — full-map search ran on the real robot (~60-90 s,
  2478 positions × 72 headings) and published the result. On the old sample
  map it reported margin 0.075 and the GUI warned about a 180° twin, which is
  the intended behaviour: that map does not match where the robot was standing
- ⬜ **click-to-goal not yet driven on the real robot** — the publish path is
  the same one `wall_task` used, but this tab has not sent a goal that moved
  the car
- ⬜ the two-click **goal heading** path has not been driven either (the pose
  two-click path has)
- ⬜ loop closure during map+navigate is untested (the goal shifts with the map
  when `map->odom` jumps; see `wall_task/slam_nav/README.md`)

Before the first drive, check the battery in the System tab: below 11 V the
chassis delivers about 25 % of the commanded speed, which looks exactly like
"the goal was ignored".
