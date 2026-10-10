# Benchmarks & safety parameters (measured over WiFi)

## Video

- Astra color/depth at 640×480 over the robot hotspot: **~1.2 fps each**, ~900 KB/frame. This is the WiFi bandwidth ceiling, not GUI processing power. For higher fps, lower the resolution or use wired Ethernet.
- GUI shows images at native 1:1 resolution (capped at 640×480 display size) and auto-falls-back across color topics after 2.5 s without data.

## Motion safety

- Velocity command loop: 20 Hz; auto-brake after 0.5 s (keyboard teleop) to 5 s (GUI) with no input; brake-on-window-close and brake-on-exit.
- Hard caps in the GUI/teleop: linear 0.4 m/s, angular 2.0 rad/s. New commands preempt in-flight ones.
- Arm slider limits follow the URDF: base 0–225°, lift 15–240°, gripper 60–180°; gimbal yaw 60–120°.

## Arm readback

- `scripts/arm_probe.py` takes repeated `/CurrentAngle` reads and uses the median; pass threshold is 5° error per joint.
- Evaluated joints: base (id 7), lift (id 8), gripper (id 9).

## Startup baseline

- After power-on, the robot's autostart stack takes ~1 min and exposes ~18–20 nodes. `check_robot.sh` gates everything on: ping → `/voltage` → `/scan` → `/odom` → `/CurrentAngle` → node count.
