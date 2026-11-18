# arm_min — three ways to send `/TargetAngle`

The same job — tell one servo "go to this angle, take this long" — written three
ways, from a bare command to something with guards.

```bash
source ../../env.sh

# 0. no code at all
rostopic pub -1 /TargetAngle transbot_msgs/Arm \
  "{joint: [{id: 7, run_time: 800, angle: 100.0}]}"

# 1. minimal publisher (28 lines, 9 of them core)
python3 arm_simplest.py 7 100 800

# 2. with limit checks, a subscriber wait and angle readback
python3 arm_min.py 7 100 800
```

| Approach | Size | What you get |
|---|---:|---|
| `rostopic pub` | 0 lines | one message, enough to prove the link |
| `arm_simplest.py` | 28 lines | parameterised, no protection |
| `arm_min.py` | 69 lines | limits, waits for subscribers, reads back |
| `run_gui.sh` | 2900 lines | the full console |

Arguments: `id` 7 = base / 8 = lift / 9 = gripper, `angle` is absolute degrees,
`run_time` is milliseconds and must not be 0 — the driver silently drops those.

## The chain

```text
/TargetAngle → transbot_node → set_uart_servo_angle(id, angle, run_time)
             → /dev/ttyTHS1 → bus servos (closed loop inside the servo)
```

The client only sends a target; the loop closes inside the servo. `/CurrentAngle`
is a separate readback service and takes no part in control. The driver does not
validate angles — limit checking lives on the sending side.
