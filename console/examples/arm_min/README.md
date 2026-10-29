# arm_min — /TargetAngle 三级示例

同一件事（给某个舵机发"目标角度 + 运动时长"）的三种写法，从裸命令到带保护。

```bash
source ../../env.sh

# 0. 不写代码
rostopic pub -1 /TargetAngle transbot_msgs/Arm \
  "{joint: [{id: 7, run_time: 800, angle: 100.0}]}"

# 1. 最小发布者（28 行，核心 9 行）
python3 arm_simplest.py 7 100 800

# 2. 带限位检查、连接等待与角度读回
python3 arm_min.py 7 100 800
```

| 方式 | 规模 | 特性 |
|---|---:|---|
| `rostopic pub` | 0 行 | 单次发布，验链路 |
| `arm_simplest.py` | 28 行 | 参数化，无任何保护 |
| `arm_min.py` | 69 行 | 限位、等订阅者、读回 |
| `run_gui.sh` | 2900 行 | 完整界面 |

参数：`id` 7=底座 / 8=升降 / 9=夹爪，`angle` 为绝对角度，`run_time` 单位 ms 且不能为 0
（驱动会静默丢弃）。

## 链路

```text
/TargetAngle → transbot_node → set_uart_servo_angle(id, angle, run_time)
             → /dev/ttyTHS1 → 总线舵机（内部闭环）
```

客户端只发目标，角度闭环在舵机内部；`/CurrentAngle` 是独立读回服务，不参与控制。
驱动不校验角度，限位保护在发送端。
