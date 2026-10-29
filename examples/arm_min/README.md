# arm_min — Transbot 机械臂控制示例

本目录按由易到难的顺序演示 `/TargetAngle` 的发布流程。各版本的核心接口调用一致：

```python
j.id = 7
j.run_time = 800
j.angle = 100.0
pub.publish(Arm(joint=[j]))
```

其余代码分别处理参数解析、限位检查、连接确认和角度读回。

---

## 0. 无代码验证：`rostopic pub`

不编写 Python，直接通过 ROS 命令行发布一次消息：

```bash
source ../../env.sh

rostopic pub -1 /TargetAngle transbot_msgs/Arm \
  "{joint: [{id: 7, run_time: 800, angle: 100.0}]}"
```

字段说明：

```text
id=7          底座舵机
angle=100     目标绝对角度 100°
run_time=800  运动时长 800 ms
```

用途：验证 ROS master、`transbot_node`、串口和舵机链路是否正常。

注意事项：

- `-1`：发布一次后退出。
- 若 `transbot_node` 尚未建立订阅，消息可能丢失，可重复执行。
- 无响应时检查 `rosnode list` 和 `rostopic info /TargetAngle`。

---

## 1. 最小实现：`arm_simplest.py`

文件：`arm_simplest.py`（28 行，核心 9 行）

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Transbot 机械臂单关节控制示例（最小实现）。

用法: python3 arm_simplest.py [id] [angle] [run_time]
      id=7/8/9（底座/升降/夹爪），angle=绝对角度，run_time=毫秒且不可为 0
示例: python3 arm_simplest.py 7 100 800
说明: 仅演示 /TargetAngle 发布，不含限位检查、连接确认和角度读回。
"""
import sys
import rospy
from transbot_msgs.msg import Arm, Joint

jid = int(sys.argv[1]) if len(sys.argv) > 1 else 7
angle = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
run_time = int(sys.argv[3]) if len(sys.argv) > 3 else 800

rospy.init_node('arm_simplest', anonymous=True)
pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)

# 固定延时无法确认订阅关系；高延迟链路应轮询 get_num_connections()。
rospy.sleep(1.0)

pub.publish(Arm(joint=[Joint(id=jid, run_time=run_time, angle=angle)]))

# publish 为异步调用；等待消息发送及运动完成后再退出。
rospy.sleep(run_time / 1000.0 + 1.0)
print('published: id=%d angle=%.1f run_time=%dms' % (jid, angle, run_time))
```

运行：

```bash
source ../../env.sh

python3 arm_simplest.py                  # 默认 id=7, angle=100, run_time=800
python3 arm_simplest.py 7 100 800        # 底座
python3 arm_simplest.py 8 120 1000       # 升降
python3 arm_simplest.py 9 120 600        # 夹爪
```

执行流程：

```text
init_node()       注册 ROS 节点
Publisher()       注册 /TargetAngle 发布者
sleep(1)          固定延时等待订阅者（不保证成功）
publish()         发布一条 Arm 消息
sleep(run_time)   等待消息发送和舵机运动完成
```

核心可执行代码共 9 行：3 行参数解析 + 6 行 ROS 调用。

限制：

- 不检查角度限位。
- 不确认订阅者是否已连接。
- 不读回实际角度。
- 不校验参数范围。

适用场景：教学、接口演示、快速验证。

---

## 2. 带保护实现：`arm_min.py`

文件：`arm_min.py`（69 行）

```bash
python3 arm_min.py 7 100 800
```

参数：

```text
id        7=底座，8=升降，9=夹爪
angle     绝对角度（度）
run_time  运动时长（ms，必须大于 0）
```

与最小实现的差异：

- 校验关节 ID 和角度范围。
- 拒绝 `run_time <= 0`。
- 最多等待 5 秒，确认 `/TargetAngle` 存在订阅者。
- 无订阅者时明确报错。
- 发布后调用 `/CurrentAngle` 读回实际角度。
- 读回失败不影响已发布的控制指令。

示例：

```bash
python3 arm_min.py 7 100 800    # 底座
python3 arm_min.py 8 120 1000   # 升降
python3 arm_min.py 9 120 600    # 夹爪
```

---

## 3. 复杂度对比

| 方式 | 规模 | 特性 | 适用场景 |
|---|---:|---|---|
| `rostopic pub` | 0 行代码 | 单次发布 | 链路验证 |
| `arm_simplest.py` | 28 行（核心 9 行） | 参数化最小发布者 | 教学、原理演示 |
| `arm_min.py` | 69 行 | 限位、连接等待、读回 | 实际测试 |
| 统一控制台 GUI | 2800+ 行 | 完整界面与功能（唯一 GUI） | 日常操作 |

代码量增加对应的是工程保护，而不是控制原理变化。

---

## 4. 控制链路

```text
/TargetAngle
    ↓
transbot_node
    ↓ set_uart_servo_angle(id, angle, run_time)
/dev/ttyTHS1
    ↓
总线舵机（内部闭环）
```

客户端只负责发布目标角度和运动时长，角度闭环由舵机完成。`/CurrentAngle` 是独立的读回服务，不参与控制。

---

## 5. 注意事项

- 客户端运行于 VM，使用 Python3；机器人端驱动为 Python2 风格，不要直接迁移。
- 驱动不会钳制机械臂角度；`LIMITS` 属于客户端保护。
- 控制端统一角度范围（以最新 GUI 为准）：id7 `0~225°`，id8 `15~240°`，
  id9 `60~180°`。旧手册的 id8 `30~270°` 不再采用；GUI、URDF、真机 agent
  和本示例都使用同一组限位。
- 夹笔时 id9 实测 `158~160°` 夹紧。
- `run_time=0` 会被驱动静默丢弃，表现为机械臂不动作。
- 首次测试应使用小角度，并清空机械臂运动范围。
- 底盘具备急停，机械臂不具备；程序退出后舵机仍保持力矩。

---

## 6. 参考

更完整的原理和代码拆解：

`../../docs/机械臂控制原理.md`
