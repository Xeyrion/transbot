# Transbot

在 Yahboom Transbot 机器人（ROS 1 Melodic）上做的一组东西：一个 PC 端遥控台、一个机械臂模型与仿真环境、一个墙面标注的可行性研究。三者共用同一套几何口径和关节限位。

![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue) ![Python](https://img.shields.io/badge/Python-2%20%7C%203-green) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

| 目录 | 是什么 | 一句话 |
|---|---|---|
| [`console/`](console/) | PC 端控制台 | 6 页签 tkinter GUI + 键盘遥控 + 体检脚本；连上 ROS master 就能开机器人 |
| [`arm/`](arm/) | 机械臂模型与仿真 | 2R 臂的 Gazebo 场景 + 控制面板 + 真机 UDP 同步 + 画线几何 |
| [`wallmark/`](wallmark/) | 墙面标注研究 | 机械臂夹笔画墙面裂缝：能力边界、关节标定、为什么只能画竖线 |

![console 界面](console/docs/images/gui-demo.png)

## 两台机器，不是一台

这一组东西横跨两台计算机，混用是最常见的错误来源：

| | 控制 PC | 机器人主板 |
|---|---|---|
| 架构 / 系统 | x86_64，Ubuntu 18.04 | aarch64，Jetson Nano / L4T (JetPack 4.5) |
| SSH 用户 | 你自己的账号 | `jetson` |
| 拥有 | `console/`、`arm/` 的仿真、rviz | `/dev/ttyTHS1`、`/dev/rplidar`、`/scan`、`/odom` |
| 能跑 | 遥控、可视化、Gazebo | 底盘驱动、雷达、建图、导航 |

凡是碰串口设备的（底盘驱动、雷达、建图、导航）**只能在主板上跑**；`console/` 只发话题和做可视化，`arm/` 的 Gazebo 场景跑在 PC 上，通过 UDP agent 与真机同步。

## 从哪看起

- 想遥控机器人 → [`console/README.md`](console/README.md)，先跑 `bash console/tasks/00-health-check.sh`
- 想看机械臂怎么建模、怎么和真机同步 → [`arm/README.md`](arm/README.md)
- 想看机械臂能画多长、为什么只能画竖线 → [`wallmark/docs/07-文档总览.md`](wallmark/docs/07-文档总览.md)
- 想理解双机架构和 ROS 数据流 → `console/docs/机械臂控制原理.md`

## 共同的几何口径

机械臂的几何和限位在三个子项目里保持一致，改一处要三处同步：

```text
e1  = j7 - 95        第一节仰角；j7=95 -> 第一节水平
rel = j8 - 180       小臂相对角；j8=180 -> 两节共线
e2  = e1 + rel       第二节世界仰角
L1 = 10.0 cm         j7 轴 -> j8 轴（钢卷尺实测）
L2 = 24.0 cm         j8 轴 -> 笔尖（j9≈159 夹笔）
j7 轴：x = 12.9 cm，离地 8.0 cm
```

限位：j7 `0–225°`、j8 `15–240°`、j9 `60–180°`。厂家 URDF 的 `L1=8.285` 和旧手册的 j8 `30–270°` 都**不再使用**。

## 验证状态

| 子项目 | 验证 |
|---|---|
| `console/` | 全部 shell `bash -n`、Python 编译检查通过（Ubuntu 18.04 / Python 3.6.9 实测） |
| `arm/` | 18 项 unittest 全过 |
| `wallmark/` | 55 项离线自检全过（实测角口径） |

## License

MIT — 见 [`LICENSE`](LICENSE)。
