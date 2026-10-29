# Transbot

Yahboom Transbot 机器人（ROS 1 Melodic）上的三个子项目：PC 端遥控台、机械臂模型与仿真、墙面标注可行性研究。三者共用同一套几何参数与关节限位。

![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue) ![Python](https://img.shields.io/badge/Python-2%20%7C%203-green) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

## 组成

| 目录 | 内容 | 说明 |
|---|---|---|
| [`console/`](console/) | PC 端控制台 | 6 页签 tkinter GUI、键盘遥控、体检脚本 |
| [`arm/`](arm/) | 机械臂模型与仿真 | 2R 臂 Gazebo 场景、控制面板、真机 UDP 同步、画线几何 |
| [`wallmark/`](wallmark/) | 墙面标注研究 | 能力边界、关节标定、可达性分析 |

![console 控制台界面](console/docs/images/gui-demo.png)

## 双机架构

运行环境由两台独立计算机组成，职责不可互换。

| | 控制 PC | 机器人主板 |
|---|---|---|
| 架构 / 系统 | x86_64，Ubuntu 18.04 | aarch64，Jetson Nano / L4T (JetPack 4.5) |
| SSH 用户 | 本机账号 | `jetson` |
| 资源 | `console/`、`arm/` 仿真、rviz | `/dev/ttyTHS1`、`/dev/rplidar`、`/scan`、`/odom` |
| 职责 | 遥控、可视化、Gazebo | 底盘驱动、雷达、建图、导航 |

依赖串口设备的组件（底盘驱动、雷达、建图、导航）只能在主板上运行。`console/` 仅发布话题与做可视化；`arm/` 的 Gazebo 场景运行在 PC 端，通过 UDP agent 与真机同步。

## 文档导航

| 目标 | 入口 |
|---|---|
| 遥控机器人 | [`console/README.md`](console/README.md)，先执行 `bash console/tasks/00-health-check.sh` |
| 机械臂建模与真机同步 | [`arm/README.md`](arm/README.md) |
| 机械臂可达范围与约束分析 | [`wallmark/docs/07-文档总览.md`](wallmark/docs/07-文档总览.md) |
| 双机架构与 ROS 数据流 | [`console/docs/机械臂控制原理.md`](console/docs/机械臂控制原理.md) |

## 几何参数

三个子项目使用同一组参数，修改时需同步。

```text
e1  = j7 - 95        第一节仰角；j7=95 -> 第一节水平
rel = j8 - 180       小臂相对角；j8=180 -> 两节共线
e2  = e1 + rel       第二节世界仰角
L1 = 10.0 cm         j7 轴 -> j8 轴（钢卷尺实测）
L2 = 24.0 cm         j8 轴 -> 笔尖（j9≈159 夹笔）
j7 轴：x = 12.9 cm，离地 8.0 cm
```

限位：j7 `0–225°`、j8 `15–240°`、j9 `60–180°`。厂家 URDF 的 `L1=8.285` 与旧手册的 j8 `30–270°` 均不再使用。

## 验证状态

| 子项目 | 验证方式 |
|---|---|
| `console/` | 全部 shell `bash -n`、Python 编译检查通过（Ubuntu 18.04 / Python 3.6.9） |
| `arm/` | 18 项 unittest |
| `wallmark/` | 55 项离线自检（实测角口径） |

## License

MIT，见 [`LICENSE`](LICENSE)。
