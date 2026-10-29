# tasks 一键入口

每个脚本都会自己 `source ../env.sh`，不用手动 source。
先跑 `00-health-check.sh`，全绿再往下。

| 脚本 | 作用 | 前提 / 看什么 |
|---|---|---|
| `00-health-check.sh` | 体检：ping/电压/雷达/odom/机械臂/节点数 | 机器人开机约 1 分钟 |
| `01-keyboard-drive.sh` | 键盘开车 `i/,/j/l/k` | 车动 = 底盘 + 网络通 |
| `02-view-camera.sh` | RViz：RGB + 深度点云 + 雷达 + TF | 相机栈已起 |
| `03-view-mapping.sh` | RViz：`map` 坐标系，Map + 雷达 | 机器人端 gmapping/map_server 在跑 |
| `04-view-image.sh [usb\|astra]` | 快速看相机图像 | 默认 astra；`usb` 看 USB 相机 |
| `05-lidar-demo.sh [avoid\|track\|warn]` | 雷达避障/跟随/警戒 | 先留 2m 空地 |
| `06-line-follow.sh` | 黑线巡线 | 车放线上、相机朝下 |
| `07-arm-teleop.sh` | 机械臂 + 云台 + 底盘三合一遥控 | `s` 回位最安全 |
| `08-demo-actions.sh` | 灯光/蜂鸣/云台/机械臂自动演示 | 底盘不动，适合展示 |
| `09-gui.sh` | 图形控制台（唯一 GUI） | 推荐日常使用 |
| `10-mapping-ssh.sh` | 只打印机器人端建图/导航 SSH 命令 | 不自动执行 |

## 图形控制台

`09-gui.sh` / `run_gui.sh` 打开 6 个页签：底盘运动、灯光声效、云台与机械臂、
视觉传感器、建图导航、系统状态。系统状态页可切换中英文。

机械臂关节限位以 GUI 为准：j7 `0~225°`、j8 `15~240°`、j9 `60~180°`。

## 建图 / 导航

建图/导航必须在机器人端跑（稳定吃 `/scan` + `/odom`），本机只做 RViz 可视化。
`10-mapping-ssh.sh` 只打印命令，不会自动 SSH 或启动。
