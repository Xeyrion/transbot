#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Transbot 全方位 Demo 控制台
# 依赖: python3-tk + ROS melodic(本机) + 机器人已开机(AP 192.168.1.11)
# 用法(推荐): bash run_gui.sh
# 用法(手动): source ./env.sh && python3 transbot_gui.py

import os
import re
import shutil
import signal
import subprocess
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog

import rospy
import geometry_msgs.msg
import nav_msgs.msg
from transbot_msgs.srv import RGBLight, Buzzer, RobotArm, CamDevice
from transbot_msgs.msg import PWMServo, Arm, Joint, Battery

try:
    from sensor_msgs.msg import Image as RosImage
    _HAS_IMG_MSG = True
except Exception:
    RosImage = None
    _HAS_IMG_MSG = False
try:
    from sensor_msgs.msg import LaserScan as RosScan
    _HAS_SCAN = True
except Exception:
    RosScan = None
    _HAS_SCAN = False
try:
    import cv2 as cv
    _HAS_CV2 = True
except Exception:
    cv = None
    _HAS_CV2 = False
try:
    import numpy as np
    _HAS_NP = True
except Exception:
    np = None
    _HAS_NP = False
try:
    from cv_bridge import CvBridge
    _HAS_BRIDGE = True
except Exception:
    CvBridge = None
    _HAS_BRIDGE = False
try:
    from PIL import Image as PilImage, ImageTk as PilImageTk
    _HAS_PIL = True
except Exception:
    PilImage = PilImageTk = None
    _HAS_PIL = False
_CAM_OK = _HAS_IMG_MSG and _HAS_CV2 and _HAS_NP and _HAS_PIL

# 建图/导航面板：三种模式共用一块地图画布。导入失败不致命——其他页签照常
# 可用，只在建图页显示一行错误，比整个 GUI 起不来好。
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
try:
    from nav.model import NavState
    from nav.panel import MapPanel
    _HAS_NAV = True
except Exception as _nav_err:      # noqa: N816  (keep the reason for the log)
    NavState = MapPanel = None
    _HAS_NAV = False
    NAV_IMPORT_ERROR = str(_nav_err)
else:
    NAV_IMPORT_ERROR = ''

NODE = 'transbot_gui'
STATE = {'ok': False, 'master': '', 'err': ''}

# 图像显示尺寸上限。Astra 原生 640x480，按 1:1 显示即可看清细节；
# 若换更高分辨率的相机则等比缩小到该上限，避免图像超出窗口。
DISPLAY_SIZE = (640, 480)

# ---------------------------------------------------------------------------
# i18n：界面文案双语化
# 约定：所有用户可见字符串写成 L('中文原文')，英文查 STRINGS['en']。
# 查不到就回退成中文原文，所以漏翻译不会白屏、只会保持中文。
# 切换语言 = 销毁并重建整棵控件树（tk 的 Label/Button 文本不可整体刷新，
# 但重建是唯一能保证 140 多处文案一次性换掉的办法）。
# ---------------------------------------------------------------------------
LANG = {'code': 'en'}  # Default UI language is English; switchable to Chinese in the System tab.

STRINGS = {
    # 中文是源文案，不需要映射表；保留空表只是让语言切换逻辑对称。
    'zh': {},
    'en': {
        # --- 连接 / 底层错误 ---
        'ROS 环境已关闭':
            'ROS environment has shut down',
        '机器人控制服务未就绪，请确认热点已连接且机器人已上电启动':
            'Robot control services are not ready. Check that the hotspot is '
            'connected and the robot is powered on.',
        '尚未连接至 ROS Master': 'Not connected to ROS Master',
        '✗ ROS 环境初始化失败：%s': '✗ ROS initialization failed: %s',
        '● 连接失败': '● Connection failed',
        '● 已连接 ': '● Connected ',
        '● 正在建立连接…': '● Connecting…',
        '✗ 连接失败：%s': '✗ Connection failed: %s',
        '✓ 已建立连接：ROS Master %s': '✓ Connected to ROS Master %s',

        # --- 窗口 / 页签 ---
        'Transbot 全方位机器人控制台 — Yahboom':
            'Transbot Robot Control Console — Yahboom',
        'Transbot 全方位机器人控制台': 'Transbot Robot Control Console',
        ' 底盘运动 ': ' Chassis ',
        ' 灯光与声效 ': ' Lights & Audio ',
        ' 云台与机械臂 ': ' Gimbal & Arm ',
        ' 视觉传感器 ': ' Vision Sensors ',
        ' 系统状态 ': ' System ',

        # --- 页1 底盘 ---
        '底盘运动（键盘控制）': 'Chassis Motion (keyboard)',
        '前进 / 后退': 'forward / reverse',
        '左转 / 右转': 'turn left / right',
        '立即停车': 'stop now',
        '空格': 'Space',
        '停止': 'Stop',
        '连续速度控制（键盘用这两个值）':
            'Continuous speed (the keys use these two values)',
        '线速度 m/s': 'Linear speed (m/s)',
        '角速度 rad/s': 'Angular speed (rad/s)',
        '紧急停止': 'Emergency Stop',
        '按住方向键持续运动，松开即停；只有本页可见时键盘才生效。'
        '导航运行中按方向键会先取消目标再接管。'
        '速度范围取自底盘驱动限幅（0.4 m/s、2.0 rad/s）。':
            'Hold a key to move, release to stop. The keys only work while this '
            'page is visible. Pressing one during navigation cancels the goal '
            'first. Speed range follows the driver limits (0.4 m/s, 2.0 rad/s).',
        '速度范围取自底盘驱动限幅参数（线速度 0.4 m/s、角速度 2.0 rad/s）。'
                    '指令以 20 Hz 周期持续发布，5 秒无操作自动制动，'
                    '关闭窗口时同样会先发送制动指令。':
            'Speed range follows the chassis driver limits (linear 0.4 m/s, '
            'angular 2.0 rad/s). Commands are republished at 20 Hz, brake '
            'automatically after 5 s of no input, and a braking command is '
            'also sent before the window closes.',

        # --- 页2 灯光与声效 ---
        'RGB 灯带特效（协议定义 0~6）': 'RGB Strip Effects (protocol IDs 0-6)',
        '流水灯': 'Running Water',
        '跑马灯': 'Marquee',
        '呼吸灯': 'Breathing',
        '渐变灯': 'Gradient',
        '星光点点': 'Starlight',
        '电量显示': 'Battery Display',
        '变化速度（1 最快 ~ 10 最慢）': 'Effect speed (1 = fastest, 10 = slowest)',
        '应用速度': 'Apply Speed',
        '速度参数对已播放的特效立即生效；'
        '“电量显示”为静态效果，速度参数对其无视觉影响':
            'Applies to the running effect at once; "Battery Display" is '
            'static.',
        '蜂鸣器': 'Buzzer',
        '短鸣 0.3 s': 'Short beep 0.3 s',
        '长鸣 2 s': 'Long beep 2 s',
        '停止鸣响': 'Stop Buzzer',
        '○ 当前为停止状态，请先选择一个特效再调节速度':
            '○ The LED strip is currently off; select an effect before adjusting '
            'the speed',
        '当前：%s，速度 %d': 'Current: %s, speed %d',

        # --- 页3 云台与机械臂 ---
        '云台偏航（关节 id1，角度范围 60~120）':
            'Gimbal Yaw (joint id1, range 60-120)',
        '偏航角 (id1)': 'Yaw angle (id1)',
        '发送指令': 'Send Command',
        '复位至 90°': 'Reset to 90°',
        '机械臂关节角度控制': 'Arm Joint Angle Control',
        '读取当前角度': 'Read Current Angles',
        '经 /CurrentAngle 服务读取关节 7/8/9 的实测角度，'
        '结果自动回填至下方滑块':
            'Reads the measured angles of joints 7/8/9 via the /CurrentAngle '
            'service and fills them into the sliders below.',
        '关节限位依据 URDF 定义：底座 0~225°　升降 15~240°　夹爪 60~180°':
            'Joint limits per the URDF: Base 0-225°　Lift 15-240°　'
            'Gripper 60-180°',
        '发送': 'Send',
        '步长 (°)': 'Step (°)',
        '✗ 步长无效，请输入 0.5~30 之间的数字':
            '✗ Invalid step; enter a number between 0.5 and 30',
        '三轴同步下发': 'Send All Three Joints Together',

        # --- 页4 视觉 ---
        '视觉传感器图像（Astra：左彩色 + 右深度伪彩；USBCam：仅彩色）':
            'Vision-Sensor Images (Astra: color left + depth pseudo-color '
            'right; USBCam: color only)',
        '数据源：待连接…': 'Source: awaiting connection…',
        '开始采集': 'Start Capture',
        '停止采集': 'Stop Capture',
        '未采集（默认关闭以节省无线带宽）':
            'Not capturing (off by default to save wireless bandwidth)',
        '显示依赖缺失：需安装 sensor_msgs、opencv、numpy、PIL':
            'Display dependencies missing: install sensor_msgs, opencv, '
            'numpy, PIL',
        '彩色图像：等待数据…': 'Color image: waiting for data…',
        '深度图像：等待数据…': 'Depth image: waiting for data…',
        '图像按原始分辨率 1:1 显示。实测无线带宽下彩色与深度各约 '
        '1.2 帧/秒（单帧 900 KB），帧率受 WiFi 限制而非本程序处理能力；'
        '若画面为空，可在终端用 rostopic hz 检查 /camera/rgb/image_raw、'
        '/image、/camera/depth/image_raw 的发布频率':
            'Images are shown 1:1 at their native resolution. Over the '
            'wireless link, color and depth each run at about 1.2 fps '
            '(900 KB per frame); the frame rate is limited by WiFi '
            'bandwidth, not by this program. If the view is blank, check the '
            'publish rates with "rostopic hz" on /camera/rgb/image_raw, '
            '/image, /camera/depth/image_raw.',
        '⚠ 尚未连接至机器人，无法启动图像采集':
            '⚠ The robot is not connected, so image capture cannot start',
        '✗ 显示依赖缺失（cv2 / numpy / PIL / sensor_msgs 之一未安装）':
            '✗ Display dependencies missing (cv2 / numpy / PIL / sensor_msgs)',
        '○ 图像采集已处于运行状态': '○ Image capture is already running',
        '数据源：%s%s': 'Source: %s%s',
        '（无深度流）': ' (no depth stream)',
        '采集中…': 'Capturing…',
        '✓ 图像采集已启动 %s': '✓ Image capture started %s',
        '✗ 图像采集启动失败：%s': '✗ Failed to start image capture: %s',
        '未知原因': 'Unknown reason',
        '已停止': 'Stopped',
        '数据源：%s（上一话题无数据，已自动切换）':
            'Source: %s (previous topic had no data, switched automatically)',
        '○ 当前彩色话题无数据，已自动切换订阅 %s':
            '○ The current color topic had no data; switched subscription to %s',
        '彩色图像解码失败': 'Failed to decode the color image',
        '彩色图像：话题无数据': 'Color image: topic has no data',
        '当前相机无深度数据流': 'Current camera has no depth stream',
        '　彩色帧延迟 %.1f s': '  Color-frame latency: %.1f s',

        # --- 页5 系统状态 ---
        '运行状态（单击条目可复制其内容）':
            'Runtime Status (click a row to copy its value)',
        '已复制到剪贴板：': 'Copied to clipboard: ',
        '相机型号': 'Camera Model',
        '底盘速度': 'Chassis Speed',
        '电池电压': 'Battery Voltage',
        '节点数量': 'Node Count',
        '复制全部日志': 'Copy All Logs',
        '清空日志': 'Clear Logs',
        '复制选中': 'Copy Selection',
        '复制全部': 'Copy All',
        '清空': 'Clear',
        '统计中…': 'Counting…',
        '等待识别': 'Awaiting detection',
        'ROS Master 地址已复制到剪贴板':
            'ROS Master address copied to clipboard',

        # --- 速度中继（键盘与导航共用一个 /cmd_vel 出口） ---
        '○ 速度中继已就绪（键盘与导航共用 /cmd_vel）':
            '○ Speed relay ready (keyboard and planner share /cmd_vel)',
        '○ 速度中继已重新拉起':
            '○ Speed relay restarted',
        '✗ 速度中继没起来，遥控将直连 /cmd_vel':
            '✗ Speed relay did not start; teleop goes straight to /cmd_vel',
        '✗ 速度中继启动失败：%s': '✗ Failed to start the speed relay: %s',
        '✗ 速度中继反复失败，遥控直连 /cmd_vel':
            '✗ Speed relay keeps failing; teleop goes straight to /cmd_vel',
        '✗ 制动失败：%s': '✗ Braking failed: %s',
        '○ 已有速度中继在运行，直接沿用':
            '○ An existing speed relay is already running; reusing it',
        '○ 还没有图像帧：Astra 驱动可能没启动，点「启动Astra驱动」':
            '○ No image frames yet: the Astra driver is probably not running; '
            'press "Start Astra Driver"',

        # --- 通用指令日志 ---
        '⚠ 尚未连接至机器人，本次操作已忽略':
            '⚠ The robot is not connected; the command was ignored',
        '✓ %s 指令已下发：参数 %s，服务响应 %s':
            '✓ %s command sent: parameters %s, response %s',
        '✓ %s 指令已下发：参数 %s': '✓ %s command sent: parameters %s',
        '✗ %s 指令失败：服务端异常 %s': '✗ %s command failed: service error: %s',
        '✗ %s 指令失败：%s': '✗ %s command failed: %s',
        '✗ 执行失败：%s': '✗ Execution failed: %s',

        # --- 相机型号识别 ---
        '○ 相机型号查询失败，不影响运动控制：%s':
            '○ Camera-model query failed, but motion control is unaffected: %s',
        '○ 已识别相机型号：%s': '○ Detected camera model: %s',

        # --- Astra 驱动远启 ---
        '启动Astra驱动': 'Start Astra Driver',
        '停止Astra驱动': 'Stop Astra Driver',
        '→ 正在机器人上启动Astra驱动…':
            '→ Starting the Astra driver on the robot…',
        '✓ Astra驱动已启动，话题 %s':
            '✓ Astra driver started, topic %s',
        '○ Astra驱动已在运行，话题 %s':
            '○ Astra driver already running, topic %s',
        '✗ Astra驱动启动失败：%s':
            '✗ Failed to start the Astra driver: %s',
        '→ 正在停止Astra驱动…': '→ Stopping the Astra driver…',
        '✓ Astra驱动已停止': '✓ Astra driver stopped',
        '✗ Astra驱动停止失败：%s': '✗ Failed to stop the Astra driver: %s',
        '✗ Astra驱动停止后话题仍有数据，已升级 SIGKILL 仍未停，'
        '请在机器人上检查 rosnode list：%s':
            '✗ Astra driver still publishing after SIGKILL; '
            'check rosnode list on the robot: %s',
        '✗ 本机缺sshpass且无免密SSH，无法远启：%s':
            '✗ No sshpass and no key-based SSH on this host: %s',
        '○ Astra操作进行中，请稍候':
            '○ Astra operation already in progress, please wait',

        # --- 建图与导航 ---
        '建图与导航': 'Mapping & Navigation',
        ' 建图与导航 ': ' Mapping & Navigation ',
        '运行模式': 'Mode',
        '启动': 'Start',
        '纯建图': 'Mapping',
        '纯导航': 'Navigation',
        '边建图边导航': 'Map + navigate',
        '保存为': 'Save as',
        '使用地图（仅导航）': 'Use map (nav only)',
        '保存地图': 'Save map',
        '改名': 'Rename',
        '刷新列表': 'Refresh',
        '设初始位姿': 'Set pose',
        '指定目标朝向': 'Goal heading',
        '读当前': 'Read',
        '发送位姿': 'Send pose',
        '重新定位': 'Re-localise',
        '手动': 'manual',
        '点选': 'click',
        '⚠ 已有同名地图，保存会覆盖':
            '⚠ A map with this name already exists; saving overwrites it',
        '○ 还没有位姿（等 TF 或 /amcl_pose）':
            '○ No pose yet (waiting for TF or /amcl_pose)',
        '○ 已读当前位姿 %.2f, %.2f, %.0f°':
            '○ Current pose %.2f, %.2f, %.0f°',
        '✗ 位姿要填数字：X / Y 用米，θ 用度':
            '✗ Pose fields must be numbers: X/Y in metres, θ in degrees',
        '✗ 还没连上 ROS，位姿发不出去':
            '✗ Not connected to ROS yet; the pose cannot be sent',
        '✗ 位姿下发失败：%s': '✗ Failed to send the pose: %s',
        '✓ 初始位姿 (%.2f, %.2f, %.0f°) 已下发（%s）':
            '✓ Initial pose (%.2f, %.2f, %.0f°) sent (%s)',
        '○ 重新定位要在纯导航模式下、并且选好地图':
            '○ Re-localising needs navigation mode with a map selected',
        '✗ 找不到 relocalize.py': '✗ relocalize.py not found',
        '✗ 还没有 /scan，等雷达出数据再试':
            '✗ No /scan yet; wait for the lidar',
        '● 正在重新定位…': '● Re-localising…',
        '→ 重新定位中，约 30-90 s（全图似然场搜索）':
            '→ Re-localising, 30-90 s (likelihood-field search over the map)',
        '⚠ 似然只领先第二名 %.3f，可能有 180° 孪生解，'
        '建议改用「设初始位姿」点两下':
            '⚠ The best match leads the runner-up by only %.3f; a 180° twin '
            'is likely. Use "set pose" and click twice instead.',
        '○ 似然领先 %.3f，解唯一':
            '○ Likelihood leads by %.3f; the solution is unique',
        '✗ 重新定位失败：%s': '✗ Re-localise failed: %s',
        '○ 目标位置 (%.2f, %.2f)，再点一下定朝向':
            '○ Goal position (%.2f, %.2f); click again to set the heading',
        '⚠ 电池 %.1f V，低于 11 V 时底盘只出 25%% 的速度，'
        '看着像"按了没反应"':
            '⚠ Battery %.1f V; below 11 V the chassis delivers about 25 %% of '
            'the commanded speed, which looks like "nothing happened"',
        '⚠ 定位不确定度 σ=%.2f/%.2f m 偏大：先用「重新定位」，'
        '或用「设初始位姿」点两下':
            '⚠ Localisation sigma is %.2f/%.2f m: use "re-localise", or '
            '"set pose" and click twice',
        '「设初始位姿」点两下：位置→朝向；'
        '「重新定位」重找位置':
            '"Set pose": two clicks. '
            '"Re-localise": match the scan against the map.',
        '键盘遥控': 'Keyboard teleop',
        'W/S 前后，A/D 转向，空格停': 'W/S drive, A/D turn, space stops',
        '拖动 = 平移　滚轮 = 缩放　Shift+拖动 = 旋转　'
        '[ ] 旋转　+ - 缩放　双击 = 复位':
            'Drag = pan   Wheel = zoom   Shift+drag = rotate   '
            '[ ] = rotate   + - = zoom   Double click = reset',
        '● 未启动': '● Not running',
        '位姿': 'Pose',
        '旋转': 'Rotation',
        '复位': 'Reset',
        '相机预览': 'Camera',
        '相机未开启': 'Camera off',
        '相机无数据': 'No camera data',
        '○ 相机没有数据：Astra 驱动多半没起，'
        '去「视觉传感器」页点启动 Astra':
            '○ No camera data: the Astra driver is probably not '
            'running. Start it on the Vision Sensors tab.',
        '已知': 'Known',
        '覆盖率': 'Coverage',
        '目标': 'Goal',
        '缩放': 'Zoom',
        '● 正在启动…': '● Starting…',
        '● 正在停止…': '● Stopping…',
        '● 建图中': '● Mapping',
        '● 导航中': '● Navigating',
        '● 边建图边导航': '● Mapping + navigating',
        '○ 从空地图开始建，不会加载或修改已有地图':
            '○ Mapping starts from an empty map; the selected map is '
            'not loaded or modified',
        '○ 地图保存名：%s（可随时改名）':
            '○ This map will be saved as %s (rename it any time)',
        '○ 地图 %s；定位不准就在地图上点两下设初始位姿':
            '○ Map %s; if the pose looks wrong, click twice on the map to '
            'set the initial pose',
        '○ 地图列表：%s': '○ Maps: %s',
        '（空）': '(none)',
        '✗ 读取地图列表失败：%s': '✗ Failed to read the map list: %s',
        '→ 正在保存地图 %s …': '→ Saving map %s…',
        '✓ 地图已保存：maps/%s.yaml': '✓ Map saved: maps/%s.yaml',
        '✗ 保存失败：%s': '✗ Failed to save the map: %s',
        '○ 请先输入地图名': '○ Enter a map name first',
        '○ 没有叫 %s 的地图，先保存一次':
            '○ There is no map called %s yet; save one first',
        '地图已存在': 'Map already exists',
        '已有同名地图 %s，覆盖它？':
            'A map called %s already exists. Overwrite it?',
        '新地图名（%s）': 'New map name (was %s)',
        '✓ 已改名：%s → %s': '✓ Renamed: %s → %s',
        '✗ 改名失败：%s': '✗ Failed to rename: %s',
        '✗ 启动失败：%s': '✗ Failed to start: %s',
        '✓ 已停止': '✓ Stopped',
        '✗ 停止失败：%s': '✗ Failed to stop: %s',
        '○ 还没有地图数据': '○ No map data yet',
        '○ 初始位置 (%.2f, %.2f)，再点一下定朝向':
            '○ Position (%.2f, %.2f); click again to set the heading',
        '✓ 初始位姿 (%.2f, %.2f, %.0f°) 已下发':
            '✓ Initial pose (%.2f, %.2f, %.0f°) sent',
        '○ 当前模式不接收导航目标':
            '○ This mode does not take navigation goals',
        '✗ 目标点不可用（%s），换一个已知空闲区域':
            '✗ Goal rejected (%s); pick a spot that is known and free',
        '✗ 这一点不在已知地图内（%s），请点在地图上；'
        '要精确坐标就直接填 X/Y/θ':
            '✗ That point is not inside the known map (%s); click on the map, '
            'or type X/Y/θ for an exact pose',
        '→ 目标 (%.2f, %.2f) %.0f°': '→ Goal (%.2f, %.2f) %.0f°',
        '○ 目标已取消': '○ Goal cancelled',
        '○ 键盘遥控已开启（W/S 前后，A/D 转向，空格停）':
            '○ Teleop on (W/S drive, A/D turn, space stops)',
        '○ 键盘遥控已关闭': '○ Teleop off',
        '○ 已停车': '○ Stopped',
        '○ 手动遥控接管，导航目标已取消':
            '○ Teleop took over; the navigation goal was cancelled',
        '✗ 遥控指令失败：%s': '✗ Teleop command failed: %s',
        '○ 地图订阅已建立': '○ Map subscriptions are up',
        '✗ 订阅失败：%s': '✗ Failed to subscribe: %s',
        '✓ 建图/导航栈已停止': '✓ Mapping/navigation stack stopped',
        '✗ 停止建图/导航栈失败：%s':
            '✗ Failed to stop the mapping/navigation stack: %s',
        '✗ 建图导航面板初始化失败：%s':
            '✗ The map panel failed to initialise: %s',

        # --- 机械臂 ---
        '✓ 机械臂关节角度：%s': '✓ Arm joint angles: %s',
        '○ 关节滑块已同步至实测角度':
            '○ Joint sliders synchronized to the measured angles',
        '✗ 读取机械臂角度失败，请确认 ROS Master 与机械臂驱动节点状态：%s':
            '✗ Failed to read arm angles; check ROS Master and the arm driver '
            'node: %s',
        '✗ 关节 %d 目标角度 %.0f° 超出安全范围 %d°~%d°':
            '✗ Joint %d target angle %.0f° exceeds the safe range %d°-%d°',
        '→ 关节 %d（%s）目标角度 %.0f°，动作时长 %d ms':
            '→ Joint %d (%s): target %.0f°, duration %d ms',
        '关节 %d：%.0f°（允许 %d°~%d°）':
            'joint %d: %.0f° (allowed %d°-%d°)',
        '✗ 超出安全范围的角度已被拦截：%s':
            '✗ Out-of-range angles were rejected: %s',
        '→ 三轴同步目标角度：%s':
            '→ Synchronized targets for all three joints: %s',
        '○ 机械臂初始角度读取失败，不影响其他功能，'
        '可稍后手动点击“读取当前角度”：%s':
            '○ Initial arm-angle read failed; other controls are unaffected. '
            'Click "Read Current Angles" later: %s',
        '○ 机械臂初始角度回填失败：%s':
            '○ Failed to load the initial arm angles into the sliders: %s',

        # --- 连续速度控制 ---
        '○ 线速度与角速度均为 0，已按紧急制动处理':
            '○ Both speeds are 0, treated as an emergency stop',
        '→ 连续控制已启动：线速度 %.2f m/s，角速度 %.2f rad/s'
        '（5 秒后自动制动）':
            '→ Continuous control started: linear %.2f m/s, angular %.2f rad/s'
            ' (auto-brake after 5 s)',

        # --- 剪贴板 / 退出 ---
        '✗ 复制到剪贴板失败：%s\n': '✗ Failed to copy to clipboard: %s\n',
        '已将全部运行日志（%d 字符）复制到剪贴板':
            'Copied all runtime log (%d characters) to the clipboard',
        '运行日志为空，无需复制':
            'Runtime log is empty, nothing to copy',
        '✗ 复制到剪贴板失败：%s': '✗ Failed to copy to clipboard: %s',
        '✓ 程序退出前已发送制动指令':
            '✓ Braking command sent before exit',
        '✗ 退出前发送制动指令失败：%s':
            '✗ Failed to send the braking command before exit: %s',

        # --- 显示依赖 ---
        '缺少 cv2/numpy/PIL/sensor_msgs 显示库':
            'Missing display libraries: cv2 / numpy / PIL / sensor_msgs',
        '图像字节数不足：%d < %d': 'Image buffer too small: %d < %d',

        # --- 关节 / 指令名称 ---
        '底座 (id7)': 'Base (id7)',
        '升降 (id8)': 'Lift (id8)',
        '夹爪 (id9)': 'Gripper (id9)',
        '底盘前进': 'Chassis Forward',
        '底盘后退': 'Chassis Reverse',
        '底盘转向': 'Chassis Turn',
        '紧急制动': 'Emergency Stop',
        '灯带特效': 'LED Strip Effect',
        '云台偏航': 'Gimbal Yaw',
        '机械臂': 'Robotic Arm',
        '机械臂角度读取': 'Arm Angle Read',

        # --- 语言切换 ---
        '界面语言': 'Interface Language',
        '切换语言只重建界面文字，运行日志与设备状态保持不变。':
            'Only the interface text is rebuilt; logs and readings are kept.',
        '界面语言已切换为中文': 'Interface language switched to Chinese',
        '✗ 界面重建失败：%s': '✗ Failed to rebuild the interface: %s',
        '界面语言已切换为英文': 'Interface language switched to English',
    },
}


def L(text):
    """按当前语言翻译；缺词回退中文原文。"""
    return STRINGS.get(LANG['code'], {}).get(text, text)


def T(d):
    """把 {key: 中文} 的名称表整体翻译，如关节名、指令名。"""
    return dict((k, L(v)) for k, v in d.items())


def fit(text, minimum=0):
    """按钮/标签的字符宽度，保证文字不被截断。

    Tk 的 width 单位是"平均字符宽"，一个汉字约占两个单位。英文比中文长
    （"长鸣 2 s" -> "Long beep 2 s"），写死 width 在英文下会被截成 "Running W…"，
    所以按当前语言的实际文本算宽度。
    """
    units = sum(2 if ord(ch) > 0x2E80 else 1 for ch in text)
    return max(int(minimum), units + 2)


# ---------------------------------------------------------------------------
# ROS 控制端：所有会阻塞的调用放后台线程，GUI 只触发命令
# 注意：
#  * rospy.init_node() 必须在主线程调用（Python 的 signal.signal 只允许主线程，
#    否则报 "signal only works in main thread"），且传 disable_signals=True
#    把 Ctrl-C 交给 Tk mainloop。
#  * 所有触碰 Tk 控件的操作必须回到主线程，后台线程只做 ROS IO。
#    Tk 的 root.after 不能从子线程直接调（实测子线程 after 回调不会执行），
#    所以用 queue.Queue 做中转：子线程 put，主线程的 _tick 定时取出来执行。
# ---------------------------------------------------------------------------
class RobotROS:
    def __init__(self):
        self.cmd_pub = None
        self.servo_pub = None
        self.arm_pub = None
        self.rgb_srv = self.buz_srv = self.hl_srv = None
        self._arm_srv = None
        self._cmd_lock = threading.Lock()
        self._arm_lock = threading.Lock()
        self._astra_lock = threading.Lock()
        self._inited = False
        # 运动代次：每次新的运动命令 +1，进行中的命令发现代次变了就立刻刹车退出。
        # 解决"连点两个前进会提前急停 / 前进中点后退会中途反向"——旧的运动必须让位。
        self._motion_gen = 0
        self._motion_lock = threading.Lock()
        # 手动微调的重发线程句柄（20Hz 发 /cmd_vel）
        self._hold_thread = None
        self._hold_stop = threading.Event()

    # ---- 初始化 / 连接（主线程调用） ----
    def init_ros(self):
        """主线程调用：init_node + 建 publisher/proxy，不阻塞很久。"""
        master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        STATE['master'] = master
        if not self._inited:
            # disable_signals=True 是关键：Tk 程序不能让 rospy 抢 signal handler
            rospy.init_node(NODE, anonymous=True, disable_signals=True)
            self._inited = True
        # 键盘永远发 /cmd_vel_teleop，由常驻中继和规划器一起仲裁；
        # 中继没起来时才退回直连 /cmd_vel（见 _publish_cmd）。
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=2)
        self.cmd_pub_teleop = rospy.Publisher(
            '/cmd_vel_teleop', geometry_msgs.msg.Twist, queue_size=2)
        self.relay_up = False
        self.servo_pub = rospy.Publisher('/PWMServo', PWMServo, queue_size=10)
        self.arm_pub = rospy.Publisher('/TargetAngle', Arm, queue_size=10)
        self.rgb_srv = rospy.ServiceProxy('/RGBLight', RGBLight)
        self.buz_srv = rospy.ServiceProxy('/Buzzer', Buzzer)

    def wait_for_services(self, timeout=8):
        """后台线程调用：阻塞等机器人服务上线。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if rospy.is_shutdown():
                raise RuntimeError(L('ROS 环境已关闭'))
            try:
                rospy.wait_for_service('/RGBLight', timeout=1.0)
                STATE['ok'] = True
                return
            except rospy.ROSException:
                time.sleep(0.3)
        raise RuntimeError(L(
            '机器人控制服务未就绪，请确认热点已连接且机器人已上电启动'))

    # 兼容老入口：仅在主线程直接调时可用；GUI 内部不再这样用
    def connect(self):
        self.init_ros()
        self.wait_for_services()

    def connected(self):
        return STATE['ok']

    # ---- 运动 ----
    # 驱动侧硬限幅（transbot_driver.py cmd_vel_callback）：
    #   linear_max  = ~linear_speed_limit  默认 0.4
    #   angular_max = ~angular_speed_limit 默认 2.0
    # GUI 也按这个限幅，免得滑杆拉到 1.0 而车只走 0.4（看着像"滑杆失灵"）。
    LIN_MAX = 0.4
    ANG_MAX = 2.0

    def _publish_cmd(self, x, z):
        if not self.connected():
            raise RuntimeError(L('尚未连接至 ROS Master'))
        x = max(-self.LIN_MAX, min(self.LIN_MAX, float(x)))
        z = max(-self.ANG_MAX, min(self.ANG_MAX, float(z)))
        with self._cmd_lock:
            tw = geometry_msgs.msg.Twist()
            tw.linear.x = x
            tw.angular.z = z
            pub = self.cmd_pub_teleop if self.relay_up else self.cmd_pub
            pub.publish(tw)

    def _new_motion(self):
        """开始一段运动：抢下运动权，返回本次代号。旧代号立刻失效。"""
        with self._motion_lock:
            self._motion_gen += 1
            return self._motion_gen

    def _motion_stale(self, gen):
        with self._motion_lock:
            return gen != self._motion_gen

    def straight(self, m):
        """前进/后退 m 米；m>0 前进，m<0 后退。速度0.3m/s，走够时间后发停。"""
        sign = 1 if m >= 0 else -1
        dist = abs(m)
        if dist <= 0.01:
            return self.stop()
        speed = 0.3
        dur = dist / speed
        gen = self._new_motion()
        self._publish_cmd(sign * speed, 0)
        # 分片 sleep：每 50ms 查一次代号，被新命令抢占就立刻收手
        t_end = time.time() + dur
        try:
            while time.time() < t_end:
                if self._motion_stale(gen):
                    return False
                time.sleep(0.05)
        finally:
            # 只有仍是当前运动时才发停车，否则会把新命令的运动也刹掉
            if not self._motion_stale(gen):
                self._publish_cmd(0, 0)
        return True

    def turn(self, sense, deg=45):
        """原地转 sense∈{L,R}，deg 度。角速度 0.9rad/s。"""
        w = 0.9
        dur = abs(deg) / 180.0 * 3.14159 / w
        z = w if sense == 'L' else -w
        gen = self._new_motion()
        self._publish_cmd(0, z)
        t_end = time.time() + dur
        try:
            while time.time() < t_end:
                if self._motion_stale(gen):
                    return False
                time.sleep(0.05)
        finally:
            if not self._motion_stale(gen):
                self._publish_cmd(0, 0)
        return True

    def stop(self):
        """急停：作废当前运动代次 + 停掉手动重发 + 发一条 0 速度。"""
        with self._motion_lock:
            self._motion_gen += 1
        self._hold_stop.set()
        self._publish_cmd(0, 0)
        return True

    def manual(self, lx, az, hold=5.0, on_done=None):
        """手动微调：20Hz 重发 /cmd_ver，超时或被抢占自动停。

        之前只发一条就撒手，驱动认的是最后一条速度，GUI 一关/一崩车就一路开下去。
        驱动侧没有看门狗，所以这个重发+超时必须由 GUI 自己兜住。
        hold<=0 表示持续发到 stop()/窗口关闭。
        """
        # 每次运动用一个新的 Event：老代码对同一个 Event set() 再 clear()，
        # 旧线程醒来时标志已经清掉，会继续按旧速度发，和新命令交替下发。
        prev_evt, stop_evt = self._hold_stop, threading.Event()
        self._hold_stop = stop_evt
        prev_evt.set()
        lx = max(-self.LIN_MAX, min(self.LIN_MAX, float(lx)))
        az = max(-self.ANG_MAX, min(self.ANG_MAX, float(az)))
        if abs(lx) < 1e-3 and abs(az) < 1e-3:
            return self.stop()
        gen = self._new_motion()
        t_end = time.time() + hold if hold and hold > 0 else None

        def loop():
            try:
                while not stop_evt.is_set() and not rospy.is_shutdown():
                    if self._motion_stale(gen):
                        return
                    if t_end is not None and time.time() >= t_end:
                        break
                    try:
                        self._publish_cmd(lx, az)
                    except Exception:
                        return
                    stop_evt.wait(0.05)   # 20Hz
            finally:
                # 只在还持有运动权时停车，别打断别人的新命令
                if not self._motion_stale(gen):
                    try:
                        self._publish_cmd(0, 0)
                    except Exception:
                        pass
                if on_done is not None:
                    try:
                        on_done()
                    except Exception:
                        pass

        th = threading.Thread(target=loop, daemon=True)
        self._hold_thread = th
        th.start()
        return True

    def stop_manual(self):
        """只停手动重发并发停车，不影响别的东西。"""
        self._hold_stop.set()
        with self._motion_lock:
            self._motion_gen += 1
        self._publish_cmd(0, 0)
        return True

    # ---- 灯带 / 蜂鸣 ----
    # effect 定义以官方 transbot_joy.py/Program_Ctrl.py 注释为准：
    # 0停止 1流水灯 2跑马灯 3呼吸灯 4渐变灯 5星光点点 6电量显示（静态，按电压显示颜色）
    # 注意：没有 7，发 7 固件直接忽略；speed [1,10]，越小越快
    # 驱动 transbot_driver.py:180 把 speed 原样传给 Transbot_Lib.set_colorful_effect，
    # 官方文档确认该参数生效（数值越小变化越快）。
    def rgb(self, effect, speed=5):
        effect = max(0, min(6, int(effect)))
        speed = max(1, min(10, int(speed)))
        return self.rgb_srv(effect=effect, speed=speed)

    def buzzer(self, hz, secs=None):
        r = self.buz_srv(buzzer=hz)
        if secs:
            time.sleep(secs)
            self.buz_srv(buzzer=0)
        return r

    # ---- 云台 ----
    # 官方定义（joy/mono/follow 三处一致）：id1=左右偏航，id2=上下俯仰。
    # 本机装的 Astra 是单轴支架，只有 id1，GUI 因此只留偏航。
    # 限位见 transbot_driver.py:sub_PWMServocallback：CameraDevice=astra 时
    # id1 被钳在 60~120（USBCam 时可到 0~180），滑杆就用驱动实际接受的区间。
    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, int(v)))

    def servo(self, sid, angle):
        """id1=偏航左右。滑杆已限制在 60~120，这里再钳一次做兜底。

        驱动侧（CameraDevice=astra）本来也会把 id1 钳到 60~120，但驱动那边
        钳完照样往下发，GUI 这层先拦掉，日志里也能看出是否超范围。
        """
        angle = self._clamp(angle, 60, 120) if sid == 1 else self._clamp(angle, 0, 140)
        self.servo_pub.publish(PWMServo(id=sid, angle=angle))

    # ---- 机械臂 ----
    # 控制限位以最新 GUI / 真机读数为准，不从旧 URDF 反推：
    #   j7 底座  0~225°
    #   j8 升降  15~240°
    #   j9 夹爪  60~180°（夹笔实测 158~160°）
    # run_time 单位 ms（0 会被驱动 set_uart_servo_angle 直接丢弃）
    ARM_LIMITS = {7: (0, 225), 8: (15, 240), 9: (60, 180)}
    ARM_NAMES = {7: '底座 (id7)', 8: '升降 (id8)', 9: '夹爪 (id9)'}
    CMD_NAMES = {'forward': '底盘前进', 'backward': '底盘后退', 'turn': '底盘转向',
                 'stop': '紧急制动', 'rgb': '灯带特效',
                 'buz': '蜂鸣器', 'servo': '云台偏航', 'arm': '机械臂',
                 'read_arm': '机械臂角度读取'}

    def arm(self, joints, wait=1.5):
        # 串行化：连点时排队执行，否则后一条打断前一条，看起来“没反应/没到位”
        with self._arm_lock:
            a = Arm()
            max_rt = 0
            for jid, ang, rt in joints:
                a.joint.append(Joint(id=int(jid), run_time=int(rt), angle=float(ang)))
                max_rt = max(max_rt, int(rt))
            # 之前靠"发两次隔 0.15s"补首包丢失，但驱动 sub_armcallback 每条都执行，
            # 结果整个动作被触发两遍（顿挫/回跳）。改成等订阅者连上再发一次。
            deadline = time.time() + 1.5
            try:
                while (self.arm_pub.get_num_connections() == 0
                       and time.time() < deadline
                       and not rospy.is_shutdown()):
                    time.sleep(0.05)
            except Exception:
                pass   # 老版本 rospy 没这接口就直接发
            self.arm_pub.publish(a)
            # 等待至少覆盖最大 run_time
            time.sleep(max(wait, max_rt / 1000.0 + 0.8))

    def read_angles(self):
        """调 /CurrentAngle 读回 7/8/9 真实角度，用于区分“网络不通”还是“舵机不动”。"""
        try:
            proxy = self._arm_srv
            if proxy is None:
                proxy = rospy.ServiceProxy('/CurrentAngle', RobotArm)
                self._arm_srv = proxy
            rospy.wait_for_service('/CurrentAngle', timeout=3.0)
            # 舵机总线偶尔会漏答一个关节（驱动对 -1 直接跳过），单次读可能
            # 只回来 2 个。重试到 3 个为止，保留关节最多的那次结果。
            best = None
            for _ in range(3):
                resp = proxy(apply='')
                joints = getattr(getattr(resp, 'RobotArm', resp), 'joint', [])
                if best is None or len(joints) > best[0]:
                    best = (len(joints), resp)
                if best[0] >= 3:
                    break
            return best[1]
        except Exception:
            raise

    def get_cam_device(self):
        """查相机型号：Astra=单轴云台（只有偏航，无俯仰，硬件如此非故障）；
        USBCam=双轴云台。对应 URDF：astra只有一个 astra_joint(±30°)，camera才有
        camera_joint1+camera_joint2 两个轴。"""
        proxy = rospy.ServiceProxy('/CamDevice', CamDevice)
        rospy.wait_for_service('/CamDevice', timeout=3.0)
        r = proxy(GetGev='GetDevice!!!')
        return getattr(r, 'camDevice', '')

    # ---- Astra 驱动远启（跑在机器人端，GUI 经 SSH 下发） ----
    # Astra 相机经 USB 接在机器人上，驱动节点只能跑在机器人端（与 transbot_node
    # 同理，VM 无硬件设备）。ROS_MASTER_URI 指向机器人，故本机 wait_for_message
    # 能直接确认远端话题。调用方必须在后台线程调用（SSH + 等话题均阻塞）。
    ROBOT_USER = 'jetson'
    ASTRA_LAUNCH = 'astra_camera astrapro.launch'
    ASTRA_RGB_TOPIC = '/camera/rgb/image_raw'
    ASTRA_PID_FILE = '/tmp/astra_gui.pid'
    ASTRA_LOG_FILE = '/tmp/astra_gui.log'
    ASTRA_START_TIMEOUT = 60
    # 停止时必须覆盖的远端进程模式。astrapro.launch 会拉起三类进程：
    #   roslaunch astra_camera astrapro.launch（父进程，PID 记在 ASTRA_PID_FILE）
    #   .../lib/astra_camera/camera_node（/camera/camera_rgb，发布 RGB）
    #   .../lib/nodelet/nodelet manager（/camera/camera_nodelet_manager，
    #     承载 AstraDriverNodelet 与深度处理，占着 USB 设备）
    # 旧代码只杀前两类，manager 残留会一直占用 USB：下次 start 能拉起 launch
    # 但新 camera_node 抢不到设备 → 60s 等不到数据（2026-10-06 日志复现）。
    ASTRA_KILL_PATTERNS = (
        '[a]stra_camera',
        '[a]strapro.launch',
        '[c]amera_nodelet_manager',
    )

    @staticmethod
    def _robot_host():
        """从 ROS_MASTER_URI 解析机器人 IP，STA 换网后无需改代码。"""
        m = re.match(r'^\w+://([^:/]+)', os.environ.get('ROS_MASTER_URI', ''))
        return m.group(1) if m else '192.168.1.11'

    @staticmethod
    def _ssh_pass():
        """机器人 SSH 口令：实验室默认值，可用 ROBOT_SSH_PASS 环境变量覆盖。"""
        return os.environ.get('ROBOT_SSH_PASS', 'yahboom')

    def _ssh_run(self, remote_cmd, timeout=30):
        """经 sshpass 在机器人上执行单条命令，返回 CompletedProcess。"""
        if shutil.which('sshpass') is None:
            raise RuntimeError('sshpass not installed and no key auth configured')
        env = dict(os.environ, SSHPASS=self._ssh_pass())
        cmd = ['sshpass', '-e', 'ssh', '-o', 'StrictHostKeyChecking=no',
               '-o', 'ConnectTimeout=8', '%s@%s' % (self.ROBOT_USER, self._robot_host()),
               remote_cmd]
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=timeout, env=env)

    def astra_running(self, timeout=3):
        """RGB 话题有数即视为驱动在跑（只读探测，无副作用）。"""
        if not _HAS_IMG_MSG:
            return False
        try:
            rospy.wait_for_message(self.ASTRA_RGB_TOPIC, RosImage, timeout=timeout)
            return True
        except Exception:
            return False

    def astra_start(self, wait=ASTRA_START_TIMEOUT):
        """在机器人上启动 Astra 驱动并等 RGB 话题出数，返回话题名。

        互斥：并发调用直接抛错返回，两个驱动实例会抢同一 USB 设备导致双双失败。
        """
        if not self._astra_lock.acquire(blocking=False):
            raise RuntimeError('another Astra operation is still in progress')
        try:
            return self._astra_start_locked(wait)
        finally:
            self._astra_lock.release()

    def _astra_clean_remote(self):
        """清场专用远端脚本：杀残留 + 清僵尸注册。

        必须独立一次 SSH：cmdline 里只放方括号形式的 pattern。
        若把 `roslaunch astra_camera astrapro.launch` 字面量和 pkill 放进
        同一个远端 shell，pkill 会先匹配到承载本次 SSH 的父 bash 自己，
        把 SSH 会话杀掉（ssh rc=255、无任何输出，2026-10-06 实测）。
        """
        term = ' '.join('pkill -f "{p}" 2>/dev/null;'.format(p=p)
                        for p in self.ASTRA_KILL_PATTERNS)
        kill9 = ' '.join('pkill -9 -f "{p}" 2>/dev/null;'.format(p=p)
                         for p in self.ASTRA_KILL_PATTERNS)
        h = self._robot_host()
        remote = (
            'PID=$(cat {pid} 2>/dev/null); [ -n "$PID" ] && kill "$PID" 2>/dev/null; '
            '{term} sleep 2; {kill9} '
            'source /opt/ros/melodic/setup.bash; '
            'source ~/transbot_ws/devel/setup.bash; '
            'source ~/software/transbot_library/devel/setup.bash; '
            'export ROS_MASTER_URI=http://{h}:11311; export ROS_IP={h}; '
            'echo y 2>/dev/null | timeout 15 rosnode cleanup >/dev/null 2>&1; '
            'rm -f {pid}; echo CLEANED'
        ).format(pid=self.ASTRA_PID_FILE, h=h, term=term, kill9=kill9)
        # 远端含 sleep 2 + cleanup(≤15s)，超时按 40s 算
        r = self._ssh_run(remote, timeout=40)
        if r.returncode != 0:
            raise RuntimeError((r.stderr.strip() or 'ssh command failed'))

    def _astra_start_locked(self, wait):
        # 话题在 `rostopic list` 里 ≠ 驱动活着：被 SIGKILL 杀掉的节点无法
        # 注销，master 会残留僵尸注册（本次 rostopic info 实锤：
        # Publishers 里 /camera/camera 还在，但机器人上已无此进程）。
        # 旧代码见话题在 list 里就 echo ALREADY 跳过拉起，然后 60s 等数据超时。
        # 所以只认真实数据：5s 内收到一帧才算活着，否则清掉残留并重新拉起。
        if self.astra_running(timeout=5):
            return self.ASTRA_RGB_TOPIC
        h = self._robot_host()
        # 先清场（独立 SSH，见 _astra_clean_remote 注释），再拉起（独立 SSH，
        # 含字面量但不含 pkill，同样安全）。绝不能把两者拼进同一条远端命令。
        self._astra_clean_remote()
        remote = (
            'source /opt/ros/melodic/setup.bash; '
            'source ~/transbot_ws/devel/setup.bash; '
            'source ~/software/transbot_library/devel/setup.bash; '
            'export ROS_MASTER_URI=http://{h}:11311; export ROS_IP={h}; '
            'rm -f {pid}; '
            '(nohup roslaunch {launch} >{log} 2>&1 & echo $! > {pid}); echo LAUNCHED'
        ).format(h=h, launch=self.ASTRA_LAUNCH,
                 log=self.ASTRA_LOG_FILE, pid=self.ASTRA_PID_FILE)
        # 拉起本身很快，默认 30s SSH 超时足够
        r = self._ssh_run(remote)
        if r.returncode != 0:
            raise RuntimeError((r.stderr.strip() or 'ssh command failed'))
        if not _HAS_IMG_MSG:
            raise RuntimeError('sensor_msgs missing on this host')
        try:
            rospy.wait_for_message(self.ASTRA_RGB_TOPIC, RosImage, timeout=wait)
        except Exception as e:
            raise RuntimeError('no data on %s within %ss (%s)'
                               % (self.ASTRA_RGB_TOPIC, wait, e))
        return self.ASTRA_RGB_TOPIC

    def _astra_kill_remote(self, sig=''):
        """在机器人上杀一轮 Astra 相关进程。sig='' 为 SIGTERM，'-9' 为 SIGKILL。"""
        parts = ['PID=$(cat {pid} 2>/dev/null); '
                 '[ -n "$PID" ] && kill "$PID" 2>/dev/null; '.format(
                     pid=self.ASTRA_PID_FILE)]
        for pat in self.ASTRA_KILL_PATTERNS:
            if sig:
                parts.append('pkill {sig} -f "{pat}" 2>/dev/null; '.format(
                    sig=sig, pat=pat))
            else:
                parts.append('pkill -f "{pat}" 2>/dev/null; '.format(pat=pat))
        parts.append('rm -f {pid}; echo STOPPED'.format(pid=self.ASTRA_PID_FILE))
        r = self._ssh_run(''.join(parts))
        if r.returncode != 0:
            raise RuntimeError((r.stderr.strip() or 'ssh command failed'))

    def astra_stop(self, verify=True):
        """停掉 Astra 驱动并验证 RGB 话题确已无数据。

        先 SIGTERM 给 roslaunch 优雅退出的机会，再用本地 wait_for_message
        验证；仍有数据则升级 SIGKILL 后复验，还不停才抛错（之前只发一轮
        SIGTERM 就报“已停止”，残留的 manager 占着 USB 导致下次启动 60s 超时）。
        """
        if not self._astra_lock.acquire(blocking=False):
            raise RuntimeError('another Astra operation is still in progress')
        try:
            self._astra_kill_remote(sig='')
            time.sleep(3.5)
            if verify and self.astra_running(timeout=3):
                self._astra_kill_remote(sig='-9')
                time.sleep(2.5)
            if verify and self.astra_running(timeout=5):
                raise RuntimeError(L(
                    '✗ Astra驱动停止后话题仍有数据，已升级 SIGKILL 仍未停，'
                    '请在机器人上检查 rosnode list：%s') % self.ASTRA_RGB_TOPIC)
            return True
        finally:
            self._astra_lock.release()



# ---------------------------------------------------------------------------
# 相机画面：RGB 走压缩流（省 WiFi 带宽），深度走原始流
# 话题（与官方脚本一致）：
#   Astra: /camera/rgb/image_raw/compressed + /camera/depth/image_raw
#   USB  : /usb_cam/image_raw/compressed（无深度流）
# 线程模型：ROS 回调只存 msg，主线程 _camera_tick 解码上屏（Tk 只能主线程碰）
# ---------------------------------------------------------------------------
class CameraViewer:
    # 彩色话题没有候选链：只认驱动直出的 /camera/rgb/image_raw（Astra）或
    # /usb_cam/image_raw（USB）。历史上的第二候选 /image 来自出厂自启程序的
    # msgToimg 链（TransbotPatrol→/image_msg→msgToimg→/image），杀掉自启程序
    # 后这条链就没了；留着它只会导致 fallback 到一个僵尸话题然后黑屏。
    RGB_CANDIDATES_ASTRA = ('/camera/rgb/image_raw',)
    RGB_CANDIDATES_USB = ('/usb_cam/image_raw',)

    def __init__(self):
        self.rgb_topic = self.RGB_CANDIDATES_ASTRA[0]
        self._candidates = list(self.RGB_CANDIDATES_ASTRA)
        self._cand_idx = 0
        self.depth_topic = '/camera/depth/image_raw'
        self._subs = []
        self._rgb_sub = None
        self._lock = threading.Lock()
        self._rgb_msg = None
        self._rgb_t = 0.0
        self._depth_msg = None
        self._depth_t = 0.0
        self.viewing = False
        self._last_err = ''
        self._rgb_start_t = 0.0

    def set_source(self, cam_device):
        if cam_device == 'USBCam':
            self._candidates = list(self.RGB_CANDIDATES_USB)
            self.depth_topic = None
        else:  # Astra 或型号未知都按 Astra 处理
            self._candidates = list(self.RGB_CANDIDATES_ASTRA)
            self.depth_topic = '/camera/depth/image_raw'
        self._cand_idx = 0
        self.rgb_topic = self._candidates[0]
        self._rgb_msg = None
        self._rgb_t = 0.0

    def _sub_rgb(self):
        self._rgb_sub = rospy.Subscriber(
            self.rgb_topic, RosImage, self._on_rgb, queue_size=1)
        self._subs.append(self._rgb_sub)

    def maybe_fallback(self):
        """当前彩色话题 2.5 秒还没数据 -> 换下一个候选。切了返回新 topic，没切返回 None。
        只在主线程（_camera_tick）里调。"""
        if not self.viewing or self._rgb_msg is not None:
            return None
        if self._cand_idx + 1 >= len(self._candidates):
            return None
        if time.time() - self._rgb_start_t < 2.5:
            return None
        try:
            if self._rgb_sub is not None:
                try:
                    self._rgb_sub.unregister()
                except Exception:
                    pass
                if self._rgb_sub in self._subs:
                    self._subs.remove(self._rgb_sub)
                self._rgb_sub = None
            self._cand_idx += 1
            self.rgb_topic = self._candidates[self._cand_idx]
            self._sub_rgb()
            self._rgb_start_t = time.time()
            self._last_err = ''
            return self.rgb_topic
        except Exception as e:
            self._last_err = str(e)
            return None

    def start(self):
        # 幂等：已经打开时重复点不再建订阅，直接算成功（之前误报“打开失败”）
        if self.viewing:
            self._last_err = 'already viewing'
            return True
        if not _CAM_OK:
            self._last_err = L('缺少 cv2/numpy/PIL/sensor_msgs 显示库')
            return False
        self.stop()
        try:
            self._sub_rgb()
            self._rgb_start_t = time.time()
            if self.depth_topic:
                self._subs.append(rospy.Subscriber(
                    self.depth_topic, RosImage, self._on_depth, queue_size=1))
            self.viewing = True
            self._last_err = ''
            return True
        except Exception as e:
            self._last_err = str(e)
            self.stop()
            return False

    def stop(self):
        for s in self._subs:
            try:
                s.unregister()
            except Exception:
                pass
        self._subs = []
        self._rgb_sub = None
        self.viewing = False

    def _on_rgb(self, msg):
        with self._lock:
            self._rgb_msg = msg
            self._rgb_t = time.time()

    def _on_depth(self, msg):
        with self._lock:
            self._depth_msg = msg
            self._depth_t = time.time()

    def latest(self):
        with self._lock:
            return self._rgb_msg, self._rgb_t, self._depth_msg, self._depth_t


def _unpad_rows(arr, h, w, step):
    """按 step 剥掉每行尾部的对齐填充，还原成 (h, w)。单通道，1 元素 = 1 像素。

    图像消息的 step 是"每行字节数"，可能 > w*元素字节数（对齐填充）。
    直接 arr[:h*w].reshape(h, w) 会把上一行的填充当成下一行的像素，
    画面整体斜着错位——RGB 那条路径早就按 step 处理了，深度这里以前没有。

    数据不够就抛异常，由调用方的 except 转成 None：宁可不出图，也不出错图。
    """
    # 调用方都是 np.frombuffer 出来的一维数组；万一传了二维，先拍平，
    # 否则下面的按行切片会切错。
    arr = arr.reshape(-1)
    esz = arr.itemsize          # 每像素字节数（uint16=2, float32=4）
    row = max(step // esz, w)   # 每行像素数，step 缺失或过小时退回 w
    if arr.size < h * row:
        raise ValueError(L('图像字节数不足：%d < %d') % (arr.size, h * row))
    if row == w:
        return arr[:h * w].reshape(h, w)
    # 逐行只取前 w 个像素，拷进 (h, w)
    out = np.empty((h, w), arr.dtype)
    for r in range(h):
        out[r] = arr[r * row: r * row + w]
    return out


def decode_rgb_raw(msg):
    """sensor_msgs/Image -> BGR numpy。直读 numpy 不依赖 cv_bridge
    （melodic 的 bridge 是 py2 编译版，py3 调 boost 会崩）。
    实测：/image 是 bgr8，/camera/rgb/image_raw 是 rgb8，640x480，step 可能带填充。"""
    try:
        h, w = int(msg.height), int(msg.width)
        enc = (getattr(msg, 'encoding', '') or '').lower()
        step = int(getattr(msg, 'step', 0)) or 0
        buf = bytes(getattr(msg, 'data', b''))
        arr = np.frombuffer(buf, dtype=np.uint8)
        if enc in ('rgb8', 'bgr8'):
            ch = 3
        elif enc in ('rgba8', 'bgra8'):
            ch = 4
        elif enc in ('mono8', '8uc1'):
            row = step if step >= w else w
            if arr.size < h * row:
                return None
            gray = arr[:h * row].reshape(h, row)[:, :w].reshape(h, w)
            return cv.cvtColor(gray, cv.COLOR_GRAY2BGR)
        else:
            return None
        row = step if step >= w * ch else w * ch
        if arr.size < h * row:
            return None
        img = arr[:h * row].reshape(h, row)[:, :w * ch].reshape(h, w, ch)
        if ch == 4:
            img = img[:, :, :3]
            if enc == 'rgba8':
                img = img[:, :, ::-1]
        elif enc == 'rgb8':
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    except Exception:
        pass
    return None


def decode_depth_colormap(msg):
    """/camera/depth/image_raw -> BGR 伪彩色。16UC1(mm)/32FC1(m)都兼容，无效点黑色。
    直读 numpy 不依赖 cv_bridge（melodic 的 bridge 是 py2 编译版，py3 调 boost 会崩）。"""
    try:
        h, w = int(msg.height), int(msg.width)
        enc = getattr(msg, 'encoding', '')
        step = int(getattr(msg, 'step', 0)) or 0
        buf = bytes(getattr(msg, 'data', b''))
        if enc in ('16UC1', 'mono16', '16SC1'):
            d = np.frombuffer(buf, dtype=np.uint16)
            d = _unpad_rows(d, h, w, step).astype(np.float32) / 1000.0  # mm -> m
        elif enc in ('32FC1', '32FC2', '32FC3', '32FC4'):
            d = _unpad_rows(np.frombuffer(buf, dtype=np.float32), h, w, step)
        elif _HAS_BRIDGE:
            try:
                d = np.asarray(CvBridge().imgmsg_to_cv2(
                    msg, desired_encoding='passthrough'), dtype=np.float32)
                if d.max() > 100:
                    d = d / 1000.0
            except Exception:
                return None
        else:
            return None
        valid = np.isfinite(d) & (d > 0.2) & (d < 6.0)
        norm = np.zeros_like(d, dtype=np.float32)
        norm[valid] = 1.0 - (np.clip(d[valid], 0.3, 5.0) - 0.3) / 4.7  # 近红远蓝
        gray = (norm * 255).astype(np.uint8)
        color = cv.applyColorMap(gray, cv.COLORMAP_JET)
        color[~valid] = (0, 0, 0)
        if valid.any():
            txt = 'min %.2fm' % float(d[valid].min())
            cv.putText(color, txt, (8, 22), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return color
    except Exception:
        return None


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class DriveRelay(object):
    """The single writer of /cmd_vel.

    move_base publishes /cmd_vel_raw, the keyboard publishes /cmd_vel_teleop,
    and helpers/cmd_vel_scale.py arbitrates: teleop wins for a short window,
    the planner is forwarded otherwise. One process, one output topic, so the
    driver never sees two 20 Hz streams alternating.

    Started with the console (not with the nav stack) so keyboard driving works
    the same whether navigation is running or not. If it dies the GUI notices
    within a second, restarts it, and meanwhile publishes teleop straight to
    /cmd_vel so the robot still moves.
    """

    LOG_PATH = '/tmp/transbot_relay.log'

    def __init__(self, helpers_dir, log):
        self.script = os.path.join(helpers_dir, 'cmd_vel_scale.py')
        self.log = log
        self.proc = None
        self.last_start = 0.0
        self._lock = threading.Lock()   # start()/stop() from two threads
        # 第二个控制台实例不要抢中继：/cmd_vel 只允许一个写入者，
        # 两个实例各杀各的中继会互相踢，日志里刷"已重新拉起"。
        self.adopted = False
        self._adopt_check = 0.0

    def _relay_node_up(self):
        """Cheap liveness probe for a relay we did not spawn ourselves."""
        try:
            out = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL,
                                 universal_newlines=True, timeout=6).stdout
        except Exception:
            return False
        return any(n.startswith('/cmd_vel_scale') for n in out.split())

    def _relay_publishing(self):
        """Stronger check: is something actually publishing /cmd_vel as a relay?"""
        try:
            out = subprocess.run(['rostopic', 'info', '/cmd_vel'],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                 universal_newlines=True, timeout=8).stdout
        except Exception:
            return False
        if 'Publishers' not in out:
            return False
        seg = out.split('Publishers:')[1].split('Subscribers:')[0]
        return 'cmd_vel_scale' in seg

    def alive(self):
        if self.proc is not None:
            return self.proc.poll() is None
        if not self.adopted:
            return False
        # 借用的中继：每 4 秒确认一次节点还在，别一直假装它活着
        now = time.time()
        if now - self._adopt_check < 4.0:
            return True
        self._adopt_check = now
        if not self._relay_node_up():
            self.adopted = False
            return False
        return True

    def last_error(self, n=3):
        """Last lines of the relay's own stderr, for the log panel."""
        try:
            with open(self.LOG_PATH, 'r', errors='replace') as fh:
                lines = [l.strip() for l in fh.read().strip().splitlines() if l.strip()]
            return ' | '.join(lines[-n:])
        except Exception:
            return ''

    def start(self, quiet=False):
        """(Re)start the relay. Kills a stale one first, ours or not."""
        with self._lock:
            if not os.path.isfile(self.script):
                if not quiet:
                    self.log(L('✗ 速度中继启动失败：%s') % ('script missing'))
                return False
            self._stop_locked()
            # 已经有中继在跑（另一个控制台实例，或上次没退干净的）就沿用：
            # 杀掉再起一个，两个窗口会互相踢。
            if self._relay_publishing():
                self.adopted = True
                self._adopt_check = time.time()
                self.last_start = time.time()
                if not quiet:
                    self.log(L('○ 已有速度中继在运行，直接沿用'))
                return True
            self.adopted = False
            # a relay left over from a crashed session would double-publish
            subprocess.call(['pkill', '-f', '[c]md_vel_scale.py'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Keep the child's own output: without it a crash is invisible and
            # the watchdog just restarts it in a loop.
            try:
                err = open(self.LOG_PATH, 'w')
            except Exception:
                err = subprocess.DEVNULL
            try:
                self.proc = subprocess.Popen(
                    ['python3', self.script,
                     '--in-topic', '/cmd_vel_raw',
                     '--out-topic', '/cmd_vel',
                     '--angular-scale', '0.709',
                     '--teleop-topic', '/cmd_vel_teleop'],
                    stdout=err, stderr=subprocess.STDOUT,
                    start_new_session=True)
            except Exception as e:
                if not quiet:
                    self.log(L('✗ 速度中继启动失败：%s') % e)
                self.proc = None
                return False
            finally:
                try:
                    if err is not subprocess.DEVNULL:
                        err.close()
                except Exception:
                    pass
            self.last_start = time.time()
            time.sleep(0.8)
            ok = self.alive()
            if ok and not quiet:
                self.log(L('○ 速度中继已就绪（键盘与导航共用 /cmd_vel）'))
            elif not ok and not quiet:
                self.log(L('✗ 速度中继没起来，遥控将直连 /cmd_vel'))
                tail = self.last_error()
                if tail:
                    self.log('   ' + tail)
            return ok

    def stop(self):
        with self._lock:
            self._stop_locked()

    def _stop_locked(self):
        if self.adopted:
            # 借来的中继不归我们关：另一个窗口还在用它
            self.adopted = False
            return
        p, self.proc = self.proc, None
        if p is None or p.poll() is not None:
            return
        try:
            if os.getpgid(p.pid) != os.getpgid(0):
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            else:
                p.terminate()
        except Exception:
            try:
                p.terminate()
            except Exception:
                pass


# 底盘页的方向键：值为 (线速度符号, 角速度符号)
MOVE_KEYMAP = {
    'w': (1, 0), 'up': (1, 0),
    's': (-1, 0), 'down': (-1, 0),
    'a': (0, 1), 'left': (0, 1),
    'd': (0, -1), 'right': (0, -1),
}


class App:
    def __init__(self, root):
        self.root = root
        self.robot = RobotROS()
        self._t_odom = tk.StringVar(value='—')
        self._t_volt = tk.StringVar(value='—')
        self._t_nodes = tk.StringVar(value=L('统计中…'))
        self._t_master = tk.StringVar(value=os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
        self._t_cam = tk.StringVar(value=L('等待识别'))
        # ROS 回调线程写这里，主线程的 _tick 再搬到界面（Tk 只能主线程碰）
        self._cache = {'odom': '—', 'volt': '—'}
        self._cache_lock = threading.Lock()
        # 子线程 -> 主线程的 UI 任务队列（Tk after 不能跨线程调，实测无效）
        self._ui_queue = queue.Queue()
        self._cam = CameraViewer()
        # 当前生效的灯带特效编号（0=停止）。必须只初始化一次：
        # _build_light() 在每次界面重建时都会跑，在这里赋初值会丢状态。
        self._rgb_effect = 0
        # 各关节上一次已知角度，用来按位移估 run_time（避免大角度动作被时间截断）
        self._arm_cur = {7: 90.0, 8: 100.0, 9: 150.0}
        self._cam_tk_rgb = self._cam_tk_depth = None
        self._nav_cam_t = None
        self._nav_cam_photo = None
        self._cam_last_rgb_t = self._cam_last_depth_t = 0.0
        self._cam_fps_n = 0
        self._cam_fps_t0 = time.time()
        self._cam_fps = 0.0
        # ---- 建图与导航页状态 ----
        # 栈、订阅、地图图都放在 NavState 里：切语言会销毁并重建所有控件，
        # 状态放控件上就会丢（正在跑的导航栈会变成没人管的孤儿进程）。
        self.nav = NavState(os.path.join(BASE_DIR, 'nav')) if _HAS_NAV else None
        # 速度中继：/cmd_vel 的唯一写入者（键盘与规划器共用出口）
        self.relay = DriveRelay(os.path.join(BASE_DIR, 'nav', 'helpers'),
                                self._log)
        self.panel = None
        self._move_keys = set()      # 底盘页当前按住的键
        # X11 的按键自动重复会成对发 KeyPress/KeyRelease：真的每 33 ms 停一次，
        # 车就是一冲一顿。松键后等 150 ms 再真正停车，中间来了新的 press 就取消。
        self._move_rel = {}          # key -> after id
        self._build_ui()
        # 键盘遥控：绑在 root 上一次即可。面板每次切语言都会重建，绑在面板上
        # 会越绑越多（add='+' 只增不减），最后一次按键被处理 N 遍。
        self.nb.bind('<<NotebookTabChanged>>', self._on_tab_change, add='+')
        self.root.bind('<KeyPress>', self._forward_key, add='+')
        self.root.bind('<KeyRelease>', self._forward_key_up, add='+')
        # 关键修复1：init_node 必须在主线程做，否则
        # "signal only works in main thread"
        try:
            self.robot.init_ros()
        except Exception as e:
            self._log(L('✗ ROS 环境初始化失败：%s') % e)
            self._set_status(L('● 连接失败'), '#d9534f')
        else:
            self._start_worker(self._ros_connect)
        self._spawn_subscribers()
        self.root.after(500, self._tick)
        self.root.after(200, self._panel_tick)
        self.root.after(200, self._camera_tick)
        # 关闭窗口时顺手 shutdown ROS，避免后台线程残留
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ================= 界面 =================
    def _build_ui(self):
        R = self.root
        R.title(L('Transbot 全方位机器人控制台 — Yahboom'))
        # 双图 1:1 显示需要 640x480 * 2 + 边距，初始给足宽度并允许自由缩放；
        # 高度 820 是为了让建图页底部的“紧急停止”在默认尺寸下也完整可见。
        R.geometry('1420x820')
        R.minsize(1000, 660)
        R.configure(bg='#1e2228')
        ts = ttk.Style()
        ts.theme_use('clam')
        ts.configure('.', font=('Ubuntu', 11))
        ts.configure('TFrame', background='#1e2228')
        ts.configure('TLabel', background='#1e2228', foreground='#d8dee9')
        ts.configure('H.TLabel', font=('Ubuntu', 13, 'bold'), foreground='#8cc5ff')
        ts.configure('Big.TLabel', font=('Ubuntu', 24, 'bold'), foreground='#ffd479')
        ts.configure('Green.TButton', background='#2f8a46', foreground='#ffffff')
        ts.configure('Red.TButton', background='#b33630', foreground='#ffffff')

        # 禁用态必须一眼看得出来。clam 默认的禁用按钮仍是浅灰底、禁用输入框
        # 仍是白底，在深色界面上跟可用控件几乎没区别——"灰掉"等于没灰。
        # 这里只改 disabled 这一档：禁用 = 沉进背景的深色 + 暗字，可用态
        # 的颜色一个不动。
        for st, bg, fg in (('TButton', '#dcdad5', '#000000'),
                           ('Green.TButton', '#2f8a46', '#ffffff'),
                           ('Red.TButton', '#b33630', '#ffffff')):
            ts.map(st,
                   background=[('disabled', '#2a2f36'), ('!disabled', bg)],
                   foreground=[('disabled', '#5b6570'), ('!disabled', fg)])
        ts.map('TEntry',
               fieldbackground=[('disabled', '#22262b'),
                                ('!disabled', '#ffffff')],
               foreground=[('disabled', '#5b6570'), ('!disabled', '#000000')])
        ts.map('TCombobox',
               fieldbackground=[('disabled', '#22262b'),
                                ('readonly', '#ffffff'),
                                ('!disabled', '#ffffff')],
               foreground=[('disabled', '#5b6570')])
        # 勾选框不在这里动：这个面板里的勾选框从来不会被禁用，改了反而把
        # 指示器涂成背景色，看不出选没选。

        # 所有控件都挂在这个容器下：切语言时整棵销毁重建，
        # 避免逐个控件去改 text。
        self._ui_root = tk.Frame(R, bg='#1e2228')
        self._ui_root.pack(fill='both', expand=True)

        # 顶部状态条（点击可复制 master 地址）
        head = tk.Frame(self._ui_root, bg='#1e2228')
        head.pack(fill='x', padx=12, pady=(10, 6))
        tk.Label(head, text=L('Transbot 全方位机器人控制台'),
                 font=('Ubuntu', 17, 'bold'),
                 fg='#e6f1ff', bg='#1e2228').pack(side='left')
        self._status = tk.Label(head, text=L('● 正在建立连接…'),
                                font=('Ubuntu', 12, 'bold'),
                                fg='#ffd479', bg='#1e2228', cursor='hand2')
        self._status.pack(side='right')
        self._status.bind('<Button-1>', lambda e: self._copy_text(
            STATE.get('master', '') or self._t_master.get(),
            L('ROS Master 地址已复制到剪贴板')))

        nb = ttk.Notebook(self._ui_root)
        nb.pack(fill='both', expand=True, padx=10, pady=(0, 8))

        self.nb = nb
        t_move = ttk.Frame(nb)
        self._f_move = t_move
        t_light = ttk.Frame(nb)
        t_arm = ttk.Frame(nb)
        t_cam = ttk.Frame(nb)
        t_map = ttk.Frame(nb)
        self._f_map = t_map          # 键盘路由要按页签 id 比较，不能拿面板对象比
        t_status = ttk.Frame(nb)
        nb.add(t_move, text=L(' 底盘运动 '))
        nb.add(t_light, text=L(' 灯光与声效 '))
        nb.add(t_arm, text=L(' 云台与机械臂 '))
        nb.add(t_cam, text=L(' 视觉传感器 '))
        nb.add(t_map, text=L(' 建图与导航 '))
        nb.add(t_status, text=L(' 系统状态 '))

        self._build_move(t_move)
        self._build_light(t_light)
        self._build_arm(t_arm)
        self._build_camera(t_cam)
        self._build_mapnav(t_map)
        self._build_status(t_status)

    # ---- 语言切换：快照 -> 销毁重建 -> 回填 ----
    def _switch_lang(self, code):
        if code not in STRINGS or code == LANG['code']:
            return
        snap = self._snapshot_ui()
        LANG['code'] = code
        try:
            self._ui_root.destroy()
        except Exception as e:
            self._log(L('✗ 界面重建失败：%s') % e)
            LANG['code'] = 'zh' if code == 'en' else 'en'
            return
        self._build_ui()
        self._restore_ui(snap)
        self._sync_rgb_buttons()
        self._log(L('界面语言已切换为英文') if code == 'en'
                  else L('界面语言已切换为中文'))

    def _snapshot_ui(self):
        """切换语言前把用户可见状态存下来，重建后原样回填。

        只回填与语言无关的原始量（速度/角度/电压/计数/话题）。
        _status / _t_camsrc / _t_camstat 里存的是已经翻译过的整句，
        照搬旧值会把中文串留在英文界面里，这三处改由 _rerender_status 重绘。
        """
        s = {}
        try:
            s['logs'] = self._st_txt.get('1.0', 'end-1c')
        except Exception:
            s['logs'] = ''
        s['speed'] = self._sp_speed.get()
        s['arm'] = [self._arm_sliders[j].get() for j in (7, 8, 9)]
        for name in ('_t_odom', '_t_volt', '_t_nodes', '_t_cam', '_t_master'):
            s[name] = getattr(self, name).get()
        return s

    def _restore_ui(self, s):
        try:
            self._st_txt.delete('1.0', 'end')
            self._st_txt.insert('1.0', s['logs'])
        except Exception:
            pass
        try:
            self._sp_speed.set(s['speed'])
            for j, v in zip((7, 8, 9), s['arm']):
                self._arm_sliders[j].set(v)
            for name in ('_t_odom', '_t_volt', '_t_nodes', '_t_cam', '_t_master'):
                getattr(self, name).set(s[name])
        except Exception:
            pass
        self._rerender_status()

    def _rerender_status(self):
        """语言切换后，用新语言重绘顶部连接状态与相机源/状态两行。"""
        try:
            if STATE.get('ok'):
                self._status.configure(
                    text=L('● 已连接 ')
                    + STATE.get('master', '').replace('http://', ''),
                    fg='#7fd88f')
            elif STATE.get('err'):
                self._status.configure(text=L('● 连接失败'), fg='#d9534f')
            else:
                self._status.configure(
                    text=L('● 正在建立连接…'), fg='#ffd479')
        except Exception:
            pass
        try:
            if getattr(self._cam, 'viewing', False):
                self._t_camsrc.set(L('数据源：%s%s') % (
                    self._cam.rgb_topic,
                    (' + ' + self._cam.depth_topic)
                    if self._cam.depth_topic else L('（无深度流）')))
            else:
                self._t_camsrc.set(L('数据源：待连接…'))
                self._t_camstat.set(
                    L('未采集（默认关闭以节省无线带宽）'))
        except Exception:
            pass

    # ---- 页1: 底盘运动 ----
    def _build_move(self, f):
        # 整页只有三块内容，全部居中放：贴左上角会显得像没做完。
        # 之前用 place(relx=.5) 居中，但内容宽度超过窗口时会被左右切掉，
        # 所以改成 pack(expand=True) + 长文案 wraplength 自动折行。
        body = ttk.Frame(f)
        body.pack(expand=True, fill='both')
        card = ttk.Frame(body)
        card.pack(expand=True)

        ttk.Label(card, text=L('底盘运动（键盘控制）'), style='H.TLabel')\
            .pack(pady=(0, 10))

        # 键位图例：两列等宽，整体居中
        keys = ttk.Frame(card)
        keys.pack(pady=(0, 4))
        legend = [
            ('W  /  S', '前进 / 后退'),
            ('A  /  D', '左转 / 右转'),
            (L('空格'), '立即停车'),
        ]
        for r, (k, what) in enumerate(legend):
            ttk.Label(keys, text=k, font=('TkFixedFont', 14, 'bold'),
                      width=9, anchor='e').grid(row=r, column=0, sticky='e', pady=4)
            ttk.Label(keys, text=L(what), anchor='w').grid(
                row=r, column=1, sticky='w', padx=(14, 0))

        ttk.Label(card, text=L('连续速度控制（键盘用这两个值）'), style='H.TLabel')\
            .pack(pady=(18, 6))

        joystick = ttk.Frame(card)
        joystick.pack()
        # 范围按驱动硬限幅设（linear_speed_limit=0.4 / angular=2.0），
        # 之前给到 ±1.0 但驱动会钳回 0.4，滑杆看着"失灵"。
        # 标题用独立 Label：tk.Scale 自带的 label 会按 length 截断（"Angular
        # speed (ra…"），而且会把槽体挤到框外。
        cols = [
            ('线速度 m/s', '_sl_speed', -RobotROS.LIN_MAX, RobotROS.LIN_MAX, 0.05, 0.20),
            ('角速度 rad/s', '_sl_turn', -RobotROS.ANG_MAX, RobotROS.ANG_MAX, 0.1, 0.60),
        ]
        for title, attr, lo, hi, res, init in cols:
            col = ttk.Frame(joystick)
            col.pack(side='left', padx=12)
            ttk.Label(col, text=L(title)).pack(pady=(0, 2))
            sl = tk.Scale(col, from_=lo, to=hi, resolution=res, orient='horizontal',
                          length=240, width=14, sliderlength=20, showvalue=True)
            # 默认给一个"按下去就能走"的值：0 会让方向键看起来没反应。
            sl.set(init)
            sl.pack()
            setattr(self, attr, sl)

        ttk.Button(card, text=L('紧急停止'), style='Red.TButton',
                   command=self.cmd_stop).pack(pady=(18, 0))

        self._move_hint = ttk.Label(
            card, justify='center', wraplength=640,
            text=L('按住方向键持续运动，松开即停；只有本页可见时键盘才生效。'
                   '导航运行中按方向键会先取消目标再接管。'
                   '速度范围取自底盘驱动限幅（0.4 m/s、2.0 rad/s）。'))
        self._move_hint.pack(pady=(22, 0))
        # 窗口变窄时按宽度重新折行，避免文案被切掉
        card.bind('<Configure>', self._wrap_move_hint)

    def _wrap_move_hint(self, ev):
        try:
            self._move_hint.configure(wraplength=max(320, ev.width - 40))
        except Exception:
            pass

    # ---- 页2: 灯光与声效 ----
    def _build_light(self, f):
        ttk.Label(f, text=L('RGB 灯带特效（协议定义 0~6）'), style='H.TLabel')\
            .pack(anchor='w', padx=16, pady=(12, 4))
        fx = ttk.Frame(f)
        fx.pack(anchor='w', padx=16)
        fx2 = ttk.Frame(f)
        fx2.pack(anchor='w', padx=16, pady=(6, 0))
        # 特效编号依据 transbot_joy.py:309 的官方注释。
        # 6=电量显示是静态的（按电池电压显示颜色），不变化属于正常现象。
        self._EFFECTS = [
            ('停止', 0), ('流水灯', 1), ('跑马灯', 2), ('呼吸灯', 3),
            ('渐变灯', 4), ('星光点点', 5), ('电量显示', 6),
        ]
        # self._rgb_effect 在 __init__ 里初始化：这里若再赋初值，
        # 切语言重建控件时会把用户已选的灯效抹掉。
        self._rgb_btns = {}
        # 第一行 4 个在 fx，第二行 3 个在 fx2；两个 Frame 各自从 row 0 排，
        # 否则第二组会出现一个空行（之前 i//4 让第二组落在 row=1）。
        ew = max(fit(L(t)) for t, _ in self._EFFECTS)
        for i, (txt, eid) in enumerate(self._EFFECTS):
            if i < 4:
                b = ttk.Button(fx, text=L(txt), width=ew,
                               command=lambda e=eid: self._set_rgb_effect(e))
                b.grid(row=0, column=i, padx=4, pady=4)
            else:
                b = ttk.Button(fx2, text=L(txt), width=ew,
                               command=lambda e=eid: self._set_rgb_effect(e))
                b.grid(row=0, column=i - 4, padx=4, pady=4)
            self._rgb_btns[eid] = b
        self._sync_rgb_buttons()

        sp = ttk.Frame(f)
        sp.pack(anchor='w', padx=16, pady=(8, 0))
        ttk.Label(sp, text=L('变化速度（1 最快 ~ 10 最慢）')).pack(side='left')
        self._sp_speed = tk.Scale(sp, from_=1, to=10, orient='horizontal', length=180)
        self._sp_speed.set(5)
        self._sp_speed.pack(side='left', padx=8)
        ttk.Button(sp, text=L('应用速度'), style='Green.TButton',
                   width=fit(L('应用速度')),
                   command=self._apply_rgb_speed).pack(side='left', padx=(4, 0))
        self._lb_rgb_state = tk.StringVar(value='')
        ttk.Label(sp, textvariable=self._lb_rgb_state,
                  foreground='#9aa5b1').pack(side='left', padx=10)
        ttk.Label(f, text=L('速度参数对已播放的特效立即生效；'
                            '“电量显示”为静态效果，速度参数对其无视觉影响'),
                  foreground='#9aa5b1').pack(anchor='w', padx=16, pady=(2, 0))

        ttk.Label(f, text=L('蜂鸣器'), style='H.TLabel')\
            .pack(anchor='w', padx=16, pady=(16, 4))
        bx = ttk.Frame(f)
        bx.pack(anchor='w', padx=16)
        bz = max(fit(L(t)) for t in ('短鸣 0.3 s', '长鸣 2 s', '停止鸣响'))
        ttk.Button(bx, text=L('短鸣 0.3 s'), width=bz,
                   command=lambda: self.cmd('buz', 200, 0.3)).pack(side='left', padx=4)
        ttk.Button(bx, text=L('长鸣 2 s'), width=bz,
                   command=lambda: self.cmd('buz', 300, 2.0)).pack(side='left', padx=4)
        ttk.Button(bx, text=L('停止鸣响'), style='Red.TButton', width=bz,
                   command=lambda: self.cmd('buz', 0)).pack(side='left', padx=4)

    # ---- 灯带：特效选择与速度下发 ----
    EFFECT_NAMES = {0: '停止', 1: '流水灯', 2: '跑马灯', 3: '呼吸灯',
                    4: '渐变灯', 5: '星光点点', 6: '电量显示'}

    def arm_name(self, jid):
        """关节短名（去掉 '(id7)' 后缀），随界面语言翻译。"""
        return T(RobotROS.ARM_NAMES)[jid].split('(')[0].strip()

    def _sync_rgb_buttons(self):
        """把当前生效的特效按钮标绿，其余恢复默认，便于一眼看出灯带在放什么。"""
        for eid, b in self._rgb_btns.items():
            try:
                b.configure(style='Green.TButton' if eid == self._rgb_effect
                            else 'TButton')
            except Exception:
                pass

    def _sync_lang_buttons(self):
        """高亮当前生效的界面语言。"""
        for code, b in getattr(self, '_lang_btns', {}).items():
            try:
                b.configure(style='Green.TButton' if code == LANG['code']
                            else 'TButton')
            except Exception:
                pass

    def _set_rgb_effect(self, effect):
        self._rgb_effect = int(effect)
        self._sync_rgb_buttons()
        self._apply_rgb_speed()

    def _apply_rgb_speed(self):
        """把当前特效 + 当前速度重新下发一次。
        speed 是 set_colorful_effect 的真实参数（官方文档：1~10，越小越快），
        重复下发同一个 effect 只会改速度，不会重启特效。
        """
        eff = self._rgb_effect
        spd = max(1, min(10, int(round(float(self._sp_speed.get())))))
        if eff == 0:
            self._log(L('○ 当前为停止状态，请先选择一个特效再调节速度'))
            return
        self._lb_rgb_state.set(
            L('当前：%s，速度 %d') % (T(self.EFFECT_NAMES)[eff], spd))
        self.cmd('rgb', eff, spd)

    # ---- 页3: 云台 + 机械臂 ----
    def _build_arm(self, f):
        # 云台只留偏航(id1)。俯仰(id2)不要了：Astra 单轴支架上根本没有这个舵机，
        # 发了不会有任何动作；USBCam 双轴版虽然有，但这个机器人装的就是单轴的。
        # 限位取自 transbot_driver.py:sub_PWMServocallback（CameraDevice=astra 时
        # id1 被钳在 60~120），滑杆就用这个区间，避免推到钳制区外"看着没反应"。
        ttk.Label(f, text=L('云台偏航（关节 id1，角度范围 60~120）'),
                  style='H.TLabel').pack(anchor='w', padx=16, pady=(12, 4))
        gx = ttk.Frame(f)
        gx.pack(anchor='w', padx=16)
        self._s_yaw = tk.Scale(gx, from_=60, to=120, orient='horizontal',
                               length=280, label=L('偏航角 (id1)'),
                               showvalue=True)
        self._s_yaw.set(90)
        self._s_yaw.pack(side='left', padx=12)
        ttk.Button(gx, text=L('发送指令'), style='Green.TButton',
                   command=lambda: self.cmd('servo', 1, self._s_yaw.get())
                   ).pack(side='left', padx=(4, 8))
        ttk.Button(gx, text=L('复位至 90°'),
                   command=lambda: self.cmd('servo', 1, 90)).pack(side='left')
        self._lb_gimbal_note = ttk.Label(
            gx, text='', foreground='#ffd479', background='#1e2228', wraplength=200)
        self._lb_gimbal_note.pack(side='left', padx=12)

        # ---- 机械臂：关节角度手动控制 ----
        # 不提供预设姿态：机械臂可安装在任意朝向与位置，写死的角度组合只在
        # 某一种装法下成立。滑杆以实测角度为基准，与安装姿态无关。
        ttk.Label(f, text=L('机械臂关节角度控制'), style='H.TLabel')\
            .pack(anchor='w', padx=16, pady=(20, 4))
        bx2 = ttk.Frame(f)
        bx2.pack(anchor='w', padx=16)
        ttk.Button(bx2, text=L('读取当前角度'), width=fit(L('读取当前角度')),
                   command=lambda: self.cmd('read_arm')).pack(side='left', padx=(0, 8))
        ttk.Label(bx2, text=L('经 /CurrentAngle 服务读取关节 7/8/9 的实测角度，'
                              '结果自动回填至下方滑块')).pack(side='left')
        ttk.Label(f, text=L('关节限位依据 URDF 定义：底座 0~225°　升降 15~240°　夹爪 60~180°'),
                  foreground='#9aa5b1').pack(anchor='w', padx=16, pady=(4, 0))
        self._arm_sliders = {}
        for jid in (7, 8, 9):
            lo, hi = self.robot.ARM_LIMITS[jid]
            row = ttk.Frame(f)
            row.pack(fill='x', padx=16, pady=2)
            ttk.Label(row, text=L(RobotROS.ARM_NAMES[jid]),
                      width=max(fit(L(v)) for v in RobotROS.ARM_NAMES.values()))\
                .pack(side='left')
            sl = tk.Scale(row, from_=lo, to=hi, resolution=1, orient='horizontal',
                          length=300, showvalue=True)
            sl.pack(side='left', padx=(0, 8))
            self._arm_sliders[jid] = sl
            ttk.Button(row, text='−', width=3,
                       command=lambda j=jid: self._jog_arm(j, -1)).pack(side='left')
            ttk.Button(row, text='+', width=3,
                       command=lambda j=jid: self._jog_arm(j, +1)).pack(side='left')
            ttk.Button(row, text=L('发送'), width=fit(L('发送')), style='Green.TButton',
                       command=lambda j=jid: self._send_arm_joint(j)).pack(side='left', padx=(2, 0))
        # 滑块初值不对应任何标准姿态：连接建立后会自动读取实测角度并回填，
        # 因此初值取限位下限，发送前建议先确认读数。
        for jid in (7, 8, 9):
            self._arm_sliders[jid].set(self.robot.ARM_LIMITS[jid][0])

        ab = ttk.Frame(f)
        ab.pack(anchor='w', padx=16, pady=(8, 0))
        ttk.Label(ab, text=L('步长 (°)')).pack(side='left')
        self._step_entry = ttk.Entry(ab, width=6)
        self._step_entry.insert(0, '5')
        self._step_entry.pack(side='left', padx=(4, 12))
        ttk.Button(ab, text=L('三轴同步下发'), style='Green.TButton',
                   command=self._send_arm_all).pack(side='left')

    # ---- 相机画面（RGB + 深度） ----
    def _build_camera(self, f):
        ttk.Label(f, text=L('视觉传感器图像（Astra：左彩色 + 右深度伪彩；USBCam：仅彩色）'),
                  style='H.TLabel').pack(anchor='w', padx=16, pady=(12, 4))
        bar = ttk.Frame(f)
        bar.pack(fill='x', padx=16, pady=(0, 6))
        self._t_camsrc = tk.StringVar(value=L('数据源：待连接…'))
        ttk.Label(bar, textvariable=self._t_camsrc).pack(side='left', padx=(0, 12))
        ttk.Button(bar, text=L('开始采集'), style='Green.TButton',
                   command=self._cam_open).pack(side='left', padx=4)
        ttk.Button(bar, text=L('停止采集'), style='Red.TButton',
                   command=self._cam_close).pack(side='left', padx=4)
        ttk.Button(bar, text=L('启动Astra驱动'), style='Green.TButton',
                   command=self._astra_open).pack(side='left', padx=(12, 4))
        ttk.Button(bar, text=L('停止Astra驱动'), style='Red.TButton',
                   command=self._astra_close).pack(side='left', padx=4)
        self._t_camstat = tk.StringVar(
            value=L('未采集（默认关闭以节省无线带宽）'))
        ttk.Label(bar, textvariable=self._t_camstat).pack(side='left', padx=12)
        if not _CAM_OK:
            ttk.Label(f, text=L('显示依赖缺失：需安装 sensor_msgs、opencv、numpy、PIL'))\
                .pack(anchor='w', padx=16)

        vf = ttk.Frame(f)
        vf.pack(fill='both', expand=True, padx=16, pady=6)
        # 1 像素宽高 + 图像驱动 Label 自动撑开到实际图像尺寸，
        # 这样占位文字和真实图像两种状态下窗口大小都稳定。
        self._lb_rgb = tk.Label(vf, text=L('彩色图像：等待数据…'), bg='#101318',
                                fg='#8cc5ff', width=1, height=1, compound='top',
                                anchor='center')
        self._lb_rgb.pack(side='left', padx=(0, 10), fill='both', expand=True)
        self._lb_depth = tk.Label(vf, text=L('深度图像：等待数据…'), bg='#101318',
                                  fg='#8cc5ff', width=1, height=1, compound='top',
                                  anchor='center')
        self._lb_depth.pack(side='left', fill='both', expand=True)
        ttk.Label(f, text=L('图像按原始分辨率 1:1 显示。实测无线带宽下彩色与深度各约 '
                            '1.2 帧/秒（单帧 900 KB），帧率受 WiFi 限制而非本程序处理能力；'
                            '若画面为空，可在终端用 rostopic hz 检查 '
                            '/camera/rgb/image_raw、'
                            '/image、/camera/depth/image_raw 的发布频率'),
                  wraplength=880).pack(anchor='w', padx=16, pady=(0, 8))

    def _cam_open(self):
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，无法启动图像采集'))
            return
        if not _CAM_OK:
            self._log(L('✗ 显示依赖缺失（cv2 / numpy / PIL / sensor_msgs 之一未安装）'))
            return
        # 按已探测到的相机型号选话题
        dev = self._t_cam.get()
        self._cam.set_source('USBCam' if 'USBCam' in dev else 'Astra')
        if self._cam.viewing:
            self._log(L('○ 图像采集已处于运行状态'))
            return
        if self._cam.start():
            self._t_camsrc.set(L('数据源：%s%s') % (
                self._cam.rgb_topic,
                (' + ' + self._cam.depth_topic) if self._cam.depth_topic
                else L('（无深度流）')))
            self._t_camstat.set(L('采集中…'))
            self._log(L('✓ 图像采集已启动 %s') % self._t_camsrc.get())
            # 话题订阅成功 /= 有画面：驱动没起时订阅照样成立，画面永远空着。
            self.root.after(3500, self._cam_nudge)
        else:
            self._log(L('✗ 图像采集启动失败：%s')
                      % (self._cam._last_err or L('未知原因')))

    def _cam_nudge(self):
        """采集开了却没帧：多半是 Astra 驱动没起，指一下旁边那个按钮。"""
        try:
            if not self._cam.viewing:
                return
            rgb, _t, _d, _dt = self._cam.latest()
            if rgb is not None:
                return
            self._log(L('○ 还没有图像帧：Astra 驱动可能没启动，点「启动Astra驱动」'))
        except Exception:
            pass

    def _cam_close(self):
        self._cam.stop()
        self._t_camstat.set(L('已停止'))

    def _astra_open(self):
        """后台远启 Astra 驱动（SSH 阻塞，不可占主线程）。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return

        def work():
            try:
                if self.robot.astra_running(timeout=2):
                    self._log(L('○ Astra驱动已在运行，话题 %s')
                              % self.robot.ASTRA_RGB_TOPIC)
                    return
                self._log(L('→ 正在机器人上启动Astra驱动…'))
                topic = self.robot.astra_start()
                self._log(L('✓ Astra驱动已启动，话题 %s') % topic)
            except Exception as e:
                msg = str(e)
                if 'in progress' in msg:
                    self._log(L('○ Astra操作进行中，请稍候'))
                elif 'sshpass' in msg or 'key auth' in msg:
                    self._log(L('✗ 本机缺sshpass且无免密SSH，无法远启：%s') % e)
                else:
                    self._log(L('✗ Astra驱动启动失败：%s') % e)
        self._start_worker(work)

    def _astra_close(self):
        """后台停止 Astra 驱动。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return

        def work():
            try:
                self._log(L('→ 正在停止Astra驱动…'))
                self.robot.astra_stop()
                # 驱动已停，本地订阅再留着只会显示 stale 帧并 fallback 到 /image，
                # 主动停掉本地采集（经 _ui 回主线程），界面回到“已停止”。
                self._ui(self._cam.stop)
                self._ui(lambda: self._t_camstat.set(L('已停止')))
                self._log(L('✓ Astra驱动已停止'))
            except Exception as e:
                if 'in progress' in str(e):
                    self._log(L('○ Astra操作进行中，请稍候'))
                else:
                    self._log(L('✗ Astra驱动停止失败：%s') % e)
        self._start_worker(work)

    def _camera_tick(self):
        # 主线程解码上屏。刷新上限 8fps，实际受 WiFi 带宽限制约 1.2fps。
        try:
            if self._cam.viewing and _CAM_OK:
                fb = self._cam.maybe_fallback()
                if fb:
                    self._t_camsrc.set(
                        L('数据源：%s（上一话题无数据，已自动切换）') % fb)
                    self._log(L('○ 当前彩色话题无数据，已自动切换订阅 %s') % fb)
                rgb_msg, rgb_t, depth_msg, depth_t = self._cam.latest()
                now = time.time()
                if rgb_msg is not None and rgb_t != self._cam_last_rgb_t:
                    self._cam_last_rgb_t = rgb_t
                    img = decode_rgb_raw(rgb_msg)
                    if img is not None:
                        # 只在超出目标尺寸时缩小，不放大小图，避免无谓的插值开销
                        h, w = img.shape[:2]
                        if (w, h) != DISPLAY_SIZE:
                            scale = min(DISPLAY_SIZE[0] / float(w),
                                        DISPLAY_SIZE[1] / float(h))
                            if scale < 1.0:
                                img = cv.resize(img, (int(w * scale), int(h * scale)),
                                                interpolation=cv.INTER_AREA)
                        rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
                        tk_img = PilImageTk.PhotoImage(PilImage.fromarray(rgb))
                        self._cam_tk_rgb = tk_img  # 保持引用防回收
                        self._lb_rgb.config(image=tk_img, text='')
                        self._cam_fps_n += 1
                    elif now - rgb_t > 3:
                        self._lb_rgb.config(text=L('彩色图像解码失败'))
                elif rgb_msg is None and now - self._cam_fps_t0 > 3 and self._cam.viewing:
                    self._lb_rgb.config(text=L('彩色图像：话题无数据'))
                if self._cam.depth_topic is None:
                    _no_depth = L('当前相机无深度数据流')
                    if str(self._lb_depth.cget('text')) != _no_depth:
                        self._lb_depth.config(image='', text=_no_depth)
                elif depth_msg is not None and depth_t != self._cam_last_depth_t:
                    self._cam_last_depth_t = depth_t
                    dimg = decode_depth_colormap(depth_msg)
                    if dimg is not None:
                        h, w = dimg.shape[:2]
                        if (w, h) != DISPLAY_SIZE:
                            scale = min(DISPLAY_SIZE[0] / float(w),
                                        DISPLAY_SIZE[1] / float(h))
                            if scale < 1.0:
                                dimg = cv.resize(dimg, (int(w * scale), int(h * scale)),
                                                 interpolation=cv.INTER_AREA)
                        tk_d = PilImageTk.PhotoImage(
                            PilImage.fromarray(cv.cvtColor(dimg, cv.COLOR_BGR2RGB)))
                        self._cam_tk_depth = tk_d
                        self._lb_depth.config(image=tk_d, text='')
                # 帧率与延迟统计
                if now - self._cam_fps_t0 >= 2.0:
                    self._cam_fps = self._cam_fps_n / max(0.1, now - self._cam_fps_t0)
                    self._cam_fps_n = 0
                    self._cam_fps_t0 = now
                    age = (L('　彩色帧延迟 %.1f s') % (now - rgb_t)) if rgb_t else ''
                    self._t_camstat.set('%.1f fps%s' % (self._cam_fps, age))
        except Exception:
            pass
        finally:
            try:
                self.root.after(120, self._camera_tick)
            except Exception:
                pass

    # ---- 页5: 建图与导航 ----
    # 正本模型：地图文件只认机器人 ~/transbot_ws/src/transbot_nav/maps/，
    # 本机只看（scp 下 pgm 进 /tmp 弹窗）或冷备份。删除/重命名去机器人端做。
    def _build_mapnav(self, f):
        """建图与导航：三种模式共用一块地图画布。

        面板在 nav/panel.py，这里只负责挂上去。控件每次切语言都会重建，
        但运行中的栈、订阅和地图图在 App.nav（NavState）里，所以切语言
        不会把正在跑的导航弄丢。
        """
        try:
            self.panel = MapPanel(f, self, L)
            self.panel.pack(fill='both', expand=True)
        except Exception as e:
            self.panel = None
            self._log(L('✗ 建图导航面板初始化失败：%s')
                      % (NAV_IMPORT_ERROR or e))

    # ---- 建图/导航面板的胶水（面板自己管 8 Hz 重绘，这里只管转发） ----
    def nav_camera_frame(self, box_w, box_h):
        """建图页右侧的小预览：复用视觉页的采集，只做缩放，不另开订阅。"""
        if not (_CAM_OK and self._cam.viewing):
            return None
        rgb_msg, rgb_t, _depth, _dt = self._cam.latest()
        if rgb_msg is None:
            return None
        if rgb_t == self._nav_cam_t:
            return self._nav_cam_photo
        self._nav_cam_t = rgb_t
        img = decode_rgb_raw(rgb_msg)
        if img is None:
            return None
        h, w = img.shape[:2]
        scale = min(box_w / float(w), box_h / float(h))
        if scale < 1.0:
            img = cv.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))),
                            interpolation=cv.INTER_AREA)
        self._nav_cam_photo = PilImageTk.PhotoImage(
            PilImage.fromarray(cv.cvtColor(img, cv.COLOR_BGR2RGB)))
        return self._nav_cam_photo

    def _sync_nav(self):
        """按连接状态刷新建图页按钮。主线程调用。"""
        try:
            if self.panel is not None:
                self.panel._sync()
        except Exception:
            pass

    def _panel_tick(self):
        try:
            if self.panel is not None:
                self.panel.tick()
        except Exception:
            pass
        finally:
            self.root.after(80, self._panel_tick)

    def _tab_kind(self):
        """当前可见的是哪一页：'move'（底盘）、'nav'（建图导航）或 None。"""
        try:
            cur = self.nb.select()
            if cur == str(self._f_move):
                return 'move'
            if self.panel is not None and cur == str(getattr(self, '_f_map', None)):
                return 'nav'
        except Exception:
            pass
        return None

    def _forward_key(self, ev):
        kind = self._tab_kind()
        if kind == 'move':
            return self._move_key(ev)
        if kind == 'nav' and self.panel is not None:
            try:
                return self.panel.on_key(ev)
            except Exception:
                return None
        return None

    def _forward_key_up(self, ev):
        kind = self._tab_kind()
        if kind == 'move':
            return self._move_key_up(ev)
        if kind == 'nav' and self.panel is not None:
            try:
                return self.panel.on_key_up(ev)
            except Exception:
                return None
        return None

    # ---- 底盘页：按住即走 ----
    KEY_RELEASE_DELAY = 150          # ms

    def _typing_somewhere(self):
        """焦点在输入框里时不要抢键（否则在日志框/地图名里打字会开车）。"""
        try:
            focus = self.root.focus_get()
        except Exception:
            focus = None
        return isinstance(focus, (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox))

    def _move_key(self, ev):
        if not self.robot.connected():
            return None
        if self._typing_somewhere():
            return None
        key = ev.keysym.lower()
        if key == 'space':
            self.cmd_stop()
            return 'break'
        if key not in MOVE_KEYMAP:
            return None
        # 自动重复的 press 到达 -> 取消刚排上的停车
        t = self._move_rel.pop(key, None)
        if t is not None:
            try:
                self.root.after_cancel(t)
            except Exception:
                pass
        if key not in self._move_keys:
            self._move_keys.add(key)
            self._apply_move()
        return 'break'

    def _move_key_up(self, ev):
        key = ev.keysym.lower()
        if key not in self._move_keys:
            return None
        t = self._move_rel.pop(key, None)
        if t is not None:
            try:
                self.root.after_cancel(t)
            except Exception:
                pass
        self._move_rel[key] = self.root.after(
            self.KEY_RELEASE_DELAY, lambda k=key: self._release_move_key(k))
        return 'break'

    def _release_move_key(self, key):
        self._move_rel.pop(key, None)
        if key in self._move_keys:
            self._move_keys.discard(key)
            self._apply_move()

    def _apply_move(self):
        """按住哪些键 -> 发什么速度。速度幅度取两个滑杆。"""
        lin = abs(self._sl_speed.get())
        ang = abs(self._sl_turn.get())
        lx = az = 0.0
        for k in self._move_keys:
            f, t = MOVE_KEYMAP[k]
            lx += f * lin
            az += t * ang
        lx = max(-RobotROS.LIN_MAX, min(RobotROS.LIN_MAX, lx))
        az = max(-RobotROS.ANG_MAX, min(RobotROS.ANG_MAX, az))
        try:
            if abs(lx) < 1e-3 and abs(az) < 1e-3:
                self.robot.stop_manual()
            else:
                # 导航正在跑就先把目标取消：键盘接管，别让 move_base 和手抢
                if self.nav is not None and self.nav.goal is not None:
                    self.nav.cancel_goal()
                    self._log(L('○ 手动遥控接管，导航目标已取消'))
                self.robot.manual(lx, az, hold=0)
        except Exception as e:
            self._log(L('✗ 遥控指令失败：%s') % e)

    def cmd_stop(self):
        """停车：清空按键 + 发一条 0 速度。"""
        self._move_keys.clear()
        try:
            self.cmd('stop')
        except Exception as e:
            self._log(L('✗ 制动失败：%s') % e)

    def _on_tab_change(self, _ev=None):
        """切页时清掉按住的键：否则按住 W 切页，车会一直走。"""
        for t in list(self._move_rel.values()):
            try:
                self.root.after_cancel(t)
            except Exception:
                pass
        self._move_rel.clear()
        if self._move_keys:
            self._move_keys.clear()
            try:
                self.robot.stop_manual()
            except Exception:
                pass


    # ---- 页6: 系统状态（可复制） ----
    def _build_status(self, f):
        ttk.Label(f, text=L('运行状态（单击条目可复制其内容）'), style='H.TLabel')\
            .pack(anchor='w', padx=16, pady=(12, 4))

        # ---- 界面语言切换 ----
        langbar = ttk.Frame(f)
        langbar.pack(anchor='w', padx=16, pady=(0, 2))
        ttk.Label(langbar, text=L('界面语言'), style='H.TLabel')\
            .pack(side='left', padx=(0, 10))
        self._lang_btns = {}
        # 语言名称固定使用各自母语写法，不随当前界面语言翻译：
        # 这样用户在任何语言下都能一眼认出目标语言。
        for code, name in (('zh', '中文'), ('en', 'English')):
            b = ttk.Button(langbar, text=name, width=10,
                           command=lambda c=code: self._switch_lang(c))
            b.pack(side='left', padx=4)
            self._lang_btns[code] = b
        self._sync_lang_buttons()
        ttk.Label(f, text=L('切换语言只重建界面文字，运行日志与设备状态保持不变。'),
                  foreground='#9aa5b1').pack(anchor='w', padx=16, pady=(0, 6))

        info = ttk.Frame(f)
        info.pack(fill='x', padx=16, pady=(0, 6))
        # 用 readonly Entry 展示：原生支持鼠标选中 + Ctrl+C
        def _row(r, title, var):
            ttk.Label(info, text=title).grid(row=r, column=0, sticky='w', pady=2)
            e = ttk.Entry(info, textvariable=var, state='readonly', width=52)
            e.grid(row=r, column=1, sticky='we', padx=(4, 4), pady=2)
            e.bind('<Button-1>', lambda ev, v=var: self._copy_text(
                v.get(), L('已复制到剪贴板：') + v.get()))
            return e
        info.columnconfigure(1, weight=1)
        _row(0, 'ROS Master', self._t_master)
        _row(1, L('相机型号'), self._t_cam)
        _row(2, L('底盘速度'), self._t_odom)
        _row(3, L('电池电压'), self._t_volt)
        _row(4, L('节点数量'), self._t_nodes)

        bar = ttk.Frame(f)
        bar.pack(fill='x', padx=16, pady=(2, 4))
        ttk.Button(bar, text=L('复制全部日志'),
                   command=self._copy_all_logs).pack(side='left', padx=(0, 6))
        ttk.Button(bar, text=L('清空日志'),
                   command=self._clear_logs).pack(side='left')

        box = ttk.Frame(f)
        box.pack(fill='both', expand=True, padx=16, pady=6)
        # 关键修复2：之前 state='disabled' 的 Text 鼠标选不中、Ctrl+C 无效。
        # 这里用只读式 Text：state 保持 normal 以支持选中/复制，用按键拦截禁编辑。
        self._st_txt = tk.Text(box, height=12, font=('monospace', 11), bg='#101318',
                               fg='#9fe0a0', relief='flat', wrap='word',
                               selectbackground='#264f78', inactiveselectbackground='#264f78')
        sb = ttk.Scrollbar(box, orient='vertical', command=self._st_txt.yview)
        self._st_txt.configure(yscrollcommand=sb.set)
        self._st_txt.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        self._st_txt.bind('<Key>', self._on_log_key)
        # 右键菜单：复制选中 / 复制全部 / 清空
        self._menu = tk.Menu(self._st_txt, tearoff=0)
        self._menu.add_command(label=L('复制选中'), command=self._copy_selected_logs)
        self._menu.add_command(label=L('复制全部'), command=self._copy_all_logs)
        self._menu.add_separator()
        self._menu.add_command(label=L('清空'), command=self._clear_logs)
        self._st_txt.bind('<Button-3>', self._popup_menu)
        self._st_txt.bind('<Control-c>', lambda e: self._copy_selected_logs())
        self._st_txt.bind('<Control-a>', lambda e: (self._st_txt.tag_add('sel', '1.0', 'end'), 'break')[1])

    # ================= 运行 =================
    def cmd(self, kind, *a):
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        fn = {'forward': self.robot.straight,
              'backward': self.robot.straight,
              'turn': self.robot.turn,
              'stop': self.robot.stop,
              'rgb': self.robot.rgb,
              'buz': self.robot.buzzer,
              'servo': self.robot.servo,
              'arm': self.robot.arm,
              'read_arm': self._read_arm_and_log}[kind]
        # backward 必须传负距离（界面已传 -0.5）；这里不再做符号变换

        label = L(RobotROS.CMD_NAMES.get(kind, kind))

        def wrap():
            try:
                r = fn(*a)
                args = '/'.join(str(x) for x in a)
                if kind == 'read_arm':
                    pass  # _read_arm_and_log 内部已记日志
                elif kind in ('rgb', 'buz'):
                    self._log(L('✓ %s 指令已下发：参数 %s，服务响应 %s')
                              % (label, args, r))
                else:
                    self._log(L('✓ %s 指令已下发：参数 %s') % (label, args))
            except rospy.ServiceException as e:
                self._log(L('✗ %s 指令失败：服务端异常 %s') % (label, e))
            except Exception as e:
                self._log(L('✗ %s 指令失败：%s') % (label, e))
        threading.Thread(target=wrap, daemon=True).start()

    def _read_arm_and_log(self):
        try:
            r = self.robot.read_angles()
            joints = getattr(getattr(r, 'RobotArm', r), 'joint', [])
            parts = ['id%d=%.0f' % (j.id, j.angle) for j in joints]
            self._log(L('✓ 机械臂关节角度：%s')
                      % (' '.join(parts) if parts else r))
            # 读回的值回填滑杆：这样滑杆显示的是真实姿态，不只是上次输入值。
            # 放在后台线程，所以要经 _ui 回到主线程才能动 Tk。
            def _sync():
                for j in joints:
                    sl = self._arm_sliders.get(int(j.id))
                    if sl is not None:
                        sl.set(int(round(float(j.angle))))
                if joints:
                    self._log(L('○ 关节滑块已同步至实测角度'))
            self._ui(_sync)
        except Exception as e:
            self._log(L('✗ 读取机械臂角度失败，请确认 ROS Master 与机械臂驱动节点状态：%s')
                      % e)
            raise

    # run_time：角度差越大给越长时间，避免大角度动作被时间截断
    @staticmethod
    def _arm_run_time(jid, target, current):
        span = max(0, current - RobotROS.ARM_LIMITS[jid][0])
        ratio = abs(target - current) / float(max(1, span))
        return int(max(400, min(1500, 400 + ratio * 900)))

    def _send_arm_joint(self, jid):
        """只动一个关节。夹爪动作小，给短 run_time；底座/升降给足时间。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        ang = int(self._arm_sliders[jid].get())
        lo, hi = self.robot.ARM_LIMITS[jid]
        if not (lo <= ang <= hi):
            self._log(L('✗ 关节 %d 目标角度 %.0f° 超出安全范围 %d°~%d°')
                      % (jid, ang, lo, hi))
            return
        cur = float(self._arm_cur.get(jid, ang))
        rt = self._arm_run_time(jid, ang, cur)
        self.cmd('arm', [(jid, ang, rt)], 1.0)
        self._arm_cur[jid] = ang
        self._log(L('→ 关节 %d（%s）目标角度 %.0f°，动作时长 %d ms')
                  % (jid, self.arm_name(jid), ang, rt))

    def _jog_arm(self, jid, direction):
        """点动：滑杆按步长框增减后立即下发，适合 1~5° 微调。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        try:
            step = float(self._step_entry.get())
        except (TypeError, ValueError):
            step = -1.0
        if not 0.5 <= step <= 30:
            self._log(L('✗ 步长无效，请输入 0.5~30 之间的数字'))
            return
        lo, hi = self.robot.ARM_LIMITS[jid]
        tgt = max(lo, min(hi, int(self._arm_sliders[jid].get()) + direction * step))
        self._arm_sliders[jid].set(int(round(tgt)))
        self._send_arm_joint(jid)

    def _send_arm_all(self):
        """三轴同时发一条 Arm 消息，驱动收到后并行执行。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        joints, rts, bad = [], [], []
        for jid in (7, 8, 9):
            ang = int(self._arm_sliders[jid].get())
            lo, hi = self.robot.ARM_LIMITS[jid]
            if not (lo <= ang <= hi):
                bad.append(L('关节 %d：%.0f°（允许 %d°~%d°）')
                           % (jid, ang, lo, hi))
                continue
            rt = self._arm_run_time(jid, ang, float(self._arm_cur.get(jid, ang)))
            joints.append((jid, ang, rt))
            rts.append(rt)
            self._arm_cur[jid] = ang
        if bad:
            self._log(L('✗ 超出安全范围的角度已被拦截：%s') % '；'.join(bad))
        if not joints:
            return
        self.cmd('arm', joints, max(rts) / 1000.0 + 0.8)
        self._log(L('→ 三轴同步目标角度：%s')
                  % '　'.join('%s %.0f°' % (self.arm_name(j), a)
                              for j, a, _ in joints))

    def _run(self, fn):
        def wrap():
            try:
                fn()
            except Exception as e:
                self._log(L('✗ 执行失败：%s') % e)
        threading.Thread(target=wrap, daemon=True).start()

    def _manual_go(self):
        """连续速度控制：交给 robot.manual 做 20Hz 重发 + 超时自停。"""
        if not self.robot.connected():
            self._log(L('⚠ 尚未连接至机器人，本次操作已忽略'))
            return
        lx = self._sl_speed.get()
        az = self._sl_turn.get()
        if abs(lx) < 1e-3 and abs(az) < 1e-3:
            self._log(L('○ 线速度与角速度均为 0，已按紧急制动处理'))
            self.cmd('stop')
            return
        self._log(L('→ 连续控制已启动：线速度 %.2f m/s，角速度 %.2f rad/s'
                    '（5 秒后自动制动）') % (lx, az))
        # 导航在跑时，中继会按优先级仲裁：遥控消息一到，规划器那路自动静音
        # 0.3 s，所以这里不需要停任何进程。
        self._run(lambda: self.robot.manual(lx, az, hold=5.0))

    def _start_worker(self, fn):
        threading.Thread(target=fn, daemon=True).start()

    def _ros_connect(self):
        # 后台线程：只做阻塞等待，绝不碰 Tk（通过 _set_status/_log 转发到主线程）
        try:
            self.robot.wait_for_services(timeout=8)
            # 服务通了才起中继：它要连 master，早于连接起来会反复重连刷日志
            self.robot.relay_up = self.relay.start()
            self._set_status(
                L('● 已连接 ') + STATE['master'].replace('http://', ''), '#7fd88f')
            self._log(L('✓ 已建立连接：ROS Master %s') % STATE['master'])
            # 建图页按钮按连接状态开关：连上后刷新一次，否则“启动”一直是灰的
            self._ui(self._sync_nav)
            # 查相机型号，决定 RGB/深度订阅哪个话题（Astra 有深度，USBCam 没有）
            try:
                dev = self.robot.get_cam_device()
            except Exception as e:
                dev = ''
                self._log(L('○ 相机型号查询失败，不影响运动控制：%s') % e)
            if dev:
                self._ui(lambda d=dev: self._t_cam.set(d))
                self._ui(lambda d=dev: self._cam.set_source(d) or self._t_camsrc.set(
                    L('数据源：%s%s') % (
                        self._cam.rgb_topic,
                        (' + ' + self._cam.depth_topic)
                        if self._cam.depth_topic else L('（无深度流）'))))
                self._log(L('○ 已识别相机型号：%s') % dev)
            # 连接后读取一次机械臂真实角度并回填滑块。缺少该步骤时滑块初值
            # 为固定值，下发指令会导致关节突然移动到非预期角度。
            try:
                self.robot.read_angles()
            except Exception as e:
                self._log(L('○ 机械臂初始角度读取失败，不影响其他功能，'
                            '可稍后手动点击“读取当前角度”：%s') % e)
                return
            try:
                self.cmd('read_arm')
            except Exception as e:
                self._log(L('○ 机械臂初始角度回填失败：%s') % e)
        except Exception as e:
            STATE['ok'] = False
            self._set_status(L('● 连接失败'), '#d9534f')
            self._log(L('✗ 连接失败：%s') % str(e))

    # ---- 线程安全的 UI 更新：子线程 put，主线程 _tick 取 ----
    def _ui(self, fn):
        try:
            self._ui_queue.put(fn)
        except Exception:
            pass

    def _pump_ui_queue(self):
        # 只在主线程（_tick）调用
        try:
            while True:
                fn = self._ui_queue.get_nowait()
                try:
                    fn()
                except Exception:
                    pass
        except queue.Empty:
            pass

    def _set_status(self, text, color):
        self._ui(lambda: self._status.config(text=text, fg=color))

    def _log(self, msg):
        line = time.strftime('[%H:%M:%S] ') + msg + '\n'

        def _do():
            try:
                self._st_txt.insert('end', line)
                self._st_txt.see('end')
                # 防止无限增长
                n = int(self._st_txt.index('end-1c').split('.')[0])
                if n > 2000:
                    self._st_txt.delete('1.0', '500.0')
            except Exception:
                pass
        # 主线程直接执行更及时，子线程进队列等 _tick 消费
        if threading.current_thread() is threading.main_thread():
            _do()
        else:
            self._ui(_do)

    # ---- 日志复制 / 清空 ----
    def _on_log_key(self, e):
        # 只读：允许复制/全选/翻页/方向键，其余按键吞掉
        if e.keysym in ('c', 'C', 'a', 'A', 'Insert') and (e.state & 0x4):
            return None  # 放行 Ctrl+C / Ctrl+A / Ctrl+Insert
        if e.keysym in ('Up', 'Down', 'Left', 'Right', 'Home', 'End',
                        'Prior', 'Next', 'Shift_L', 'Shift_R', 'Control_L', 'Control_R'):
            return None
        return 'break'

    def _popup_menu(self, e):
        try:
            self._menu.tk_popup(e.x_root, e.y_root)
        finally:
            self._menu.grab_release()

    def _copy_text(self, s, tip=None):
        def _do():
            try:
                self.root.clipboard_clear()
                self.root.clipboard_append(s)
                if tip:
                    # 这里已在主线程，直接写日志避免再进队列延迟
                    line = time.strftime('[%H:%M:%S] ') + tip + '\n'
                    try:
                        self._st_txt.insert('end', line)
                        self._st_txt.see('end')
                    except Exception:
                        pass
            except Exception as ex:
                try:
                    self._st_txt.insert(
                        'end', time.strftime('[%H:%M:%S] ')
                        + L('✗ 复制到剪贴板失败：%s\n') % ex)
                except Exception:
                    pass
        if threading.current_thread() is threading.main_thread():
            _do()
        else:
            self._ui(_do)

    def _copy_selected_logs(self):
        try:
            s = self._st_txt.get('sel.first', 'sel.last')
        except Exception:
            s = ''
        if s:
            self._copy_text(s)
        else:
            self._copy_all_logs()
        return 'break'

    def _copy_all_logs(self):
        try:
            s = self._st_txt.get('1.0', 'end-1c')
            if s.strip():
                self._copy_text(
                    s, L('已将全部运行日志（%d 字符）复制到剪贴板') % len(s))
            else:
                self._log(L('运行日志为空，无需复制'))
        except Exception as ex:
            self._log(L('✗ 复制到剪贴板失败：%s') % ex)

    def _clear_logs(self):
        self._st_txt.delete('1.0', 'end')

    def _on_close(self):
        # 先停车再关 ROS：驱动认最后一条 /cmd_vel，GUI 直接退出会把车留在
        # 上一条速度上一直走（之前没发停车，关窗口 = 车继续开）。
        try:
            if self.robot.connected():
                self.robot.stop_manual()
                self._log(L('✓ 程序退出前已发送制动指令'))
        except Exception as e:
            self._log(L('✗ 退出前发送制动指令失败：%s') % e)
        try:
            self._cam.stop()
        except Exception:
            pass
        try:
            self.relay.stop()
        except Exception:
            pass
        # 建图/导航栈是本地子进程，关窗口必须收干净：否则 gmapping 和
        # move_base 会一直留在后台，下次启动时抢 /map 和 map->odom。
        try:
            if self.panel is not None:
                self.panel.shutdown()
                self._log(L('✓ 建图/导航栈已停止'))
        except Exception as e:
            self._log(L('✗ 停止建图/导航栈失败：%s') % e)
        try:
            rospy.signal_shutdown('gui closed')
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass

    # ---- 订阅 odom/voltage（回调在 ROS 线程，只写缓存） ----
    def _spawn_subscribers(self):
        def sub():
            try:
                while not STATE['ok']:
                    if rospy.is_shutdown():
                        return
                    time.sleep(0.3)
                rospy.Subscriber('/odom', nav_msgs.msg.Odometry, self._on_odom)
                rospy.Subscriber('/voltage', Battery, self._on_volt)
                # 轮询节点数。直接问 master，不再 fork `rosnode list`
                # 子进程（每 5 秒一个进程，浪费且会卡）。
                # rospy 没有 Master 类，正确入口是 get_master()。
                while not rospy.is_shutdown():
                    try:
                        code, _msg, state = rospy.get_master().getSystemState()
                        if code == 1:
                            names = set()
                            for group in state:
                                for _topic, nodes in group:
                                    names.update(nodes)
                            with self._cache_lock:
                                self._cache['nodes'] = '%d' % len(names)
                    except Exception:
                        pass
                    time.sleep(5)
            except Exception:
                pass
        threading.Thread(target=sub, daemon=True).start()

    def _on_odom(self, msg):
        try:
            v = msg.twist.twist
            with self._cache_lock:
                self._cache['odom'] = '%.2f m/s · %.2f rad/s' % (v.linear.x, v.angular.z)
        except Exception:
            pass

    def _on_volt(self, msg):
        try:
            with self._cache_lock:
                self._cache['volt'] = '%.1f V' % msg.Voltage
        except Exception:
            pass

    def _tick(self):
        # 主线程定时器：先消费子线程的 UI 队列，再把缓存搬到界面
        # （之前版本每 400ms 写一条日志，会刷屏，已去掉）
        try:
            self._pump_ui_queue()
        except Exception:
            pass
        try:
            with self._cache_lock:
                odom = self._cache.get('odom', '—')
                volt = self._cache.get('volt', '—')
                nodes = self._cache.get('nodes', None)
            if odom != '—':
                self._t_odom.set(odom)
            if volt != '—':
                self._t_volt.set(volt)
            if nodes is not None:
                self._t_nodes.set(nodes)
            self._t_master.set(STATE.get('master', '') or os.environ.get('ROS_MASTER_URI', ''))
        except Exception:
            pass
        finally:
            self._relay_watchdog()
            self.root.after(1000, self._tick)

    def _relay_watchdog(self):
        """Keep exactly one relay alive; restart it if it died."""
        try:
            if not self.robot.connected():
                return
            if self.relay.alive():
                self.robot.relay_up = True
                return
            # 中继死了就立刻改走直连：relay_up 还是 True 的话，遥控会发到
            # /cmd_vel_teleop，而那个话题已经没人听，车就不动了。
            self.robot.relay_up = False
            if time.time() - self.relay.last_start < 5.0:
                return
            if getattr(self, '_relay_restarting', False):
                return
            self._relay_restarting = True

            def work():
                try:
                    self.robot.relay_up = self.relay.start(quiet=True)
                    if self.robot.relay_up:
                        self._log(L('○ 速度中继已重新拉起'))
                    else:
                        self._log(L('✗ 速度中继反复失败，遥控直连 /cmd_vel'))
                        tail = self.relay.last_error()
                        if tail:
                            self._log('   ' + tail)
                finally:
                    self._relay_restarting = False

            threading.Thread(target=work, daemon=True).start()
        except Exception:
            pass


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()