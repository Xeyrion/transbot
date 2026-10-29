#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""wallmark_kin.py -- 墙面画笔运动学（纯计算，不依赖 ROS / gazebo）

用途：把 draw_ready.py 里散落的运动学抽成单一事实来源，任何后端
（真机 /TargetAngle+CurrentAngle / 仿真 /arm_controller/command）都用同一套
FK/IK，这样"仿真验证出来的"和"真机跑的"就是同一个算法。

标定来源（2026-10-29 裁决）：
  docs/05-关节标定.md 的人工实测值是唯一控制口径：
      L1 = 10.0 cm, L2 = 24.0 cm, ARM_X = 12.9 cm
  厂家 URDF 反推的 8.285 / 18.341 只保留作历史对照，
  见 docs/07-文档总览.md 的 C3/C4；控制路径不要用 `urdf` 档。

角度约定与 `../arm/line_tool.py` 完全一致（真机 /CurrentAngle 读数，度）：
  e1  = j7 - 95       第一节世界仰角
  rel = j8 - 180      小臂相对角（j8=180 时两节共线）
  e2  = e1 + rel = j7 + j8 - 275
单位：长度 cm，x = 车头方向，z = 上，原点 = j7 轴心
"""
from __future__ import division, print_function

import math

# ---------------------------------------------------------------- 标定档位
# 'doc'  : 2026-10-09 人工实测值，2026-10-29 用户确认，唯一控制口径。
# 'urdf' : 从厂家 URDF 反推的几何，仅保留作历史对照（LEGACY，不要用于控制）。
PROFILES = {
    'doc': {
        'L1': 10.0,        # cm  j7轴 -> j8轴（钢卷尺实测，±0.5）
        'L2': 24.0,        # cm  j8轴 -> 笔尖（含夹爪+笔，j9~159）
        'ARM_X': 12.9,     # cm  j7轴心 x 相对 base_link（实测）
        'ARM_Z': 0.0,      # cm  j7轴心 z 相对 base_link（base_link 离地 8cm）
    },
    'urdf': {
        'L1': 8.285,       # cm  LEGACY: 厂家 URDF 实算，与真机不符
        'L2': 24.0,        # cm  URDF 夹爪末端 17.714 + 笔 6.268
        'ARM_X': 18.341,   # cm  LEGACY: 厂家 URDF 实算，与真机不符
        'ARM_Z': -0.164,   # cm  LEGACY
    },
}
LEGACY_PROFILES = ('urdf',)
ACTIVE = 'doc'

L1 = PROFILES[ACTIVE]['L1']
L2 = PROFILES[ACTIVE]['L2']
ARM_X = PROFILES[ACTIVE]['ARM_X']
ARM_Z = PROFILES[ACTIVE]['ARM_Z']

BASE_LINK_Z = 8.0                       # cm  base_footprint(地面) -> base_link
ARM_GROUND = BASE_LINK_Z + ARM_Z        # cm  j7 轴心离地（随档位变）

J7_OFF = 95.0       # e1 = j7 - J7_OFF，j7=95 时第一节水平
J7_J8_SUM = 275.0   # e2 = j7 + j8 - 275；共线看 rel=j8-180=0，不是看这个和
J7_LIM = (0.0, 225.0)
J8_LIM = (15.0, 240.0)

# 真机 URDF 关节角 <-> 经验模型角度 的换算（2026-10-21 由 transbot_camera.urdf 反推）
URDF_T1_OFF = 230.0     # t1 = j7 - 230      (e1 = t1 + 135)
URDF_T2_OFF = 139.83    # t2 = j7 + j8 - 139.83  (rel = t2 - 135.17)

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

# 奇异判定阈值：det = L1*L2*sin(e2-e1)，单位 cm^2
SINGULAR_DET = 30.0


def use_profile(name, l1=None, l2=None, arm_x=None, arm_z=None):
    """切换标定档位，返回生效的 dict。

    l1/l2/arm_x/arm_z 可临时覆盖单项（卷尺复测后不用改文件）。
    """
    global ACTIVE, L1, L2, ARM_X, ARM_Z, ARM_GROUND
    if name not in PROFILES:
        raise KeyError('unknown profile %r (have %s)' % (name, sorted(PROFILES)))
    ACTIVE = name
    p = dict(PROFILES[name])
    for key, val in (('L1', l1), ('L2', l2), ('ARM_X', arm_x), ('ARM_Z', arm_z)):
        if val is not None:
            p[key] = val
    L1, L2, ARM_X, ARM_Z = p['L1'], p['L2'], p['ARM_X'], p['ARM_Z']
    ARM_GROUND = BASE_LINK_Z + ARM_Z
    return p


# ---------------------------------------------------------------- 正解
def fk(j7, j8):
    """关节角 -> 笔尖 (x, z)，相对 j7 轴心。x 朝车头，z 朝上。"""
    e1 = (j7 - J7_OFF) * D2R
    e2 = (j7 + j8 - J7_J8_SUM) * D2R   # e2 = e1 + (j8 - 180)
    return (L1 * math.cos(e1) + L2 * math.cos(e2),
            L1 * math.sin(e1) + L2 * math.sin(e2))


def link_angles(j7, j8):
    """返回 (e1, e2, rel)，e1/e2/rel 单位度；rel = j8 - 180。"""
    e1 = j7 - J7_OFF
    rel = j8 - 180.0
    return (e1, e1 + rel, rel)


def det(j7, j8):
    """雅可比行列式 det = L1*L2*sin(e2-e1)，0 = 奇异(伸直或折叠)。"""
    _, _, rel = link_angles(j7, j8)
    return L1 * L2 * math.sin(rel * D2R)


# ---------------------------------------------------------------- 逆解
def ik(x, z):
    """目标笔尖位置 (x, z) -> [(j7, j8, bend), ...]，bend=肘部弯角(度)。

    关节限位内 + 躲奇异（|L2-L1|+1 ~ L1+L2-2）才有解。
    多个解时按 bend 升序返回（bend 小 = 肘更弯）。
    """
    d = math.hypot(x, z)
    d_min = abs(L2 - L1) + 1.0
    d_max = L1 + L2 - 2.0
    if d < d_min or d > d_max:
        return []
    c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    c = max(-1.0, min(1.0, c))
    phi = math.atan2(z, x)
    sols = []
    for sgn in (1.0, -1.0):
        beta = sgn * math.acos(c)
        e1 = phi - math.atan2(L2 * math.sin(beta), L1 + L2 * math.cos(beta))
        j7 = e1 / D2R + J7_OFF
        j8 = beta / D2R + 180.0
        if J7_LIM[0] <= j7 <= J7_LIM[1] and J8_LIM[0] <= j8 <= J8_LIM[1]:
            sols.append((j7, j8, abs(beta / D2R)))
    sols.sort(key=lambda s: s[2])
    return sols


def ik_best(x, z, prefer=None):
    """挑一个解：优先离当前关节位形最近的（prefer=(j7,j8)），否则取 bend 最小的。"""
    sols = ik(x, z)
    if not sols:
        return None
    if prefer is not None:
        sols = sorted(sols, key=lambda s: abs(s[0] - prefer[0]) + abs(s[1] - prefer[1]))
    return sols[0]


# ---------------------------------------------------------------- 步进
def jac_step(j7, j8, dx, dz):
    """保持法向 dx 不变、竖直方向走 dz，返回 (dj7, dj8)（度）。

    双关节协调解走雅可比；接近奇异(|det|<SINGULAR_DET)时退化成 j8 单动。
    """
    e1, e2, _ = link_angles(j7, j8)
    e1r, e2r = e1 * D2R, e2 * D2R
    dt = L1 * L2 * math.sin(e2r - e1r)
    if abs(dt) > SINGULAR_DET:
        a11, a12 = -L1 * math.sin(e1r), -L2 * math.sin(e2r)
        a21, a22 = L1 * math.cos(e1r), L2 * math.cos(e2r)
        de1 = (dx * a22 - a12 * dz) / dt
        de2 = (a11 * dz - dx * a21) / dt
        dj7 = de1 / D2R
        dj8 = (de2 - de1) / D2R
    else:
        dj7 = 0.0
        dj8 = dz / (0.419 * math.cos(e2r)) if abs(math.cos(e2r)) > 1e-6 else 0.0
    return dj7, dj8


def clamp_pose(j7, j8):
    """关节限位裁剪。"""
    return (max(J7_LIM[0], min(J7_LIM[1], j7)),
            max(J8_LIM[0], min(J8_LIM[1], j8)))


def plan_vertical_line(j7, j8, length_cm, direction='down', step_cm=0.25):
    """规划一条竖直线（法向 dx=0）。

    返回 [(j7, j8), ...] 关节序列，含起点。direction: 'down' 笔往下走。
    """
    sgn = -1.0 if direction == 'down' else 1.0
    steps = int(round(length_cm / step_cm))
    out = [(j7, j8)]
    for _ in range(steps):
        dj7, dj8 = jac_step(j7, j8, 0.0, sgn * step_cm)
        j7, j8 = clamp_pose(j7 + dj7, j8 + dj8)
        out.append((j7, j8))
    return out


def plan_to_z(j7, j8, z_target, step_cm=0.25, max_steps=500):
    """从当前姿态走到目标高度 z_target（相对 j7 轴心），法向 dx=0 保持。

    返回 [(j7, j8), ...] 关节序列（含起点、终点）。
    """
    out = [(j7, j8)]
    z0 = fk(j7, j8)[1]
    down = z_target < z0
    for _ in range(max_steps):
        z = fk(j7, j8)[1]
        if down and z <= z_target:
            break
        if (not down) and z >= z_target:
            break
        dj7, dj8 = jac_step(j7, j8, 0.0, -step_cm if down else step_cm)
        j7, j8 = clamp_pose(j7 + dj7, j8 + dj8)
        out.append((j7, j8))
    return out


def reachable_runs(dist, gap=0.5, z_lo=-30.0, z_hi=40.0, dz=0.05):
    """给定"j7 轴心到墙的距离"，返回可落笔的 z 区间（可能多段）。

    返回 (几何段, 限位段)，各为 [(zmin, zmax), ...]；近距离时中间会断开
    （臂要么折回来、要么伸上去），只看 min/max 会给出假的连续带。
    """
    x = dist - gap
    d_min = abs(L2 - L1) + 1.0
    d_max = L1 + L2 - 2.0
    geo = []
    lim = []
    g = None
    cur = None
    z = z_lo
    while z <= z_hi:
        d = math.hypot(x, z)
        if d_min <= d <= d_max:
            if g is None:
                g = [z, z]
            else:
                g[1] = z
        else:
            if g is not None:
                geo.append(tuple(g))
                g = None
        if ik(x, z):
            if cur is None:
                cur = [z, z]
            else:
                cur[1] = z
        else:
            if cur is not None:
                lim.append(tuple(cur))
                cur = None
        z += dz
    if g is not None:
        geo.append(tuple(g))
    if cur is not None:
        lim.append(tuple(cur))
    return geo, lim


def _largest(runs):
    return max(runs, key=lambda r: r[1] - r[0]) if runs else None


def reachable_band(dist, gap=0.5, z_lo=-30.0, z_hi=40.0, dz=0.05):
    """给定"j7 轴心到墙的距离"，返回最长的一段可落笔 z 区间。

    返回 (几何带, 限位带)，各为 (zmin, zmax) 或 None。
    断开的情况用 reachable_runs() 看。
    """
    geo, lim = reachable_runs(dist, gap=gap, z_lo=z_lo, z_hi=z_hi, dz=dz)
    return _largest(geo), _largest(lim)


# ---------------------------------------------------------------- 距离 -> 范围
def drawable_range(dist, gap=0.5, floor_margin=2.5,
                   z_lo=-30.0, z_hi=40.0, dz=0.05):
    """给"j7 轴心->墙"的距离，算出可画的 z 区间。

    纯几何函数：只依赖当前标定档位和距离，不看机器人当前姿态，
    不依赖 ROS / 后端 / GUI。同一个 dist 永远得到同一个结果。

    dist         : j7 轴心 -> 墙面 的水平距离, cm
    gap          : 墙面 -> 笔尖平面 的余量, cm
    floor_margin : 笔尖最低离地高度, cm（笔身/夹爪不蹭地）
    z_lo/z_hi/dz : 扫描区间与步长, cm

    返回 dict:
      band       : 最终可画的最长一段 (zmin, zmax) 或 None
      runs       : 所有可画段 [(zmin, zmax), ...]（地面余量已裁剪）
      geo        : 只看臂展的最长区间
      lim        : 加关节限位后的最长区间（未裁地面）
      floor_z    : 地面余量对应的 z 下限
      floor_hit  : 限位区间是否本来伸到地面以下
      length     : band 长度, cm（无解为 0）
    """
    geo_runs, lim_runs = reachable_runs(dist, gap=gap, z_lo=z_lo,
                                        z_hi=z_hi, dz=dz)
    floor_z = floor_margin - ARM_GROUND
    runs = []
    for a, b in lim_runs:
        if b >= floor_z:
            runs.append((max(a, floor_z), b))
    runs = [r for r in runs if r[1] >= r[0]]
    band = _largest(runs)
    return {
        'dist': dist,
        'band': band,
        'runs': runs,
        'geo': _largest(geo_runs),
        'geo_runs': geo_runs,
        'lim': _largest(lim_runs),
        'lim_runs': lim_runs,
        'floor_z': floor_z,
        'floor_hit': any(a < floor_z for a, _ in lim_runs),
        'length': (band[1] - band[0]) if band else 0.0,
    }


def dist_window(z1, z2, gap=0.5, floor_margin=2.5,
                d_lo=6.0, d_hi=40.0, step=0.1, dz=0.2):
    """反查：想画 z1~z2，j7 轴心到墙的距离可以取哪些值。

    返回 (dmin, dmax) 或 None。扫描法，精度 = step。
    """
    if z1 > z2:
        z1, z2 = z2, z1
    ok = []
    d = d_lo
    while d <= d_hi + 1e-9:
        r = drawable_range(d, gap=gap, floor_margin=floor_margin,
                           z_lo=z1 - 2.0, z_hi=z2 + 2.0, dz=dz)
        if any(a <= z1 + 1e-9 and b >= z2 - 1e-9 for a, b in r['runs']):
            ok.append(d)
        d += step
    if not ok:
        return None
    return (ok[0], ok[-1])


# ---------------------------------------------------------------- 后端映射（LEGACY）
# 以下换算来自已归档的厂家 transbot_camera.urdf，仅用于历史数据对照。
# 当前正确模型是 ../arm（arm_simple）：arm_joint1 = radians(j7-95),
# arm_joint2 = radians(j8-180)，不要再用这里的旧映射去控制真机。
def to_urdf_theta(j7, j8):
    """LEGACY: 脚本关节 (j7,j8) -> 旧 URDF 关节角 (theta1, theta2)，rad。"""
    t1 = (j7 - URDF_T1_OFF) * D2R
    t2 = (j7 + j8 - URDF_T2_OFF) * D2R
    return t1, t2


def from_urdf_theta(t1, t2):
    """LEGACY: 旧 URDF 关节角 (rad) -> 脚本关节 (j7, j8)，度。"""
    j7 = t1 * R2D + URDF_T1_OFF
    j8 = URDF_T2_OFF - j7 + t2 * R2D
    return j7, j8


def tip_in_base_link(j7, j8):
    """笔尖相对 base_link 的 (x, z)，cm。"""
    x, z = fk(j7, j8)
    return (x + ARM_X, z + ARM_Z)


def urdf_theta_limits_ok(j7, j8):
    """该姿态换算到 URDF 角度后是否落在真机限位内。"""
    t1, t2 = to_urdf_theta(j7, j8)
    return (-3.927 <= t1 <= 0.0) and (-0.2617 <= t2 <= 3.926875)
