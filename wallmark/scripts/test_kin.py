#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""test_kin.py -- wallmark 运动学离线自检（不连 ROS / gazebo，秒级跑完）

用法: python test_kin.py
退出码 0 = 全部通过；非 0 = 有 FAIL
"""
from __future__ import division, print_function

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import wallmark_kin as K

FAILS = []
CHECKS = [0]
WARNS = []


def ck(name, cond, detail=''):
    CHECKS[0] += 1
    if cond:
        print('  PASS  %-46s %s' % (name, detail))
    else:
        print('  FAIL  %-46s %s' % (name, detail))
        FAILS.append(name)


def warn(name, detail=''):
    WARNS.append(name)
    print('  WARN  %-46s %s' % (name, detail))


def close(a, b, tol):
    return abs(a - b) <= tol


def section(t):
    print('\n== %s ==' % t)


# ---------------------------------------------------------------- 1. FK/IK 往返
section('1. FK <-> IK 往返（可达工作空间全域）')
worst = 0.0
n_solved = 0
n_total = 0
for xi in range(-30, 33, 3):
    for zi in range(-24, 25, 3):
        n_total += 1
        sols = K.ik(xi, zi)
        if not sols:
            continue
        n_solved += 1
        for (j7, j8, bend) in sols:
            x, z = K.fk(j7, j8)
            worst = max(worst, math.hypot(x - xi, z - zi))
ck('IK->FK 往返最大误差 < 0.01cm', worst < 0.01,
   'worst=%.2e cm, 目标点 %d/%d 可解' % (worst, n_solved, n_total))

# ---------------------------------------------------------------- 2. doc 05 表
section('2. docs/05-关节标定.md 姿态读数复算（正确角口径）')
# 2026-10-29 勘误：旧表把第一节仰角 e1 当成了第二节仰角 e2。
# 正确关系 rel=j8-180, e2=e1+rel。下面四个姿态的真实 e2 是 0/0/0/1°。
for (j7, j8, e1_exp, rel_exp, e2_exp) in [
        (115, 160, 20, -20, 0),
        (127, 148, 32, -32, 0),
        (141, 134, 46, -46, 0),
        (163, 113, 68, -67, 1)]:
    e1, e2, rel = K.link_angles(j7, j8)
    ck('(%d,%d) e1=%d' % (j7, j8, e1_exp), close(e1, e1_exp, 0.5),
       'e1=%.1f rel=%.1f e2=%.1f' % (e1, rel, e2))
    ck('(%d,%d) rel=%d' % (j7, j8, rel_exp), close(rel, rel_exp, 0.5),
       'rel=%.1f' % rel)
    ck('(%d,%d) e2=%d' % (j7, j8, e2_exp), close(e2, e2_exp, 0.5),
       'e2=%.1f' % e2)
    # 竖直灵敏度 dz/dj8 = L2*cos(e2)*pi/180；e2 才是决定竖直分量的角。
    dz_dj8 = K.L2 * math.cos(e2 * K.D2R) * K.D2R
    need = 1.0 / dz_dj8
    ck('(%d,%d) 走1cm需 |dj8|=%.2f°' % (j7, j8, need), close(need, 2.39, 0.02),
       '算得 %.2f°' % need)

# ---------------------------------------------------------------- 3. 符号
section('3. 方向符号（旧表符号写反了，这里锁定正确行为）')
j7, j8 = 127, 148
z0 = K.fk(j7, j8)[1]
z_up = K.fk(j7, j8 + 1.0)[1]
z_dn = K.fk(j7, j8 - 1.0)[1]
ck('j8 增大 -> 笔尖升高', z_up > z0, 'z: %.3f -> %.3f (+%.3fcm/°)' % (z0, z_up, z_up - z0))
ck('j8 减小 -> 笔尖降低', z_dn < z0, 'z: %.3f -> %.3f' % (z0, z_dn))
ck('上1cm应加 j8（不是减）', close(1.0 / (z_up - z0), 2.39, 0.02),
   '+1°j8 => %+.3f cm, 上1cm=+%.2f°' % (z_up - z0, 1.0 / (z_up - z0)))

# ---------------------------------------------------------------- 4. doc 06 结论
section('4. docs/06-首次试算.md 结论复算')
sols27 = K.ik(27.0 - K.ARM_X - 0.5, 8.0)
ck('dist=27cm 无解（折叠奇异+超限位）', len(sols27) == 0, '解数=%d' % len(sols27))

best40 = K.ik_best(40.0 - K.ARM_X - 0.5, 8.0)
ck('dist=40cm, z=8 -> j7~169.6 j8~101.5',
   best40 is not None and close(best40[0], 169.6, 1.5) and close(best40[1], 101.5, 2.0),
   '算得 (%.1f, %.1f) bend=%.0f°' % (best40[0], best40[1], best40[2]) if best40 else 'None')

best42 = K.ik_best(42.0 - K.ARM_X - 0.5, 8.0)
ck('dist=42cm, z=8 -> j7~157.5 j8~115.4',
   best42 is not None and close(best42[0], 157.5, 1.5) and close(best42[1], 115.4, 2.0),
   '算得 (%.1f, %.1f) bend=%.0f°' % (best42[0], best42[1], best42[2]) if best42 else 'None')

if best40 and best42:
    ck('两解肘弯都在 60~82°', 60 <= best40[2] <= 82 and 60 <= best42[2] <= 82,
       'bend=%.1f / %.1f' % (best40[2], best42[2]))

win = [d for d in [x * 0.5 for x in range(60, 100)] for z in (8.0, 12.0)
       if K.ik(d - K.ARM_X - 0.5, z)]
ck('可行窗口存在', len(win) > 0,
   'base->board 可行 %.1f ~ %.1f cm' % (min(win), max(win)) if win else '无')

# ---------------------------------------------------------------- 5. 竖直线规划
section('5. 雅可比竖直线规划（dx=0 保持，dz 精确）')
for direction in ('down', 'up'):
    plan = K.plan_vertical_line(best40[0], best40[1], 3.0, direction=direction)
    pts = [K.fk(a, b) for (a, b) in plan]
    x0, z0 = pts[0]
    max_dx = max(abs(p[0] - x0) for p in pts)
    ck('%s: 法向漂移 dx < 0.05cm' % direction, max_dx < 0.05,
       'max|dx|=%.4f cm, 线长=%.3f cm, 步数=%d' % (max_dx, abs(pts[-1][1] - z0), len(plan) - 1))
    ck('%s: 竖直方向单调' % direction,
       all((pts[i + 1][1] - pts[i][1]) * (1 if direction == 'up' else -1) > 0
           for i in range(len(pts) - 1)), '')

# ---------------------------------------------------------------- 6. 奇异
section('6. 奇异判定与退化分支')
ck('rel=0 (共线伸直) det=0', abs(K.det(115.0, 180.0)) < 1e-6, 'det=%.4f' % K.det(115.0, 180.0))
ck('rel=-180 (完全折叠) det=0', abs(K.det(115.0, 0.0)) < 1e-6, 'det=%.4f' % K.det(115.0, 0.0))
dj7, dj8 = K.jac_step(115.0, 180.0, 0.0, -0.25)
ck('奇异时退化为 j8 单动 (dj7=0)', abs(dj7) < 1e-9, 'dj7=%.6f dj8=%.3f' % (dj7, dj8))
z_chk = K.fk(115.0, 180.0 + dj8)[1] - K.fk(115.0, 180.0)[1]
ck('退化分支确实走 dz=-0.25cm', close(z_chk, -0.25, 0.01), '实际 %.4f cm' % z_chk)

# ---------------------------------------------------------------- 7. 限位
section('7. 关节限位')
ck('裁剪到 j7 范围', K.clamp_pose(999, 100) == (225.0, 100.0), str(K.clamp_pose(999, 100)))
ck('裁剪到 j8 范围', K.clamp_pose(50, -50) == (50.0, 15.0), str(K.clamp_pose(50, -50)))
ck('IK 不会给出超限位解',
   all(K.J7_LIM[0] <= s[0] <= K.J7_LIM[1] and K.J8_LIM[0] <= s[1] <= K.J8_LIM[1]
       for d in [x * 0.5 for x in range(60, 100)] for z in (4.0, 8.0, 12.0)
       for s in K.ik(d - K.ARM_X - 0.5, z)), 'ok')

# ---------------------------------------------------------------- 8. URDF 映射
section('8. LEGACY: 脚本关节 <-> 旧厂家 URDF 关节映射（仅历史对照）')
for (j7, j8) in [(95.0, 180.0), (115.0, 160.0), (127.0, 148.0), (141.0, 134.0),
                 (163.0, 113.0), (169.0, 27.0), (157.5, 52.9), (225.0, 15.0)]:
    t1, t2 = K.to_urdf_theta(j7, j8)
    b7, b8 = K.from_urdf_theta(t1, t2)
    ck('往返 (%.1f,%.1f)' % (j7, j8), close(b7, j7, 1e-6) and close(b8, j8, 1e-6),
       'theta=(%.3f, %.3f) rad' % (t1, t2))
    ck('  theta 落在真机 URDF 限位内', K.urdf_theta_limits_ok(j7, j8),
       't1=%.3f t2=%.3f' % (t1, t2))

# ---------------------------------------------------------------- 9. 标定口径
section('9. 标定口径（实测唯一权威，URDF 仅历史对照）')
K.use_profile('doc')
ck('默认档位是实测 doc', K.ACTIVE == 'doc', 'ACTIVE=%s' % K.ACTIVE)
pose = (169.6, 101.5)
xd, zd = K.fk(*pose)
K.use_profile('urdf')
xu, zu = K.fk(*pose)
K.use_profile('doc')
ck('URDF 档与实测档有已知差异', math.hypot(xu - xd, zu - zd) > 1.0,
   '同姿态 (%.1f,%.1f): doc=(%.2f,%.2f) urdf=(%.2f,%.2f) 差 %.2fcm'
   % (pose[0], pose[1], xd, zd, xu, zu, math.hypot(xu - xd, zu - zd)))
print('  INFO  URDF 档 (L1=8.285, ARM_X=18.341) 已冻结为 LEGACY，'
      '不参与控制/IK')

# ---------------------------------------------------------------- 10. base_link 换算
section('10. base_link 坐标换算')
x, z = K.fk(169.6, 101.5)
bx, bz = K.tip_in_base_link(169.6, 101.5)
ck('tip_in_base_link = fk + ARM 偏移', close(bx, x + K.ARM_X, 1e-9) and close(bz, z + K.ARM_Z, 1e-9),
   'base=(%.2f, %.2f) cm' % (bx, bz))

print('\n%s' % ('=' * 68))
print('共 %d 项检查, FAIL %d 项' % (CHECKS[0], len(FAILS)))
if FAILS:
    for f in FAILS:
        print('  FAIL  %s' % f)
    sys.exit(1)
print('ALL PASS')
