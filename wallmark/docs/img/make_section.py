#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""section.py -- 侧视图: 为什么 2R 臂只能在墙上画竖线 (PIL 画, 不依赖 matplotlib 中文)"""
import math
import os
from PIL import Image, ImageDraw, ImageFont

# ---------------- 几何 (cm) ----------------
L1, L2 = 10.0, 24.0
D_OUT = L1 + L2 - 2.0        # 32.0
D_IN = L2 - L1 + 1.0         # 15.0
DIST = 27.17                 # j7 轴 -> 墙
GAP = 0.5
X_PEN = DIST - GAP           # 26.67
FLOOR = -8.0
Z_LO, Z_HI = -15.15, 15.15
Z_FLOOR = FLOOR + 2.5        # -5.5

# ---------------- 画布 / 坐标 ----------------
S = 12.0                     # px per cm
XL, XR = -34.0, 41.0
ZT, ZB = 33.0, -11.5
PW = int((XR - XL) * S)      # 900
PH = int((ZT - ZB) * S)      # 534
ML, MT = 76, 104             # 绘图区左上角
W, H = PW + ML + 40, PH + MT + 78

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
def F(sz, idx=2):
    return ImageFont.truetype(FONT, sz, index=idx)

f_title = F(23)
f_big   = F(19)
f_txt   = F(16)
f_sm    = F(14)

def P(x, z):
    """数据 -> 绘图区内的像素 (相对 plot 图)"""
    return ((x - XL) * S, (ZT - z) * S)

img = Image.new('RGB', (W, H), 'white')
d = ImageDraw.Draw(img)

def T(x, z, s, font=f_txt, fill='black', anchor='la'):
    px, py = P(x, z)
    d.text((px + ML, py + MT), s, font=font, fill=fill, anchor=anchor)

def ring(dr, cx, cy, r, color, width=2, dash=7):
    n = int(2 * math.pi * r)
    i = 0
    while i < n:
        a0 = i / r
        a1 = min(a0 + dash / r, 2 * math.pi)
        dr.arc([cx - r, cy - r, cx + r, cy + r],
               start=math.degrees(a0), end=math.degrees(a1),
               fill=color, width=width)
        i += 2 * dash

# ---------------- 绘图区 (独立小图, 自动裁掉出界部分) ----------------
plot = Image.new('RGB', (PW, PH), 'white')
g = ImageDraw.Draw(plot)
cx, cy = P(0, 0)
ro, ri = D_OUT * S, D_IN * S

g.ellipse([cx - ro, cy - ro, cx + ro, cy + ro], fill='#eaf1f8')     # 圆环外
g.ellipse([cx - ri, cy - ri, cx + ri, cy + ri], fill='white')       # 挖掉内圆
ring(g, cx, cy, ro, '#7fa3c4')
ring(g, cx, cy, ri, '#7fa3c4')

# 墙: x >= DIST 的整块灰色
g.rectangle([P(DIST, ZT)[0], 0, PW, PH], fill='#d8d8d8')
g.line([P(DIST, ZT)[0], 0, P(DIST, ZB)[0], PH], fill='#222222', width=3)

# 地面 + 斜线
gy = P(0, FLOOR)[1]
g.line([0, gy, PW, gy], fill='#8a6d3b', width=3)
xx = -PH
while xx < PW:
    g.line([xx, gy, xx + 40, PH], fill='#c9b892', width=1)
    xx += 12

# 笔尖平面
gpx = P(X_PEN, 0)[0]
yy = 0
while yy < PH:
    g.line([gpx, yy, gpx, min(yy + 6, PH)], fill='#2e9e5b', width=2)
    yy += 12

# 可画带 / 红线
g.line([gpx, P(0, Z_FLOOR)[1], gpx, P(0, min(Z_HI, 28))[1]], fill='#3ddc84', width=10)
g.line([gpx, P(0, 9.64)[1], gpx, P(0, 6.64)[1]], fill='#e03030', width=6)

# 臂 (起点 z=8)
e1, e2 = math.radians(74.60), math.radians(0.0)
ex, ez = L1 * math.cos(e1), L1 * math.sin(e1)
tx, tz = ex + L2 * math.cos(e2), ez + L2 * math.sin(e2)
g.line([P(0, 0), P(ex, ez)], fill='#3a3a3a', width=7)
g.line([P(ex, ez), P(tx, tz)], fill='#606060', width=7)
for pt, r, c in (((0, 0), 7, '#e8c34a'), ((ex, ez), 5, '#3a3a3a'), ((tx, tz), 6, '#e03030')):
    px, py = P(*pt)
    g.ellipse([px - r, py - r, px + r, py + r], fill=c, outline='white')

img.paste(plot, (ML, MT))
d = ImageDraw.Draw(img)

# ---------------- 文字 ----------------
d.text((ML, 22), '为什么只能画“墙上的竖线” —— 侧视图', font=f_title, fill='#111111')
d.text((ML, 56), 'j7/j8 都是俯仰：笔尖被锁在 x-z 平面内，y 方向没有任何自由度',
       font=f_txt, fill='#555555')

T(-27.5, 21.0, '2R 可达圆环', font=f_big, fill='#4d7aa8')
T(-27.5, 18.2, '内径 %.1f  外径 %.1f' % (D_IN, D_OUT), font=f_sm, fill='#4d7aa8')

T(DIST + 1.2, 30.5, '墙 = 竖直面', font=f_big, fill='#222222')
T(DIST + 1.2, 27.8, 'x = %.1f cm' % DIST, font=f_sm, fill='#222222')

T(X_PEN - 1.2, 23.5, '笔尖平面 x=%.1f (离墙5mm)' % X_PEN, font=f_sm, fill='#1f8f4e', anchor='ra')
T(25.2, 13.0, '可画\nz %.1f ~ %.1f' % (Z_FLOOR, Z_HI), font=f_txt,
  fill='#1f8f4e', anchor='ra')

T(-33.0, -5.9, '地面 z=%.1f  (臂轴心离地 %.1f)' % (FLOOR, -FLOOR), font=f_sm, fill='#8a6d3b')

T(-2.2, -2.6, '臂座 (0,0)', font=f_sm, fill='#a07a10', anchor='ra')
T(ex - 1.2, ez + 1.8, '肘', font=f_sm, fill='#333333', anchor='ra')

# 红线的引线 + 说明
T(20.5, -1.2, '画出来的线：只有 z 变，x 不变\n所以永远是竖直的', font=f_sm, fill='#c02020', anchor='ra')
ax, ay = P(21.2, 0.2)
bx, by = P(X_PEN - 0.5, 6.6)
d.line([(ax + ML, ay + MT), (bx + ML, by + MT)], fill='#c02020', width=1)

# 轴标
d.text((ML + PW // 2, MT + PH + 34), 'x  (cm, 朝墙)', font=f_txt, fill='#444444', anchor='ma')
d.text((34, MT + PH // 2), 'z 向上', font=f_sm, fill='#444444', anchor='mm')

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   'section_why_vertical.png')
img.save(out)
print('saved', out, img.size)
