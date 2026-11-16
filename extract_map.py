#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""把一个会话包里已有的 /map 导出成 map_server 栅格（.pgm + .yaml）。

录制时 SLAM 在跑的包，地图本来就在 /map 话题里 —— 重新跑一遍 gmapping 不但
多余，还会和包里已有的 map->odom 打架。这里只把最后一张栅格抄出来，落到
`map_<时间戳>.pgm/.yaml`（和 record_full.sh 收尾时存的文件同名规则）。

    python2 extract_map.py <任意一个分卷> [更多分卷...]

输出写在第一个分卷旁边。已经有同名文件就拒绝，不覆盖。
"""
import math
import os
import re
import sys
import time

import rosbag


def session_stamp(path):
    """从分卷名里取录制时刻（cam_20261112_011503_0.bag → 20261112_011503）。

    不用 /map 消息的 header 时间：Jetson 没有 RTC 电池，那串时间可能差好几个
    小时，两个会话会撞成同一个文件名。包名是录制那一刻的目录名，更可靠。
    """
    match = re.search(r'_(\d{8}_\d{6})(?:_\d+)?\.bag$', os.path.basename(path))
    return match.group(1) if match else None


def write_pgm(path, msg):
    """占用栅格 → PGM。未知格写 205，和 map_server/map_saver 的约定一致。"""
    width, height = msg.info.width, msg.info.height
    data = msg.data
    pixels = bytearray()
    if len(data) != width * height:
        raise ValueError('OccupancyGrid dimensions do not match data')
    for y in range(height - 1, -1, -1):
        for value in data[y * width:(y + 1) * width]:
            pixels.append(205 if value < 0 else int(round((100 - value) * 254 / 100.0)))
    with open(path, 'wb') as f:
        f.write('P5\n%d %d\n255\n' % (width, height))
        f.write(bytes(pixels))


def main():
    if len(sys.argv) < 2:
        sys.exit('用法：extract_map.py <分卷> [更多分卷...]')
    paths = sys.argv[1:]
    last = None
    for path in paths:
        bag = rosbag.Bag(path)
        try:
            for _topic, msg, _t in bag.read_messages(topics=['/map']):
                last = msg
        finally:
            bag.close()
    if last is None:
        sys.exit('NO_MAP: 包里没有 /map')
    stamp = session_stamp(paths[0])
    if stamp is None:
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(last.header.stamp.to_sec()))
    stem = os.path.join(os.path.dirname(os.path.abspath(paths[0])), 'map_' + stamp)
    if os.path.exists(stem + '.yaml') or os.path.exists(stem + '.pgm'):
        sys.exit('已有同名地图，不覆盖：' + stem)
    write_pgm(stem + '.pgm', last)
    info = last.info
    q = info.origin.orientation
    # map_server 的 origin 第三项是 yaw 角，不是四元数的 z 分量
    yaw = 2 * math.atan2(q.z, q.w)
    with open(stem + '.yaml', 'w') as f:
        f.write('image: %s\n' % os.path.basename(stem + '.pgm'))
        f.write('resolution: %g\n' % info.resolution)
        f.write('origin: [%g, %g, %g]\n' % (info.origin.position.x,
                                            info.origin.position.y, yaw))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    print('地图已从包里导出：' + stem + '.pgm + .yaml')


if __name__ == '__main__':
    main()
