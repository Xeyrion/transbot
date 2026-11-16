#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Batch offline mapping: give every session under the bags tree a map.

`rebuild_map.py` builds one session and refuses to touch anything that already
has output. This walks the tree, works out which sessions still need a map, and
runs that tool once per session -- so nobody has to remember which split to
point at, or which sessions were already done.

    ./build_maps.sh                    # 扫描 bags/，把还没有地图的都建一遍
    ./build_maps.sh --dry-run          # 只列清单，不动手
    ./build_maps.sh bags/session_x     # 只处理一个会话

Nothing here needs the robot: it is all offline playback on the control PC.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from viewer_core import session_files


def bags_in(folder):
    """一个目录里的源分卷。排除地图轨迹，但保留还在写的 .bag.active ——
    只写了 .active 的会话也要出现在清单里（标成"还在录制"），不能凭空消失。
    """
    out = []
    for name in sorted(os.listdir(folder)):
        path = os.path.join(folder, name)
        if not os.path.isfile(path) or name.endswith('.mapping.bag'):
            continue
        if name.endswith('.bag') or name.endswith('.bag.active'):
            out.append(path)
    return out


def all_bags(root):
    """root 下的源分卷：本层 + 一层子目录（会话文件夹就在这一层）。"""
    found = bags_in(root)
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name)
        if os.path.isdir(path) and not os.path.islink(path) and not name.startswith('.'):
            found += bags_in(path)
    return found


def units(root):
    """每个"建图单元"用它第一个分卷代表，分组规则与 rebuild_map 完全一致。"""
    seen = []
    for path in all_bags(root):
        # .active 只是"正在写的那一卷"，分组时按它将来会变成的名字算
        base = path[:-len('.active')] if path.endswith('.active') else path
        first = session_files(base)[0]
        if first not in seen:
            seen.append(first)
    return seen


# 真产物。`.mapping.log` 不算：一次中途失败的尝试也会留下它，把它当"已有地图"
# 会让那个会话被永久跳过。
ARTIFACTS = ('.map.json', '.mapping.pgm', '.mapping.yaml', '.mapping.bag')


def classify(first):
    """→ (要不要建, 原因)。先判跳过，省得白跑一遍 ROS。"""
    folder = os.path.dirname(first)
    if glob.glob(os.path.join(folder, '*.bag.active')):
        return False, '还在录制（有 .bag.active）'
    for suffix in ARTIFACTS:
        if os.path.exists(first + suffix):
            return False, '已有地图'
    if glob.glob(os.path.join(folder, 'map_*.yaml')):
        return False, '已有地图'          # 录制收尾时存下的栅格
    return True, ''


def clean_partial(first):
    """失败后清掉本次跑出来的半成品。

    rebuild_map 见到 `stem*` 就拒绝动手，不清的话这个会话会被自己的失败卡住，
    下次批量建图永远跳过它。只在确实没产出地图时才清（有 .map.json 就是成功）。
    """
    if os.path.exists(first + '.map.json'):
        return []
    removed = []
    for suffix in ('.mapping.log', '.mapping.bag', '.mapping.pgm', '.mapping.yaml'):
        path = first + suffix
        if os.path.exists(path):
            os.remove(path)
            removed.append(os.path.basename(path))
    return removed


def run_one(first, rate, port):
    """调 rebuild_map.py 出一个会话的地图。→ ('ok'|'skip'|'fail', 说明)。

    rebuild_map 用退出码表态：0 = 有产出（重建，或把包里已有的 /map 导出来），
    3 = 数据本身不需要/不可能，其余 = 真失败。不解析输出里的字串 —— 回溯会把
    检查脚本源码原样打出来，按字串匹配会误判。
    """
    cmd = [sys.executable, os.path.join(HERE, 'rebuild_map.py'), first,
           '--rate', str(rate), '--port', str(port)]
    started = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            universal_newlines=True)
    lines = []
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            lines.append(line)
    proc.wait()
    spent = time.time() - started
    if proc.returncode == 0:
        done = [l for l in lines if l.startswith(('地图已从包里导出', '地图和轨迹已关联'))]
        return 'ok', done[-1] if done else '用时 %.0f 秒' % spent
    if proc.returncode == 3:
        return 'skip', lines[-1] if lines else '跳过'
    tail = '退出码 %d' % proc.returncode
    for line in reversed(lines):
        if line.startswith(('RuntimeError', 'ValueError', 'subprocess.')):
            tail = line
            break
    leftovers = clean_partial(first)
    note = tail[:160]
    if leftovers:
        note += '（已清掉半成品：%s）' % ', '.join(leftovers)
    return 'fail', note


def main():
    ap = argparse.ArgumentParser(description='批量离线建图')
    ap.add_argument('root', nargs='?', default=os.path.join(HERE, 'bags'),
                    help='扫描起点（默认 bagtools/bags）')
    ap.add_argument('--dry-run', action='store_true', help='只列清单')
    ap.add_argument('--rate', type=float, default=2., help='回放倍速（默认 2）')
    ap.add_argument('--port', type=int, default=11329, help='私有 ROS master 端口')
    args = ap.parse_args()

    root = os.path.abspath(os.path.expanduser(args.root))
    if os.path.isfile(root):
        # 顺手支持直接点一个包（从 rebuild_map.sh 过来的习惯），只建这一个会话
        base = root[:-len('.active')] if root.endswith('.active') else root
        todo_units, base_dir = [session_files(base)[0]], os.path.dirname(root)
    elif os.path.isdir(root):
        todo_units, base_dir = units(root), root
    else:
        sys.exit('目录不存在：' + root)

    todo, skip = [], []
    for first in todo_units:
        need, why = classify(first)
        (todo if need else skip).append((first, why))

    def shown(path):
        return os.path.relpath(path, base_dir)

    print('扫描 %s' % root)
    for first, why in skip:
        print('  跳过  %s  —— %s' % (shown(first), why))
    for first, _ in todo:
        print('  建图  %s' % shown(first))
    if not todo:
        print('\n没有需要建图的会话。')
        return 0
    print('\n共 %d 个待建图，%d 个跳过。' % (len(todo), len(skip)))
    if todo:
        print('（每个会话动手前先确认有没有 /scan、有没有 map->odom，不满足的会跳过并写明原因）')
    if args.dry_run:
        print('（--dry-run：只列清单，没有动手）')
        return 0

    ok, failed = [], []
    for i, (first, _) in enumerate(todo, 1):
        print('\n[%d/%d] %s' % (i, len(todo), shown(first)), flush=True)
        status, note = run_one(first, args.rate, args.port)
        label = {'ok': '完成', 'skip': '跳过', 'fail': '未完成'}[status]
        print('      %s —— %s' % (label, note), flush=True)
        if status == 'ok':
            ok.append((first, note))
        elif status == 'skip':
            skip.append((first, note))
        else:
            failed.append((first, note))

    print('\n==== 汇总 ====')
    print('建好 %d 个，跳过 %d 个，未完成 %d 个。' % (len(ok), len(skip), len(failed)))
    for first, note in failed:
        print('  未完成 %s —— %s' % (shown(first), note))
    if ok:
        print('地图在各自会话目录里：<分卷>.mapping.pgm/.yaml，打开阅读器即可看到。')
    return 0 if not failed else 1


if __name__ == '__main__':
    raise SystemExit(main())
