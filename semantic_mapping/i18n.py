# -*- coding: utf-8 -*-
"""Shared UI catalogue. Language is selected at launch and inherited by tools."""
from __future__ import unicode_literals
import io
import json
import os
import re

_catalog = None


def language():
    return 'en' if os.environ.get('BAGTOOLS_LANG','zh').lower().startswith('en') else 'zh'


def english_catalog():
    global _catalog
    if _catalog is None:
        with io.open(os.path.join(os.path.dirname(__file__),'locale','en.json'),encoding='utf-8') as f:
            _catalog=json.load(f)
    return _catalog


def tr(text):
    if language()!='en':return text
    key=text.decode('utf-8') if isinstance(text,bytes) else text
    return english_catalog().get(key,text)


# ---------------------------------------------------------------------------
# 脚本输出（remote_record.sh / record_full.sh / auto_record_cam.sh / *.py）的翻译
#
# 为什么不在脚本里做 i18n：机器人在 ssh 那头跑，环境变量不会跟着过去，而且同一份
# 脚本要给中英文两个入口用。所以脚本一律说中文，在控制机的显示层（GUI 日志、
# 状态面板）统一翻译 —— 只有这一处需要维护。
#
# 三层匹配，从具体到宽泛：
#   1. 整行精确命中词表        —— 固定文案，绝大多数
#   2. 去掉缩进后命中词表      —— 保留原来的缩进层级
#   3. 正则规则                —— 带变量（路径、PID、容量）的行
# 三层都不命中就原样输出，宁可留中文也不吞掉诊断信息。
# ---------------------------------------------------------------------------
_RULES = [
    # ---- 固定前缀 / 带变量的状态行 ----
    (r'^== 启动机器人录制：(.*) ==$', r'== Start robot recording: \1 =='),
    (r'^已在后台启动，PID=(.*)$', r'Session launched in background; PID=\1'),
    (r'^用 .*status.*看进度.*$', r"Use './remote_record.sh status' for progress, 'log' to follow the live log."),
    (r'^取消会话 (.*)；收尾录制进程 (.*)$', r'Cancel sessions \1; finalize recorders \2'),
    (r'^=== 会话收尾：完成 (.*) 卷，未关闭 (.*) 卷 ===$', r'=== Finalized: \1 parts; unfinished: \2 ==='),
    (r'^本次会话目录：(.*)$', r'Session directory: \1'),

    # 摘要块：带单位的行要排在对应的通用规则前面
    (r'^保存目录\s*:\s*(.*)（剩余 (.*) GB）$', r'Output directory: \1 (\2 GB free)'),
    (r'^保存目录\s*:\s*(.*)$', r'Output directory: \1'),
    (r'^模式\s*:\s*(.*)$', r'Profile: \1'),
    (r'^时长\s*:\s*一直录，Ctrl-C 停$', r'Duration: until Ctrl-C'),
    (r'^时长\s*:\s*(.*)$', r'Duration: \1'),
    (r'^压缩\s*:\s*lz4，每\s*(.*?)\s*MB 分卷$', r'Compression: lz4, \1 MB per split'),
    (r'^保留卷数\s*:\s*最新 (.*) 卷（自动删旧）$', r'Keep splits: newest \1, older ones pruned'),
    (r'^话题\s*:\s*(.*)$', r'Topics: \1'),

    (r'^地图已保存：(.*)$', r'Map saved: \1'),
    (r'^检查 ROS master：(.*)$', r'Checking ROS master: \1'),
    (r'^等待话题（最多 (.*)s）\.\.\.$', r'Waiting for topics (up to \1 s)...'),
    (r'^(.*) 有数据$', r'\1 is streaming'),

    # ---- 缺数据 / 等不到话题 ----
    (r'^WARN 等不到 (.*)，--allow-missing 继续（这个包缺这一路）$',
     r'WARN gave up waiting for \1; --allow-missing, this bag lacks it'),
    (r'^WARN 这些话题 6 秒内没有收到任何消息：(.*)$',
     r'WARN no messages within 6 s on: \1'),
    (r'^ERROR: 这些话题 6 秒内没有收到任何消息：(.*)$',
     r'ERROR: no messages within 6 s on: \1'),
    (r'^ERROR: 等不到 (.*)$', r'ERROR: gave up waiting for \1'),
    (r'^ERROR: 60 秒内等不到 (.*)，看日志：$', r'ERROR: \1 did not appear within 60 s; see the log:'),
    (r'^ERROR: 60 秒内没等到图像，看日志：(.*)$', r'ERROR: no images within 60 s; see the log: \1'),
    (r'^ERROR: (.*) 只剩 (.*) GB，至少需要 (.*) GB$',
     r'ERROR: \1 has only \2 GB free, \3 GB required'),
    (r'^换目录：(.*)$', r'Other directory: \1'),
    (r'^或清理：rm -f (.*)（确认不要了再删）$',
     r'Or clean up: rm -f \1 (only once you are sure you do not need them)'),
    (r'^提示：当前是 (.*)（不像 Jetson）。full 模式含相机原图，$',
     r'Note: this is \1 (not a Jetson). The full profile includes raw camera images;'),

    # ---- 锁 / 会话冲突 ----
    (r'^ERROR: 已经有一个录制会话在跑（PID (.*)），先停掉再开：$',
     r'ERROR: a recording session is already running (PID \1); stop it first:'),
    (r'^ERROR: 找不到 (.*)$', r'ERROR: cannot find \1'),
    (r'^ERROR: 建不了会话目录：(.*)$', r'ERROR: cannot create the session folder: \1'),
    (r'^ERROR: 同一秒里的同名会话太多：(.*)$',
     r'ERROR: too many sessions with that name in one second: \1'),
    (r'^已经有一个录制会话在跑（PID (.*)），先 stop 或等它结束：$',
     r'A recording session is already running (PID \1); stop it or wait for it to finish:'),
    (r'^（清理过期的锁：PID (.*) 已经不在了）$', r'(cleared a stale lock: PID \1 is gone)'),

    # ---- status / 部署 ----
    (r'^时间：(.*)$', r'Time: \1'),
    (r'^状态：(.*)$', r'State: \1'),
    (r'^目录：(.*)；可用 (.*) GiB$', r'Directory: \1; \2 GiB free'),
    (r'^会话数：(.*)；本次分卷合计 (.*) MiB$',
     r'Sessions: \1; \2 MiB total across this session'),
    (r'^== 机器人：(.*)   目标：(.*) ==$', r'== Robot: \1   target: \2 =='),
    (r'^未知命令：(.*)$', r'Unknown command: \1'),
    (r'^未知参数：(.*)$', r'Unknown argument: \1'),
    (r'^有 (.*) 项不合格。TF 链路问题（(.*)）$',
     r'\1 checks failed. TF chain problems (\2)'),
]

_CJK = re.compile(u'[\u4e00-\u9fff]')


def _translate_line(line):
    catalog = english_catalog()
    hit = catalog.get(line)
    if hit is not None:
        return hit
    body = line.strip()
    if not body:
        return line
    indent = line[:len(line) - len(body)]
    hit = catalog.get(body)
    if hit is not None:
        return indent + hit
    # 规则一律对着去掉缩进的内容匹配：脚本里的提示行缩进是排版，不该影响翻译。
    for pattern, replacement in _RULES:
        if re.match(pattern, body):
            return indent + re.sub(pattern, replacement, body)
    return line


def translate_log(text):
    """Translate known recorder status lines; preserve paths and raw diagnostics."""
    if language() != 'en' or not text:
        return text
    return '\n'.join(_translate_line(line) for line in text.splitlines())


def untranslated(text):
    """Return the lines that still contain Chinese after translation.

    Used by the test suite so a new script message cannot silently ship as
    Chinese-only in the English demo.
    """
    return [line for line in translate_log(text).splitlines() if _CJK.search(line)]
