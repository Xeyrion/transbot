"""Lightweight directory browsing; never open bags while listing sessions."""
import os
import re
from i18n import tr

# 会话目录名的形状：session_20261111_014312_走廊建图
# 时间戳永远在前 —— 会话列表是按名字倒序排的，前缀一变时间顺序就乱了。
STAMP_PREFIX = re.compile(r'^((?:session|validation)_\d{8}_\d{6})')
SESSION_PREFIXES = ('session_', 'validation_')

# 目录名里允许出现的东西：字母数字、汉字、点、下划线、连字符。
# 其余（空格、斜杠、引号、冒号……）一律换成下划线 —— 这些字符要么会把
# shell/rsync 的参数拆开，要么在 Windows 回传落地时是非法字符。
_LABEL_BAD = re.compile(u'[^0-9A-Za-z._\\-\u4e00-\u9fff]+')


def session_label(text, limit=40):
    """用户敲的会话名 → 一段能安全当目录名用的标签（可能为空）。"""
    text = _LABEL_BAD.sub('_', (text or '').strip())
    return text.strip('._-')[:limit].strip('._-')


def session_dir_name(label, stamp):
    """新建会话的目录名。标签为空就只留时间戳，不再塞随机串。"""
    base = 'session_%s' % stamp
    label = session_label(label)
    return '%s_%s' % (base, label) if label else base


def rename_target(current, text):
    """改名对话框的输入 → 新目录名。

    只写标签时接在原来的时间戳后面（保住排序和来源）；直接写
    `session_...` 这种完整名字则原样采用，留一个完全自己掌控的出口。
    """
    label = session_label(text, limit=60)
    if not label:
        return None
    if label.startswith(SESSION_PREFIXES):
        return label
    match = STAMP_PREFIX.match(current)
    if match:
        return '%s_%s' % (match.group(1), label)
    return 'session_%s' % label


def bag_names(folder):
    return sorted(n for n in os.listdir(folder) if n.endswith('.bag')
                  and not n.endswith('.mapping.bag') and os.path.isfile(os.path.join(folder, n)))


def sessions(root):
    result = []
    for name in sorted(os.listdir(root), reverse=True):
        p = os.path.join(root, name)
        if not os.path.isdir(p) or os.path.islink(p) or name.startswith('.'):
            continue
        bags = bag_names(p)
        active = any(n.endswith('.bag.active') for n in os.listdir(p))
        if bags or active:
            result.append(dict(name=name, path=p, bags=len(bags), active=active,
                               size=sum(os.path.getsize(os.path.join(p,n)) for n in bags)))
    loose = bag_names(root)
    if loose:
        named = os.path.basename(root).startswith(('session_','validation_')) or any(
            n.endswith('.session.json') for n in os.listdir(root))
        result.append(dict(name=os.path.basename(root) if named else tr('历史散包（当前目录）'),
                           path=root, bags=len(loose), active=any(n.endswith('.bag.active') for n in os.listdir(root)),
                           size=sum(os.path.getsize(os.path.join(root,n)) for n in loose)))
    return result
