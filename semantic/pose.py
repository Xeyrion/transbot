# -*- coding: utf-8 -*-
"""语义地图用的坐标链：静态环节的已知值 + 极简 3D 变换（py2/py3 通用）。

默认安装值只用于显式开启的估计模式，不代表每台机器人的标定真值。

    base_footprint→base_link   轮子高度，URDF 里的固定关节
    base_link→camera_link      相机安装，transbot_record.launch 里那条
"""
import math
import bisect

# (parent, child) → ((x, y, z), yaw 弧度)
FILL_INS = {
    ('base_footprint', 'base_link'): ((0.0, 0.0, 0.08), 0.0),
    ('base_link', 'camera_link'): ((0.08, 0.0, 0.10403), -0.05),
}


def quat_matrix(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12 or not all(not math.isnan(v) and not math.isinf(v) for v in q):
        raise ValueError('Invalid quaternion')
    q = [v / norm for v in q]
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def transform(xyz, quat):
    """4x4 齐次矩阵；返回嵌套 list，两个 Python 版本都好处理。"""
    r = quat_matrix(quat)
    return [r[0] + [xyz[0]], r[1] + [xyz[1]], r[2] + [xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def yaw_quat(rad):
    return (0.0, 0.0, math.sin(rad / 2.0), math.cos(rad / 2.0))


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def inverse(m):
    """刚体变换的逆：R^T 和 -R^T·t。"""
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [t[0]], r[1] + [t[1]], r[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def apply(m, p):
    """4x4 × 点（3 或 4 维）。"""
    return [sum(m[i][k] * p[k] for k in range(3)) + m[i][3] for i in range(3)]


def matrix_quat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.) * 2
        return [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s,
                (m[1][0]-m[0][1])/s, s/4]
    i = max(range(3), key=lambda j: m[j][j])
    j, k = (i+1)%3, (i+2)%3
    s = math.sqrt(max(0., 1.+m[i][i]-m[j][j]-m[k][k]))*2
    q = [0., 0., 0., 0.]
    q[i], q[j], q[k], q[3] = s/4, (m[i][j]+m[j][i])/s, (m[i][k]+m[k][i])/s, (m[k][j]-m[j][k])/s
    return q


def interpolate(a, b, u):
    qa, qb = matrix_quat(a), matrix_quat(b)
    dot = sum(x*y for x, y in zip(qa, qb))
    if dot < 0:
        qb, dot = [-v for v in qb], -dot
    if dot > .9995:
        q = [(1-u)*x+u*y for x, y in zip(qa, qb)]
    else:
        theta = math.acos(max(-1., min(1., dot)))
        q = [(math.sin((1-u)*theta)*x+math.sin(u*theta)*y)/math.sin(theta)
             for x, y in zip(qa, qb)]
    return transform([(1-u)*a[i][3]+u*b[i][3] for i in range(3)], q)


class Tf(object):
    """SE(3) index. Static edges do not expire; dynamic queries have bounded age."""

    def __init__(self):
        self.edges = {}
        self.parent = {}
        self.static = {}
        self.times = {}
        self.estimated = []

    def add(self, parent, child, stamp, matrix, static=False):
        parent, child = parent.strip('/'), child.strip('/')
        if not parent or not child or parent == child:
            raise ValueError('Invalid TF frame pair')
        if math.isnan(stamp) or math.isinf(stamp):
            raise ValueError('Invalid TF timestamp')
        if not all(not math.isnan(v) and not math.isinf(v) for row in matrix for v in row):
            raise ValueError('Non-finite TF matrix')
        if static:
            old = self.static.get((parent, child))
            if old and max(abs(old[i][j]-matrix[i][j]) for i in range(4) for j in range(4)) > 1e-5:
                raise ValueError('Conflicting static TF: %s -> %s' % (parent, child))
            self.static[(parent, child)] = matrix
        self.edges.setdefault((parent, child), []).append((stamp, matrix))

    def finish(self):
        for key in self.edges:
            self.edges[key].sort(key=lambda e: e[0])
            self.times[key] = [v[0] for v in self.edges[key]]
            for i,(stamp,matrix) in enumerate(self.edges[key]):
                reference=self.static.get(key)
                if reference is None and i and stamp==self.edges[key][i-1][0]: reference=self.edges[key][i-1][1]
                if reference is not None and max(abs(reference[r][c]-matrix[r][c]) for r in range(4) for c in range(4))>1e-5:
                    raise ValueError('Conflicting TF values at '+str(key))
        self.parent = {}
        for parent, child in sorted(self.edges):
            if child in self.parent and self.parent[child] != parent:
                raise ValueError('Multiple TF parents for ' + child)
            self.parent[child] = parent
        for frame in self.parent:
            seen=set(); cur=frame
            while cur in self.parent:
                if cur in seen: raise ValueError('TF graph cycle at '+cur)
                seen.add(cur); cur=self.parent[cur]

    def fill_known(self, log=None, allow=False):
        """Opt-in legacy estimates, with provenance retained in estimated."""
        added = []
        for key in sorted(FILL_INS):
            if allow and key not in self.edges:
                xyz, yaw = FILL_INS[key]
                self.add(key[0], key[1], 0., transform(xyz, yaw_quat(yaw)), static=True)
                added.append(key)
        self.estimated = added
        self.finish()
        if log:
            for parent, child in added:
                log('ESTIMATED installation: %s -> %s' % (parent, child))
        return added

    def at(self, key, stamp):
        entries = self.edges.get(key)
        if not entries:
            return None
        if key in self.static:
            return self.static[key]
        limit = 10.0 if key[0] == 'map' else .5
        i = bisect.bisect_right(self.times[key], stamp)
        if i == 0:
            return entries[0][1] if entries[0][0] - stamp <= limit else None
        before = entries[i-1]
        if stamp - before[0] > limit:
            return None
        if i == len(entries) or key[0] == 'map':
            return before[1]
        after = entries[i]
        if after[0] - stamp > limit:
            return before[1]
        u = (stamp-before[0])/(after[0]-before[0])
        return interpolate(before[1], after[1], u)

    def chain_to(self, frame, stamp, root='map'):
        """root ← frame 的 4x4；中间缺链返回 None。"""
        m = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        cur = frame
        seen = set()
        while cur != root:
            if cur in seen:
                return None
            seen.add(cur)
            parent = self.parent.get(cur)
            if parent is None:
                return None
            step = self.at((parent, cur), stamp)
            if step is None:
                return None
            m = matmul(step, m)
            cur = parent
        return m
