#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""line_tool.py -- straight-vertical-line math for the simple 2R arm.

Pure computation, no ROS: FK / IK / reachable-range on a vertical line.
Angle convention (corrected 2026-10-28, verified live in Gazebo):

    e1  = j7 - 95          link1 world elevation [deg]
    rel = j8 - 180         elbow *relative* angle [deg] (servo reading)
    e2  = e1 + rel         link2 world elevation [deg]

Coordinates (cm), all from the j7 axis:
    (lx, lz) : pen tip relative to the j7 axis; lz = 0 IS j7 height.
    DIST (user input) = lx, the horizontal j7-axis distance.
    z0/z1 (user input) = lz, height above (+) / below (-) the j7 axis.
    World conversions (to_world/from_world) exist only for reference.
"""
from __future__ import division, print_function

import math

J7_ZERO = 95.0
J8_ZERO = 180.0
J7_MIN, J7_MAX = 0.0, 225.0
J8_MIN, J8_MAX = 15.0, 240.0

# Measured installation, NOT the vendor URDF.  The real arm was installed by
# hand: the j7->j8 centre distance is 10.0 cm (steel tape, +/-0.5 cm), while
# the vendor URDF says 8.285 cm.  Using the URDF value here makes the real
# Cartesian loop believe the pen is 1.7 cm away from where it actually is.
# Source: CALIBRATION.md (copied from the archived 2026-10-09 measurement).
L1 = 10.0                 # j7 axis -> j8 axis [cm]  (measured)
L2 = 24.0                 # j8 axis -> pen tip [cm]  (measured, j9~159)
J7_AXIS_X = 0.129         # j7 axis world x [m]      (measured)
J7_AXIS_Z = 0.08          # j7 axis world height [m] (measured)
# The real arm's j7 axis is about 8 cm above the table.  Keep the pen at
# least FLOOR_MARGIN above the table; below this the arm pushes into it.
FLOOR_MARGIN = 2.5        # cm
Z_FLOOR_REL = -100.0 * J7_AXIS_Z + FLOOR_MARGIN  # = -5.5 cm

D2R = math.pi / 180.0
REACH_MAX = L1 + L2       # 34.0 cm (full extension)
# Keep the elbow slightly bent when planning a line.  At full extension the
# arm is singular: gravity torque is highest, the servo is most likely to
# stall, and the two IK branches merge.  The archived wall project used the
# same 2 cm margin; without it 30 cm line endpoints sit at d=32.28 cm.
REACH_MARGIN = 2.0        # cm


def fk(j7, j8):
    """(j7, j8) deg -> (lx, lz) cm, tip relative to the j7 axis."""
    e1 = (j7 - J7_ZERO) * D2R
    e2 = ((j7 - J7_ZERO) + (j8 - J8_ZERO)) * D2R
    return (L1 * math.cos(e1) + L2 * math.cos(e2),
            L1 * math.sin(e1) + L2 * math.sin(e2))


def to_world(lx, lz):
    """j7-relative (cm) -> world (cm)."""
    return (100.0 * J7_AXIS_X + lx, 100.0 * J7_AXIS_Z + lz)


def from_world(x, z):
    """world (cm) -> j7-relative (cm)."""
    return (x - 100.0 * J7_AXIS_X, z - 100.0 * J7_AXIS_Z)


def ik(lx, lz, margin=0.0):
    """Tip (lx, lz) cm rel j7 axis -> [(j7, j8), ...] within servo limits.

    Both elbow branches; empty list means unreachable.  rel comes from the
    law of cosines: d^2 = L1^2 + L2^2 + 2*L1*L2*cos(rel).
    margin > 0 rejects targets closer than that to full extension; the line
    planner uses REACH_MARGIN, while plain FK/IK round-trips can use 0.
    """
    d = math.hypot(lx, lz)
    d_max = REACH_MAX - max(0.0, float(margin))
    if d < 1e-9 or d > d_max:
        return []
    c = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c < -1.0 or c > 1.0:
        return []
    phi = math.atan2(lz, lx)
    out = []
    for sgn in (1.0, -1.0):
        rel = sgn * math.acos(max(-1.0, min(1.0, c)))
        e1 = phi - math.atan2(L2 * math.sin(rel),
                              L1 + L2 * math.cos(rel))
        j7 = math.degrees(e1) + J7_ZERO
        j8 = math.degrees(rel) + J8_ZERO
        if J7_MIN - 1e-9 <= j7 <= J7_MAX + 1e-9 \
                and J8_MIN - 1e-9 <= j8 <= J8_MAX + 1e-9:
            out.append((j7, j8))
    return out


def ik_branch(lx, lz, down, margin=0.0):
    """IK solutions on one elbow branch: down=True keeps j8<180.

    down=None returns both branches (same as ik()).
    """
    sols = ik(lx, lz, margin=margin)
    if down is None:
        return sols
    return [s for s in sols if (s[1] < J8_ZERO) == down]


def _branch_down(ref):
    """Which elbow branch ref is on: True=down (j8<180), False=up.

    None means ref sits exactly at full extension (j8=180), where the two
    branches meet and either one is acceptable for the first step.
    """
    if ref is None:
        return None
    if abs(ref[1] - J8_ZERO) < 1e-9:
        return None
    return ref[1] < J8_ZERO


def branch_down(ref):
    """Public wrapper: True=elbow-down (j8<180), False=up, None=singular."""
    return _branch_down(ref)


def _same_branch(sols, down):
    """Filter IK solutions to one elbow branch; raise if it is not there."""
    if down is None:
        return sols
    out = [s for s in sols if (s[1] < J8_ZERO) == down]
    if not out:
        raise ValueError('this pose needs the other elbow branch')
    return out


def approach_points(dist, z_target, ref, step_cm=0.5, floor_z=None):
    """Joint waypoints from ref's tip straight to (dist, z_target) [cm].

    The tip follows a Cartesian straight line, so it cannot bow toward the
    wall or dip below the start on the way in.  The elbow branch of ref is
    kept for the whole approach; if the target is only reachable on the other
    branch, ValueError says so instead of silently flipping through the
    straight-arm singularity.

    floor_z: if the tip starts below this height and the target is above it,
    first lift straight up at the current x to floor_z, then move over.  This
    is the recovery path for a pen that is stuck below the table: it does not
    push farther into the wall while still under the tabletop.

    Returned list includes the target but not ref.  Use margin=0 here: the
    approach is a transit, and the current pose itself may be closer to full
    extension than the drawing margin allows.
    """
    if ref is None:
        raise ValueError('approach needs the current joint pose')
    lx0, lz0 = fk(ref[0], ref[1])
    targets = []
    if floor_z is not None and lz0 < float(floor_z) - 1e-9 \
            and float(z_target) >= float(floor_z) - 1e-9:
        targets.append((lx0, float(floor_z)))
    targets.append((float(dist), float(z_target)))

    down = _branch_down(ref)
    out = []
    prev = ref
    for tx, tz in targets:
        px, pz = fk(prev[0], prev[1])
        dx = tx - px
        dz = tz - pz
        length = math.hypot(dx, dz)
        if length < 1e-6:
            continue
        n = max(1, int(math.ceil(length / max(0.05, float(step_cm)))))
        for k in range(1, n + 1):
            lx = px + dx * k / n
            lz = pz + dz * k / n
            sols = ik(lx, lz, margin=0.0)
            if not sols:
                raise ValueError('approach point unreachable: x=%.2f z=%.2f'
                                 % (lx, lz))
            try:
                sols = _same_branch(sols, down)
            except ValueError:
                raise ValueError(
                    'approach needs the other elbow branch at x=%.2f z=%.2f'
                    % (lx, lz))
            q = min(sols, key=lambda s: (s[0] - prev[0]) ** 2 +
                    (s[1] - prev[1]) ** 2)
            if down is None:
                down = q[1] < J8_ZERO   # lock the branch after the first step
            out.append(q)
            prev = q
    return out


def line_range(dist, z_lo=-30.0, z_hi=30.0, dz=0.05):
    """Reachable height runs (rel j7 axis) on the vertical line lx=dist.

    dist : horizontal j7-axis distance [cm] (user input).
    Returns ([(z0, z1), ...] in cm above/below the j7 axis, branch) where
    branch is the j8 at mid-run ('elbow-down' j8<180 or 'elbow-up'
    j8>180), '' if none.
    """
    runs = []
    cur = None
    z = z_lo
    while z <= z_hi + 1e-9:
        lx = float(dist)
        lz = z
        sols = ik(lx, lz, margin=REACH_MARGIN)
        if sols:
            if cur is None:
                cur = [z, z]
            else:
                cur[1] = z
        else:
            if cur is not None:
                runs.append(tuple(cur))
                cur = None
        z += dz
    if cur is not None:
        runs.append(tuple(cur))
    branch = ''
    if runs:
        m = (runs[0][0] + runs[0][1]) / 2.0
        sols = ik(float(dist), m, margin=REACH_MARGIN)
        if sols:
            j8 = min(sols, key=lambda s: abs(s[1] - 140.0))[1]
            branch = 'elbow-down' if j8 < 180.0 else 'elbow-up'
    return runs, branch


def draw_points(dist, z0, z1, step_cm=0.5, ref=None):
    """j7-relative height segment -> [(j7, j8), ...] with branch continuity.

    z0/z1 in cm above (+) / below (-) the j7 axis.
    ref=(j7, j8) picks the first IK branch closest to the current arm, so a
    real arm already on one elbow branch does not flip through the singular
    straight-arm pose.  Without ref the old sols[0] choice is kept.
    Raises ValueError if an endpoint (or any sample) is unreachable.
    """
    if z1 < z0:
        z0, z1 = z1, z0
    n = max(2, int(math.ceil((z1 - z0) / step_cm)) + 1)
    pts = []
    prev = None
    down = _branch_down(ref)
    for k in range(n):
        z = z0 + (z1 - z0) * k / (n - 1)
        sols = ik(float(dist), z, margin=REACH_MARGIN)
        if not sols:
            raise ValueError('line point unreachable: dist=%.2f z=%.2f'
                             % (dist, z))
        try:
            sols = _same_branch(sols, down)
        except ValueError:
            raise ValueError(
                'line needs the other elbow branch: dist=%.2f z=%.2f'
                % (dist, z))
        if prev is None and ref is not None:
            q = min(sols, key=lambda s: (s[0] - ref[0]) ** 2 +
                    (s[1] - ref[1]) ** 2)
        elif prev is None:
            q = sols[0]
        else:
            q = min(sols, key=lambda s: (s[0] - prev[0]) ** 2 +
                    (s[1] - prev[1]) ** 2)
        pts.append(q)
        prev = q
    return pts


def cartesian_path(dist, z0, z1, ref, step_cm=0.5, floor_z=None):
    """Validated path as both joint angles and Cartesian tip points.

    Returns (cart, joints):
      cart[i]   = (lx, lz) of joints[i], relative to the j7 axis [cm]
      joints[i] = (j7, j8) servo angles [deg]

    The path goes from ref's tip through the same approach as
    approach_points() (including the optional straight-up recovery) and then
    along the vertical segment z0..z1 at x=dist.  Both lists include the
    target endpoint.  Raises ValueError if any part is unreachable or would
    require switching elbow branches.
    """
    z_start = min(z0, z1)
    approach = approach_points(dist, z_start, ref, step_cm=step_cm,
                               floor_z=floor_z)
    line_ref = approach[-1] if approach else ref
    line = draw_points(dist, z0, z1, step_cm=step_cm, ref=line_ref)
    if approach and line and max(
            abs(approach[-1][0] - line[0][0]),
            abs(approach[-1][1] - line[0][1])) < 1e-6:
        joints = approach[:-1] + line
    else:
        joints = approach + line
    # Include the measured start pose so the closed loop's arc length starts
    # at zero exactly where the pen is now.
    joints = [ref] + joints
    cart = [fk(*q) for q in joints]
    return cart, joints


def describe(dist):
    """Human-readable range report for `draw_line.py DIST`."""
    runs, branch = line_range(dist)
    lines = ['dist = %.2f cm (j7 axis -> line, horizontal)' % dist,
             'heights rel j7 axis: 0 = j7 height, + = above, - = below']
    if not runs:
        lines.append('unreachable: no joint pose puts the tip on this line.')
        return '\n'.join(lines)
    for a, b in runs:
        lines.append('drawable z in [%+.2f, %+.2f] cm  (height %.2f cm)'
                     % (a, b, b - a))
    lines.append('branch: %s' % (branch or '?'))
    return '\n'.join(lines)
