#!/usr/bin/env python3
"""Arm probe: move to a known neutral pose, probe each joint, verify readback.
Uses median of repeated /CurrentAngle reads; reports MOVED / FAILED per joint.
"""
import rospy
import time
from transbot_msgs.srv import RobotArm
from transbot_msgs.msg import Arm, Joint

rospy.init_node('arm_test', anonymous=True)

pub_arm = rospy.Publisher('/TargetAngle', Arm, queue_size=10)

# NOTE: HOME/SAFE 是探针用的保守中位（保证三关节都能 ±20° 活动），
# 不是 URDF 全行程。URDF 全行程见 gui/app.py ARM_LIMITS：
# base 0-225° / lift 15-240° / gripper 60-180°。
HOME = {7: 45.0, 8: 45.0, 9: 45.0}

DELTA = 20.0
MOVE_TIME = 600
SETTLE = 2.0
BURST = 6
SAFE = (0.0, 180.0)


def read_angles():
    try:
        rospy.wait_for_service('/CurrentAngle', timeout=5)
        return rospy.ServiceProxy('/CurrentAngle', RobotArm)(apply='').RobotArm.joint
    except Exception as e:
        print("  read failed:", e)
        return []


def snapshot():
    tally = {}
    for _ in range(BURST):
        for j in read_angles():
            tally.setdefault(j.id, []).append(j.angle)

    pose, agree = {}, {}
    for jid, vals in tally.items():
        vals.sort()
        pose[jid] = vals[len(vals) // 2]
        agree[jid] = vals.count(pose[jid])

    print("  read %dx%d burst, agreement: %s"
          % (BURST, len(tally) or 0,
             ", ".join("%d:%d/%d" % (j, agree[j], BURST) for j in sorted(agree))))
    return pose


def command(jid, ang, run_time=MOVE_TIME):
    ang = max(SAFE[0], min(SAFE[1], ang))
    pub_arm.publish(Arm(joint=[Joint(id=jid, run_time=run_time, angle=float(ang))]))
    return ang


def goto(target, label):
    print("-- going to %s" % label)
    for jid in sorted(target):
        print("  joint %d -> %.1f" % (jid, command(jid, target[jid])))
        time.sleep(SETTLE)
    return snapshot()


def check(expected, label):
    now = snapshot()
    bad = []
    for jid in sorted(expected):
        if jid not in now:
            bad.append("%d NO REPLY" % jid)
            continue
        err = abs(now[jid] - expected[jid])
        print("  joint %d: %.1f (want %.1f, err %.1f)" % (jid, now[jid], expected[jid], err))
        if err > 5.0:
            bad.append("%d off by %.1f" % (jid, err))
    print("  %s: %s" % (label, "OK" if not bad else "PROBLEM " + ", ".join(bad)))
    return not bad


def probe(jid, base):
    start = base.get(jid)
    if start is None:
        print("  joint %d: NOT REPORTED (no bus reply)" % jid)
        return False

    target = start + DELTA
    if target > SAFE[1]:
        target = start - DELTA
    if not (SAFE[0] <= target <= SAFE[1]):
        print("  joint %d: at limit %.1f, skipped" % (jid, start))
        return False

    print("  joint %d: %.1f -> %.1f" % (jid, start, target))
    command(jid, target)
    time.sleep(SETTLE)

    after = snapshot().get(jid)
    if after is None:
        print("  joint %d: STOPPED RESPONDING after move" % jid)
        return False

    err = abs(after - target)
    print("  joint %d: readback %.1f, err %.1f deg -> %s"
          % (jid, after, err, "MOVED" if err <= 5.0 else "NO RESPONSE"))
    command(jid, start)
    time.sleep(SETTLE)
    return err <= 5.0


print("== arm test: goto known neutral, probe each joint, return")
print("HOME = %s" % ", ".join("%d=%.1f" % (k, HOME[k]) for k in sorted(HOME)))

base = goto(HOME, "HOME")
if not base:
    print("no arm data at all -- check /CurrentAngle and transbot_node")
    raise SystemExit(1)
check(HOME, "home reached")

ok = []
for jid in sorted(HOME):
    print("-- probing joint %d" % jid)
    ok.append(probe(jid, base))

check(HOME, "final home")
print("== result: " + ", ".join(
    "joint %d %s" % (j, "MOVED" if r else "FAILED")
    for j, r in zip(sorted(HOME), ok)))
print("DONE")
