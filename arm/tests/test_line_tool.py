# -*- coding: utf-8 -*-
"""Regression tests for line_tool.py (correct relative-angle convention).

The archived arm_model.py used rel = j7+j8-275 (treating j8 as a world
angle).  These tests pin the corrected convention e1=j7-95, rel=j8-180.
Geometry uses the measured installation (L1=10.0, L2=24.0), not the
vendor URDF; see CALIBRATION.md.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import line_tool as L  # noqa: E402


class TestLineTool(unittest.TestCase):
    def test_ready_pose_fk(self):
        # With the measured L1=10 cm this pose is (26.66, 9.64).  The vendor
        # URDF value (8.285) would give (26.2, 8.0), which is not the real arm.
        lx, lz = L.fk(169.6, 105.4)
        self.assertAlmostEqual(lx, 26.66, places=1)
        self.assertAlmostEqual(lz, 9.64, places=1)

    def test_old_bug_counterexample(self):
        # Under the archived (wrong) formula, (169.6, 30.8) -> (26.2, 8.0).
        # With the corrected relative-angle convention the tip is elsewhere
        # (in fact below ground): the old "ready" would bury the pen.
        lx, lz = L.fk(169.6, 30.8)
        self.assertLess(lx, 12.0)
        self.assertLess(lz, 0.0)

    def test_level_pose_fk(self):
        lx, lz = L.fk(95.0, 180.0)
        self.assertAlmostEqual(lx, L.L1 + L.L2, places=4)
        self.assertAlmostEqual(lz, 0.0, places=4)

    def test_ik_roundtrip(self):
        for q in ((169.6, 105.4), (150.0, 120.0), (120.0, 200.0),
                  (95.0, 180.0), (200.0, 60.0)):
            lx, lz = L.fk(*q)
            sols = L.ik(lx, lz)
            self.assertTrue(sols, 'no IK for %s' % (q,))
            best = min(sols, key=lambda s: abs(s[0] - q[0])
                       + abs(s[1] - q[1]))
            self.assertAlmostEqual(best[0], q[0], places=4)
            self.assertAlmostEqual(best[1], q[1], places=4)

    def test_ik_unreachable(self):
        self.assertEqual(L.ik(100.0, 0.0), [])   # beyond max reach
        self.assertEqual(L.ik(0.0, 0.0), [])     # at the axis

    def test_line_range_contains_ready(self):
        # ready tip is ~8 cm above the j7 axis now that z is j7-relative
        runs, branch = L.line_range(26.2)
        self.assertTrue(runs)
        self.assertTrue(any(a <= 7.99 <= b for a, b in runs))
        self.assertEqual(branch, 'elbow-down')

    def test_line_range_unreachable_dist(self):
        runs, _ = L.line_range(100.0)
        self.assertEqual(runs, [])

    def test_draw_points_continuity(self):
        runs, _ = L.line_range(26.2)
        a, b = runs[0]
        z0 = max(a, -3.0)
        z1 = min(b, 7.0)
        pts = L.draw_points(26.2, z0, z1, step_cm=1.0)
        self.assertGreater(len(pts), 2)
        for (j7, j8) in pts:
            lx, lz = L.fk(j7, j8)
            self.assertAlmostEqual(lx, 26.2, places=4)
        for p, q in zip(pts, pts[1:]):
            self.assertLess(abs(p[0] - q[0]) + abs(p[1] - q[1]), 5.0)

    def test_draw_points_rejects_outside(self):
        self.assertRaises(ValueError, L.draw_points, 26.2, 25.0, 30.0)

    def test_line_range_keeps_reach_margin(self):
        # At 30 cm the geometric limit is +-16.0 cm (reach 34 cm), but that
        # is full extension.  The planner must stop REACH_MARGIN short of it.
        runs, _ = L.line_range(30.0)
        self.assertTrue(runs)
        a, b = runs[0]
        z_safe = math.sqrt((L.REACH_MAX - L.REACH_MARGIN) ** 2 - 30.0 ** 2)
        # line_range scans in 0.05 cm steps, so it may stop just inside.
        self.assertLessEqual(max(abs(a), abs(b)), z_safe + 1e-6)
        self.assertGreater(b, z_safe - 0.1)
        self.assertLess(a, -(z_safe - 0.1))

    def test_floor_safe_range(self):
        runs, _ = L.line_range(26.2, z_lo=L.Z_FLOOR_REL)
        self.assertTrue(runs)
        self.assertAlmostEqual(runs[0][0], L.Z_FLOOR_REL, places=2)

    def test_ik_margin(self):
        d = L.REACH_MAX - 1.0
        self.assertTrue(L.ik(d, 0.0))
        self.assertEqual(L.ik(d, 0.0, margin=L.REACH_MARGIN), [])

    def test_approach_follows_straight_tip_path(self):
        ref = (151.0, 163.0)
        dist, z_target = 30.0, -4.1
        pts = L.approach_points(dist, z_target, ref)
        self.assertGreater(len(pts), 10)
        lx, lz = L.fk(*pts[-1])
        self.assertAlmostEqual(lx, dist, places=4)
        self.assertAlmostEqual(lz, z_target, places=4)
        prev = ref
        for q in pts:
            lx, lz = L.fk(*q)
            # The tip stays on the straight segment; it cannot bow toward
            # the wall (x>dist) or dip below the line start.
            self.assertLessEqual(lx, dist + 1e-6)
            self.assertGreaterEqual(lz, z_target - 1e-6)
            self.assertLess(
                max(abs(q[0] - prev[0]), abs(q[1] - prev[1])), 45.0)
            prev = q
        self.assertLess(
            max(abs(pts[0][0] - ref[0]), abs(pts[0][1] - ref[1])), 15.0)

    def test_approach_from_jammed_pose(self):
        # Last live reading before the arm stopped: tip was below the table.
        pts = L.approach_points(26.2, -5.5, (76.0, 167.0))
        self.assertTrue(pts)
        lx, lz = L.fk(*pts[-1])
        self.assertAlmostEqual(lx, 26.2, places=4)
        self.assertAlmostEqual(lz, -5.5, places=4)

    def test_approach_lifts_before_leaving_floor(self):
        ref = (76.0, 167.0)
        lx0, lz0 = L.fk(*ref)
        self.assertLess(lz0, L.Z_FLOOR_REL)
        pts = L.approach_points(30.0, -4.1, ref, floor_z=L.Z_FLOOR_REL)
        lift = None
        for q in pts:
            lx, lz = L.fk(*q)
            if abs(lz - L.Z_FLOOR_REL) < 1e-6:
                lift = (lx, lz)
                break
        self.assertIsNotNone(lift)
        self.assertAlmostEqual(lift[0], lx0, places=3)
        lx, lz = L.fk(*pts[-1])
        self.assertAlmostEqual(lx, 30.0, places=4)
        self.assertAlmostEqual(lz, -4.1, places=4)

    def test_approach_keeps_elbow_branch(self):
        # 26.2 cm / z=-5.5 is only reachable elbow-down; an elbow-up start
        # must refuse instead of flipping through full extension.
        self.assertRaises(ValueError, L.approach_points, 26.2, -5.5,
                          (120.0, 190.0))
        # The same start can reach the upper part of a 30 cm line on the
        # elbow-up branch; z=-4.1 would force a branch flip halfway.
        pts = L.approach_points(30.0, 8.0, (120.0, 190.0))
        self.assertTrue(pts)
        self.assertTrue(all(p[1] > 180.0 for p in pts))

    def test_draw_points_locks_branch(self):
        self.assertRaises(ValueError, L.draw_points, 26.2, -5.5, 10.0,
                          ref=(120.0, 190.0))

    def test_cartesian_path_polyline(self):
        ref = (151.0, 163.0)
        cart, joints = L.cartesian_path(26.2, -5.5, 10.0, ref)
        self.assertEqual(len(cart), len(joints))
        self.assertGreater(len(cart), 10)
        # The path starts at the measured tip and ends at the line end.
        self.assertAlmostEqual(cart[0][0], L.fk(*ref)[0], places=6)
        self.assertAlmostEqual(cart[0][1], L.fk(*ref)[1], places=6)
        self.assertAlmostEqual(cart[-1][0], 26.2, places=4)
        self.assertAlmostEqual(cart[-1][1], 10.0, places=4)
        for (lx, lz), (j7, j8) in zip(cart, joints):
            fx, fz = L.fk(j7, j8)
            self.assertAlmostEqual(lx, fx, places=6)
            self.assertAlmostEqual(lz, fz, places=6)
        # The line part stays at x=dist after the approach reaches it.
        self.assertAlmostEqual(cart[-2][0], 26.2, places=4)


if __name__ == '__main__':
    unittest.main()
