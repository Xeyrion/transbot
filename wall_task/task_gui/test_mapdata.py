import os
import unittest

import numpy as np

from mapdata import MapData


class ClearanceTest(unittest.TestCase):
    def test_unknown_and_obstacle_are_rejected(self):
        m = MapData.__new__(MapData)
        m.img = np.full((5, 5), 254, dtype=np.uint8)
        m.h = m.w = 5
        m.res = 1.0
        m.ox = m.oy = 0.0
        self.assertTrue(m.clear(2, 2, 0)[0])
        m.img[2, 2] = 205
        self.assertFalse(m.clear(2, 2, 0)[0])
        m.img[2, 2] = 0
        self.assertFalse(m.clear(2, 2, 0)[0])


if __name__ == '__main__':
    unittest.main()
