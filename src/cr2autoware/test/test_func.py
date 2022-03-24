import unittest
import sys
sys.path.append('..')
from cr2autoware.cr2autoware import Cr2Auto
import math
from geometry_msgs.msg import PoseStamped, Quaternion, Point


class MyTestCase(unittest.TestCase):
    def test_orientation2quaternion(self):
        test = 0.2
        w = math.cos(test * 0.5)
        z = math.sin(test * 0.5)
        quad = Cr2Auto.orientation2quaternion(0.2)

        self.assertEqual(z, quad.z)  # add assertion here
        self.assertEqual(w, quad.w)

    def test_quaternion2orientation(self):
        test = Quaternion()
        z, w = 0.5, 0.6
        test.z = z
        test.w = w

        mag2 = (z * z) + (w * w)
        epsilon = 1e-6
        if abs(mag2 - 1.0) > epsilon:
            mag = 1.0 / math.sqrt(mag2)
            z *= mag
            w *= mag

        y = 2.0 * w * z
        x = 1.0 - 2.0 * z * z
        res = math.atan2(y, x)

        sol = Cr2Auto.quaternion2orientation(test)

        self.assertEqual(res, sol)  # add assertion here


if __name__ == '__main__':
    unittest.main()
