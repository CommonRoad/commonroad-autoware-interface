from asyncore import read
from difflib import unified_diff
from lib2to3.refactor import RefactoringTool
from matplotlib import pyplot as plt
import unittest
import pickle
import os
import math

from cr2autoware.rp_interface import RP2Interface

"""
This unittest class makes debugging issues with the reactive planner easier and reproducible.
The reactive planner configuration and parameters can simply be stored in a pickle file
(corresponding method is provided in cr2autoware/rp_interface.py) and directly be loaded here
"""
class CurveTest(unittest.TestCase):

    # test if the path first and last point of the path are different
    # map used: sample-map-planning
    def test_car_moving(self):
        planner = self.reactive_planner_from_pickle("rp_interface_moving", "rp_params_moving")
        path = [(cs.position[0], cs.position[1]) for cs in planner.valid_states]
        print("reactive planner states: " + str(path))
        self.assertNotEqual(path[0], path[-1], msg="Start and end of planned path are the same!!")

    # test that no curvilinear projection domain error is thrown
    # map used: sample-map-planning
    def test_curvilinear_projection_domain(self):
        self.reactive_planner_from_pickle("rp_interface_curv_proj", "rp_params_curv_proj")
        self.assertTrue(True)
            

    # test if reactive planner follows the curve of the reference path
    # map used: sample-map-planning
    def test_car_follows_curve(self):
        planner = self.reactive_planner_from_pickle("rp_interface_curve", "rp_params_curve")
        path = [(cs.position[0], cs.position[1]) for cs in planner.valid_states]
        print("reactive planner states: " + str(path))
        reference_path = self.load_pickle("rp_params_curve")[2]
        print("reactive planner reference path: " + str(reference_path))
        plt.plot(*zip(*path), label="planned trajectory")
        plt.plot(*zip(*reference_path), label="reference path")
        plt.legend(loc="upper right")
        plt.title("Reactive planner reference path vs. planned trajectory\n(test_car_follows_curve)")
        plt.show()

        # calculate distance between last point of trajectory and reference path
        te = path[-1]
        min_dist = math.inf
        for rpp in reference_path:
            dist = math.sqrt((te[0] - rpp[0])**2 + (te[1] - rpp[1])**2)
            if dist < min_dist:
                min_dist = dist
        self.assertLess(min_dist, 2, msg="The end of the planned trajectory differs too much from the reference path! The car isn't following the curve!! See displayed plot for details")

    # load a reactive planner configuration from a pickle file and run the reactive planner with parameters from another pickle file
    # returns the planner object
    def reactive_planner_from_pickle(self, config_filename, params_filename):
        config = self.load_pickle(config_filename)
        params = self.load_pickle(params_filename)

        planner = RP2Interface(*config)
        planner._run_reactive_planner(*params)

        return planner

    def load_pickle(self, filename):
        test_path = "/home/drivingsim/Documents/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/test/pickle_files"
        file = os.path.join(test_path, filename + ".pkl")
        with open(file, 'rb') as input:
            p = pickle.load(input)
        return p

    # instead of loading a config from pickle, a default reactive planner config can be used. This might help to find out whether the config itself is flawful
    def mockConfig(self):
        pass

if __name__ == '__main__':
    unittest.main()