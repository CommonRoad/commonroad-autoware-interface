from asyncore import read
from difflib import unified_diff
from lib2to3.refactor import RefactoringTool
from matplotlib import pyplot as plt
import unittest
import pickle
import os
import math

from cr2autoware.rp_interface import RP2Interface
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad
from commonroad.common.file_reader import CommonRoadFileReader

"""
This unittest class makes debugging issues with the reactive planner easier and reproducible.
The reactive planner configuration and parameters can simply be stored in a pickle file
(corresponding method is provided in cr2autoware/rp_interface.py) and directly be loaded here
"""
class CurveTest(unittest.TestCase):

    TEST_PATH = "/home/drivingsim/autoware_tum/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/test/pickle_files"
    TEST_SCENARIOS_PATH = "/home/drivingsim/autoware_tum/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/test/scenarios"
    RP_CONFIG = "/home/drivingsim/autoware_tum/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/reactive-planner/configurations/defaults/"

    # test if the path first and last point of the path are different
    # map used: sample-map-planning
    def _test_car_moving(self):
        planner = self.reactive_planner_from_pickle("rp_interface", "rp_params_moving")
        path = [(cs.position[0], cs.position[1]) for cs in planner.valid_states]
        print("reactive planner states: " + str(path))
        self.assertNotEqual(path[0], path[-1], msg="Start and end of planned path are the same!!")

    # test that no curvilinear projection domain error is thrown
    # map used: sample-map-planning
    def _test_curvilinear_projection_domain(self):
        self.reactive_planner_from_pickle("rp_interface", "rp_params_curv_proj")
        self.assertTrue(True)
            

    # test if reactive planner follows the curve of the reference path
    # map used: sample-map-planning
    def test_car_follows_curve(self):
        planner = self.reactive_planner_from_pickle_with_default_config("rp_params_curve", "curve_ZAM_OpenDrive-123")
        print("planner is optimal?" , str(planner.optimal))
        path = [(cs.position[0], cs.position[1]) for cs in planner.valid_states]
        print("reactive planner states: " + str(path))
        reference_vel = self.load_pickle("rp_params_curve")[3]
        print("reactive planner reference vel: " + str(reference_vel))
        reference_path = self.load_pickle("rp_params_curve")[2]
        print("reactive planner reference path: " + str(reference_path))
        sc = self.load_xml("curve_ZAM_OpenDrive-123")
        initial_pose = self.load_pickle("rp_params_curve")[0]
        # goal_pos = self.load_pickle("rp_params_curve")[1]

        plt.plot(initial_pose.position[0], initial_pose.position[1], marker='x', label="initial position")
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

    def reactive_planner_from_pickle_with_default_config(self, params_filename, scenario_name):
        params = self.load_pickle(params_filename)

        planner = self.mockConfig(scenario_name)
        planner._run_reactive_planner(*params)

        return planner

    def load_pickle(self, filename):
        file = os.path.join(CurveTest.TEST_PATH, filename + ".pkl")
        with open(file, 'rb') as input:
            p = pickle.load(input)
        return p

    def load_xml(self, filename):
        file = os.path.join(CurveTest.TEST_SCENARIOS_PATH, filename + ".xml")
        reader = CommonRoadFileReader(file)
        scenario, _ = reader.open()
        return scenario

    # instead of loading a config from pickle, a default reactive planner config can be used. This might help to find out whether the config itself is flawful
    # works for sample-map-driving
    def mockConfig(self, scenario_name):

        scenario = self.load_xml(scenario_name)

        planner = RP2Interface(scenario,
                                CurveTest.RP_CONFIG,
                                d_min = -3,
                                d_max = 3,
                                t_min = 0.4,            
                                dt = 0.1,
                                planning_horizon = 30 * 0.1,                                      
                                v_length = 3.0,
                                v_width = 2.0,
                                v_wheelbase = 2.0)

        return planner

if __name__ == '__main__':
    unittest.main()