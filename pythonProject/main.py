import os
import sys
import math
import multiprocessing as mp

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from utils import visualize_solution, display_steps

from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory
from builtin_interfaces.msg import Duration
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State


def angle_to_heading(angle):
    real = math.cos(angle * 0.5)
    imag = math.sin(angle * 0.5)
    return real, imag


def f1(scenario, planning_problem, planner_DFS):
    # prepare input for visualization
    scenario_data = (scenario, planner_DFS.state_initial, planner_DFS.shape_ego, planning_problem)

    # display search steps
    p1 = mp.Process(target=display_steps, args=(scenario_data, planner_DFS.execute_search, planner_DFS.config_plot))
    p1.start()


def f2(path):
    traj = create_trajectory_from_list_states(path)
    p2 = mp.Process(target=visualize_solution, args=(scenario, planning_problem_set, traj))
    p2.start()


# read in scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader("/home/drivingsim/workspace/dfg-car/output/1.xml").open()
# retrieve the first planning problem in the problem set
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# load the xml with stores the motion primitives
name_file_motion_primitives = 'V_9.0_9.0_Vstep_0_SA_-0.2_0.2_SAstep_0.4_T_0.5_Model_BMW320i.xml'
# generate automaton
automaton = ManeuverAutomaton.generate_automaton(name_file_motion_primitives)


# construct motion planner
planner_DFS = MotionPlanner.BreadthFirstSearch(scenario=scenario,
                                               planning_problem=planning_problem,
                                               automaton=automaton)
                                                                                                                                                                                                                                                                                                                                                                                                                                                        
f1(scenario, planning_problem, planner_DFS)

from SMP.motion_planner.utility import create_trajectory_from_list_states
path, _, _ = planner_DFS.execute_search()
if path is not None:
    f2(path)

positions = []
traj = Trajectory()
traj.header.frame_id = "map"
if path is not None:
    for primitive in path:
        for state in primitive:
            point = TrajectoryPoint()
            point.time_from_start = Duration(sec=state.time_step)
            point.x = state.position[0]
            point.y = state.position[1]
            point.z = 0.0
            real, imag = angle_to_heading(state.orientation)
            point.heading.real = real
            point.heading.imag = imag
            point.longitudinal_velocity_mps = state.velocity
            # point.heading_rate_rps = state.yaw_rate

            traj.points.append(point)
            positions.append(state.position)

rnd = MPRenderer()
for position in positions:
    ego_vehicle_id = scenario.generate_object_id()
    ego_vehicle_type = ObstacleType.CAR
    ego_vehicle_shape = Rectangle(width=2, length=3)
    ego_vehicle_state = State(position=position, orientation=0)
    static = StaticObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, ego_vehicle_state)
    scenario.add_objects(static)
scenario.draw(rnd, draw_params={'lanelet': {"show_label": False}})
rnd.render()
plt.show()
