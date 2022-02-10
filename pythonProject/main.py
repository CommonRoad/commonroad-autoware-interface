import os
import sys
import matplotlib.pyplot as plt

path_notebook = '/home/yashuai/Software/adehome/workspace/commonroad-search/tutorials/1_search_algorithms'
# add the SMP folder to python path
sys.path.append(os.path.join(path_notebook, "../../"))

# add the 1_search_algorithms folder to python path
sys.path.append(os.path.join(path_notebook, "../"))

from commonroad.common.file_reader import CommonRoadFileReader
#from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.visualization.mp_renderer import MPRenderer

from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.utility import plot_primitives, display_steps

# load scenario
path_scenario = os.path.join(path_notebook, "../../scenarios/tutorial/")
id_scenario = 'ZAM_Tutorial_Urban-3_2'

# read in scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader("/home/yashuai/Software/adehome/workspace/dfg-car/output/1.xml").open()
# retrieve the first planning problem in the problem set
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# plot the scenario and the planning problem set
plt.figure(figsize=(15, 5))
rnd = MPRenderer()
scenario.draw(rnd, draw_params={'lanelet': {"show_label": True}})
planning_problem_set.draw(rnd)
rnd.render()

# load the xml with stores the motion primitives
name_file_motion_primitives = 'V_0.0_20.0_Vstep_2.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml'
# generate automaton
automaton = ManeuverAutomaton.generate_automaton(name_file_motion_primitives)


# construct motion planner
planner_DFS = MotionPlanner.DepthFirstSearch(scenario=scenario,
                                             planning_problem=planning_problem,
                                             automaton=automaton)
# prepare input for visualization
scenario_data = (scenario, planner_DFS.state_initial, planner_DFS.shape_ego, planning_problem)

# display search steps
display_steps(scenario_data=scenario_data,
              algorithm=planner_DFS.execute_search,
              config=planner_DFS.config_plot)
print('####################')
plt.show()