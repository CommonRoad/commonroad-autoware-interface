import os

from commonroad.common.solution import CommonRoadSolutionReader, CommonRoadSolutionWriter
from commonroad_rp.reactive_planner import CartesianState
from commonroad_rp.utility.evaluation import create_planning_problem_solution, create_full_solution_trajectory

"""
Store and load vehicle trajectories as CommonRoad solution file
"""
class TrajectoryLogger():
    def __init__(self, reactive_planner_config, logger, detailed_log):
        self.logged_states = []
        self.timestep = 0
        self.reactive_planner_config = reactive_planner_config

        self.detailed_log = detailed_log
        self.logger = logger

    def set_config(self, reactive_planner_config):
        self.reactive_planner_config = reactive_planner_config

    def reset_logger(self):
        self.logged_states = []
        self.timestep = 0

    # call this method every time that you want to log a state
    def log_state(self, state):
        
        state.time_step = self.timestep

        self.logged_states.append(state)

        if self.detailed_log:
            self.logger.info("Logged current state!")

        self.timestep += 1

    # store the trajectory as a CommonRoad solution file in the solutions folder
    def store_trajectory(self, scenario, planning_problem, filepath):
        solution_trajectory = create_full_solution_trajectory(self.reactive_planner_config, self.logged_states)
        solution = create_planning_problem_solution(self.reactive_planner_config, solution_trajectory, scenario, planning_problem)

        writer = CommonRoadSolutionWriter(solution)
        dir = os.path.dirname(filepath)
        file = os.path.basename(filepath)
        writer.write_to_file(dir, file, overwrite=True)

        self.logger.info("Solution trajectory stored!")

    # load a trajectory from a CommonRoad solution file
    def load_trajectory(self, solution_path, min_velocity=0.1):
        solution = CommonRoadSolutionReader.open(solution_path)
        solution_traj = solution.planning_problem_solutions[0].trajectory
        if self.detailed_log:
            self.logger.info("Solution trajectory: " + str(solution_traj))

        states = solution_traj.state_list

        # velocity shouldn't be 0, otherwise car won't drive
        for i in range(len(states)):
            states[i].velocity = max(min_velocity, states[i].velocity)

        for i in range(len(states)-1):
            states[i].acceleration = states[i+1].velocity - states[i].velocity
        states[len(states)-1].acceleration = 0

        return states