"""
TrajectoryLogger Utils
===========================

This module contains utility functions for storing and loading vehicle trajectories as CommonRoad solution files.

---------------------------
"""
import os

from commonroad.common.solution import CommonRoadSolutionReader, CommonRoadSolutionWriter
from commonroad_rp.utility.evaluation import create_planning_problem_solution, create_full_solution_trajectory


class TrajectoryLogger():
    """
    Store and load vehicle trajectories as CommonRoad solution file.

    :var logged_states: list of logged states
    :var timestep: current timestep
    :var reactive_planner_config: reactive planner configuration
    :var detailed_log: detailed logging
    :var logger: logger
    """
    def __init__(self, reactive_planner_config, logger, detailed_log):
        """
        Constructor of the TrajectoryLogger class.

        :param reactive_planner_config: reactive planner configuration
        :param logger: logger
        :param detailed_log: detailed logging
        """
        self.logged_states = []
        self.timestep = 0
        self.reactive_planner_config = reactive_planner_config

        self.detailed_log = detailed_log
        self.logger = logger

    def set_config(self, reactive_planner_config) -> None:
        """
        Set the reactive planner configuration.

        :param reactive_planner_config: reactive planner configuration
        """
        self.reactive_planner_config = reactive_planner_config

    def reset_logger(self) -> None:
        """ Reset the logger."""
        self.logged_states = []
        self.timestep = 0

    def log_state(self, state) -> None:
        """
        Log the current state.

        Call this method every time that you want to log a state.

        :param state: current state
        """
        
        state.time_step = self.timestep

        self.logged_states.append(state)

        if self.detailed_log:
            self.logger.info("Logged current state!")

        self.timestep += 1

    def store_trajectory(self, scenario, planning_problem, filepath) -> None:
        """
        Store the trajectory as a CommonRoad solution file in the solutions folder.

        :param scenario: scenario
        :param planning_problem: planning problem
        :param filepath: file path
        """
        solution_trajectory = create_full_solution_trajectory(self.reactive_planner_config, self.logged_states)
        solution = create_planning_problem_solution(self.reactive_planner_config, solution_trajectory, scenario, planning_problem)

        writer = CommonRoadSolutionWriter(solution)
        dir = os.path.dirname(filepath)
        file = os.path.basename(filepath)
        writer.write_to_file(dir, file, overwrite=True)

        self.logger.info("Solution trajectory stored!")

    def load_trajectory(self, solution_path, min_velocity=0.1) -> list:
        """
        Load a trajectory from a CommonRoad solution file.

        :param solution_path: solution path
        :param min_velocity: minimum velocity
        :return: list of states
        """
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
