from copy import deepcopy
import os
import pickle

from commonroad_rp.configuration_builder import ConfigurationBuilder
from commonroad_rp.reactive_planner import ReactivePlanner

from cr2autoware.trajectory_planner_interface import TrajectoryPlannerInterface


class RP2Interface(TrajectoryPlannerInterface):
    def __init__(
        self,
        scenario,
        dt,
        trajectory_logger,
        params,
    ):
        # construct reactive planner
        self.scenario = scenario
        self.config = ConfigurationBuilder.build_configuration(
            name_scenario=str(self.scenario.scenario_id),
            dir_config_default=params.dir_config_default.as_posix(),
        )
        self.reactive_planner = ReactivePlanner(self.config)
        self.reactive_planner.set_d_sampling_parameters(params.d_min, params.d_max)
        self.reactive_planner.set_t_sampling_parameters(params.t_min, dt, params.planning_horizon)
        self.reactive_planner.vehicle_params.length = params.vehicle_length
        self.reactive_planner.vehicle_params.width = params.vehicle_width
        self.reactive_planner.vehicle_params.wheelbase = params.vehicle_wheelbase

        # self.save_to_pickle("rp_interface", scenario, dir_config_default, d_min, d_max, t_min, dt, planning_horizon, v_length, v_width, v_wheelbase, trajectory_logger)
        trajectory_logger.set_config(self.config)

    def plan(self, init_state, goal, reference_path, reference_velocity):
        """Run one cycle of reactive planner."""
        if not hasattr(init_state, "acceleration"):
            init_state.acceleration = 0.0

        x_0 = deepcopy(init_state)

        # self.save_to_pickle("rp_params", init_state, goal, reference_path, reference_velocity)
        self.reactive_planner.set_desired_velocity(reference_velocity)

        # set collision checker
        self.reactive_planner.set_collision_checker(self.scenario)
        # set route
        self.reactive_planner.set_reference_path(reference_path)
        self.valid_states = []
        # run planner and plan trajectory once
        self.optimal = self.reactive_planner.plan(
            x_0
        )  # returns the planned (i.e., optimal) trajectory
        # if the planner fails to find an optimal trajectory -> terminate
        if self.optimal:
            # correct orientation angle
            self.planner_state_list = self.reactive_planner.shift_orientation(self.optimal[0])
            for state in self.planner_state_list.state_list:
                if len(self.valid_states) > 0:
                    last_state = self.valid_states[-1]
                    if last_state.time_step == state.time_step:
                        continue
                self.valid_states.append(state)

    # This function can be used to generate test cases for test_rp.py
    def save_to_pickle(self, filename, *args):
        # save file to test/pickle_files
        test_path = os.path.dirname(__file__) + "/test/pickle_files"
        file = os.path.join(test_path, filename + ".pkl")
        with open(file, "wb") as output:
            pickle.dump(args, output, pickle.HIGHEST_PROTOCOL)
