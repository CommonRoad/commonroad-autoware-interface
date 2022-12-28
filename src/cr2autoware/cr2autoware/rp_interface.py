from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.configuration_builder import ConfigurationBuilder
import numpy as np

from copy import deepcopy


class RP2Interface:
    
    def __init__(self, scenario, dir_config_default, d_min, d_max, t_min, dt, planning_horizon, v_length, v_width, v_wheelbase, desired_velocity):
        # construct reactive planner
        self.scenario=scenario
        self.config = ConfigurationBuilder.build_configuration(name_scenario=str(self.scenario.scenario_id), dir_config_default=dir_config_default)
        self.reactive_planner = ReactivePlanner(self.config)
        self.reactive_planner.set_d_sampling_parameters(d_min, d_max)
        self.reactive_planner.set_t_sampling_parameters(t_min, dt, planning_horizon)
        self.reactive_planner.vehicle_params.length = v_length
        self.reactive_planner.vehicle_params.width = v_width
        self.reactive_planner.vehicle_params.wheelbase = v_wheelbase
        self.reactive_planner.set_desired_velocity(desired_velocity)

    def _run_reactive_planner(self, init_state, goal, reference_path):
        """
        Run one cycle of reactive planner.
        """
        if not hasattr(init_state, 'acceleration'):
            init_state.acceleration = 0.0

        x_0 = deepcopy(init_state)      
                
        # goal state configuration
        """if hasattr(goal.state_list[0], 'velocity'):
            desired_velocity = (goal.state_list[0].velocity.start +
                                goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = x_0.velocity
        # set desired velocity
        self.reactive_planner.set_desired_velocity(desired_velocity)"""


        # set collision checker
        self.reactive_planner.set_collision_checker(self.scenario)
        # set route
        self.reactive_planner.set_reference_path(reference_path)
        self.valid_states = []
        # run planner and plan trajectory once
        self.optimal = self.reactive_planner.plan(x_0)  # returns the planned (i.e., optimal) trajectory
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