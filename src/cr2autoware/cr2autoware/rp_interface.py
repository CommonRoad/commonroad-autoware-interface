from copy import deepcopy
import os
import pickle
import typing
import logging

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
        veh_length,
        veh_width,
        veh_wheelbase,
        veh_wb_rear_axle,
        veh_max_steering_angle,
        veh_max_acceleration
    ):
        traj_planner_params = params.trajectory_planner
        rp_params = params.rp_interface

        # construct reactive planner
        self.scenario = scenario
        self.config = ConfigurationBuilder.build_configuration(
            name_scenario=str(self.scenario.scenario_id),
            dir_config_default=rp_params.dir_config_default.as_posix(),
        )
        self.reactive_planner = ReactivePlanner(self.config)
        self.reactive_planner.set_d_sampling_parameters(rp_params.d_min, rp_params.d_max)
        self.reactive_planner.set_t_sampling_parameters(rp_params.t_min, dt, traj_planner_params.planning_horizon)
        self.reactive_planner.vehicle_params.length = veh_length
        self.reactive_planner.vehicle_params.width = veh_width
        self.reactive_planner.vehicle_params.wheelbase = veh_wheelbase
        self.reactive_planner.vehicle_params.rear_ax_distance = veh_wb_rear_axle
        self.reactive_planner.vehicle_params.delta_max = veh_max_steering_angle
        self.reactive_planner.vehicle_params.a_max = veh_max_acceleration

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


###########################################################
# NEW Class for updated reactive planner
###########################################################
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger

from cr2autoware.configuration import RPInterfaceParams
from cr2autoware.ego_vehicle_handler import EgoVehicleHandler

from rclpy.publisher import Publisher


class ReactivePlannerInterface(TrajectoryPlannerInterface):
    """
    Trajectory planner interface for the CommonRoad Reactive Planner
    """
    def __init__(self, traj_pub: Publisher,
                 scenario,
                 planning_problem,
                 dt,
                 horizon,
                 rp_interface_params: RPInterfaceParams,
                 ego_vehicle_handler: EgoVehicleHandler):
        # set scenario
        self.scenario = scenario

        # create reactive planner config
        rp_config = ReactivePlannerConfiguration.load(file_path=rp_interface_params.dir_config_default.as_posix(),
                                                      scenario_name=str(scenario.scenario_id))
        rp_config.update(scenario=scenario, planning_problem=planning_problem)

        # overwrite time step and horizon
        rp_config.planning.dt = dt
        rp_config.planning.planning_horizon = horizon

        # overwrite vehicle params in planner config
        rp_config.vehicle.length = ego_vehicle_handler.vehicle_length
        rp_config.vehicle.width = ego_vehicle_handler.vehicle_width
        rp_config.vehicle.wheelbase = ego_vehicle_handler.vehicle_wheelbase
        rp_config.vehicle.rear_ax_distance = ego_vehicle_handler.vehicle_wb_rear_axle
        rp_config.vehicle.delta_min = -ego_vehicle_handler.vehicle_max_steer_angle
        rp_config.vehicle.delta_max = ego_vehicle_handler.vehicle_max_steer_angle
        rp_config.vehicle.a_max = ego_vehicle_handler.vehicle_max_acceleration

        # initialize reactive planner logger
        initialize_logger(rp_config)

        # initialize reactive planner object
        reactive_planner: ReactivePlanner = ReactivePlanner(rp_config)

        # adjust sampling settings from ROS params
        reactive_planner.set_t_sampling_parameters(t_min=rp_interface_params.get_ros_param("t_min"))
        reactive_planner.set_d_sampling_parameters(delta_d_min=rp_interface_params.get_ros_param("d_min"),
                                                   delta_d_max=rp_interface_params.get_ros_param("d_max"))

        # init parent class
        super().__init__(traj_planner=reactive_planner, traj_pub=traj_pub)

    def plan(self, current_state, goal, reference_path=None, reference_velocity=None):
        """Overrides plan method from base class and calls the planning algorithm of the reactive planner"""
        pass

    def update(self):
        """Overrides update method from base class"""
        pass
