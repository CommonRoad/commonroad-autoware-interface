# third party imports
import numpy as np

# commonroad imports
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

# commonroad-dc
import commonroad_dc.pycrcc as pycrcc

# route planner
from commonroad_route_planner.route_planner import Route

# commonroad-rp imports
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.reactive_planner import ReactivePlanner

# cr2autoware
from cr2autoware.configuration import RPInterfaceParams
from cr2autoware.ego_vehicle_handler import EgoVehicleHandler, EgoVehicleState
from cr2autoware.trajectory_planner_interface import TrajectoryPlannerInterface

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger


class ReactivePlannerInterface(TrajectoryPlannerInterface):
    """
    Trajectory planner interface for the CommonRoad Reactive Planner
    """
    def __init__(self, traj_pub: Publisher,
                 logger: RcutilsLogger,
                 verbose: bool,
                 scenario: Scenario,
                 planning_problem: PlanningProblem,
                 road_boundary: pycrcc.CollisionObject,
                 dt: float,
                 horizon: float,
                 rp_interface_params: RPInterfaceParams,
                 ego_vehicle_handler: EgoVehicleHandler):

        # init parent class
        super().__init__(traj_pub=traj_pub, logger=logger.get_child("rp_interface"), verbose=verbose)

        # set scenario
        self.scenario = scenario

        # set road boundary
        self._road_boundary = road_boundary

        # create reactive planner config
        rp_config = ReactivePlannerConfiguration().load(rp_interface_params.path_rp_config)
        rp_config.update(scenario=self.scenario, planning_problem=planning_problem)

        # overwrite time step and horizon
        rp_config.planning.dt = dt
        rp_config.planning.planning_horizon = horizon
        rp_config.planning.time_steps_computation = int(horizon/dt)

        # overwrite vehicle params in planner config
        rp_config.vehicle.length = ego_vehicle_handler.vehicle_length
        rp_config.vehicle.width = ego_vehicle_handler.vehicle_width
        rp_config.vehicle.wheelbase = ego_vehicle_handler.vehicle_wheelbase
        rp_config.vehicle.wb_rear_axle = ego_vehicle_handler.vehicle_wb_rear_axle
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

        # init trajectory planner
        self._planner: ReactivePlanner = reactive_planner

    def plan(self, current_state: EgoVehicleState, goal, reference_velocity=None):
        """Overrides plan method from base class and calls the planning algorithm of the reactive planner"""
        # set reference velocity for planner
        self._planner.set_desired_velocity(desired_velocity=reference_velocity, current_speed=current_state.velocity)

        # update collision checker (self.scenario is updated continuously as it is a reference to the scenario handler)
        self._planner.set_collision_checker(self.scenario, road_boundary_obstacle=self._road_boundary)

        # reset planner state
        if not hasattr(current_state, "acceleration"):
            # current_state uses acceleration localization (see ego_vehicle_handler)
            current_state.acceleration = 0.0
        x0_planner_cart = ReactivePlannerState()
        x0_planner_cart = current_state.convert_state_to_state(x0_planner_cart)
        self._planner.reset(initial_state_cart=x0_planner_cart,
                            initial_state_curv=None,
                            collision_checker=self._planner.collision_checker,
                            coordinate_system=self._planner.coordinate_system)

        # call plan function and generate trajectory
        optimal_traj = self._planner.plan()

        # check if valid trajectory is found
        if optimal_traj:
            # add to planned trajectory
            self._cr_state_list = optimal_traj[0].state_list

            # record planned state and input TODO check this
            self._planner.record_state_and_input(optimal_traj[0].state_list[1])
        else:
            # TODO: sample emergency brake trajectory if no trajectory is found
            self._cr_state_list = None

    def update(self, planning_problem: PlanningProblem = None, route: Route = None, reference_path: np.ndarray = None):
        """
        Updates externals of the trajectory planner
        """
        # set planning problem if provided
        if planning_problem is not None:
            self._planner.config.planning_problem = planning_problem
        # set new reference path for planner if provided
        if reference_path is not None:
            rp_coordinate_system = CoordinateSystem(reference=reference_path, smooth_reference=False)
            self._planner.set_reference_path(coordinate_system=rp_coordinate_system)
