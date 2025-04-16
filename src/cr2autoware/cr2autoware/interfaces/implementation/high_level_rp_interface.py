import time
from typing import List, Tuple, Optional

# third party imports
import numpy as np

# commonroad imports
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

# commonroad-dc
import commonroad_dc.pycrcc as pycrcc

# commonroad-rp imports
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import create_coordinate_system
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.high_level_planner import HighLevelPlanner

# cr2autoware
from cr2autoware.common.configuration import (
    RPInterfaceParams,
    TrajectoryPlannerParams
)
from cr2autoware.handlers.ego_vehicle_handler import (
    EgoVehicleHandler,
    EgoVehicleState
)
from cr2autoware.interfaces.base.trajectory_planner_interface import TrajectoryPlannerInterface
from cr2autoware.interfaces.implementation.rp_helper.world_updater import WorldUpdater
from cr2autoware.common.utils.trajectory_utils import _lerp

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger


class HighLevelReactivePlannerInterface(TrajectoryPlannerInterface):
    """
    Trajectory planner interface for the CommonRoad Reactive Planner.

    This class implements the abstract methods of the base class TrajectoryPlannerInterface.

    :var scenario: reference to the scenario
    :var _road_boundary: reference to the road boundary as a collision object
    :var _planner: reference to the reactive planner
    """
    def __init__(self, traj_pub: Publisher,
                 logger: RcutilsLogger,
                 verbose: bool,
                 scenario: Scenario,
                 planning_problem: PlanningProblem,
                 road_boundary: pycrcc.CollisionObject,
                 dt: float,
                 traj_planner_params: TrajectoryPlannerParams,
                 rp_interface_params: RPInterfaceParams,
                 ego_vehicle_handler: EgoVehicleHandler):
        """
        Constructor for ReactivePlannerInterface class.

        :param traj_pub: ROS2 node publisher for trajectory
        :param logger: ROS2 node logger
        :param verbose: Flag for verbose logging
        :param scenario: CommonRoad scenario
        :param planning_problem: CommonRoad planning problem
        :param road_boundary: road boundary as a collision object
        :param dt: time step for the reactive planner
        :param traj_planner_params: General Trajectory Planner parameters
        :param rp_interface_params: Reactive Planner Interface parameters
        :param ego_vehicle_handler: Ego Vehicle Handler
        """

        # init parent class
        super().__init__(
            traj_pub=traj_pub,
            traj_planner_params=traj_planner_params,
            logger=logger.get_child("rp_interface"),
            verbose=verbose
        )

        # set scenario
        self.scenario = scenario

        # set road boundary
        self._road_boundary = road_boundary

        # store previous CVLN long and lat trajectories for replanning
        self._prev_lon_traj: Optional[List[Tuple]] = None
        self._prev_lat_traj: Optional[List[Tuple]] = None

        # create reactive planner config
        rp_config = ReactivePlannerConfiguration().load(rp_interface_params.path_rp_config)
        
        # init world updater
        self._world_updater = WorldUpdater(self.scenario, logger=self._logger, world_parameters=rp_config.create_world_params(dt=self.scenario.dt))

        # update config with scenario and planning problem
        rp_config.update(scenario=self.scenario, planning_problem=planning_problem, world=self._world_updater.world)

        # overwrite time step and horizon
        rp_config.planning.dt = dt
        rp_config.planning.planning_horizon = traj_planner_params.planning_horizon
        rp_config.planning.time_steps_computation = int(traj_planner_params.planning_horizon/dt)

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

        # initialize high level planner object
        hl_planner = HighLevelPlanner(rp_config)

        # adjust sampling settings from ROS params
        hl_planner.set_t_sampling_parameters(t_min=rp_interface_params.get_ros_param("t_min"))
        hl_planner.set_d_sampling_parameters(delta_d_min=rp_interface_params.get_ros_param("d_min"),
                                                   delta_d_max=rp_interface_params.get_ros_param("d_max"))
        
        hl_planner.ros_logger = self._logger
        hl_planner._planner.ros_logger = self._logger

        # init trajectory planner
        self._planner: HighLevelPlanner = hl_planner

    def _plan(self, init_state: EgoVehicleState, goal, reference_velocity=None, **kwargs) -> None:
        """
        Implements _plan method from base class and calls the algorithm of the reactive planner.

        :param init_state: current state of the ego vehicle
        :param goal: goal state of the ego vehicle
        :param reference_velocity: reference velocity for the planner
        :param kwargs: additional keyword arguments
        """
        # set reference velocity for planner
        self._planner.set_desired_velocity(desired_velocity=reference_velocity, current_speed=init_state.velocity)

        # update collision checker (self.scenario is updated continuously as it is a reference to the scenario handler)
        self._planner.set_collision_checker(self.scenario, road_boundary_obstacle=self._road_boundary)

        # update obstacles in C++ World
        self._world_updater.scenario_updated()
        num_obs = len(self._planner.config.rule_monitor.get_world().obstacles)
        self._logger.info(f"Number of obstacles in C++ world: {num_obs}")

        # reset stored trace of monitor
        self._planner.config.rule_monitor.reset_trace()

        # Cartesian initial state
        if not hasattr(init_state, "acceleration"):
            # current_state uses acceleration localization (see ego_vehicle_handler)
            init_state.acceleration = 0.0
        x0_planner_cart: ReactivePlannerState = ReactivePlannerState()
        x0_planner_cart = init_state.convert_state_to_state(x0_planner_cart)

        # replan from Cartesian state
        if self._prev_lon_traj is None and self._prev_lat_traj is None:
            self._logger.info("Replanning from Cartesian initial state.")
            self._planner.reset(
                initial_state_cart=x0_planner_cart,
                initial_state_curv=None,
                collision_checker=self._planner.collision_checker,
                coordinate_system=self._planner.coordinate_system
                )
        # replan from CVLN state
        else:
            self._logger.info("Replanning from CVLN lon/lat initial state.")
            x_0_planner_lon, x_0_planner_lat = self._calc_cvln_init_state(init_pos_cart=init_state.position)
            self._planner.reset(
                initial_state_cart=x0_planner_cart,
                initial_state_curv=(x_0_planner_lon, x_0_planner_lat),
                collision_checker=self._planner.collision_checker,
                coordinate_system=self._planner.coordinate_system
            )

        # call plan function and generate trajectory
        tic = time.perf_counter()
        optimal_traj = self._planner.plan()
        toc = time.perf_counter()
        self._logger.debug(f"Planning time: {(toc - tic) * 1000:.2f} ms")

        self._logger.info("===== Rejected Trajectories =====")
        self._logger.info(f"Rejected {self._planner.infeasible_count_kinematics} infeasible trajectories due to kinematics")
        for constraint in self._planner.config.planning.constraints_to_check:
            self._logger.info(f"\tInfeasible {constraint}: {self._planner.infeasible_reason_dict[constraint]}")
        self._logger.info(f"Rejected {self._planner.infeasible_count_collision} infeasible trajectories due to collisions")
        self._logger.info(f"Rejected {self._planner.infeasible_count_rules} infeasible trajectories due to rule violations")
        self._logger.info("===== End Rejected Trajectories =====")

        # check if valid trajectory is found
        if optimal_traj:
            # add to planned trajectory
            self._cr_state_list = optimal_traj[0].state_list

            # update previously planned trajectory
            self._prev_state_list = optimal_traj[0].state_list

            # record planned state and input
            self._planner.record_state_and_input(optimal_traj[0].state_list[1])

            # update previously planned CVLN long and lat trajectories (TODO: check indexing!)
            self._prev_lon_traj = optimal_traj[1]
            self._prev_lat_traj = optimal_traj[2]
        else:
            # TODO: sample emergency brake trajectory if no trajectory is found?
            self._logger.warning("Reactive planner could not find a feasible trajectory!")
            self._cr_state_list = None
            self._prev_state_list = None
            self._prev_lon_traj = None
            self._prev_lat_traj = None

    def _calc_cvln_init_state(self, init_pos_cart: np.ndarray) -> Tuple[List, List]:
        """
        Calculates initial longitudinal state (s, s_dot, s_ddot) and lateral state (d _d_dot, d_ddot) based on Cartesian
        initial position and previously planned long and lat trajectories.
        
        :param init_pos_cart: Position from Cartesian initial state
        """
        # initial long, lat position
        s_0, d_0 = self._planner.coordinate_system.convert_to_curvilinear_coords(init_pos_cart[0], init_pos_cart[1])

        # get closest time idx (based on long position)
        s_array = np.array([lon_state[0] for lon_state in self._prev_lon_traj])
        closest_idx = np.argmin(np.abs(s_array - s_0))

        x_0_lon_interp = self._prev_lon_traj[closest_idx]
        x_0_lat_interp = self._prev_lat_traj[closest_idx]

        return x_0_lon_interp, x_0_lat_interp
    
    @staticmethod
    def _get_interpolated_state(curr_state, next_state, interp_ratio):
        """
        Interpolates state between two given CVLN (lon or lat states) with given ratio
        """
        pos_interp = _lerp(curr_state[0], next_state[0], interp_ratio)
        vel_interp = _lerp(curr_state[1], next_state[1], interp_ratio)
        acc_interp = _lerp(curr_state[2], next_state[2], interp_ratio)

        return [pos_interp, vel_interp, acc_interp]

    def update(self, planning_problem: PlanningProblem = None, reference_path: np.ndarray = None, route_lanelet_ids: List[int] = None) -> None:
        """
        Updates externals of the trajectory planner.

        :param planning_problem: planning problem
        :param reference_path: reference path
        """
        # set planning problem if provided
        if planning_problem is not None:
            self._planner.config.planning_problem = planning_problem
        # set new reference path for planner if provided
        if reference_path is not None:
            assert route_lanelet_ids is not None, "Reference path given but no route lanelet IDs"
            rp_coordinate_system = create_coordinate_system(reference_path)
            self._planner.set_reference_path(rp_coordinate_system, route_lanelet_ids)
