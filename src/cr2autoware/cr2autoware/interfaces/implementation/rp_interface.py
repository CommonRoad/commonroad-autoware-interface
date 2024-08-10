# third party imports
import numpy as np
from typing import List, Set, Tuple

# commonroad imports
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import (
    Occupancy
)

# commonroad-dc
import commonroad_dc.pycrcc as pycrcc

# commonroad-rp imports
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.reactive_planner import ReactivePlanner

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

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger


class ReactivePlannerInterface(TrajectoryPlannerInterface):
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

        # create reactive planner config
        rp_config = ReactivePlannerConfiguration().load(rp_interface_params.path_rp_config)
        rp_config.update(scenario=self.scenario, planning_problem=planning_problem)

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

        # initialize reactive planner object
        reactive_planner: ReactivePlanner = ReactivePlanner(rp_config)

        # adjust sampling settings from ROS params
        reactive_planner.set_t_sampling_parameters(t_min=rp_interface_params.get_ros_param("t_min"))
        reactive_planner.set_d_sampling_parameters(delta_d_min=rp_interface_params.get_ros_param("d_min"),
                                                   delta_d_max=rp_interface_params.get_ros_param("d_max"))

        # init trajectory planner
        self._planner: ReactivePlanner = reactive_planner

    def _plan(self, init_state: EgoVehicleState, goal, reference_velocity=None, **kwargs) -> None:
        """
        Implements _plan method from base class and calls the algorithm of the reactive planner.

        :param init_state: current state of the ego vehicle
        :param goal: goal state of the ego vehicle
        :param reference_velocity: reference velocity for the planner
        :param kwargs: additional keyword arguments
        """
        # check for narrow passage scenario

        # get lanelets for the current position
        lanelets: List[Lanelet] = []
        lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([current_state.position])
        for lanelet_id in lanelet_ids:
            lanelets.append(self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0]))
        self._logger.info(f"Initial lanelets: {lanelet_ids}")

        # get lanelets for future position in the defined horizon
        # TODO: use rp planning horizon instead of fixed horizon
        horizon: int = 5
        for i in range(1, horizon):
            # TODO: use trajectory prediction to get future position (curren_state.position global frame, current implementation is for vehicle frame)
            # future position leads to wrong global position!!!!
            future_position = [current_state.position[0] + i * current_state.velocity, current_state.position[1]]
            self._logger.info(f"Future position: {future_position}")
            future_lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([future_position])
            self._logger.info(f"Future lanelets: {future_lanelet_ids}")
            for lanelet_id in future_lanelet_ids:
                self._logger.info(f"Future lanelet: {lanelet_id}")
                # check if lanlet_id list has elements (future position is in a lanelet)
                if lanelet_id:
                    future_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0])
                    if future_lanelet and future_lanelet not in lanelets:
                        lanelets.append(future_lanelet)
                        self._logger.info(f"added: {lanelet_id}")

        self._logger.info(f"Lanelets in the horizon: {lanelets}")

        ### Not tested yet: 

        # for the lanelet list check for obstacles and possible blockades
        for lanelet in lanelets:
            # get all obstacles in the lanelet
            self._logger.info(f"Current lanelet: {lanelet}")
            obstacles: Set = set()
            obstacles.update(lanelet.dynamic_obstacles_on_lanelet)
            obstacles.update(lanelet.static_obstacles_on_lanelet)
            self._logger.info(f"Obstacles in {lanelet.lanelet_id}: {obstacles}")

            y_blockades: List[Occupancy] = []
            for obstacle in obstacles:
                occupancy = obstacle.occupancy_at_time(self.scenario.time)
                
                self._logger.info("OCCUPANCY SHAPE: " + str(occupancy.shape))

        # set reference velocity for planner
        self._planner.set_desired_velocity(desired_velocity=reference_velocity, current_speed=init_state.velocity)

        # update collision checker (self.scenario is updated continuously as it is a reference to the scenario handler)
        self._planner.set_collision_checker(self.scenario, road_boundary_obstacle=self._road_boundary)

        # reset planner state
        if not hasattr(init_state, "acceleration"):
            # current_state uses acceleration localization (see ego_vehicle_handler)
            init_state.acceleration = 0.0
        x0_planner_cart: ReactivePlannerState = ReactivePlannerState()
        x0_planner_cart = init_state.convert_state_to_state(x0_planner_cart)
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

            # update previously planned trajectory
            self._prev_state_list = optimal_traj[0].state_list

            # record planned state and input
            self._planner.record_state_and_input(optimal_traj[0].state_list[1])
        else:
            # TODO: sample emergency brake trajectory if no trajectory is found ?
            self._cr_state_list = None
            self._prev_state_list = None

    def update(self, planning_problem: PlanningProblem = None, reference_path: np.ndarray = None) -> None:
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
            rp_coordinate_system = CoordinateSystem(reference=reference_path, smooth_reference=False)
            self._planner.set_reference_path(coordinate_system=rp_coordinate_system)
