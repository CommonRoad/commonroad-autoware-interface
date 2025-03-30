from typing import Tuple, TYPE_CHECKING

# third party imports
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

# commonroad imports
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

# commonroad-dc
import commonroad_dc.pycrcc as pycrcc
import commonroad_dc.pycrccosy as pycrccosy

# commonroad-rp imports
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.reactive_planner import ReactivePlanner

# commonroad-reach-flow imports
import cr_reach_flow.cr_reach_flow_core as reach_core
from cr_reach_flow.visualization.scenario import convert_to_cartesian_polygons

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
from cr2autoware.common.utils.cr_conversion_utils import commonroad_polygons_to_marker
from cr2autoware.common.ros_interface.create import create_publisher
from cr2autoware.common.ros_interface.specs_publisher import spec_reach_debug
if TYPE_CHECKING:
    from cr2autoware import Cr2Auto

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger


class ReactivePlannerReachInterface(TrajectoryPlannerInterface):
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
                 ego_vehicle_handler: EgoVehicleHandler,
                 node: "Cr2Auto"):
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
        
        self._propagation_layer = reach_core.layers.propagation.PointMassPropagator(dt, self._create_point_mass_params(ego_vehicle_handler))
        self._repartition_layer = reach_core.layers.repartition.PositionRepartitioner()
        self._post = self._create_post(ego_vehicle_handler)

        self._node = node
        self._reach_pub = create_publisher(node, spec_reach_debug)
        self._rear_wb = ego_vehicle_handler.vehicle_wb_rear_axle

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
        # set reference velocity for planner
        self._planner.set_desired_velocity(desired_velocity=reference_velocity, current_speed=init_state.velocity)

        # update collision checker (self.scenario is updated continuously as it is a reference to the scenario handler)
        self._planner.set_collision_checker(self.scenario, road_boundary_obstacle=self._road_boundary)

        if not hasattr(init_state, "acceleration"):
            # current_state uses acceleration localization (see ego_vehicle_handler)
            init_state.acceleration = 0.0

        self._logger.debug("Starting reachability analysis")
        initial_uncertainty = 0.01
        ccs = self._planner.coordinate_system.ccosy
        init = reach_core.initializers.base_set.CurvilinearUncertaintyInitializer(ccs, *([initial_uncertainty] * 4))
        layers = [
            self._propagation_layer,
            # reach_core.layers.collision.CollisionFilter(self._planner.collision_checker),
            self._repartition_layer,
        ]
        layer = reach_core.layers.meta.Sequential(layers)
        reach = reach_core.executors.DynamicReachabilityAnalysis(0, self._planner.config.planning.time_steps_computation, init, layer, self._post)
        reach.initialize(*self._initialize(init_state))
        reach.run_to_next_goal()
        g = reach.reach_graph
        self._logger.debug(f"Before post {g.num_nodes}")
        reach_graph = reach.get_post_processed_reach_graph()
        self._logger.debug(f"Size of reach graph {reach_graph.num_nodes}")
        comp_graph = reach_core.graphs.DynamicComponentGraph(reach_graph)
        dc_extractor = reach_core.driving_corridor.DynamicDrivingCorridorExtractor()
        corridors = dc_extractor.extract(comp_graph, max_corridors=1)
        time_stamp = self._node.get_clock().now().to_msg()
        z = self._node.scenario_handler.z_coordinate
        origin = self._node.origin_transformation
        if len(corridors) > 0:
            self._logger.debug("Found driving corridor")
            corridor: reach_core.driving_corridor.DynamicDrivingCorridor = corridors[0]
            corridor_graph: reach_core.graphs.DynamicReachGraph = corridor.reach_graph
            markers = MarkerArray()
            init_step = corridor_graph.initial_step
            final_step = corridor_graph.final_step + 1
            for step in range(init_step, final_step):
                cart_polygons = []
                for node in corridor_graph.get_nodes_at_step(step):
                    drivable_area = node.set.position_rectangle.bounds
                    cart_polygons += convert_to_cartesian_polygons(drivable_area, ccs, split_wrt_angle=True)
                marker = commonroad_polygons_to_marker(cart_polygons, origin, z, time_stamp)
                marker.id = step
                marker.ns = "reachable_set"
                marker.color.a = (final_step - step) / (final_step - init_step) * 0.7 + 0.3
                markers.markers.append(marker)
            self._reach_pub.publish(markers)
        else:
            self._logger.debug("No driving corridor found")

        # reset planner state
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

    def _initialize(
        self,
        init_state: EgoVehicleState
    ) -> Tuple[int, float, float, float, float, float]:
        """Create arguments to initialize an executor from an ego vehicle state."""
        theta = init_state.orientation
        x = init_state.position[0] + np.cos(theta) * self._rear_wb
        y = init_state.position[1] + np.sin(theta) * self._rear_wb
        # We always start at time step 0 here
        return 0, x, y, init_state.velocity, init_state.acceleration, init_state.orientation


    @staticmethod
    def _create_point_mass_params(ego_vehicle_handler: EgoVehicleHandler) -> reach_core.layers.propagation.PointMassParameters:
        point_mass_params = reach_core.layers.propagation.PointMassParameters()
        point_mass_params.a_lon_min = -ego_vehicle_handler.vehicle_max_acceleration * 0.2
        point_mass_params.a_lon_max = ego_vehicle_handler.vehicle_max_acceleration * 0.2
        point_mass_params.a_lat_min = -2.0 * 0.2
        point_mass_params.a_lat_max = 2.0 * 0.2
        point_mass_params.v_lon_min = 0.0
        point_mass_params.v_lon_max = 10.0
        point_mass_params.v_lat_min = -4.0
        point_mass_params.v_lat_max = 4.0

        return point_mass_params
    
    @staticmethod
    def _create_post(ego_vehicle_handler: EgoVehicleHandler) -> reach_core.post_processors.PostProcessor:
        post = [
            reach_core.post_processors.pruning.DanglingNodePruner(),
            reach_core.post_processors.CenterToRearShifter(ego_vehicle_handler.vehicle_wb_rear_axle)
        ]
        return reach_core.post_processors.meta.Sequential(post)

    @staticmethod
    def _create_layers(dt: float, ego_vehicle_handler: EgoVehicleHandler) -> Tuple[reach_core.layers.Layer, reach_core.post_processors.PostProcessor]:
        layers = [
            reach_core.layers.propagation.PointMassPropagator(dt, ReactivePlannerReachInterface._create_point_mass_params(ego_vehicle_handler)),
            # reach_core.layers.collision.CollisionFilter(self.collision_checker),
            reach_core.layers.repartition.PositionRepartitioner(),
        ]
        post = [
            reach_core.post_processors.pruning.DanglingNodePruner(),
            reach_core.post_processors.CenterToRearShifter(ego_vehicle_handler.vehicle_wb_rear_axle)
        ]
        return reach_core.layers.meta.Sequential(layers), reach_core.post_processors.meta.Sequential(post)
