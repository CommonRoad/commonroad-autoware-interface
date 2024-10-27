# third party imports
import numpy as np
from typing import List, Set, Tuple, Optional
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point as PointShapely, MultiPolygon, LineString
import time

# commonroad imports
from commonroad.geometry.shape import Rectangle, Polygon, Circle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import TraceState
from commonroad.scenario.lanelet import Lanelet
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import Occupancy

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
from cr2autoware.common.utils.transform import utm2map
from cr2autoware.handlers.scenario_handler import ScenarioHandler
from cr2autoware.interfaces.base.trajectory_planner_interface import TrajectoryPlannerInterface

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time

# ROS messages
from geometry_msgs.msg import Point as PointMsg


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
                 scenario_handler: ScenarioHandler,
                 dt: float,
                 traj_planner_params: TrajectoryPlannerParams,
                 rp_interface_params: RPInterfaceParams,
                 ego_vehicle_handler: EgoVehicleHandler,
                 narrow_passage_obstacles_pub: Publisher,
                 narrow_passage_clearance_pub: Publisher):
        """
        Constructor for ReactivePlannerInterface class.

        :param traj_pub: ROS2 node publisher for trajectory
        :param logger: ROS2 node logger
        :param verbose: Flag for verbose logging
        :param scenario: CommonRoad scenario
        :param planning_problem: CommonRoad planning problem
        :param scenario_handler: CommonRoad scenario handler
        :param dt: time step for the reactive planner
        :param traj_planner_params: General Trajectory Planner parameters
        :param rp_interface_params: Reactive Planner Interface parameters
        :param ego_vehicle_handler: Ego Vehicle Handler
        :var external_velocity_limit: External velocity limit
        :var narrow_passage_obstacles_pub: ROS2 node publisher for narrow passage obstacles
        :var narrow_passage_clearance_pub: ROS2 node publisher for narrow passage clearance
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

        # set scenario handler
        self.scenario_handler = scenario_handler

        # set road boundary
        self._road_boundary: pycrcc.CollisionObject = self.scenario_handler.road_boundary

        # set narrow passage publishers
        self._narrow_passage_obstacles_pub = narrow_passage_obstacles_pub
        self._narrow_passage_clearance_pub = narrow_passage_clearance_pub

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
        # if optimal trajectory is found, check for narrow passage
        if self._cr_state_list:
            # function to set max velocity for narrow passage
            reference_velocity = self.narrow_passage_velocity_function(current_state, self._cr_state_list, reference_velocity, **kwargs)

        else:
            self._logger.debug("No optimal trajectory found. Narrow passage check skipped!")

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

    def narrow_passage_velocity_function(self, current_state: EgoVehicleHandler, cr_state_list: Optional[List[TraceState]], reference_velocity, **kwargs) -> float:
        """
        Check for narrow passages in the scenario and adjust the reference velocity.

        This function searches for all relevant lanlets in the scenario. A relevant lanelet is a lanelet that is
        on the current position of the ego vehicle, on a position of the optimal trajectory, or an adjacent lanelet to
        these lanelets. All obstacles on these lanelets are merged to a combined occupancy polygon. Also, the reference
        path is filtered to include only a look ahead distance of the vehicle. The distance between the filtered 
        reference path to the combined occupancy is calculated. Depending on this distance, the reference velocity
        is adjusted.

        :param current_state: current state of the ego vehicle
        :param cr_state_list: list of states in the optimal trajectory
        :param reference_velocity: reference velocity for the planner
        :return: adjusted reference velocity
        """
        t_start = time.perf_counter()
        if reference_velocity is None:
            return None
                        
        # initialize narrow passage radius
        narrow_passage_radius = 1000.0

        previous_lanelet_ids = None
        combined_polygon = None
        relevant_lanelets = set()

        # Get nearest point in the reference path to the current vehicle position
        current_position = np.array(current_state.position)
        reference_path = np.array(self._planner.reference_path)
        distances = np.linalg.norm(reference_path - current_position, axis=1)
        nearest_index = np.argmin(distances)

        # Filter reference_path to include only points after the nearest point
        filtered_reference_path = self._planner.reference_path[nearest_index:]
        positions = [filtered_reference_path[0]]
        combined_distance: float = 0.0
        # calculate reaction distance, a look ahead distance for the vehicle to react to obstacles
        if current_state.velocity > 5.0:
            look_ahead_distance = current_state.velocity * 4.0
        else:
            look_ahead_distance = 20.0

        for point in range(1, len(filtered_reference_path)):
            point_distance = np.linalg.norm(filtered_reference_path[point] - filtered_reference_path[point - 1])
            combined_distance += point_distance
            if combined_distance < look_ahead_distance:
                positions.append(filtered_reference_path[point])

        #self._logger.debug(f"Positions: {positions}")
        for position in positions:         
            # TODO: check distance between two positions and if the distance is too large add intermediate points
            # Get lanelet ids for the current position
            lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position([position])

            for lanelet_id in lanelet_ids:

                # If the lanelet ids are the same as the previous ones, skip the processing and check the next position
                if lanelet_ids == previous_lanelet_ids:
                    break
                
                # Update previous lanelet ids and relevant lanelets
                previous_lanelet_ids = lanelet_ids

                # Collect all relevant lanelets
                lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0])
                relevant_lanelets.add(lanelet)
                if lanelet.adj_left is not None:
                    left_adjacent_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                    relevant_lanelets.add(left_adjacent_lanelet)
                if lanelet.adj_right is not None:
                    right_adjacent_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                    relevant_lanelets.add(right_adjacent_lanelet)
            
        # Merge obstacle sets from the relevant lanelets
        combined_obstacle_set = set()
        for lanelet in relevant_lanelets:
            # only consider obstacles on the first timestep
            # TODO: Check if all obstacles are removed if they are not on the lanelet anymore
            if lanelet.dynamic_obstacles_on_lanelet:
                _, obstacle_set = next(iter(lanelet.dynamic_obstacles_on_lanelet.items()))
                combined_obstacle_set.update(obstacle_set)
    
        # Calculate the combined occupancy polygon for all obstacles on the relevant lanelets
        combined_polygon = None    
        if combined_obstacle_set:
            for obstacle_id in combined_obstacle_set:
                obstacle = self.scenario.obstacle_by_id(obstacle_id)
                if obstacle is not None:
                    occupancy = obstacle.occupancy_at_time(0)
                    
                    # Convert occupancy to polygon
                    if isinstance(occupancy, Occupancy):
                        shape = occupancy.shape
                        occupancy_polygon = None
                        if isinstance(shape, Rectangle):
                            occupancy_polygon = occupancy.shape._shapely_polygon
                        elif isinstance(shape, Polygon):
                            occupancy_polygon = occupancy
                        elif isinstance(shape, Circle):
                            occupancy_polygon = occupancy.shape.shapely_object
                        else:
                            raise TypeError("Unsupported CommonRoad shape type: " + str(shape))
                        
                        # the union function returns a Polygon object if only one polygon is in the MultiPolygon
                        # convert it to a MultiPolygon object
                        if combined_polygon is None:
                            combined_polygon = MultiPolygon([occupancy_polygon])
                        else:
                            combined_polygon = combined_polygon.union(occupancy_polygon)

        if combined_polygon is not None:
            for position in positions:
                # Calculate the distance to the combined polygon
                position_point = PointShapely(position)
                radius = position_point.distance(combined_polygon)
                
                # Update maximum radius
                if radius < narrow_passage_radius:
                    narrow_passage_radius = radius

            # get external velocity limits
            external_velocity_limit_max = kwargs.get("external_velocity_limit_max")
            external_velocity_limit_min = kwargs.get("external_velocity_limit_min")
            #TODO: kwarg external_velocity_min is also used for set goal velocity; new vehicle parameter for min velocity necessary
            external_velocity_limit_min = 2.0
            # initialize minimum and maximum radius of narrow passages
            # minmimal radius is the width of the vehicle
            min_radius = self._planner.vehicle_params.width * 0.5 
            # maximal radius is the double width of the vehicle
            max_radius = self._planner.vehicle_params.width
            
            # set proposed reference velocity based on the narrow passage radius
            if narrow_passage_radius < min_radius:
                # narrow passage is smaller than the width of the vehicle, set reference velocity to minimum
                proposed_reference_velocity = external_velocity_limit_min
            elif narrow_passage_radius > max_radius:
                # narrow passage is larger than the double width of the vehicle, set reference velocity to maximum
                proposed_reference_velocity = external_velocity_limit_max
            else:
                # narrow passage is between the width and double width of the vehicle
                # calculate normalized radius and use a quadratic function for velocity adjustment
                normalized_radius = (narrow_passage_radius - min_radius) / (max_radius - min_radius)
                proposed_reference_velocity = external_velocity_limit_min + (external_velocity_limit_max - external_velocity_limit_min) * (normalized_radius)**2
            
            reference_velocity = min(reference_velocity, proposed_reference_velocity)            
            self._logger.debug(f"Reference velocity: {reference_velocity*3.6} km/h")
        t_end = time.perf_counter()
        self._logger.debug(f"Time for narrow passage velocity function: {t_end - t_start}")

        # Debugging: Publish obstacles, clearance and trajectory  
        self.publish_obstacles(combined_polygon)
        self.publish_clearance(positions)

        return reference_velocity
    
    def publish_obstacles(self, multipolygon: MultiPolygon):
        """
        Publishes the obstacles in the narrow passage scenario to the ROS2 node.

        :param multipolygon: MultiPolygon of the obstacles in the narrow passage scenario
        """
        marker_array = MarkerArray()

        if multipolygon is not None:
            if isinstance(multipolygon, Polygon):
                polygons = [multipolygon]
            elif isinstance(multipolygon, MultiPolygon):
                polygons = multipolygon.geoms
            else:
                self._logger.error("Unsupported geometry type for multipolygon")
                return

            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            
            for i, polygon in enumerate(polygons):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = Time().to_msg()
                marker.ns = "combined_polygons"
                marker.id = i
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0 
                marker.color.b = 0.0

                # Add points of the polygon to the marker
                for x, y in polygon.exterior.coords:
                    p = utm2map(self.scenario_handler.origin_transformation, [x, y])
                    p.z = self.scenario_handler.z_coordinate
                    marker.points.append(p)

                # Add first point again to close the polygon
                if len(polygon.exterior.coords) > 0:
                    first_point = polygon.exterior.coords[0]
                    p = utm2map(self.scenario_handler.origin_transformation, [first_point[0], first_point[1]])
                    p.z = self.scenario_handler.z_coordinate
                    marker.points.append(p)

                marker_array.markers.append(marker)
                
        else: 
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        self._narrow_passage_obstacles_pub.publish(marker_array)

    def publish_clearance(self, positions: List[Tuple[float, float]]):
        """
        Publishes the clearance of the narrow passage scenario to the ROS2 node.

        :param position: list of positions of the ego vehicle and the optimal trajectory
        """
        marker_array = MarkerArray()

        if positions is not None:
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
            for i, position in enumerate(positions):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = Time().to_msg()
                marker.ns = "clearance"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position = PointMsg()
                p = utm2map(self.scenario_handler.origin_transformation, [position[0], position[1]])
                marker.pose.position.x = p.x
                marker.pose.position.y = p.y
                marker.pose.position.z = self.scenario_handler.z_coordinate - 0.1
                marker.scale.x = 0.25
                marker.scale.y = 0.25
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
        else: 
            marker = Marker()
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)

        self._narrow_passage_clearance_pub.publish(marker_array)
