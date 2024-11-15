# third party imports
import numpy as np
from typing import List, Set, Tuple, Optional
from visualization_msgs.msg import Marker, MarkerArray
from shapely.geometry import Point, Polygon, MultiPolygon, LineString
from scipy.spatial import cKDTree
import time

# commonroad imports
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
                 lateral_clearance_obstacles_pub: Publisher,
                 lateral_clearance_pub: Publisher):
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
        :var lateral_clearance_obstacles_pub: ROS2 node publisher for lateral clearance obstacles
        :var lateral_clearance_pub: ROS2 node publisher for lateral clearance clearance
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

        # set lateral clearance publishers
        self._lateral_clearance_obstacles_pub = lateral_clearance_obstacles_pub
        self._lateral_clearance_pub = lateral_clearance_pub

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
        # check for lateral distance scenario
        # if optimal trajectory is found, check lateral distance
        if self._cr_state_list:
            # function to set max velocity for lateral distance
            reference_velocity = self.reference_velocity_based_on_lateral_clearance(current_state, self._cr_state_list, reference_velocity, **kwargs)

        else:
            self._logger.debug("No optimal trajectory found. Lateral distance check skipped!")

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

    def reference_velocity_based_on_lateral_clearance(self, current_state: EgoVehicleHandler, cr_state_list: Optional[List[TraceState]], reference_velocity: float, **kwargs) -> float:
        """
        Check lateral clearance in the scenario and adjust the reference velocity.

        All static obstacles and dynamic obstacles on relevant lanelets, with different orientations, and within a limited time to the reference path are merged into 
        a combined occupancy polygon. The distance between the filtered reference path and the combined occupancy polygon is calculated. Based on this distance, 
        the reference velocity is adjusted.

        :param current_state: current state of the ego vehicle
        :param cr_state_list: list of states in the optimal trajectory
        :param reference_velocity: reference velocity for the planner
        :return: adjusted reference velocity based on the lateral clearance
        """
        t_start = time.perf_counter()
        if reference_velocity is None:
            return None
        
        # initialize parameters for lateral clearance
        # minimal lateral clearance radius in meters
        min_lateral_clearance: float = float('inf') 
        # minimal velocity (in m/s) for dynamic obstacles, otherwise they are considered as static obstacles
        dynamic_velocity_threshold: float = kwargs.get("dynamic_velocity_threshold")
        # look ahead time (in seconds) for the vehicle to react to obstacles
        look_ahead_time: float = kwargs.get("look_ahead_time")
        # minimal look ahead distance (in meters) for the vehicle to react to obstacles
        min_look_ahead_distance: float = kwargs.get("min_look_ahead_distance")
        # maximal time step for prediction of dynamic obstacles
        max_time_step: int = int(look_ahead_time / self.scenario.dt)
        # time threshold (in seconds) for intersection of trajectories of ego vehicle and dynamic obstacles
        time_threshold: float = kwargs.get("time_threshold")
        # get velocity limits in m/s
        max_reference_velocity: float = kwargs.get("max_reference_velocity")
        min_reference_velocity: float = kwargs.get("min_reference_velocity")
        # initialize minimum and safe distance (radius) for lateral clearance in meters
        min_distance: float = self._planner.vehicle_params.width * 0.5 
        safe_distance: float = self._planner.vehicle_params.width
        # flag to publish lateral clearance topics
        publish_lateral_clearance_topics: bool = kwargs.get("publish_lateral_clearance_topics")

        # Get nearest point in the reference path to the current vehicle position
        reference_path = np.array(self._planner.reference_path)
        current_position = np.array(current_state.position)

        ref_path_tree = cKDTree(reference_path)
        _, nearest_index = ref_path_tree.query(current_position)

        # Filter reference_path to include only points after the nearest point
        # -1 to get the point before the nearest point for tangent calculation
        filtered_reference_path = self._planner.reference_path[(nearest_index-1):]
        positions = np.array(filtered_reference_path[0])
        combined_distance: float = 0.0
        # calculate reaction distance, a look ahead distance for the vehicle to react to obstacles
        look_ahead_distance = current_state.velocity * look_ahead_time
        if look_ahead_distance < min_look_ahead_distance:
            look_ahead_distance = min_look_ahead_distance

        for point in range(1, len(filtered_reference_path)):
            point_distance = np.linalg.norm(filtered_reference_path[point] - filtered_reference_path[point - 1])
            combined_distance += point_distance
            if combined_distance < look_ahead_distance:
                positions = np.vstack([positions, filtered_reference_path[point]])
            else:
                # add the last point to the positions for the orientation calculation
                positions = np.vstack([positions, filtered_reference_path[point]])
                break

        # calculate for each position the orientation of the vehicle
        dt_ref_traj = np.dtype([('position', float, (2,)), ('orientation', float), ('time_step', int), ('normal', float, (2,2)), ('lateral_distance', float), ('intersection', object)])

        trajectory_positions: List = []

        for i in range(1, len(positions)-1):
            prev_point = positions[i - 1]
            curr_point = positions[i]
            next_point = positions[i + 1] 
            # calculate orientation of the vehicle
            tangent = next_point - prev_point
            tangent = tangent / np.linalg.norm(tangent)
            orientation = np.arctan2(tangent[1], tangent[0])

            # time step of the trajectory point
            time_step = i

            # calculate normal line
            normal = np.array([-tangent[1], tangent[0]])
            # calculate normal line endpoints for clearance calculation
            normal_endpoint_pos = [curr_point[0] + normal[0] * 100.0, curr_point[1] + normal[1] * 100.0]
            normal_endpoint_neg = [curr_point[0] - normal[0] * 100.0, curr_point[1] - normal[1] * 100.0]
            normal_radius = np.array([normal_endpoint_neg, normal_endpoint_pos])

            trajectory_positions.append((curr_point, orientation, time_step, normal_radius, min_lateral_clearance, None))
                       
        trajectory_positions = np.array(trajectory_positions, dtype=dt_ref_traj)

        # create set of relevant lanelets
        lanelet_ids = self.scenario.lanelet_network.find_lanelet_by_position(trajectory_positions["position"].tolist()) 
        # Collect all relevant lanelets
        relevant_lanelets = set()
        for lanelet_id in lanelet_ids:
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
        #TODO: Currently, obstacles on lanelets is not working as intended, so all obstacles are considered
        # for lanelet in relevant_lanelets:
        #     if lanelet.dynamic_obstacles_on_lanelet:
        #         for time_step, obstacle_set in lanelet.dynamic_obstacles_on_lanelet.items():
        #             combined_obstacle_set.update(obstacle_set)
        #             if time_step >= max_time_step:
        #                 break
        if self.scenario.dynamic_obstacles is not None:
            for obs in self.scenario.dynamic_obstacles:
                combined_obstacle_set.add(obs.obstacle_id)

        # Calculate the combined occupancy polygon for all obstacles on the relevant lanelets
        dt_dyn_obstacle = np.dtype([('obstacle_id', int), ('position', float, (2,)), ('orientation', float), ('time_step', int), ('distance', float), ('index', int)])
        obstacles_polygon = MultiPolygon()
        dyn_obstacles = []
        if combined_obstacle_set:
            for obstacle_id in combined_obstacle_set:
                obstacle = self.scenario.obstacle_by_id(obstacle_id)
                if obstacle is None:
                    continue

                # check if the obstacle is static or dynamic
                initial_state = obstacle.state_at_time(0)
                if initial_state.velocity < dynamic_velocity_threshold:
                    # for static obstacles, only consider the occupancy at time 0
                    occupancy = obstacle.occupancy_at_time(0)
                    # Convert occupancy to polygon
                    occupancy_polygon = occupancy.shape.shapely_object
                    obstacles_polygon = obstacles_polygon.union(occupancy_polygon)  
                
                # check if the obstacle is dynamic
                elif initial_state.velocity >= dynamic_velocity_threshold:
                    # for dynamic obstacles, consider the occupancy for the look ahead time
                    i = 0
                    for i in range(max_time_step + 1):
                        state = obstacle.state_at_time(i)
                        if state is None:
                            break
                        state_position = state.position
                        state_orientation = state.orientation
                        dyn_obstacles.append((obstacle_id, state_position, state_orientation, i, 0.0, 0))
                
                else:
                    raise ValueError("Obstacle velocity is not defined!")


        dyn_obstacles = np.array(dyn_obstacles, dtype=dt_dyn_obstacle)

        # assign each dynamic obstacle predicted position to the nearest trajectory point and calculate the distance
        dyn_obstacles_positions = dyn_obstacles['position']
        ref_trajetory_positions = trajectory_positions['position']
        tree = cKDTree(ref_trajetory_positions)
        distance_nearest_point, indices = tree.query(dyn_obstacles_positions)

        dyn_obstacles['distance'] = distance_nearest_point
        dyn_obstacles['index'] = indices

        # calculate the combined occupancy polygon for all dynamic obstacles
        ob_polygon = Polygon()
        prev_dyn_obstacle_id = None
        for dyn_obstacle in dyn_obstacles:
            # only consider obstacles, if the distance from the trajectory point to the dynamic obstacle is smaller than 20.0 m
            if dyn_obstacle['distance'] > 20.0:
                continue
            
            # only consider obstacles, if the time step of the dynamic obstacle is in similar range from the time step of the trajectory 
            time_step_diff = np.abs(dyn_obstacle['time_step'] - trajectory_positions[dyn_obstacle['index']]['time_step'])
            time_diff = time_step_diff * self.scenario.dt
            if time_diff > time_threshold:
                continue

            # only consider obstacle, if the orientation of the dynamic obstacle is different from the orientation of the trajectory point
            orientation_dyn_obs = dyn_obstacle['orientation']
            orientation_traj = trajectory_positions[dyn_obstacle['index']]['orientation']
            orientation_diff = np.abs(orientation_dyn_obs - orientation_traj)

            if orientation_diff > np.pi/3:
                obstacle = self.scenario.obstacle_by_id(dyn_obstacle['obstacle_id'])
                if obstacle is None:
                    continue
                occupancy = obstacle.occupancy_at_time(int(dyn_obstacle['time_step']))
                occupancy_polygon = occupancy.shape.shapely_object
                
                # check if dyn_obstacle is the same as the previous dyn_obstacle (same obstacle id, different time step)
                if prev_dyn_obstacle_id is not None and prev_dyn_obstacle_id != dyn_obstacle['obstacle_id']:
                    # obstacle id is different, add the previous obstacle polygon to the combined polygon
                    obstacles_polygon = obstacles_polygon.union(ob_polygon)
                    # reset obstacle polygon
                    ob_polygon = Polygon()

                ob_polygon = ob_polygon.union(occupancy_polygon)
                prev_dyn_obstacle_id = dyn_obstacle['obstacle_id']

        if not ob_polygon.is_empty:
            # add the last obstacle polygon to the combined polygon
            obstacles_polygon = obstacles_polygon.union(ob_polygon)
        
        if not obstacles_polygon.is_empty:
            # Calculate the minimum lateral distance from the trajectory points to the combined polygon
            for point in trajectory_positions:
                trajectory_point = Point(point['position'])
                normal_points = point['normal']
                normal_line = LineString([normal_points[0], normal_points[1]])
                
                # check for intersection with multipolygon
                intersection = normal_line.intersection(obstacles_polygon)
                point['intersection'] = intersection
                
                if not intersection.is_empty:
                    distance = trajectory_point.distance(intersection)
                    point['lateral_distance'] = distance
            
            min_lateral_clearance = min(trajectory_positions['lateral_distance'])

            # set proposed reference velocity based on the lateral clearance
            if min_lateral_clearance < min_distance:
                # lateral clearance is smaller than the minimal distance, set reference velocity to minimum
                proposed_reference_velocity = min_reference_velocity
            elif min_lateral_clearance > safe_distance:
                # lateral clearance is larger than the safe distance, set reference velocity to maximum
                proposed_reference_velocity = max_reference_velocity
            else:
                # lateral clearance is between the minimal and safe distance
                # calculate normalized radius and use a quadratic function for velocity adjustment
                normalized_radius = (min_lateral_clearance - min_distance) / (safe_distance - min_distance)
                proposed_reference_velocity = min_reference_velocity + (max_reference_velocity - min_reference_velocity) * (normalized_radius)**2
            
            reference_velocity = min(reference_velocity, proposed_reference_velocity)            
            self._logger.debug(f"Reference velocity: {reference_velocity*3.6} km/h")


        t_end = time.perf_counter()
        self._logger.debug(f"Time for lateral clearance velocity function: {t_end - t_start}")
        if publish_lateral_clearance_topics:
            self.publish_obstacles(obstacles_polygon)
            self.publish_clearance(trajectory_positions, min_distance, safe_distance)

        return reference_velocity

    def publish_obstacles(self, multipolygon: MultiPolygon):
        """
        Publishes the considerd obstacles for lateral clearance calculation to the ROS2 node.

        :param multipolygon: MultiPolygon of the considered obstacles
        """
        marker_array = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        if multipolygon is not None:
            if isinstance(multipolygon, Polygon):
                polygons = [multipolygon]
            elif isinstance(multipolygon, MultiPolygon):
                polygons = multipolygon.geoms
            else:
                self._logger.error("Unsupported geometry type for multipolygon")
                return
            
            for i, polygon in enumerate(polygons):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = Time().to_msg()
                marker.ns = "obstacle_polygon"
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
                
        self._lateral_clearance_obstacles_pub.publish(marker_array)

    def publish_clearance(self, trajectory_points: np.array, min_distance: float, safe_distance: float):
        """
        Publishes the lateral clearance to the ROS2 node.

        Red: no lateral clearance
        Yellow: minimal lateral clearance, but not safe lateral clearance
        Green: safe lateral clearance
        
        :param trajectory_points: trajectory points of the ego vehicle
        :param min_distance: minimum radius of the lateral clearance
        :param safe_distance: safe radius of the lateral clearance
        """
        marker_array = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)
        if trajectory_points is not None:
            time = Time().to_msg()
            # Create markers for normal lines
            normal_marker_red = Marker()
            normal_marker_red.header.frame_id = "map"
            normal_marker_red.header.stamp = time
            normal_marker_red.ns = "normal_marker_red"
            normal_marker_red.id = -3
            normal_marker_red.type = Marker.LINE_LIST
            normal_marker_red.action = Marker.ADD
            normal_marker_red.scale.x = 0.1
            normal_marker_red.color.a = 1.0
            normal_marker_red.color.r = 1.0
            normal_marker_red.color.g = 0.0
            normal_marker_red.color.b = 0.0

            normal_marker_yellow = Marker()
            normal_marker_yellow.header.frame_id = "map"
            normal_marker_yellow.header.stamp = time
            normal_marker_yellow.ns = "normal_marker_yellow"
            normal_marker_yellow.id = -2
            normal_marker_yellow.type = Marker.LINE_LIST
            normal_marker_yellow.action = Marker.ADD
            normal_marker_yellow.scale.x = 0.1
            normal_marker_yellow.color.a = 1.0
            normal_marker_yellow.color.r = 1.0
            normal_marker_yellow.color.g = 1.0
            normal_marker_yellow.color.b = 0.0

            normal_marker_green = Marker()
            normal_marker_green.header.frame_id = "map"
            normal_marker_green.header.stamp = time
            normal_marker_green.ns = "normal_marker_green"
            normal_marker_green.id = -1
            normal_marker_green.type = Marker.LINE_LIST
            normal_marker_green.action = Marker.ADD
            normal_marker_green.scale.x = 0.1
            normal_marker_green.color.a = 1.0
            normal_marker_green.color.r = 0.0
            normal_marker_green.color.g = 1.0
            normal_marker_green.color.b = 0.0
            
            for i, point in enumerate(trajectory_points['position']):
                # Create marker for trajectory points
                traj_marker = Marker()
                traj_marker.header.frame_id = "map"
                traj_marker.header.stamp = time
                traj_marker.ns = "trajectory_points"
                traj_marker.id = i
                traj_marker.type = Marker.CYLINDER
                traj_marker.action = Marker.ADD
                traj_marker.pose.position = PointMsg()
                p = utm2map(self.scenario_handler.origin_transformation, [point[0], point[1]])
                traj_marker.pose.position.x = p.x
                traj_marker.pose.position.y = p.y
                traj_marker.pose.position.z = self.scenario_handler.z_coordinate - 0.1
                traj_marker.scale.x = 0.25
                traj_marker.scale.y = 0.25
                traj_marker.scale.z = 0.01
                traj_marker.color.a = 1.0
                traj_marker.color.r = 0.0
                traj_marker.color.g = 1.0
                traj_marker.color.b = 0.0
                marker_array.markers.append(traj_marker)

                # Create marker for normal line
                intersection = trajectory_points['intersection'][i]
                trajectory_point = Point(point)
                # check if intersection is a point, a line or a MultiLineString
                if intersection is None or intersection.is_empty:
                    continue
                elif intersection.geom_type == "Point":
                    start_point = utm2map(self.scenario_handler.origin_transformation, point)
                    end_point = utm2map(self.scenario_handler.origin_transformation, [intersection.x, intersection.y])
                elif intersection.geom_type == "LineString":
                    inter_x, inter_y = intersection.xy
                    # create a buffer around the intersection line, to check if trajectory point is on the intersection line
                    intersection_buffered = intersection.buffer(0.1)
                    # check which intersection point is closer to the trajectory point
                    if intersection_buffered.contains(trajectory_point):
                        # if obstacle is on reference path, normal line is the line between the two intersection points
                        start_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[0], inter_y[0]])
                        end_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[1], inter_y[1]])
                    elif trajectory_point.distance(Point(inter_x[0], inter_y[0])) < trajectory_point.distance(Point(inter_x[1], inter_y[1])):
                        start_point = utm2map(self.scenario_handler.origin_transformation, point)
                        end_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[0], inter_y[0]])
                    else:
                        start_point = utm2map(self.scenario_handler.origin_transformation, point)
                        end_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[1], inter_y[1]])
                elif intersection.geom_type == "MultiLineString":
                    end_points = []
                    linestrings = intersection.geoms
                    for line in linestrings:
                        inter_x, inter_y = line.xy
                        buffered_line = line.buffer(0.1)
                        # check which intersection point is closer to the trajectory point
                        if buffered_line.contains(trajectory_point):
                            start_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[0], inter_y[0]])
                            end_point = utm2map(self.scenario_handler.origin_transformation, [inter_x[1], inter_y[1]])
                            break
                        elif trajectory_point.distance(Point(inter_x[0], inter_y[0])) < trajectory_point.distance(Point(inter_x[1], inter_y[1])):
                            end_points.append(Point(inter_x[0], inter_y[0]))
                        else:
                            end_points.append(Point(inter_x[1], inter_y[1]))                            
                    # check which intersection point is closer to the trajectory point
                    if end_points:
                        min_distance = float('inf')
                        nearest_end_point = None
                        for end_point in end_points:
                            distance = trajectory_point.distance(end_point)
                            if distance < min_distance:
                                min_distance = distance
                                nearest_end_point = end_point
                        start_point = utm2map(self.scenario_handler.origin_transformation, point)
                        end_point = utm2map(self.scenario_handler.origin_transformation, [nearest_end_point.x, nearest_end_point.y])

                start_point.z = self.scenario_handler.z_coordinate
                end_point.z = self.scenario_handler.z_coordinate

                # change color depending on the lateral distance
                if trajectory_points['lateral_distance'][i] < min_distance:
                    # distance is smaller than the width of the vehicle, set color to red
                    normal_marker_red.points.append(start_point)
                    normal_marker_red.points.append(end_point)
                elif trajectory_points['lateral_distance'][i] > safe_distance:
                    # distance is larger than the double width of the vehicle, set color to green
                    normal_marker_green.points.append(start_point)
                    normal_marker_green.points.append(end_point)
                else:
                    # distance is between the width and double width of the vehicle, set color to yellow
                    normal_marker_yellow.points.append(start_point)
                    normal_marker_yellow.points.append(end_point)
            
            marker_array.markers.append(normal_marker_red)
            marker_array.markers.append(normal_marker_yellow)
            marker_array.markers.append(normal_marker_green)

        self._lateral_clearance_pub.publish(marker_array)
