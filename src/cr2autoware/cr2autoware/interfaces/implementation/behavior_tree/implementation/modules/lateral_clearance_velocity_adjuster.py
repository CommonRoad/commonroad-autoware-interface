import py_trees
from ...base.base_tree import BaseTree
from py_trees.common import Status
from py_trees.behaviour import Behaviour
from ...behavior_utils import copy_from_blackboard
import numpy as np
import time
from scipy.spatial import cKDTree
from shapely.geometry import LineString, MultiPolygon, Point, Polygon

# commonroad imports
from commonroad.scenario.scenario import Scenario

# cr2autoware
from cr2autoware.common.utils.transform import utm2map
from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from cr2autoware.common.configuration import CR2AutowareParams, BehaviorPlannerParams

# ROS imports
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time

# ROS messages
from geometry_msgs.msg import Point as PointMsg
from visualization_msgs.msg import Marker, MarkerArray

class LateralClearanceVelocityAdjusterTree(BaseTree):
    """
    Submodule for lateral clearance velocity adjusting.

    :var logger: ROS2 node logger
    :var verbose: Flag for verbose logging
    :var root: Root node of the behavior tree    
    """
    def __init__(self, logger: RcutilsLogger, verbose: bool):
        super(LateralClearanceVelocityAdjusterTree, self).__init__(logger, verbose)
        self.root = self.create_behavior_tree()
    
    def create_behavior_tree(self):
        root = LateralClearanceVelocityAdjuster(name="LateralClearanceVelocityAdjuster", logger=self.logger)
        return root


class LateralClearanceVelocityAdjuster(Behaviour):
    """
    Behavior Node for lateral clearance velocity adjusting.

    :var name: Name of the behavior node
    :var logger: ROS2 node logger
    :var blackboard: Blackboard for the behavior tree
    :var global_inputs: Blackboard client for global inputs
    :var inputs: Blackboard client for module inputs
    :var outputs: Blackboard client for module outputs
    :var global_params: CR2AutowareParams
    :var params: BehaviorPlannerParams
    """
    def __init__(self, name: str, logger: RcutilsLogger):
        super().__init__(name)
        self._logger = logger

        self.init_blackboard(name)

    def init_blackboard(self, name):
        """
        Initialize the blackboard parameters.

        :param name: Name of the behavior node
        """
        # Global Blackboard
        self.blackboard = py_trees.blackboard.Client(name=(name + "Blackboard"))
        self.blackboard.register_key("global_params", access=py_trees.common.Access.READ)
        self.blackboard.register_key("params", access=py_trees.common.Access.READ)

        # Register keys for Global Inputs
        self.global_inputs = py_trees.blackboard.Client(name=(name + "GlobalInputs"), namespace="inputs")
        self.global_inputs.register_key("scenario", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_state", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("input_path", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("coordinate_system", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("input_path_curvilinear", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("origin_transformation", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("last_velocity_profile", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_position_curvilinear", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("z_coordinate", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_time_msg", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("current_position_index", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("input_path_orientation", access=py_trees.common.Access.READ)
        self.global_inputs.register_key("empty_velocity_profile", access=py_trees.common.Access.READ)

        # Register keys for Module Inputs
        self.inputs = py_trees.blackboard.Client(name=(name + "Inputs"), namespace="/modules/lateral_clearance/inputs")
        self.inputs.register_key("lateral_clearance_path", access=py_trees.common.Access.WRITE)

        # Register keys for Module Outputs
        self.outputs = py_trees.blackboard.Client(name=(name + "Outputs"), namespace="/modules/lateral_clearance/outputs")
        self.outputs.register_key("velocity_profile", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("lateral_clearance_marker_array", access=py_trees.common.Access.WRITE)
        self.outputs.register_key("lateral_clearance_obstacles_marker_array", access=py_trees.common.Access.WRITE)
       
        # Init Parameter
        self.global_params: CR2AutowareParams = self.blackboard.global_params
        self.params: BehaviorPlannerParams = self.blackboard.params

    def setup(self) -> None:
        pass

    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        """
        Check lateral clearance in the scenario and adjust the reference velocity.

        All static obstacles and dynamic obstacles on relevant lanelets, with different orientations, and within a limited time to the reference path are merged into 
        a combined occupancy polygon. The distance between the filtered reference path and the combined occupancy polygon is calculated. Based on this distance, 
        the reference velocity is adjusted.
        """
        t_start = time.perf_counter()

        # initialize parameters for lateral clearance
        scenario: Scenario = self.global_inputs.scenario
        # minimal velocity (in m/s) for dynamic obstacles, otherwise they are considered as static obstacles
        dynamic_velocity_threshold: float = self.params.dynamic_velocity_threshold
        # look ahead time (in seconds) for the vehicle to react to obstacles
        look_ahead_time: float = self.params.look_ahead_time
        # minimal look ahead distance (in meters) for the vehicle to react to obstacles
        min_look_ahead_distance: float = self.params.min_look_ahead_distance
        # maximal time step for prediction of dynamic obstacles
        max_time_step: int = int(look_ahead_time / scenario.dt)
        # time threshold (in seconds) for intersection of trajectories of ego vehicle and dynamic obstacles
        time_threshold: float = self.params.time_threshold
        # get velocity limits in m/s
        max_reference_velocity: float = self.params.velocity_limit
        min_reference_velocity: float = self.params.min_reference_velocity
        # initialize minimum and safe distance (radius) for lateral clearance in meters
        vehicle_width: float = self.global_params.vehicle.wheel_tread + self.global_params.vehicle.right_overhang + self.global_params.vehicle.left_overhang
        min_distance: float = vehicle_width * 0.5
        safe_distance: float = vehicle_width
        # flag to publish lateral clearance topics
        publish_lateral_clearance_topics: bool = self.params.publish_lateral_clearance_topics

        # Get the reference path from the reactive planner
        reference_path_cartesian: np.ndarray = self.global_inputs.input_path
        reference_path_curvilinear: np.ndarray = self.global_inputs.input_path_curvilinear
        reference_orientation_curvilinear: np.ndarray = self.global_inputs.input_path_orientation
        current_state: EgoVehicleState = self.global_inputs.current_state

        # Get nearest point in the reference path to the current vehicle position
        current_position_index: int = self.global_inputs.current_position_index

        # filter the reference path from the nearest index to the end
        reference_path_cartesian = reference_path_cartesian[current_position_index:]
        reference_path_curvilinear = reference_path_curvilinear[current_position_index:]
        reference_orientation_curvilinear = reference_orientation_curvilinear[current_position_index:]

        # calculate reaction distance, a look ahead distance for the vehicle to react to obstacles
        look_ahead_distance = current_state.velocity * look_ahead_time
        combined_distance: float = 0.0

        if look_ahead_distance < min_look_ahead_distance:
            look_ahead_distance = min_look_ahead_distance

        time_steps = [0]
        for point in range(1, len(reference_path_curvilinear)):
            point_distance = np.abs(reference_path_curvilinear[point] - reference_path_curvilinear[point - 1])
            combined_distance += point_distance
            time_steps.append(point)
            if combined_distance > look_ahead_distance:
                break                

        # calculate the reference_path_dt to check time steps for dynamic obstacles
        trajectory_dt = look_ahead_time / (len(time_steps) - 1)

        # calculate for each position the orientation of the vehicle
        dt_ref_traj = np.dtype([('position', float, (2,)), ('position_curvilinear', float), ('orientation', float), ('time_step', int), ('normal', float, (2,2)), ('lateral_distance', float), ('intersection', object), ('velocity', float)])
        trajectory_positions = np.zeros(len(time_steps), dtype=dt_ref_traj)

        trajectory_positions['position'] = reference_path_cartesian[:len(time_steps)]
        trajectory_positions['position_curvilinear'] = reference_path_curvilinear[:len(time_steps)]
        trajectory_positions['orientation'] = reference_orientation_curvilinear[:len(time_steps)]
        trajectory_positions['time_step'] = time_steps
        trajectory_positions['lateral_distance'] = np.full(len(time_steps), float('inf'))
        trajectory_positions['intersection'] = np.full(len(time_steps), None)
        trajectory_positions['velocity'] = np.full(len(time_steps), float('inf'))

        # calculate the normal endpoints for each trajectory point
        for i, point in enumerate(trajectory_positions):
            normal = np.array([np.cos(point['orientation'] + np.pi/2), np.sin(point['orientation'] + np.pi/2)])
            normal_end_point_pos = point['position'] + normal * 100.0
            normal_end_point_neg = point['position'] - normal * 100.0
            trajectory_positions[i]['normal'] = np.array([normal_end_point_pos, normal_end_point_neg])

        # TODO: Currently, obstacles on lanelets is not working as intended, so all obstacles are considered
        # # create set of relevant lanelets
        # lanelet_ids = scenario.lanelet_network.find_lanelet_by_position(trajectory_positions["position"].tolist())
        # # Collect all relevant lanelets
        # relevant_lanelets = set()
        # for lanelet_id in lanelet_ids:
        #     lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id[0])
        #     relevant_lanelets.add(lanelet)
        #     if lanelet.adj_left is not None:
        #         left_adjacent_lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
        #         relevant_lanelets.add(left_adjacent_lanelet)
        #     if lanelet.adj_right is not None:
        #         right_adjacent_lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
        #         relevant_lanelets.add(right_adjacent_lanelet)

        # Merge obstacle sets from the relevant lanelets
        combined_obstacle_set = set()
        # TODO: Currently, obstacles on lanelets is not working as intended, so all obstacles are considered
        # for lanelet in relevant_lanelets:
        #     if lanelet.dynamic_obstacles_on_lanelet:
        #         for time_step, obstacle_set in lanelet.dynamic_obstacles_on_lanelet.items():
        #             combined_obstacle_set.update(obstacle_set)
        #             if time_step >= max_time_step:
        #                 break
        if scenario.dynamic_obstacles is not None:
            for obs in scenario.dynamic_obstacles:
                combined_obstacle_set.add(obs.obstacle_id)

        # Calculate the combined occupancy polygon for all obstacles on the relevant lanelets
        dt_dyn_obstacle = np.dtype([('obstacle_id', int), ('position', float, (2,)), ('orientation', float), ('time_step', int), ('distance', float), ('index', int)])
        obstacles_polygon = MultiPolygon()
        dyn_obstacles = []
        if combined_obstacle_set:
            for obstacle_id in combined_obstacle_set:
                obstacle = scenario.obstacle_by_id(obstacle_id)
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
            # convert time steps to seconds
            time_dyn_obs = dyn_obstacle['time_step'] * scenario.dt
            time_ref_traj = trajectory_positions[dyn_obstacle['index']]['time_step'] * trajectory_dt
            time_diff = np.abs(time_dyn_obs - time_ref_traj)
            if time_diff > time_threshold:
                continue

            # only consider obstacle, if the orientation of the dynamic obstacle is different from the orientation of the trajectory point
            orientation_dyn_obs = dyn_obstacle['orientation']
            orientation_traj = trajectory_positions[dyn_obstacle['index']]['orientation']
            orientation_diff = np.abs(np.arctan2(np.sin(orientation_dyn_obs - orientation_traj), np.cos(orientation_dyn_obs - orientation_traj)))

            if orientation_diff > np.pi/3:
                obstacle = scenario.obstacle_by_id(dyn_obstacle['obstacle_id'])
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

                distance = float('inf')
                if not intersection.is_empty:
                    distance = trajectory_point.distance(intersection)
                    point['lateral_distance'] = distance

                # set proposed reference velocity based on the lateral clearance
                if distance < min_distance:
                    # lateral clearance is smaller than the minimal distance, set reference velocity to minimum
                    proposed_reference_velocity = min_reference_velocity
                elif distance > safe_distance:
                    # lateral clearance is larger than the safe distance, set reference velocity to maximum
                    proposed_reference_velocity = max_reference_velocity
                else:
                    # lateral clearance is between the minimal and safe distance
                    # calculate normalized radius and use a quadratic function for velocity adjustment
                    normalized_radius = (distance - min_distance) / (safe_distance - min_distance)
                    proposed_reference_velocity = min_reference_velocity + (max_reference_velocity - min_reference_velocity) * (normalized_radius)**2

                point['velocity'] = proposed_reference_velocity

        # Adjust the velocity profile and consider vehicle length and buffer
        vehicle_front_to_origin = self.global_params.vehicle.front_overhang + self.global_params.vehicle.wheel_base
        vehicle_origin_to_rear = self.global_params.vehicle.rear_overhang
        buffer_front = self.params.front_buffer
        buffer_rear = self.params.rear_buffer

        trajectory_lenght = len(trajectory_positions)
        adjusted_velocity_profile = np.full(trajectory_lenght, float('inf'))

        for i in range(trajectory_lenght):
            # Calculate the indices considering the vehicle length and buffer
            front_index = min(trajectory_lenght - 1, i + int(np.ceil(buffer_front + vehicle_front_to_origin)))
            rear_index = max(0, i - int(np.ceil(vehicle_origin_to_rear + buffer_rear)))

            # Apply the proposed velocity to the adjusted profile
            adjusted_velocity_profile[rear_index:front_index + 1] = np.minimum(adjusted_velocity_profile[rear_index:front_index + 1], trajectory_positions['velocity'][i])

        # set the velocity profile with lateral clearance to the blackboard
        velocity_profile = copy_from_blackboard(self.global_inputs.empty_velocity_profile)
        # save the velocity profile with lateral clearance to the blackboard
        start_index = current_position_index
        end_index = start_index + len(adjusted_velocity_profile)

        velocity_profile[start_index:end_index] = adjusted_velocity_profile[:len(velocity_profile[start_index:end_index])]
        self.outputs.velocity_profile = velocity_profile

        t_end = time.perf_counter()
        self._logger.debug(f"Time for lateral clearance velocity function: {t_end - t_start}")
        if publish_lateral_clearance_topics:
            self.publish_obstacles(obstacles_polygon)
            self.publish_clearance(trajectory_positions, min_distance, safe_distance)

        return py_trees.common.Status.SUCCESS

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
            
            origin_transformation = self.global_inputs.origin_transformation
            z_coordinate = self.global_inputs.z_coordinate

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
                    p = utm2map(origin_transformation, [x, y])
                    p.z = z_coordinate
                    marker.points.append(p)

                # Add first point again to close the polygon
                if len(polygon.exterior.coords) > 0:
                    first_point = polygon.exterior.coords[0]
                    p = utm2map(origin_transformation, [first_point[0], first_point[1]])
                    p.z = z_coordinate
                    marker.points.append(p)

                marker_array.markers.append(marker)

        self.outputs.lateral_clearance_obstacles_marker_array = marker_array

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

            origin_transformation = self.global_inputs.origin_transformation
            z_coordinate = self.global_inputs.z_coordinate

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
                p = utm2map(origin_transformation, [point[0], point[1]])
                traj_marker.pose.position.x = p.x
                traj_marker.pose.position.y = p.y
                traj_marker.pose.position.z = z_coordinate - 0.1
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
                    start_point = utm2map(origin_transformation, point)
                    end_point = utm2map(origin_transformation, [intersection.x, intersection.y])
                elif intersection.geom_type == "LineString":
                    inter_x, inter_y = intersection.xy
                    # create a buffer around the intersection line, to check if trajectory point is on the intersection line
                    intersection_buffered = intersection.buffer(0.1, quadsegs=1, cap_style=2)
                    # check which intersection point is closer to the trajectory point
                    if intersection_buffered.contains(trajectory_point):
                        # if obstacle is on reference path, normal line is the line between the two intersection points
                        start_point = utm2map(origin_transformation, [inter_x[0], inter_y[0]])
                        end_point = utm2map(origin_transformation, [inter_x[1], inter_y[1]])
                    elif trajectory_point.distance(Point(inter_x[0], inter_y[0])) < trajectory_point.distance(Point(inter_x[1], inter_y[1])):
                        start_point = utm2map(origin_transformation, point)
                        end_point = utm2map(origin_transformation, [inter_x[0], inter_y[0]])
                    else:
                        start_point = utm2map(origin_transformation, point)
                        end_point = utm2map(origin_transformation, [inter_x[1], inter_y[1]])
                elif intersection.geom_type == "MultiLineString":
                    end_points = []
                    check_end_points = True
                    linestrings = intersection.geoms
                    for line in linestrings:
                        inter_x, inter_y = line.xy
                        buffered_line = line.buffer(0.1, quadsegs=1, cap_style=2)
                        # check which intersection point is closer to the trajectory point
                        if buffered_line.contains(trajectory_point):
                            start_point = utm2map(origin_transformation, [inter_x[0], inter_y[0]])
                            end_point = utm2map(origin_transformation, [inter_x[1], inter_y[1]])
                            check_end_points = False
                            break
                        elif trajectory_point.distance(Point(inter_x[0], inter_y[0])) < trajectory_point.distance(Point(inter_x[1], inter_y[1])):
                            end_points.append(Point(inter_x[0], inter_y[0]))
                        else:
                            end_points.append(Point(inter_x[1], inter_y[1]))                            
                    # check which intersection point is closer to the trajectory point
                    if check_end_points:
                        min_distance_to_traj = float('inf')
                        nearest_end_point = None
                        for end_point in end_points:
                            distance = trajectory_point.distance(end_point)
                            if distance < min_distance_to_traj:
                                min_distance_to_traj = distance
                                nearest_end_point = end_point
                        start_point = utm2map(origin_transformation, point)
                        end_point = utm2map(origin_transformation, [nearest_end_point.x, nearest_end_point.y])

                start_point.z = z_coordinate
                end_point.z = z_coordinate

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

        self.outputs.lateral_clearance_marker_array = marker_array

    def terminate(self, new_status) -> None:
        pass


