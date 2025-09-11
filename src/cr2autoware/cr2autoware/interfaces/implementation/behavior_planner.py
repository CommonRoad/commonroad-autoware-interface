# standard imports
import math
from typing import List, Optional

# third party imports
import numpy as np
import time

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_msgs.msg import Bool

# ROS message imports
from builtin_interfaces.msg import Duration

# Autoware.Auto message imports
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory

# commonroad-dc imports
from commonroad_dc.geometry.util import compute_orientation_from_polyline

# cr2autoware imports
from cr2autoware.common.utils.transform import orientation2quaternion
from cr2autoware.common.utils.transform import utm2map, map2utm
from cr2autoware.interfaces.implementation.behavior_tree.implementation.behavior_tree import BehaviorTree
from cr2autoware.common.configuration import BehaviorPlannerParams, CR2AutowareParams
from cr2autoware.handlers.scenario_handler import ScenarioHandler
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem
from cr2autoware.interfaces.implementation.behavior_tree.behavior_utils import BehaviorScenarioParams

from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from commonroad.scenario.scenario import Scenario
from scipy.interpolate import interp1d

import py_trees
from visualization_msgs.msg import MarkerArray


class BehaviorPlanner:
    """
    Behavior Planner Module for the CommonRoad to Autoware interface.

    This module is responsible for planning a reference trajectory with velocity information for the ego vehicle.
    In addition, it provides the lateral offset parameters for the reactive planner.

    Within a behavior tree, the planner calculates a velocity profile for a given reference path. A CommonRoad
    scenario provides information about the environment, including other vehicles and obstacles. The behavior planner 
    adjusts the velocity profile and lateral offset parameters based on the scenario, ego vehicle state, reference path,
    and other parameters.

    ---------------
    **Publishers:**

    * _ref_path_pub:
        * Description: Publishes reference path with velocity profile to motion velocity smoother
        * Topic: `/planning/scenario_planning/trajectory_smoothed`
        * Message Type: `autoware_auto_planning_msgs/Trajectory`
    * lane_keeping_markers_pub:
        * Description: Lane keeping visualization.
        * Topic: `/planning/commonroad/behavior_planning/lane_keeping_marker`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * lateral_clearance_pub:
        * Description: Lateral clearance visualization.
        * Topic: `/planning/commonroad/behavior_planning/lateral_clearance`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * traffic_light_marker_pub:
        * Description: Traffic light visualization.
        * Topic: `/planning/commonroad/behavior_planning/traffic_light_marker`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * slowdown_pub:
        * Description: Slowdown message.
        * Topic: `/planning/commonroad/behavior_planning/slowdown`
        * Message Type: `std_msgs.msg.Bool`

    ---------------
    :var _ref_path_pub: reference to ROS2 publisher for reference path
    :var _logger: reference to ROS2 logger
    :var _verbose: constant for verbose logging
    :var _reference_trajectory: reference trajectory with velocity profile; trajectory is a (n x 3) numpy array,
    where each row contains x, y, v for a certain point on the reference trajectory; Coordinates in AW map frame
    :var _tail: tail of reference path behind goal position; Coordinates in AW map frame
    :var _lookahead_dist: lookahead distance for velocity planning
    :var _lookahead_time: lookahead time for velocity planning
    """
    def __init__(self, ref_path_pub: Publisher, traffic_light_marker_pub: Publisher, lateral_clearance_pub: Publisher, lane_keeping_markers_pub: Publisher, slowdown_pub: Publisher, logger: RcutilsLogger, verbose: bool,
                 lookahead_dist: float, lookahead_time: float, origin_transformation: List, global_params: CR2AutowareParams, scenario_handler: ScenarioHandler) -> None:
        """
        Constructor for BehaviorPlanner class.

        :param ref_path_pub: ROS2 node publisher for reference path
        :param traffic_light_marker_pub: ROS2 node publisher for traffic light marker
        :param lateral_clearance_pub: ROS2 node publisher for lateral clearance marker
        :param lane_keeping_markers_pub: ROS2 node publisher for lane keeping marker
        :param slowdown_pub: ROS2 node publisher for slowdown message
        :param logger: ROS2 node logger
        :param verbose: Flag for verbose logging
        :param lookahead_dist: Lookahead distance for velocity planning
        :param lookahead_time: Lookahead time for velocity planning
        :param origin_transformation: translation of origin between CR and AW map coordinates
        :param global_params: Global parameters for the planner
        :param scenario_handler: Scenario handler for the planner
        """

        # initialize publisher to behavior planner
        self._ref_path_pub = ref_path_pub
        self._traffic_light_marker_pub = traffic_light_marker_pub
        self._lateral_clearance_pub = lateral_clearance_pub
        self._lane_keeping_markers_pub = lane_keeping_markers_pub
        self._slowdown_pub = slowdown_pub

        self._verbose = verbose
        self._logger = logger

        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Initializing planner with lookahead distance "
                              + str(lookahead_dist) + " and lookahead time " + str(lookahead_time))

        # variable indicates if velocity planning for latest published route is completed
        self._is_velocity_planning_completed = False

        # init reference trajectory (ref path with velocity)
        # reference trajectory is a (n x 3) numpy array, where each row contains x, y, v for a certain
        # point on the reference trajectory
        # Coordinates in AW map frame
        self._reference_trajectory: Optional[np.ndarray] = None

        # Init Parameter
        self.global_params: CR2AutowareParams = global_params
        self.params: BehaviorPlannerParams = self.global_params.behavior_planner
        self.blackboard = py_trees.blackboard.Client(name="GlobalBehaviorTreeBlackboard")
        self.blackboard.register_key("params", access=py_trees.common.Access.WRITE)
        self.blackboard.params = self.params
        self.blackboard.register_key("global_params", access=py_trees.common.Access.WRITE)
        self.blackboard.global_params = self.global_params
        # Register keys for Callbacks
        self.blackboard.register_key("/modules/lane_keeping/inputs/ros_condition", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.lane_keeping.inputs.ros_condition = False        
        # Register keys for ROS Publisher
        self.blackboard.register_key("/slowdown/bool", access=py_trees.common.Access.WRITE)
        self.blackboard.slowdown.bool = False
        self.blackboard.register_key("/modules/traffic_lights/outputs/traffic_light_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.traffic_lights.outputs.traffic_light_marker_array = MarkerArray()
        self.blackboard.register_key("/modules/lateral_clearance/outputs/lateral_clearance_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_marker_array = MarkerArray()
        self.blackboard.register_key("/modules/lane_keeping/outputs/lane_keeping_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.lane_keeping.outputs.lane_keeping_marker_array = MarkerArray()

        # for testdrive logger
        self.blackboard.register_key("/inputs/current_position_curvilinear", access=py_trees.common.Access.READ)

        # Initialize the Behavior Tree
        self.behavior_tree = BehaviorTree(self._logger, self._verbose)

        # init tail (part of ref path behind goal position)
        # Coordinates in AW map frame
        self._tail = None

        # slowdown current position in curvilinear coordinates
        self.slowdown_current_position_curvilinear = None

        # set scenario_handler
        self.scenario_handler = scenario_handler

        # lookahead distance and time
        self._lookahead_dist: float = lookahead_dist
        self._lookahead_time: float = lookahead_time

        # Origin transformation between CR and AW map coordinates
        self.origin_transformation = origin_transformation

        # coordinate system & collision checker
        self._co: Optional[CoordinateSystem] = None

    @property
    def reference_trajectory(self) -> Optional[np.ndarray]:
        """
        Computed reference trajectory after velocity planning.

        Coordinates in AW map frame.

        :return: reference trajectory
        """
        if self._reference_trajectory is None:
            raise ValueError("Reference trajectory is not computed yet!")
        return self._reference_trajectory

    @property
    def reference_positions(self) -> Optional[np.ndarray]:
        """
        Reference trajectory positions.

        Coordinates in AW map frame.

        :return: reference trajectory positions
        """
        if self._reference_trajectory is None:
            raise ValueError("Reference trajectory is not computed yet!")
        else:
            return self._reference_trajectory[:, 0:2]

    @property
    def reference_velocities(self) -> Optional[np.ndarray]:
        """
        Reference trajectory velocities.

        :return: reference trajectory velocities
        """
        if self._reference_trajectory is None:
            raise ValueError("Reference trajectory is not computed yet!")
        else:
            return self._reference_trajectory[:, 2]

    @property
    def is_velocity_planning_completed(self) -> bool:
        """
        Indicates if velocity planning for latest published route is completed.
        
        :return: completion status of velocity planning
        """
        return self._is_velocity_planning_completed

# Copied from Reactive Planner
    @property
    def coordinate_system(self) -> CoordinateSystem:
        return self._co
    
    @property
    def path_in_cartesian(self) -> np.ndarray:
        return self._co.reference

    @property
    def path_in_curvilinear(self) -> np.ndarray:
        return self._co.ref_pos

    @property
    def path_orientation(self) -> np.ndarray:
        return self._co.ref_theta

    @property
    def output_d_min(self) -> float:
        return self.behavior_tree.outputs.d_min

    @property
    def output_d_max(self) -> float:
        return self.behavior_tree.outputs.d_max
    
    @property
    def current_position_curvilinear(self) -> np.ndarray:
        return self.blackboard.inputs.current_position_curvilinear
    
    @property
    def slowdown_current_position(self) -> np.ndarray:
        return self.slowdown_current_position_curvilinear

    @property
    def scenario_params(self) -> BehaviorScenarioParams:
        """
        Scenario parameters set by behavior planner.
        :return: scenario parameters
        """
        if self.behavior_tree.outputs.scenario_params is None:
            raise ValueError("Scenario parameters are not set yet!")
        return self.behavior_tree.outputs.scenario_params

    def plan(self, reference_path: np.ndarray, goal_pos: np.ndarray, scenario: Scenario, current_state: EgoVehicleState) -> None:
        """
        Calls behavior planner.

        Computes a velocity profile for a given reference path.
        Resulting reference trajectory (i.e., path with velocity information) is stored.


        :param reference_path: in CR coordinates
        :param goal_pos: in CR coordinates
        :param scenario: CommonRoad scenario
        :param current_state: current state of the ego vehicle
        """
        self._is_velocity_planning_completed = False

        plan_start_time = time.time()

        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Planning velocity profile")

        # Clip original reference path so that it ends at the goal position
        goal_idx = self._get_closest_point_idx_on_path(reference_path, goal_pos)
        tail_orig = reference_path[goal_idx + 1:]
        input_path = reference_path[:goal_idx + 1]

        # transform points of tail to AW map coordinates
        tail_mod = list()
        for i in range(len(tail_orig)):
            _tmp = tail_orig[i] - np.array(self.origin_transformation)
            tail_mod.append(_tmp)
        self._tail = np.array(tail_mod)

        plan_start_time_2 = time.time()
        
        # Create Curvilinear Coordinate System for preprocessing
        self.set_reference_path(input_path)

        plan_start_time_3 = time.time()

        # Prepare Inputs for Behavior Planner
        self.behavior_tree.preprocessing(
            scenario,
            current_state,
            self.path_in_cartesian,
            self.coordinate_system,
            self.path_in_curvilinear,
            self.origin_transformation,
            self.scenario_handler.z_coordinate,
            self.scenario_handler.ros_time,
            self.path_orientation,
            )
        
        plan_start_time_4 = time.time()

        # Call Behavior Planner
        self.behavior_tree.plan()

        plan_end_time = time.time()

        self.behavior_tree.prepare_output()

        velocity_path = self.convert_velocity_profile(self.path_in_cartesian, self.behavior_tree.velocity_profile, input_path)

        # save velocity profile for testdrive logger
        self.velocity_profile_data = velocity_path
        self.input_path = input_path

        # Call _pub_ref_path
        self._pub_ref_path(input_path, velocity_path, self.origin_transformation)

        # Publish slowdown message
        self._pub_slowdown()

        # Publish traffic light marker
        self._pub_traffic_light_marker()

        # Publish lateral clearance velocity adjuster marker
        self._pub_lateral_clearance_marker()

        self._pub_lane_keeping_markers()

        plan_end_time_2 = time.time()

        if self._verbose:
            self._logger.info("[SVEN] [TIME] Behavior Planner: " + str(plan_end_time - plan_start_time))
            self._logger.info("[SVEN] [TIME] Pre Planning Transforms: " + str(plan_start_time_2 - plan_start_time))
            self._logger.info("[SVEN] [TIME] Pre Planning Curvilinear: " + str(plan_start_time_3 - plan_start_time_2))
            self._logger.info("[SVEN] [TIME] Pre Planning Behavior Planner: " + str(plan_start_time_4 - plan_start_time_3))
            self._logger.info("[SVEN] [TIME] Planning Behavior Planner: " + str(plan_end_time - plan_start_time_4))
            self._logger.info("[SVEN] [TIME] Post Planning: " + str(plan_end_time_2 - plan_end_time))

    def slowdown_planning(self, current_state: EgoVehicleState, reference_path: np.ndarray, goal_pos: np.ndarray) -> None:
        """
        Velocity planning in case of failure of behavior planner.

        Computes a zero velocity profile for a given reference path.

        :param current_state: current state of the ego vehicle
        :param reference_path: in CR coordinates
        :param goal_pos: in CR coordinates
        """
        self._is_velocity_planning_completed = False

        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Planning Slowdown velocity profile")

        # Clip original reference path so that it ends at the goal position
        goal_idx = self._get_closest_point_idx_on_path(reference_path, goal_pos)
        tail_orig = reference_path[goal_idx + 1:]
        input_path = reference_path[:goal_idx + 1]

        # transform points of tail to AW map coordinates
        tail_mod = list()
        for i in range(len(tail_orig)):
            _tmp = tail_orig[i] - np.array(self.origin_transformation)
            tail_mod.append(_tmp)
        self._tail = np.array(tail_mod)
        
        # Create Curvilinear Coordinate System for preprocessing
        self.set_reference_path(input_path)

        # Set current position in curvilinear coordinates
        self.slowdown_current_position_curvilinear = self.coordinate_system.convert_to_curvilinear_coords(current_state.position[0], current_state.position[1])

        # set velocity profile to zero
        velocity_path = np.zeros(len(input_path))
        
        # save velocity profile for testdrive logger
        self.velocity_profile_data = velocity_path

        # Call _pub_ref_path
        self._pub_ref_path(input_path, velocity_path, self.origin_transformation)


    def _prepare_traj_msg(self, input_path: np.ndarray, velocity_path: np.ndarray, origin_transformation: List) -> AWTrajectory:
        """
        Converts reference path to AWTrajectory message type for publishing to Motion Velocity Smoother.
        
        :param input_path: reference path in CR coordinates
        :param velocity_path: velocity profile for reference path
        :param origin_transformation: translation of origin between CR and AW map coordinates
        :return: AWTrajectory message
        """
        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Preparing reference path message for motion velocity smoother")

        # AW Trajectory message
        traj = AWTrajectory()
        traj.header.frame_id = "map"

        # compute orientations
        orientations = compute_orientation_from_polyline(input_path)

        if len(input_path) != len(velocity_path):
            raise ValueError("Length of input path and velocity path should be equal")
            
        for i in range(0, len(input_path)):
            velocity = float(velocity_path[i])
            new_point = TrajectoryPoint()
            new_point.time_from_start = Duration(sec=0, nanosec=i)
            new_point.pose.position = utm2map(origin_transformation, input_path[i])
            new_point.pose.orientation = orientation2quaternion(orientations[i])
            new_point.longitudinal_velocity_mps = velocity
            new_point.acceleration_mps2 = 0.0
            traj.points.append(new_point)

        return traj

    def _pub_ref_path(self, input_path: np.ndarray, velocity_path: np.ndarray, origin_transformation: List) -> None:
        """
        Publishes reference path to Motion Velocity Smoother.
        
        :param input_path: reference path in CR coordinates
        :param velocity_path: velocity profile for reference path
        :param origin_transformation: translation of origin between CR and AW map coordinates
        """
        traj_msg = self._prepare_traj_msg(input_path, velocity_path, origin_transformation)

        self._ref_path_pub.publish(traj_msg)

        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Reference path published to motion velocity smoother.")

    def _pub_slowdown(self) -> None:
        if self.blackboard.slowdown.bool:
            # publish slowdown message
            msg = Bool()
            msg.data = True
            self._slowdown_pub.publish(msg)
            self.blackboard.slowdown.bool = False

    def _pub_traffic_light_marker(self) -> None:
        self._traffic_light_marker_pub.publish(self.blackboard.modules.traffic_lights.outputs.traffic_light_marker_array)

    def _pub_lateral_clearance_marker(self) -> None:
        self._lateral_clearance_pub.publish(self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_marker_array)

    def _pub_lane_keeping_markers(self) -> None:
        self._lane_keeping_markers_pub.publish(self.blackboard.modules.lane_keeping.outputs.lane_keeping_marker_array)

    def convert_velocity_profile(self, source_path: np.ndarray, source_velocity_profile: np.ndarray, target_path: np.ndarray) -> np.ndarray:
        """
        Convert and Interpolate a velocity profile from a source_path to a velocity_profile for a target_path.
        
        :param source_path: path from behavior planner
        :param source_velocity_profile: velocity profile from behavior planner
        :param target_path: reference path
        :return: velocity profile for reference path
        """
        def compute_cumulative_distance(path):
            """Compute cumulative distances for a path."""
            diffs = np.diff(path, axis=0)
            segment_lengths = np.linalg.norm(diffs, axis=1)
            return np.concatenate(([0], np.cumsum(segment_lengths)))

        # Compute cumulative distances
        source_distances = compute_cumulative_distance(source_path)
        target_distances = compute_cumulative_distance(target_path)

        # Create an interpolation function
        velocity_interp = interp1d(source_distances, source_velocity_profile, kind='linear', fill_value="extrapolate")

        # Interpolate velocities for path2
        velocity_path = velocity_interp(target_distances)

        return velocity_path

    def smoothed_trajectory_callback(self, msg: AWTrajectory) -> None:
        """
        Call back function which subscribes to output of motion velocity smoother.
        
        :param msg: AWTrajectory message with velocity profile
        """
        start_time = time.time()
        if self._tail is None:
            return
        
        if self._verbose:
            self._logger.info("<BehaviorPlanner>: Path with velocity profile received from motion velocity smoother")

        point_list = list()
        velocity_list = list()
        # get velocities for each point of the reference path
        for point in msg.points:
            point_list.append([point.pose.position.x, point.pose.position.y])
            velocity_list.append(point.longitudinal_velocity_mps)

        # append tail of reference trajectory
        zeros = [0] * len(self._tail)

        if len(self._tail) == 0:
            positions_arr = np.array(point_list)
        else:
            positions_arr = np.concatenate((np.array(point_list), self._tail), axis=0)

        velocities_arr = np.array(velocity_list + zeros)

        _len_vel_arr = len(velocities_arr)

        # get reference trajectory
        self._reference_trajectory = np.concatenate((positions_arr, velocities_arr.reshape(_len_vel_arr, 1)), axis=1)
        self._is_velocity_planning_completed = True
        # self._logger.info("[SVEN] [TIME] Velocity Smoother Callback completed in " + str(time.time() - start_time) + " seconds")

    def get_lookahead_velocity_for_current_state(self, curr_position, curr_velocity) -> Optional[float]:
        """
        Gets velocity from velocity profile with lookahead for a given position and velocity.
        
        :param curr_position: current position of the vehicle
        :param curr_velocity: current velocity of the vehicle
        :return: velocity with lookahead
        """

        curr_position_arr = np.array([curr_position.x, curr_position.y])

        closest_idx = self._get_closest_point_idx_on_path(self.reference_positions, curr_position_arr)
        lookahead_dist = self._lookahead_dist + self._lookahead_time * curr_velocity

        vel_index = closest_idx
        total_dist = 0
        while vel_index < len(self.reference_positions)-1:
            last_pos = self.reference_positions[vel_index]
            vel_index += 1
            new_pos = self.reference_positions[vel_index]
            dist_to_last = math.sqrt((last_pos[0] - new_pos[0])**2 + (last_pos[1] - new_pos[1])**2)
            total_dist += dist_to_last
            if total_dist >= lookahead_dist:
                break

        if self._verbose:
            self._logger.info("Nearest index: " + str(closest_idx) + ", lookahead index: " + str(vel_index))
        
        return self.reference_velocities[vel_index]
    
    def get_velocity_for_current_state(self, curr_position) -> Optional[float]:
        """
        Gets velocity from velocity profile for a given position and velocity.

        :param curr_position: current position of the vehicle
        :return: velocity for the current state
        """
        curr_position_arr = np.array([curr_position.x, curr_position.y])
        closest_idx = self._get_closest_point_idx_on_path(self.reference_positions, curr_position_arr)
        vel_index = closest_idx

        return self.reference_velocities[vel_index]

    def get_behavior_velocity_for_current_state(self, curr_position) -> Optional[float]:
        """
        Gets behavior velocity from velocity profile for a given position and velocity.

        :param curr_position: current position of the vehicle
        :return: behavior velocity for the current state
        """
        curr_position_arr = map2utm(self.origin_transformation, curr_position)
        closest_idx = self._get_closest_point_idx_on_path(self.input_path, curr_position_arr)
        vel_index = closest_idx
        return self.velocity_profile_data[vel_index]

    def keep_lane_callback(self, msg: Bool) -> None:
        """
        Call back function which subscribes to keep lane boolean.
        
        :param msg: Bool message
        """
        if msg.data is True:
            self.blackboard.modules.lane_keeping.inputs.ros_condition = True
        
        if msg.data is False:
            self.blackboard.modules.lane_keeping.inputs.ros_condition = False

    @staticmethod
    def _get_closest_point_idx_on_path(path: np.ndarray, position: np.ndarray) -> int:
        """
        Get index of closest point on path to a given position.

        :param path: 2D ndarray with Euclidean (x, y) positions of a path
        :param single: Euclidean point (x, y) given as a ndarray
        :return: idx of closest point on path
        """
        dist = np.linalg.norm(path-position, axis=1)
        closest_idx = np.argmin(dist)
        return closest_idx

    def set_reference_path(self, reference_path: np.ndarray = None, coordinate_system: CoordinateSystem = None):
            """
            Automatically creates a curvilinear coordinate system from a given reference path or sets a given
            curvilinear coordinate system for the planner to use
            :param reference_path: reference path as polyline
            :param coordinate_system: given CoordinateSystem object which is used by the planner
            """
            if coordinate_system is None:
                assert reference_path is not None, '<set reference path>: Please provide a reference path OR a ' \
                                                'CoordinateSystem object to the planner.'
                self._co: CoordinateSystem = CoordinateSystem(reference_path)
            else:
                assert reference_path is None, '<set reference path>: Please provide a reference path OR a ' \
                                            'CoordinateSystem object to the planner.'
                self._co: CoordinateSystem = coordinate_system
