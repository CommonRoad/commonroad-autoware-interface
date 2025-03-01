# standard imports
import math
from typing import List, Optional

# third party imports
import numpy as np

# ROS imports
from rclpy.publisher import Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger

# ROS message imports
from builtin_interfaces.msg import Duration

# Autoware.Auto message imports
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory

# commonroad-dc imports
from commonroad_dc.geometry.util import compute_orientation_from_polyline

# cr2autoware imports
from cr2autoware.common.utils.transform import orientation2quaternion
from cr2autoware.common.utils.transform import utm2map
from cr2autoware.interfaces.implementation.behavior_tree.implementation.behavior_tree import BehaviorTree
from cr2autoware.common.configuration import BehaviorPlannerParams, CR2AutowareParams
from cr2autoware.handlers.scenario_handler import ScenarioHandler
from commonroad_rp.utility.utils_coordinate_system import CoordinateSystem

from cr2autoware.handlers.ego_vehicle_handler import EgoVehicleState
from commonroad.scenario.scenario import Scenario
from scipy.interpolate import interp1d

import py_trees
from visualization_msgs.msg import MarkerArray


class BehaviorPlanner:
    """
    TODO:**WIP**
    Class for behavior planner using the motion velocity smoother node from AW.Universe. 
    
    The velocity planner converts a planned reference path (polyline) to a reference trajectory by 
    velocity information to the path (similar to path-velocity-decomposition techniques in motion planning)

    ---------------
    **Publishers:**

    * _ref_path_pub:
        * Description: Publishes reference path with velocity profile to motion velocity smoother
        * Topic: `/planning/scenario_planning/trajectory_smoothed`
        * Message Type: `autoware_auto_planning_msgs/Trajectory`

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
    def __init__(self, ref_path_pub: Publisher, traffic_light_marker_pub: Publisher, lateral_clearance_pub: Publisher, lateral_clearance_obstacles_pub: Publisher, logger: RcutilsLogger, verbose: bool,
                 lookahead_dist: float, lookahead_time: float, origin_transformation: List, global_params: CR2AutowareParams, scenario_handler: ScenarioHandler) -> None:
        """
        TODO:**WIP**
        Constructor for VelocityPlanner class.

        :param ref_path_pub: ROS2 node publisher for reference path
        :param logger: ROS2 node logger
        :param verbose: Flag for verbose logging
        :param lookahead_dist: Lookahead distance for velocity planning
        :param lookahead_time: Lookahead time for velocity planning
        :param params: Parameters for behavior planner
        :param origin_transformation: translation of origin between CR and AW map coordinates
        """

        # initialize publisher to behavior planner
        self._ref_path_pub = ref_path_pub
        self._traffic_light_marker_pub = traffic_light_marker_pub
        self._lateral_clearance_pub = lateral_clearance_pub
        self._lateral_clearance_obstacles_pub = lateral_clearance_obstacles_pub

        self._verbose = verbose
        self._logger = logger

        if self._verbose:
            self._logger.info("<Behavior Planner>: Initializing planner with lookahead distance "
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
        # Register keys for ROS Publisher
        self.blackboard.register_key("/modules/traffic_lights/outputs/traffic_light_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.traffic_lights.outputs.traffic_light_marker_array = MarkerArray()
        self.blackboard.register_key("/modules/lateral_clearance/outputs/lateral_clearance_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_marker_array = MarkerArray()
        self.blackboard.register_key("/modules/lateral_clearance/outputs/lateral_clearance_obstacles_marker_array", access=py_trees.common.Access.WRITE)
        self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_obstacles_marker_array = MarkerArray()

        # Initialize the Behavior Tree
        self.behavior_tree = BehaviorTree(self._logger, self._verbose)

        # init tail (part of ref path behind goal position)
        # Coordinates in AW map frame
        self._tail = None

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
        return self._reference_trajectory
    
    # @reference_trajectory.setter
    # def reference_trajectory(self, trajectory: np.ndarray) -> None:
    #     """
    #     Setter for reference trajectory.

    #     :param trajectory: reference trajectory
    #     """
    #     assert isinstance(trajectory, np.ndarray), "Reference trajectory should be a numpy array"
    #     assert trajectory.shape[0] > 0, "Reference trajectory should have at least one point"
    #     assert trajectory.shape[1] == 3, "Reference trajectory should have 3 columns (x, y, v)"

    #     self._reference_trajectory = trajectory

    @property
    def reference_positions(self) -> Optional[np.ndarray]:
        """
        Reference trajectory positions.

        Coordinates in AW map frame.

        :return: reference trajectory positions
        """
        if self._reference_trajectory is None:
            return None
        else:
            return self._reference_trajectory[:, 0:2]

    @property
    def reference_velocities(self) -> Optional[np.ndarray]:
        """
        Reference trajectory velocities.

        :return: reference trajectory velocities
        """
        if self._reference_trajectory is None:
            return None
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

# Copied from Reactive Planner
    @property
    def path_in_curvilinear(self) -> np.ndarray:
        return self._co.ref_pos

# Copied from Reactive Planner
    @property
    def path_orientation(self) -> np.ndarray:
        return self._co.ref_theta

    @property
    def output_d_min(self) -> float:
        return self.behavior_tree.outputs.d_min

    @property
    def output_d_max(self) -> float:
        return self.behavior_tree.outputs.d_max

    def plan(self, reference_path: np.ndarray, goal_pos: np.ndarray, scenario: Scenario, current_state: EgoVehicleState) -> None:
        """
        Calls behavior planner.

        Computes a velocity profile for a given reference path.
        Resulting reference trajectory (i.e., path with velocity information) is stored.


        :param reference_path: in CR coordinates
        :param goal_pos: in CR coordinates
        """
        self._is_velocity_planning_completed = False

        if self._verbose:
            self._logger.info("<Velocity planner>: Planning velocity profile")

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

        # Call Behavior Planner
        self.behavior_tree.plan()

        self.behavior_tree.prepare_output()

        velocity_path = self.convert_velocity_profile(self.path_in_cartesian, self.behavior_tree.velocity_profile, input_path)

        # Call _pub_ref_path
        self._pub_ref_path(input_path, velocity_path, self.origin_transformation)

        # Publish traffic light marker
        self._pub_traffic_light_marker()

        # Publish lateral clearance velocity adjuster marker
        self._pub_lateral_clearance_marker()
    
    def _behavior_planner(self, input_path: np.ndarray, origin_transformation: List) -> np.ndarray:
        """
        Behavior planner for velocity planning.

        :param input_path: reference path in CR coordinates
        :param origin_transformation: translation of origin between CR and AW map coordinates
        :return: velocity profile
        """
        if self._verbose:
            self._logger.info("<Behavior planner>: Behavior planner for velocity planning")
        
        velocity_update = np.zeros(len(input_path))
        # set velocity to 10 m/s
        velocity_update[:] = 20.0

        velocity_update = self.behavior_tree.plan(input_path, origin_transformation)

        return velocity_update


    def _prepare_traj_msg(self, input_path: np.ndarray, velocity_path: np.ndarray, origin_transformation: List) -> AWTrajectory:
        """
        Converts reference path to AWTrajectory message type for publishing to Motion Velocity Smoother.
        
        :param input_path: reference path in CR coordinates
        :param velocity_path: velocity profile for reference path
        :param origin_transformation: translation of origin between CR and AW map coordinates
        :return: AWTrajectory message
        """
        if self._verbose:
            self._logger.info("<Velocity planner>: Preparing reference path message for motion velocity smoother")

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
            self._logger.info("<Velocity planner>: Reference path published to motion velocity smoother.")
    
    def _pub_traffic_light_marker(self) -> None:
        self._traffic_light_marker_pub.publish(self.blackboard.modules.traffic_lights.outputs.traffic_light_marker_array)

    def _pub_lateral_clearance_marker(self) -> None:
        self._lateral_clearance_pub.publish(self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_marker_array)
        self._lateral_clearance_obstacles_pub.publish(self.blackboard.modules.lateral_clearance.outputs.lateral_clearance_obstacles_marker_array)

    def convert_velocity_profile(self, source_path: np.ndarray, source_velocity_profile: np.ndarray, target_path: np.ndarray) -> np.ndarray:
        """
        Convert and Interpolate a velocity profile from a source_path to a velocity_profile for a target_path.
        
        :param velocity_profile: velocity profile from behavior planner
        :return: velocity profile for reference path
        """
        def compute_cumulative_distance(path):
            """Compute cumulative distances for a path."""
            diffs = np.diff(path, axis=0)
            segment_lengths = np.linalg.norm(diffs, axis=1)
            return np.concatenate(([0], np.cumsum(segment_lengths)))

        # Compute cumulative distances
        source_distances = compute_cumulative_distance(source_path)
        self._logger.debug("Lenght source distances: " + str(len(source_distances)))
        target_distances = compute_cumulative_distance(target_path)
        self._logger.debug("Length target distances: " + str(len(target_distances)))

        # Create an interpolation function
        velocity_interp = interp1d(source_distances, source_velocity_profile, kind='linear', fill_value="extrapolate")

        # Interpolate velocities for path2
        velocity_path = velocity_interp(target_distances)
        
        self._logger.debug("Source velocities: " + str(source_velocity_profile))
        self._logger.debug("Velocity path: " + str(velocity_path))

        return velocity_path

    def smoothed_trajectory_callback(self, msg: AWTrajectory) -> None:
        """
        Call back function which subscribes to output of motion velocity smoother.
        
        :param msg: AWTrajectory message with velocity profile
        """
        if self._tail is None:
            return
        
        if self._verbose:
            self._logger.info("<Velocity Planner>: Path with velocity profile received from motion velocity smoother")

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

    def get_lookahead_velocity_for_current_state(self, curr_position, curr_velocity) -> Optional[float]:
        """
        Gets velocity from velocity profile with lookahead for a given position and velocity.
        
        :param curr_position: current position of the vehicle
        :param curr_velocity: current velocity of the vehicle
        :return: velocity with lookahead
        :raises _logger.error: if velocity planning is not completed
        """

        curr_position_arr = np.array([curr_position.x, curr_position.y])

        self._logger.debug("Current position: " + str(curr_position_arr))
        self._logger.debug("reference_positions: " + str(self.reference_positions))

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

    # Copied from Reactive Planner
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
