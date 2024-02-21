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
import cr2autoware.common.utils.utils as utils
from cr2autoware.common.utils.transform import orientation2quaternion
from cr2autoware.common.utils.transform import utm2map


class VelocityPlanner:
    """
    Class for velocity planner using the motion velocity smoother node from AW.Universe. The velocity planner converts
    a planned reference path (polyline) to a reference trajectory by velocity information to the path (similar to
    path-velocity-decomposition techniques in motion planning)

    ======== Publishers
    To motion velocity smoother (input): /planning/scenario_planning/scenario_selector/trajectory

    ======== Subscribers
    From motion velocity smoother (output): /planning/scenario_planning/trajectory_smoothed
    """
    def __init__(self, ref_path_pub: Publisher, logger: RcutilsLogger, verbose: bool,
                 lookahead_dist: float, lookahead_time: float):

        # initialize publisher to velocity planner
        self._ref_path_pub = ref_path_pub

        self._verbose = verbose
        self._logger = logger

        if self._verbose:
            self._logger.info("<Velocity Planner>: Initializing velocity planner with lookahead distance "
                              + str(lookahead_dist) + " and lookahead time " + str(lookahead_time))

        # variable indicates if velocity planning for latest published route is completed
        self._is_velocity_planning_completed = False

        # init reference trajectory (ref path with velocity)
        # reference trajectory is a (n x 3) numpy array, where each row contains x, y, v for a certain
        # point on the reference trajectory
        # Coordinates in AW map frame
        self._reference_trajectory: Optional[np.ndarray] = None

        # init tail (part of ref path behind goal position)
        # Coordinates in AW map frame
        self._tail = None

        # lookahead distance and time
        self._lookahead_dist: float = lookahead_dist
        self._lookahead_time: float = lookahead_time

    @property
    def reference_trajectory(self) -> Optional[np.ndarray]:
        """
        Computed reference trajectory after velocity planning
        Coordinates in AW map frame
        """
        return self._reference_trajectory

    @property
    def reference_positions(self) -> Optional[np.ndarray]:
        """
        Reference trajectory positions
        Coordinates in AW map frame
        """
        if self._reference_trajectory is None:
            return None
        else:
            return self._reference_trajectory[:, 0:2]

    @property
    def reference_velocities(self) -> Optional[np.ndarray]:
        """Reference trajectory velocities"""
        if self._reference_trajectory is None:
            return None
        else:
            return self._reference_trajectory[:, 2]

    @property
    def is_velocity_planning_completed(self):
        """indicates if velocity planning for latest published route is completed"""
        return self._is_velocity_planning_completed

    def plan(self, reference_path: np.ndarray, goal_pos: np.ndarray, origin_transformation: List):
        """
        Call velocity planner
        Computes a velocity profile for a given reference path
        Resulting reference trajectory (i.e., path with velocity information) is stored
        :param reference_path in CR coordinates
        :param goal_pos in CR coordinates
        :param origin_transformation translation of origin between CR and AW map coordinates
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
            _tmp = tail_orig[i] - np.array(origin_transformation)
            tail_mod.append(_tmp)
        self._tail = np.array(tail_mod)
        
        if self._verbose:
            self._logger.info("<Velocity planner>: Goal coordinates: " + str(goal_pos))
        
        if self._verbose:
            self._logger.info("<Velocity planner>: Tail coordinates: " + str(self._tail))

        # Call _pub_ref_path
        self._pub_ref_path(input_path, origin_transformation)

    def _prepare_traj_msg(self, input_path: np.ndarray, origin_transformation: List) -> AWTrajectory:
        """converts reference path to AWTrajectory message type for publishing to Motion Velocity Smoother"""
        if self._verbose:
            self._logger.info("<Velocity planner>: Preparing reference path message for motion velocity smoother")

        # AW Trajectory message
        traj = AWTrajectory()
        traj.header.frame_id = "map"

        # compute orientations
        orientations = compute_orientation_from_polyline(input_path)

        for i in range(0, len(input_path)):
            new_point = TrajectoryPoint()
            new_point.time_from_start = Duration(sec=0, nanosec=i)
            new_point.pose.position = utm2map(origin_transformation, input_path[i])
            new_point.pose.orientation = orientation2quaternion(orientations[i])
            new_point.longitudinal_velocity_mps = 20.0
            new_point.acceleration_mps2 = 0.0
            traj.points.append(new_point)

        return traj

    def _pub_ref_path(self, input_path: np.ndarray, origin_transformation: List):
        """Publishes reference path to Motion Velocity Smoother"""
        traj_msg = self._prepare_traj_msg(input_path, origin_transformation)

        self._ref_path_pub.publish(traj_msg)

        if self._verbose:
            self._logger.info("<Velocity planner>: Reference path published to motion velocity smoother.")

    def smoothed_trajectory_callback(self, msg: AWTrajectory):
        """Call back function which subscribes to output of motion velocity smoother"""
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

        positions_arr = np.concatenate((np.array(point_list), self._tail), axis=0)
        velocities_arr = np.array(velocity_list + zeros)

        _len_vel_arr = len(velocities_arr)

        # get reference trajectory
        self._reference_trajectory = np.concatenate((positions_arr, velocities_arr.reshape(_len_vel_arr, 1)), axis=1)
        self._is_velocity_planning_completed = True

    def get_lookahead_velocity_for_current_state(self, curr_position, curr_velocity) -> Optional[float]:
        """Gets velocity from velocity profile with lookahead for a given position and velocity"""
        if not self._is_velocity_planning_completed:
            self._logger.error("<Velocity Planner>: Velocity planning not completed: No velocity can be returned")
            return None

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

    @staticmethod
    def _get_closest_point_idx_on_path(path: np.ndarray, position: np.ndarray) -> int:
        """
        :param path 2D ndarray with Euclidean (x, y) positions of a path
        :param single Euclidean point (x, y) given as a ndarray
        :return idx of closest point on path
        """
        dist = np.linalg.norm(path-position, axis=1)
        closest_idx = np.argmin(dist)
        return closest_idx
