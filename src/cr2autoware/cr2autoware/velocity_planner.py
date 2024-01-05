# third party imports
import numpy as np
import math

# ROS message imports
from builtin_interfaces.msg import Duration

# Autoware.Auto message imports
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory

# commonroad-dc imports
from commonroad_dc.geometry.util import compute_orientation_from_polyline

# cr2autoware imports
from cr2autoware.utils import orientation2quaternion


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
    def __init__(self, verbose, logger, lookahead_dist, lookahead_time):

        self.verbose = verbose
        self.logger = logger

        if self.verbose:
            self.logger.info("Initializing velocity planner with lookahead distance " + str(lookahead_dist) +
                             " and lookahead time " + str(lookahead_time))

        # variable indicates if velocity planning for latest published route is completed
        self.velocity_planning_completed = False
        self.point_list = None
        self.velocities = None
        self.tail = None

        self.lookahead_dist = lookahead_dist
        self.lookahead_time = lookahead_time

    def set_publisher(self, pub):
        self.pub = pub

    # convert reference path to a trajectory and publish it to the motion velocity smoother module
    def send_reference_path(self, input_point_list, goal_pos):

        if self.verbose:
            self.logger.info("Preparing velocity planner message...")

        self.velocity_planning_completed = False

        # Shorten reference path so that it ends at the goal position
        goal_id = self._get_pathpoint_near_aw_position(input_point_list, goal_pos)
        self.tail = input_point_list[goal_id+1:]
        point_list = input_point_list[:goal_id+1]

        # compute orientations
        polyline = np.array([(p.x, p.y) for p in point_list])
        orientations = compute_orientation_from_polyline(polyline)

        traj = AWTrajectory()
        traj.header.frame_id = "map"

        for i in range(0, len(point_list)):
            new_point = TrajectoryPoint()
            new_point.time_from_start = Duration(sec=0, nanosec=i)
            new_point.pose.orientation = orientation2quaternion(orientations[i])
            new_point.pose.position = point_list[i]
            new_point.longitudinal_velocity_mps = 20.0
            new_point.acceleration_mps2 = 0.0
            traj.points.append(new_point)

        self.pub.publish(traj)

        if self.verbose:
            self.logger.info("Velocity planner message published!")

    def smoothed_trajectory_callback(self, msg: AWTrajectory):

        if self.verbose:
            self.logger.info("Smoothed AW Trajectory received!")

        point_list = []
        velocity_list = []
        # get velocities for each point of the reference path
        for point in msg.points:
            point_list.append(point.pose.position)
            velocity_list.append(point.longitudinal_velocity_mps)

        # append tail of reference trajectory
        zeros = [0] * len(self.tail)

        self.point_list = point_list + self.tail
        self.velocities = velocity_list + zeros
        self.velocity_planning_completed = True

    # get the velocities belonging to the current reference path
    def get_reference_velocities(self):
        return self.point_list, self.velocities

    # find the closest reference path point for a given position and return it's velocity
    def get_velocity_at_aw_position(self, position):
        v = self.velocities[self._get_pathpoint_near_aw_position(self.point_list, position)]
        return v

    # get velocity with lookahead: lookahead_dist + lookahead_time * current_speed
    def get_velocity_at_aw_position_with_lookahead(self, position, vehicle_speed):
        nearest_index = self._get_pathpoint_near_aw_position(self.point_list, position)
        lookahead_dist = self.lookahead_dist + self.lookahead_time * vehicle_speed

        vel_index = nearest_index
        total_dist = 0
        while vel_index < len(self.point_list)-1:
            last_pos = self.point_list[vel_index]
            vel_index += 1
            new_pos = self.point_list[vel_index]
            dist_to_last = math.sqrt((last_pos.x - new_pos.x)**2 + (last_pos.y - new_pos.y)**2)
            total_dist += dist_to_last
            if total_dist >= lookahead_dist:
                break

        self.logger.info("Nearest index: " + str(nearest_index) + ", lookahead index: " + str(vel_index))
        
        return self.velocities[vel_index]

    # get closest pathpoint to a given position (in Autoware coordinate system)
    def _get_pathpoint_near_aw_position(self, list, position):
        min_dist = None
        min_i = 0
        for i, point in enumerate(list):
            dist = math.sqrt((point.x - position.x)**2 + (point.y - position.y)**2)
            if min_dist == None or dist <= min_dist:
                min_dist = dist
                min_i = i
        return min_i

    # indicates if velocity planning for latest published route is completed
    def get_is_velocity_planning_completed(self):
        return self.velocity_planning_completed