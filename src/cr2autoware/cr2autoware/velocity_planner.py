import os
import numpy as np

from cr2autoware.utils import orientation2quaternion

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Twist, Quaternion
from autoware_auto_planning_msgs.msg import PathWithLaneId, Path, PathPoint, PathPointWithLaneId, TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory
from commonroad_dc.geometry.util import compute_orientation_from_polyline

class VelocityPlanner():
    """
    VelocityPlanner sends the planned reference path to the Autoware Velocity Planner and retrieve velocity information
    The Autoware gets its path information from /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id
    and publishes its results to /planning/scenario_planning/lane_driving/behavior_planning/path
    """
    def __init__(self, detailed_log, logger):
        self.detailed_log = detailed_log
        self.logger = logger

        # variable indicates if velocity planning for latest published route is completed
        self.velocity_planning_completed = False
        self.point_list = None
        self.velocities = None

    def set_publisher(self, pub):
        self.pub = pub

    def send_reference_path(self, point_list):

        if self.detailed_log:
            self.logger.info("Preparing velocity planner message...")

        self.velocity_planning_completed = False

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

        if self.detailed_log:
            self.logger.info("Velocity planner message published!")

    def smoothed_trajectory_callback(self, msg: AWTrajectory):
        if self.detailed_log:
            self.logger.info("Smoothed AW Trajectory received!")

        point_list = []
        velocity_list = []
        # get velocities for each point of the reference path
        for point in msg.points:
            point_list.append(point.pose.position)
            velocity_list.append(point.longitudinal_velocity_mps)

        self.point_list = point_list
        self.velocities = velocity_list
        self.velocity_planning_completed = True

    # get the velocities belonging to the current reference path
    def get_reference_velocities(self):
        return self.point_list, self.velocities

    """ Connection to Autoware Velocity Planner Module
    # send Autoware message /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id of type PathWithLaneId
    # point_list: list of points
    def send_reference_path(self, point_list):

        if self.detailed_log:
            self.logger.info("Preparing velocity planner message...")

        self.velocity_planning_completed = False

        path = PathWithLaneId()
        path.header.frame_id = "map"

        # transform point_list to poses
        path_points_with_lane_ids = []
        for point in point_list:
            pose = Pose()
            pose.position = point
            pose.orientation = Quaternion() # TODO set correct orientation x, y, z, w

            path_point = PathPoint()
            path_point.longitudinal_velocity_mps = 30.0
            path_point.pose = pose # longitudinal_velocity_mps, lateral_velocity_mps, heading_rate_rps set to 0 and is_final set to False by default

            pp = PathPointWithLaneId()
            pp.point = path_point
            pp.lane_ids = [1] # TODO retrieve lane ids from lanelet2 file
            path_points_with_lane_ids.append(pp)

        path.points = path_points_with_lane_ids
        # in theory, PathWithLaneIds also possesses a left_bound and right_bound point list, but they aren't used by the autoware velocity planner
        # self.pub.publish(path)

        if self.detailed_log:
            self.logger.info("Velocity planner message published!")

    # this can be used to debug (because ros2 topic echo is not possible with this message type)
    def test_callback(self, msg: PathWithLaneId):
        if self.detailed_log:
            self.logger.info("Path with lane id received!")
            #self.logger.info("MSG: " + str(msg))

        if self.debug1:
            self.debug1 = False
            debug_path = "/home/drivingsim/autoware_tum/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/debug"
            file = os.path.join(debug_path, "pathwithlaneid.txt")
            with open(file, 'w+') as output:
                output.write(str(msg))
            self.logger.info("Debug 1 complete!")

    def path_with_velocity_callback(self, msg: Path):
        if self.detailed_log:
            self.logger.info("Path with velocity information received!")
        #    self.logger.info("MSG: " + str(msg))
        self.velocity_planning_completed = True

        if self.debug2:
            self.debug2 = False
            debug_path = "/home/drivingsim/autoware_tum/autoware/src/universe/autoware.universe/planning/tum_commonroad_planning/dfg-car/src/cr2autoware/debug"
            file = os.path.join(debug_path, "path.txt")
            with open(file, 'w+') as output:
                output.write(str(msg))
            self.logger.info("Debug 2 complete!")
    """

    # indicates if velocity planning for latest published route is completed
    def get_is_velocity_planning_completed(self):
        return self.velocity_planning_completed