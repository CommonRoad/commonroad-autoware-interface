from geometry_msgs.msg import Pose, Twist, Quaternion
from autoware_auto_planning_msgs.msg import PathWithLaneId, Path, PathPoint, PathPointWithLaneId

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

    def set_publisher(self, pub):
        self.pub = pub

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
            path_point.pose = pose # longitudinal_velocity_mps, lateral_velocity_mps, heading_rate_rps set to 0 and is_final set to False by default

            pp = PathPointWithLaneId()
            pp.point = path_point
            pp.lane_ids = [1] # TODO retrieve lane ids from lanelet2 file
            path_points_with_lane_ids.append(pp)

        path.points = path_points_with_lane_ids
        # in theory, PathWithLaneIds also possesses a left_bound and right_bound point list, but they aren't used by the autoware velocity planner
        self.pub.publish(path)

        if self.detailed_log:
            self.logger.info("Velocity planner message published!")

    # this can be used to debug (because ros2 topic echo is not possible with this message type)
    def test_callback(self, msg: PathWithLaneId):
        pass
        #if self.detailed_log:
        #    self.logger.info("Path with lane id received!")
        #    self.logger.info("MSG: " + str(msg))

    def path_with_velocity_callback(self, msg: Path):
        if self.detailed_log:
            self.logger.info("Path with velocity information received!")
            self.logger.info("MSG: " + str(msg))
        self.velocity_planning_completed = True

    # indicates if velocity planning for latest published route is completed
    def get_is_velocity_planning_completed(self):
        return self.velocity_planning_completed