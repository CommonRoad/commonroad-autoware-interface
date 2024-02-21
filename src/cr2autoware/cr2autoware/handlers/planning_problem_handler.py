# standard imports
import glob
from pathlib import Path
import typing
from typing import List
from typing import Optional
from typing import Tuple

# third party imports
import numpy as np

# ROS message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

# ROS imports
from rclpy.publisher import Publisher

# commonroad imports
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Circle
from commonroad.geometry.shape import Polygon
from commonroad.geometry.shape import Rectangle
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario

# cr2autoware imports
from .base import BaseHandler
import cr2autoware.common.utils.utils as utils

# Avoid circular imports
if typing.TYPE_CHECKING:
    from cr2autoware.cr2autoware import Cr2Auto


# TODO: At the moment this class will only load the first planning problem from the CommonRoad file.
# It does not handle updates to the planning problem triggered by new goal state msgs from Autoware yet.
# -> Migrate this functionality from cr2autoware.py
class PlanningProblemHandler(BaseHandler):
    """Handles communication with autoware for CommonRoad PlanningProblem relevant data.

    Keeps an up to date state of the current planning problem in CommonRoad format.
    """

    # Constants and parameters
    MAP_PATH: str

    _planning_problem: Optional[PlanningProblem]

    # Publishers
    _INITIAL_POSE_PUBLISHER: Publisher
    _GOAL_POSE_PUBLISHER: Publisher
    _GOAL_REGION_PUBLISHER: Publisher

    def __init__(self, node: "Cr2Auto", scenario: Scenario, origin_transformation: List[float]):
        # init base class
        super().__init__(node=node,
                         logger=node.get_logger().get_child("planning_problem_handler"),
                         verbose=node.verbose)

        # Get parameters from the node
        self._init_parameters()

        # initialize subscriptions to relevant topics
        self._init_subscriptions()

        # initialize publishers
        self._init_publishers()

        # Loading the planning problem from CommonRoad file (if available)
        self._planning_problem = self._load_planning_problem(self.MAP_PATH)

        # Publish initial states
        if self._planning_problem is not None:
            self.publish_initial_states(self._planning_problem, scenario, origin_transformation)

    def _init_parameters(self):
        self.MAP_PATH = self._get_param("general.map_path").string_value
        if not Path(self.MAP_PATH).exists():
            raise ValueError("Can't find given map path: %s" % self.MAP_PATH)

    def _init_subscriptions(self):
        pass

    def _init_publishers(self):
        # publish initial pose
        self._INITIAL_POSE_PUBLISHER = self._node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )
        # publish goal pose
        self._GOAL_POSE_PUBLISHER = self._node.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            1,
        )
        # publish goal marker
        self._GOAL_REGION_PUBLISHER = self._node.create_publisher(
            MarkerArray, "/goal_region_marker_array", 1
        )

    def _load_planning_problem(self, map_path: str) -> Optional[PlanningProblem]:
        """Load the planning problem from the CommonRoad file."""
        xml_files = glob.iglob(Path(map_path).joinpath("*.[xX][mM][lL]").as_posix())
        try:
            commonroad_map_file = next(xml_files)
        except StopIteration:
            self._logger.info(
                f"Could not find CommonRoad scenario file in {map_path}\n "
                "No planning problem will be loaded."
            )
            return
        commonroad_reader = CommonRoadFileReader(commonroad_map_file)
        planning_problem_set = commonroad_reader.open()[1]
        # We only care about the first planning problem
        list_pp = list(planning_problem_set.planning_problem_dict.values())
        if list_pp:
            return list_pp[0]
        else:
            return None

    @property
    def planning_problem(self) -> Optional[PlanningProblem]:
        """Get the planning problem in CommonRoad format."""
        if self._planning_problem is None:
            self._logger.info("No planning problem loaded.")
        return self._planning_problem

    @planning_problem.setter
    def planning_problem(self, planning_problem: PlanningProblem):
        self._planning_problem = planning_problem

    def publish_initial_states(
        self,
        planning_problem: PlanningProblem,
        scenario: Scenario,
        origin_transformation: List[float],
    ):
        """Publish the initial state, the goal, and the goal region from the commonroad scenario to Autoware."""
        self._publish_initial_pose(planning_problem, origin_transformation)
        self._publish_initial_goal(planning_problem, scenario, origin_transformation)

    def _publish_initial_pose(
        self,
        planning_problem: PlanningProblem,
        origin_transformation: List[float],
    ):
        # publish the initial pose to the autoware PoseWithCovarianceStamped
        initial_state = planning_problem.initial_state
        initial_pose_msg = PoseWithCovarianceStamped()
        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        header.frame_id = "map"
        initial_pose_msg.header = header
        pose = Pose()
        pose.position = utils.utm2map(origin_transformation, initial_state.position)
        pose.orientation = utils.orientation2quaternion(initial_state.orientation)
        initial_pose_msg.pose.pose = pose
        self._INITIAL_POSE_PUBLISHER.publish(initial_pose_msg)
        self._logger.info(
            f"Published initial pose with position {pose.position.x}, {pose.position.y}"
        )

    def _publish_initial_goal(
        self,
        planning_problem: PlanningProblem,
        scenario: Scenario,
        origin_transformation: List[float],
    ):
        # Don't publish if there is no goal
        if len(planning_problem.goal.state_list) <= 0:
            return

        # We only care about the first goal state
        goal_state = planning_problem.goal.state_list[0]

        position, orientation = None, None

        try:
            goal_position = goal_state.position  # type: ignore
        except AttributeError as e:
            self._logger.warning(f"Goal state of type {type(goal_state)} has no position.")
            self._logger.debug(str(e))
            return

        # If the goal is a lanelet, this is stored as a ShapeGroup
        if isinstance(goal_position, ShapeGroup):
            position, orientation = self._get_goal_from_lanelet(goal_position, scenario)
            if position is None or orientation is None:
                self._logger.warning("Failed to get goal position from lanelet.")
                return

        # If orientation cannot be inferred from the lanelet, try to get it from the goal state
        if orientation is None:
            try:
                orientation = utils.orientation2quaternion(goal_state.orientation)  # type: ignore
            except AttributeError as e:
                self._logger.warning(f"Goal state of type {type(goal_state)} has no orientation.")
                self._logger.debug(str(e))
                return

        # If position cannot be inferred from the lanelet, get it from the goal position
        if position is None:
            if isinstance(goal_position, (Rectangle, Polygon, Circle)):
                position = goal_position.center
            elif isinstance(goal_position, np.ndarray):
                position = goal_position
            else:
                self._logger.warning("Failed to get goal position. ")
                return

        goal_msg = PoseStamped()
        header = Header()
        header.stamp = self._node.get_clock().now().to_msg()
        header.frame_id = "map"
        goal_msg.header = header
        pose = Pose()
        pose.position = utils.utm2map(origin_transformation, position)
        pose.orientation = orientation
        goal_msg.pose = pose
        self._GOAL_POSE_PUBLISHER.publish(goal_msg)
        self._logger.info(
            f"Published goal pose with position {pose.position.x}, {pose.position.y} "
            f" and orientation {pose.orientation}"
        )

        # visualize goal region(s)
        marker_id = 0xFFFF  # just a value
        goal_region_msgs = MarkerArray()
        for goal_state in planning_problem.goal.state_list:
            if not hasattr(goal_state, "position"):
                continue
            shapes = []
            goal_position = goal_state.position  # type: ignore
            if isinstance(goal_position, ShapeGroup):
                for shape in goal_position.shapes:
                    shapes.append(shape)
            else:
                shapes = [goal_position]

            for shape in shapes:
                marker = utils.create_goal_region_marker(shape, origin_transformation)
                marker.id = marker_id
                goal_region_msgs.markers.append(marker)  # type: ignore
                marker_id += 1

        self._GOAL_REGION_PUBLISHER.publish(goal_region_msgs)
        self._logger.info("Published the goal region")

    def _get_goal_from_lanelet(
        self, goal_position: ShapeGroup, scenario: Scenario
    ) -> Tuple[Optional[np.ndarray], Optional[Quaternion]]:
        shape = goal_position.shapes[0]
        assert isinstance(shape, Polygon)

        # starting middle point on the goal lanelet
        position = shape.center
        possible_lanelets = scenario.lanelet_network.find_lanelet_by_shape(shape)
        for pl in possible_lanelets:  # find an appropriate lanelet
            try:
                orientation = utils.orientation2quaternion(
                    scenario.lanelet_network.find_lanelet_by_id(pl).orientation_by_position(
                        position
                    )
                )
                return position, orientation
            except AssertionError:
                continue
        return (None, None)
