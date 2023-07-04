# general imports
from copy import deepcopy
import os
import traceback
from typing import Optional

# Autoware message imports
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.geometry.shape import ShapeGroup
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.obstacle import ObstacleType

# commonroad imports
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.scenario import Tag
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_dc.geometry.util import resample_polyline
from commonroad_route_planner.route_planner import RoutePlanner

# ROS message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np

# ROS imports
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from scipy.interpolate import splev
from scipy.interpolate import splprep
from std_msgs.msg import Header
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import yaml

from cr2autoware.configuration import RPParams
from cr2autoware.ego_vehicle_handler import EgoVehicleHandler
from cr2autoware.planning_problem_handler import PlanningProblemHandler
from cr2autoware.rp_interface import RP2Interface
from cr2autoware.scenario_handler import ScenarioHandler

# local imports
from cr2autoware.tf2_geometry_msgs import do_transform_pose
from cr2autoware.trajectory_logger import TrajectoryLogger
import cr2autoware.utils as utils
from cr2autoware.velocity_planner import VelocityPlanner

matplotlib.use("TkAgg")


class Cr2Auto(Node):
    """Cr2Auto class that is an interface between Autoware and CommonRoad."""

    scenario_handler: ScenarioHandler
    plan_prob_handler: PlanningProblemHandler

    def __init__(self):
        """Construct Cr2Auto class."""
        # ignore typing due to bug in rclpy
        super().__init__(node_name="cr2autoware")  # type: ignore

        # Declare ros parameters
        with open(os.path.dirname(__file__) + "/ros_param.yaml", "r") as stream:
            param = yaml.load(stream, Loader=yaml.Loader)
        for key, value in param.items():
            self.declare_parameter(key, value)

        self.get_logger().info(
            "Map path is: " + self.get_parameter("map_path").get_parameter_value().string_value
        )
        self.get_logger().info(
            "Solution path is: "
            + self.get_parameter("solution_file").get_parameter_value().string_value
        )

        # Init class attributes
        self.write_scenario = self.get_parameter("write_scenario").get_parameter_value().bool_value
        self.callback_group = ReentrantCallbackGroup()  # Callback group for async execution
        self.rnd = None

        self.ego_vehicle_handler = EgoVehicleHandler(self)

        # buffer for static obstacles
        self.last_trajectory = None
        self.PUBLISH_OBSTACLES = (
            self.get_parameter("publish_obstacles").get_parameter_value().bool_value
        )
        self.solution_path = self.get_parameter("solution_file").get_parameter_value().string_value

        self.planning_problem_set = None
        self.route_planned = False
        self.planner_state_list = None
        self.is_computing_trajectory = False  # stop update scenario when trajectory is compuself.declare_parameter('velocity_planner.lookahead_dist', 2.0)
        self.ego_vehicle_handler.create_ego_vehicle_info()  # compute ego vehicle width and height
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames
        self.aw_state = AutowareState()
        self.engage_status = False
        self.reference_path_published = False
        self.new_initial_pose = False
        self.new_pose_received = False
        # vars to save last messages
        # https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl
        self.last_msg_aw_state = 1  # 1 = initializing
        self.goal_msgs = []
        self.current_goal_msg = None
        self.last_goal_reached = self.get_clock().now()
        self.reference_path = None

        self.trajectory_logger = TrajectoryLogger(
            None,
            self.get_logger(),
            self.get_parameter("detailed_log").get_parameter_value().bool_value,
        )

        self.scenario_handler = ScenarioHandler(self)
        self.origin_transformation = self.scenario_handler.origin_transformation

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            from rclpy.logging import LoggingSeverity

            self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.current_state_sub = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.current_state_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe initial pose
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.initial_pose_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe goal pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            "/planning/mission_planning/goal",
            self.goal_pose_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe autoware engage
        self.engage_sub = self.create_subscription(
            Engage,
            "/api/external/cr2autoware/engage",
            self.engage_callback,
            1,
            callback_group=self.callback_group,
        )
        # publish goal pose
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            1,
        )
        # publish trajectory
        self.traj_pub = self.create_publisher(
            AWTrajectory, "/planning/scenario_planning/trajectory", 1
        )
        # publish autoware state
        # list of states: https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/AutowareState.idl
        self.aw_state_pub = self.create_publisher(AutowareState, "/autoware/state", 1)
        # publish autoware engage
        self.engage_pub = self.create_publisher(Engage, "/autoware/engage", 1)
        self.vehicle_engage_pub = self.create_publisher(Engage, "/vehicle/engage", 1)
        self.ext_engage_pub = self.create_publisher(Engage, "/api/external/get/engage", 1)
        self.api_engage_pub = self.create_publisher(Engage, "/api/autoware/get/engage", 1)
        # publish route marker
        qos_route_pub = QoSProfile(depth=1)
        qos_route_pub.history = QoSHistoryPolicy.KEEP_LAST
        qos_route_pub.reliability = QoSReliabilityPolicy.RELIABLE
        qos_route_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(
            MarkerArray, "/planning/mission_planning/route_marker", qos_route_pub
        )
        # publish initial state of the scenario
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        # publish goal region(s) of the scenario
        self.goal_region_pub = self.create_publisher(MarkerArray, "/goal_region_marker_array", 1)

        self.plan_prob_handler = PlanningProblemHandler(
            self, self.scenario, self.origin_transformation
        )
        if self.write_scenario:
            self._write_scenario()

        self.set_state(AutowareState.WAITING_FOR_ROUTE)

        self.plan_prob_handler = PlanningProblemHandler(
            self, self.scenario, self.origin_transformation
        )
        if self.write_scenario:
            self._write_scenario()

        self.initialize_velocity_planner()
        self.initialize_trajectory_planner()

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Detailed log is enabled")
            self.get_logger().info("Init complete!")

        self.initialize_mode()

    @property
    def scenario(self) -> Scenario:
        """Get scenario object retrieved from the scenario_handler.

        Caution: Does not trigger an update of the scenario.
        """
        if self.scenario_handler is None:
            raise RuntimeError("Scenario handler not initialized.")
        return self.scenario_handler.scenario

    def initialize_trajectory_planner(self):
        """Define planner according to trajectory_planner_type."""
        self.trajectory_planner_type = (
            self.get_parameter("trajectory_planner_type").get_parameter_value().integer_value
        )
        if self.trajectory_planner_type == 1:  # Reactive planner
            # not used
            # _cur_file_path = os.path.dirname(os.path.realpath(__file__))
            # _rel_path_conf_default = (
            #     self.get_parameter("reactive_planner.default_yaml_folder")
            #     .get_parameter_value()
            #     .string_value
            # )
            # not used
            # dir_conf_default = os.path.join(_cur_file_path, _rel_path_conf_default)
            params = RPParams(self.get_parameter)
            self.trajectory_planner = RP2Interface(
                self.scenario, self.scenario.dt, self.trajectory_logger, params
            )
        else:
            self.get_logger().error("Planner type is not correctly specified!")

        self.set_state(AutowareState.WAITING_FOR_ROUTE)

    def initialize_mode(self):
        """Decide whether planner goes in interactive planner mode or trajectory following mode."""
        if self.solution_path == "":
            # create a timer to run planner
            self.interactive_mode = True
            self.get_logger().info("Starting interactive planning mode...")
            self.timer_solve_planning_problem = self.create_timer(
                timer_period_sec=self.get_parameter("planner_update_time")
                .get_parameter_value()
                .double_value,
                callback=self.solve_planning_problem,
                callback_group=self.callback_group,
            )
        else:
            # follow solution trajectory
            self.interactive_mode = False
            self.get_logger().info("Loading solution trajectory...")

            self.follow_solution_trajectory()

            self.timer_follow_trajectory_mode_update = self.create_timer(
                timer_period_sec=self.get_parameter("planner_update_time")
                .get_parameter_value()
                .double_value,
                callback=self.follow_trajectory_mode_update,
                callback_group=self.callback_group,
            )

    @property
    def scenario(self) -> Scenario:
        """Get scenario object retrieved from the scenario_handler.

        Caution: Does not trigger an update of the scenario.
        """
        if self.scenario_handler is None:
            raise RuntimeError("Scenario handler not initialized.")
        return self.scenario_handler.scenario

    @property
    def planning_problem(self) -> Optional[PlanningProblem]:
        """Get planning problem object retrieved from the planning problem handler.

        Caution: Does not trigger an update of the planning problem."""
        if self.plan_prob_handler is None:
            raise RuntimeError("Planning problem handler not initialized.")
        return self.plan_prob_handler.planning_problem

    @planning_problem.setter
    def planning_problem(self, planning_problem):
        self.plan_prob_handler.planning_problem = planning_problem

    def solve_planning_problem(self) -> None:
        """Update loop for interactive planning mode and solve planning problem with algorithms offered by commonroad."""
        try:
            # avoid parallel processing issues by checking if a planning problem is already being solved
            if not self.is_computing_trajectory:
                # Compute trajectory
                self.is_computing_trajectory = True
                self.ego_vehicle_handler.update_ego_vehicle()
                self.scenario_handler.update_scenario()
                self.plot_save_scenario()

                # check if initial pose was changed (if true: recalculate reference path)
                if self.new_initial_pose:
                    # check if the current_vehicle_state was already updated (=pose received by current state callback), otherwise wait one planning cycle
                    if not self.new_pose_received:
                        self.is_computing_trajectory = False
                        return

                    self.new_initial_pose = False
                    self.new_pose_received = False
                    self.get_logger().info("Replanning route to goal")

                    # insert current goal into list of goal messages and set route_planned to false to trigger route planning
                    if self.current_goal_msg:
                        self.goal_msgs.insert(0, self.current_goal_msg)
                    self.route_planned = False

                if not self.route_planned:
                    # if currently no active goal, set a new goal (if one exists)
                    try:
                        self._set_new_goal()
                    except Exception:
                        self.get_logger().error(traceback.format_exc())

                if self.route_planned:
                    if not self.velocity_planner.get_is_velocity_planning_completed():
                        self.get_logger().info(
                            "Can't run route planner because interface is still waiting for velocity planner"
                        )
                        self.velocity_planner.send_reference_path(
                            [
                                utils.utm2map(self.origin_transformation, point)
                                for point in self.reference_path
                            ],
                            self.current_goal_msg.pose.position,
                        )
                        self.is_computing_trajectory = False
                        return

                    if not self.reference_path_published:
                        # publish current reference path
                        self._pub_route(*self.velocity_planner.get_reference_velocities())

                    if self.get_state() == AutowareState.DRIVING:
                        # log current position
                        self.trajectory_logger.log_state(self.ego_vehicle_handler.ego_vehicle_state)

                    if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                        self.get_logger().info("Solving planning problem!")

                    if self.trajectory_planner_type == 1:  # Reactive Planner
                        reference_velocity = max(
                            1,
                            self.velocity_planner.get_velocity_at_aw_position_with_lookahead(
                                self.ego_vehicle_handler.current_vehicle_state.pose.pose.position,
                                self.ego_vehicle_handler.ego_vehicle_state.velocity,
                            ),
                        )

                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info("Running reactive planner")
                            self.get_logger().info(
                                "Reactive planner init_state position: "
                                + str(self.ego_vehicle_handler.ego_vehicle_state.position)
                            )
                            self.get_logger().info(
                                "Reactive planner init_state velocity: "
                                + str(max(self.ego_vehicle_handler.ego_vehicle_state.velocity, 0.1))
                            )
                            self.get_logger().info(
                                "Reactive planner reference path velocity: "
                                + str(reference_velocity)
                            )
                            self.get_logger().info(
                                "Reactive planner reference path length: "
                                + str(len(self.reference_path))
                            )
                            if len(self.reference_path > 1):
                                self.get_logger().info(
                                    "Reactive planner reference path: "
                                    + str(self.reference_path[0])
                                    + "  --->  ["
                                    + str(len(self.reference_path) - 2)
                                    + " states skipped]  --->  "
                                    + str(self.reference_path[-1])
                                )

                        # when starting the route and the initial velocity is 0, the reactive planner would return zero velocity for
                        # it's first state and thus never start driving. As a result, we increase the velocity a little bit here
                        init_state = deepcopy(self.ego_vehicle_handler.ego_vehicle_state)
                        if init_state.velocity < 0.1:
                            init_state.velocity = 0.1

                        self.trajectory_planner.plan(
                            init_state=init_state,
                            goal=self.planning_problem.goal,
                            reference_path=self.reference_path,
                            reference_velocity=reference_velocity,
                        )

                        assert self.trajectory_planner.optimal is not False
                        assert self.trajectory_planner.valid_states != []
                        assert max([s.velocity for s in self.trajectory_planner.valid_states]) > 0

                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info(
                                "Reactive planner trajectory: "
                                + str([self.trajectory_planner.valid_states[0].position])
                                + " -> ... -> "
                                + str([self.trajectory_planner.valid_states[-1].position])
                            )
                            self.get_logger().info(
                                "Reactive planner velocities: "
                                + str([s.velocity for s in self.trajectory_planner.valid_states])
                            )
                            self.get_logger().info(
                                "Reactive planner acc: "
                                + str(
                                    [s.acceleration for s in self.trajectory_planner.valid_states]
                                )
                            )

                        # calculate velocities and accelerations of planner states
                        # self._calculate_velocities(self.planner.valid_states, self.ego_vehicle_handler.ego_vehicle_state.velocity)

                        # publish trajectory
                        self._prepare_traj_msg(self.trajectory_planner.valid_states)
                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info("Autoware state and engage messages published!")

                    # check if goal is reached
                    self._is_goal_reached()

                self.is_computing_trajectory = False
            else:
                if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                    self.get_logger().info("already solving planning problem")

        except Exception:
            self.get_logger().error(traceback.format_exc())

    def follow_trajectory_mode_update(self):
        """Update mode for follow trajectory mode. It checks if the goal position is reached."""
        try:
            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("Next cycle of follow trajectory mode")

            # update scenario
            self.ego_vehicle_handler.update_ego_vehicle()
            self.scenario_handler.update_scenario()
            self.plot_save_scenario()

            if not self.route_planned:
                # try to set a new goal position
                self._set_new_goal()
            else:
                # check if goal is reached
                self._is_goal_reached()
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def plot_save_scenario(self):
        if self.get_parameter("plot_scenario").get_parameter_value().bool_value:
            self._plot_scenario()

        if self.write_scenario:
            self._write_scenario()

    def _is_goal_reached(self):
        """Check if vehicle is in goal region. If in goal region set new goal."""
        if self.planning_problem:
            if (
                self.planning_problem.goal.is_reached(self.ego_vehicle_handler.ego_vehicle_state)
                and (self.get_clock().now() - self.last_goal_reached).nanoseconds > 5e8
            ):
                self.get_logger().info("Car arrived at goal!")
                self.last_goal_reached = self.get_clock().now()

                if (
                    self.interactive_mode
                    and self.get_parameter("store_trajectory").get_parameter_value().bool_value
                ):
                    self.trajectory_logger.store_trajectory(
                        self.scenario,
                        self.planning_problem,
                        self.get_parameter("store_trajectory_file")
                        .get_parameter_value()
                        .string_value,
                    )

                if self.goal_msgs == []:
                    self.route_planned = False
                    self.planning_problem = None
                    self.set_state(AutowareState.ARRIVED_GOAL)

                    # publish empty trajectory
                    self._pub_route([], [])
                else:
                    self._set_new_goal()

    def follow_solution_trajectory(self):
        """Follow a trajectory provided by a CommonRoad solution file."""
        states = self.trajectory_logger.load_trajectory(self.solution_path)

        # set initial pose to first position in solution trajectory and publish it
        initial_pose_msg = PoseWithCovarianceStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        initial_pose_msg.header = header
        pose = Pose()
        pose.position = utils.utm2map(self.origin_transformation, states[0].position)
        pose.orientation = utils.orientation2quaternion(states[0].orientation)
        initial_pose_msg.pose.pose = pose
        self.initial_pose_pub.publish(initial_pose_msg)

        # set goal to last position in solution trajectory and publish it
        goal_msg = PoseStamped()
        goal_msg.header = header
        pose = Pose()
        pose.position = utils.utm2map(self.origin_transformation, states[-1].position)
        pose.orientation = utils.orientation2quaternion(states[-1].orientation)
        goal_msg.pose = pose
        self.goal_pose_pub.publish(goal_msg)

        # publish solution trajectory
        self._prepare_traj_msg(states)

        self.set_state(AutowareState.WAITING_FOR_ENGAGE)

    def initialize_velocity_planner(self):
        # Initialize Velocity Planner
        self.velocity_planner = VelocityPlanner(
            self.get_parameter("detailed_log").get_parameter_value().bool_value,
            self.get_logger(),
            self.get_parameter("velocity_planner.lookahead_dist")
            .get_parameter_value()
            .double_value,
            self.get_parameter("velocity_planner.lookahead_time")
            .get_parameter_value()
            .double_value,
        )

        self.traj_sub_smoothed = self.create_subscription(
            AWTrajectory,
            "/planning/scenario_planning/trajectory_smoothed",
            self.velocity_planner.smoothed_trajectory_callback,
            1,
            callback_group=self.callback_group,
        )
        # publish trajectory to motion velocity smoother
        self.velocity_pub = self.create_publisher(
            AWTrajectory, "/planning/scenario_planning/scenario_selector/trajectory", 1
        )
        self.velocity_planner.set_publisher(self.velocity_pub)

    def current_state_callback(self, msg: Odometry) -> None:
        """Callback to current kinematic state of the ego vehicle.

        Safe the message for later processing.
        :param msg: current kinematic state message
        """
        self.ego_vehicle_handler.current_vehicle_state = msg
        self.new_pose_received = True

    def _process_current_state(self) -> None:
            self.ego_vehicle_handler.process_current_state()

    def _awtrajectory_to_crtrajectory(self, mode, time_step, traj):
        """
        Generate a commonroad trajectory from autoware trajectory.
        :param mode: 1=TrajectoryPoint mode, 2=pose mode
        :param time_step: timestep from which to start
        :param traj: the trajectory in autoware format
        :return CRTrajectory: the trajectory in commonroad format
        """
        state_list = []
        work_traj = []
        if mode == 1:
            for trajectory_point in traj:
                work_traj.append(trajectory_point.pose)
        else:
            work_traj = traj
        for i in range(len(work_traj)):
            # compute new position
            position = utils.map2utm(self.origin_transformation, work_traj[i].position)
            orientation = utils.quaternion2orientation(work_traj[i].orientation)
            new_state = CustomState(position=position, orientation=orientation, time_step=i)
            state_list.append(new_state)

        return CRTrajectory(time_step, state_list)

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Callback to initial pose changes. Safe message for later processing
        :param msg: Initial Pose message
        """

        self.get_logger().info("Received new initial pose!")
        self.initial_pose = msg
        self.new_initial_pose = True
        self.new_pose_received = False

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback to goal pose. Safe message to goal message list and set as active goal if no goal is active.
        :param msg: Goal Pose message
        """
        self.get_logger().info("Received new goal pose!")
        self.goal_msgs.append(msg)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("goal msg: " + str(msg))

        self._pub_goals()
        # autoware requires that the reference path has to be published again when new goals are published
        if self.velocity_planner.get_is_velocity_planning_completed():
            self._pub_route(*self.velocity_planner.get_reference_velocities())

    def set_state(self, new_aw_state: AutowareState):
        self.aw_state.state = new_aw_state
        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Setting new state to: " + str(new_aw_state))
        self.aw_state_pub.publish(self.aw_state)

        if new_aw_state != AutowareState.DRIVING:
            self.engage_status = False

        # Send engage signal
        engage_msg = Engage()
        engage_msg.engage = self.engage_status
        self.engage_pub.publish(engage_msg)
        self.vehicle_engage_pub.publish(engage_msg)
        self.ext_engage_pub.publish(engage_msg)
        self.api_engage_pub.publish(engage_msg)

    def get_state(self):
        return self.aw_state.state

    def _set_new_goal(self) -> None:
        """
        Set the next goal of the goal message list active. Calculate route to new goal.
        Publish new goal markers and route for visualization in RVIZ.
        """
        # set new goal if we have one
        if len(self.goal_msgs) > 0:
            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("Setting new goal")

            self.set_state(AutowareState.PLANNING)

            current_msg = self.goal_msgs.pop(0)
            self.current_goal_msg = deepcopy(current_msg)

            self.get_logger().info("Pose position: " + str(current_msg.pose.position))

            position = utils.map2utm(self.origin_transformation, current_msg.pose.position)
            pos_x = position[0]
            pos_y = position[1]
            self.get_logger().info("Pose position utm: " + str(position))
            orientation = utils.quaternion2orientation(current_msg.pose.orientation)
            if self.ego_vehicle_handler.ego_vehicle_state is None:
                self.get_logger().error("ego vehicle state is None")
                return

            max_vel = self.get_parameter("vehicle.max_velocity").get_parameter_value().double_value
            min_vel = self.get_parameter("vehicle.min_velocity").get_parameter_value().double_value
            velocity_interval = Interval(min_vel, max_vel)

            # get goal lanelet and its width
            # subtract commonroad map origin
            goal_lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position(
                [np.array([pos_x, pos_y])]
            )

            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("goal pos_x: " + str(pos_x) + ", pos_y: " + str(pos_y))

            if goal_lanelet_id == [[]]:
                self.get_logger().error("No lanelet found at goal position!")
                return

            if goal_lanelet_id:
                goal_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(
                    goal_lanelet_id[0][0]
                )
                left_vertices = goal_lanelet.left_vertices
                right_vertices = goal_lanelet.right_vertices
                goal_lanelet_width = np.linalg.norm(left_vertices[0] - right_vertices[0])
            else:
                goal_lanelet_width = 3.0

            region = Rectangle(
                length=self.ego_vehicle_handler.vehicle_length + 0.25 * self.ego_vehicle_handler.vehicle_length,
                width=goal_lanelet_width,
                center=position,
                orientation=orientation,
            )
            goal_state = CustomState(
                position=region, time_step=Interval(0, 1000), velocity=velocity_interval
            )

            goal_region = GoalRegion([goal_state])
            self.planning_problem = PlanningProblem(
                planning_problem_id=1, initial_state=self.ego_vehicle_handler.ego_vehicle_state, goal_region=goal_region
            )
            self.get_logger().info("Set new goal active!")
            self._plan_route()
            self._pub_goals()

            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

            self.route_planned = True
        else:
            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("No new goal could be set")

    def state_callback(self, msg: AutowareState) -> None:
        """
        Callback to autoware state. Safe the message for later processing.
        :param msg: autoware state message
        """
        self.last_msg_aw_state = msg.state

    # The engage signal is sent by the tum_state_rviz_plugin
    # msg.engage sent by tum_state_rviz_plugin will always be true
    def engage_callback(self, msg: Engage) -> None:
        self.engage_status = not self.engage_status
        self.get_logger().info(
            "Engage message received! Engage: "
            + str(self.engage_status)
            + ", current state: "
            + str(self.get_state())
        )

        # Update Autoware state panel
        if self.engage_status and self.get_state() == AutowareState.WAITING_FOR_ENGAGE:
            self.set_state(AutowareState.DRIVING)
            if self.PUBLISH_OBSTACLES:  # publish obstacle at once after engaged
                self.scenario_handler.publish_initial_obstacles()
                self.PUBLISH_OBSTACLES = False

        if not self.engage_status and self.get_state() == AutowareState.DRIVING:
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

        # reset follow sultion trajectory simulation if interface is in trajectory follow mode and goal is reached
        if not self.interactive_mode and self.get_state() == AutowareState.ARRIVED_GOAL:
            self.follow_solution_trajectory()

    def _prepare_traj_msg(self, states):
        """
        Prepares trajectory to match autoware format. Publish the trajectory.
        :param states: trajectory points
        :param contains_goal: flag to reduce speed over the last 10 steps
        """
        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Preparing trajectory message!")

        self.traj = AWTrajectory()
        self.traj.header.frame_id = "map"

        if states == []:
            self.traj_pub.publish(self.traj)
            self.get_logger().info("New empty trajectory published !!!")
            return

        position_list = []
        for i in range(0, len(states)):
            new_point = TrajectoryPoint()
            new_point.pose.position = utils.utm2map(self.origin_transformation, states[i].position)
            position_list.append([states[i].position[0], states[i].position[1]])
            new_point.pose.orientation = utils.orientation2quaternion(states[i].orientation)
            new_point.longitudinal_velocity_mps = float(states[i].velocity)

            # front_wheel_angle_rad not given by autoware planner
            # new_point.front_wheel_angle_rad = states[i].steering_angle
            new_point.acceleration_mps2 = float(states[i].acceleration)
            self.traj.points.append(new_point)

        self.traj_pub.publish(self.traj)
        self.get_logger().info("New trajectory published !!!")
        # visualize_solution(self.scenario, self.planning_problem, create_trajectory_from_list_states(path)) #ToDo: test

    # plan route
    def _plan_route(self):
        """
        Plan a route using commonroad route planner and the current scenario and planning problem.
        """

        self.reference_path_published = False

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Planning route")

        route_planner = RoutePlanner(self.scenario, self.planning_problem)
        reference_path = route_planner.plan_routes().retrieve_first_route().reference_path
        # smooth reference path
        tck, u = splprep(reference_path.T, u=None, k=3, s=0.0)
        u_new = np.linspace(u.min(), u.max(), 200)
        x_new, y_new = splev(u_new, tck, der=0)
        reference_path = np.array([x_new, y_new]).transpose()
        reference_path = resample_polyline(reference_path, 1)
        # remove duplicated vertices in reference path
        _, idx = np.unique(reference_path, axis=0, return_index=True)
        reference_path = reference_path[np.sort(idx)]
        self.reference_path = reference_path
        self.velocity_planner.send_reference_path(
            [utils.utm2map(self.origin_transformation, point) for point in self.reference_path],
            self.current_goal_msg.pose.position,
        )

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Route planning completed!")

    def _pub_goals(self):
        """
        Publish the goals as markers to visualize in RVIZ.
        """
        goals_msg = MarkerArray()

        # first delete all marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.ns = "goals"
        marker.action = Marker.DELETEALL
        goals_msg.markers.append(marker)

        if self.current_goal_msg is not None:
            marker = utils.create_goal_marker(self.current_goal_msg.pose.position)
            marker.id = 1
            marker.ns = "goals"
            goals_msg.markers.append(marker)

        if len(self.goal_msgs) > 0:
            for i in range(len(self.goal_msgs)):
                marker = utils.create_goal_marker(self.goal_msgs[i].pose.position)
                marker.id = i + 2
                marker.ns = "goals"
                goals_msg.markers.append(marker)

        self.route_pub.publish(goals_msg)

    def _pub_route(self, path, velocities):
        """
        Publish planned route as marker to visualize in RVIZ.
        """
        self.reference_path_published = True
        self.route_pub.publish(utils.create_route_marker_msg(path, velocities))

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Reference path published!")
            self.get_logger().info("Path length: " + str(len(path)))
            self.get_logger().info("Velocities length: " + str(len(velocities)))

    def _plot_scenario(self):
        """
        Plot the commonroad scenario.
        """
        if self.rnd is None:
            self.rnd = MPRenderer()
            plt.ion()

        self.rnd.clear()
        self.ego_vehicle_handler.ego_vehicle = (
            self.ego_vehicle_handler.create_ego_with_cur_location()
        )
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.facecolor = "#ff0000"
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.edgecolor = "#000000"
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.zorder = 50
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.opacity = 1
        """self.ego_vehicle_handler.ego_vehicle.draw(self.rnd, draw_params={ # outdate cr-io
            "static_obstacle": {
                "occupancy": {
                    "shape": {
                        "rectangle": {
                            "facecolor": "#ff0000",
                            "edgecolor": '#000000',
                            "zorder": 50,
                            "opacity": 1
                        }
                    }
                }

            }
        })"""
        # self.rnd.draw_params["static_obstacle"]["occupancy"]["shape"]["rectangle"]["facecolor"] = "#ff0000"
        self.ego_vehicle.draw(self.rnd)

        # self.rnd.draw_params.lanelet.show_label = False
        self.scenario.draw(self.rnd)
        # self.planning_problem.draw(self.rnd) #ToDo: check if working
        self.rnd.render()
        plt.pause(0.1)

    def _write_scenario(self):
        """
        Store converted map as CommonRoad scenario.
        """
        planning_problem = PlanningProblemSet()
        if self.planning_problem:
            planning_problem.add_planning_problem(self.planning_problem)
        writer = CommonRoadFileWriter(
            scenario=self.scenario,
            planning_problem_set=planning_problem,
            author="",
            affiliation="Technical University of Munich",
            source="",
            tags={Tag.URBAN},
        )
        os.makedirs("output", exist_ok=True)
        writer.write_to_file(
            os.path.join("output", "".join([str(self.scenario.scenario_id), ".xml"])),
            OverwriteExistingFile.ALWAYS,
        )

    def transform_pose(self, pose_in: PoseStamped, target_frame: str = "map") -> PoseStamped:
        """Transform a `PoseStamped` message to the target frame.

        Args:
            pose_in (PoseStamped): Pose to transform.
            target_frame (str, optional): Target frame. Defaults to "map".
        """
        source_frame = pose_in.header.frame_id

        is_possible = self.tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
        )
        if not is_possible:
            self.get_logger().error(
                f"Failed to transform from {source_frame} to {target_frame} frame."
            )
            return

        try:
            tf_map = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time.from_msg(pose_in.header.stamp)
            )
        except tf2_ros.ExtrapolationException:
            tf_map = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

        pose_out = do_transform_pose(pose_in, tf_map)
        return pose_out


def main(args=None):
    rclpy.init(args=args)
    cr2auto = Cr2Auto()
    # Create executor for multithreded execution
    executor = MultiThreadedExecutor()
    executor.add_node(cr2auto)
    executor.spin()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cr2auto.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
