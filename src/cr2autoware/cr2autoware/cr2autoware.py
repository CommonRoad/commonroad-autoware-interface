# standard imports
from copy import deepcopy
import os
import traceback
from typing import Optional
from dataclasses import asdict, fields

# third party imports
import numpy as np
import yaml
import matplotlib

if os.environ.get('DISPLAY') is not None:
    matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import time

# Autoware.Auto message imports
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_planning_msgs.msg import TrajectoryPoint  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState # type: ignore
from autoware_adapi_v1_msgs.srv import ChangeOperationMode # type: ignore


# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit   # type: ignore

# ROS message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# ROS imports
# rclpy
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
# tf2
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# commonroad-io imports
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.scenario import Tag
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.visualization.mp_renderer import MPRenderer

# cr2autoware imports
from cr2autoware.configuration import CR2AutowareParams
from cr2autoware.ego_vehicle_handler import EgoVehicleHandler
from cr2autoware.planning_problem_handler import PlanningProblemHandler
from cr2autoware.route_planner import RoutePlannerInterface
from cr2autoware.rp_interface import RP2Interface
from cr2autoware.scenario_handler import ScenarioHandler
from cr2autoware.tf2_geometry_msgs import do_transform_pose
from cr2autoware.trajectory_logger import TrajectoryLogger
import cr2autoware.utils as utils
from cr2autoware.velocity_planner import VelocityPlanner

try:
    from cr2autoware.spot_handler import SpotHandler
except ImportError:
    SpotHandler = None


class Cr2Auto(Node):
    """
    Cr2Auto class that is an instance of a ROS2 Node.
    This node serves as a the main interface between CommonRoad and Autoware.Universe
    """

    # TODO: Why are these defined as class attributes?
    scenario_handler: ScenarioHandler
    plan_prob_handler: PlanningProblemHandler
    spot_handler: Optional["SpotHandler"]  # May be None if Spot is not installed or disabled

    def __init__(self):
        """
        Constructor of the Cr2Auto class.
        """
        # ignore typing due to bug in rclpy
        super().__init__(node_name="cr2autoware")  # type: ignore

        # Declare ROS parameters and add to params class
        self.params: CR2AutowareParams = CR2AutowareParams(_node=self)

        # get logger and set verbosity level
        self._logger = self.get_logger()
        self.verbose = self.params.general.detailed_log
        if self.verbose:
            self._logger.info("Verbose logging is enabled. Setting Log Level to DEBUG.")
            self._logger.set_level(LoggingSeverity.DEBUG)

        # log map path and solution path
        self._logger.info(
            "Map path is: " + self.get_parameter("general.map_path").get_parameter_value().string_value)
        self._logger.info(
            "Solution path is: " + self.get_parameter("general.solution_file").get_parameter_value().string_value)

        # initialize callback group
        self.callback_group = ReentrantCallbackGroup()  # Callback group for async execution

        # initialize commonroad-specfic attributes
        self.write_scenario = self.get_parameter("general.write_scenario").get_parameter_value().bool_value
        self.PUBLISH_OBSTACLES = self.get_parameter("scenario.publish_obstacles").get_parameter_value().bool_value
        self.solution_path = self.get_parameter("general.solution_file").get_parameter_value().string_value
        self.rnd = None

        # initialize CR trajectory logger (optionally store CR solution file)
        self.trajectory_logger = TrajectoryLogger(
            None,
            self._logger,
            self.verbose,
        )

        # ========= CommonRoad Handlers =========
        # scenario handler
        self.scenario_handler: ScenarioHandler = ScenarioHandler(self)
        self.origin_transformation = self.scenario_handler.origin_transformation

        # ego vehicle handler
        self.ego_vehicle_handler: EgoVehicleHandler = EgoVehicleHandler(self)

        # planning problem handler
        self.plan_prob_handler: PlanningProblemHandler = PlanningProblemHandler(self, 
                                                                                self.scenario, 
                                                                                self.origin_transformation)
        # SPOT handler (optional)
        if self.get_parameter("general.enable_spot").get_parameter_value().bool_value:
            if SpotHandler is None:
                self._logger.error(
                    "The Spot module has been enabled but wasn't imported! "
                    "Have you installed SPOT into your Python environment? "
                    "Continuing without SPOT.")
                self.spot_handler = None
            else:
                self.spot_handler = SpotHandler(self)
        else:
            self.spot_handler = None
        
        # write CommonRoad scenario to xml file
        if self.write_scenario:
            self._write_scenario()
        
        # intiialize planning-specific attributes
        self.planning_problem_set = None
        self.route_planned = False
        self.planner_state_list = None
        self.is_computing_trajectory = False  # stop update scenario when trajectory is being computed
        self.reference_path = None
        self.reference_path_published = False
        self.new_initial_pose = False
        self.new_pose_received = False
        self.external_velocity_limit = 1337.0

        # initialize tf
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames

        # initialize AutowareState and Engage Status and Auto Button Status and waiting_for_velocity_0 and Routing State

        self.aw_state = AutowareState()
        self.engage_status = False
        self.auto_button_status = False
        self.waiting_for_velocity_0 = False
        self.routing_state = RouteState()

        # vars to save last messages
        # https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl
        self.last_msg_aw_state = 1  # 1 = initializing
        self.goal_msgs = []
        self.current_goal_msg = None
        self.last_goal_reached = self.get_clock().now()

        # ========= Subscribers =========
        # subscribe current state from odometry
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
            "/initialpose3d",
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
        # subscribe autoware engage message
        self.auto_button_sub = self.create_subscription(
            Engage,
            "/autoware/engage",
            self.auto_button_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe velocity limit from API
        # (Here we directly use the value from the API velocity limit setter in RVIZ.
        # In the default AW.Universe this value is input to the node external_velocity_limit_selector
        # which selects the velocity limit from different values)
        self.create_subscription(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity_default",
            self.velocity_limit_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe routing state
        self.routing_state_sub = self.create_subscription(
            RouteState,
            "/api/routing/state",
            self.routing_state_callback,
            1,
            callback_group=self.callback_group,
        )
        # subscribe autoware state
        self.autoware_state_sub = self.create_subscription(
            AutowareState,
            "/autoware/state",
            self.state_callback,
            1,
            callback_group=self.callback_group,
        )
        # ========= Publishers =========
        # publish goal pose
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            "/planning/mission_planning/goal",
            1,
        )
        # publish trajectory
        # We do not publish the trajectory directly to /planning/scenario_planning/trajectory
        # but instead use a dummy-topic that is analyzed by the planning_validator, which in turn is checked
        # by the system_error_monitor
        self.traj_pub = self.create_publisher(
            AWTrajectory, 
            "/planning/commonroad/trajectory", 
            1
        )
        # publish autoware state
        # list of states: https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/AutowareState.idl
        self.aw_state_pub = self.create_publisher(
            AutowareState, 
            "/autoware/state", 
            1
        )
        # publish vehicle engage for AW Planning Simulation
        self.vehicle_engage_pub = self.create_publisher(
            Engage, 
            "/vehicle/engage", 
            1
        )
        # publish engage required by node /control/operation_mode_transistion_manager
        self.api_engage_pub = self.create_publisher(
            Engage, 
            "/api/autoware/get/engage", 
            1
        )
        # publish routing state
        qos_routing_state_pub = utils.create_qos_profile(QoSHistoryPolicy.KEEP_LAST,
                                                         QoSReliabilityPolicy.RELIABLE,
                                                         QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                                         depth=1)
        self.routing_state_pub = self.create_publisher(
            RouteState, 
            "/api/routing/state", 
            qos_routing_state_pub
        )
        # publish route marker
        qos_route_pub = utils.create_qos_profile(QoSHistoryPolicy.KEEP_LAST,
                                                 QoSReliabilityPolicy.RELIABLE,
                                                 QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                                 depth=1)
        self.route_pub = self.create_publisher(
            MarkerArray,
            "/planning/mission_planning/route_marker",
            qos_route_pub,
        )
        # publish initial state of the scenario (replay solution trajectory mode)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            "/initialpose3d", 
            1
        )
        # publish goal region(s) of the scenario (TODO: check, currently not used)
        self.goal_region_pub = self.create_publisher(
            MarkerArray, 
            "/goal_region_marker_array", 
            1
        )
        # publish max velocity limit
        # (currently only subscribed by Motion Velocity Smoother, if we use the external_velocity_limit_selector node,
        #  this publisher can be removed)
        self.velocity_limit_pub = self.create_publisher(
            VelocityLimit,
            "/planning/scenario_planning/max_velocity",
            1
        )
        # publish current velocity limit for display in RVIZ
        # (this separate topic is currently only subscribed by RVIZ)
        qos_velocity_limit_pub_vis = utils.create_qos_profile(QoSHistoryPolicy.KEEP_LAST,
                                                              QoSReliabilityPolicy.RELIABLE,
                                                              QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                                              depth=1)
        self.velocity_limit_pub_vis = self.create_publisher(
            VelocityLimit,
            "/planning/scenario_planning/current_max_velocity",
            qos_velocity_limit_pub_vis
        )

        # ========= Service Clients =========
        # client for change to stop service call (only for publishing "stop" if goal arrived)

        self.change_to_stop_client = self.create_client(
            ChangeOperationMode, 
            "/api/operation_mode/change_to_stop"
        )
        self.change_to_stop_request = ChangeOperationMode.Request()

        # ========= Set up Planner Interfaces =========
        # set initial Autoware State (Waiting for route)
        self.set_state(AutowareState.WAITING_FOR_ROUTE)

        # get trajectory_planner_type
        self.trajectory_planner_type = (self.get_parameter("trajectory_planner.trajectory_planner_type").get_parameter_value().integer_value)

        # set route planner
        self.route_planner = self._set_route_planner()

        # set velocity planner
        self._set_velocity_planner()

        # set trajectory planner using factory function
        self.trajectory_planner = self._trajectory_planner_factory()

        # set mode of interface (interactive or replay mode)
        self._set_cr2auto_mode()

        # set initial velocity limit (from launch config, the limit can be changed during runtime via API)
        self._set_external_velocity_limit(
            vel_limit=self.get_parameter("vehicle.max_velocity").get_parameter_value().double_value)

        # ========= Finish init() =========
        if self.verbose:
            self._logger.info("Cr2Auto initialization is completed!")

    @property
    def scenario(self) -> Scenario:
        """
        Get scenario object retrieved from the scenario_handler.
        Caution: Does not trigger an update of the scenario.
        """
        if self.scenario_handler is None:
            raise RuntimeError("Scenario handler not initialized.")
        return self.scenario_handler.scenario

    @property
    def planning_problem(self) -> Optional[PlanningProblem]:
        """
        Get planning problem object retrieved from the planning problem handler.
        Caution: Does not trigger an update of the planning problem.
        """
        if self.plan_prob_handler is None:
            raise RuntimeError("Planning problem handler not initialized.")
        return self.plan_prob_handler.planning_problem

    @planning_problem.setter
    def planning_problem(self, planning_problem):
        """Set planning problem in the planning problem handler."""
        self.plan_prob_handler.planning_problem = planning_problem
    
    def _set_route_planner(self) -> RoutePlannerInterface:
        """Initializes the route planner"""
        return RoutePlannerInterface(self.verbose, self._logger, self.scenario, self.route_pub)

    def _set_velocity_planner(self):
        """Initializes the velocity planner"""
        self.velocity_planner = VelocityPlanner(
            self.verbose,
            self._logger,
            self.get_parameter("velocity_planner.lookahead_dist").get_parameter_value().double_value,
            self.get_parameter("velocity_planner.lookahead_time").get_parameter_value().double_value,
        )

        # subscribe trajectory from motion velocity smoother
        self.traj_sub_smoothed = self.create_subscription(
            AWTrajectory,
            "/planning/scenario_planning/trajectory_smoothed",
            self.velocity_planner.smoothed_trajectory_callback,
            1,
            callback_group=self.callback_group)
        # publish reference trajectory to motion velocity smoother
        self.velocity_pub = self.create_publisher(
            AWTrajectory,
            "/planning/scenario_planning/scenario_selector/trajectory",
            1)
        self.velocity_planner.set_publisher(self.velocity_pub)

    def _trajectory_planner_factory(self):
        """
        Factory function to initialize trajectory planner according to specified type.
        """
        if self.trajectory_planner_type == 1:  # Reactive planner
            return RP2Interface(self.scenario, self.scenario.dt, self.trajectory_logger, self.params,
                                self.ego_vehicle_handler.vehicle_length,
                                self.ego_vehicle_handler.vehicle_width,
                                self.ego_vehicle_handler.vehicle_wheelbase,
                                self.ego_vehicle_handler.vehicle_wb_rear_axle,
                                self.ego_vehicle_handler.vehicle_max_steer_angle,
                                self.ego_vehicle_handler.vehicle_max_acceleration)
        else:
            self._logger.error("Planner type is not correctly specified!")

    def _set_cr2auto_mode(self):
        """
        Decide whether the CR2Autoware interface goes in interactive planning mode or trajectory following mode.
        """
        if self.solution_path == "":
            # set interactive mode to true
            self.interactive_mode = True
            self._logger.info("Starting interactive planning mode...")

            # create a timer for periodically solving planning problem
            self.timer_solve_planning_problem = self.create_timer(
                timer_period_sec=self.get_parameter("trajectory_planner.planner_update_time")
                .get_parameter_value()
                .double_value,
                callback=self.solve_planning_problem,
                callback_group=self.callback_group)
        else:
            # follow solution trajectory
            self.interactive_mode = False
            self._logger.info("Loading solution trajectory...")

            self.follow_solution_trajectory()

            # create a timer for periodically updating trajectory following
            self.timer_follow_trajectory_mode_update = self.create_timer(
                timer_period_sec=self.get_parameter("trajectory_planner.planner_update_time")
                .get_parameter_value()
                .double_value,
                callback=self.follow_trajectory_mode_update,
                callback_group=self.callback_group)

    def _set_external_velocity_limit(self, vel_limit: float):
        """
        Sets the velocity limit for CR2Autoware
        Publishes the velocity limit for RVIZ
        TODO: Remove if external_velocity_limit_selector node is used
        """
        # set limit
        self.external_velocity_limit = max(0, vel_limit)

        # publish velocity limit message
        vel_limit_msg = VelocityLimit()
        vel_limit_msg.max_velocity = vel_limit
        # publish max velocity for other modules
        self.velocity_limit_pub.publish(vel_limit_msg)
        # publish max velocity for display in RVIZ
        self.velocity_limit_pub_vis.publish(vel_limit_msg)

    def solve_planning_problem(self) -> None:
        """
        Main planning function
        Update loop for interactive planning mode and solve planning problem with algorithms offered by CommonRoad.
        """
        try:
            # avoid parallel processing issues by checking if a planning problem is already being solved
            if not self.is_computing_trajectory:
                # Compute trajectory
                self.is_computing_trajectory = True
                self.ego_vehicle_handler.update_ego_vehicle()
                self.scenario_handler.update_scenario()
                self.plot_save_scenario()
                if self.planning_problem and self.spot_handler is not None:
                    self.spot_handler.update(
                        self.scenario,
                        self.origin_transformation,
                        self.planning_problem,
                    )

                # check if initial pose was changed (if true: recalculate reference path)
                if self.new_initial_pose:
                    # check if the current_vehicle_state was already updated (=pose received by current state callback), otherwise wait one planning cycle
                    if not self.new_pose_received:
                        self.is_computing_trajectory = False
                        return

                    self.new_initial_pose = False
                    self.new_pose_received = False
                    self._logger.info("Replanning route to goal")

                    # insert current goal into list of goal messages and set route_planned to false to trigger route planning
                    if self.current_goal_msg:
                        self.goal_msgs.insert(0, self.current_goal_msg)
                    self.route_planned = False

                if not self.route_planned:
                    # if currently no active goal, set a new goal (if one exists)
                    try:
                        self._set_new_goal()
                    except Exception:
                        self._logger.error(traceback.format_exc())

                if self.route_planned:
                    if not self.velocity_planner.get_is_velocity_planning_completed():
                        self._logger.info(
                            "Can't run route planner because interface is still waiting for velocity planner"
                        )
                        self.velocity_planner.send_reference_path(
                            [utils.utm2map(self.origin_transformation, point) for point in self.route_planner.reference_path],
                            self.current_goal_msg.pose.position,
                        )
                        self.is_computing_trajectory = False
                        return

                    if not self.route_planner.reference_path_published:
                        # publish current reference path
                        point_list, reference_velocities = self.velocity_planner.get_reference_velocities()
                        # post process reference path z coordinate
                        for point in point_list: 
                            point.z = self.scenario_handler.get_z_coordinate()
                        self.route_planner._pub_route(point_list, reference_velocities)

                    if self.get_state() == AutowareState.DRIVING:
                        # log current position
                        self.trajectory_logger.log_state(self.ego_vehicle_handler.ego_vehicle_state)

                    if self.verbose:
                        self._logger.info("Solving planning problem!")

                    if self.trajectory_planner_type == 1:  # Reactive Planner
                        reference_velocity = max(
                            1,
                            self.velocity_planner.get_velocity_at_aw_position_with_lookahead(
                                self.ego_vehicle_handler.current_vehicle_state.pose.pose.position,
                                self.ego_vehicle_handler.ego_vehicle_state.velocity,
                            ),
                        )

                        # if self.verbose:
                            # self._logger.info("Running reactive planner")
                            # self._logger.info(
                            #     "Reactive planner init_state position: "
                            #     + str(self.ego_vehicle_handler.ego_vehicle_state.position)
                            # )
                            # self._logger.info(
                            #     "Reactive planner init_state velocity: "
                            #     + str(max(self.ego_vehicle_handler.ego_vehicle_state.velocity, 0.1))
                            # )
                            # self._logger.info(
                            #     "Reactive planner reference path velocity: "
                            #     + str(reference_velocity)
                            # )
                            # self._logger.info(
                            #     "Reactive planner reference path length: "
                            #     + str(len(self.route_planner.reference_path))
                            # )
                            # if len(self.route_planner.reference_path > 1):
                            #     self._logger.info(
                            #         "Reactive planner reference path: "
                            #         + str(self.route_planner.reference_path[0])
                            #         + "  --->  ["
                            #         + str(len(self.route_planner.reference_path) - 2)
                            #         + " states skipped]  --->  "
                            #         + str(self.route_planner.reference_path[-1])
                            #     )

                        # when starting the route and the initial velocity is 0, the reactive planner would return zero velocity for
                        # it's first state and thus never start driving. As a result, we increase the velocity a little bit here
                        init_state = deepcopy(self.ego_vehicle_handler.ego_vehicle_state)
                        if init_state.velocity < 0.1:
                            init_state.velocity = 0.1
                        
                        # set reference velocity considering external limit
                        ref_vel = min(reference_velocity, self.external_velocity_limit)

                        # call the one-step plan function
                        self.trajectory_planner.plan(
                            init_state=init_state,
                            goal=self.planning_problem.goal,
                            reference_path=self.route_planner.reference_path,
                            reference_velocity=ref_vel,
                        )

                        assert self.trajectory_planner.optimal is not False
                        assert self.trajectory_planner.valid_states != []
                        assert max([s.velocity for s in self.trajectory_planner.valid_states]) > 0

                        # if self.verbose:
                            # self._logger.info(
                            #     "Reactive planner trajectory: "
                            #     + str([self.trajectory_planner.valid_states[0].position])
                            #     + " -> ... -> "
                            #     + str([self.trajectory_planner.valid_states[-1].position])
                            # )
                            # self._logger.info(
                            #     "Reactive planner velocities: "
                            #     + str([s.velocity for s in self.trajectory_planner.valid_states])
                            # )
                            # self._logger.info(
                            #     "Reactive planner acc: "
                            #     + str(
                            #         [s.acceleration for s in self.trajectory_planner.valid_states]
                            #     )
                            # )

                        # calculate velocities and accelerations of planner states
                        # self._calculate_velocities(self.planner.valid_states, self.ego_vehicle_handler.ego_vehicle_state.velocity)

                        # publish trajectory
                        self._prepare_traj_msg(self.trajectory_planner.valid_states)
                        if self.verbose:
                            self._logger.info("Autoware state and engage messages published!")

                    # check if goal is reached
                    self._is_goal_reached()

                self.is_computing_trajectory = False
            else:
                if self.verbose:
                    self._logger.info("already solving planning problem")

        except Exception:
            self._logger.error(traceback.format_exc())

    def follow_trajectory_mode_update(self):
        """
        Update mode for follow trajectory mode. It checks if the goal position is reached.
        """
        try:
            if self.verbose:
                self._logger.info("Next cycle of follow trajectory mode")

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
            self._logger.error(traceback.format_exc())

    def plot_save_scenario(self):
        if self.get_parameter("general.plot_scenario").get_parameter_value().bool_value:
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
                self._logger.info("Car arrived at goal!")
                self.last_goal_reached = self.get_clock().now()

                if (
                    self.interactive_mode
                    and self.get_parameter("general.store_trajectory").get_parameter_value().bool_value
                ):
                    self.trajectory_logger.store_trajectory(
                        self.scenario,
                        self.planning_problem,
                        self.get_parameter("general.store_trajectory_file")
                        .get_parameter_value()
                        .string_value,
                    )

                if not self.goal_msgs:
                    self.route_planned = False
                    self.planning_problem = None
                    self.set_state(AutowareState.ARRIVED_GOAL)

                    # publish empty trajectory
                    self.route_planner._pub_route([], [])
                else:
                    self._set_new_goal()

    def follow_solution_trajectory(self):
        """
        Follow/Replay a trajectory provided by a CommonRoad solution file.
        """
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
        Callback to initial pose changes. Save message for later processing
        :param msg: Initial Pose message
        """

        self._logger.info("Received new initial pose!")
        self.initial_pose = msg
        self.new_initial_pose = True
        self.new_pose_received = False

    def routing_state_callback(self, msg: RouteState) -> None:
        """
        Callback to routing state. Checks if clear route button was pressed.
        """

        self.routing_state = msg.state
       
        # clear route if clear_route button is pressed in RVIZ 
        # routing state gets set to UNSET when clear_route button is pressed
        if msg.state == RouteState.UNSET:
            # save last AutowareState and Time Stamp
            aw_stamp = self.last_msg_aw_stamp
            aw_state = self.last_msg_aw_state

            # check for correct AutowareState:
            # set_state() method is setting a wrong AutowareState with Time Stamp 0.0, so we have to wait until a correct AutowareState is published
            if aw_stamp.sec == 0:
                self._logger.info("set_state() published AutowareState with Time Stamp: 0. Waiting for new AutowareState message!")
                # !!! Sleep time implemented to wait for correct AutowareState !!!
                # !!! Waiting time is depending on the AutowareState publisher rate !!!
                time.sleep(0.11)
                aw_stamp = self.last_msg_aw_stamp
                aw_state = self.last_msg_aw_state

                # check if AutowareState is still wrong and wait again
                # wrong AutowareState with Time Stamp 0.0 is published in total 2 times. No further waiting is necessary.
                if aw_stamp.sec == 0:
                    self._logger.info("set_state() published AutowareState with Time Stamp: 0. Waiting for new AutowareState message!")
                    # !!! Sleep time implemented to wait for correct AutowareState !!!
                    # !!! Waiting time is depending on the AutowareState publisher rate !!!
                    time.sleep(0.11)
                    aw_stamp = self.last_msg_aw_stamp
                    aw_state = self.last_msg_aw_state

            # only clear route if AutowareState is PLANNING or WAITING_FOR_ENGAGE or DRIVING
            # for DRIVING we have to wait until velocity is zero before clearing the route
            if aw_state == AutowareState.PLANNING or aw_state == AutowareState.WAITING_FOR_ENGAGE:
                self._logger.info("Clearing route!")
                self.clear_route()
            elif aw_state == AutowareState.DRIVING:
                self._logger.info("Clear route while driving!")
                self.waiting_for_velocity_0 = True
                self.clear_route()
        
    def clear_route(self):
        """
        Clear route and set AutowareState to WAITING_FOR_ROUTE.
        """
        self.route_planned = False
        self.planning_problem = None
        self.set_state(AutowareState.WAITING_FOR_ROUTE)

        # publish empty trajectory
        self.route_planner._pub_route([], [])

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback to goal pose. Safe message to goal message list and set as active goal if no goal is active.
        :param msg: Goal Pose message
        """
        self._logger.info("Received new goal pose!")
        # post process goal pose z coordinate
        msg.pose.position.z = self.scenario_handler.get_z_coordinate()

        self.goal_msgs.append(msg)

        if self.verbose:
            self._logger.info("goal msg: " + str(msg))

        self._pub_goals()
        # autoware requires that the reference path has to be published again when new goals are published
        if self.velocity_planner.get_is_velocity_planning_completed():
            point_list, reference_velocities = self.velocity_planner.get_reference_velocities()
            # post process reference path z coordinate
            for point in point_list: 
                point.z = self.scenario_handler.get_z_coordinate()
            self.route_planner._pub_route(point_list, reference_velocities)
    
    def velocity_limit_callback(self, msg: VelocityLimit) -> None:
        """
        Callback to external velocity limit from API. We directly subscribe the topic
        /planning/scenario_planning/max_velocity_default sent from the API in RVIZ.
        :param msg: Veloctiy Limit message
        """
        self._set_external_velocity_limit(vel_limit=msg.max_velocity)

    def set_state(self, new_aw_state: AutowareState):
        self.aw_state.state = new_aw_state
        if self.verbose:
            self._logger.info("Setting new AutowareState to: " + str(new_aw_state))
        self.aw_state_pub.publish(self.aw_state)

        # publish /vehicle/engage=True if in DRIVING
        if new_aw_state == AutowareState.DRIVING:
            self.engage_status = True
        elif new_aw_state == AutowareState.ARRIVED_GOAL:
            # publish routing state if goal arrived
            routing_state_msg = RouteState()
            routing_state_msg.state = 3
            self.routing_state_pub.publish(routing_state_msg)
            # call client for change to stop service
            change_to_stop_response = self.change_to_stop_client.call(self.change_to_stop_request)
            # set /vehicle/engage to False if goal arrived
            self.engage_status = False
        elif self.waiting_for_velocity_0:
            # wait until velocity is zero
            init_state = self.ego_vehicle_handler.ego_vehicle_state
            start_time = time.time()
            while abs(init_state.velocity) > 0.01:
                init_state = self.ego_vehicle_handler.ego_vehicle_state
                # self._logger.debug("Stop initiated! Velocity is not zero.")
                self._logger.debug("Velocity: " + str(init_state.velocity))
                if time.time() - start_time > 15:
                    self._logger.error("Stop initiated! Velocity is not zero. Timeout!")
                    break
                time.sleep(0.5)
            # set /vehicle/engage to False if stop button is pressend and velocity of vehicle is zero
            self.engage_status = False
            self._logger.debug("Vehicle stoped! Setting /vehicle/engage to False")

            self.waiting_for_velocity_0 = False    
        else:
            self.engage_status = False

        # Send /vehicle/engage signal
        engage_msg = Engage()
        engage_msg.engage = self.engage_status
        self.vehicle_engage_pub.publish(engage_msg)
        
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
            if self.verbose:
                self._logger.info("Setting new goal")

            self.set_state(AutowareState.PLANNING)

            current_msg = self.goal_msgs.pop(0)
            self.current_goal_msg = deepcopy(current_msg)

            self._logger.info("Pose position: " + str(current_msg.pose.position))

            position = utils.map2utm(self.origin_transformation, current_msg.pose.position)
            pos_x = position[0]
            pos_y = position[1]
            self._logger.info("Pose position utm: " + str(position))
            orientation = utils.quaternion2orientation(current_msg.pose.orientation)
            if self.ego_vehicle_handler.ego_vehicle_state is None:
                self._logger.error("ego vehicle state is None")
                return

            max_vel = self.get_parameter("vehicle.max_velocity").get_parameter_value().double_value
            min_vel = self.get_parameter("vehicle.min_velocity").get_parameter_value().double_value
            velocity_interval = Interval(min_vel, max_vel)

            # get goal lanelet and its width
            # subtract commonroad map origin
            goal_lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position(
                [np.array([pos_x, pos_y])]
            )

            if self.verbose:
                self._logger.info("goal pos_x: " + str(pos_x) + ", pos_y: " + str(pos_y))

            if goal_lanelet_id == [[]]:
                self._logger.error("No lanelet found at goal position!")
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
                length=self.ego_vehicle_handler.vehicle_length
                + 0.25 * self.ego_vehicle_handler.vehicle_length,
                width=goal_lanelet_width,
                center=position,
                orientation=orientation,
            )
            goal_state = CustomState(
                position=region,
                time_step=Interval(0, 1000),
                velocity=velocity_interval,
            )

            goal_region = GoalRegion([goal_state])
            self.planning_problem = PlanningProblem(
                planning_problem_id=1,
                initial_state=self.ego_vehicle_handler.ego_vehicle_state,
                goal_region=goal_region,
            )
            self._logger.info("Set new goal active!")
            self.route_planner.plan(self.planning_problem)
            self.velocity_planner.send_reference_path(
                [
                    utils.utm2map(self.origin_transformation, point)
                    for point in self.route_planner.reference_path
                ],
                self.current_goal_msg.pose.position,
            )
            self._pub_goals()
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)
            self.route_planned = True
        else:
            if self.verbose:
                self._logger.info("No new goal could be set")

    def state_callback(self, msg: AutowareState) -> None:
        """
        Callback to autoware state. Save the message for later processing.
        :param msg: autoware state message
        """
        self.last_msg_aw_stamp = msg.stamp
        self.last_msg_aw_state = msg.state

    # The engage signal is sent by the tum_state_rviz_plugin
    # msg.engage sent by tum_state_rviz_plugin will always be true
    def auto_button_callback(self, msg: Engage) -> None:
        self.auto_button_status = msg.engage
        
        self._logger.info(
            "Auto Button message received! Auto Button Status: "
            + str(self.auto_button_status)
            + ", current AutowareState: "
            + str(self.get_state())
        )

        # Update Autoware state panel
        if self.auto_button_status and self.get_state() == AutowareState.WAITING_FOR_ENGAGE and self.routing_state != RouteState.UNSET:
            self.set_state(AutowareState.DRIVING)
            if self.PUBLISH_OBSTACLES:  # publish obstacle at once after engaged
                self.scenario_handler.publish_initial_obstacles()
                self.PUBLISH_OBSTACLES = False

        if not self.engage_status and self.get_state() == AutowareState.DRIVING:
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

        if not self.auto_button_status and self.get_state() == AutowareState.DRIVING and self.routing_state == RouteState.SET:
            self.waiting_for_velocity_0 = True
            self._logger.info("Stop button pressed!")
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)
            
            self.route_planner.plan(self.planning_problem)
            self.velocity_planner.send_reference_path(
                [
                    utils.utm2map(self.origin_transformation, point)
                    for point in self.route_planner.reference_path
                ],
                self.current_goal_msg.pose.position,
            )
            # self._pub_goals()
            # self.set_state(AutowareState.WAITING_FOR_ENGAGE)
            # self.route_planned = True
     


                        
        """
        # reset follow sultion trajectory simulation if interface is in trajectory follow mode and goal is reached
        if not self.interactive_mode and self.get_state() == AutowareState.ARRIVED_GOAL:
            self.follow_solution_trajectory()"""

    def _prepare_traj_msg(self, states):
        """
        Prepares trajectory to match autoware format. Publish the trajectory.
        :param states: trajectory points
        :param contains_goal: flag to reduce speed over the last 10 steps
        """
        if self.verbose:
            self._logger.info("Preparing trajectory message!")

        self.traj = AWTrajectory()
        self.traj.header.frame_id = "map"

        if states == []:
            self.traj_pub.publish(self.traj)
            self._logger.info("New empty trajectory published !!!")
            return

        position_list = []
        for i in range(0, len(states)):
            new_point = TrajectoryPoint()
            new_point.pose.position = utils.utm2map(self.origin_transformation, states[i].position)
            # post process trajectory z coordinate
            new_point.pose.position.z = self.scenario_handler.get_z_coordinate()
            position_list.append([states[i].position[0], states[i].position[1], self.scenario_handler.get_z_coordinate()])
            new_point.pose.orientation = utils.orientation2quaternion(states[i].orientation)
            new_point.longitudinal_velocity_mps = float(states[i].velocity)

            # front_wheel_angle_rad not given by autoware planner
            # new_point.front_wheel_angle_rad = states[i].steering_angle
            new_point.acceleration_mps2 = float(states[i].acceleration)
            self.traj.points.append(new_point)

        self.traj_pub.publish(self.traj)
        self._logger.info("New trajectory published !!!")
        # visualize_solution(self.scenario, self.planning_problem, create_trajectory_from_list_states(path)) #ToDo: test

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
            self._logger.error(
                f"Failed to transform from {source_frame} to {target_frame} frame."
            )
            return

        try:
            tf_map = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time.from_msg(pose_in.header.stamp),
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
