# standard imports
from copy import deepcopy
import os
import traceback
from typing import Optional
import datetime
import time
import datetime

# third party imports
import numpy as np
import matplotlib

if os.environ.get('DISPLAY') is not None:
    matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# Autoware.Auto message imports
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory  # type: ignore
from autoware_auto_system_msgs.msg import AutowareState  # type: ignore
from autoware_auto_vehicle_msgs.msg import Engage  # type: ignore

# Autoware AdAPI message imports
from autoware_adapi_v1_msgs.msg import RouteState  # type: ignore
from autoware_adapi_v1_msgs.srv import ChangeOperationMode  # type: ignore

# Tier IV message imports
from tier4_planning_msgs.msg import VelocityLimit  # type: ignore

# ROS message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
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
from cr2autoware.common.configuration import CR2AutowareParams
from .handlers.scenario_handler import ScenarioHandler
from .handlers.ego_vehicle_handler import EgoVehicleHandler
from .handlers.data_generation_handler import DataGenerationHandler
from .handlers.planning_problem_handler import PlanningProblemHandler
from .interfaces.implementation.cr_route_planner import CommonRoadRoutePlanner
from .interfaces.implementation.velocity_planner import VelocityPlanner
from .interfaces.implementation.rp_interface import ReactivePlannerInterface
from cr2autoware.common.utils.tf2_geometry_msgs import do_transform_pose
from cr2autoware.common.utils.trajectory_logger import TrajectoryLogger
from .common.utils.transform import orientation2quaternion
from .common.utils.transform import quaternion2orientation
from .common.utils.transform import map2utm
from .common.utils.transform import utm2map
from .common.utils.message import create_goal_marker
from .common.ros_interface.create import create_subscription, create_publisher, create_client

# subscriber specifications
from .common.ros_interface.specs_subscriptions import \
    spec_initial_pose_sub, spec_auto_button_sub, spec_velocity_limit_sub, spec_routing_state_sub, \
    spec_autoware_state_sub, spec_echo_back_goal_pose_sub

# publisher specifications
from .common.ros_interface.specs_publisher import \
    spec_goal_pose_pub, spec_traj_pub, spec_aw_state_pub, spec_vehicle_engage_pub, spec_api_engage_pub, \
    spec_routing_state_pub, spec_route_pub, spec_velocity_pub, spec_initial_pose_pub, spec_goal_region_pub, \
    spec_velocity_limit_pub, spec_velocity_limit_pub_vis

# service client specifications
from .common.ros_interface.specs_clients import \
    spec_change_to_stop_client


class Cr2Auto(Node):
    """
    Cr2Auto class that is an instance of a ROS2 Node.

    This node serves as a the main interface between CommonRoad and Autoware.Universe

    ----------------
    **Publishers:**

    * goal_pose_pub: 
        * Description: Goal pose of the vehicle.
        * Topic: `/planning/mission_planning/goal`
        * Message Type: `geometry_msgs.msg.PoseStamped`
    * traj_pub:
        * Description: Trajectory of the vehicle.
        * Topic: `/planning/commonroad/trajectory`
        * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
    * aw_state_pub:
        * Description: Autoware state.
        * Topic: `/autoware/state`
        * Message Type: `autoware_auto_system_msgs.msg.AutowareState`
    * vehicle_engage_pub:
        * Description: Engage message for Autoware Planning Simulation.
        * Topic: `/vehicle/engage`
        * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
    * api_engage_pub:
        * Description: Publish engage message for node `/control/operation_mode_transistion_manager`.
        * Topic: `/api/autoware/get/engage`
        * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
    * routing_state_pub:
        * Description: Routing state.
        * Topic: `/api/routing/state`
        * Message Type: `autoware_adapi_v1_msgs.msg.RouteState`
    * route_pub:
        * Description: Route marker.
        * Topic: `/planning/mission_planning/route_marker`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * velocity_pub:
        * Description: Reference trajectory to motion velocity smoother.
        * Topic: `/planning/scenario_planning/scenario_selector/trajectory`
        * Message Type: `autoware_auto_planning_msgs.msg.Trajectory`
    * initial_pose_pub:
        * Description: Initial pose of the vehicle.
        * Topic: `/initialpose3d`
        * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
    * goal_region_pub:
        * Description: Goal region of the vehicle.
        * Topic: `/goal_region_marker_array`
        * Message Type: `visualization_msgs.msg.MarkerArray`
    * velocity_limit_pub:
        * Description: Maximum velocity limit.
        * Topic: `/planning/scenario_planning/max_velocity`
        * Message Type: `tier4_planning_msgs.msg.VelocityLimit`
    * velocity_limit_pub_vis:
        * Description: Maximum velocity limit for visualization in RVIZ.
        * Topic: `/planning/scenario_planning/current_max_velocity`
        * Message Type: `tier4_planning_msgs.msg.VelocityLimit`

    ----------------
    **Subscribers:**

    * initialpose_sub:
        * Description: Initial pose of the vehicle
        * Topic: `/initialpose3d`
        * Message Type: `geometry_msgs.msg.PoseWithCovarianceStamped`
    * goal_pose_sub:
        * Description: Goal pose of the vehicle
        * Topic: `/planning/mission_planning/echo_back_goal_pose`
        * Message Type: `geometry_msgs.msg.PoseStamped`
    * auto_button_sub:
        * Description: Engage message for Autoware
        * Topic: `/autoware/engage`
        * Message Type: `autoware_auto_vehicle_msgs.msg.Engage`
    * vel_limit_sub:
        * Description: Maximum velocity limit
        * Topic: `/planning/scenario_planning/max_velocity_default`
        * Message Type: `tier4_planning_msgs.msg.VelocityLimit`
    * autoware_state_sub:
        * Description: Autoware state
        * Topic: `/autoware/state`
        * Message Type: `autoware_auto_system_msgs.msg.AutowareState`
    * routing_state_sub:
        * Description: Routing state
        * Topic: `/api/routing/state`
        * Message Type: `autoware_adapi_v1_msgs.msg.RouteState`

    ----------------
    **Service Clients:**

    * `change_to_stop_client`: Service client for change to stop service call
    
    ----------------
    :var scenario_handler: Instance of the ScenarioHandler class
    :var plan_prob_handler: Instance of the PlanningProblemHandler class
    :var params: Instance of the CR2AutowareParams class
    :var _logger: Instance of the logger
    :var verbose: Boolean to enable verbose logging
    :var callback_group: Callback group for async execution
    :var write_scenario: Boolean to write scenario to file
    :var solution_path: Path to solution file
    :var rnd: MPRenderer
    :var trajectory_logger: Instance of the TrajectoryLogger class
    :var origin_transformation: Transformation from map to UTM coordinates
    :var ego_vehicle_handler: Instance of the EgoVehicleHandler class
    :var is_computing_trajectory: Boolean to check if trajectory is being computed
    :var new_initial_pose: Boolean to check if new initial pose is received
    :var external_velocity_limit: External velocity limit
    :var tf_buffer: Buffer for tf2_ros
    :var tf_listener: Transform listener for tf2_ros
    :var aw_state: Instance of the AutowareState class
    :var engage_status: Boolean to check if vehicle is engaged
    :var auto_button_status: Boolean to check if auto button is pressed
    :var waiting_for_velocity_0: Boolean to check if waiting for velocity to be zero
    :var routing_state: Instance of the RouteState class
    :var last_msg_aw_state: Last message AutowareState
    :var last_msg_aw_stamp: Last message timestamp
    :var goal_msgs: List of goal messages
    :var current_goal_msg: Current goal message
    :var last_goal_reached: Last goal reached timestamp
    :var initial_pose: Initial pose message
    :var trajectory_planner_type: Trajectory planner type
    :var route_planner: Instance of the CommonRoadRoutePlanner class
    :var trajectory_planner: Instance of the ReactivePlannerInterface class
    :var interactive_mode: Boolean to check if interactive mode is enabled
    :var timer_solve_planning_problem: Timer for solving planning problem
    :var timer_follow_trajectory_mode_update: Timer for updating follow trajectory mode
    :var save_data_path: Path to save data
    :var data_generation_handler: Instance of the DataGenerationHandler class
    """

    scenario_handler: ScenarioHandler
    plan_prob_handler: PlanningProblemHandler

    def __init__(self):
        """
        Constructor of the Cr2Auto class.

        Initializes the Cr2Auto node and sets up the ROS2 interfaces. Sets Autoware state to `WAITING_FOR_ROUTE`.
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

        # initialize planning-specific attributes
        self.is_computing_trajectory = False  # stop update scenario when trajectory is being computed
        self.new_initial_pose = False
        self.external_velocity_limit = 1337.0

        # initialize tf
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames

        # initialize AW API-related variables
        self.aw_state = AutowareState()
        self.engage_status = False
        self.auto_button_status = False
        self.waiting_for_velocity_0 = False
        self.routing_state = RouteState()

        # vars to save last messages
        # https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl
        self.last_msg_aw_state = 1  # 1 = initializing
        self.last_msg_aw_stamp = self.get_clock().now()
        self.goal_msgs = []
        self.current_goal_msg = None
        self.last_goal_reached = self.get_clock().now()
        self.initial_pose = None

        # ========= Subscribers =========
        # subscribe initial pose
        self.initialpose_sub = create_subscription(self, spec_initial_pose_sub, self.initial_pose_callback,
                                                   self.callback_group)

        # subscribe goal pose
        self.goal_pose_sub = create_subscription(self, spec_echo_back_goal_pose_sub, self.goal_pose_callback,
                                                 self.callback_group)

        # subscribe autoware engage message
        self.auto_button_sub = create_subscription(self, spec_auto_button_sub, self.auto_button_callback,
                                                   self.callback_group)

        # subscribe velocity limit from API
        self.vel_limit_sub = create_subscription(self, spec_velocity_limit_sub, self.velocity_limit_callback,
                                                 self.callback_group)

        # subscribe autoware state
        self.autoware_state_sub = create_subscription(self, spec_autoware_state_sub, self.state_callback,
                                                      self.callback_group)

        # subscribe routing state
        self.routing_state_sub = create_subscription(self, spec_routing_state_sub, self.routing_state_callback,
                                                     self.callback_group)

        # ========= Publishers =========
        # publish goal pose
        self.goal_pose_pub = create_publisher(self, spec_goal_pose_pub)

        # publish trajectory
        self.traj_pub = create_publisher(self, spec_traj_pub)

        # publish autoware state
        self.aw_state_pub = create_publisher(self, spec_aw_state_pub)

        # publish vehicle engage for AW Planning Simulation
        self.vehicle_engage_pub = create_publisher(self, spec_vehicle_engage_pub)

        # publish engage required by node /control/operation_mode_transistion_manager
        self.api_engage_pub = create_publisher(self, spec_api_engage_pub)

        # publish routing state
        self.routing_state_pub = create_publisher(self, spec_routing_state_pub)

        # publish route marker
        self.route_pub = create_publisher(self, spec_route_pub)

        # publish reference trajectory to motion velocity smoother
        self.velocity_pub = create_publisher(self, spec_velocity_pub)

        # publish initial state of the scenario (replay solution trajectory mode)
        self.initial_pose_pub = create_publisher(self, spec_initial_pose_pub)

        # publish goal region(s) of the scenario (TODO: use correct topic)
        self.goal_region_pub = create_publisher(self, spec_goal_region_pub)

        # publish max velocity limit
        # (currently only subscribed by Motion Velocity Smoother, if we use the external_velocity_limit_selector node,
        #  this publisher can be removed)
        self.velocity_limit_pub = create_publisher(self, spec_velocity_limit_pub)

        # publish current velocity limit for display in RVIZ
        # (this separate topic is currently only subscribed by RVIZ)
        self.velocity_limit_pub_vis = create_publisher(self, spec_velocity_limit_pub_vis)

        # ========= Service Clients =========
        # client for change to stop service call (only for publishing "stop" if goal arrived)
        self.change_to_stop_client = create_client(self, spec_change_to_stop_client)

        self.change_to_stop_request = ChangeOperationMode.Request()

        # ========= Set up Planner Interfaces =========
        # set initial Autoware State (Waiting for route)
        self.set_state(AutowareState.WAITING_FOR_ROUTE)

        # get trajectory_planner_type
        self.trajectory_planner_type = \
            self.get_parameter("trajectory_planner.trajectory_planner_type").get_parameter_value().integer_value

        # set route planner using factory method
        self.route_planner = self._route_planner_factory()

        # set velocity planner
        self._set_velocity_planner()

        # set trajectory planner using factory function
        self.trajectory_planner = self._trajectory_planner_factory()

        # set mode of interface (interactive or replay mode)
        self._set_cr2auto_mode()

        # set initial velocity limit (from launch config, the limit can be changed during runtime via API)
        self._set_external_velocity_limit(
            vel_limit=self.get_parameter("vehicle.max_velocity").get_parameter_value().double_value)

        # data generation handler
        self.save_data_path: str = self.get_parameter(
            "general.map_path").get_parameter_value().string_value + "/data_saving"

        self.data_generation_handler: DataGenerationHandler = DataGenerationHandler(
            cr2aw_node=self,
            save_path=self.save_data_path,
            save_id=datetime.datetime.now().strftime("%Y_%m_%d_%H_%M"),
            origin_transformation=self.origin_transformation
        )

        self.data_generation_handler.start_recording()

        # ========= Finish init() =========
        if self.verbose:
            self._logger.info("Cr2Auto initialization is completed!")

    @property
    def scenario(self) -> Scenario:
        """
        Get scenario object retrieved from the scenario_handler.

        Caution: Does not trigger an update of the scenario.

        :return: CommonRoad scenario
        """
        if self.scenario_handler is None:
            raise RuntimeError("Scenario handler not initialized.")
        return self.scenario_handler.scenario

    @property
    def planning_problem(self) -> Optional[PlanningProblem]:
        """
        Get planning problem object retrieved from the planning problem handler.

        Caution: Does not trigger an update of the planning problem.

        :return: CommonRoad planning problem
        """
        if self.plan_prob_handler is None:
            raise RuntimeError("Planning problem handler not initialized.")
        return self.plan_prob_handler.planning_problem

    @planning_problem.setter
    def planning_problem(self, planning_problem):
        """Set planning problem in the planning problem handler."""
        self.plan_prob_handler.planning_problem = planning_problem

    def _set_velocity_planner(self) -> None:
        """Initializes the velocity planner"""
        self.velocity_planner = VelocityPlanner(
            self.velocity_pub,
            self._logger,
            self.verbose,
            self.get_parameter("velocity_planner.lookahead_dist").get_parameter_value().double_value,
            self.get_parameter("velocity_planner.lookahead_time").get_parameter_value().double_value,
        )

        # subscribe trajectory from motion velocity smoother
        self.traj_sub_smoothed = self.create_subscription(
            AWTrajectory,
            "/planning/scenario_planning/trajectory_smoothed",
            self.velocity_planner.smoothed_trajectory_callback,
            1,
            callback_group=self.callback_group
        )

    # TODO move factory method to separate module
    def _trajectory_planner_factory(self) -> ReactivePlannerInterface:
        """
        Factory function to initialize trajectory planner according to specified type.

        :return: Instance of the ReactivePlannerInterface class
        :raises _logger.error: If trajectory planner type is invalid
        """
        if self.trajectory_planner_type == 1:  # Reactive planner
            return ReactivePlannerInterface(self.traj_pub,
                                            self._logger,
                                            self.verbose,
                                            self.scenario,
                                            self.planning_problem,
                                            self.scenario_handler.road_boundary,
                                            self.scenario.dt,
                                            self.params.trajectory_planner.planning_horizon,
                                            self.params.rp_interface,
                                            self.ego_vehicle_handler)
        else:
            self._logger.error("<Trajectory Planner Factory> Planner type is invalid")

    # TODO move factory method to separate module
    def _route_planner_factory(self) -> CommonRoadRoutePlanner:
        """
        Factory function to initialize route planner according to specified type.
        
        :return: Instance of the CommonRoadRoutePlanner class
        """
        return CommonRoadRoutePlanner(route_pub=self.route_pub,
                                      logger=self._logger,
                                      verbose=self.verbose,
                                      lanelet_network=self.scenario_handler.lanelet_network,
                                      planning_problem=self.planning_problem)

    def _set_cr2auto_mode(self) -> None:
        """Decide whether the CR2Autoware interface goes in interactive planning mode or trajectory following mode."""
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

    def _set_external_velocity_limit(self, vel_limit: float) -> None:
        """
        Sets the velocity limit for CR2Autoware. 

        Publishes the velocity limit for RVIZ. (Can be removed if external_velocity_limit_selector node is used).

        :param vel_limit: Maximum velocity limit
        """
        # set limit
        self.external_velocity_limit = max(0.0, vel_limit)

        # publish velocity limit message
        vel_limit_msg = VelocityLimit()
        vel_limit_msg.max_velocity = vel_limit
        # publish max velocity for other modules
        self.velocity_limit_pub.publish(vel_limit_msg)
        # publish max velocity for display in RVIZ
        self.velocity_limit_pub_vis.publish(vel_limit_msg)

    def solve_planning_problem(self) -> None:
        """
        Main planning function.

        Update loop for interactive planning mode and solve planning problem with algorithms offered by CommonRoad.

        * Checks if a planning problem is already being solved
        * Compute reference path if initial pose was changed
        * Plan and publish trajectory


        :raises Exception: If an error occurs during planning
        :raises Exception: if new goal pose cannot be set
        :raises _logger.info: If a planning problem is already being solved
        """
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
                    # check if the current_vehicle_state was already updated (pose received by current state callback),
                    # otherwise wait one planning cycle
                    if not self.ego_vehicle_handler.new_pose_received:
                        self.is_computing_trajectory = False
                        return

                    self.new_initial_pose = False
                    self.ego_vehicle_handler.new_pose_received = False
                    self._logger.info("Replanning route to goal")

                    # insert current goal into list of goal messages
                    if self.current_goal_msg:
                        self.goal_msgs.insert(0, self.current_goal_msg)
                    # call reset function of route planner
                    self.route_planner.reset()

                if not self.route_planner.is_route_planned:
                    # if currently no active goal, set a new goal (if one exists)
                    try:
                        self._set_new_goal()
                    except Exception:
                        self._logger.error(traceback.format_exc())

                if self.route_planner.is_route_planned:
                    if not self.velocity_planner.is_velocity_planning_completed:
                        self._logger.info(
                            "Can't run route planner because interface is still waiting for velocity planner"
                        )
                        _goal_pos_cr = map2utm(self.origin_transformation, self.current_goal_msg.pose.position)
                        self.velocity_planner.plan(self.route_planner.reference_path, _goal_pos_cr,
                                                   self.origin_transformation)
                        self.is_computing_trajectory = False
                        return

                    if not self.route_planner.is_ref_path_published:
                        # publish current reference path
                        point_list = self.velocity_planner.reference_positions
                        reference_velocities = self.velocity_planner.reference_velocities
                        # call publisher
                        self.route_planner.publish(point_list, reference_velocities,
                                                   self.scenario_handler.z_coordinate)

                    if self.verbose:
                        self._logger.info("Solving planning problem!")

                    # Get current initial state for planning
                    # The initial velocity needs to be increase here due to a hardcoded velocity threshold in
                    # AW. Universe Shift_Decider Package (If velocity is below 0.01, the gear will remain in park)

                    init_state = self.ego_vehicle_handler.ego_vehicle_state
                    if init_state.velocity < 0.01:
                        init_state.velocity = 0.01

                    if self.trajectory_planner_type == 1:  # Reactive Planner
                        reference_velocity = max(
                            1.0,
                            self.velocity_planner.get_lookahead_velocity_for_current_state(
                                self.ego_vehicle_handler.current_vehicle_state.pose.pose.position,
                                self.ego_vehicle_handler.ego_vehicle_state.velocity),
                        )

                        if self.verbose:
                            self._logger.info("Running trajectory planner")

                        # set reference velocity considering external limit
                        ref_vel = min(reference_velocity, self.external_velocity_limit)

                        # call the one-step plan function
                        self.trajectory_planner.plan(
                            current_state=init_state,
                            goal=self.planning_problem.goal,
                            reference_velocity=ref_vel)

                        # publish trajectory
                        self.trajectory_planner.publish(self.origin_transformation,
                                                        self.scenario_handler.z_coordinate)

                    # check if goal is reached
                    self._is_goal_reached()

                self.is_computing_trajectory = False
            else:
                if self.verbose:
                    self._logger.info("already solving planning problem")

        except Exception:
            self._logger.error(traceback.format_exc())

    def follow_trajectory_mode_update(self) -> None:
        """
        Update mode for follow trajectory mode. It checks if the goal position is reached.

        :raises Exception: If an error occurs during the update
        """
        try:
            if self.verbose:
                self._logger.info("Next cycle of follow trajectory mode")

            # update scenario
            self.ego_vehicle_handler.update_ego_vehicle()
            self.scenario_handler.update_scenario()
            self.plot_save_scenario()

            if not self.route_planner.is_route_planned:
                # try to set a new goal position
                self._set_new_goal()
            else:
                # check if goal is reached
                self._is_goal_reached()
        except Exception:
            self._logger.error(traceback.format_exc())

    def plot_save_scenario(self) -> None:
        """Plot and save scenario if enabled in launch file."""
        if self.get_parameter("general.plot_scenario").get_parameter_value().bool_value:
            self._plot_scenario()

        if self.write_scenario:
            self._write_scenario()

    def _is_goal_reached(self) -> None:
        """Check if vehicle is in goal region. If in goal region set new goal."""
        if self.planning_problem:
            if (
                    self.planning_problem.goal.is_reached(self.ego_vehicle_handler.ego_vehicle_state)
                    and (self.get_clock().now() - self.last_goal_reached).nanoseconds > 5e8
            ):
                self._logger.info("Vehicle arrived at goal!")
                self.last_goal_reached = self.get_clock().now()

                # Data saving
                self._logger.info("calling data_generation_handler to save scenario")
                self.data_generation_handler.stop_recording_and_save_data()

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
                    # call reset function of route planner
                    self.route_planner.reset()
                    self.plan_prob_handler.planning_problem = None
                    self.set_state(AutowareState.ARRIVED_GOAL)
                else:
                    self._set_new_goal()

    def follow_solution_trajectory(self) -> None:
        """Follow/Replay a trajectory provided by a CommonRoad solution file."""
        states = self.trajectory_logger.load_trajectory(self.solution_path)

        # set initial pose to first position in solution trajectory and publish it
        initial_pose_msg = PoseWithCovarianceStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        initial_pose_msg.header = header
        pose = Pose()
        pose.position = utm2map(self.origin_transformation, states[0].position)
        pose.orientation = orientation2quaternion(states[0].orientation)
        initial_pose_msg.pose.pose = pose
        self.initial_pose_pub.publish(initial_pose_msg)

        # set goal to last position in solution trajectory and publish it
        goal_msg = PoseStamped()
        goal_msg.header = header
        pose = Pose()
        pose.position = utm2map(self.origin_transformation, states[-1].position)
        pose.orientation = orientation2quaternion(states[-1].orientation)
        goal_msg.pose = pose
        self.goal_pose_pub.publish(goal_msg)

        # publish solution trajectory
        # TODO this doesn't work anymore -> FIX

        self.set_state(AutowareState.WAITING_FOR_ENGAGE)

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Callback to initial pose changes. Save message for later processing.

        :param msg: Initial Pose message
        """

        self._logger.info("Received new initial pose!")
        self.initial_pose = msg
        self.new_initial_pose = True
        self.ego_vehicle_handler.new_pose_received = False

        # (re)-compute elevation (z-coordinate) when new initial pose is received
        self.scenario_handler.compute_z_coordinate(self.initial_pose, self.current_goal_msg)

    def routing_state_callback(self, msg: RouteState) -> None:
        """
        Callback to routing state. Checks if "Clear route" button was pressed.

        Pressing the "Clear Route" button in RVIZ sets the routing state to `UNSET`. Then the currently planned route and
        reference path should be removed.

        :param msg: Route State message
        """
        self.routing_state = msg.state

        # routing state = UNSET when"Clear route" button is pressed in RVIZ
        if msg.state == RouteState.UNSET:
            # save last AutowareState and Time Stamp
            aw_stamp = self.last_msg_aw_stamp
            aw_state = self.last_msg_aw_state

            # check for correct AutowareState:
            # set_state() method sets a wrong AutowareState with Time Stamp 0.0
            # We have to wait until a correct AutowareState is published
            if aw_stamp.sec == 0:
                self._logger.info("set_state() published AutowareState with Time Stamp: 0. "
                                  "Waiting for new AutowareState message!")
                # Sleep time implemented to wait for correct AutowareState
                # Waiting time is depending on the AutowareState publisher rate
                time.sleep(0.11)
                aw_stamp = self.last_msg_aw_stamp
                aw_state = self.last_msg_aw_state

                # check if AutowareState is still wrong and wait again
                # wrong AutowareState with Time Stamp 0.0 is published in total 2 times.
                # No further waiting is necessary.
                if aw_stamp.sec == 0:
                    self._logger.info("set_state() published AutowareState with Time Stamp: 0. "
                                      "Waiting for new AutowareState message!")
                    # Sleep time implemented to wait for correct AutowareState
                    # Waiting time is depending on the AutowareState publisher rate
                    time.sleep(0.11)
                    aw_stamp = self.last_msg_aw_stamp
                    aw_state = self.last_msg_aw_state

            # Clear route if AutowareState is PLANNING or WAITING_FOR_ENGAGE
            if aw_state == AutowareState.PLANNING or aw_state == AutowareState.WAITING_FOR_ENGAGE:
                self._logger.info("Clearing route!")
                self.clear_route()
            # Clear route if AutowareState is DRIVING: This is equivalent to first pressing STOP button and then
            # clearing the route
            elif aw_state == AutowareState.DRIVING:
                self._logger.info("Clear route while driving!")
                self.waiting_for_velocity_0 = True
                self.clear_route()

    def clear_route(self) -> None:
        """Clear route and set AutowareState to `WAITING_FOR_ROUTE`."""
        # call reset function of route planner
        self.route_planner.reset()
        self.plan_prob_handler.planning_problem = None
        self.set_state(AutowareState.WAITING_FOR_ROUTE)

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback to goal pose. Safe message to goal message list and set as active goal if no goal is active.

        :param msg: Goal Pose message
        """
        self._logger.info("Received new goal pose!")

        self.goal_msgs.append(msg)

        if self.verbose:
            self._logger.info("goal msg: " + str(msg))

        # (re)-compute elevation (z-coordinate) when new goal pose is received
        self.scenario_handler.compute_z_coordinate(self.initial_pose, msg)

        self._pub_goals()
        # autoware requires that the reference path has to be published again when new goals are published
        if self.velocity_planner.is_velocity_planning_completed:
            point_list = self.velocity_planner.reference_positions
            reference_velocities = self.velocity_planner.reference_velocities
            # call publisher
            self.route_planner.publish(point_list, reference_velocities,
                                       self.scenario_handler.z_coordinate)

    def velocity_limit_callback(self, msg: VelocityLimit) -> None:
        """
        Callback to external velocity limit from API.

        We directly subscribe the topic /planning/scenario_planning/max_velocity_default sent from the API in RVIZ.

        :param msg: Velocity Limit message
        """
        self._set_external_velocity_limit(vel_limit=msg.max_velocity)

    def set_state(self, new_aw_state: AutowareState) -> None:
        """
        Set new Autoware state and publish it.

        For specific Autoware states, the `/vehicle/engage` signal is set.

        * AW state `DRIVING`: Set `/vehicle/engage` to `True`
        * AW state `ARRIVED_GOAL`: Set `/vehicle/engage` to `False`

        Special case: If the flag `waiting_for_velocity_0` is set, the vehicle waits until the velocity is zero and
        then sets `/vehicle/engage` to `False`. This flag is used for two cases:

        * When the stop button is pressed while driving
        * When clear route is pressed while driving

        :param new_aw_state: New Autoware state
        """
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
            # set /vehicle/engage to False if stop button is pressed and velocity of vehicle is zero
            self.engage_status = False
            self._logger.debug("Vehicle stopped! Setting /vehicle/engage to False")

            self.waiting_for_velocity_0 = False
        else:
            self.engage_status = False

        # Send /vehicle/engage signal
        engage_msg = Engage()
        engage_msg.engage = self.engage_status
        self.vehicle_engage_pub.publish(engage_msg)

        self.api_engage_pub.publish(engage_msg)

    def get_state(self) -> AutowareState:
        """
        Returns current Autoware state.
        
        :return: Current Autoware state
        """
        return self.aw_state.state

    def _set_new_goal(self) -> None:
        """
        Set the next goal of the goal message list active. 
        
        Calculate route to new goal. Publish new goal markers and route for visualization in RVIZ.
        """
        # set new goal if we have one
        if len(self.goal_msgs) > 0:
            if self.verbose:
                self._logger.info("Setting new goal")

            self.set_state(AutowareState.PLANNING)

            # get current goal messages
            current_msg = self.goal_msgs.pop(0)
            # TODO remove deeopcopy
            self.current_goal_msg = deepcopy(current_msg)

            self._logger.info("Goal pose position: " + str(current_msg.pose.position))

            # TODO move goal creation to PlanningProbHandler
            position = map2utm(self.origin_transformation, current_msg.pose.position)
            pos_x = position[0]
            pos_y = position[1]
            self._logger.info("Pose position utm: " + str(position))
            orientation = quaternion2orientation(current_msg.pose.orientation)
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
            self.plan_prob_handler.planning_problem = PlanningProblem(
                planning_problem_id=1,
                initial_state=self.ego_vehicle_handler.ego_vehicle_state,
                goal_region=goal_region,
            )
            self._logger.info("Set new goal active!")

            # plan route and reference path
            self.route_planner.plan(planning_problem=self.planning_problem)

            # plan velocity profile (-> reference trajectory)
            _goal_pos_cr = map2utm(self.origin_transformation, self.current_goal_msg.pose.position)
            self.velocity_planner.plan(self.route_planner.reference_path, _goal_pos_cr,
                                       self.origin_transformation)

            # publish reference path and velocity
            point_list = self.velocity_planner.reference_positions
            reference_velocities = self.velocity_planner.reference_velocities
            # TODO Uncomment after Route Pub QOS is fixed
            # self.route_planner.publish(point_list, reference_velocities, self.scenario_handler.get_z_coordinate())

            # publish goal
            self._pub_goals()

            # set AW state to Waiting for Engage
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

            # update reference path of trajectory planner
            self.trajectory_planner.update(reference_path=self.route_planner.reference_path,
                                           planning_problem=self.planning_problem)

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
        """
        Method handles processing of AUTO Button.

        * AUTO Button pressed: AutowareState is set to `DRIVING` if route is set
        * STOP Button pressed: Stopping in standstill is initiated

        :param msg: AUTO button engage message
        """
        self.auto_button_status = msg.engage

        self._logger.info(
            "AUTO Button message received! Auto Button Status: "
            + str(self.auto_button_status)
            + ", current AutowareState: "
            + str(self.get_state()))

        # Update Autoware state panel
        if self.auto_button_status and self.get_state() == AutowareState.WAITING_FOR_ENGAGE and \
                self.routing_state != RouteState.UNSET:
            self.set_state(AutowareState.DRIVING)

        if not self.engage_status and self.get_state() == AutowareState.DRIVING:
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

        # STOP button handling
        if not self.auto_button_status and self.get_state() == AutowareState.DRIVING and \
                self.routing_state == RouteState.SET:
            self.waiting_for_velocity_0 = True
            self._logger.info(
                "STOP Button message received! Auto Button Status: "
                + str(self.auto_button_status) + ", current AutowareState: " + str(self.get_state()))

            # waiting for standstill loop is implemented in set_state()
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

            # Re-plan the route and reference path
            self.route_planner.plan(planning_problem=self.planning_problem)

            # Re-plan the velocity profile
            _goal_pos_cr = map2utm(self.origin_transformation, self.current_goal_msg.pose.position)
            self.velocity_planner.plan(self.route_planner.reference_path, _goal_pos_cr,
                                       self.origin_transformation)

        # reset following trajectory simulation if interface is in trajectory follow mode and goal is reached
        # if not self.interactive_mode and self.get_state() == AutowareState.ARRIVED_GOAL:
        #    self.follow_solution_trajectory()

    def _pub_goals(self) -> None:
        """Publish the goals as markers to visualize in RVIZ."""
        # TODO move this function to Planning Problem Handler??
        goals_msg = MarkerArray()

        # first delete all marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.ns = "goals"
        marker.action = Marker.DELETEALL
        goals_msg.markers.append(marker)

        if self.current_goal_msg is not None:
            marker = create_goal_marker(self.current_goal_msg.pose.position)
            marker.id = 1
            marker.ns = "goals"
            goals_msg.markers.append(marker)

        if len(self.goal_msgs) > 0:
            for i in range(len(self.goal_msgs)):
                marker = create_goal_marker(self.goal_msgs[i].pose.position)
                marker.id = i + 2
                marker.ns = "goals"
                goals_msg.markers.append(marker)

        # TODO why is the route pub used here?? Should be the goal publisher
        self.route_pub.publish(goals_msg)

    def _plot_scenario(self) -> None:
        """ Plot the commonroad scenario."""
        # TODO: Test function
        if self.rnd is None:
            self.rnd = MPRenderer()
            plt.ion()

        self.rnd.clear()

        cr_ego_vehicle = self.ego_vehicle_handler.get_cr_ego_vehicle()

        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.facecolor = "#ff0000"
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.edgecolor = "#000000"
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.zorder = 50
        # self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.opacity = 1
        # self.ego_vehicle_handler.ego_vehicle.draw(self.rnd, draw_params={ # outdate cr-io
        #     "static_obstacle": {
        #         "occupancy": {
        #             "shape": {
        #                 "rectangle": {
        #                     "facecolor": "#ff0000",
        #                     "edgecolor": '#000000',
        #                     "zorder": 50,
        #                     "opacity": 1
        #                 }
        #             }
        #         }
        #     }
        # })
        # self.rnd.draw_params["static_obstacle"]["occupancy"]["shape"]["rectangle"]["facecolor"] = "#ff0000"
        cr_ego_vehicle.draw(self.rnd)

        # self.rnd.draw_params.lanelet.show_label = False
        self.scenario.draw(self.rnd)
        # self.planning_problem.draw(self.rnd)
        self.rnd.render()
        plt.pause(0.1)

    def _write_scenario(self) -> None:
        """Store converted map as CommonRoad scenario."""
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
        """
        Transform a `PoseStamped` message to the target frame.

        :param pose_in: Pose to transform
        :param target_frame: Target frame
        :return: Transformed pose
        :raises Exception: If tf2_ros.ExtrapolationException occurs
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
    """
    Initialize the ROS2 node and execute it in a multi-threaded environment.

    This function initializes the ROS2 node for the Cr2Auto class, adds it to a multi-threaded executor,
    and keeps it spinning to handle callbacks. Before shutting down the ROS2 context, it ensures the node
    is properly destroyed.

    :param args: Arguments passed to the ROS2 node initialization, defaults to None.
    """
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
