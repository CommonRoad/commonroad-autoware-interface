# general imports
import os
from re import A
from turtle import position
import utm
from dataclasses import dataclass
import math
import numpy as np
from pyproj import Proj
from copy import deepcopy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import traceback
import glob
import yaml
from yaml.loader import SafeLoader
from decimal import Decimal

# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# ROS message imports
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

# Autoware message imports
from autoware_auto_perception_msgs.msg import DetectedObjects, PredictedObjects
from autoware_auto_planning_msgs.msg import TrajectoryPoint, PathWithLaneId, Path
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import Engage

# commonroad imports
from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.scenario.state import CustomState
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer

from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.motion_planner.utility import create_trajectory_from_list_states
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton

#from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

from commonroad_route_planner.route_planner import RoutePlanner

# local imports
from cr2autoware.tf2_geometry_msgs import do_transform_pose
from cr2autoware.utils import visualize_solution, display_steps, orientation2quaternion, quaternion2orientation
from cr2autoware.velocity_planner import VelocityPlanner

from cr2autoware.rp_interface import RP2Interface

class Cr2Auto(Node):
    """
    Cr2Auto class that is an interface between Autoware and CommonRoad
    """
    def __init__(self):
        """
        Constructor of Cr2Auto
        """
        super().__init__('cr2autoware')

        # declare ros parameters
        self.declare_parameter('vehicle.max_velocity', 5.0)
        self.declare_parameter('vehicle.min_velocity', 1.0)
        self.declare_parameter("planner_type", 1)
        #self.declare_parameter("latitude", 0.0)
        #self.declare_parameter("longitude", 0.0)
        #self.declare_parameter("elevation", 0.0)
        #self.declare_parameter("origin_offset_lat", 0.0)
        #self.declare_parameter("origin_offset_lon", 0.0)
        self.declare_parameter('vehicle.cg_to_front', 1.0)
        self.declare_parameter('vehicle.cg_to_rear', 1.0)
        self.declare_parameter('vehicle.width', 2.0)
        self.declare_parameter('vehicle.front_overhang', 0.5)
        self.declare_parameter('vehicle.rear_overhang', 0.5)
        self.declare_parameter('map_path', '')
        #self.declare_parameter('map_osm_path', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)
        self.declare_parameter('reactive_planner.default_yaml_folder', '')
        self.declare_parameter('reactive_planner.sampling.d_min', -3)
        self.declare_parameter('reactive_planner.sampling.d_max', 3)
        self.declare_parameter('reactive_planner.sampling.t_min', 0.4)
        self.declare_parameter('reactive_planner.planning.dt', 0.1)
        self.declare_parameter('reactive_planner.planning.planning_horizon', 0.4)
        
        self.declare_parameter("write_scenario", False)
        self.declare_parameter("plot_scenario", False)
        self.declare_parameter("scenario_update_time", 0.5)
        self.declare_parameter("planner_update_time", 0.5)
        self.declare_parameter("goal_is_reached_update_time", 0.1)
        self.declare_parameter("detailed_log", False)

        self.get_logger().info("Map path is: " + self.get_parameter("map_path").get_parameter_value().string_value)

        self.init_proj_str()

        self.rnd = None
        self.ego_vehicle = None
        self.ego_vehicle_state: CustomState = None
        # buffer for static obstacles
        self.dynamic_obstacles_ids = {}  # a list to save dynamic obstacles id. key: from cr, value: from autoware
        self.last_trajectory = None
        self.origin_transformation_x, self.origin_transformation_y = None, None
        self.vehicle_length, self.vehicle_width = None, None
        self.scenario = None

        self.planning_problem = None
        self.route_planned = False
        self.planner_state_list = None
        name_file_motion_primitives = 'V_0.0_20.0_Vstep_4.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml'
        self.automaton = ManeuverAutomaton.generate_automaton(name_file_motion_primitives)
        self.write_scenario = self.get_parameter('write_scenario').get_parameter_value().bool_value
        self.is_computing_trajectory = False  # stop update scenario when trajectory is computing
        self.create_ego_vehicle_info()  # compute ego vehicle width and height
        
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames

        self.aw_state = AutowareState()
        self.engage_status = False
        self.reference_path_published = False

        self.new_initial_pose = False
        self.new_pose_received = False

        self.build_scenario()  # build scenario from osm map

        self.convert_origin()
        
        # Define Planner 
        self.planner_type = self.get_parameter("planner_type").get_parameter_value().integer_value
        if self.planner_type == 1:
            self.planner = MotionPlanner.BreadthFirstSearch
        elif self.planner_type == 2:
            self.planner=RP2Interface(self.scenario,
                                      dir_config_default=self.get_parameter("reactive_planner.default_yaml_folder").get_parameter_value().string_value,
                                      d_min = self.get_parameter('reactive_planner.sampling.d_min').get_parameter_value().integer_value,
                                      d_max = self.get_parameter('reactive_planner.sampling.d_max').get_parameter_value().integer_value,
                                      t_min = self.get_parameter('reactive_planner.sampling.t_min').get_parameter_value().double_value,            
                                      dt = self.get_parameter('reactive_planner.planning.dt').get_parameter_value().double_value,
                                      planning_horizon = self.get_parameter('reactive_planner.planning.planning_horizon').get_parameter_value().double_value,                                      
                                      v_length = self.vehicle_length,
                                      v_width = self.vehicle_width,
                                      v_wheelbase = self.vehicle_wheelbase)
        else:
            self.get_logger().warn("Planner type is not correctly specified ... Using Default Planner")
            self.planner_type = 1
            self.planner = MotionPlanner.BreadthFirstSearch

        # Initialize Velocity Planner
        self.velocity_planner = VelocityPlanner(self.get_parameter("detailed_log").get_parameter_value().bool_value, self.get_logger())

        # create callback group for async execution
        self.callback_group = ReentrantCallbackGroup()

        # subscribe current position of vehicle
        self.current_state_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.current_state_callback,
            1,
            callback_group=self.callback_group
        )
        # subscribe static obstacles
        self.static_obs_sub = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/detection/objects',
            self.static_obs_callback,
            1,
            callback_group=self.callback_group
        )
        # subscribe dynamic obstacles
        self.dynamic_obs_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.dynamic_obs_callback,
            1,
            callback_group=self.callback_group
        )
        # subscribe goal pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal',
            self.goal_pose_callback,
            1,
            callback_group=self.callback_group
        )
        # subscribe autoware states
        """self.aw_state_sub = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.state_callback,
            1,
            callback_group=self.callback_group
        )"""

        # subscribe initial pose
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            1,
            callback_group=self.callback_group
        )

        """
        # subscribe velocity planner
        self.velocity_sub = self.create_subscription(
            Path,
            '/planning/scenario_planning/lane_driving/behavior_planning/path',
            self.velocity_planner.path_with_velocity_callback,
            1,
            callback_group=self.callback_group
        )

        # test subscription
        self.velocity_sub_test = self.create_subscription(
            PathWithLaneId,
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.velocity_planner.test_callback,
            1,
            callback_group=self.callback_group
        )
        """

        self.traj_sub_smoothed = self.create_subscription(
            AWTrajectory,
            '/planning/scenario_planning/trajectory_smoothed',
            self.velocity_planner.smoothed_trajectory_callback,
            1,
            callback_group=self.callback_group
        )

        # subscribe autoware engage
        self.engage_sub = self.create_subscription(
            Engage,
            '/api/external/cr2autoware/engage',
            self.engage_callback,
            1,
            callback_group=self.callback_group
        )

        """
        # publish path to velocity planner
        self.velocity_pub = self.create_publisher(
            PathWithLaneId,
            '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            1
        )
        self.velocity_planner.set_publisher(self.velocity_pub)
        """

        # publish trajectory to motion velocity smoother
        self.velocity_pub = self.create_publisher(
            AWTrajectory,
            '/planning/scenario_planning/scenario_selector/trajectory',
            1
        )
        self.velocity_planner.set_publisher(self.velocity_pub)

        # publish trajectory
        self.traj_pub = self.create_publisher(
            AWTrajectory,
            '/planning/scenario_planning/trajectory',
            1
        )

        # publish autoware state
        # list of states: https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_system_msgs/msg/AutowareState.idl
        self.aw_state_pub = self.create_publisher(
            AutowareState,
            '/autoware/state',
            1
        )

        # publish autoware engage
        self.engage_pub = self.create_publisher(
            Engage,
            '/autoware/engage',
            1
        )
        self.vehicle_engage_pub = self.create_publisher(
            Engage,
            '/vehicle/engage',
            1
        )
        self.ext_engage_pub = self.create_publisher(
            Engage,
            '/api/external/get/engage',
            1
        )
        self.api_engage_pub = self.create_publisher(
            Engage,
            '/api/autoware/get/engage',
            1
        )

        # publish route marker
        qos_route_pub = QoSProfile(depth=1)
        qos_route_pub.history = QoSHistoryPolicy.KEEP_LAST
        qos_route_pub.reliability = QoSReliabilityPolicy.RELIABLE
        qos_route_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(
            MarkerArray,
            '/planning/mission_planning/route_marker',
            qos_route_pub
        )

        # vars to save last messages
        self.current_vehicle_state = None
        self.last_msg_static_obs = None
        self.last_msg_dynamic_obs = None
        # https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl
        self.last_msg_aw_state = 1  # 1 = initializing
        self.goal_msgs = []
        self.current_goal_msg = None
        self.last_goal_reached = self.get_clock().now()
        self.reference_path = None

        # create a timer to update scenario
        """self.timer_update_scenario = self.create_timer(
            timer_period_sec=self.get_parameter("scenario_update_time").get_parameter_value().double_value,
            callback=self.update_scenario, callback_group=self.callback_group)"""

        # create a timer to run planner
        self.timer_solve_planning_problem = self.create_timer(
            timer_period_sec=self.get_parameter("planner_update_time").get_parameter_value().double_value,
            callback=self.solve_planning_problem, callback_group=self.callback_group)

        # create a timer to check if goal is reached
        #self.timer_is_goal_reached = self.create_timer(
        #    timer_period_sec=self.get_parameter("goal_is_reached_update_time").get_parameter_value().double_value,
        #    callback=self._is_goal_reached, callback_group=self.callback_group)

        self.set_state(AutowareState.WAITING_FOR_ROUTE)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Detailed log is enabled")
            self.get_logger().info("Map path is: " + self.get_parameter("map_path").get_parameter_value().string_value)
            self.get_logger().info("Init complete!")

    def update_scenario(self):
        """
        Update the commonroad scenario with the latest vehicle state and obstacle messages received.
        """
        # process last state message
        if self.current_vehicle_state is not None:
            self._process_current_state()
        else:
            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("has not received a vehicle state yet!")
            return

        # process last static obstacle message
        self._process_static_obs()

        # process last dynamic obstacle message
        self._process_dynamic_obs()

        # plot scenario if plot_scenario = True
        if self.get_parameter('plot_scenario').get_parameter_value().bool_value:
            self._plot_scenario()

        if self.write_scenario:
            self._write_scenario()

    def solve_planning_problem(self) -> None:
        """
        Solve planning problem with algorithms offered by commonroad.
        """

        try:

            # check if initial pose was changed (if true: recalculate reference path)
            if self.new_initial_pose:

                # update scenario
                self.update_scenario()

                # check if the current_vehicle_state was already updated (=pose received by current state callback), otherwise wait one planning cycle
                if not self.new_pose_received:
                    return
                
                self.new_initial_pose = False
                self.new_pose_received = False

                self.get_logger().info("Replanning route to goal")

                # insert current goal into list of goal messages and set route_planned to false to trigger route planning
                if self.current_goal_msg:
                    self.goal_msgs.insert(0, self.current_goal_msg)
                self.route_planned = False
            else:
                self.update_scenario()

            if not self.route_planned:
                # if currently no active goal, set a new goal (if one exists)
                try:
                    self._set_new_goal()
                except Exception:
                    self.get_logger().error(traceback.format_exc())

            if self.route_planned:

                if not self.velocity_planner.get_is_velocity_planning_completed():
                    self.get_logger().info("Can't run route planner because interface is still waiting for velocity planner")
                    return

                if not self.reference_path_published:
                    # publish current reference path
                    self._pub_route(*self.velocity_planner.get_reference_velocities())

                if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                    self.get_logger().info("Solving planning problem!")

                if not self.is_computing_trajectory:
                    # Compute trajectory
                    self.is_computing_trajectory = True

                    if self.planner_type == 1:  # Breadth First Search
                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info("Running breadth first search")
                        self._run_search_planner()

                    if self.planner_type == 2:  # Reactive Planner

                        #reference_velocity = self.velocity_planner.get_velocity_at_aw_position(self.current_vehicle_state.pose.pose.position)
                        reference_velocity = 5.0

                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info("Running reactive planner")
                            self.get_logger().info("Reactive planner init_state position: " + str(self.ego_vehicle_state.position))
                            self.get_logger().info("Reactive planner init_state velocity: " + str(self.ego_vehicle_state.velocity))
                            self.get_logger().info("Reactive planner reference path velocity: " + str(reference_velocity))
                            self.get_logger().info("Reactive planner reference path length: " + str(len(self.reference_path)))
                            if len(self.reference_path > 1):
                                self.get_logger().info("Reactive planner reference path: " + str(self.reference_path[0]) + "  --->  ["  \
                                    + str(len(self.reference_path)-2) + " states skipped]  --->  " + str(self.reference_path[-1]))

                        self.planner._run_reactive_planner(init_state=self.ego_vehicle_state, goal=self.planning_problem.goal, reference_path=self.reference_path, reference_velocity=reference_velocity)

                        assert(self.planner.optimal != False)
                        assert(self.planner.valid_states != [])
                        assert(max([s.velocity for s in self.planner.valid_states]) > 0)

                        self.get_logger().info("Reactive planner velocities: " + str([s.velocity for s in self.planner.valid_states]))
                        self.get_logger().info("Reactive planner acc: " + str([s.acceleration for s in self.planner.valid_states]))

                        # calculate velocities and accelerations of planner states
                        self._calculate_velocities(self.planner.valid_states, self.ego_vehicle_state.velocity)

                        # publish trajectory
                        self._prepare_traj_msg(self.planner.valid_states)
                        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                            self.get_logger().info("Autoware state and engage messages published!")

                    self.is_computing_trajectory = False
                else:
                    if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                        self.get_logger().info("already computing trajectory")

                # check if goal is reached
                self._is_goal_reached()

        except Exception:
            self.get_logger().error(traceback.format_exc())

    def _is_goal_reached(self):
        """
        Check if vehicle is in goal region. If in goal region set new goal.
        """
        if self.planning_problem:
            
            if self.planning_problem.goal.is_reached(self.ego_vehicle_state) and \
                                                    (self.get_clock().now() - self.last_goal_reached).nanoseconds > 5e8:
                self.get_logger().info("Car arrived at goal!")
                self.last_goal_reached = self.get_clock().now()

                if self.goal_msgs == []:
                    self.route_planned = False
                    self.planning_problem = None

                    self.set_state(AutowareState.ARRIVED_GOAL)

                    # publish empty trajectory
                    self._prepare_traj_msg([])
                    self._pub_route([], [])
                else:
                    # set next goal
                    self._set_new_goal()

    
    def init_proj_str(self):

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        if not os.path.exists(map_path):
            raise ValueError("Can't find given map path: %s" % map_path)

        map_config_tmp = list(glob.iglob(os.path.join(map_path, '*.[yY][aA][mM][lL]')))
        if not map_config_tmp:
            raise ValueError("Couldn't load map origin! No YAML file exists in: %s" % map_path)

        with open(map_config_tmp[0]) as f:
            data = yaml.load(f, Loader=SafeLoader)
        self.aw_origin_latitude = Decimal(data["/**"]["ros__parameters"]["map_origin"]["latitude"])
        self.aw_origin_longitude = Decimal(data["/**"]["ros__parameters"]["map_origin"]["longitude"])

        utm_str = utm.from_latlon(float(self.aw_origin_latitude), float(self.aw_origin_longitude))
        self.proj_str = "+proj=utm +zone=%d +datum=WGS84 +ellps=WGS84" % (utm_str[2])
        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Proj string: %s" % self.proj_str)
    

    def convert_origin(self):
        """
        Compute coordinate of the origin in UTM (used in commonroad) frame.
        """
        proj = Proj(self.proj_str)

        # Get Autoware map origin from osm file
        aw_origin_latitude = self.aw_origin_latitude
        aw_origin_longitude = self.aw_origin_longitude

        aw_origin_x, aw_origin_y = proj(aw_origin_longitude, aw_origin_latitude)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Autoware origin lat: %s,   origin lon: %s" % (aw_origin_latitude, aw_origin_longitude))
            self.get_logger().info("Autoware origin x: %s,   origin y: %s" % (aw_origin_x, aw_origin_y))

        
        # Get CommonRoad map origin
        cr_origin_lat = Decimal(self.scenario.location.gps_latitude)
        cr_origin_lon = Decimal(self.scenario.location.gps_longitude)

        cr_origin_x, cr_origin_y = proj(cr_origin_lon, cr_origin_lat)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("CommonRoad origin lat: %s,   origin lon: %s" % (cr_origin_lat, cr_origin_lon))
            self.get_logger().info("CommonRoad origin x: %s,   origin y: %s" % (cr_origin_x, cr_origin_y))
        
        
        self.origin_transformation_x = aw_origin_x - cr_origin_x
        self.origin_transformation_y = aw_origin_y - cr_origin_y
        self.get_logger().info("origin transformation x: %s,   origin transformation y: %s" % (self.origin_transformation_x, self.origin_transformation_y))


    def create_ego_vehicle_info(self):
        """
        Compute the dimensions of the ego vehicle.
        """
        cg_to_front = self.get_parameter("vehicle.cg_to_front").get_parameter_value().double_value
        cg_to_rear = self.get_parameter("vehicle.cg_to_rear").get_parameter_value().double_value
        width = self.get_parameter("vehicle.width").get_parameter_value().double_value
        front_overhang = self.get_parameter("vehicle.front_overhang").get_parameter_value().double_value
        rear_overhang = self.get_parameter("vehicle.rear_overhang").get_parameter_value().double_value
        self.vehicle_length = front_overhang + cg_to_front + cg_to_rear + rear_overhang
        self.vehicle_width = width
        self.vehicle_wheelbase = cg_to_front + cg_to_rear


    def build_scenario(self):
        """
        Transform map from osm/lanelet2 format to commonroad scenario format.
        """
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        # get osm file in folder map_path
        map_filename = list(glob.iglob(os.path.join(map_path, '*.[oO][sS][mM]')))[0]
        
        left_driving = self.get_parameter('left_driving').get_parameter_value().bool_value
        adjacencies = self.get_parameter('adjacencies').get_parameter_value().bool_value
        self.scenario = lanelet_to_commonroad(map_filename,
                                              proj=self.proj_str,
                                              left_driving=left_driving,
                                              adjacencies=adjacencies)
        if self.write_scenario:
            # save map
            self._write_scenario()


    def current_state_callback(self, msg: Odometry) -> None:
        """
        Callback to current kinematic state of the ego vehicle. Safe the message for later processing.
        :param msg: current kinematic state message
        """
        self.current_vehicle_state = msg
        self.new_pose_received = True

    def _process_current_state(self) -> None:
        """
        Calculate the current commonroad state from the autoware latest state message.
        """
        if self.current_vehicle_state is not None:
            source_frame = self.current_vehicle_state.header.frame_id
            time_step = 0
            # lookup transform
            succeed = self.tf_buffer.can_transform("map",
                                                   source_frame,
                                                   rclpy.time.Time(),
                                                   )
            if not succeed:
                self.get_logger().error(f"Failed to transform from {source_frame} to map frame")
                return None

            if source_frame != "map":
                temp_pose_stamped = PoseStamped()
                temp_pose_stamped.header = self.current_vehicle_state.header
                temp_pose_stamped.pose = self.current_vehicle_state.pose.pose
                pose_transformed = self._transform_pose_to_map(temp_pose_stamped)
                position = self.map2utm(pose_transformed.pose.position)
                orientation = quaternion2orientation(pose_transformed.pose.orientation)
            else:
                position = self.map2utm(self.current_vehicle_state.pose.pose.position)
                orientation = quaternion2orientation(self.current_vehicle_state.pose.pose.orientation)
            #steering_angle=  arctan2(wheelbase * yaw_rate, velocity)
            steering_angle=np.arctan2(self.vehicle_wheelbase * self.current_vehicle_state.twist.twist.angular.z, self.current_vehicle_state.twist.twist.linear.x)

            self.ego_vehicle_state = CustomState(position=position,
                                           orientation=orientation,
                                           velocity=self.current_vehicle_state.twist.twist.linear.x,
                                           yaw_rate=self.current_vehicle_state.twist.twist.angular.z,
                                           slip_angle=0.0,
                                           time_step=time_step,
                                           steering_angle = steering_angle)

    def static_obs_callback(self, msg: DetectedObjects) -> None:
        """
        Callback to static obstacles. Safe the message for later processing.
        :param msg: static obstacles message
        """
        self.last_msg_static_obs = msg

    def _process_static_obs(self) -> None:
        """
        Convert static autoware obstacles to commonroad obstacles and add them to the scenario.
        """
        if self.last_msg_static_obs is not None:
            # ToDo: remove the dynamic obstacles from the static list

            temp_pose = PoseStamped()
            temp_pose.header = self.last_msg_static_obs.header
            self.scenario.remove_obstacle(self.scenario.static_obstacles)

            for box in self.last_msg_static_obs.objects:
                # ToDo: CommonRoad can also consider uncertain states of obstacles, which we could derive from the covariances
                temp_pose.pose.position.x = box.kinematics.pose_with_covariance.pose.position.x
                temp_pose.pose.position.y = box.kinematics.pose_with_covariance.pose.position.y
                temp_pose.pose.position.z = box.kinematics.pose_with_covariance.pose.position.z
                temp_pose.pose.orientation.x = box.kinematics.pose_with_covariance.pose.orientation.x
                temp_pose.pose.orientation.y = box.kinematics.pose_with_covariance.pose.orientation.y
                temp_pose.pose.orientation.z = box.kinematics.pose_with_covariance.pose.orientation.z
                temp_pose.pose.orientation.w = box.kinematics.pose_with_covariance.pose.orientation.w
                pose_map = self._transform_pose_to_map(temp_pose)
                if pose_map is None:
                    continue

                pos = self.map2utm(pose_map.pose.position)
                orientation = quaternion2orientation(pose_map.pose.orientation)
                width = box.shape.dimensions.y
                length = box.shape.dimensions.x

                obs_state = CustomState(position=pos,
                                  orientation=orientation,
                                  time_step=0)

                obs_id = self.scenario.generate_object_id()
                # ToDo: get object type from autoware --> see https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_perception_msgs/msg/ObjectClassification.idl
                obs_type = ObstacleType.UNKNOWN
                obs_shape = Rectangle(width=width, length=length)

                self.scenario.add_objects(StaticObstacle(obs_id, obs_type, obs_shape, obs_state))

    def dynamic_obs_callback(self, msg: PredictedObjects) -> None:
        """
        Callback to dynamic obstacles. Safe the message for later processing.
        :param msg: dynamic obstacles message
        """
        self.last_msg_dynamic_obs = msg

    def traj_linear_interpolate(self, point_1: Pose, point_2: Pose, smaller_dt: float, bigger_dt: float) -> Pose:
        """
        interpolation for a point between two points
        :param point_1: point which will be smaller than interpolated point (on left-side)
        :param point_1: point which will be bigger than interpolated point (on right-side)
        :param smaller_dt: time step for the point will be interpolated
        :param bigger_dt: time step for the points which will be used for interpolation
        :return: pose of the interpolated point
        """
        new_point = Pose()
        new_point.position.x = point_1.position.x + \
                               ((point_2.position.x - point_1.position.x) / smaller_dt) * \
                               (bigger_dt - smaller_dt)
        new_point.position.y = point_1.position.y + \
                               ((point_2.position.y - point_1.position.y) / smaller_dt) * \
                               (bigger_dt - smaller_dt)
        new_point.position.z = point_1.position.z + \
                               ((point_2.position.z - point_1.position.z) / smaller_dt) * \
                               (bigger_dt - smaller_dt)
        new_point.orientation.x = point_1.orientation.x + \
                                  ((point_2.orientation.x - point_1.orientation.x) / smaller_dt) * \
                                  (bigger_dt - smaller_dt)
        new_point.orientation.y = point_1.orientation.y + \
                                  ((point_2.orientation.y - point_1.orientation.y) / smaller_dt) * \
                                  (bigger_dt - smaller_dt)
        new_point.orientation.z = point_1.orientation.z + \
                                  ((point_2.orientation.z - point_1.orientation.z) / smaller_dt) * \
                                  (bigger_dt - smaller_dt)
        new_point.orientation.w = point_1.orientation.w + \
                                  ((point_2.orientation.w - point_1.orientation.w) / smaller_dt) * \
                                  (bigger_dt - smaller_dt)
        return new_point

    def _process_dynamic_obs(self) -> None:
        """
        Convert dynamic autoware obstacles to commonroad obstacles and add them to the scenario.
        """
        if self.last_msg_dynamic_obs is not None:
            for object in self.last_msg_dynamic_obs.objects:
                position = self.map2utm(object.kinematics.initial_pose_with_covariance.pose.position)
                orientation = quaternion2orientation(
                    object.kinematics.initial_pose_with_covariance.pose.orientation)
                velocity = object.kinematics.initial_twist_with_covariance.twist.linear.x
                yaw_rate = object.kinematics.initial_twist_with_covariance.twist.angular.z
                width = object.shape.dimensions.y
                length = object.shape.dimensions.x
                time_step = 0
                traj = []
                highest_conf_val = 0
                highest_conf_idx = 0
                for i in range(len(object.kinematics.predicted_paths)):
                    conf_val = object.kinematics.predicted_paths[i].confidence
                    if conf_val > highest_conf_val:
                        highest_conf_val = conf_val
                        highest_conf_idx = i

                obj_traj_dt = object.kinematics.predicted_paths[highest_conf_idx].time_step.nanosec

                planning_dt = self.get_parameter('reactive_planner.planning.dt').get_parameter_value().double_value
                if obj_traj_dt > planning_dt * 1e9:
                    # upsample predicted path of obstacles to match dt
                    if obj_traj_dt % (planning_dt * 1e9) == 0.0:
                        dt_ratio = int(obj_traj_dt / (planning_dt * 1e9)) + 1
                    else:
                        dt_ratio = math.ceil(obj_traj_dt / (planning_dt * 1e9))

                    for point in object.kinematics.predicted_paths[highest_conf_idx].path:
                        traj.append(point)
                        if len(traj) >= 2:
                            point_2 = traj[-1]
                            point_1 = traj[-2]
                            new_points_x = np.linspace(point_1.position.x, point_2.position.x, dt_ratio)
                            new_points_y = np.linspace(point_1.position.y, point_2.position.y, dt_ratio)
                            new_points_z = np.linspace(point_1.position.z, point_2.position.z, dt_ratio)
                            new_points_ort_x = np.linspace(point_1.orientation.x, point_2.orientation.x, dt_ratio)
                            new_points_ort_y = np.linspace(point_1.orientation.y, point_2.orientation.y, dt_ratio)
                            new_points_ort_z = np.linspace(point_1.orientation.z, point_2.orientation.z, dt_ratio)
                            new_points_ort_w = np.linspace(point_1.orientation.w, point_2.orientation.w, dt_ratio)
                            for i in range(1, dt_ratio - 1):  # don't take first and last samples, they were already appended
                                new_point_pos = Point()
                                new_point_pos.x = new_points_x[i]
                                new_point_pos.y = new_points_y[i]
                                new_point_pos.z = new_points_z[i]
                                new_point_ort = Quaternion()
                                new_point_ort.x = new_points_ort_x[i]
                                new_point_ort.y = new_points_ort_y[i]
                                new_point_ort.z = new_points_ort_z[i]
                                new_point_ort.w = new_points_ort_w[i]
                                new_traj_point = Pose()
                                new_traj_point.position = new_point_pos
                                new_traj_point.orientation = new_point_ort
                                traj.insert(-1, new_traj_point)  # upsampled trajectory list

                elif obj_traj_dt < planning_dt * 1e9:
                    # downsample predicted path of obstacles to match dt.
                    # if the time steps are divisible without reminder,
                    # get the trajectories at the steps according to ratio
                    if (planning_dt * 1e9) % obj_traj_dt == 0.0:
                        dt_ratio = (planning_dt * 1e9) / obj_traj_dt
                        for idx, point in enumerate(object.kinematics.predicted_paths[highest_conf_idx].path):
                            if (idx + 1) % dt_ratio == 0:
                                traj.append(point)
                    else:
                        # make interpolation according to time steps
                        dt_ratio = math.ceil((planning_dt * 1e9) / obj_traj_dt)
                        for idx, point in enumerate(object.kinematics.predicted_paths[highest_conf_idx].path):
                            if (idx + 1) % dt_ratio == 0:
                                point_1 = object.kinematics.predicted_paths[highest_conf_idx].path[idx - 1]
                                point_2 = point
                                new_point = self.traj_linear_interpolate(point_1, point_2, obj_traj_dt, planning_dt * 1e9)
                                traj.append(new_point)
                else:
                    for point in object.kinematics.predicted_paths[highest_conf_idx].path:
                        traj.append(point)

                object_id_aw = object.object_id.uuid
                aw_id_list = [list(value) for value in self.dynamic_obstacles_ids.values()]
                if list(object_id_aw) not in aw_id_list:
                    dynamic_obstacle_initial_state = CustomState(position=position,
                                                           orientation=orientation,
                                                           velocity=velocity,
                                                           yaw_rate=yaw_rate,
                                                           time_step=time_step)
                    object_id_cr = self.scenario.generate_object_id()
                    self.dynamic_obstacles_ids[object_id_cr] = object_id_aw
                    self._add_dynamic_obstacle(dynamic_obstacle_initial_state, traj, width, length, object_id_cr,
                                               time_step)
                else:
                    for key, value in self.dynamic_obstacles_ids.items():
                        if np.array_equal(object_id_aw, value):
                            dynamic_obs = self.scenario.obstacle_by_id(key)
                            if dynamic_obs:
                                dynamic_obs.initial_state = CustomState(position=position,
                                                                  orientation=orientation,
                                                                  velocity=velocity,
                                                                  yaw_rate=yaw_rate,
                                                                  time_step=time_step)
                                dynamic_obs.obstacle_shape = Rectangle(width=width, length=length)
                                if len(traj) > 2:
                                    dynamic_obs.prediction = TrajectoryPrediction(
                                        self._awtrajectory_to_crtrajectory(2, dynamic_obs.initial_state.time_step,
                                                                           traj),
                                        dynamic_obs.obstacle_shape)

    def _add_dynamic_obstacle(self, initial_state, traj, width, length, object_id, time_step) -> None:
        """
        Add dynamic obstacles with their trajectory
        :param initial_state: initial state of obstacle
        :param traj: trajectory of obstacle
        :param width: width of obstacle
        :param length: length of obstacle
        :param object_id: id of obstacle
        :param time_step: time step of the obstacle
        """
        dynamic_obstacle_shape = Rectangle(width=width, length=length)
        # ToDo: get object type from autoware
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle_id = object_id
        if len(traj) > 2:
            # create the trajectory of the obstacle, starting at time_step
            dynamic_obstacle_trajectory = self._awtrajectory_to_crtrajectory(2, time_step, traj)

            # create the prediction using the trajectory and the shape of the obstacle
            dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

            dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                               dynamic_obstacle_type,
                                               dynamic_obstacle_shape,
                                               initial_state,
                                               dynamic_obstacle_prediction)
        else:
            dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                               dynamic_obstacle_type,
                                               dynamic_obstacle_shape,
                                               initial_state)
        # add dynamic obstacle to the scenario
        self.scenario.add_objects(dynamic_obstacle)

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
            for TrajectoryPoint in traj:
                work_traj.append(TrajectoryPoint.pose)
        else:
            work_traj = traj
        for i in range(len(work_traj)):
            # compute new position
            position = self.map2utm(work_traj[i].position)
            orientation = quaternion2orientation(work_traj[i].orientation)
            # create new state
            new_state = CustomState(position=position, orientation=orientation, time_step=i)
            # add new state to state_list
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
        # save msg to list of goals
        self.goal_msgs.append(msg)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("goal msg: " + str(msg))

        self._pub_goals()

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

            position = self.map2utm(current_msg.pose.position)
            pos_x = position[0]
            pos_y = position[1]
            self.get_logger().info("Pose position utm: " + str(position))
            orientation = quaternion2orientation(current_msg.pose.orientation)
            if self.ego_vehicle_state is None:
                self.get_logger().error("ego vehicle state is None")
                return

            max_vel = self.get_parameter('vehicle.max_velocity').get_parameter_value().double_value
            min_vel = self.get_parameter('vehicle.min_velocity').get_parameter_value().double_value
            velocity_interval = Interval(min_vel, max_vel)

            # get goal lanelet and its width
            # subtract commonroad map origin
            goal_lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position([np.array([pos_x, pos_y])])

            if self.get_parameter("detailed_log").get_parameter_value().bool_value:
                self.get_logger().info("goal pos_x: " + str(pos_x) + ", pos_y: " + str(pos_y))

            if goal_lanelet_id == [[]]:
                self.get_logger().error("No lanelet found at goal position!")
                return

            if goal_lanelet_id:
                goal_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(goal_lanelet_id[0][0])
                left_vertices = goal_lanelet.left_vertices
                right_vertices = goal_lanelet.right_vertices
                goal_lanelet_width = np.linalg.norm(left_vertices[0] - right_vertices[0])
            else:
                goal_lanelet_width = 3.0

            region = Rectangle(length=self.vehicle_length + 0.25 * self.vehicle_length, width=goal_lanelet_width,
                               center=position, orientation=orientation)
            goal_state = CustomState(position=region, time_step=Interval(0, 1000), velocity=velocity_interval)

            goal_region = GoalRegion([goal_state])
            self.planning_problem = PlanningProblem(planning_problem_id=1,
                                                    initial_state=self.ego_vehicle_state,
                                                    goal_region=goal_region)
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

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Engage message received! Engage: " + str(self.engage_status) + ", current state: " + str(self.get_state()))

        # Update Autoware state panel
        if self.engage_status and self.get_state() == AutowareState.WAITING_FOR_ENGAGE:
            self.set_state(AutowareState.DRIVING)
        if not self.engage_status and self.get_state() == AutowareState.DRIVING:
            self.set_state(AutowareState.WAITING_FOR_ENGAGE)

    def _calculate_velocities(self, states, init_velocity):

        if states == []:
            return

        cur_vel = max(0.1, init_velocity)
        # target_vel = 6

        max_acceleration = 1 # max acceleration
        # time interval of planning
        dt = self.scenario.dt

        for i, state in enumerate(states):
            old_vel = cur_vel
            target_vel = max(2, self.velocity_planner.get_velocity_at_aw_position(self.utm2map(state.position)))
            if cur_vel < target_vel:
                cur_vel = min(cur_vel + max_acceleration * dt, target_vel)
            else:
                cur_vel = target_vel
            state.velocity = cur_vel

            if i < len(states) - 1:
                state.acceleration = (cur_vel - old_vel) / self.scenario.dt
            else:
                state.acceleration = 0.0


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
            self.get_logger().info('New empty trajectory published !!!')
            return

        position_list = []

        for i in range(0, len(states)):
            new_point = TrajectoryPoint()
            # time_from_start not given by autoware planner
            # t = states[i].time_step * self.scenario.dt
            # nano_sec, sec = math.modf(t)
            # new_point.time_from_start = Duration(sec=int(sec), nanosec=int(nano_sec * 1e9))
            new_point.pose.position = self.utm2map(states[i].position)
            position_list.append([states[i].position[0], states[i].position[1]])
            new_point.pose.orientation = orientation2quaternion(states[i].orientation)
            new_point.longitudinal_velocity_mps = float(states[i].velocity)
            # self.get_logger().info(str(states[i].velocity))

            # front_wheel_angle_rad not given by autoware planner
            # new_point.front_wheel_angle_rad = states[i].steering_angle
            new_point.acceleration_mps2 = float(states[i].acceleration)

            self.traj.points.append(new_point)

        self.traj_pub.publish(self.traj)
        self.get_logger().info('New trajectory published !!!')

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
        self.reference_path = route_planner.plan_routes().retrieve_first_route().reference_path
        self.velocity_planner.send_reference_path([self.utm2map(point) for point in self.reference_path], self.current_goal_msg.pose.position)

        if self.get_parameter("detailed_log").get_parameter_value().bool_value:
            self.get_logger().info("Route planning completed!")

    def _run_search_planner(self):
        """
        Run one cycle of search based planner.
        """
        # construct motion planner
        planner = self.planner(scenario=self.scenario,
                               planning_problem=self.planning_problem,
                               automaton=self.automaton)
        # visualize searching process
        # scenario_data = (self.scenario, planner.state_initial, planner.shape_ego, self.planning_problem)
        # display_steps(scenario_data, planner.execute_search, planner.config_plot)

        path, _, _ = planner.execute_search()

        if path is not None:
            valid_states = []
            # there are duplicated points, which will arise "same point" exception in AutowareAuto
            for states in path:
                for state in states:
                    if len(valid_states) > 0:
                        last_state = valid_states[-1]
                        if last_state.time_step == state.time_step:
                            continue
                    valid_states.append(state)

            self._prepare_traj_msg(valid_states)
            # self._set_new_goal()
            # visualize_solution(self.scenario, self.planning_problem, create_trajectory_from_list_states(path)) #ToDo: check if working
        else:
            self.get_logger().error("Failed to solve the planning problem.")

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
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = 1
            marker.ns = "goals"
            marker.frame_locked = True
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = self.current_goal_msg.pose.position.x
            marker.pose.position.y = self.current_goal_msg.pose.position.y
            marker.pose.position.z = self.current_goal_msg.pose.position.z
            goals_msg.markers.append(marker)

        if len(self.goal_msgs) > 0:
            for i in range(len(self.goal_msgs)):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = i + 2
                marker.ns = "goals"
                marker.frame_locked = True
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.position.x = self.goal_msgs[i].pose.position.x
                marker.pose.position.y = self.goal_msgs[i].pose.position.y
                marker.pose.position.z = self.goal_msgs[i].pose.position.z
                goals_msg.markers.append(marker)

        self.route_pub.publish(goals_msg)

    def _pub_route(self, path, velocities):
        """
        Publish planned route as marker to visualize in RVIZ.
        """

        self.reference_path_published = True

        route = Marker()
        route.header.frame_id = "map"
        route.id = 1
        route.ns = "route"
        route.frame_locked = True
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.scale.x = 0.1
        route.scale.y = 0.1
        route.scale.z = 0.1
        route.color.r = 0.0
        route.color.g = 0.0
        route.color.b = 1.0
        route.color.a = 0.3

        if path != []:
            max_velocity = max(velocities)
            if max_velocity < 0.1:
                max_velocity = 0.1

        for i in range(0, len(path)):

            point = path[i]
            if i < len(velocities):
                vel = velocities[i]
            else:
                # change config parameters of velocity smoother if whole path not calculated
                vel = 0

            #p = self.utm2map(point)
            #p.z = 0
            p = point
            route.points.append(p)

            c = ColorRGBA()
            c.r = 1.0 * vel / max_velocity
            c.g = 0.0
            c.b = 1.0 - 1.0 * vel/ max_velocity
            c.a = 1.0
            route.colors.append(c)

        route_msg = MarkerArray()
        route_msg.markers.append(route)
        self.route_pub.publish(route_msg)

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
        self.ego_vehicle = self._create_ego_with_cur_location()
        #self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.facecolor = "#ff0000"
        #self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.edgecolor = "#000000"
        #self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.zorder = 50
        #self.rnd.draw_params.static_obstacle.occupancy.shape.rectangle.opacity = 1
        """self.ego_vehicle.draw(self.rnd, draw_params={ # outdate cr-io
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
        #self.rnd.draw_params["static_obstacle"]["occupancy"]["shape"]["rectangle"]["facecolor"] = "#ff0000"
        self.ego_vehicle.draw(self.rnd)

        #self.scenario.draw(self.rnd, draw_params={'lanelet': {"show_label": False}}) # outdated cr-io version
        #self.rnd.draw_params.lanelet.show_label = False
        self.scenario.draw(self.rnd)
        # self.planning_problem.draw(self.rnd) #ToDo: check if working
        self.rnd.render()
        plt.pause(0.1)

    def _create_ego_with_cur_location(self):
        """
        Create a new ego vehicle with current position for visualization.
        """
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        if self.last_trajectory is None:
            return DynamicObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state)
        else:
            pred_traj = TrajectoryPrediction(self._awtrajectory_to_crtrajectory(1, self.ego_vehicle_state.time_step,
                                                                                self.last_trajectory.points),
                                             ego_vehicle_shape)
            return DynamicObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state,
                                   prediction=pred_traj)

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
        os.makedirs('output', exist_ok=True)
        writer.write_to_file(os.path.join('output', "".join([str(self.scenario.scenario_id),".xml"])), OverwriteExistingFile.ALWAYS)

    def _transform_pose_to_map(self, pose_in):
        """
        Transform pose to pose in map frame.
        :param pose_in: pose in other frame
        :return: pose in map frame
        """
        source_frame = pose_in.header.frame_id
        # lookup transform validity
        succeed = self.tf_buffer.can_transform("map",
                                               source_frame,
                                               rclpy.time.Time(),
                                               )
        if not succeed:
            self.get_logger().error(f"Failed to transform from {source_frame} to map frame")
            return None

        try:
            tf_map = self.tf_buffer.lookup_transform("map", source_frame,
                                                     rclpy.time.Time.from_msg(pose_in.header.stamp))
        except tf2_ros.ExtrapolationException:
            tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time())

        pose_out = do_transform_pose(pose_in, tf_map)
        return pose_out

    def map2utm(self, p: Point) -> np.array:
        """
        Transform position (in autoware) to position (in commonroad).
        :param p: position autoware
        :return: position commonroad
        """
        _x = self.origin_transformation_x + p.x
        _y = self.origin_transformation_y + p.y
        return np.array([_x, _y])

    def utm2map(self, position: np.array) -> Point:
        """
        Transform position (in commonroad) to position (in autoware).
        :param position: position commonroad
        :return: position autoware
        """
        p = Point()
        p.x = position[0] - self.origin_transformation_x
        p.y = position[1] - self.origin_transformation_y
        return p


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


if __name__ == '__main__':
    main()
