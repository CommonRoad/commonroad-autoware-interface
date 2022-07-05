import os
import sys
from threading import Thread
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Duration
import math
import numpy as np
from pyproj import Proj
import matplotlib.pyplot as plt
# import necessary classes from different modules
from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.planning.goal import GoalRegion
from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory as CRTrajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer

sys.path.append("/root/workspace/commonroad-search")
from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton

from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.configuration import build_configuration
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.utility.evaluation import create_planning_problem_solution, reconstruct_inputs, reconstruct_states
from copy import deepcopy

#from crdesigner.input_output.api import lanelet_to_commonroad
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from autoware_auto_perception_msgs.msg import DetectedObjects, PredictedObjects
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory
from autoware_auto_system_msgs.msg import AutowareState
from autoware_auto_vehicle_msgs.msg import Engage

from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cr2autoware.tf2_geometry_msgs import do_transform_pose
from cr2autoware.utils import visualize_solution, display_steps


@dataclass
class Box:
    x: float
    y: float
    width: float
    length: float
    orientation: float


class Cr2Auto(Node):
    def __init__(self):
        super().__init__('cr2autoware')

        # declare ros parameter
        self.declare_parameter('vehicle.max_velocity', 5.0)
        self.declare_parameter('vehicle.min_velocity', 1.0)
        self.declare_parameter("planner_type", 1)
        self.declare_parameter("latitude", 0.0)
        self.declare_parameter("longitude", 0.0)
        self.declare_parameter("elevation", 0.0)
        self.declare_parameter("origin_offset_lat", 0.0)
        self.declare_parameter("origin_offset_lon", 0.0)
        self.declare_parameter('vehicle.cg_to_front_m', 1.0)
        self.declare_parameter('vehicle.cg_to_rear_m', 1.0)
        self.declare_parameter('vehicle.width_m', 2.0)
        self.declare_parameter('vehicle.front_overhang_m', 0.5)
        self.declare_parameter('vehicle.rear_overhang_m', 0.5)
        self.declare_parameter('map_osm_file', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)
        self.declare_parameter('proj_str', '')
        self.declare_parameter('reactive_planner.default_yaml_folder', '')
        self.declare_parameter('reactive_planner.sampling.d_min', -3)
        self.declare_parameter('reactive_planner.sampling.d_max', 3)
        self.declare_parameter('reactive_planner.sampling.t_min', 0.4)
        self.declare_parameter('reactive_planner.planning.dt', 0.1)
        self.declare_parameter('reactive_planner.planning.planning_horizon', 0.4)
        self.declare_parameter('reactive_planner.planning.replanning_frequency', 3)
        self.declare_parameter("write_scenario", False)

        self.proj_str = self.get_parameter('proj_str').get_parameter_value().string_value

        self.ego_vehicle = None
        self.ego_vehicle_state: State = None
        # buffer for static obstacles
        self.static_obstacles = []  # a list save static obstacles from at the latest time
        self.dynamic_obstacles = []  # a list save dynamic obstacles from at the latest time
        self.last_trajectory = None

        self.planning_problem = None
        # load the xml with stores the motion primitives
        name_file_motion_primitives = 'V_0.0_20.0_Vstep_4.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml'
        self.automaton = ManeuverAutomaton.generate_automaton(name_file_motion_primitives)
        self.is_computing_trajectory = False  # stop update scenario when trajectory is computing

        self.planner_type = self.get_parameter("planner_type").get_parameter_value().integer_value
        if self.planner_type == 1:
            self.planner = MotionPlanner.BreadthFirstSearch
        elif self.planner_type == 2:
            self.planner = ReactivePlanner
            self.config = build_configuration(dir_default_config=self.get_parameter("reactive_planner.default_yaml_folder").get_parameter_value().string_value)
        else:
            self.get_logger().warn("Planner type is not correctly specified ... Using Default Planner")
            self.planner_type = 1
            self.planner = MotionPlanner.BreadthFirstSearch

        # https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_system_msgs/msg/AutowareState.idl
        self._aw_state = 1  # 1 = initializing
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames

        self.convert_origin()
        self.ego_vehicle_info()  # compute ego vehicle width and height
        self.build_scenario()  # build scenario from osm map

        #self.callback_group = ReentrantCallbackGroup()

        # subscribe current position of vehicle
        self.current_state_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.current_state_callback,
            10)#,
            #callback_group=self.callback_group
        #)
        # subscribe static obstacles
        self.static_obs_sub = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/detection/objects',
            self.static_obs_callback,
            10
        )
        # subscribe dynamic obstacles
        self.static_obs_sub = self.create_subscription(
            PredictedObjects,
            '/perception/object_recognition/objects',
            self.dynamic_obs_callback,
            10
        )
        # subscribe goal pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/planning/mission_planning/goal',
            self.goal_pose_callback,
            10)#,
            #callback_group=self.callback_group
        #)
        # subscribe autoware states
        self.aw_state_sub = self.create_subscription(
            AutowareState,
            '/autoware/state',
            self.state_callback,
            10
        )
        # publish trajectory
        self.traj_pub = self.create_publisher(
            AWTrajectory,
            '/planning/scenario_planning/trajectory',
            10
        )
        # publish route marker
        qos_route_pub = QoSProfile(depth=5)
        qos_route_pub.history = QoSHistoryPolicy.KEEP_LAST
        qos_route_pub.reliability = QoSReliabilityPolicy.RELIABLE
        qos_route_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.route_pub = self.create_publisher(
            MarkerArray,
            '/planning/mission_planning/route_marker',
            qos_route_pub
        )
        # publish autoware engage
        self.engage_pub = self.create_publisher(
            Engage,
            '/autoware/engage',
            10
        )
        # create a timer to update scenario
        self.rnd = MPRenderer()
        plt.ion()
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.update_scenario)

    def convert_origin(self):
        """
        compute coordinate of the origin in UTM (used in commonroad) frame
        :return:
        """
        origin_latitude = self.get_parameter("latitude").get_parameter_value().double_value
        origin_longitude = self.get_parameter("longitude").get_parameter_value().double_value
        origin_elevation = self.get_parameter("elevation").get_parameter_value().double_value
        origin_offset_lat = self.get_parameter("origin_offset_lat").get_parameter_value().double_value
        origin_offset_lon = self.get_parameter("origin_offset_lon").get_parameter_value().double_value

        origin_latitude = origin_latitude + origin_offset_lat
        origin_longitude = origin_longitude + origin_offset_lon
        self.get_logger().info("origin lat: %s,   origin lon: %s" % (origin_latitude, origin_longitude))

        proj = Proj(self.proj_str)
        self.origin_x, self.origin_y = proj(origin_longitude, origin_latitude)
        self.get_logger().info("origin x: %s,   origin  y: %s" % (self.origin_x, self.origin_y))

    def ego_vehicle_info(self):
        """
        compute size of ego vehicle: (length, width)
        :return:
        """
        cg_to_front = self.get_parameter("vehicle.cg_to_front_m").get_parameter_value().double_value
        cg_to_rear = self.get_parameter("vehicle.cg_to_rear_m").get_parameter_value().double_value
        width = self.get_parameter("vehicle.width_m").get_parameter_value().double_value
        front_overhang = self.get_parameter("vehicle.front_overhang_m").get_parameter_value().double_value
        rear_overhang = self.get_parameter("vehicle.rear_overhang_m").get_parameter_value().double_value
        self.vehicle_length = front_overhang + cg_to_front + cg_to_rear + rear_overhang
        self.vehicle_width = width

    def build_scenario(self):
        """
        transform map from osm format to commonroad scenario
        :return:
        """
        map_filename = self.get_parameter('map_osm_file').get_parameter_value().string_value
        left_driving = self.get_parameter('left_driving').get_parameter_value().bool_value
        adjacencies = self.get_parameter('adjacencies').get_parameter_value().bool_value
        self.scenario = lanelet_to_commonroad(map_filename,
                                              proj=self.proj_str,
                                              left_driving=left_driving,
                                              adjacencies=adjacencies)
        # save map
        self.write_scenario()

    def current_state_callback(self, msg: Odometry) -> None:
        """
        position: (state.x, state.y)
        velocity: state.longitudinal_velocity_mps
        orientation:
        yaw_rate:   state.heading_rate_rps
        slip_angle: 0
        time_step: Interval()
        """
        source_frame = msg.header.frame_id
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
            temp_pose_stamped.header = msg.header
            temp_pose_stamped.pose = msg.pose.pose
            pose_transformed = self._transform_pose_to_map(temp_pose_stamped)
            position = self.map2utm(pose_transformed.pose.position)
            orientation = Cr2Auto.quaternion2orientation(pose_transformed.pose.orientation)
        else:
            position = self.map2utm(msg.pose.pose.position)
            orientation = Cr2Auto.quaternion2orientation(msg.pose.pose.orientation)
        self.ego_vehicle_state = State(position=position,
                                       orientation=orientation,
                                       velocity=msg.twist.twist.linear.x,
                                       yaw_rate=msg.twist.twist.angular.z,
                                       slip_angle=0.0,
                                       time_step=0)
        #self.get_logger().info("state update")

    def static_obs_callback(self, msg: DetectedObjects) -> None:
        """
        Callback to static obstacles, which are transformed and add to scenario
        :param msg:
        """
        # ToDo: remove the dynamic obstacles from the static list
        # clear the obstacles from past
        self.static_obstacles.clear()

        temp_pose = PoseStamped()
        temp_pose.header = msg.header
        for box in msg.objects:
            # ToDo: COmmonRoad can also consider uncertain states of obstacles, which we could derive from the covariances
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

            x = pose_map.pose.position.x + self.origin_x
            y = pose_map.pose.position.y + self.origin_y
            orientation = Cr2Auto.quaternion2orientation(pose_map.pose.orientation)
            width = box.shape.dimensions.y
            length = box.shape.dimensions.x

            self.static_obstacles.append(Box(x, y, width, length, orientation))

    def dynamic_obs_callback(self, msg: PredictedObjects) -> None:
        """
        Callback to dynamic obstacles, which are transformed and add to scenario
        :param msg:
        """
        self.dynamic_obstacles = []
        for object in msg.objects:
            width = object.shape.dimensions.y
            length = object.shape.dimensions.x
            position = self.map2utm(object.kinematics.initial_pose_with_covariance.pose.position)
            orientation = Cr2Auto.quaternion2orientation(
                object.kinematics.initial_pose_with_covariance.pose.orientation)
            dynamic_obstacle_initial_state = State(position=position,
                                                   orientation=orientation,
                                                   velocity=object.kinematics.initial_twist_with_covariance.twist.linear.x,
                                                   yaw_rate=object.kinematics.initial_twist_with_covariance.twist.angular.z,
                                                   time_step=0)
            traj = []
            highest_conf_val = 0
            highest_conf_idx = 0
            for i in range(len(object.kinematics.predicted_paths)):
                conf_val = object.kinematics.predicted_paths[i].confidence
                if conf_val > highest_conf_val:
                    highest_conf_val = conf_val
                    highest_conf_idx = i

            for point in object.kinematics.predicted_paths[highest_conf_idx].path:
                traj.append(point)

            # ToDo: sync timesteps of autoware and commonroad
            self.add_dynamic_obstacle(dynamic_obstacle_initial_state, traj, width, length)

    def add_dynamic_obstacle(self, initial_state, traj, width, length) -> None:
        """
        Add dynamic obstacles with their trajectory
        :param initial_state: initial state of obstacle
        :param traj: trajectory of obstacle
        :param width: width of obstacle
        :param length: length of obstacle
        """
        dynamic_obstacle_shape = Rectangle(width=width, length=length)
        dynamic_obstacle_id = self.scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR

        if len(traj) > 2:
            # create the trajectory of the obstacle, starting at time step 1
            dynamic_obstacle_trajectory = self._awtrajectory_to_crtrajectory(2, 1, traj)

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
        self.dynamic_obstacles.append(dynamic_obstacle)

    def _awtrajectory_to_crtrajectory(self, mode, timestep, traj):
        """
        Add dynamic obstacles with their trajectories
        :param mode: 1=TrajectoryPoint mode, 2=pose mode
        :param timestep: timestep from which to start
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
        for i in range(timestep, len(work_traj)):
            # compute new position
            position = self.map2utm(work_traj[i].position)
            orientation = Cr2Auto.quaternion2orientation(work_traj[i].orientation)
            # create new state
            new_state = State(position=position, orientation=orientation, time_step=i)
            # add new state to state_list
            state_list.append(new_state)

        return CRTrajectory(timestep, state_list)

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback to goal pose. Create goal region with given goal pose and planning problem.
        :param msg: Goal Pose
        """
        self.get_logger().info("Received Goal Pose ...")
        position = self.map2utm(msg.pose.position)
        orientation = Cr2Auto.quaternion2orientation(msg.pose.orientation)
        if self.ego_vehicle_state is None:
            self.get_logger().error("ego vehicle state is None")
            return

        max_vel = self.get_parameter('vehicle.max_velocity').get_parameter_value().double_value
        min_vel = self.get_parameter('vehicle.min_velocity').get_parameter_value().double_value
        velocity_interval = Interval(min_vel, max_vel)

        pos_x = round(position[0])
        pos_y = round(position[1])
        goal_lanelet_id = self.scenario.lanelet_network.find_lanelet_by_position([np.array([pos_x, pos_y])])
        if goal_lanelet_id:
            goal_lanelet = self.scenario.lanelet_network.find_lanelet_by_id(goal_lanelet_id[0][0])
            left_vertices = goal_lanelet.left_vertices
            right_vertices = goal_lanelet.right_vertices
            goal_lanelet_width = np.linalg.norm(left_vertices[0] - right_vertices[0])
        else:
            goal_lanelet_width = 3.0

        region = Rectangle(length=self.vehicle_length + 0.25 * self.vehicle_length, width=goal_lanelet_width, center=position, orientation=orientation)
        goal_state = State(position=region, time_step=Interval(0, 1000), velocity=velocity_interval)

        goal_region = GoalRegion([goal_state])
        self.planning_problem = PlanningProblem(planning_problem_id=1,
                                                initial_state=self.ego_vehicle_state,
                                                goal_region=goal_region)
        self._solve_planning_problem(msg)

    def state_callback(self, msg: AutowareState) -> None:
        self._aw_state = msg.state

    def _solve_planning_problem(self, msg: PoseStamped) -> None:
        """
        Solve planning problem with algorithms offered by commonroad. Now BreadthFirstSearch is in use, but can
        explore other algorithms in the future. The returned path is transformed to trajectory message of autoware.
        :param msg:
        """
        self.is_computing_trajectory = True

        if self.planner_type == 1:  # Breadth First Search
            self.run_search_planner()

        if self.planner_type == 2:  # Reactive Planner
            self.run_reactive_planner()

        self.is_computing_trajectory = False

    def prepare_traj_msg(self, states):
        traj = AWTrajectory()
        traj.header.frame_id = "map"

        for i in range(0, len(states)):
            new_point = TrajectoryPoint()
            t = states[i].time_step * self.scenario.dt
            nano_sec, sec = math.modf(t)
            new_point.time_from_start = Duration(sec=int(sec), nanosec=int(nano_sec * 1e9))
            new_point.pose.position = self.utm2map(states[i].position)
            new_point.pose.orientation = Cr2Auto.orientation2quaternion(states[i].orientation)
            if states[0].velocity == 0.0:
                states[0].velocity = 0.1
            new_point.longitudinal_velocity_mps = states[i].velocity
            new_point.front_wheel_angle_rad = states[i].steering_angle
            if "acceleration" in states[i].attributes:
                new_point.acceleration_mps2 = states[i].acceleration
            else:
                if i < len(states) - 1:
                    cur_vel = states[i].velocity
                    next_vel = states[i + 1].velocity
                    acc = (next_vel - cur_vel) / self.scenario.dt
                    new_point.acceleration_mps2 = acc
                else:
                    new_point.acceleration_mps2 = 0.0  # acceleration is 0 for the last state

            traj.points.append(new_point)

        self.traj_pub.publish(traj)
        self.get_logger().info('New trajectory published !!!')

        engage_msg = Engage()
        engage_msg.engage = True
        self.engage_pub.publish(engage_msg)
        # visualize_solution(self.scenario, self.planning_problem, create_trajectory_from_list_states(path)) #ToDo: test

    def run_search_planner(self):
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

            self.prepare_traj_msg(valid_states)
            # visualize_solution(self.scenario, self.planning_problem, create_trajectory_from_list_states(path)) #ToDo: check if working
        else:
            self.get_logger().error("Failed to solve the planning problem.")

    def run_reactive_planner(self):
        problem_init_state = self.planning_problem.initial_state
        current_velocity = problem_init_state.velocity
        if not hasattr(problem_init_state, 'acceleration'):
            problem_init_state.acceleration = 0.
        x_0 = deepcopy(problem_init_state)

        # goal state configuration
        goal = self.planning_problem.goal
        if hasattr(self.planning_problem.goal.state_list[0], 'velocity'):
            if self.planning_problem.goal.state_list[0].velocity.start != 0:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start +
                                    self.planning_problem.goal.state_list[0].velocity.end) / 2
            else:
                desired_velocity = (self.planning_problem.goal.state_list[0].velocity.start
                                    + self.planning_problem.goal.state_list[0].velocity.end) / 2
        else:
            desired_velocity = x_0.velocity

        replanning_frequency = self.get_parameter('reactive_planner.planning.replanning_frequency').get_parameter_value().integer_value

        # construct motion planner
        planner = self.planner(self.config)
        planner.set_d_sampling_parameters(self.get_parameter('reactive_planner.sampling.d_min').get_parameter_value().integer_value,
                                          self.get_parameter('reactive_planner.sampling.d_max').get_parameter_value().integer_value)
        planner.set_t_sampling_parameters(self.get_parameter('reactive_planner.sampling.t_min').get_parameter_value().double_value,
                                          self.get_parameter('reactive_planner.planning.dt').get_parameter_value().double_value,
                                          self.get_parameter('reactive_planner.planning.planning_horizon').get_parameter_value().double_value)
        planner.vehicle_params.length = self.vehicle_length
        planner.vehicle_params.width = self.vehicle_width
        planner.vehicle_params.wheelbase = self.get_parameter("vehicle.cg_to_front_m").get_parameter_value().double_value \
                                           + self.get_parameter("vehicle.cg_to_rear_m").get_parameter_value().double_value

        # set collision checker
        planner.set_collision_checker(self.scenario)

        # initialize route planner and set reference path
        route_planner = RoutePlanner(self.scenario, self.planning_problem)
        ref_path = route_planner.plan_routes().retrieve_first_route().reference_path
        planner.set_reference_path(ref_path)
        self.pub_route(ref_path)

        record_state_list = list()
        record_input_list = list()

        record_state_list.append(x_0)
        delattr(record_state_list[0], "slip_angle")
        record_state_list[0].steering_angle = np.arctan2(self.config.vehicle.wheelbase * record_state_list[0].yaw_rate,
                                                         record_state_list[0].velocity)
        record_input_state = State(
            steering_angle=np.arctan2(self.config.vehicle.wheelbase * x_0.yaw_rate, x_0.velocity),
            acceleration=x_0.acceleration,
            time_step=x_0.time_step,
            steering_angle_speed=0.)
        record_input_list.append(record_input_state)

        #while self._aw_state != 6:

        x_cl = None
        #x_0 = deepcopy(self.ego_vehicle_state)
        #optimal = None
        #new_state_list = None
        #self.get_logger().info("Reactive Planner Running")
        valid_states = []

        #self.get_logger().info("1")

        # Run planner
        while not goal.is_reached(x_0):  # or self._aw_state == 6:  # 6 = arrived goal
            #x_0 = deepcopy(self.ego_vehicle_state)
            current_count = len(record_state_list) - 1
            if current_count % replanning_frequency == 0:
                # new planning cycle -> plan a new optimal trajectory

                current_velocity = x_0.velocity
                planner.set_desired_velocity(desired_velocity)

                # plan trajectory
                optimal = planner.plan(x_0, x_cl)  # returns the planned (i.e., optimal) trajectory
                #self.get_logger().info("2")

                # if the planner fails to find an optimal trajectory -> terminate
                if not optimal:
                    self.get_logger().info("not optimal")
                    return

                # correct orientation angle
                new_state_list = planner.shift_orientation(optimal[0])

                # add new state to recorded state list
                new_state = new_state_list.state_list[1]
                new_state.time_step = current_count + 1
                record_state_list.append(new_state)

                # update init state and curvilinear state
                x_0 = deepcopy(record_state_list[-1])
                x_cl = (optimal[2][1], optimal[3][1])

            else:
                # not a planning cycle -> no trajectories sampled -> set sampled_trajectory_bundle to None

                # continue on optimal trajectory
                temp = current_count % replanning_frequency

                # add new state to recorded state list
                new_state = new_state_list.state_list[1 + temp]
                new_state.time_step = current_count + 1
                record_state_list.append(new_state)

                # update init state and curvilinear state
                x_0 = deepcopy(record_state_list[-1])
                x_cl = (optimal[2][1 + temp], optimal[3][1 + temp])


            # there are duplicated points, which will arise "same point" exception in AutowareAuto
            valid_states = []
            for state in optimal[0].state_list:
                if len(valid_states) > 0:
                    last_state = valid_states[-1]
                    if last_state.time_step == state.time_step:
                        continue
                valid_states.append(state)

            self.prepare_traj_msg(valid_states)
            planner.set_collision_checker(self.scenario)
        self.get_logger().info("reactive planner ended")

    def pub_route(self, path):
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
        route.color.a = 1.0
        for point in path:
            p = Point()
            p.x = point[0] - self.origin_x
            p.y = point[1] - self.origin_y
            p.z = 0.0
            route.points.append(p)
            route.colors.append(route.color)

        route_msg = MarkerArray()
        route_msg.markers.append(route)
        self.route_pub.publish(route_msg)

    def plot_scenario(self):
        self.rnd.clear()
        self.ego_vehicle = self._create_ego_with_cur_location()
        self.ego_vehicle.draw(self.rnd, draw_params={
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
        })
        self.scenario.draw(self.rnd, draw_params={'lanelet': {"show_label": False}})
        #self.planning_problem.draw(self.rnd) #ToDo: check if working
        self.rnd.render()
        plt.pause(0.1)

    def _create_ego_with_cur_location(self):
        # create a new ego vehicle with current position
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        if self.last_trajectory is None:
            return DynamicObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state)
        else:

            pred_traj = TrajectoryPrediction(self._awtrajectory_to_crtrajectory(1, 0, self.last_trajectory.points),
                                             ego_vehicle_shape)
            return DynamicObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state,
                                   prediction=pred_traj)

    def update_scenario(self):
        if self.ego_vehicle_state is None:
            self.get_logger().info("has not received a vehicle state yet!")
            return

        if not self.is_computing_trajectory:
            # remove past obstacles
            self.scenario.remove_obstacle(self.scenario.static_obstacles)
            self.scenario.remove_obstacle(self.scenario.dynamic_obstacles)
            # add current obstacles
            for static_obs in self.static_obstacles:
                obs_id = self.scenario.generate_object_id()
                obs_type = ObstacleType.UNKNOWN
                obs_shape = Rectangle(width=static_obs.width, length=static_obs.length)
                obs_state = State(position=np.array([static_obs.x, static_obs.y]), orientation=static_obs.orientation,
                                  time_step=0)
                self.scenario.add_objects(StaticObstacle(obs_id, obs_type, obs_shape, obs_state))
            for dynamic_obs in self.dynamic_obstacles:
                self.scenario.add_objects(dynamic_obs)
            self.plot_scenario()

    def write_scenario(self, filename='ZAM_Lanelet-1_1-T1.xml'):
        # save map
        # store converted file as CommonRoad scenario
        if self.get_parameter('write_scenario').get_parameter_value().bool_value:
            writer = CommonRoadFileWriter(
                scenario=self.scenario,
                planning_problem_set=self.planning_problem_set,
                author="",
                affiliation="Technical University of Munich",
                source="",
                tags={Tag.URBAN},
            )
            os.makedirs('output', exist_ok=True)
            writer.write_to_file(os.path.join('output', filename), OverwriteExistingFile.ALWAYS)

    def _transform_pose_to_map(self, pose_in):
        """
        transform pose to pose in map frame.
        :param pose_in:
        :return:
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

    @staticmethod
    def orientation2quaternion(orientation: float) -> Quaternion:
        """
        transform orientation (in commonroad) to quaternion (in autoware).
        :param orientation:
        :return:
        """
        quat = Quaternion()
        quat.w = math.cos(orientation * 0.5)
        quat.z = math.sin(orientation * 0.5)
        return quat

    @staticmethod
    def quaternion2orientation(quaternion: Quaternion) -> float:
        """
        transform quaternion (in autoware) to orientation (in commonroad).
        :param quaternion:
        :return:
        """
        z = quaternion.z
        w = quaternion.w
        mag2 = (z * z) + (w * w)
        epsilon = 1e-6
        if abs(mag2 - 1.0) > epsilon:
            mag = 1.0 / math.sqrt(mag2)
            z *= mag
            w *= mag

        y = 2.0 * w * z
        x = 1.0 - 2.0 * z * z
        return math.atan2(y, x)

    def map2utm(self, p: Point) -> np.array:
        """
        transform position (in autoware) to position (in commonroad).
        :param p:
        :return:
        """
        _x = self.origin_x + p.x
        _y = self.origin_y + p.y
        return np.array([_x, _y])

    def utm2map(self, position: np.array) -> Point:
        """
        transform position (in commonroad) to position (in autoware).
        :param position:
        :return:
        """
        p = Point()
        p.x = position[0] - self.origin_x
        p.y = position[1] - self.origin_y
        return p


def main(args=None):
    rclpy.init(args=args)
    cr2auto = Cr2Auto()

    #spin_thread = Thread(target=rclpy.spin, args=(cr2auto,))
    #spin_thread.start()

    #executor = MultiThreadedExecutor()
    #executor.add_node(cr2auto)
    #executor.spin()

    rclpy.spin(cr2auto)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cr2auto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
