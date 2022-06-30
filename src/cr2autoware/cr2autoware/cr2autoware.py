import os
import sys
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
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

# from crdesigner.input_output.api import lanelet_to_commonroad
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from autoware_auto_perception_msgs.msg import DetectedObjects, PredictedObjects
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_planning_msgs.msg import Trajectory as AWTrajectory
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
        # TODO: get zone value from yaml file
        self.proj_str = "+proj=utm +zone=54 +datum=WGS84 +ellps=WGS84"

        self.declare_parameter("write_scenario", False)

        self.ego_vehicle = None
        self.ego_vehicle_state: State = None
        # buffer for static obstacles
        self.static_obstacles = []  # a list save static obstacles from at the latest time
        self.dynamic_obstacles = []  # a list save dynamic obstacles from at the latest time
        self.last_trajectory = None

        self.current_planning_problem_id = 0
        self.planning_problem_set = PlanningProblemSet()
        # load the xml with stores the motion primitives
        name_file_motion_primitives = 'V_0.0_20.0_Vstep_4.0_SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml'
        self.automaton = ManeuverAutomaton.generate_automaton(name_file_motion_primitives)
        self.is_computing_trajectory = False  # stop update scenario when trajectory is computing
        self.planner = MotionPlanner.BreadthFirstSearch

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)  # convert among frames

        self.convert_origin()
        self.ego_vechile_info()  # compute ego vehicle width and height
        self.build_scenario()  # build scenario from osm map

        # subscribe current position of vehicle
        self.current_state_sub = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.current_state_callback,
            10
        )
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
            10
        )
        # publish trajectory
        self.traj_pub = self.create_publisher(
            AWTrajectory,
            '/planning/scenario_planning/trajectory',
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
        self.declare_parameter("latitude", 0.0)
        self.declare_parameter("longitude", 0.0)
        self.declare_parameter("elevation", 0.0)
        self.declare_parameter("origin_offset_lat", 0.0)
        self.declare_parameter("origin_offset_lon", 0.0)
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

    def ego_vechile_info(self):
        """
        compute size of ego vehicle: (length, width)
        :return:
        """
        self.declare_parameter('vehicle.cg_to_front_m', 1.0)
        self.declare_parameter('vehicle.cg_to_rear_m', 1.0)
        self.declare_parameter('vehicle.width_m', 2.0)
        self.declare_parameter('vehicle.front_overhang_m', 0.5)
        self.declare_parameter('vehicle.rear_overhang_m', 0.5)
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
        self.declare_parameter('map_osm_file', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)
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

        region = Rectangle(length=4, width=4, center=position, orientation=orientation)
        goal_state = State(position=region, time_step=Interval(10, 50), velocity=Interval(0.0, 0.1))
        goal_region = GoalRegion([goal_state])
        self.current_planning_problem_id += 1  # generate new id when a new planning problem is added
        planning_problem = PlanningProblem(planning_problem_id=self.current_planning_problem_id,
                                           initial_state=self.ego_vehicle_state,
                                           goal_region=goal_region)
        self.planning_problem_set.add_planning_problem(planning_problem)
        self._solve_planning_problem(msg)

    def _solve_planning_problem(self, msg: PoseStamped) -> None:
        """
        Solve planning problem with algorithms offered by commonroad. Now BreadthFirstSearch is in use, but can
        explore other algorithms in the future. The returned path is transformed to trajectory message of autoware.
        :param msg:
        """
        self.is_computing_trajectory = True
        # construct motion planner
        planner = self.planner(scenario=self.scenario,
                               planning_problem=self.planning_problem_set.find_planning_problem_by_id(
                                   self.current_planning_problem_id),
                               automaton=self.automaton)

        # visualize searching process
        # scenario_data = (self.scenario, planner.state_initial, planner.shape_ego, self.planning_problem)
        # display_steps(scenario_data, planner.execute_search, planner.config_plot)
        path, _, _ = planner.execute_search()

        if path is not None:
            self.get_logger().info('Found trajectory to goal region !!!')
            traj = AWTrajectory()
            traj.header.frame_id = "map"

            valid_states = []
            # there are duplicated points, which will arise "same point" exception in AutowareAuto
            for states in path:
                for state in states:
                    if len(valid_states) > 0:
                        last_state = valid_states[-1]
                        if last_state.time_step == state.time_step:
                            continue
                    valid_states.append(state)

            for i in range(0, len(valid_states)):
                new_point = TrajectoryPoint()
                t = valid_states[i].time_step * self.scenario.dt
                nano_sec, sec = math.modf(t)
                new_point.time_from_start = Duration(sec=int(sec), nanosec=int(nano_sec * 1e9))
                new_point.pose.position = self.utm2map(valid_states[i].position)
                new_point.pose.orientation = Cr2Auto.orientation2quaternion(valid_states[i].orientation)
                if valid_states[0].velocity == 0.0:
                    valid_states[0].velocity = 0.1
                new_point.longitudinal_velocity_mps = valid_states[i].velocity
                # self.get_logger().info(f"longitudinal_velocity_mps{i}: {new_point.longitudinal_velocity_mps}")
                new_point.front_wheel_angle_rad = valid_states[i].steering_angle
                if "acceleration" in valid_states[i].attributes:
                    new_point.acceleration_mps2 = valid_states[i].acceleration
                else:
                    if i < len(valid_states) - 1:
                        cur_vel = valid_states[i].velocity
                        next_vel = valid_states[i + 1].velocity
                        acc = (next_vel - cur_vel) / self.scenario.dt
                        new_point.acceleration_mps2 = acc
                    else:
                        new_point.acceleration_mps2 = 0.0  # acceleration is 0 for the last state

                traj.points.append(new_point)

            self.traj_pub.publish(traj)
            self.last_trajectory = traj
        else:
            self.get_logger().error("Failed to solve the planning problem.")

        self.is_computing_trajectory = False

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
        # self.planning_problem_set.draw(self.rnd)
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

    rclpy.spin(cr2auto)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cr2auto.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
