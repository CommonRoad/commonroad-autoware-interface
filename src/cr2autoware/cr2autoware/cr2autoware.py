import geometry_msgs.msg
from numpy.lib.utils import source
import os
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from lxml import etree
import math
import numpy as np
from pyproj import Proj
import pyproj
import matplotlib.pyplot as plt
# import necessary classes from different modules
from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.planning.goal import GoalRegion
from commonroad.common.util import Interval, AngleInterval
from commonroad.geometry.shape import Rectangle, Polygon
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer

from crdesigner.input_output.api import lanelet_to_commonroad

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_vehicle_msgs.msg import VehicleKinematicState
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cr2autoware.tf2_geometry_msgs import do_transform_pose


@dataclass
class Box:
    x:           float
    y:           float
    width:       float
    length:      float
    orientation: float


class Cr2Auto(Node):

    def __init__(self):
        super().__init__('cr2autoware')
        self.proj_str = "+proj=utm +zone=10 +datum=WGS84 +ellps=WGS84"
        self.proj = Proj(self.proj_str)
        self.ego_vehicle = None
        self.ego_vehicle_cur_pose_map = None      # current pose in map frame
        self.ego_vehicle_state: State = None
        self.ego_vehicle_traj = []
        # buffer for static obstacles
        self.static_obstacles = []                  # a list save static obstacles from at the lastest time
        self.planning_problem = None

        self.convert_origin()
        self.ego_vechile_info()                 # compute ego vehicle width and height
        self.build_scenario()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listner = TransformListener(self.tf_buffer, self)               # convert among frames

        # subscribe inital pose of vehicle
        '''
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/initialpose',
            self.initial_pose_callback,
            10
        )
        '''
        # subscribe current position of vehicle
        self.current_state_sub = self.create_subscription(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            self.current_state_callback,
            10
        )
        # subscribe static obstacles
        self.static_obs_sub = self.create_subscription(
            BoundingBoxArray,
            '/perception/lidar_bounding_boxes_filtered',
            self.static_obs_callback,
            10
        )
        # subscribe goal pose
        self.goal_pose = self.create_subscription(
            PoseStamped,
            '/planning/goal_pose',
            self.goal_pose_callback,
            10
        )
        # create a timer to update scenario
        self.rnd = MPRenderer()
        plt.ion()
        #plt.figure(figsize=(8, 8))
        #plt.tight_layout()
        #plt.axis('off')
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.update_scenario)

    def convert_origin(self):
        self.declare_parameter("latitude", 0.0)
        self.declare_parameter("longitude", 0.0)
        self.declare_parameter("elevation", 0.0)
        self.declare_parameter("origin_offset_lat", 0.0)
        self.declare_parameter("origin_offset_lon", 0.0)
        self.origin_latitude = self.get_parameter("latitude").get_parameter_value().double_value
        self.origin_longitude = self.get_parameter("longitude").get_parameter_value().double_value
        self.origin_elevation = self.get_parameter("elevation").get_parameter_value().double_value
        self.orgin_offset_lat = self.get_parameter("origin_offset_lat").get_parameter_value().double_value
        self.orgin_offset_lon = self.get_parameter("origin_offset_lon").get_parameter_value().double_value

        self.origin_latitude = self.origin_latitude #+ self.orgin_offset_lat
        self.origin_longitude = self.origin_longitude #+ self.orgin_offset_lon
        self.get_logger().info("origin lat: %s,   origin lon: %s" % (self.origin_latitude, self.origin_longitude))
        self.proj = Proj(self.proj_str)
        self.origin_x, self.origin_y = self.proj(self.origin_longitude, self.origin_latitude)
        self.get_logger().info("origin x: %s,   origin  y: %s" % (self.origin_x, self.origin_y))

    def ego_vechile_info(self):
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
        self.declare_parameter('map_osm_file', '')
        self.declare_parameter('left_driving', False)
        self.declare_parameter('adjacencies', False)
        self.map_filename = self.get_parameter('map_osm_file').get_parameter_value().string_value
        self.left_driving = self.get_parameter('left_driving').get_parameter_value()
        self.adjacencies = self.get_parameter('adjacencies').get_parameter_value()
        self.scenario = lanelet_to_commonroad(self.map_filename,
                                              proj=self.proj_str, 
                                              left_driving=self.left_driving, 
                                              adjacencies=self.adjacencies)
        # add ego vehicle localization
        # add static obstacles
        # add dynamic obstacles
        # add problemset
        ## save map
        self.write_scenario()

    '''
        def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped) -> None:
        # do set current location if ego vehicle already has its location
        if self.ego_vehicle_cur_pose_map is not None:
            return

        self.get_logger().info('Subscribing initial pose ...')
        # update current pose of ego vehicle
        self.ego_vehicle_cur_pose_map = initial_pose.pose.pose
        
        x = self.ego_vehicle_cur_pose_map.position.x + self.origin_x
        y = self.ego_vehicle_cur_pose_map.position.y + self.origin_y     
        self.get_logger().info('After transform x: %s, y: %s' % (x, y))
        
        rotation_w = self.ego_vehicle_cur_pose_map.orientation.w
        orientation = 2 * math.acos(rotation_w)

        # generate the static obstacle according to the specification, refer to API for details of input parameters
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        ego_vehicle_initial_state = State(position=np.array([x, y]), orientation=orientation, time_step=0)

        # feed in the required components to construct a static obstacle
        self.ego_vehicle = StaticObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, ego_vehicle_initial_state)

        # add the static obstacle to the scenario
        #self.scenario.add_objects(ego_vehicle)

    def current_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:

        # update current pose of ego vehicle
        if msg.header.frame_id != "map":
            temp_pose_stamped = PoseStamped()
            temp_pose_stamped.header = msg.header
            temp_pose_stamped.pose = msg.pose.pose
            pose_stamped_map = self._transform_pose_to_map(temp_pose_stamped)
            if pose_stamped_map is None:
                return
            temp_pose = pose_stamped_map.pose       # current pose in map frame
        else:
            temp_pose = msg.pose
        self.ego_vehicle_cur_pose_map = temp_pose
    '''

    def current_state_callback(self, msg: VehicleKinematicState) -> None:
        """
        position: (state.x, state.y)
        velocity: state.longitudinal_velocity_mps
        orientation:
        yaw_rate:   state.heading_rate_rps
        slip_angle: 0
        time_step: Interval()
        """
        source_frame = msg.header.frame_id
        # lookup transform validty
        succeed = self.tf_buffer.can_transform("map",
                                               source_frame,
                                               rclpy.time.Time(),
                                               )
        if not succeed:
            self.get_logger().error(f"Failed to transform from {source_frame} to map frame")
            return None

        if source_frame != "map":
            try:
                tf_map = self.tf_buffer.lookup_transform("map", source_frame,
                                                         rclpy.time.Time.from_msg(msg.header.stamp))
            except tf2_ros.ExtrapolationException:
                # self.get_logger().warning("tf2 Extrapolation Exception")
                tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time())
            msg.state = self._transform_vehicle_state(msg.state, tf_map)
        position = np.array([msg.state.x+self.origin_x, msg.state.y+self.origin_y])

        orientation = self._to_angle(msg.state.heading)
        self.ego_vehicle_state = State(position=position,
                                       orientation=orientation,
                                       velocity=msg.state.longitudinal_velocity_mps,
                                       yaw_rate=msg.state.heading_rate_rps,
                                       slip_angle=0.0,
                                       time_step=0)

    def static_obs_callback(self, msg: BoundingBoxArray) -> None:
        # clear the obstacles from past
        self.static_obstacles.clear()

        source_frame = msg.header.frame_id
        #self.get_logger().info('Subscribe %d static obstacles ... (%s)' % (len(msg.boxes), source_frame))
        temp_pose = PoseStamped()
        temp_pose.header = msg.header
        for box in msg.boxes:
            temp_pose.pose.position.x = box.centroid.x
            temp_pose.pose.position.y = box.centroid.y
            temp_pose.pose.position.z = box.centroid.z
            temp_pose.pose.orientation.x = box.orientation.x
            temp_pose.pose.orientation.y = box.orientation.y
            temp_pose.pose.orientation.z = box.orientation.z
            temp_pose.pose.orientation.w = box.orientation.w
            pose_map = self._transform_pose_to_map(temp_pose)
            if pose_map is None:
                continue
            
            x = pose_map.pose.position.x + self.origin_x
            y = pose_map.pose.position.y + self.origin_y
            rotation_w = pose_map.pose.orientation.w
            orientation = 2 * math.acos(rotation_w)
            width = box.size.x
            length = box.size.y

            self.static_obstacles.append(Box(x, y, width, length, orientation))

    def add_dynamic_obstacles(self, traj):
        # initial state has a time step of 0
        x = traj[0].position.x + self.origin_x
        y = traj[0].position.y + self.origin_y
        rotation_w = traj[0].orientation.w
        orientation = 2 * math.acos(rotation_w)
        dynamic_obstacle_initial_state = State(position=np.array([x, y]),
                                               orientation=orientation,
                                               time_step=0) 
        state_list = []
        for i in range(1, len(traj)):
            # compute new position
            x = traj[i].position.x + self.origin_x
            y = traj[i].position.y + self.origin_y
            rotation_w = traj[i].orientation.w
            orientation = 2 * math.acos(rotation_w)
            # create new state
            new_state = State(position=np.array([x, y]), orientation=orientation, time_step=i) # TODO: real time step
            # add new state to state_list
            state_list.append(new_state)

        # create the trajectory of the obstacle, starting at time step 1
        dynamic_obstacle_trajectory = Trajectory(1, state_list)

        # create the prediction using the trajectory and the shape of the obstacle
        dynamic_obstacle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

        # generate the dynamic obstacle according to the specification
        dynamic_obstacle_id = self.scenario.generate_object_id()
        dynamic_obstacle_type = ObstacleType.CAR
        dynamic_obstacle = DynamicObstacle(dynamic_obstacle_id,
                                        dynamic_obstacle_type,
                                        dynamic_obstacle_shape,
                                        dynamic_obstacle_initial_state,
                                        dynamic_obstacle_prediction)
        # add dynamic obstacle to the scenario
        #self.scenario.add_objects(dynamic_obstacle)
        #self.write_scenario('traj_100.xml')

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        self.get_logger().info("Subscribe Goal Pose ...")
        x = msg.pose.position.x + self.origin_x
        y = msg.pose.position.y + self.origin_y
        self.get_logger().info("Goal: (%s, %s)" % (x, y))
        if self.ego_vehicle_state is None:
            self.get_logger().error("ego vehicle state is None")
            return

        region = Rectangle(length=10, width=10, center=np.array([x, y]), orientation=0.0)
        goal_state = State(position=region, orientation=AngleInterval(0.1, 1), time_step=Interval(3, 5))
        goal_region = GoalRegion([goal_state])
        self.planning_problem = PlanningProblem(planning_problem_id=10,
                                                initial_state=self.ego_vehicle_state,
                                                goal_region=goal_region)

    def plot_scenario(self):
        self.rnd.clear()
        self.scenario.draw(self.rnd, draw_params={'lanelet': {"show_label": False}})
        if self.planning_problem is not None:
            self.planning_problem.draw(self.rnd)
        self.rnd.render()
        plt.pause(0.1)

    def _create_ego_with_cur_location(self):
        # create a new ego vehicle with current position
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width=self.vehicle_width, length=self.vehicle_length)
        #self.get_logger().info("Current Position X: %f, Y: %f" % (x - self.origin_x, y - self.origin_y))
        return StaticObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, self.ego_vehicle_state)

    def update_scenario(self):
        # remove past obstacles
        self.scenario.remove_obstacle(self.scenario.static_obstacles)
        #self.scenario.remove_obstacle(self.scenario.dynamic_obstacles)

        # add current obstacles
        for static_obs in self.static_obstacles:
            obs_id = self.scenario.generate_object_id()
            obs_type = ObstacleType.UNKNOWN
            obs_shape = Rectangle(width=static_obs.width, length=static_obs.length)
            obs_state = State(position=np.array([static_obs.x, static_obs.y]), orientation=static_obs.orientation, time_step=0)
            self.scenario.add_objects(StaticObstacle(obs_id, obs_type, obs_shape, obs_state))
        # update current location of ego vehicle
        if self.ego_vehicle_state is not None:
            if self.ego_vehicle is None:
                self.ego_vehicle = self._create_ego_with_cur_location()
                self.scenario.add_objects(self.ego_vehicle)
            else:
                # remove past ego vehicle location
                self.scenario.remove_obstacle(self.ego_vehicle)
                self.ego_vehicle = self._create_ego_with_cur_location()
                self.scenario.add_objects(self.ego_vehicle)
        self.write_scenario()
        self.plot_scenario()

    def write_scenario(self, filename='ZAM_Lanelet-1_1-T1.xml'):
        # save map
        # store converted file as CommonRoad scenario
        writer = CommonRoadFileWriter(
            scenario=self.scenario,
            planning_problem_set=PlanningProblemSet(),
            author="",
            affiliation="Technical University of Munich",
            source="",
            tags={Tag.URBAN},
        )
        writer.write_to_file(os.path.join('output', filename), OverwriteExistingFile.ALWAYS)

    def _transform_pose_to_map(self, pose_in):
        source_frame = pose_in.header.frame_id
        # lookup transform validty
        succeed = self.tf_buffer.can_transform("map", 
                                               source_frame,
                                               rclpy.time.Time(),
                                               )
        if not succeed:
            self.get_logger().error(f"Failed to transform from {source_frame} to map frame")
            return None
        
        try:
            tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time.from_msg(pose_in.header.stamp))
        except tf2_ros.ExtrapolationException:
            #self.get_logger().warning("tf2 Extrapolation Exception")
            tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time())

        pose_out = do_transform_pose(pose_in, tf_map)
        return pose_out

    def _transform_vehicle_state(self, state: TrajectoryPoint, transform: TransformStamped):
        """
        This function is an reimplementation of do_transform in motion_common.cpp,
        in order to transform vehicle state from odom to map.
        :param state:
        :return:
        """
        q = transform.transform.rotation
        r2 = q.w * q.w
        i2 = q.x * q.x
        j2 = q.y * q.y
        k2 = q.z * q.z

        s = 1.0 / (r2 + i2 + j2 + k2)
        a11 = 1.0 - (2.0 * s * (j2 + k2))
        a12 = 2.0 * s * ((q.x * q.y) - (q.z * q.w))
        a21 = 2.0 * s * ((q.x * q.y) + (q.z * q.w))
        a22 = 1.0 - (2.0 * s * (i2 + k2))
        # Apply rotation
        x, y = state.x, state.y
        state.x = x * a11 + y * a12
        state.y = x * a21 + y * a22

        s = 1.0 / math.sqrt(r2 + k2)
        w = q.w * s
        z = q.z * s
        real = state.heading.real
        imag = state.heading.imag
        state.heading.real = real * w - imag * z
        state.heading.imag = real * z + imag * w

        state.x += transform.transform.translation.x
        state.y += transform.transform.translation.y
        return state

    def _to_angle(self, heading):
        """
        This function is an reimplementation of to_angle in motion_common.cpp,
        in order to calculate orientation from heading information of vehicle state
        :param heading: information from vehicle kinematic state
        :return: orientation in radians
        """
        mag2 = (heading.real * heading.real) + (heading.imag * heading.imag)
        epsilon = 1e-6
        if abs(mag2 - 1.0) > epsilon:
            imag = 1.0 / math.sqrt(mag2)
            heading.real *= imag
            heading.imag *= imag

        y = 2.0 * heading.real * heading.imag
        x = 1.0 - 2.0 * heading.imag * heading.imag
        return math.atan2(y, x)


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
