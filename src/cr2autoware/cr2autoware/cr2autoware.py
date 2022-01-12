from numpy.lib.utils import source
import os
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
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.geometry.shape import Rectangle, Polygon
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType, DynamicObstacle
from commonroad.scenario.trajectory import State
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.visualization.mp_renderer import MPRenderer

from crdesigner.input_output.api import lanelet_to_commonroad

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from autoware_auto_perception_msgs.msg import BoundingBoxArray
from autoware_auto_vehicle_msgs.msg import VehicleKinematicState
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from cr2autoware.tf2_geometry_msgs import do_transform_pose


class Cr2Auto(Node):

    def __init__(self):
        super().__init__('cr2autoware')
        self.proj_str = "+proj=utm +zone=10 +datum=WGS84 +ellps=WGS84"
        self.proj = Proj(self.proj_str)
        self.ego_vehicle_cur_pose_map = None      # current pose in map frame
        self.ego_vehicle_initial_pose = None
        self.ego_vehicle_traj = []

        self.convert_origin()
        self.ego_vechile_info()                 # compute ego vehicle width and height
        self.build_scenario()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listner = TransformListener(self.tf_buffer, self)               # convert among frames
        
        """
        self.static_obs_subscriber = self.create_subscription(                  # static obstacles
            BoundingBoxArray,
            '/perception/lidar_bounding_boxes_filtered',
            self.static_obstacle_callback,
            10
        )
        """
        # subscribe inital pose of vehicle
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/initialpose',
            self.initial_pose_callback,
            10
        )
        # subscribe current position of vehicle
        self.current_pose_sub = self.create_subscription(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            self.current_pose_callback,
            10
        )
        # subscribe static obstacles
        static_obs_sub = self.create_subscription(
            BoundingBoxArray,
            '/perception/lidar_bounding_boxes_filtered',
            self.static_obs_callback,
            10
        )




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

    def initial_pose_callback(self, initial_pose: PoseWithCovarianceStamped) -> None:
        self.get_logger().info('Subscribing initial pose ...')
        # update current pose of ego vehicle
        self.ego_vehicle_initial_pose = initial_pose.pose.pose
        self.ego_vehicle_cur_pose_map = initial_pose.pose.pose
        
        x = self.ego_vehicle_cur_pose_map.position.x + self.origin_x
        y = self.ego_vehicle_cur_pose_map.position.y + self.origin_y     
        self.get_logger().info('After transform x: %s, y: %s' % (x, y))
        
        rotation_w = self.ego_vehicle_cur_pose_map.orientation.w
        orientation = 2 * math.acos(rotation_w)

        # generate the static obstacle according to the specification, refer to API for details of input parameters
        ego_vehicle_id = self.scenario.generate_object_id()
        ego_vehicle_type = ObstacleType.CAR
        ego_vehicle_shape = Rectangle(width = self.vehicle_width, length = self.vehicle_length)
        ego_vehicle_initial_state = State(position = np.array([x, y]), orientation = orientation, time_step = 0)

        # feed in the required components to construct a static obstacle
        ego_vehicle = StaticObstacle(ego_vehicle_id, ego_vehicle_type, ego_vehicle_shape, ego_vehicle_initial_state)

        # add the static obstacle to the scenario
        self.scenario.add_objects(ego_vehicle)

        self.write_scenario()

    def current_pose_callback(self, msg: VehicleKinematicState) -> None:
        if self.ego_vehicle_initial_pose is None:
            return
        self.get_logger().info('Subscribing current pose of ego vehicle ...')
        # update current pose of ego vehicle
        if msg.header.frame_id != "map":
            temp_pose_stamped = PoseStamped()
            temp_pose_stamped.header = msg.header
            temp_pose_stamped.pose = msg.state.pose
            pose_stamped_map = self._transform_pose_to_map(temp_pose_stamped)
            if pose_stamped_map is None:
                return
            temp_pose = pose_stamped_map.pose       # current pose in map frame
        else:
            temp_pose = msg.state.pose
        
        # refuse to update if there is almost no change, JUST FOR TEST!!!
        delta_x = temp_pose.position.x - self.ego_vehicle_cur_pose_map.position.x
        delta_y = temp_pose.position.y - self.ego_vehicle_cur_pose_map.position.y
        if delta_x > 0.1 or delta_y > 0.1:
            self.ego_vehicle_cur_pose_map = temp_pose
            self.ego_vehicle_traj.append(temp_pose)
        self.get_logger().info('Length of ego vehicle traj: %d' % len(self.ego_vehicle_traj))
        if len(self.ego_vehicle_traj) == 100:
            # add dynamic obstacle for testing
            self.add_dynamic_obstacles(self.ego_vehicle_traj)

    def static_obs_callback(self, msg: BoundingBoxArray)->None:
        source_frame = msg.header.frame_id
        self.get_logger().info(' Subscribe static obstacles ... (%s)' % source_frame)
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

            obs_id = self.scenario.generate_object_id()
            obs_type = ObstacleType.UNKNOWN
            obs_shape = Rectangle(width=width, length=length)
            obs_state = State(position=np.array([x, y]), orientation=orientation, time_step=0)
            static_obs = StaticObstacle(obs_id, obs_type, obs_shape, obs_state)
            # add the static obstacle to the scenario
            self.scenario.add_objects(static_obs)

        self.write_scenario()

        
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
        self.scenario.add_objects(dynamic_obstacle)
        self.write_scenario('traj_100.xml')

    def static_obstacle_callback(self, boxArray: BoundingBoxArray):
        self.get_logger().info('Subscribing static obstacles ...')

    def plot_scenario(self):
        plt.figure(figsize=(10, 10))
        rnd = MPRenderer()
        self.scenario.draw(rnd, draw_params={'lanelet': {"show_label": True}})
        rnd.render()
        plt.show()

    def write_scenario(self, filename='ZAM_Lanelet-1_1-T1.xml'):
        ## save map
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
                                                timeout=Duration(seconds=1.0))
        if not succeed:
            self.get_logger().error("Failed to transform pose to map frame")
            return None
        
        try:
            tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time.from_msg(pose_in.header.stamp))
        except tf2_ros.ExtrapolationException:
            tf_map = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0))

        pose_out = do_transform_pose(pose_in, tf_map)
        return pose_out

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
